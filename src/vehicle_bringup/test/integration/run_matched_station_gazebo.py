#!/usr/bin/env python3
"""Matched CAD/free POSE/preinsert/known-load evidence; optional axial contact.

Ground-truth evaluation is separate from the force controller. A successful
transport/action result never substitutes for CAD physical mating evidence.
"""
import argparse
from contextlib import contextmanager
import json
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np

from run_fr10_pose_gazebo_smoke import WORKSPACE, ProbeContext, list_controllers
from fr10_feedback_trace import write_feedback_trace
from fr10_wrench_calibration import compose_rotation, check_known_load


def controller_profile(path):
    import importlib.util
    spec = importlib.util.spec_from_file_location('controller_profile', WORKSPACE /
        'src/fr10_arm_control/fr10_pose_controller/launch/controller_profile.py')
    loader = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loader)
    return {'fr10_pose_controller': {'ros__parameters': loader.load_controller_profile(path)}}


def await_future(future, timeout=10):
    deadline = time.monotonic()+timeout
    while not future.done() and time.monotonic()<deadline: time.sleep(.02)
    if not future.done() or future.result() is None: raise RuntimeError('ROS request timed out')
    return future.result()


@contextmanager
def stopping_probe(domain, summary):
    """Keep DDS and physics alive until the arm acknowledges a HOLD boundary."""
    with ProbeContext(domain) as probe:
        try:
            yield probe
        finally:
            summary['shutdown_hold_confirmed'] = False
            if summary.get('ready'):
                probe.publish_controlled_stop()
                deadline = time.monotonic()+10
                while time.monotonic()<deadline:
                    status = probe.status()
                    if 'state=HOLDING' in status and 'reason=CONTROLLED_STOP' in status:
                        summary['shutdown_hold_confirmed'] = True
                        break
                    time.sleep(.05)
                summary['shutdown_status'] = probe.status()


def stop_owned_motion_node(processes, runtime_path, summary):
    """Finish the node's shutdown brake before its owned physics is stopped."""
    candidates=re.findall(r'\[fr10_pose_controller_node-\d+\]: process started with pid \[(\d+)\]',
                          runtime_path.read_text(errors='replace'))
    summary['motion_shutdown_before_physics']=False
    if not candidates:
        summary['motion_shutdown_detail']='No owned motion-node PID in launch log'
        return
    pid=int(candidates[-1]); proc=Path('/proc')/str(pid)
    try:
        argv=(proc/'cmdline').read_bytes().split(b'\0')
        from ament_index_python.packages import get_package_prefix
        expected=(Path(get_package_prefix('fr10_pose_controller'))/
                  'lib/fr10_pose_controller/fr10_pose_controller_node').resolve()
        if not argv[0] or Path(os.fsdecode(argv[0])).resolve()!=expected:
            raise RuntimeError('Motion-node executable does not match this runtime')
        ancestor=pid; owned={p.pid for p in processes}
        for _ in range(12):
            if ancestor in owned: break
            status=(Path('/proc')/str(ancestor)/'status').read_text()
            ancestor=int(re.search(r'^PPid:\s+(\d+)',status,re.M).group(1))
        else: raise RuntimeError('Motion node is not a descendant of this run')
        os.kill(pid,signal.SIGINT)
        deadline=time.monotonic()+15.
        while proc.exists() and time.monotonic()<deadline:
            time.sleep(.05)
        summary['motion_shutdown_before_physics']=not proc.exists()
        summary['motion_shutdown_detail']='SIGINT to verified owned node while physics remained alive'
    except FileNotFoundError:
        summary['motion_shutdown_detail']='Motion node had already exited'
    except (OSError, RuntimeError, AttributeError) as error:
        summary['motion_shutdown_detail']=str(error)


def wait_pose_result(probe, position, orientation, commands, first_event, timeout):
    """Wait for measured arrival and HOLD, stopping early on a typed rejection."""
    deadline = time.monotonic()+timeout
    while time.monotonic()<deadline:
        rejected = [event for event in commands[first_event:] if event['phase'] == 7]
        if rejected:
            raise RuntimeError(f'POSE command rejected: {rejected[-1]}')
        if probe.wait_pose(position, orientation, timeout=.65) and 'state=HOLDING' in probe.status():
            return True
    return False


def run(output, domain, port, station_name, mounted, overlay, insert, calibrate=False, six_axis=False, pose_input=False,
        backend='gazebo', station_file=None, initial_state='home', run_timeout=900.,
        native_probe=None, retract_after_hold=False, fault_after_hold=None,
        pose_lateral_mm=(0., 0.), pose_rotation_deg=(0., 0., 0.), gazebo_step_s=None,
        gazebo_solver_iterations=None, arm_camera=True, pose_error_limit=2., pose_axial_mm=0.):
    from fr10_pose_uncertainty import input_socket_pose
    if pose_error_limit not in (2., 5.) or (pose_error_limit == 5. and backend != 'mujoco'):
        raise ValueError('The extended 5 mm / 5 degree envelope requires MuJoCo')
    error_limits = dict(max_position_mm=pose_error_limit, max_rotation_deg=pose_error_limit, axial_mm=pose_axial_mm)
    input_socket_pose([0., 0., 0.], [0., 0., 0., 1.], pose_lateral_mm, pose_rotation_deg, **error_limits)
    if (any(pose_lateral_mm) or any(pose_rotation_deg) or pose_axial_mm != 0.) and not (insert and pose_input):
        raise ValueError('Pose uncertainty requires one-pose automatic insertion')
    if not 0 <= domain <= 232:
        raise ValueError('Fast DDS test domain must be in 0..232; no launch started')
    if not 1024 <= port <= 65535:
        raise ValueError('Gazebo test port must be in 1024..65535; no launch started')
    if not math.isfinite(run_timeout) or run_timeout <= 0:
        raise ValueError('run_timeout must be positive and finite')
    if gazebo_step_s is not None and (backend!='gazebo' or isinstance(gazebo_step_s, bool) or
            not math.isfinite(gazebo_step_s) or not .00005 <= gazebo_step_s <= .001):
        raise ValueError('Gazebo convergence step requires Gazebo and 50 us..1 ms')
    if gazebo_solver_iterations is not None and (backend != 'gazebo' or
            isinstance(gazebo_solver_iterations, bool) or
            not isinstance(gazebo_solver_iterations, int) or
            not 50 <= gazebo_solver_iterations <= 2000):
        raise ValueError('Gazebo solver iterations require Gazebo and an integer in 50..2000')
    if fault_after_hold and (backend!='mujoco' or not (insert and pose_input) or retract_after_hold):
        raise ValueError('HOLD fault evidence requires MuJoCo automatic insertion without retract')
    if insert and overlay is None:
        overlay=WORKSPACE/'src/fr10_arm_control/fr10_pose_controller/config/matched_socket_commissioning.yaml'
    if retract_after_hold and not (insert and pose_input):
        raise ValueError('Explicit retract evidence requires one-pose automatic insertion')
    output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
               GAZEBO_MASTER_URI=f'http://127.0.0.1:{port}', GAZEBO_MODEL_DATABASE_URI='',
               ROS_LOG_DIR=str(output/'ros_log'))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    # The in-process probe uses the same writable run log directory as children.
    os.environ['ROS_LOG_DIR'] = env['ROS_LOG_DIR']
    command = ['ros2', 'launch', 'vehicle_bringup', 'bringup.launch.py',
        'use_arm:=true', f'use_vehicle:={str(mounted).lower()}', 'use_gazebo:=true',
        'use_force:=true', f'station:={station_name}', f'initial_state:={initial_state}',
        'gui:=false', 'rviz:=false', f'gazebo_port:={port}']
    if overlay: command.append(f'force_controller_overlay:={overlay.resolve()}')
    if gazebo_step_s is not None: command.append(f'matched_physics_step_s:={gazebo_step_s}')
    if gazebo_solver_iterations is not None:
        command.append(f'matched_solver_iterations:={gazebo_solver_iterations}')
    command.append(f'use_arm_camera:={str(arm_camera).lower()}')
    source_parameters = controller_profile(overlay)['fr10_pose_controller']['ros__parameters'] if overlay else {}
    commands_to_launch = [command]
    if backend in ('mujoco','fairino'):
        if mounted or overlay is None:
            raise ValueError('Matched MuJoCo/native virtual contact requires arm-only runtime and commissioning overlay')
        import yaml
        import importlib.util
        spec=importlib.util.spec_from_file_location('charging_station',
            WORKSPACE/'src/vehicle_bringup/launch/charging_station.py')
        station_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(station_module)
        if station_file is None:
            model=WORKSPACE/'src/vehicle_description/models/matched_socket_v2'
            selected_station=station_module.select_station(
                WORKSPACE/'src/vehicle_bringup/config/charging_stations.yaml',model/'geometry.json',
                station_name,'<robot/>',False,
                preinsert_distance_m=source_parameters.get('force.goal.approach_distance_m'))
            selected_station['model_path']=str(model)
        else:
            selected_station=json.loads(station_file.read_text())
        if selected_station['name'] != station_name:
            raise ValueError('Station file does not match the requested station')
        selected_station['initial_state'] = initial_state
        station_file = output/'selected_station.json'
        station_file.write_text(json.dumps(selected_station, indent=2)+'\n')
        parameters = controller_profile(overlay)
        values = parameters['fr10_pose_controller']['ros__parameters']
        geometry = json.loads((Path(json.loads(station_file.read_text())['model_path'])/'geometry.json').read_text())
        # The same station/CAD authority supplies clearance for both backends,
        # including older external station JSON lacking this derived metadata.
        values.setdefault('force.search.yaw_free_half_span_rad', station_module.keyed_yaw_half_span(geometry))
        values.setdefault('force.search.xy_free_half_span_m', float(geometry['simulation_design']['radial_clearance_mm'])*.001)
        values.update({'force.wrench_source_frame':'charging_tcp','force.wrench_target_frame':'charging_tcp',
            'force.wrench_source_sign':1.0,'force.wrench_transform.target_to_source_m':[0.,0.,0.],
            'force.gravity_compensation.enabled':False,'force.gravity_compensation.calibrated':False,
            'force.gravity_compensation.owner':'source',
            'force.geometry.mating_clearance_m':geometry['simulation_design']['radial_clearance_mm']*.001,
            'force.geometry.mating_feature_lever_m':.001*max(math.hypot(math.hypot(*p['center_mm'])+p['bore_inscribed_radius_mm'],
                geometry['simulation_design']['nose_length_mm']) for p in geometry['pins'])})
        active_overlay = output/'mujoco_overlay.yaml'; active_overlay.write_text(yaml.safe_dump(parameters))
        source_parameters = values
        commands_to_launch = [
            [sys.executable,'src/vehicle_bringup/test/integration/fr10_mujoco_bridge.py','--station-file',str(station_file.resolve()),
             '--evidence-directory',str(output), '--period', '.001' if backend=='mujoco' else '.008'],
            ['ros2','launch','fr10_pose_controller','fr10_pose_controller.launch.py','start_robot_stack:=false',
             'use_gazebo:=true','enable_force_sensor:=true','use_rviz:=false','charging_tool_profile:=matched_socket_v2',
             f'controller_overlay:={active_overlay}'],
            [sys.executable,'src/vehicle_bringup/scripts/matched_station_publisher.py','--ros-args',
             '-p',f'station_file:={station_file.resolve()}','-p','use_sim_time:=true']]
        if backend=='fairino':
            if native_probe is None: raise ValueError('Native Docker run requires a read-only SDK probe')
            from fr10_simmachine_preflight import collect_report
            report = collect_report('fairino-container',native_probe)
            proof = output/'native_simulator_proof.json'; proof.write_text(json.dumps(report,indent=2)+'\n')
            import importlib.util
            spec=importlib.util.spec_from_file_location('native_simulator_gate',
                WORKSPACE/'src/fr10_arm_control/fr10_pose_controller/launch/native_simulator_gate.py')
            gate=importlib.util.module_from_spec(spec);spec.loader.exec_module(gate)
            try: gate.verify_simulator_proof(proof)
            except Exception as error:
                (output/'native_gate.json').write_text(json.dumps({'status':'BLOCKED','reason':str(error)})+'\n')
                raise
            native_state=json.loads(report['native_probe_stdout'])
            native_q=np.deg2rad(native_state['joint_position_deg']).tolist()
            selected_station['initial_joint_rad']['native_measured']=native_q
            selected_station['initial_state']='native_measured'
            station_file.write_text(json.dumps(selected_station,indent=2)+'\n')
            commands_to_launch[0].append('--observe-native-joints')
            commands_to_launch[1]=['ros2','launch','fr10_pose_controller','fr10_pose_controller.launch.py',
                'use_fake_hardware:=false','use_gazebo:=false','use_rviz:=false','start_robot_stack:=true',
                'enable_force_sensor:=true','charging_tool_profile:=matched_socket_v2',
                f'controller_overlay:={active_overlay}',f'native_simulator_proof:={proof}']
            commands_to_launch[2][-1]='use_sim_time:=false'
    summary = {'command': command, 'domain': domain, 'port': port, 'mounted': mounted,
        'real_contact': False, 'force_self_alignment_gate': 'NOT_TESTED', 'physical_seating_gate': 'NOT_TESTED'}
    if backend=='gazebo':
        from ode_transport_runtime import observer_binary
        observer=observer_binary()
        commands_to_launch.append([str(observer)])
        env.update(ODE_OUT=str(output),ODE_WORLD_NAME='default',
            ODE_TARGET_ENTITY='my_mobile_manipulator::charging_connector_link',
            ODE_REFERENCE_LINK='my_mobile_manipulator::'+('base_footprint' if mounted else 'fr10_base_link'),
            ODE_OBSERVE_CONTACTS='true')
    summary.update(backend=backend,commands=commands_to_launch)
    wrench_topic = source_parameters.get('force.wrench_topic', '/ft_sensor/wrench')
    gravity_in_source = source_parameters.get('force.gravity_compensation.owner', 'application') == 'source'
    summary.update(wrench_topic=wrench_topic, gravity_compensation_owner='source' if gravity_in_source else 'application')
    if calibrate and gravity_in_source:
        raise ValueError('Gravity orientation calibration requires the raw payload-inclusive load cell')
    if backend=='fairino':
        summary.update(contact_source='virtual CAD plant following measured native SDK joints',
            native_image_digest=report['image_digest'],native_container_id=report['container_id'],
            sdk_cad_physics=False,native_initial_joint_rad=native_q,
            initial_state_source='measured SDK packet; observer initialization only')
    from fr10_matched_run_manifest import capture
    capture(WORKSPACE, output, [observer] if backend=='gazebo' else [native_probe] if backend=='fairino' else [])
    (output/'launch_commands.json').write_text(json.dumps(commands_to_launch, indent=2)+'\n')
    raw_trace, feedback_trace, commands, contact_trace = [], [], [], []
    diagnostic_wrench_trace = []
    reported_tcp_trace, independent_tcp_trace, joint_reference_trace = [], [], []
    automatic_results = []
    base_trace, physical_base_trace, parking_trace = [], [], []
    with (output/'runtime.log').open('w') as log:
        # Gazebo transport setup must follow master/server readiness. Starting
        # it concurrently can leave the client connected to no advertised world.
        startup_commands = commands_to_launch[:-1] if backend=='gazebo' else commands_to_launch
        processes = [subprocess.Popen(item, cwd=WORKSPACE, env=env, stdout=log,
                     stderr=subprocess.STDOUT,start_new_session=True) for item in startup_commands]
        launch = processes[0]
        try:
            with stopping_probe(domain, summary) as probe:
                from geometry_msgs.msg import PoseStamped, WrenchStamped
                from rcl_interfaces.srv import GetParameters
                from rclpy.qos import qos_profile_sensor_data
                from fr10_motion_interfaces.msg import MotionCommandStatus
                from moveit_msgs.srv import GetPlanningScene
                from gazebo_msgs.msg import ContactsState
                from fr10_motion_interfaces.action import Insert
                from std_msgs.msg import Float64MultiArray
                if mounted:
                    from nav_msgs.msg import Odometry
                    from fr10_motion_interfaces.msg import VehicleParkingState
                    from rclpy.time import Time
                    from rclpy.duration import Duration
                    from scipy.spatial.transform import Rotation
                    base_tf_buffer=probe.tf_buffer
                    def record_base(message,destination):
                        p=message.pose.pose.position;q=message.pose.pose.orientation;t=message.twist.twist
                        destination.append({'wall':time.monotonic(),
                            'source':message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                            'frame':message.header.frame_id,'child':message.child_frame_id,
                            'position':[p.x,p.y,p.z],'orientation_xyzw':[q.x,q.y,q.z,q.w],
                            'twist':[t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z]})
                    probe.create_subscription(Odometry,'/odom',lambda m:record_base(m,base_trace),10)
                    probe.create_subscription(Odometry,'/fr10_sim/base_odometry',
                        lambda m:record_base(m,physical_base_trace),10)
                    probe.create_subscription(VehicleParkingState,'/vehicle/parking_state',
                        lambda m:parking_trace.append({'wall':time.monotonic(),
                            'source':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                            'owner':m.owner_id,'sequence':m.sequence,'state':m.state}),10)
                probe.create_subscription(Float64MultiArray,'/fr10_force/joint_reference',
                    lambda m:joint_reference_trace.append({'wall':time.monotonic(),
                        'sequence':m.layout.data_offset,'qva':list(m.data)}),qos_profile_sensor_data)
                def record_tcp(message,destination):
                    pose=message.pose
                    destination.append({'wall':time.monotonic(),
                        'source':message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                        'position':[pose.position.x,pose.position.y,pose.position.z],
                        'orientation_xyzw':[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]})
                probe.create_subscription(PoseStamped,'/fr10_motion/tcp_pose',
                    lambda m:record_tcp(m,reported_tcp_trace),qos_profile_sensor_data)
                if backend in ('mujoco','fairino'):
                    probe.create_subscription(PoseStamped,'/fr10_mujoco/tcp_pose',
                        lambda m:record_tcp(m,independent_tcp_trace),qos_profile_sensor_data)
                probe.create_subscription(Insert.Result, '/fr10_force/result',
                    lambda m: automatic_results.append(m), 10)
                def on_contacts(message):
                    contact_trace.append({'wall': time.monotonic(),
                        'source': message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                        'contacts': [{'a': state.collision1_name, 'b': state.collision2_name,
                                      'depths_m': list(state.depths)} for state in message.states]})
                probe.create_subscription(ContactsState, '/fr10_sim/station_contacts', on_contacts, qos_profile_sensor_data)
                def record_wrench(message, destination):
                    destination.append({'wall': time.monotonic(),
                        'source': message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                        'frame': message.header.frame_id,
                        'wrench': [message.wrench.force.x,message.wrench.force.y,message.wrench.force.z,
                                   message.wrench.torque.x,message.wrench.torque.y,message.wrench.torque.z]})
                probe.create_subscription(WrenchStamped, wrench_topic,
                    lambda m:record_wrench(m,raw_trace), qos_profile_sensor_data)
                if backend=='gazebo':
                    diagnostic_topic = '/ft_sensor/wrench' if gravity_in_source else '/fr10_simulation/contact_wrench'
                    probe.create_subscription(WrenchStamped, diagnostic_topic,
                        lambda m:record_wrench(m,diagnostic_wrench_trace), qos_profile_sensor_data)
                    summary['diagnostic_wrench_topic'] = diagnostic_topic
                probe.create_subscription(MotionCommandStatus, '/fr10_motion/command_status',
                    lambda m: commands.append({'wall': time.monotonic(), 'id': m.command_id,
                        'generation': m.generation, 'phase': m.phase, 'reason': m.reason}), 10)
                free_pose = probe.create_publisher(PoseStamped, '/fr10_motion/pose_target', 10)
                deadline = time.monotonic()+120
                summary['ready'] = False
                while all(process.poll() is None for process in processes) and time.monotonic()<deadline:
                    if probe.wait_ready(timeout=1): summary['ready'] = True; break
                if not summary['ready']: raise RuntimeError('matched runtime not ready')
                last_health_check = 0.; missing_motion_checks = 0
                def check_runtime():
                    nonlocal last_health_check, missing_motion_checks
                    if any(process.poll() is not None for process in processes):
                        raise RuntimeError('Owned runtime process exited during motion')
                    stamp=time.monotonic()
                    if stamp-last_health_check < 1.: return
                    last_health_check=stamp
                    missing_motion_checks = (0 if 'fr10_pose_controller' in probe.get_node_names()
                                             else missing_motion_checks+1)
                    if missing_motion_checks >= 2:
                        raise RuntimeError('Motion node exited before a terminal result; stop outcome unconfirmed')
                if backend=='gazebo':
                    processes.append(subprocess.Popen(commands_to_launch[-1], cwd=WORKSPACE,
                        env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
                    deadline=time.monotonic()+15
                    trace=output/'transport_state.jsonl'
                    while time.monotonic()<deadline and processes[-1].poll() is None:
                        if trace.is_file() and trace.stat().st_size>0: break
                        time.sleep(.05)
                    if not trace.is_file() or trace.stat().st_size==0:
                        raise RuntimeError('Independent Gazebo transport observer is not receiving poses')
                from fr10_model_evidence import capture_live_model
                summary.update(capture_live_model(probe,output))
                summary['controllers'] = list_controllers(probe)
                parameter_client=probe.create_client(GetParameters,'/fr10_pose_controller/get_parameters')
                if not parameter_client.wait_for_service(timeout_sec=10):
                    raise RuntimeError('Motion parameter evidence unavailable')
                parameter_request=GetParameters.Request()
                parameter_request.names=['controller.control_rate_hz','controller.feedback_timeout_s',
                    'controller.pose_policies.free_space.joint_velocity_scale',
                    'controller.pose_policies.free_space.joint_acceleration_scale',
                    'controller.pose_policies.free_space.joint_jerk_scale',
                    'controller.pose_policies.free_space.linear_scale',
                    'force.goal.insertion_speed_m_s','force.admittance.max_velocity',
                    'force.target_validity_s','force.command_lease_s','force.admittance.stiffness',
                    'force.admittance.damping','force.admittance.drive_deadband',
                    'force.depth_tolerance_m','force.max_nominal_lead_m','force.task_frame',
                    'force.base_motion_pose_window_s']
                parameter_values=await_future(parameter_client.call_async(parameter_request)).values
                from rcl_interfaces.msg import ParameterType
                parsed=[]
                for name,value in zip(parameter_request.names,parameter_values):
                    if value.type==ParameterType.PARAMETER_DOUBLE_ARRAY:
                        parsed.append(list(value.double_array_value))
                    elif name=='force.task_frame' and value.type==ParameterType.PARAMETER_STRING:
                        parsed.append(value.string_value)
                    elif name not in ('force.admittance.max_velocity','force.task_frame') and value.type==ParameterType.PARAMETER_DOUBLE:
                        parsed.append(value.double_value)
                    else: raise RuntimeError(f'Required motion parameter has wrong/undeclared type: {name}')
                summary['motion_parameters']=dict(zip(parameter_request.names,parsed))
                (output/'motion_parameters.json').write_text(json.dumps(summary['motion_parameters'],indent=2)+'\n')
                client = probe.create_client(GetParameters, '/matched_station_publisher/get_parameters')
                if not client.wait_for_service(timeout_sec=10): raise RuntimeError('station parameters unavailable')
                request = GetParameters.Request(); request.names = ['station_file']
                station_file = await_future(client.call_async(request)).values[0].string_value
                station = json.loads(Path(station_file).read_text())
                (output/'station.json').write_text(json.dumps(station, indent=2)+'\n')
                if backend=='gazebo':
                    world = Path(station_file).with_name('force_world.world')
                    if not world.is_file(): raise RuntimeError('Generated Gazebo physics world evidence missing')
                    import shutil, hashlib
                    shutil.copyfile(world, output/'physics.world')
                    summary['physics_world_sha256'] = hashlib.sha256(world.read_bytes()).hexdigest()
                summary['canonical_sha256'] = station['canonical_sha256']
                scene_client = probe.create_client(GetPlanningScene, '/get_planning_scene')
                if not scene_client.wait_for_service(timeout_sec=10): raise RuntimeError('PlanningScene unavailable')
                scene_request = GetPlanningScene.Request(); scene_request.components.components = 16
                deadline = time.monotonic()+15; object_ids = []
                while time.monotonic()<deadline:
                    scene = await_future(scene_client.call_async(scene_request)).scene
                    object_ids = [obj.id for obj in scene.world.collision_objects]
                    if {'charging_port_mating_surfaces','charging_port_protected_rear'} <= set(object_ids): break
                    time.sleep(.25)
                summary['scene_objects'] = object_ids
                if not {'charging_port_mating_surfaces','charging_port_protected_rear'} <= set(object_ids):
                    raise RuntimeError('exact CAD scene objects were not acknowledged')
                if mounted:
                    from fr10_matched_run_evidence import stationary_base_window
                    # Physics can initialize slower than wall time. Do not
                    # spend the controller's five-source-second parking
                    # admission on an unsettled startup fixture. The controller
                    # still acquires/rechecks its own parking lease after input.
                    startup_started=time.monotonic()
                    deadline=startup_started+run_timeout
                    while not stationary_base_window(list(physical_base_trace),time.monotonic(),
                            summary['motion_parameters']['force.base_motion_pose_window_s']):
                        check_runtime()
                        if time.monotonic()>=deadline:
                            raise RuntimeError('physical mounted startup did not qualify stationary feedback')
                        time.sleep(.02)
                    summary['physical_startup_stationary_wall']=time.monotonic()
                    summary['physical_startup_wait_wall_s']=time.monotonic()-startup_started
                    summary['physical_startup_source']=physical_base_trace[-1]['source']
                time.sleep(2)
                initial = probe.tcp_pose(); summary['initial_tcp'] = initial
                steady_start = time.monotonic()
                time.sleep(2)
                idle = [row for row in raw_trace if row['wall']>=steady_start]
                if idle:
                    values = np.array([row['wrench'] for row in idle])
                    summary['idle_raw_mean'] = values.mean(axis=0).tolist()
                    summary['idle_raw_std'] = values.std(axis=0).tolist()
                    summary['raw_frame'] = sorted({row['frame'] for row in idle})
                    summary['known_3kg_force_norm_n'] = float(np.linalg.norm(values.mean(axis=0)[:3]))
                    summary['idle_wrench_stable'] = bool(np.max(values.std(axis=0)[:3])<.3 and
                                                        np.max(values.std(axis=0)[3:])<.05)
                    summary['known_load_pass'] = summary['idle_wrench_stable'] and (abs(summary['known_3kg_force_norm_n']-3*9.80665)<.3 if not gravity_in_source
                        else summary['known_3kg_force_norm_n']<.05)
                    summary['idle_wrench_reference'] = 'summed physical contact wrench at TCP; zero in free space' if gravity_in_source else '3 kg gravity load'
                if not summary.get('known_load_pass', False):
                    raise RuntimeError('idle physical wrench failed known-load calibration; motion was not admitted')
                def publish_pose(position, orientation):
                    message = PoseStamped(); message.header.frame_id = 'fr10_base_link'
                    message.pose.position.x,message.pose.position.y,message.pose.position.z = position
                    message.pose.orientation.x,message.pose.orientation.y,message.pose.orientation.z,message.pose.orientation.w = orientation
                    free_pose.publish(message)
                preinsert = station['preinsert_position_arm_m']; orientation = station['orientation_arm_xyzw']
                if not pose_input:
                    target = list(initial[0]); target[0] += .020
                    first_event = len(commands)
                    publish_pose(target, initial[1])
                    summary['free_pose_reached'] = wait_pose_result(probe, target, initial[1], commands, first_event, 90)
                    if not summary['free_pose_reached']: raise RuntimeError('free POSE did not complete')
                    first_event = len(commands)
                    publish_pose(preinsert, orientation)
                    summary['preinsert_reached'] = wait_pose_result(probe, preinsert, orientation, commands, first_event, 120)
                    summary['preinsert_tcp'] = probe.tcp_pose()
                    summary['preinsert_status'] = probe.status()
                    summary.update(write_feedback_trace(probe, output))
                    if not summary['preinsert_reached']: raise RuntimeError('CAD preinsert path failed')
                else:
                    summary['input_mode'] = 'one_socket_pose_from_initial_state'
                if calibrate:
                    geometry = json.loads((Path(station['model_path'])/'geometry.json').read_text())
                    checks = []
                    for axis, degrees in [(0,0), (0,5), (0,-5), (1,5), (1,-5), (2,5)]:
                        q = compose_rotation(orientation, axis, math.radians(degrees))
                        first_event = len(commands); publish_pose(preinsert, q)
                        if not wait_pose_result(probe, preinsert, q, commands, first_event, 60):
                            raise RuntimeError('gravity orientation calibration POSE failed')
                        time.sleep(.5); started = time.monotonic(); time.sleep(1.)
                        samples = [row for row in raw_trace if row['wall'] >= started]
                        if len(samples) < 20: raise RuntimeError('insufficient unique load samples')
                        mean = np.asarray([row['wrench'] for row in samples]).mean(axis=0)
                        item = check_known_load(mean, probe.tcp_pose()[1], geometry['tool_mass_kg'],
                                                geometry['tool_cog_sensor_m'])
                        item.update(axis=axis, degrees=degrees, sample_count=len(samples))
                        checks.append(item)
                    summary['gravity_orientation_checks'] = checks
                    summary['gravity_orientation_pass'] = all(item['pass'] for item in checks)
                    first_event = len(commands); publish_pose(preinsert, orientation)
                    if not wait_pose_result(probe, preinsert, orientation, commands, first_event, 60):
                        raise RuntimeError('return from calibration failed')
                if insert:
                    from rclpy.action import ActionClient
                    from fr10_motion_interfaces.action import Insert
                    action = ActionClient(probe, Insert, '/fr10_force/insert')
                    if not action.wait_for_server(timeout_sec=5): raise RuntimeError('Insert action unavailable')
                    goal = Insert.Goal(); goal.target_pose.header.frame_id = 'fr10_base_link'
                    goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,goal.target_pose.pose.position.z = station['seated_position_arm_m']
                    goal.target_pose.pose.orientation.x,goal.target_pose.pose.orientation.y,goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w = orientation
                    goal.preinsert_distance = station['preinsert_distance_m']; goal.insertion_depth = station['insertion_depth_m']
                    goal.insertion_speed = .002; goal.desired_wrench.force.z = 5.
                    goal.selected_axes = [True]*6 if six_axis else [False,False,True,False,False,False]
                    goal.max_force = [50.,50.,50.]; goal.max_torque = [10.,10.,10.]
                    goal.timeout.sec = 150; goal.profile_id = 'matched_socket_v2'; goal.completion_behavior = 0
                    last_feedback_state = None
                    def on_feedback(message):
                        nonlocal last_feedback_state
                        item = message.feedback
                        if item.state != last_feedback_state:
                            last_feedback_state = item.state
                            print(f'Force phase: {item.detail.split(";")[0]}; '
                                  f'depth={item.depth * 1000.:.2f} mm; '
                                  f'Fz={item.measured_wrench.force.z:.3f} N', flush=True)
                        feedback_trace.append({'wall': time.monotonic(), 'state': item.state,
                            'depth': item.depth, 'correction': list(item.correction), 'detail': item.detail,
                            'contact_state':item.contact_state,'source_age_s':item.source_age,
                            'force_error':[item.force_error.force.x,item.force_error.force.y,item.force_error.force.z,
                                item.force_error.torque.x,item.force_error.torque.y,item.force_error.torque.z],
                            'nominal': [item.nominal_pose.position.x,item.nominal_pose.position.y,item.nominal_pose.position.z],
                            'corrected': [item.corrected_pose.position.x,item.corrected_pose.position.y,item.corrected_pose.position.z],
                            'wrench': [item.measured_wrench.force.x,item.measured_wrench.force.y,item.measured_wrench.force.z,
                                       item.measured_wrench.torque.x,item.measured_wrench.torque.y,item.measured_wrench.torque.z]})
                        with (output/'insert_feedback_live.jsonl').open('a') as stream:
                            stream.write(json.dumps(feedback_trace[-1])+'\n')
                    if pose_input:
                        probe.create_subscription(Insert.Feedback, '/fr10_force/feedback',
                            lambda m: on_feedback(type('Feedback', (), {'feedback': m})()), qos_profile_sensor_data)
                        mouth_pub = probe.create_publisher(PoseStamped, '/charging_port/raw_pose', 10)
                        time.sleep(.5)
                        message = PoseStamped(); message.header.frame_id = 'fr10_base_link'
                        # The probe runs on wall time; the TCP stamp is from
                        # the controller's simulation clock. Do not mix clocks.
                        with probe.lock:
                            message.header.stamp = probe.latest_tcp.header.stamp
                        input_position=station['mouth_position_arm_m'];input_orientation=orientation
                        if mounted:
                            observation=base_tf_buffer.lookup_transform('fr10_base_link','world',Time(),
                                timeout=Duration(seconds=5.))
                            frame=observation.transform
                            message.header.stamp=observation.header.stamp
                            turn=Rotation.from_quat([frame.rotation.x,frame.rotation.y,frame.rotation.z,frame.rotation.w])
                            input_position=(turn.apply(station['world_mouth_position_m'])+
                                np.array([frame.translation.x,frame.translation.y,frame.translation.z])).tolist()
                            input_orientation=(turn*Rotation.from_euler('xyz',station['world_mouth_rpy'])).as_quat().tolist()
                        summary['true_socket_input_pose'] = {'position': list(input_position),
                            'orientation_xyzw': list(input_orientation)}
                        input_position, input_orientation = input_socket_pose(input_position, input_orientation,
                            pose_lateral_mm, pose_rotation_deg, **error_limits)
                        summary['pose_uncertainty'] = {'lateral_mm': list(pose_lateral_mm),
                            'translation_mm': [*pose_lateral_mm, pose_axial_mm],
                            'rotation_vector_deg': list(pose_rotation_deg), 'physics_geometry_modified': False,
                            'input_error_limit_mm_deg': pose_error_limit}
                        message.pose.position.x,message.pose.position.y,message.pose.position.z=input_position
                        message.pose.orientation.x,message.pose.orientation.y,message.pose.orientation.z,message.pose.orientation.w=input_orientation
                        summary['input_socket_pose']={'frame':message.header.frame_id,
                            'position':input_position,'orientation_xyzw':input_orientation}
                        mouth_pub.publish(message)
                        # Mesh solvers may run slower than real time. The core
                        # goal deadline and wrench safety remain unchanged;
                        # this is only the external experiment wall-time cap.
                        deadline = time.monotonic()+run_timeout
                        while not automatic_results and time.monotonic()<deadline:
                            check_runtime()
                            time.sleep(.05)
                        if not automatic_results: raise RuntimeError('automatic socket pose timed out')
                        result = automatic_results[-1]
                        summary['insert_accepted'] = True
                        summary['insert_result'] = {'success': result.success, 'code': result.code,
                            'depth': result.final_depth, 'message': result.message,
                            'controlled_stop_completed': result.controlled_stop_completed}
                        handle = None
                    else:
                        handle = await_future(action.send_goal_async(goal, feedback_callback=on_feedback))
                    if handle is not None: summary['insert_accepted'] = handle.accepted
                    if handle is not None and handle.accepted:
                        response = await_future(handle.get_result_async(), timeout=run_timeout)
                        result = response.result
                        summary['insert_result'] = {'success': result.success, 'code': result.code,
                            'depth': result.final_depth, 'message': result.message,
                            'controlled_stop_completed': result.controlled_stop_completed}
                    summary['result_received_wall']=time.monotonic()
                    def measured_source():
                        with probe.lock:
                            if probe.latest_joint is None:
                                raise RuntimeError('No measured joint source clock for HOLD')
                            stamp=probe.latest_joint.header.stamp
                            return stamp.sec+stamp.nanosec*1e-9
                    summary['hold_observation_minimum_source_s']=5.0
                    if summary['insert_result']['success']:
                        from fr10_matched_run_evidence import hold_duration_complete,hold_feedback_boundary
                        transition_deadline=time.monotonic()+5.
                        hold_boundary=None
                        while hold_boundary is None:
                            hold_boundary=hold_feedback_boundary(feedback_trace,summary['result_received_wall'])
                            if hold_boundary is not None: break
                            check_runtime()
                            if time.monotonic()>transition_deadline:
                                raise RuntimeError('Successful result was not followed by observed HOLD')
                            time.sleep(.01)
                        # Start after a new measured source frame, beyond any
                        # backlog present when the independent result arrived.
                        with probe.lock:
                            tcp_stamp=probe.latest_tcp.header.stamp
                            joint_stamp=probe.latest_joint.header.stamp
                            source_floor=max(joint_stamp.sec+joint_stamp.nanosec*1e-9,
                                             tcp_stamp.sec+tcp_stamp.nanosec*1e-9)
                        while measured_source()<=source_floor:
                            check_runtime()
                            if time.monotonic()>transition_deadline:
                                raise RuntimeError('No fresh measured source frame at HOLD boundary')
                            time.sleep(.005)
                    source_start=measured_source()
                    wall_start=time.monotonic()
                    summary['hold_observation_start_wall']=wall_start
                    summary['hold_transition_wait_wall_s']=wall_start-summary['result_received_wall']
                    hold_start = probe.tcp_pose()
                    if summary['insert_result']['success']:
                        hold_feedback_begin=hold_boundary+1
                        while not hold_duration_complete(wall_start,time.monotonic(),source_start,measured_source()):
                            check_runtime()
                            if time.monotonic()-wall_start>run_timeout:
                                raise RuntimeError('Measured HOLD source clock did not complete observation')
                            new_feedback=feedback_trace[hold_feedback_begin:]
                            hold_feedback_begin+=len(new_feedback)
                            if any(s['state']!=Insert.Feedback.HOLDING for s in new_feedback):
                                raise RuntimeError('Force session left HOLD during source-time observation')
                            time.sleep(.05)
                    else:
                        time.sleep(5.)
                    summary['hold_observation_duration_s']=time.monotonic()-wall_start
                    summary['hold_observation_source_duration_s']=measured_source()-source_start
                    hold_end = probe.tcp_pose()
                    summary['hold_drift_m'] = float(np.linalg.norm(np.asarray(hold_end[0])-np.asarray(hold_start[0])))
                    held_feedback=[sample for sample in feedback_trace if sample['wall']>=summary['hold_observation_start_wall']]
                    summary['held_force_feedback_samples']=len(held_feedback)
                    summary['insertion_hold_pass'] = ('state=HOLDING' in probe.status() and
                        summary['hold_observation_source_duration_s']>=5. and
                        summary['hold_drift_m'] < .0005 and len(held_feedback)>=3 and
                        all(sample['state']==Insert.Feedback.HOLDING for sample in held_feedback))
                    summary['hold_evidence_end_wall']=time.monotonic()
                    if fault_after_hold:
                        if not (summary['insert_result']['success'] and summary['insertion_hold_pass']):
                            raise RuntimeError('Fault injection requires verified seated HOLD')
                        from fr10_matched_run_evidence import evaluate_physical_fit
                        fit=evaluate_physical_fit(list(independent_tcp_trace),station,geometry,
                            summary['result_received_wall'],summary['hold_evidence_end_wall'])
                        summary['pre_fault_physical_fit']=fit
                        if fit.get('physical_seating_gate')!='PASS':
                            raise RuntimeError('Fault injection requires independent physical seated fit')
                        from std_msgs.msg import String
                        fault_pub=probe.create_publisher(String,'/fr10_mujoco/wrench_fault',1)
                        time.sleep(.25);first_result=len(automatic_results)
                        summary['fault_requested_wall']=time.monotonic()
                        fault_pub.publish(String(data=fault_after_hold))
                        deadline=time.monotonic()+20
                        while len(automatic_results)<=first_result and time.monotonic()<deadline:
                            check_runtime()
                            time.sleep(.05)
                        if len(automatic_results)<=first_result:raise RuntimeError('Fault did not produce a controlled-stop result')
                        result=automatic_results[-1]
                        time.sleep(.5)
                        summary['fault_result']={'success':result.success,'code':result.code,
                            'message':result.message,'controlled_stop_completed':result.controlled_stop_completed}
                        summary['fault_mode']=fault_after_hold;summary['fault_status']=probe.status()
                        expected={'duplicate':3,'nan':1,'overload':5,'torque':6,'lost_contact':10}[fault_after_hold]
                        summary['fault_evidence_pass']=not result.success and result.controlled_stop_completed and \
                            f'safety_fault={expected};' in result.message and 'state=HOLDING' in probe.status()
                    if retract_after_hold:
                        if not (summary['insert_result']['success'] and summary['insertion_hold_pass']):
                            raise RuntimeError('Retract was requested but seated HOLD was not verified')
                        from std_msgs.msg import Empty
                        retract_pub=probe.create_publisher(Empty,'/fr10_motion/retract',10)
                        time.sleep(.25)
                        first_result=len(automatic_results)
                        summary['retract_requested_wall']=time.monotonic()
                        retract_pub.publish(Empty())
                        deadline=time.monotonic()+run_timeout
                        while len(automatic_results)<=first_result and time.monotonic()<deadline:
                            check_runtime()
                            time.sleep(.05)
                        if len(automatic_results)<=first_result: raise RuntimeError('Explicit retract result timed out')
                        result=automatic_results[-1]
                        summary['retract_result']={'success':result.success,'code':result.code,
                            'depth':result.final_depth,'message':result.message}
                        summary['retract_result_wall']=time.monotonic()
                        time.sleep(1.)
                        summary['retract_status']=probe.status()
                        summary['retract_reported_tcp']=probe.tcp_pose()
                    summary.update(write_feedback_trace(probe, output))
                    action.destroy()
        except Exception as error:
            summary['error'] = repr(error)
        finally:
            if 'probe' in locals():
                summary.update(write_feedback_trace(probe, output))
                with probe.lock:
                    feedback=[{'wall':state[0],'source':stamp[1],
                               'position':state[1],'velocity':state[2]}
                              for state,stamp in zip(probe.joint_history,probe.joint_source_history)]
                (output/'joint_feedback.json').write_text(json.dumps(feedback)+'\n')
            stop_owned_motion_node(processes, output/'runtime.log', summary)
            summary['process_exit_before_shutdown'] = [process.poll() for process in processes]
            for process in reversed(processes):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try: process.wait(timeout=20)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM); process.wait(timeout=10)
    runtime = (output/'runtime.log').read_text(errors='replace')
    summary['ode_lcp_errors'] = runtime.count('LCP internal error')
    summary['shutdown_braking_errors']=runtime.count('Shutdown braking deadline expired')
    pairs = [contact for row in contact_trace for contact in row['contacts']]
    summary['station_contact_samples'] = len(contact_trace)
    summary['station_contact_pairs'] = sorted({(item['a'], item['b']) for item in pairs})
    summary['station_peak_penetration_m'] = max([depth for item in pairs for depth in item['depths_m']], default=0.)
    summary['unexpected_station_contacts'] = sum(1 for item in pairs if
        not (any('::charging_mating_nose' in name or '::charging_mating_shoulder' in name or 'fixed_joint_lump__mating_surface_' in name or 'fixed_joint_lump__seating_shoulder' in name
                 for name in (item['a'], item['b'])) and
             any(name.endswith('::mating_surface') for name in (item['a'], item['b']))))
    if backend in ('mujoco','fairino'):
        summary['unexpected_station_contacts'] = sum(1 for item in pairs if not (
            any(name.startswith(('charging_mating_nose__','charging_mating_shoulder__')) for name in (item['a'],item['b'])) and
            any(name.startswith('socket_surface_') for name in (item['a'],item['b']))))
        summary['contact_surface_representation'] = 'rigid original nonconvex triangle flex, zero radius'
    summary['real_contact'] = backend in ('gazebo','mujoco') and bool(pairs)
    summary['physical_robot_contact'] = False
    summary['physics_contacts_observed'] = bool(pairs)
    summary['all_robot_contact_gate'] = ('ALL_FIXTURE_CONTACTS_OBSERVED_SELF_BY_PLANNING_SCENE'
        if backend in ('mujoco','fairino') else 'NOT_OBSERVED_BY_STATION_SENSOR')
    if backend=='gazebo' and (output/'transport_contacts.jsonl').is_file():
        def allowed_world(contact):
            names=(contact['a'],contact['b'])
            mating=any('::charging_mating_nose' in name or '::charging_mating_shoulder' in name or
                'fixed_joint_lump__mating_surface_' in name or 'fixed_joint_lump__seating_shoulder' in name for name in names)
            socket=any(name.endswith('::mating_surface') for name in names)
            wheel_ground=mounted and any('::Wheel' in name for name in names) and any('matched_ground::' in name for name in names)
            return (mating and socket) or wheel_ground
        count=unexpected=0;world_pairs=set();maximum_depth=0.
        with (output/'transport_contacts.jsonl').open() as stream:
            for line in stream:
                row=json.loads(line);count+=1
                for contact in row['contacts']:
                    world_pairs.add((contact['a'],contact['b']))
                    unexpected+=not allowed_world(contact)
                    maximum_depth=max(maximum_depth,max(contact['depths_m'],default=0.))
        summary.update(world_contact_samples=count,unexpected_world_contacts=unexpected,
            world_contact_pairs=sorted(world_pairs),world_peak_penetration_m=maximum_depth)
        summary['all_robot_contact_gate']='WORLD_CONTACTS_OBSERVED' if count else 'NO_WORLD_CONTACT_SAMPLES'
    from fr10_matched_run_evidence import physical_tcp_trace,evaluate_physical_fit,evaluate_fk_consistency
    physical=physical_tcp_trace(output,backend,independent_tcp_trace)
    (output/'physical_tcp.json').write_text(json.dumps(physical)+'\n')
    (output/'reported_tcp.json').write_text(json.dumps(reported_tcp_trace)+'\n')
    if physical and 'station' in locals():
        geometry=json.loads((Path(station['model_path'])/'geometry.json').read_text())
        if insert and 'result_received_wall' in summary:
            summary.update(evaluate_physical_fit(physical,station,geometry,summary['result_received_wall'],
                summary.get('hold_evidence_end_wall')))
        if backend=='gazebo':
            summary.update(evaluate_fk_consistency(physical,reported_tcp_trace))
            if 'result_received_wall' in summary:
                held=[s for s in physical if summary['result_received_wall']<=s['wall']<=summary.get('hold_evidence_end_wall',float('inf'))]
                held_fk=evaluate_fk_consistency(held,reported_tcp_trace)
                summary['stationary_fk_consistency']=held_fk
    if retract_after_hold and 'retract_result_wall' in summary and 'station' in locals():
        from fr10_matched_run_evidence import evaluate_retract
        summary.update(evaluate_retract(physical,station,summary['retract_result_wall']))
        recent_raw=[s for s in raw_trace if s['wall']>=summary['retract_result_wall']+.1]
        unloaded=[]
        if not gravity_in_source and physical:
            physical_times=np.asarray([s['source'] for s in physical])
            geometry=json.loads((Path(station['model_path'])/'geometry.json').read_text())
            for sample in recent_raw:
                index=int(np.argmin(np.abs(physical_times-sample['source'])))
                if abs(physical_times[index]-sample['source'])>.055: continue
                check=check_known_load(sample['wrench'],physical[index]['orientation_xyzw'],
                    geometry['tool_mass_kg'],geometry['tool_cog_sensor_m'],force_tolerance=1.,torque_tolerance=.1)
                unloaded.append(check['pass'])
        else:
            unloaded=[np.linalg.norm(s['wrench'][:3])<1. for s in recent_raw]
        summary['retract_unloaded_samples']=len(unloaded)
        summary['retract_unloaded_pass']=len(unloaded)>=3 and all(unloaded)
        summary['parking_release_pass'] = not mounted or any(
            row['wall'] > summary['retract_result_wall'] and row['owner'] == 0 and row['state'] == 0
            for row in parking_trace)
        summary['retract_pass']=bool(summary['retract_result']['success']) and \
            summary['physical_retract_gate']=='PASS' and summary['retract_unloaded_pass'] and \
            summary['parking_release_pass'] and \
            'state=HOLDING' in summary['retract_status'] and 'owner=FORCE_ONLINE' not in summary['retract_status']
    motion_keys = ('ready','insert_accepted') if pose_input else ('ready','free_pose_reached','preinsert_reached')
    summary['motion_pass'] = all(summary.get(key, False) for key in motion_keys) and \
        summary['ode_lcp_errors'] == 0 and 'error' not in summary
    summary['contact_pass'] = (bool(summary.get('insert_result', {}).get('success')) and
        summary.get('insertion_hold_pass',False) and summary.get('physical_seating_gate')=='PASS' and
        bool(pairs) and summary['unexpected_station_contacts']==0 and summary.get('unexpected_world_contacts',0)==0 and
        (backend!='gazebo' or (summary.get('world_contact_samples',0)>0 and
            summary.get('stationary_fk_consistency',{}).get('physical_fk_max_position_error_m',float('inf'))<=.0005)) and
        summary['station_peak_penetration_m'] <= (.0005 if backend=='gazebo' else .00005))
    summary['pass'] = summary['motion_pass'] and summary.get('known_load_pass', False) and \
        summary.get('shutdown_hold_confirmed', False) and summary.get('motion_shutdown_before_physics',False) and \
        summary['shutdown_braking_errors']==0 and (not insert or summary['contact_pass']) and \
        (not calibrate or summary.get('gravity_orientation_pass', False)) and \
        (not retract_after_hold or summary.get('retract_pass',False)) and \
        (not fault_after_hold or summary.get('fault_evidence_pass',False))
    from fr10_matched_run_evidence import self_alignment_result
    summary.update(self_alignment_result(summary))
    (output/'command_status.json').write_text(json.dumps(commands, indent=2)+'\n')
    (output/'raw_wrench.json').write_text(json.dumps(raw_trace, indent=2)+'\n')
    if backend=='gazebo':
        (output/'diagnostic_wrench.json').write_text(json.dumps(diagnostic_wrench_trace)+'\n')
    (output/'insert_feedback.json').write_text(json.dumps(feedback_trace, indent=2)+'\n')
    (output/'joint_reference.json').write_text(json.dumps(joint_reference_trace)+'\n')
    (output/'station_contacts.json').write_text(json.dumps(contact_trace, indent=2)+'\n')
    if mounted:
        (output/'base_odometry.json').write_text(json.dumps(base_trace)+'\n')
        (output/'physical_base_odometry.json').write_text(json.dumps(physical_base_trace)+'\n')
        (output/'parking_state.json').write_text(json.dumps(parking_trace)+'\n')
    (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))
    return 0 if summary['pass'] else 1

if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('output', type=Path)
    parser.add_argument('--domain', type=int, default=230); parser.add_argument('--port', type=int, default=11980)
    parser.add_argument('--station', default='front_mid'); parser.add_argument('--mounted', action='store_true')
    parser.add_argument('--initial-state', choices=['home','left','right','folded'], default='home')
    parser.add_argument('--run-timeout', type=float, default=900.)
    parser.add_argument('--overlay', type=Path); parser.add_argument('--insert', action='store_true')
    parser.add_argument('--calibrate', action='store_true')
    parser.add_argument('--six-axis', action='store_true')
    parser.add_argument('--pose-input', action='store_true')
    parser.add_argument('--backend',choices=['gazebo','mujoco','fairino'],default='gazebo')
    parser.add_argument('--fault-after-hold',choices=['duplicate','nan','overload','torque','lost_contact'])
    parser.add_argument('--retract-after-hold',action='store_true')
    parser.add_argument('--native-probe',type=Path)
    parser.add_argument('--station-file',type=Path)
    parser.add_argument('--pose-lateral-mm', type=float, nargs=2, default=(0., 0.))
    parser.add_argument('--pose-rotation-deg', type=float, nargs=3, default=(0., 0., 0.))
    parser.add_argument('--gazebo-step-s', type=float)
    parser.add_argument('--gazebo-solver-iterations', type=int)
    parser.add_argument('--no-arm-camera', action='store_true')
    parser.add_argument('--pose-error-limit', type=float, choices=(2., 5.), default=2.)
    parser.add_argument('--pose-axial-mm', type=float, default=0.)
    args = parser.parse_args(); raise SystemExit(run(args.output.resolve(), args.domain, args.port,
        args.station, args.mounted, args.overlay, args.insert, args.calibrate, args.six_axis, args.pose_input,
        args.backend,args.station_file,args.initial_state,args.run_timeout,args.native_probe,args.retract_after_hold,args.fault_after_hold,
        args.pose_lateral_mm,args.pose_rotation_deg,args.gazebo_step_s,
        args.gazebo_solver_iterations,not args.no_arm_camera,args.pose_error_limit,args.pose_axial_mm))
