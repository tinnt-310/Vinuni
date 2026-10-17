#!/usr/bin/env python3
"""Canonical station/start matrix without actuator publishers or SDK calls."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml
from rclpy.serialization import serialize_message
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from fr10_matched_run_manifest import capture

ROOT=Path(__file__).resolve().parents[4]
DESCRIPTION=ROOT/'src/vehicle_description'
POSE=ROOT/'src/fr10_arm_control/fr10_pose_controller'


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result


def run(output,domain):
    output.mkdir(parents=True,exist_ok=False)
    env=dict(os.environ,ROS_DOMAIN_ID=str(domain),RMW_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(output/'ros_log'))
    build=output/'probe_build';build.mkdir()
    (build/'CMakeLists.txt').write_text(f'''cmake_minimum_required(VERSION 3.8)
project(fr10_readonly_planning_probe)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fr10_motion_core REQUIRED)
find_package(fr10_motion_planning REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(yaml-cpp REQUIRED)
add_executable(probe "{ROOT}/src/vehicle_bringup/test/integration/fr10_planning_probe.cpp" "{POSE}/src/node_parameters.cpp")
target_include_directories(probe PRIVATE "{POSE}/include")
target_compile_features(probe PRIVATE cxx_std_17)
target_compile_options(probe PRIVATE -O3)
target_compile_definitions(probe PRIVATE EIGEN_MAX_ALIGN_BYTES=16 EIGEN_MALLOC_ALREADY_ALIGNED=1)
target_link_libraries(probe yaml-cpp)
ament_target_dependencies(probe rclcpp fr10_motion_core fr10_motion_planning moveit_ros_planning)
''')
    with (output/'build.log').open('w') as log:
        subprocess.run(['cmake','-S',str(build),'-B',str(build/'bin')],check=True,stdout=log,stderr=log,env=env)
        subprocess.run(['cmake','--build',str(build/'bin'),'-j','2'],check=True,stdout=log,stderr=log,env=env)
    binary=build/'bin/probe'
    capture(ROOT,output,[binary])
    physical=subprocess.check_output(['xacro',str(DESCRIPTION/'urdf/vehicle.urdf.xacro'),
        'enable_arm:=true','enable_vehicle:=false','enable_force_sensor:=true',
        'charging_tool_profile:=matched_socket_v2','control_backend:=none'],text=True,env=env)
    collision=module('planning_collision_model',POSE/'launch/planning_collision_model.py')
    model=DESCRIPTION/'models/matched_socket_v2'
    planning=collision.planning_collision_description(physical,model)
    (output/'model.urdf').write_text(physical);(output/'planning_model.urdf').write_text(planning)
    semantic=(DESCRIPTION/'config/fr10.srdf').read_text()
    assembly=ET.parse(model/'matched_connector.srdf').getroot()
    semantic=semantic.replace('</robot>',''.join(ET.tostring(e,encoding='unicode') for e in assembly)+'</robot>')
    station_module=module('charging_station',ROOT/'src/vehicle_bringup/launch/charging_station.py')
    publisher=module('matched_station_publisher',ROOT/'src/vehicle_bringup/scripts/matched_station_publisher.py')
    registry=ROOT/'src/vehicle_bringup/config/charging_stations.yaml'
    definitions=yaml.safe_load(registry.read_text());reports=[]
    for name in definitions['stations']:
        station=station_module.select_station(registry,model/'geometry.json',name,physical,False)
        scene=PlanningScene();scene.is_diff=True
        for object_id,filename in (('charging_port_mating_surfaces','socket_contact_surface.stl'),
                                  ('charging_port_protected_rear','socket_protected_rear.stl')):
            obj=CollisionObject();obj.header.frame_id='fr10_base_link';obj.id=object_id;obj.operation=obj.ADD
            pose=Pose();pose.position.x,pose.position.y,pose.position.z=station['mouth_position_arm_m']
            pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=station['orientation_arm_xyzw']
            obj.meshes=[publisher.cad_mesh(model/filename)];obj.mesh_poses=[pose];scene.world.collision_objects.append(obj)
        for primitive in station['world_primitives']:
            obj=CollisionObject();obj.header.frame_id='fr10_base_link';obj.id=primitive['id'];obj.operation=obj.ADD
            shape=SolidPrimitive();shape.type=shape.BOX;shape.dimensions=primitive['size_m']
            pose=Pose();pose.position.x,pose.position.y,pose.position.z=primitive['position_world_m'];pose.orientation.w=1.
            obj.primitives=[shape];obj.primitive_poses=[pose];scene.world.collision_objects.append(obj)
        # The C++ probe extends SRDF ACM with only the runtime mating pairs.
        scene_file=output/f'{name}.scene';scene_file.write_bytes(serialize_message(scene))
        for start,q in definitions['initial_joint_rad'].items():
            identifier=f'{name}_{start}';result=output/f'{identifier}.yaml'
            parameters=yaml.safe_load((POSE/'config/controller.yaml').read_text())
            values=parameters['fr10_pose_controller']['ros__parameters']
            values.update(robot_description=planning,robot_description_semantic=semantic,
                **{'probe.scene_file':str(scene_file),'probe.output_file':str(result),
                   'probe.initial_joint_rad':q,'probe.mouth_position_m':station['mouth_position_arm_m'],
                   'probe.orientation_xyzw':station['orientation_arm_xyzw'],
                   'probe.approach_m':station['preinsert_distance_m'],'probe.depth_m':station['insertion_depth_m']})
            parameter_file=output/f'{identifier}.params.yaml';parameter_file.write_text(yaml.safe_dump(parameters))
            with (output/f'{identifier}.log').open('w') as log:
                try:
                    process=subprocess.run([str(binary),'--ros-args','--params-file',str(parameter_file)],
                        env=env,stdout=log,stderr=log,timeout=180)
                    exit_code=process.returncode
                except subprocess.TimeoutExpired:
                    exit_code=124
            report=yaml.safe_load(result.read_text()) if result.is_file() else {'pass':False,'error':'probe exited without report'}
            report.update(station=name,initial_state=start,exit_code=exit_code);reports.append(report)
            print(f'{identifier}: {"PASS" if report["pass"] else "FAIL"}',flush=True)
            (output/'results.json').write_text(json.dumps(reports,indent=2)+'\n')
    summary=dict(cases=len(reports),passed=sum(bool(r['pass']) for r in reports),
        actuation=False,physical_contact_acceptance=False,results=reports)
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    return 0 if summary['passed']==summary['cases'] else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path);parser.add_argument('--domain',type=int,default=194)
    args=parser.parse_args()
    if not 0<=args.domain<=232:raise ValueError('Invalid isolated DDS domain')
    raise SystemExit(run(args.output.resolve(),args.domain))
