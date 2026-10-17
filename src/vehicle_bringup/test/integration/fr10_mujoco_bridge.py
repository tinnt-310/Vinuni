#!/usr/bin/env python3
"""MuJoCo adapter for the shared FR10 ROS force-position controller.

This is a simulator adapter, not a second force controller.  It consumes the
dense joint-position stream owned by MotionExecutor and publishes the same
JointState and WrenchStamped contracts used by the Gazebo and hardware paths.
The model is generated from vehicle_description's canonical FR10 Xacro.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[4]
DESCRIPTION = ROOT / "src/vehicle_description"
XACRO = DESCRIPTION / "urdf/vehicle.urdf.xacro"
JOINT_NAMES = [f"j{index}" for index in range(1, 7)]
HOME = np.array([0.0, -math.pi / 2.0, math.pi / 2.0,
                 -math.pi / 2.0, -math.pi / 2.0, 0.0])
# cong_sac.stl is authored in millimetres.  Its front mating plane is the
# minimum-Y face after the model.sdf +5 mm mesh offset.  The socket pattern is
# centred at X=0, Z=767.5 mm and insertion proceeds along mesh +Y.  This proper
# rotation maps the charging_tcp axes into the mesh axes:
#   TCP +X -> mesh -X, TCP +Y -> mesh +Z, TCP +Z -> mesh +Y.
PORT_TCP_IN_MESH = np.array([0.0, -0.0015, 0.7675])
PORT_VISUAL_OFFSET = np.array([0.0, 0.005, 0.0])
R_PORT_MESH_TO_TCP = np.array([
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0],
])
PORT_MESH = DESCRIPTION / "models/cong_sac/meshes/cong_sac.stl"

def simulation_pd_gains():
    values=yaml.safe_load((DESCRIPTION/'config/fr10_simulation_actuators.yaml').read_text())
    kp,kd,ki=(np.asarray(values[key],dtype=float) for key in ('kp','kd','ki'))
    if any(x.shape!=(6,) or not np.isfinite(x).all() for x in (kp,kd,ki)) or \
            np.any(kp<=0) or np.any(kd<0) or np.any(ki!=0):
        raise ValueError('Canonical MuJoCo PD gains require six finite kp>0, kd>=0, ki=0 values')
    return kp,kd


def _format(values: np.ndarray) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def _matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    quaternion = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quaternion, np.asarray(matrix, dtype=np.float64).reshape(9))
    return quaternion


def _generate_urdf(destination: Path) -> None:
    subprocess.run(
        [
            "xacro", str(XACRO),
            "enable_vehicle:=false", "enable_arm:=true",
            "enable_force_sensor:=true", "control_backend:=none",
            "-o", str(destination),
        ],
        cwd=ROOT,
        check=True,
    )
    text = destination.read_text()
    text = text.replace(
        "package://vehicle_description/", f"{DESCRIPTION}/")
    destination.write_text(text)


class Fr10MujocoPlant:
    """Fixed-base FR10 plant with the CCS2 port and a compliant force proxy."""

    def __init__(self, fixture_gap_m: float = 0.005,
                 physics_step_s: float = 0.001, station_file=None,
                 include_visuals: bool = False) -> None:
        if not math.isfinite(fixture_gap_m) or fixture_gap_m <= 0.0:
            raise ValueError("fixture_gap_m must be positive and finite")
        if not math.isfinite(physics_step_s) or physics_step_s <= 0.0:
            raise ValueError("physics_step_s must be positive and finite")
        self.fixture_gap_m = fixture_gap_m
        self.physics_step_s = physics_step_s
        if station_file is not None:
            from fr10_matched_mujoco import configure_matched_plant
            configure_matched_plant(self, station_file, physics_step_s, include_visuals=include_visuals)
            return
        self._temporary_directory = tempfile.TemporaryDirectory(
            prefix="fr10_mujoco_")
        directory = Path(self._temporary_directory.name)
        urdf_path = directory / "fr10.urdf"
        base_mjcf_path = directory / "fr10_base.xml"
        model_path = directory / "fr10_force.xml"
        _generate_urdf(urdf_path)
        from fr10_model_evidence import fixed_link_transform
        robot = ET.parse(urdf_path).getroot()
        wrist_to_tcp = fixed_link_transform(robot, 'wrist3_link', 'charging_tcp')
        wrist_to_connector = fixed_link_transform(robot, 'wrist3_link', 'charging_connector_link')
        self.tcp_in_wrist = wrist_to_tcp[:3, 3].copy()
        self.rotation_wrist_to_tcp = wrist_to_tcp[:3, :3].copy()

        base_model = mujoco.MjModel.from_xml_path(str(urdf_path))
        if base_model.nq != 6 or base_model.nv != 6:
            raise RuntimeError(
                f"canonical FR10 model has nq={base_model.nq}, nv={base_model.nv}")
        actual_joint_names = [
            mujoco.mj_id2name(base_model, mujoco.mjtObj.mjOBJ_JOINT, index)
            for index in range(base_model.njnt)
        ]
        if actual_joint_names != JOINT_NAMES:
            raise RuntimeError(f"unexpected FR10 joint order: {actual_joint_names}")
        base_data = mujoco.MjData(base_model)
        base_data.qpos[:] = HOME
        mujoco.mj_forward(base_model, base_data)
        wrist_id = mujoco.mj_name2id(
            base_model, mujoco.mjtObj.mjOBJ_BODY, "wrist3_link")
        wrist_rotation = np.array(base_data.xmat[wrist_id]).reshape(3, 3)
        wrist_position = np.array(base_data.xpos[wrist_id])
        connector_rotation = wrist_rotation @ self.rotation_wrist_to_tcp
        self.home_tcp_position = wrist_position + wrist_rotation @ self.tcp_in_wrist
        self.home_tcp_rotation = connector_rotation
        self.insertion_axis_world = connector_rotation[:, 2].copy()
        self.contact_tcp_position = (
            self.home_tcp_position +
            self.insertion_axis_world * self.fixture_gap_m)
        self.port_rotation = (
            connector_rotation @ R_PORT_MESH_TO_TCP.T)
        self.port_model_origin = (
            self.contact_tcp_position -
            self.port_rotation @ PORT_TCP_IN_MESH)

        mujoco.mj_saveLastXML(str(base_mjcf_path), base_model)
        tree = ET.parse(base_mjcf_path)
        root = tree.getroot()
        option = root.find("option")
        if option is None:
            option = ET.SubElement(root, "option")
        option.set("timestep", f"{physics_step_s:.12g}")
        option.set("gravity", "0 0 -9.80665")
        option.set("integrator", "implicitfast")

        worldbody = root.find("worldbody")
        if worldbody is None:
            raise RuntimeError("MuJoCo conversion produced no worldbody")
        asset = root.find("asset")
        if asset is None:
            asset = ET.Element("asset")
            root.insert(list(root).index(worldbody), asset)
        ET.SubElement(
            asset, "mesh", {
                "name": "charging_port_mesh",
                "file": str(PORT_MESH),
                "scale": "0.001 0.001 0.001",
            })
        for geom in worldbody.iter("geom"):
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
        wrist = next(
            (body for body in worldbody.iter("body")
             if body.get("name") == "wrist3_link"), None)
        if wrist is None:
            raise RuntimeError("MuJoCo conversion lost wrist3_link")
        ET.SubElement(
            wrist, "geom", {
                "name": "tool_contact",
                "type": "box",
                "pos": _format(wrist_to_connector[:3, 3]),
                "quat": _format(_matrix_to_quaternion(wrist_to_connector[:3, :3])),
                "size": "0.05648205 0.0675 0.0625",
                "contype": "1", "conaffinity": "1",
                "friction": "0.8 0.02 0.002",
                "rgba": "0.15 0.15 0.17 0.35",
            })

        # MuJoCo collision meshes are convexified.  Keep the supplied concave
        # CCS2 port as the exact visual reference and use the bounded compliant
        # plane below for deterministic force/contact physics.
        port_body = ET.SubElement(
            worldbody, "body", {
                "name": "charging_port",
                "pos": _format(self.port_model_origin),
                "quat": _format(_matrix_to_quaternion(self.port_rotation)),
            })
        ET.SubElement(
            port_body, "geom", {
                "name": "charging_port_visual",
                "type": "mesh", "mesh": "charging_port_mesh",
                "pos": _format(PORT_VISUAL_OFFSET),
                "contype": "0", "conaffinity": "0", "group": "2",
                "rgba": "0.08 0.08 0.10 1",
            })

        plate_half_thickness = 0.005
        plate_center = (
            self.contact_tcp_position +
            self.insertion_axis_world * plate_half_thickness)
        plate_body = ET.SubElement(
            worldbody, "body", {
                "name": "contact_compliance_body",
                "pos": _format(plate_center),
                "quat": _format(_matrix_to_quaternion(connector_rotation)),
                # Keep the spring zero at the CAD mating plane.  Without
                # gravity compensation this 2 kg test body sags 9.8 mm at
                # 2000 N/m before the connector reaches it.
                "gravcomp": "1",
            })
        ET.SubElement(
            plate_body, "joint", {
                "name": "contact_compliance",
                "type": "slide", "axis": "0 0 1",
                "limited": "true", "range": "0 0.02",
                # 5 mm fixture displacement represents a 10 N axial load.
                "stiffness": "2000", "damping": "80",
            })
        ET.SubElement(
            plate_body, "inertial", {
                "pos": "0 0 0", "mass": "2",
                "diaginertia": "0.01 0.01 0.01",
            })
        ET.SubElement(
            plate_body, "geom", {
                "name": "contact_plate", "type": "box",
                "size": f"0.15 0.15 {plate_half_thickness:.12g}",
                "contype": "1", "conaffinity": "1",
                "friction": "0.8 0.02 0.002", "solref": "0.015 1",
                "rgba": "0.25 0.45 0.75 1",
            })
        # Model a stiff joint-position backend.  MuJoCo integrates the
        # actuator damping implicitly; implementing a high-gain PD as explicit
        # qfrc_applied destabilizes the small wrist inertia at a 1 ms step.
        kp,kd = simulation_pd_gains()
        torque_limit = np.array([150.0, 150.0, 150.0, 28.0, 28.0, 28.0])
        actuators = ET.SubElement(root, "actuator")
        for index, name in enumerate(JOINT_NAMES):
            ET.SubElement(actuators, "position", {
                "name": name + "_position", "joint": name,
                "kp": f"{kp[index]:.12g}", "kv": f"{kd[index]:.12g}",
                "ctrllimited": "true",
                "ctrlrange": _format(base_model.jnt_range[index]),
                "forcelimited": "true",
                "forcerange": _format(np.array(
                    [-torque_limit[index], torque_limit[index]])),
            })
        tree.write(model_path, encoding="unicode")

        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.data.qpos[:6] = HOME
        mujoco.mj_forward(self.model, self.data)
        self.wrist_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "wrist3_link")
        self.tool_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "tool_contact")
        self.plate_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "contact_plate")
        self.port_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "charging_port")
        self.port_visual_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "charging_port_visual")
        if min(self.wrist_body_id, self.tool_geom_id, self.plate_geom_id,
               self.port_body_id, self.port_visual_geom_id) < 0:
            raise RuntimeError("MuJoCo force fixture identifiers are missing")
        self.command = HOME.copy()
        self.data.ctrl[:] = self.command
        self.kp, self.kd, self.torque_limit = kp, kd, torque_limit
        self.last_wrench = np.zeros(6)
        self.tool_geom_ids = {self.tool_geom_id}
        self.fixture_geom_ids = {self.plate_geom_id}

    def set_command(self, command: np.ndarray) -> None:
        command = np.asarray(command, dtype=np.float64)
        if command.shape != (6,) or not np.all(np.isfinite(command)):
            raise ValueError("joint command must contain six finite radians")
        self.command = np.clip(command, self.model.jnt_range[:6, 0],
                               self.model.jnt_range[:6, 1])
        self.data.ctrl[:] = self.command

    def observe_configuration(self, measured_joints: np.ndarray) -> np.ndarray:
        """Quasi-static CAD contact proxy at measured SDK joints, no integration.

        Docker has no physical FT sensor. This private model observes geometry;
        it does not simulate a second servo following an already measured robot.
        Its zero velocity is a quasi-static approximation, never JointState data.
        """
        q = np.asarray(measured_joints, dtype=np.float64)
        if not hasattr(self, 'matched_station') or self.model.nq != 6:
            raise ValueError('Native CAD observer requires the matched six-DOF model')
        if q.shape != (6,) or not np.isfinite(q).all() or \
                np.any(q < self.model.jnt_range[:6, 0]) or np.any(q > self.model.jnt_range[:6, 1]):
            raise ValueError('Native CAD observation requires six finite in-range measured joints')
        self.data.qpos[:] = q
        self.data.qvel[:] = 0.
        self.command = q.copy(); self.data.ctrl[:] = q
        mujoco.mj_step1(self.model, self.data)
        self.data.qfrc_applied[:] = self.data.qfrc_bias
        # Position/velocity stages above are already current. Compute contact
        # constraint forces once, without step2/Euler or advancing model time.
        mujoco.mj_forwardSkip(self.model, self.data, mujoco.mjtStage.mjSTAGE_VEL, 0)
        self.last_wrench = self.contact_wrench_tcp()
        return self.last_wrench.copy()

    def step(self, duration_s: float = 0.008) -> np.ndarray:
        substeps = max(1, int(round(duration_s / self.physics_step_s)))
        for _ in range(substeps):
            # The documented split step exposes qfrc_bias before control,
            # without evaluating collision/constraint dynamics twice.
            # Canonical models use implicitfast (a single-step integrator).
            mujoco.mj_step1(self.model, self.data)
            self.data.qfrc_applied[:] = 0.0
            self.data.qfrc_applied[:6] = self.data.qfrc_bias[:6]
            mujoco.mj_step2(self.model, self.data)
        # Body-pose evidence describes the newly integrated qpos, while
        # contact forces remain the physical last-substep constraint result.
        mujoco.mj_kinematics(self.model, self.data)
        self.last_wrench = self.contact_wrench_tcp()
        return self.last_wrench.copy()

    def tcp_pose(self) -> tuple[np.ndarray, np.ndarray]:
        wrist_rotation = np.array(
            self.data.xmat[self.wrist_body_id]).reshape(3, 3)
        wrist_position = np.array(self.data.xpos[self.wrist_body_id])
        position = wrist_position + wrist_rotation @ self.tcp_in_wrist
        rotation = wrist_rotation @ self.rotation_wrist_to_tcp
        return position, rotation

    def contact_wrench_tcp(self) -> np.ndarray:
        if self.data.ncon == 0:
            return np.zeros(6)
        position, rotation = self.tcp_pose()
        contacts = self.data.contact
        tool = np.isin(contacts.geom, list(self.tool_geom_ids))
        fixture = (np.isin(contacts.geom, list(self.fixture_geom_ids)) |
                   np.isin(contacts.flex, list(getattr(self, 'fixture_flex_ids', set()))))
        indices = np.flatnonzero((tool[:, 0] & fixture[:, 1]) |
                                 (tool[:, 1] & fixture[:, 0]))
        if not len(indices):
            return np.zeros(6)
        local_wrench = np.empty((len(indices), 6))
        for row, contact_index in enumerate(indices):
            mujoco.mj_contactForce(self.model, self.data, int(contact_index), local_wrench[row])
        # Batch only the wrench transformations. Every original contact still
        # contributes its solved force, free moment and lever arm at the TCP.
        frames = contacts.frame[indices].reshape(-1, 3, 3)
        direction = np.where(tool[indices, 0], 1., -1.)[:, None]
        applied = direction * np.einsum('nji,nj->ni', frames, local_wrench[:, :3])
        moment = (np.cross(contacts.pos[indices] - position, applied) +
                  direction * np.einsum('nji,nj->ni', frames, local_wrench[:, 3:]))
        return np.r_[rotation.T @ applied.sum(axis=0), rotation.T @ moment.sum(axis=0)]

    def contact_names(self, contact):
        return [self.model.geom(int(geom)).name if geom>=0 else
                mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_FLEX,int(flex))
                for geom,flex in zip(contact.geom,contact.flex)]

    def cartesian_push_command(self, distance_m: float) -> np.ndarray:
        """Linearized joint target preserving orientation at the home TCP."""
        self.data.qpos[:] = 0.0
        self.data.qpos[:6] = HOME
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        point, _ = self.tcp_pose()
        jacobian_position = np.zeros((3, self.model.nv))
        jacobian_rotation = np.zeros((3, self.model.nv))
        mujoco.mj_jac(
            self.model, self.data, jacobian_position, jacobian_rotation,
            point, self.wrist_body_id)
        jacobian = np.vstack((jacobian_position, jacobian_rotation))
        displacement = np.zeros(6)
        displacement[:3] = self.insertion_axis_world * distance_m
        delta = np.linalg.pinv(jacobian, rcond=1e-6) @ displacement
        # The seventh generalized velocity belongs to the compliant fixture,
        # not the arm command interface.
        return HOME + delta[:6]


def self_test() -> int:
    plant = Fr10MujocoPlant()
    for _ in range(250):
        plant.step()
    hold_error = float(np.max(np.abs(plant.data.qpos[:6] - HOME)))
    push = plant.cartesian_push_command(0.008)
    plant.set_command(push)
    peak_force = 0.0
    final_force = 0.0
    for _ in range(500):
        wrench = plant.step()
        if not np.all(np.isfinite(wrench)):
            print("MUJOCO_SELF_TEST FAIL non-finite wrench")
            return 1
        peak_force = max(peak_force, abs(float(wrench[2])))
        final_force = float(wrench[2])
    tracking_error = float(np.max(np.abs(plant.data.qpos[:6] - plant.command)))
    passed = hold_error < 0.02 and tracking_error < 0.05 and peak_force > 1.0
    print(
        "MUJOCO_SELF_TEST",
        "PASS" if passed else "FAIL",
        f"nq={plant.model.nq}", f"nv={plant.model.nv}",
        f"hold_error_rad={hold_error:.9f}",
        f"tracking_error_rad={tracking_error:.9f}",
        f"peak_axis_force_n={peak_force:.6f}",
        f"final_axis_force_n={final_force:.6f}",
        f"contacts={plant.data.ncon}",
    )
    return 0 if passed else 1


def run_ros(args: argparse.Namespace) -> int:
    import rclpy
    from geometry_msgs.msg import PoseStamped, WrenchStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Float64MultiArray, String
    from gazebo_msgs.msg import ContactsState, ContactState

    class Bridge(Node):
        def __init__(self) -> None:
            super().__init__("fr10_mujoco_bridge")
            self.plant = Fr10MujocoPlant(args.fixture_gap, args.physics_step,
                getattr(args,'station_file',None))
            if getattr(args, 'evidence_directory', None) is not None:
                import hashlib, json, shutil
                args.evidence_directory.mkdir(parents=True, exist_ok=True)
                destination = args.evidence_directory/'physics.xml'
                shutil.copyfile(self.plant.model_path, destination)
                (args.evidence_directory/'physics_manifest.json').write_text(json.dumps({
                    'physics_xml_sha256': hashlib.sha256(destination.read_bytes()).hexdigest(),
                    'socket_surface_manifest': self.plant.surface_manifest}, indent=2)+'\n')
            self.command_sequence = 0
            self.health_sequence = 0
            self.wrench_fault = "normal"
            self.previous_wrench_stamp = None
            self.last_report = time.monotonic()
            self.cycles = 0
            self.observe_native = args.observe_native_joints
            self.native_state = None
            self.native_receipt = 0.
            self.native_stamp_ns = 0
            if self.observe_native:
                from rclpy.qos import QoSProfile, ReliabilityPolicy
                self.native_subscriber = self.create_subscription(JointState, '/joint_states',
                    self.native_joint_callback, QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
            else:
                self.command_subscriber = self.create_subscription(
                    Float64MultiArray, "/fairino10_controller/commands", self.command_callback, 10)
                self.fault_subscriber = self.create_subscription(
                    String, "/fr10_mujoco/wrench_fault", self.fault_callback, 1)
                self.joint_publisher = self.create_publisher(JointState, "/joint_states", qos_profile_sensor_data)
                self.health_publisher = self.create_publisher(Float64MultiArray, "/fairino_driver/health", 1)
                self.clock_publisher = self.create_publisher(Clock, "/clock", 1)
            self.wrench_publisher = self.create_publisher(
                WrenchStamped, "/ft_sensor/wrench", qos_profile_sensor_data)
            self.contact_publisher = self.create_publisher(ContactsState, '/fr10_sim/station_contacts',qos_profile_sensor_data)
            self.contact_pose_publisher = self.create_publisher(
                PoseStamped, "/fr10_mujoco/contact_pose", 1)
            self.actual_pose_publisher = self.create_publisher(
                PoseStamped, "/fr10_mujoco/tcp_pose", qos_profile_sensor_data)
            self.timer = self.create_timer(args.period, self.tick)
            contact = self.plant.contact_tcp_position
            axis = self.plant.insertion_axis_world
            self.get_logger().info(
                "MuJoCo 3.14 adapter ready; canonical FR10 nq=6; "
                f"contact_tcp=[{contact[0]:.7f}, {contact[1]:.7f}, {contact[2]:.7f}] "
                f"axis=[{axis[0]:.7f}, {axis[1]:.7f}, {axis[2]:.7f}]")

        def native_joint_callback(self, message):
            stamp_ns = message.header.stamp.sec*1000000000+message.header.stamp.nanosec
            if stamp_ns <= self.native_stamp_ns or set(message.name) != set(JOINT_NAMES) or len(message.name)!=6:
                return
            if len(message.position)!=6: return
            q = np.asarray([message.position[message.name.index(name)] for name in JOINT_NAMES])
            if (not np.all(np.isfinite(q)) or np.any(q < self.plant.model.jnt_range[:6,0]) or
                    np.any(q > self.plant.model.jnt_range[:6,1])): return
            self.native_state = (q,message.header.stamp)
            self.native_stamp_ns = stamp_ns; self.native_receipt = time.monotonic()

        def command_callback(self, message: Float64MultiArray) -> None:
            if len(message.data) != 6 or not all(
                    math.isfinite(value) for value in message.data):
                self.get_logger().error("Rejected malformed joint command")
                return
            try:
                self.plant.set_command(np.array(message.data))
                self.command_sequence += 1
            except ValueError as error:
                self.get_logger().error(str(error))

        def fault_callback(self, message: String) -> None:
            if message.data in {
                    "normal", "dropout", "duplicate", "nan", "overload",
                    "lost_contact", "torque", "pause", "clock_jump"}:
                if message.data == 'clock_jump':
                    self.plant.data.time = max(0.01,self.plant.data.time-1.0)
                    self.wrench_fault = 'normal'
                    return
                self.wrench_fault = message.data

        def tick(self) -> None:
            if self.wrench_fault == 'pause': return
            if self.observe_native:
                if self.native_state is None or time.monotonic()-self.native_receipt > .050: return
                # Private quasi-static CAD observation at measured SDK joints.
                # Never publish robot commands, JointState, clock or health.
                q,stamp = self.native_state
                wrench = self.plant.observe_configuration(q)
            else:
                wrench = self.plant.step(args.period)
            if self.observe_native:
                if time.monotonic()-self.native_receipt > .050: return
            else:
                # Physics time accumulates binary doubles. Round to ROS integer
                # nanoseconds so nominal 8 ms samples never alternate by 1 ns.
                stamp = rclpy.time.Time(nanoseconds=round(self.plant.data.time*1e9)).to_msg()
                clock = Clock(); clock.clock = stamp; self.clock_publisher.publish(clock)
                joint_state = JointState(); joint_state.header.stamp = stamp
                joint_state.name = JOINT_NAMES
                joint_state.position = self.plant.data.qpos[:6].tolist()
                joint_state.velocity = self.plant.data.qvel[:6].tolist()
                joint_state.effort = (self.plant.data.qfrc_applied[:6]+self.plant.data.qfrc_actuator[:6]).tolist()
                self.joint_publisher.publish(joint_state)

            wrench_message = WrenchStamped()
            wrench_message.header.stamp = stamp
            # contact_wrench_tcp already shifts contact torques to TCP.
            wrench_message.header.frame_id = "charging_tcp"
            wrench_message.wrench.force.x = float(wrench[0])
            wrench_message.wrench.force.y = float(wrench[1])
            wrench_message.wrench.force.z = float(wrench[2])
            wrench_message.wrench.torque.x = float(wrench[3])
            wrench_message.wrench.torque.y = float(wrench[4])
            wrench_message.wrench.torque.z = float(wrench[5])
            if self.wrench_fault == "duplicate" and self.previous_wrench_stamp:
                wrench_message.header.stamp = self.previous_wrench_stamp
            elif self.wrench_fault == "nan":
                wrench_message.wrench.force.z = float("nan")
            elif self.wrench_fault == "overload":
                wrench_message.wrench.force.z = 1000.0
            elif self.wrench_fault == "torque":
                wrench_message.wrench.torque.x = 1000.0
            elif self.wrench_fault == "lost_contact":
                wrench_message.wrench.force.x = 0.0
                wrench_message.wrench.force.y = 0.0
                wrench_message.wrench.force.z = 0.0
            if self.wrench_fault != "dropout":
                self.wrench_publisher.publish(wrench_message)
            self.previous_wrench_stamp = wrench_message.header.stamp
            if hasattr(self.plant,'matched_station'):
                contacts = ContactsState(); contacts.header.stamp = stamp
                for contact in self.plant.data.contact[:self.plant.data.ncon]:
                    state = ContactState()
                    state.collision1_name,state.collision2_name = self.plant.contact_names(contact)
                    state.depths = [max(0.,-float(contact.dist))]
                    contacts.states.append(state)
                self.contact_publisher.publish(contacts)

            self.health_sequence += 1
            health = Float64MultiArray()
            health.data = [0.0] * 20
            health.data[0] = 2.0
            health.data[1] = float(self.health_sequence)
            health.data[2] = float(self.health_sequence)
            health.data[3] = self.get_clock().now().nanoseconds * 1e-9
            health.data[6] = 1.0
            health.data[7] = 0.0
            health.data[16] = float(self.command_sequence)
            health.data[17] = float(self.command_sequence)
            health.data[18] = args.period
            if not self.observe_native: self.health_publisher.publish(health)

            contact_pose = PoseStamped()
            contact_pose.header.stamp = stamp
            contact_pose.header.frame_id = "fr10_base_link"
            contact_pose.pose.position.x = float(
                self.plant.contact_tcp_position[0])
            contact_pose.pose.position.y = float(
                self.plant.contact_tcp_position[1])
            contact_pose.pose.position.z = float(
                self.plant.contact_tcp_position[2])
            quaternion = _matrix_to_quaternion(self.plant.home_tcp_rotation)
            contact_pose.pose.orientation.w = float(quaternion[0])
            contact_pose.pose.orientation.x = float(quaternion[1])
            contact_pose.pose.orientation.y = float(quaternion[2])
            contact_pose.pose.orientation.z = float(quaternion[3])
            self.contact_pose_publisher.publish(contact_pose)

            actual_position, actual_rotation = self.plant.tcp_pose()
            actual_pose = PoseStamped()
            actual_pose.header = contact_pose.header
            actual_pose.pose.position.x = float(actual_position[0])
            actual_pose.pose.position.y = float(actual_position[1])
            actual_pose.pose.position.z = float(actual_position[2])
            actual_quaternion = _matrix_to_quaternion(actual_rotation)
            actual_pose.pose.orientation.w = float(actual_quaternion[0])
            actual_pose.pose.orientation.x = float(actual_quaternion[1])
            actual_pose.pose.orientation.y = float(actual_quaternion[2])
            actual_pose.pose.orientation.z = float(actual_quaternion[3])
            self.actual_pose_publisher.publish(actual_pose)

            self.cycles += 1
            now = time.monotonic()
            if now - self.last_report >= 1.0:
                elapsed = now - self.last_report
                self.get_logger().info(
                    f"rate={self.cycles / elapsed:.2f}Hz "
                    f"fz={wrench[2]:.3f}N contacts={self.plant.data.ncon} "
                    f"commands={self.command_sequence}")
                self.cycles = 0
                self.last_report = now

    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--fixture-gap", type=float, default=0.005)
    parser.add_argument("--physics-step", type=float, default=0.001)
    parser.add_argument("--period", type=float, default=0.008)
    parser.add_argument("--station-file", type=Path)
    parser.add_argument("--evidence-directory", type=Path)
    parser.add_argument('--observe-native-joints', action='store_true',
        help='Virtual contact observer only; never publishes joints, health, clock or actuator commands')
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    return run_ros(args)


if __name__ == "__main__":
    raise SystemExit(main())
