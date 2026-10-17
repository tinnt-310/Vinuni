"""Contracts for the canonical RB Charging V3 vehicle description."""

from pathlib import Path
import math
import unittest
import xml.etree.ElementTree as ET

import xacro
import yaml


DESCRIPTION = Path(__file__).resolve().parents[1]
CONFIG = DESCRIPTION / "config" / "vehicle_geometry.yaml"
MODEL = DESCRIPTION / "urdf" / "vehicle.urdf.xacro"

RB_MESHES = {
    "base_link.stl", "steer_fl_1.stl", "steer_fr_1.stl",
    "steer_rl_1.stl", "steer_rr_1.stl", "wheel_fl_1.stl",
    "wheel_fr_1.stl", "wheel_rl_1.stl", "wheel_rr_1.stl",
}


def expand(enable_vehicle: bool, enable_arm: bool) -> ET.Element:
    return ET.fromstring(xacro.process_file(
        str(MODEL), mappings={
            "enable_vehicle": str(enable_vehicle).lower(),
            "enable_arm": str(enable_arm).lower(),
        }).toxml())


def direct_named(robot: ET.Element, tag: str, name: str) -> ET.Element:
    matches = [item for item in robot.findall(tag) if item.attrib.get("name") == name]
    if len(matches) != 1:
        raise AssertionError(f"expected one {tag} {name}, got {len(matches)}")
    return matches[0]


class RbV3ModelContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with CONFIG.open() as stream:
            cls.geometry = yaml.safe_load(stream)["vehicle"]
        cls.vehicle = expand(True, False)
        cls.arm = expand(False, True)
        cls.combined = expand(True, True)

    def test_all_rb_v3_assets_are_owned_by_vehicle_description(self):
        mesh_dir = DESCRIPTION / "meshes" / "rb_charging_v3"
        self.assertEqual({path.name for path in mesh_dir.glob("*.stl")}, RB_MESHES)
        vehicle_uris = {
            mesh.attrib["filename"] for mesh in self.vehicle.findall(".//mesh")
            if "rb_charging_v3" in mesh.attrib.get("filename", "")}
        self.assertEqual(vehicle_uris, {
            f"package://vehicle_description/meshes/rb_charging_v3/{name}"
            for name in RB_MESHES})
        for robot in (self.vehicle, self.arm, self.combined):
            self.assertNotIn("RB_CHARGING_" + "V3_description", ET.tostring(
                robot, encoding="unicode"))

    def test_geometry_uses_approved_rb_v3_dimensions(self):
        expected = {
            "wheelbase": 1.10, "track_width": 0.58,
            "wheel_radius": 0.214671, "wheel_offset_x": 0.005062,
            "body_length": 2.04292, "body_width": 1.01147,
            "body_max_height": 1.287265, "deck_top_height": 1.240780,
            "arm_mount_x": 0.502310, "arm_mount_z": 1.174780,
            "lidar_x": 0.840050,
        }
        for key, value in expected.items():
            with self.subTest(key=key):
                self.assertTrue(math.isclose(
                    float(self.geometry[key]), value, rel_tol=0.0, abs_tol=1e-9))

    def test_profiles_contain_only_the_requested_major_subsystems(self):
        vehicle_links = {link.attrib["name"] for link in self.vehicle.findall("link")}
        arm_links = {link.attrib["name"] for link in self.arm.findall("link")}
        combined_links = {link.attrib["name"] for link in self.combined.findall("link")}
        self.assertIn("base_link", vehicle_links)
        self.assertNotIn("fr10_base_link", vehicle_links)
        self.assertIn("fr10_base_link", arm_links)
        self.assertNotIn("base_link", arm_links)
        self.assertTrue({"base_link", "fr10_base_link", "charging_tcp"} <= combined_links)

    def test_swerve_joint_frames_follow_the_controller_convention(self):
        positions = ((0.55, 0.29), (0.55, -0.29),
                     (-0.55, 0.29), (-0.55, -0.29))
        for index, (x, y) in enumerate(positions, 1):
            with self.subTest(module=index):
                steer = direct_named(
                    self.vehicle, "joint", f"Revolute{index}_to_base_link")
                self.assertEqual(steer.find("axis").attrib["xyz"], "0 0 1")
                xyz = [float(value) for value in steer.find("origin").attrib["xyz"].split()]
                self.assertEqual(xyz, [x, y, 0.50878])
                axle = direct_named(
                    self.vehicle, "joint", f"Wheel{index}_to_Revolute{index}")
                self.assertEqual(axle.find("axis").attrib["xyz"], "0 1 0")
                axle_xyz = [float(value) for value in axle.find("origin").attrib["xyz"].split()]
                self.assertEqual(axle_xyz, [0.005062, 0.0, -0.295439])
                wheel = direct_named(self.vehicle, "link", f"Wheel{index}_1")
                cylinder = wheel.find("collision/geometry/cylinder")
                self.assertIsNotNone(cylinder)
                self.assertTrue(math.isclose(
                    float(cylinder.attrib["radius"]), 0.214671,
                    rel_tol=0.0, abs_tol=1e-9))

    def test_wheel_contact_parameters_are_symmetric(self):
        contacts = []
        for index in range(1, 5):
            entries = [element for element in self.vehicle.findall("gazebo")
                       if element.attrib.get("reference") == f"Wheel{index}_1"
                       and element.find("kp") is not None]
            self.assertEqual(len(entries), 1)
            contact = entries[0]
            contacts.append(tuple(contact.find(name).text.strip()
                                  for name in ("kp", "kd", "minDepth", "maxVel", "fdir1")))
        self.assertEqual(contacts, [contacts[0]] * 4)
        self.assertEqual(float(self.geometry["wheel_contact_kd"]), 10000.0)
        self.assertEqual(float(contacts[0][1]),
                         float(self.geometry["wheel_contact_kd"]))

        runtime = (DESCRIPTION.parent / "vehicle_bringup" /
                   "config" / "simulation.xacro").read_text()
        self.assertIn("libvinuni_wheel_contact_surface.so", runtime)

    def test_lidar_sensor_is_active_and_uses_gazebo_ros_plugin(self):
        lidar = direct_named(self.vehicle, "link", "lidar_link")
        self.assertIsNone(lidar.find("visual"))
        self.assertIsNone(lidar.find("collision"))
        mount = direct_named(self.vehicle, "joint", "base_to_integrated_lidar")
        self.assertEqual(mount.find("parent").attrib["link"], "base_link")
        xyz = [float(value) for value in mount.find("origin").attrib["xyz"].split()]
        self.assertEqual(xyz[:2], [0.84005, 0.0])
        self.assertTrue(math.isclose(xyz[2], 1.26592,
                                     rel_tol=0.0, abs_tol=1e-9))
        entries = [element for element in self.vehicle.findall("gazebo")
                   if element.attrib.get("reference") == "laser_frame"]
        self.assertEqual(len(entries), 1)
        sensor = entries[0].find("sensor")
        self.assertIsNotNone(sensor)
        self.assertEqual(sensor.attrib.get("type"), "ray")
        self.assertEqual(sensor.findtext("always_on"), "true")
        plugin = sensor.find("plugin")
        self.assertIsNotNone(plugin)
        self.assertEqual(plugin.attrib.get("filename"),
                         "libgazebo_ros_ray_sensor.so")

    def test_body_d455_is_seated_in_front_pocket(self):
        mount = direct_named(self.vehicle, "joint", "base_to_d455_mount")
        self.assertEqual(mount.find("parent").attrib["link"], "base_link")
        xyz = [float(value) for value in mount.find("origin").attrib["xyz"].split()]
        body_center = [xyz[0] - 0.0001, xyz[1], xyz[2]]
        self.assertEqual(body_center, [0.97993, 0.0, 0.48978])
        self.assertGreaterEqual(body_center[0] - 0.013, 0.96693)
        self.assertLessEqual(abs(body_center[1]) + 0.062, 0.06690131)
        self.assertGreaterEqual(body_center[2] - 0.0145, 0.47378)
        self.assertLessEqual(body_center[2] + 0.0145, 0.50578)

    def test_fr10_mount_uses_the_cad_recess(self):
        mount = direct_named(self.combined, "joint", "base_to_robot_base")
        self.assertEqual(mount.find("parent").attrib["link"], "base_link")
        xyz = [float(value) for value in mount.find("origin").attrib["xyz"].split()]
        rpy = [float(value) for value in mount.find("origin").attrib["rpy"].split()]
        self.assertEqual(xyz, [0.50231, 0.0, 1.17478])
        self.assertTrue(math.isclose(rpy[2], math.pi, rel_tol=0.0, abs_tol=1e-12))


if __name__ == "__main__":
    unittest.main()
