"""RViz profiles match the enabled simulation subsystem."""
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


def displays(name):
    data = yaml.safe_load((ROOT / 'rviz' / name).read_text())
    return data['Visualization Manager']


def test_vehicle_profile_has_robot_tf_scan_and_odometry():
    manager = displays('simulation_vehicle.rviz')
    classes = {row['Class'] for row in manager['Displays']}
    assert manager['Global Options']['Fixed Frame'] == 'odom'
    assert {'rviz_default_plugins/RobotModel', 'rviz_default_plugins/TF',
            'rviz_default_plugins/LaserScan',
            'rviz_default_plugins/Odometry'} <= classes


def test_arm_profile_has_no_vehicle_only_topics():
    manager = displays('simulation_arm.rviz')
    classes = {row['Class'] for row in manager['Displays']}
    assert manager['Global Options']['Fixed Frame'] == 'fr10_base_link'
    assert 'rviz_default_plugins/RobotModel' in classes
    assert 'rviz_default_plugins/TF' in classes
    assert 'rviz_default_plugins/LaserScan' not in classes
    assert 'rviz_default_plugins/Odometry' not in classes
