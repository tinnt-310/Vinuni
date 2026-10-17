# frcobot_ros2
This workspace has been simplified for FR10 usage. The active robot configuration is the FR10 package set under fairino_description, fairino10_v6_moveit2_config and the fairino_hardware driver.

Kept for FR10:
- fairino_description
- fairino10_v6_moveit2_config
- fairino_hardware_v3_9_3
- fairino_msgs

Removed from the active FR10 setup:
- other robot-specific MoveIt packages such as fairino16_v6_moveit2_config, fairino20_v6_moveit2_config, fairino30_v6_moveit2_config, fairino3_v6_moveit2_config, fairino3mt_v6_moveit2_config and fairino5_v6_moveit2_config

If you need a different robot model later, restore the corresponding package from the original repository snapshot.

Version history:
2023.7.18 V1.0
2026.8.10 FR10-focused cleanup
