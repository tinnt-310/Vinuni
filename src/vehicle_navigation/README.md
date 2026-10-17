# Vehicle navigation (ROS 2 Humble)

Hướng dẫn đầy đủ các lệnh chạy: [README_COMMANDS.md](../vehicle_bringup/README_COMMANDS.md).

Start through `ros2 launch vehicle_bringup bringup.launch.py`.

| `navigation_mode` | Started nodes |
|---|---|
| `disabled` (default) | Robot simulation only |
| `mapping` | SLAM Toolbox; map → odom TF |
| `localization` | Map server + AMCL; map → odom TF |
| `navigation` | Map server + AMCL + Nav2 planners/controllers |

Mapping and AMCL never run together. The swerve controller owns odom →
base_footprint; robot_state_publisher owns the sensor transforms. Bringup waits
for active vehicle controllers, advancing clock, scan, odometry and both TF
connections. Navigation is rejected for arm-only, real-vehicle and force-fixture
configurations.

```bash
# Mapping with the simulated vehicle
ros2 launch vehicle_bringup bringup.launch.py use_arm:=false navigation_mode:=mapping
# Save from another sourced terminal; map_name is an output path without extension
ros2 launch vehicle_bringup save_map.launch.py map_name:=$HOME/maps/site
# Stop mapping, then localize and navigate on the saved map
ros2 launch vehicle_bringup bringup.launch.py use_arm:=false navigation_mode:=navigation map:=$HOME/maps/site.yaml
```

Use RViz **2D Pose Estimate** to initialize AMCL, then **Nav2 Goal** to send a
goal. The packaged `maps/vinfast.yaml` is a sample; freshly mapped sites should
use their saved map. Map saving refuses to overwrite an existing YAML or PGM.
Mapping does not automatically explore or move the vehicle. Manual velocity
commands use `/cmd_vel` (`geometry_msgs/msg/Twist`); stop manual commands before
sending a navigation goal.

`config/nav2.yaml` owns AMCL, MPPI Omni, Theta Star, costmaps and velocity
smoothing. `config/slam.yaml` owns SLAM Toolbox. Scan defaults to `/scan`,
odometry to `/odom`, robot frame to `base_footprint`, and simulation time is used
throughout. Nav2 outputs Twist on `/navigation/cmd_vel`, the smoother outputs
Twist on `/cmd_vel`, and the existing normalizer outputs TwistStamped on
`/cmd_vel_nav`. The latter is reserved for the swerve controller.

Parameters exposed by bringup: `navigation_mode`, `map`, `nav2_params_file`,
`slam_params_file`, `scan_topic`, `odom_topic`, `cmd_vel_topic`, `autostart`,
`use_respawn`, `log_level`, `navigation_startup_timeout`, `world`,
`rviz_config_file`, plus the existing arm/vehicle/Gazebo/force switches.
Scan/odom topic overrides select external inputs; the simulation sensors retain
their standard topic names. The launch supports one robot without a namespace.
AMCL starts without a guessed initial pose. No motion goal is sent at startup.
The chassis footprint covers the base; navigation with an extended arm needs a
separately validated swept envelope.

The imported ChargingBot files and original commented research configuration
are retained under `docs/archive/navigation_import`, outside runtime install.
Humble reference: [Nav2 navigation launch](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/navigation_launch.py).

Opt-in end-to-end regression (isolated ROS domain, Gazebo only):

```bash
RUN_GAZEBO_NAVIGATION_TEST=1 python3 -m pytest -s src/vehicle_bringup/test/integration/test_navigation_gazebo.py
```

The test creates a SLAM map, saves it through `save_map.launch.py`, restarts
with AMCL/Nav2, checks lifecycle and TF, and requires a short NavigateToPose
goal to succeed. Map saving exits with an error if no map arrives; choose a
new `map_name` when an output already exists. The launch timeout reports the
missing startup inputs and shuts down the bringup rather than starting a
partially initialized navigation stack.
