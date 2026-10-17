# RB Charging V3 and FR10 description

`vehicle_description` is the canonical owner of the RB Charging V3 body, four swerve modules, FR10, wrist D435, CCS2 tool, body sensors and SRDF. All three simulation profiles expand from `vehicle_description/urdf/vehicle.urdf.xacro`; no runtime mesh URI points to the retired RB V3 package.

## Geometry baseline

FR10 Tool 0 is `plate_frame`, the centre of the flange face. Tool 1 is
`charging_tcp`, the centre of the front mating surface of the charging plug.
The single source is [config/fr10_tool.yaml](config/fr10_tool.yaml):
translation `[55.140, -46.496, 215.297] mm`, fixed XYZ angles
`[0, 0, 136.840] degrees`, relative to Tool 0. Gazebo, RViz/MoveIt, fake/native
hardware descriptions and MuJoCo resolve that same Xacro chain. Connector
visuals, collisions and support clocking follow the datum; changing only a TCP
marker is not sufficient. `legacy` and `matched_socket_v2` are mesh/contact
representations, not separate Tool 1 calibrations. Their local mesh front
offsets may differ, but their flange-to-TCP transform must be identical.

The camera mounting datum is unchanged. The simulated load cell remains a
test-only sensor at the connector origin; this does not calibrate the real
flange-mounted force sensor. Existing simulated masses/inertias and approximate
support geometry are not measured physical payload data. The approximate
1 kg physical payload has not been substituted for them without a CoG measurement.
Old test evidence and files in `docs/archive` describe previous runs and are
not runtime geometry inputs; regenerate evidence after this tool change.

The canonical values are in `vehicle_description/config/vehicle_geometry.yaml`.

- Wheelbase: `1.10 m`; track width: `0.58 m`.
- CAD wheel radius: `0.214671 m`; caster offset: `0.005062 m`.
- CAD bounds: about `2.04292 × 1.01147 m`; maximum CAD height: `1.287265 m`.
- The RB V3 STL already contains the LiDAR. `lidar_link` and `laser_frame` add TF and the Gazebo ray sensor only; they do not add a second visual, mass or sensor housing.
- The body D455 uses the camera pocket/support already present in the RB V3 CAD. Xacro adds the D455 model, TF and depth plugin without a separate L bracket.
- The FR10 mounting pocket measured from the STL is centred at `x=0.502310 m`, `y=0`, with its mounting surface at `z=1.174780 m`. The base is rotated by `π` around Z. Primitive chassis collisions leave this pocket open.

The CAD mass and inertia are the simulation baseline and have not yet been checked against physical measurements of the completed vehicle.

## Build và vận hành

Xem [README workspace](../README.md) để cài dependency, build và chạy.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vehicle_bringup bringup.launch.py use_arm:=false
```

Đầu vào Xacro là `urdf/vehicle.urdf.xacro`; các tài nguyên được tìm bằng
ament package share, không phụ thuộc thư mục làm việc. Các package FR10,
bringup và MoveIt dùng chung model này.

Kiểm thử model: `colcon test --packages-select vehicle_description`.
Kiểm thử tích hợp Gazebo nằm trong `vehicle_bringup/test/integration`.
