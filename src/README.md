# VinUni charging robot — ROS 2 Humble

Hướng dẫn đầy đủ các lệnh chạy: [README_COMMANDS.md](vehicle_bringup/README_COMMANDS.md).

Điểm khởi động vận hành duy nhất là `vehicle_bringup/bringup.launch.py`.
Môi trường: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11.

## Build và khởi động

Từ thư mục gốc workspace (thư mục chứa `src`):

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --packages-up-to vehicle_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch vehicle_bringup bringup.launch.py
```

Nếu workspace đang dùng `ros2_current` với các bản build tách biệt, dùng script
bảo trì `bash src/vehicle_bringup/scripts/build_workspace.sh`; script build/test
một bản mới rồi mới đổi liên kết runtime. Không build đè thư viện đang được
Gazebo sử dụng. Cài dependency bằng rosdep hoặc script bảo trì
`src/vehicle_bringup/scripts/install_workspace_dependencies.sh`.
Các script bảo trì không phải điểm khởi động robot.

## Lệnh vận hành

```bash
# Xe và tay FR10 (mặc định); navigation chưa bật
ros2 launch vehicle_bringup bringup.launch.py
# Chỉ xe, quét bản đồ bằng SLAM Toolbox
ros2 launch vehicle_bringup bringup.launch.py use_arm:=false navigation_mode:=mapping
# Lưu bản đồ từ terminal thứ hai đã source workspace
ros2 launch vehicle_bringup save_map.launch.py map_name:=$HOME/maps/site
# Dừng mapping, khởi động AMCL và Nav2 trên bản đồ vừa lưu
ros2 launch vehicle_bringup bringup.launch.py use_arm:=false navigation_mode:=navigation map:=$HOME/maps/site.yaml
# Chỉ định vị AMCL, không bật planner/controller Nav2
ros2 launch vehicle_bringup bringup.launch.py use_arm:=false navigation_mode:=localization map:=$HOME/maps/site.yaml
# Chỉ tay FR10
ros2 launch vehicle_bringup bringup.launch.py use_vehicle:=false
# Xem toàn bộ tham số
ros2 launch vehicle_bringup bringup.launch.py --show-args
```

Thêm `gui:=false rviz:=false` để chạy headless. AMCL cần **2D Pose Estimate**
trong RViz trước khi gửi **Nav2 Goal**. Mapping không tự điều khiển xe đi khám
phá; có thể gửi lệnh Twist vào `/cmd_vel`. Không chạy teleop và navigation goal
đồng thời. SLAM và AMCL là hai chế độ loại trừ nhau.

Các node Python trong bringup/swerve dùng cùng trình thông dịch với `ros2 launch`,
để tránh chọn nhầm Python của Conda. Runtime trên máy đã được cập nhật bản sửa này;
xem [kiểm tra với Conda](docs/command_simulation_validation.md#kiểm-tra-bổ-sung-với-conda).

Tham số bringup: `use_arm`, `use_vehicle`, `use_gazebo`, `use_detection`,
`navigation_mode`, `map`, `nav2_params_file`, `slam_params_file`, `scan_topic`,
`odom_topic`, `cmd_vel_topic`, `autostart`, `use_respawn`, `log_level`,
`navigation_startup_timeout`, `world`, `gui`, `rviz`, `rviz_config_file`,
`gazebo_port`, cùng các tham số force/station hiện có. Các alias cũ
`arm`, `vehicle`, `mode` được giữ để không làm hỏng lệnh đã dùng.

## Bố cục và trách nhiệm

| Thư mục/package | Trách nhiệm |
|---|---|
| `vehicle_bringup` | Ghép hệ thống, launch, gate khởi động, cấu hình mô phỏng, kiểm thử tích hợp |
| `vehicle_navigation` | AMCL, SLAM, Nav2, map và RViz navigation |
| `vehicle_description` | URDF/Xacro, mesh, sensor, world, cấu hình hình học chuẩn |
| `swerve_vehicle_control` | Controller swerve, odometry, chuẩn hóa lệnh vận tốc |
| `fr10_arm_control/` | Các package motion, force, interface và MoveIt của FR10 |
| `charger_detection/` | Các package nhận diện cổng sạc và message |
| `hardware_interface/` | Driver FAIRINO, backend mô phỏng, PLC commissioning |
| `docs/` | Đặc tả và dữ liệu tham chiếu/lịch sử |

Package và thư mục lá có cùng tên. Launch dùng hậu tố `.launch.py`; cấu hình,
resource dùng tên `snake_case`. File mô tả gốc là
`vehicle_description/urdf/vehicle.urdf.xacro`. Tên joint/frame và interface C++
được giữ để tương thích controller và model hiện tại. Mã vendor được giữ theo
upstream. Không có thư mục `tools` ở gốc workspace.

Các chương trình kiểm thử liên hệ cả xe/tay/Gazebo đã được chuyển vào
`vehicle_bringup/test/integration`; chúng không tham gia khởi động vận hành.
Bản cấu hình nghiên cứu cũ, file ChargingBot nhập vào và backup chỉnh sửa nằm
trong `docs/archive`, không được cài vào runtime.

## Hợp đồng ROS và kiểm tra

TF: `map → odom → base_footprint → base_link → laser_frame`.
SLAM hoặc AMCL sở hữu `map → odom`; controller swerve sở hữu
`odom → base_footprint`; một robot_state_publisher phát TF của model.

Luồng vận tốc: Nav2 `/navigation/cmd_vel` (Twist) → velocity smoother →
`/cmd_vel` (Twist) → normalizer → `/cmd_vel_nav` (TwistStamped) → swerve.
Lidar: `/scan`; odometry: `/odom`. Navigation chờ clock, scan, odometry, TF và
controller active; không khởi động bằng delay cố định. Tất cả chạy simulation
clock trong Gazebo.

```bash
colcon test --packages-select vehicle_navigation vehicle_bringup vehicle_description swerve_vehicle_control
colcon test-result --verbose
```

Đợt tích hợp này dành cho Gazebo. Xe thật vẫn bị chặn do PLC chưa đủ feedback
và watchdog. Không thay đổi thuật toán motion FR10 hay tự enable phần cứng.
Xem [hướng dẫn navigation](vehicle_navigation/README.md) và
[hướng dẫn bringup/force](vehicle_bringup/README.md).
