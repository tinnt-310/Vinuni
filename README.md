# VinUni charging robot

ROS 2 Humble / Gazebo Classic. Hướng dẫn lệnh chạy: [README_COMMANDS.md](src/vehicle_bringup/README_COMMANDS.md).
Kiến trúc workspace: [src/README.md](src/README.md).

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vehicle_bringup bringup.launch.py
# Quét bản đồ
ros2 launch vehicle_bringup bringup.launch.py use_arm:=false navigation_mode:=mapping
# Định vị và điều hướng
ros2 launch vehicle_bringup bringup.launch.py use_arm:=false navigation_mode:=navigation map:=$HOME/maps/site.yaml
```

Build bản runtime mới: `bash src/vehicle_bringup/scripts/build_workspace.sh`.
Luồng vận hành dùng ros2 launch với tham số trong bringup. Kiểm thử liên hệ xe,
tay và Gazebo nằm tại `src/vehicle_bringup/test/integration`; không còn thư mục
`tools` tại gốc. Chi tiết thay đổi đường dẫn: [file_migration.tsv](src/docs/file_migration.tsv).
