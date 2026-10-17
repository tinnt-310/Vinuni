# VinUni charging robot

Workspace robot sạc điện dùng ROS 2 Humble, Swerve Drive và tay FR10.

## Bắt đầu tại đây

| Công việc | Hướng dẫn |
|---|---|
| Chạy xe thật với PLC FX5U | [Vận hành PLC/Swerve](docs/operations/plc_swerve.md) |
| Mô phỏng Gazebo, SLAM, Nav2 và FR10 | [Các lệnh vận hành](src/vehicle_bringup/README_COMMANDS.md) |
| Theo dõi lực, TCP và thử FR10 trong MuJoCo | [FR10: công cụ và kiểm thử](src/vehicle_bringup/README_COMMANDS.md#10-fr10-công-cụ-và-kiểm-thử) |
| Kiến trúc và các package | [Cấu trúc workspace](src/README.md) |
| Kiểm kê và khôi phục dữ liệu thí nghiệm | [Chính sách dữ liệu](docs/data_inventory.md) |

README trong từng package cung cấp chi tiết riêng cho thành phần đó.

## Mô phỏng nhanh với Gazebo Classic

Chuẩn bị mỗi terminal:

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

## Tài liệu chuyên biệt

- [Kiểm thử FR10 6DOF trong MuJoCo](docs/fr10_6dof_mujoco_test_guide.md).
- [Bố trí đầu sạc CCS2 DC và camera D455](docs/fr10_photo_dc_d455.md).
- [Kiểm tra SDK FR10 độc lập](src/hardware_interface/fr10_sdk_tests/README.md).

Các báo cáo audit/debug được giữ tại vị trí hiện có. Dữ liệu đo không rõ khả
năng tái tạo được giữ nguyên; dữ liệu JSONL cục bộ và kết quả trong
`test_results/` không được thêm vào các commit mới. Lịch sử Git được giữ nguyên.
