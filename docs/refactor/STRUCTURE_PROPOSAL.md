# Bố trí đề xuất — chưa thực hiện di chuyển

Thao tác di chuyển bị automatic approval review từ chối; nguồn hiện tại chưa đổi trong lượt này.

Sáu thư mục cấp cao là nhóm chức năng. Với nhóm chứa nhiều ROS package, không đặt package.xml tại thư mục nhóm: colcon cần khám phá các package con độc lập.

```text
src/
├── vehicle_bringup/                         # package launch tổng
├── vehicle_description/
│   ├── vehicle_description/                 # model xe và tích hợp sensor/tool
│   └── fairino_description/                 # nguồn FR10 vendor duy nhất
├── fr10_arm_control/
│   ├── fr10_arm_control/                    # interface/controller tay
│   ├── fr10_pose_controller/                # motion planning/execution hiện có
│   └── fairino10_v6_moveit2_config/          # MoveIt độc lập, chưa phải MoveIt xe
├── swerve_vehicle_control/                  # package kinematics/odom
├── charger_detection/
│   ├── charger_detection/                  # perception
│   └── ccs2_socket_detector/               # message ABI, phải giữ
└── hardware_interface/                     # thư mục nhóm, không tạo package trùng ROS
    ├── plc_bridge/                         # chuyển src/plc; hiện chưa là ros2_control bridge
    ├── fr10_driver/
    │   ├── fairino_hardware_v3_9_3/
    │   └── fairino_msgs/
    └── config/                             # cấu hình backend và mapping phần cứng
```

## Mô hình

FR10 trên xe đã include trực tiếp fairino_description; fr10_arm.urdf là adapter đổi tên đế, không còn là bản sao toàn bộ tay. Tách Xacro theo cụm cơ khí là cần thiết, không đồng nghĩa description trùng lặp. Đề xuất nhóm urdf thành vehicle/, arm/, sensors/, tool/ và giữ một entrypoint. Các thay đổi đường dẫn cần cập nhật cả include, CMake install, launch, script kiểm tra rồi build sạch ở thư mục riêng để không dùng nhầm install cũ.

## Luồng điều khiển thực tế

Arm → JointTrajectoryController → ros2_control → GazeboSystem hoặc plugin FAIRINO.
Base → swerve kinematics → steering/wheel controllers → GazeboSystem.
Nhánh base REAL chưa hoàn chỉnh: src/plc là chương trình Python; chưa có ros2_control SystemInterface nối wheel/steering joint với PLC. Không được khởi chạy đồng thời hai nguồn điều khiển actuator. Đổi cấu trúc không tự tạo khả năng điều khiển phần cứng.

## Danh sách dư thừa/cần giữ, kèm hành động đề xuất

| Nguồn | Đối chiếu | Hành động chờ duyệt |
|---|---|---|
| src/Universal_Robots_ROS2_Description | Bộ UR, không dùng trong launch xe FR10; còn phụ thuộc từ stack UR | Chuyển nguyên cây sang archive/pre_modular, không xóa |
| src/Universal_Robots_ROS2_Driver | Driver UR độc lập, không phải FAIRINO | Chuyển cùng stack UR sang archive/pre_modular |
| src/Universal_Robots_ROS2_Gazebo_Simulation | Demo UR, không phải simulation xe | Chuyển cùng stack UR sang archive/pre_modular |
| src/thietkemoi | Description/launch cũ, package đã COLCON_IGNORE | Chuyển archive/pre_modular, giữ reference lịch sử |
| src/my_ur_control_cpp | Nguồn hỗn hợp cũ, đã COLCON_IGNORE; có code UR10 chưa chuyển | Chuyển archive/pre_modular, không xóa |
| src/models | Asset cũ đã được copy, đã COLCON_IGNORE | Chuyển archive/pre_modular |
| src/fr10 | Có nguồn vendor đang dùng, không được archive toàn bộ ngay | Chuyển từng package theo cây mới; chỉ archive phần tài liệu còn lại |
| src/ccs2_socket_detector | Định nghĩa message còn được detector sử dụng | Giữ dưới nhóm charger_detection |
| src/plc | Mã PLC có ích, chưa phải ROS bridge | Giữ dưới hardware_interface/plc_bridge |
| Legacy URDF trong vehicle_description | Không nằm trong danh sách install active | Giữ chờ duyệt; không xóa theo tên file |
| build/install/log | Sản phẩm sinh ra, có thể còn package cũ | Giữ; kiểm thử cấu trúc mới bằng build/install riêng |

## Phạm vi phê duyệt cần thiết

Cho phép di chuyển các nguồn theo cây trên và chuyển sáu cây legacy/UR nêu trong bảng sang archive/pre_modular có COLCON_IGNORE; không xóa file. Sau di chuyển phải cập nhật reference, kiểm tra package discovery, build, Xacro hai chế độ tay và launch trước khi bàn giao. Chưa có kết quả build cho cấu trúc đề xuất vì thao tác chưa được phép thực thi.
