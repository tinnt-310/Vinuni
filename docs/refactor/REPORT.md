# Báo cáo refactor workspace

## Trạng thái

Đã tạo và chuyển quyền sở hữu nguồn hoạt động sang năm package mới; chưa tuyên bố
nghiệm thu toàn bộ simulation có tay vì lỗi ổn định vật lý FR10 trước refactor còn tồn tại.
Không xóa file/folder nào. “Chuyển” ở đây là copy sang nơi sở hữu mới + đổi reference;
nguồn cũ được giữ nguyên để người dùng duyệt sau, không phải rename làm mất đường dẫn cũ.

## Cấu trúc

```
src/
├── vehicle_bringup/          # launch tổng, cấu hình ghép ros2_control, Nav2/maps/RViz
├── vehicle_description/      # chassis/wheels/roof/FR10/sensors, mesh/world/model
├── fr10_arm_control/         # controller tay, interface j1–j6, launch MoveIt độc lập
├── swerve_vehicle_control/   # kinematics/odom/cmd_vel, controller thân và plugin
├── charger_detection/        # CCS2/PnP/depth nodes, launch perception
├── ccs2_socket_detector/     # giữ message ABI, không duplicate detector
├── fr10/                    # giữ vendor FAIRINO/MoveIt và FR10 pose controller
└── Universal_Robots_*/       # giữ hệ UR độc lập, không thuộc launch xe
```

Ba cây nguồn cũ vẫn tồn tại và được đánh dấu COLCON_IGNORE. Không chuyển build/install/log
vào package mới. Do chưa được phép cleanup, install cũ có thể còn expose tên package cũ;
entry point được hỗ trợ sau refactor là `vehicle_bringup`.

## File chuyển quyền sở hữu

Xem [transfers.json](transfers.json): từng đường dẫn cũ → mới, original_retained=true.
Phân loại toàn bộ inventory trước sửa nằm trong [file_classification.json](file_classification.json).

- URDF/Xacro/mesh/world/model → vehicle_description; geometry YAML giữ một nguồn.
- Launch tổng, Nav2, maps và SLAM RViz → vehicle_bringup.
- swerve_drive, swerve_controller_plugin, god_view_2, goal_sender → swerve_vehicle_control.
- CCS2 detector/header, solvepnp, depth_heatmap → charger_detection.
- YAML và Xacro interface j1–j6 → fr10_arm_control; controller thực thi vẫn là
  joint_trajectory_controller hiện có, không viết lại IK hoặc FAIRINO hardware.
- plugin.cpp/donghocnghich.cpp cũ dùng tên joint UR10: giữ ở nguồn legacy, không
  đưa vào FR10 control với tên mới gây hiểu nhầm tương thích.

## Reference đã cập nhật

- package://thanxe_new_description → package://vehicle_description cho nguồn geometry đang dùng.
- Models từ vinuni_gazebo_models → vehicle_description/models.
- Launch tổng include controllers/drive/detection launch của package chức năng.
- Controller YAML ghép từ swerve_vehicle_control + fr10_arm_control theo enable_arm;
  vẫn chỉ có một GazeboSystem và một controller_manager (8 hoặc 14 joint).
- Header detector → charger_detection/ccs2_detector_node.hpp; message namespace
  ccs2_socket_detector giữ nguyên để không đổi topic type.
- Header plugin → swerve_vehicle_control/swerve_controller_plugin.hpp; giữ class ID
  my_ur_control_cpp/SwerveDriveController cho compatibility. Sửa library path trong
  plugin XML từ giá trị cũ không khớp target sang swerve_controller_plugin.
- Install rules/package dependencies tách theo nhiệm vụ; chỉ install URDF vật lý active.
- Nguồn runtime dùng ROS package lookup; XML runtime chuyển URI mesh sang file URI đã
  resolve để Gazebo đọc được. Comment XML được loại khỏi runtime để tránh lỗi parser.
- Geometry .80/.70/.21 vẫn do vehicle_description/config/vehicle_geometry.yaml cấp
  cho Xacro và node swerve; default C++ giữ giống trước, thuật toán không thay đổi.
- Detection opt-in remap topic camera cũ sang D455 hiện tại; mặc định tắt như trước.
- Auto cleanup runtime YAML tắt trong lần này theo quy tắc không xóa của người dùng.

Graph trước/sau: [before_dependencies.json](before_dependencies.json),
[after_dependencies.json](after_dependencies.json). Inventory trước: [before_inventory.json](before_inventory.json).

## Kiểm tra và giới hạn

- Full colcon build: 20 package, không package thất bại.
- Expanded URDF: check_urdf đạt; mọi link/joint và ros2_control interface bằng bản
  trước refactor sau chuẩn hóa package URI ([model_equivalence.txt](model_equivalence.txt)).
- enable_arm=false: spawn xe/trạm thành công; 3 controller active; TF odom→D455 hợp lệ;
  LiDAR, joint_states, odom, D455 RGB/depth có dữ liệu.
- enable_detection=true: CCS2 node chạy và xuất debug_image 640×480 từ D455.
  Chưa xác nhận độ chính xác detector/pose hoặc thành công docking trên dataset.
- enable_arm=true: spawn và 4 controller active; FollowJointTrajectory trả SUCCEEDED.
  Nhưng pose vật lý xe có tay vẫn bị lật như baseline. Không coi action success là
  chứng minh độ ổn định hay nghiệm thu FR10 simulation.
- G2 vẫn là geometry xấp xỉ Ø73×43 mm, chưa tìm được CAD G2 xác thực; giữ đúng baseline.
- MoveIt demo là FR10 độc lập với end-effector riêng cũ, không phải MoveIt cho xe ghép;
  wrapper mới ghi rõ dùng fake hardware. Không tự kết nối/bật phần cứng thật.
- Không đổi cấu hình/material/mass/PID để che lỗi vật lý trong một refactor bảo toàn hành vi.
- Các snapshot URDF cũ còn absolute path nhưng không nằm trong đường chạy/install URDF
  active. Không biến snapshot thành nguồn authoritative.

## Entry point

```bash
cd ~/Vinuni
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch vehicle_bringup simulation.launch.py enable_arm:=false
# FR10 (còn lỗi ổn định vật lý đã nêu): enable_arm:=true
# Perception D455 tùy chọn: enable_detection:=true
```

Gazebo port mặc định 11355; dừng launch trước khi chạy phiên khác cùng cổng/domain.
Không chạy MoveIt demo độc lập và simulation cùng ROS domain vì trùng TF/controller.

## Bước chờ duyệt

Một danh sách tổng hợp duy nhất: [REMOVAL_REVIEW.md](REMOVAL_REVIEW.md).
Không xóa gì trước khi người dùng xác nhận rõ ràng; chưa có mục nào đủ bằng chứng để
khuyến nghị DELETE tự động.
