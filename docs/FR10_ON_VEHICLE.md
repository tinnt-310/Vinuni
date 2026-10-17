# FR10 trên xe Swerve

> Bố trí camera và các bảng gá bên dưới là lịch sử tích hợp. Bố trí hiện tại
> căn D435 ở tâm mặt bích và cùng clocking Tool 1; xem
> [camera FR10 hiện tại](fr10_camera_mount.md) và mô hình `vehicle_description`.

Launch chính: `ros2 launch thanxe_new_description complete.launch.py`.
Mô hình ghép dùng FR10 v6 từ `src/fr10/frcobot_ros2/fairino_description`.
Giữ nguyên XML sáu joint `j1`–`j6`, axis, origin, giới hạn, động học và
inertia của FR10. Chỉ đổi tên link gốc tay thành `fr10_base_link` để tránh
trùng `base_link` của xe. Tay gắn vào vị trí gá UR10 trước đây.
Một `gazebo_ros2_control/GazeboSystem` phục vụ 14 joint của xe và tay;
controller tay dùng position command và position/velocity state.

## Nguồn model và phép đo

- FR10: `fairino_description/urdf/fairino10_v6.urdf`; 7 mesh STL gốc,
  giữ đúng đơn vị mét và tỷ lệ 1.
- CCS2: https://github.com/AdityaGuhagarkar/JLR_Robotic_Charging,
  commit `e7ec8f7d88b3bff4350b94987f0cb4b0fd5e7b57`,
  `CAD/Main Model/Team53_CAD.STEP`, chi tiết `charger1 v1.step`.
  Tách bằng STEPCAFControl, không lấy cả cánh tay trong assembly.
  Hộp bao CAD (mm):
  min=(-26.490305838,-70.798609637,-84.783764979),
  max=(86.473793441,64.201390563,40.216235221).
  Tâm hộp bao=(29.991743802,-3.298609537,-22.283764879).
  Mesh xuất dịch gốc về tâm này, nhân 0.001 từ mm sang m;
  không xoay, mặt các lỗ tiếp xúc ở phía +Z của CAD.
  Kích thước: 112.964099279 × 135 × 125 mm.
  File dùng: `meshes/jlr_ccs2_centered.stl`.
- D435: macro `sensor_d435` và `meshes/d435.dae` chính thức từ
  https://github.com/realsenseai/realsense-ros/tree/ros2-master/realsense2_description.
  Dùng package `realsense2_description` đã cài ở `/opt/ros/humble`.
  SHA256 d435.dae:
  `42f3b66f47a1f8f425a2e4dc07c1d9c283183167d8441f520a15623d98f9bf78`.
  Xacro: cao .025 m, rộng .090 m, sâu .02505 m.
  Mesh thực trong camera_link: X=[-.0207547,.0043],
  Y=[-.0624142,.0275001], Z=[-.0125,.0125] m.
  Offset lắp tính từ mesh; không lấy camera_link làm tâm thân camera.

## Các fixed joint mới (m, rad)

| Parent → child | xyz | rpy |
|---|---|---|
| base_link → fr10_base_link | .5 0 1 | 0 0 pi |
| wrist3_link → plate_frame | 0 0 .106 | 0 0 0 |
| plate_frame → charging_connector_link | 0 .09 .09 | 0 0 pi |
| charging_connector_link → Charge_link | 0 0 0 | 0 0 0 |
| charging_connector_link → charging_tcp | 0 0 .0625 | 0 0 0 |
| plate_frame → d435_mount_frame | 0 -.0125 .030 | 0 0 0 |
| d435_mount_frame → camera_bottom_screw_frame | -.00004295 -.0125 .0101547 | -pi/2 -pi/2 0 |
| camera_bottom_screw_frame → camera_link (macro chính thức) | .0106 .0175 .0125 | 0 0 0 |
| camera_link → camera_depth_frame / camera_infra1_frame | 0 0 0 | 0 0 0 |
| camera_link → camera_color_frame | 0 .015 0 | 0 0 0 |
| camera_link → camera_infra2_frame | 0 -.050 0 | 0 0 0 |
| mỗi camera_*_frame → camera_*_optical_frame | 0 0 0 | -pi/2 0 -pi/2 |
| camera_depth_optical_frame → arm_camera_link_optical | 0 0 0 | 0 0 0 |

Kết quả tổng hợp plate_frame → camera_link:
xyz=(.01745705,-.0125,.0507547), rpy=(-pi/2,-pi/2,0).
Camera X (forward) → plate Z; camera Y → plate X; camera Z → plate Y.
Tâm thân camera thực có X=0; gốc camera_link lệch X là đúng model chính thức.
Biên visual trong plate: X≈[-.04495715,.04495715], Y=[-.025,0],
Z=[.030,.0550547]. Mặt sau cách plate 30 mm, mép trên ngang Y=0.

## Các giả định và phạm vi

- “Tâm đầu sạc” được hiểu là tâm hộp bao CAD, không phải trọng tâm hay tâm
  từng tiếp điểm. `charging_tcp` ở tâm mặt trước hộp bao; chưa phải TCP được
  hiệu chuẩn cho ổ cắm thực.
- Khối lượng đầu sạc giữ 3 kg từ mô hình trước; inertia tính theo hộp bao.
  Đây là giả định mô phỏng, không phải số liệu vật liệu từ STEP.
- Collision đầu sạc là hộp bao bảo thủ (không mô phỏng từng lỗ/chân tiếp xúc).
  Biên thấp nhất của đầu sạc trong plate: Y=.0225, Z=.0275 m;
  camera ở Y≤0 và Z≥.030, nên không cắt plate hoặc nhau.
- Giữ collision/inertia của D435 chính thức. Upstream ghi rõ inertia chỉ
  là xấp xỉ. Plugin RGB/depth giữ FOV .8 rad, 1280×720, 10 Hz của camera
  cổ tay trước; đây không phải mô phỏng đầy đủ nhiễu/hiệu chuẩn phần cứng D435.
- Các frame đều là hệ tọa độ cục bộ; +Y plate không luôn hướng lên theo world.
- Cảm biến thân xe và lidar giữ nguyên. Đã thêm tấm gá, hai trụ camera,
  hai trụ đầu sạc, tấm lưng và hai má ôm đầu sạc. Giá đỡ là thiết kế đề xuất
  cho mô phỏng bằng box, có collision và inertia nhôm (2700 kg/m³),
  chưa phải bản vẽ gia công hoặc CAD gá chính hãng.
- Chỉ tích hợp tay FR10 vào xe. Launch FR10 độc lập trong `src/fr10` không đổi.
- `donghocnghich.cpp` và `solvepnp.cpp` vẫn chứa động học/offset UR10 cũ;
  không được launch xe hiện tại gọi. Không dùng chúng để điều khiển FR10.
  Điều khiển đã kiểm tra qua action `joint_trajectory_controller/follow_joint_trajectory`.
- World cũ vẫn thiếu một số mesh hàng rào/thùng rác bên ngoài; đây là hạn chế
  đã có, độc lập với FR10, CCS2 và D435.

## File đã thêm/sửa

### Sửa hướng sạc và bổ sung giá đỡ

Xoay đầu sạc 180° quanh Z plate: đổi trên–dưới của mặt tiếp điểm,
giữ hướng cắm +Z và tâm (0, .09, .09). Camera giữ nguyên vị trí/hướng.
Các giá đỡ dưới đây gắn fixed vào plate, rpy=(0,0,0); đơn vị mét:

| Link | xyz | Kích thước box X Y Z |
|---|---|---|
| wrist_mount_backplate | 0 .055 -.004 | .12 .19 .008 |
| camera_support_left/right | ±.025 -.0125 .014 | .008 .012 .028 |
| camera_rear_mount | 0 -.0125 .030 | .07 .012 .004 |
| charger_support_left/right | ±.045 .09 .015 | .012 .02 .03 |
| charger_rear_mount | 0 .09 .033 | .12 .10 .006 |
| charger_cradle_left/right | ±.056 .09 .063 | .008 .10 .06 |

Tấm gá kết thúc tại Z=0; các trụ đi từ tấm gá đến tấm lưng camera/đầu sạc.
Hai má ôm kéo tới Z=.093, dừng trước mặt cắm Z=.1525.
Các mối ghép có tiếp xúc/chồng nhẹ để không hở visual; không dùng hình học
này để suy ra độ bền hoặc vị trí lỗ bu lông thực.

Trong `src/thietkemoi/thanxe_new_description/`:

- `urdf/complete_visual.xacro`: ghép FR10 và cụm cổ tay.
- `urdf/fr10_arm.urdf`: bản FR10 giữ động học, đổi tên root link.
- `urdf/fr10_wrist.xacro`: plate, connector, D435 và alias frame.
- `urdf/d435_simulation.xacro`: plugin RGB/depth riêng.
- `urdf/ros2_control.xacro`: thêm interface j1–j6 vào hardware mô phỏng.
- `config/combine_controller.yaml`: controller j1–j6, giữ controller xe.
- `launch/complete.launch.py`: đường dẫn package mesh cho Gazebo server/client.
- `package.xml`: dependency fairino_description và realsense2_description.
- `meshes/jlr_ccs2_centered.stl`: mesh xuất từ CAD.

Kiểm tra bổ sung: `tools/check_fr10_model.py`, `tools/check_fr10_live.py`.

## Build và chạy

```bash
cd /home/tin/Vinuni
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch thanxe_new_description complete.launch.py
```

Dừng launch cũ bằng Ctrl+C trước khi mở lại. Không source workspace `/home/tin/fr10`
chồng lên workspace này vì có package trùng tên.

```bash
ros2 run tf2_ros tf2_echo wrist3_link plate_frame
ros2 run tf2_ros tf2_echo plate_frame camera_link
ros2 run tf2_ros tf2_echo plate_frame charging_connector_link
ros2 control list_controllers
ros2 topic hz /arm_camera/arm_camera_sensor/image_raw
```

Để kiểm tra hình học, xuất Xacro thành URDF rồi chạy `tools/check_fr10_model.py`
với Python có numpy, trimesh và pycollada. Script so sánh toàn bộ XML joint FR10,
kiểm tra cây TF/đường dẫn mesh và biên mesh D435 sau biến đổi.
`tools/check_fr10_live.py` chỉ đọc dữ liệu trên domain thử 77 để không nhầm
với phiên ROS khác.

## Kết quả kiểm tra sau khi thêm giá đỡ

- Xacro, check_urdf và kiểm tra mesh/TF: đạt; sáu joint FR10 không đổi.
- Build package thanxe_new_description: đạt (còn cảnh báo setuptools cũ).
- Gazebo riêng, ROS_DOMAIN_ID=77: TF đầu sạc có yaw=pi; RGB và depth
  đều có dữ liệu 1280×720, hướng optical +Z plate giữ nguyên.
- Gửi quỹ đạo cả j1–j6: action trả SUCCEEDED, error_code=0.
- Cảnh báo mesh của một số vật thể world cũ vẫn tồn tại như nêu trên.
