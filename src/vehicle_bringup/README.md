# Chạy tay robot và xe

Hướng dẫn đầy đủ các lệnh chạy: [README_COMMANDS.md](README_COMMANDS.md).

Sau khi source ROS 2 và workspace, chỉ cần một launch:

```bash
ros2 launch vehicle_bringup bringup.launch.py use_arm:=true use_vehicle:=true use_gazebo:=true
```

Ba cờ mặc định đều là `true`, nên lệnh ngắn nhất là:

```bash
ros2 launch vehicle_bringup bringup.launch.py
```

- Chỉ tay robot trong Gazebo: thêm `use_vehicle:=false`.
- Chỉ xe trong Gazebo: thêm `use_arm:=false`.
- Không mở cửa sổ khi test: thêm `gui:=false rviz:=false`.

Launch tự mở Gazebo/RViz, spawn model, nạp controller và khởi động pose node
sau khi controller đã active. Không cần chạy tool Python riêng. Tay robot dùng
`MotionExecutor → JointGroupPositionController → ros2_control`; xe dùng một
`SwerveDriveController`. Không khởi động JTC hoặc MoveIt demo song song.
Bringup không tự gửi mục tiêu chuyển động hay insertion; force mặc định tắt.
Nếu terminal đến từ VS Code Snap, launch tự loại môi trường Qt/GTK Snap khỏi
process con để Gazebo/RViz dùng thư viện native; không sửa môi trường shell.

## Thử force insertion vào model cổng sạc

Đây là fixture Gazebo-only, độc lập với xe và không sử dụng native FAIRINO
`FT_Control`. Nó nạp đúng mesh socket chi tiết
`vehicle_description/meshes/cong_sac/meshes/cong_sac.stl`, plugin F/T Gazebo
và controller force chung. Mesh `jlr_ccs2_centered.stl` vẫn là connector gắn
trên cổ tay FR10, không phải socket mục tiêu. Fixture mặc định giữ tọa độ
XY và hướng của pose home, nhưng nằm dưới đầu sạc: mặt sau mesh chạm sàn
ở z=0, mặt miệng cao khoảng 0,0938 m (93,8 mm), đo từ STL đã scale/rotate.
Đầu sạc vẫn khởi tạo ở home; launch không tự di chuyển robot xuống cổng.
Khi bật `use_force`, launch tự nạp clock physics 1000 Hz để executor vẫn
tiến đúng 125 Hz. Không cần thêm tham số clock vào lệnh chạy.

**Giới hạn quan trọng:** profile này chỉ bù lực theo một trục tịnh tiến
(goal mẫu chọn Z trong insertion frame). Nó không tìm lỗ, tự căn X/Y hoặc
tự xoay đầu sạc; runtime từ chối goal chọn nhiều trục cho tới khi có validation
hành lang va chạm tương ứng. Hai CAD hiện tại chưa phải cặp ghép đã kiểm chứng:
tiết diện mũi JLR rộng khoảng 87 mm, trong khi khoảng mở bên trong miệng cổng
rộng khoảng 70 mm, và pattern chân khác nhau. Không thể khắc phục bằng một offset pose.
Tấm xanh proxy không còn được hiển thị; collision/spring vẫn giữ nguyên cho
test pipeline lực, còn mesh cổng vẫn **visual-only**. `SUCCESS` trên fixture
không xác nhận connector đã cắm khớp các lỗ CAD. Không tăng lực, scale CAD
hoặc bật bù ngang để ép test qua.

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vehicle_bringup bringup.launch.py \
  use_arm:=true use_vehicle:=false use_gazebo:=true use_force:=true
```

Terminal 2 (cùng ROS domain; phải source workspace lại vì mỗi terminal có
environment riêng):

```bash
cd ~/Vinuni
source /opt/ros/humble/setup.bash
source install/setup.bash

# kiểm tra action type đã được discover (không bắt buộc)
ros2 interface show fr10_motion_interfaces/action/Insert

ros2 action send_goal --feedback /fr10_force/insert \
  fr10_motion_interfaces/action/Insert \
  "{target_pose: {header: {frame_id: fr10_base_link}, pose: {position: {x: -0.789999032108, y: -0.159000854021, z: 0.088799987}, orientation: {x: -0.707105281178, y: 0.707108281186, z: -1.66533e-16, w: 2.99999483e-6}}}, preinsert_distance: 0.005, insertion_depth: 0.005, insertion_speed: 0.002, desired_wrench: {force: {x: 0.0, y: 0.0, z: 10.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}, selected_axes: [false, false, true, false, false, false], max_force: [50.0, 50.0, 50.0], max_torque: [10.0, 10.0, 10.0], timeout: {sec: 60, nanosec: 0}, profile_id: simulation_v1, completion_behavior: 0}"
```

Theo dõi `/fr10_force/state`, `/fr10_force/filtered_wrench` và
`/fr10_force/corrected_pose`. Hủy an toàn bằng `ros2 action cancel
/fr10_force/insert`; dừng launch bằng `Ctrl-C`. Kết quả đạt yêu cầu phải đi qua
`APPROACH → CONTACT_DETECTED → FORCE_INSERTION → VERIFY_INSERTION → HOLD`.
Goal trên kết thúc sâu 5 mm dưới mặt miệng; không dùng lại z=0.611499 của
fixture cũ. Hai run gần sàn đã PASS trên spring proxy (artifact
`test_results/fr10_force_grounded_clock_20261001T111602Z/REPORT.md`);
đây không phải kiểm chứng tiếp xúc/mating của hai CAD.
Nếu muốn dùng fixture socket với mating frame `[0.55, 0, 0.0938]` và orientation charging
chuẩn (rpy `[-pi, 0, -pi/2]`) thay vì XY home, thêm
`force_fixture_profile:=socket` vào lệnh bringup trên. Launch tự chọn world
tương ứng; không cần truyền đường dẫn `force_world`.

Chế độ này chỉ chứng minh pipeline Gazebo (WrenchStamped → force-position → IK →
JointGroupPositionController). Không dùng lệnh này trên robot thật; `use_force`
bị từ chối khi tắt Gazebo hoặc bật vehicle.

`use_gazebo:=false` chọn FAIRINO hardware/SimMachine qua SDK, **không** tự chuyển
sang fake hoặc MuJoCo. Hiện chỉ hỗ trợ tay robot độc lập
(`use_vehicle:=false`), với `robot_ip` đúng endpoint. Không tự bật Auto/enable,
reset alarm hoặc chạy contact trên robot thật. Xe thật bị từ chối trước khi
khởi động node vì PLC feedback/watchdog chưa được commissioning. SimMachine
phải vượt gate SDK/state; việc launch được không chứng minh ServoJ hoạt động.

Các tên cũ `arm`, `vehicle`, `mode` vẫn tương thích. Khi cùng được truyền,
`use_arm`, `use_vehicle`, `use_gazebo` được ưu tiên. Các launch nội bộ phục vụ ghép hệ thống. Regression/benchmark nằm trong
`test/integration`, không phải cách chạy hằng ngày.

## Nav2, AMCL và SLAM

```bash
ros2 launch vehicle_bringup bringup.launch.py use_arm:=false navigation_mode:=mapping
ros2 launch vehicle_bringup save_map.launch.py map_name:=$HOME/maps/site
ros2 launch vehicle_bringup bringup.launch.py use_arm:=false navigation_mode:=navigation map:=$HOME/maps/site.yaml
```

Dừng mapping trước khi chạy AMCL/navigation. `navigation_mode:=localization`
chỉ chạy AMCL và map server; `disabled` là mặc định. Khởi tạo pose bằng
**2D Pose Estimate** trong RViz trước khi gửi goal. Map mặc định và toàn bộ
cấu hình navigation thuộc `vehicle_navigation`. Xem
[tham số và kiến trúc](../vehicle_navigation/README.md).

Bringup chỉ bật navigation sau khi controller swerve, clock, lidar, odometry
và TF sẵn sàng. Navigation không dùng đồng thời với force fixture hoặc xe thật.
