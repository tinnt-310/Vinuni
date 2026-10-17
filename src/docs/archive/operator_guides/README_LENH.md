# Lệnh chạy nhanh — tay FR10 và xe

Các ví dụ dưới đây dành cho **Gazebo**, không phải robot thật. Chỉ chạy một launch tại một thời điểm.

## 1. Chuẩn bị mỗi terminal

```bash
cd ~/Vinuni
source tools/source_simulation.sh
```

Nếu workspace chưa build, chạy `./tools/build_simulation.sh` một lần rồi source lại. Terminal điều khiển và đọc pose cũng phải source như trên, cùng `ROS_DOMAIN_ID` với terminal launch.

## 2. Khởi động

Chỉ tay robot:

```bash
ros2 launch vehicle_bringup bringup.launch.py use_arm:=true use_vehicle:=false use_gazebo:=true
```

Chỉ xe:

```bash
ros2 launch vehicle_bringup bringup.launch.py use_arm:=false use_vehicle:=true use_gazebo:=true
```

Tay robot gắn trên xe:

```bash
ros2 launch vehicle_bringup bringup.launch.py use_arm:=true use_vehicle:=true use_gazebo:=true
```

`use_arm`: bật tay; `use_vehicle`: bật xe; `use_gazebo`: chọn Gazebo. Cả ba mặc định `true`, nên lệnh chạy cả tay và xe ngắn nhất là `ros2 launch vehicle_bringup bringup.launch.py`.

Thêm `gui:=false rviz:=false` nếu không cần cửa sổ. Launch mặc định chờ lệnh, không tự di chuyển hay cắm sạc. Đợi controller active trước khi gửi lệnh:

```bash
ros2 control list_controllers
```

## 3. Điều khiển tay robot

Để xe đứng yên trước khi thử tay. JOINT đầu vào dùng **độ**, đủ sáu khớp `j1` đến `j6`.

Về cấu hình home:

```bash
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState "{name: [j1, j2, j3, j4, j5, j6], position: [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]}"
```

Ví dụ đổi J1 sang 5°:

```bash
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState "{name: [j1, j2, j3, j4, j5, j6], position: [5.0, -90.0, 90.0, -90.0, -90.0, 0.0]}"
```

POSE di chuyển tự do tới TCP yêu cầu — vị trí tính bằng **mét**, orientation là quaternion `(x, y, z, w)` trong frame `fr10_base_link`:

```bash
ros2 topic pub --once /fr10_motion/pose_target geometry_msgs/msg/PoseStamped "{header: {frame_id: fr10_base_link}, pose: {position: {x: -0.75, y: -0.159, z: 0.65}, orientation: {x: -0.70710678, y: 0.70710678, z: 0.0, w: 0.0}}}"
```

Đọc pose hiện tại trước khi chọn đích; ví dụ trên không bảo đảm hợp lệ cho mọi tool, station hoặc tư thế ban đầu. Planner kiểm tra collision và kỳ dị, có thể tìm điểm khớp trung gian cho đoạn tự do. `/fr10_motion/pose_target` là đầu vào POSE tự do.

Khi bật profile matched cùng `force.automatic_pose: true`, `/charging_port/raw_pose`
nhận **pose tâm miệng cổng**, +Z vào cổng, rồi tự chạy preinsert → chèn theo lực → HOLD.
Pose phải có timestamp mới theo clock của controller. Khi force tắt, topic này
giữ hành vi position insertion legacy. Hướng dẫn commissioning và bằng chứng:
[FR10 insertion pipeline](../../../../docs/fr10_insertion_pipeline.md).

Dừng tay có kiểm soát (không phải emergency stop phần cứng):

```bash
ros2 topic pub --once /fr10_motion/controlled_stop std_msgs/msg/Empty "{}"
```

## 4. Điều khiển xe

Mỗi lệnh vận tốc chạy liên tục ở 10 Hz. Nhấn `Ctrl-C` để ngừng lệnh đang phát trước khi thử lệnh khác. Vận tốc tịnh tiến dùng m/s, vận tốc quay dùng rad/s.

Tiến chậm; đổi `x` thành `-0.1` để lùi:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

Đi ngang; đổi dấu `y` để đổi chiều:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.1}}"
```

Quay tại chỗ; đổi dấu `z` để đổi chiều:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.1}}"
```

Dừng publisher vận tốc bằng `Ctrl-C`, rồi gửi zero:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Controller cũng dừng command khi quá watchdog 0,5 s. Khi insertion đang giữ parking lease, lệnh chạy xe bị chặn; không dùng publisher khác để vượt khóa.

## 5. Đọc pose và trạng thái

TCP hiện tại trong `fr10_base_link` (FK từ joint feedback, không phải pose mục tiêu):

```bash
ros2 topic echo --once /fr10_motion/tcp_pose --qos-reliability best_effort
```

Góc khớp thực tế — `position` ở đây dùng **radian**, xem `name` để biết thứ tự:

```bash
ros2 topic echo --once /joint_states --qos-reliability best_effort
```

Pose xe từ wheel odometry trong frame `odom`:

```bash
ros2 topic echo --once /odom --field pose.pose
```

TCP trong frame `odom` khi tay gắn trên xe (theo dõi liên tục, `Ctrl-C` để thoát):

```bash
ros2 run tf2_ros tf2_echo odom charging_tcp
```

Trạng thái tay và kết quả tiếp nhận/lập kế hoạch lệnh:

```bash
ros2 topic echo --once /controller_status --qos-durability transient_local
ros2 topic echo /fr10_motion/command_status --qos-durability transient_local
```

Bỏ `--once` ở các lệnh đọc để theo dõi liên tục. Dừng launch bằng `Ctrl-C` ở terminal khởi động.

`use_gazebo:=false` chọn hardware/SimMachine FAIRINO. MuJoCo dùng entrypoint
riêng bên dưới. Kết quả commissioning chỉ áp dụng cho đúng profile và tổ hợp
station/tư thế đầu ghi trong báo cáo.

## Commissioning cắm sạc theo một pose

Chạy tay độc lập với profile đã đo; terminal thứ hai gửi đúng một pose miệng
cổng `front_mid`. `--use-sim-time` và `stamp: now` lấy timestamp từ clock Gazebo:

```bash
ros2 launch vehicle_bringup bringup.launch.py \
  use_arm:=true use_vehicle:=false use_gazebo:=true use_force:=true \
  station:=front_mid \
  force_controller_overlay:=src/fr10_arm_control/fr10_pose_controller/config/matched_socket_commissioning.yaml

ros2 topic pub --once --use-sim-time /charging_port/raw_pose geometry_msgs/msg/PoseStamped \
  "{header: {stamp: now, frame_id: fr10_base_link}, pose: {position: {x: -0.72, y: -0.159, z: 0.32}, orientation: {x: -0.7071067811865475, y: 0.7071067811865475, z: 0.0, w: 0.0}}}"
```

Đợi controller active trước khi gửi. Robot tự tới preinsert, chèn và xác nhận
HOLD; `/fr10_force/result` trả kết quả. Muốn rút sau HOLD, gửi:

```bash
ros2 topic pub --once /fr10_motion/retract std_msgs/msg/Empty "{}"
```

Dùng entrypoint chung dưới đây sau khi source runtime. Script chọn CAD/station
canonical, gửi đúng một pose miệng cổng, lưu feedback và kiểm tra TCP vật lý,
contact, HOLD đủ5 giây nguồn đo và rút về preinsert. Đây là profile commissioning mô phỏng; trạng
thái và kết quả cụ thể nằm trong [pipeline](../../../../docs/fr10_insertion_pipeline.md).
Không cần lấy station JSON từ một lượt test trước.

Báo cáo commissioning (local: `test_results/fr10_e2e_20261002/REPORT.md`) có kết quả
cắm/HOLD/rút Gazebo trên xe, MuJoCo nghiêng/gập, fault sau HOLD và planner24/24.
Docker Auto đã PASS chu trình native83 với ServoJ và5,008 giây HOLD nguồn đo;
contact ở Docker là proxy CAD từ khớp SDK, không phải cảm biến FT vật lý.
Lượt84 gặp lỗi ServoJ14 và được giữ là FAIL. Lượt85 mới nhất đã PASS từ tư thế
người dùng đặt cách cổng1,174 m, HOLD5,032 giây nguồn đo và để robot tại cổng;
đọc lại SDK sau shutdown xác nhận đúng vị trí.

```bash
python3 tools/run_matched_station_gazebo.py test_results/my_mujoco_run \
  --backend mujoco --domain 192 --insert --pose-input --retract-after-hold

python3 tools/run_matched_station_gazebo.py test_results/my_gazebo_run \
  --backend gazebo --domain 193 --port 11987 --insert --pose-input --retract-after-hold
```

`--station` chọn `front_low`, `front_mid`, `front_high`, `left`, `right`,
`tilted_up`; `--initial-state` chọn `home`, `left`, `right`, `folded`.
Gazebo có thêm `--mounted` để kiểm tra tay gắn trên xe và parking handoff.
Mỗi output directory phải mới; chọn DDS domain riêng cho mỗi phiên.

Ví dụ chạy trọn chu trình trên xe, bao gồm kiểm tra trả parking sau rút:

```bash
python3 tools/run_matched_station_gazebo.py test_results/my_mounted_run \
  --backend gazebo --mounted --domain 198 --port 11989 \
  --insert --pose-input --retract-after-hold --run-timeout 1800
```

Với xe, cổng cố định trong `world`. Script lấy TF thực tại timestamp gửi pose;
không chép tọa độ `fr10_base_link` của ví dụ tay độc lập sang xe. Launch thường
chờ lệnh; bài thử trên tự gửi một pose sau khi đo tải, xác minh model và feedback.

Bài thử lỗi cảm biến MuJoCo sau HOLD đã xác minh:

```bash
python3 tools/run_matched_station_gazebo.py test_results/my_force_fault \
  --backend mujoco --domain 196 --insert --pose-input --fault-after-hold overload
```

`--fault-after-hold` nhận `duplicate`, `nan`, `overload`, `torque`,
`lost_contact`; không kết hợp với `--retract-after-hold`. Báo cáo chỉ PASS khi
lỗi đúng loại và có xác nhận dừng. Sau kết quả cắm thành công, lỗi phát sinh
trong HOLD phải tạo một kết quả failure mới trên `/fr10_force/result`; phiên
vẫn giữ quyền điều khiển để chờ xử lý hoặc lệnh rút tường minh.

Ma trận planner không gửi lệnh actuator:

```bash
python3 tools/run_fr10_planning_matrix.py test_results/my_planning_matrix --domain 194
```

Docker Fairino chỉ qua preflight khi đúng image/controller 3.9.3 tại
`192.168.58.2`, dữ liệu SDK khỏe và tủ đã Auto. Web ghi FR5 là nhãn của
SimMachine; benchmark dùng joint ServoJ và FK/CAD FR10 ở phía ROS. Không tự
đổi Mode, enable hoặc reset alarm.

```bash
python3 tools/run_matched_station_gazebo.py test_results/my_native_run \
  --backend fairino --domain 195 --native-probe /tmp/fr10_simmachine_readonly_probe \
  --insert --pose-input --retract-after-hold
```

Contact trong lượt Docker là proxy CAD quasi-static tại đúng khớp SDK đo được;
observer không tích phân servo, không phát joint feedback hoặc lệnh robot.
SimMachine không chứa cơ học socket CAD. Không coi đó là thử tiếp xúc trên
robot thật. Xem điều kiện preflight và compile probe trong pipeline.

### Nội bộ admittance / pose uncertainty — đang nghiệm thu

Theo dõi kết quả thực tế và các phần chưa PASS tại
`test_results/fr10_admittance_pose_error_20261003/WORK_LOG.md`.
Profile mới kế thừa `matched_socket_commissioning.yaml`; chưa dùng để nghiệm thu
robot thật. Không chạy thêm admittance_controller giành joint command.
FT Fairino raw/reference/active và API hiệu chuẩn được mô tả tại
`docs/fr10_force_sensor_handoff.md`.

Ví dụ MuJoCo với sai số đầu vào X=2 mm, Ry=2°; CAD thật giữ nguyên. Script tự
kiểm tra preinsert, contact, độ khớp cơ khí, HOLD và rút chủ động:

```bash
python3 tools/run_matched_station_gazebo.py test_results/my_pose_error_run \
  --backend mujoco --domain 192 --insert --six-axis --pose-input \
  --retract-after-hold --pose-lateral-mm 2 0 --pose-rotation-deg 0 2 0 \
  --overlay "$(ros2 pkg prefix fr10_pose_controller)/share/fr10_pose_controller/config/pose_uncertainty_commissioning.yaml" \
  --run-timeout 2500
```

Chỉ chạy một graph commissioning tại một thời điểm. Source hiện dùng chung
64/96 ms cho validation/lease ở hai profile matched; FT/encoder vẫn50 ms.
Runtime Stage51 đã build/publish ngân sách64/96 ms, gain legacy đã đo, calibration
tiếp xúc ODE dùng chung và feedback encoder theo bước physics; khi thay đổi source cần build release mới trước khi chạy. Sai số thử nằm trong norm lateral2 mm và rotation-vector2°, không phải
2 mm/2° độc lập trên từng trục. Stage43 đã PASS full cycle cho Y−2 mm/Rx+2°, lệch chéo norm2 mm/2°
từ folded, Rz−2° và X+2 mm/Ry+2°; keyed error34,7–95,3 µm, giữ gate250 µm.
Stage41 có hai ca đối xứng X+2 mm/Ry−2° và X−2 mm/Ry+2° theo revision riêng.
Stage48 default legacy và yaw+2° MuJoCo đã PASS cắm/HOLD5s-source/rút/dừng;
ca X−2mm/Ry−2° từ right đã PASS cắm/HOLD/fault duplicate FT/dừng, không rút.
Stage51 đã PASS build15 package,134 CTest/1 numerical IK SKIP có chủ đích,
83 test công cụ và19 test launch/model. Gazebo47 FAIL tại VERIFY vì encoder
quá50ms-wall lúc physics chậm; không có xác nhận dừng. Stage51 đã sửa nhịp
feedback theo bước physics và đang replay yaw+2°. Mounted Gazebo còn phải
replay sau khi chassis đứng yên bằng đo pose thực. Toàn bộ backend/matrix
chưa được chấp nhận. Xem báo cáo (local: `test_results/fr10_admittance_pose_error_20261003/REPORT.md`).

Nguồn ngoại lực Gazebo tùy chọn ở
`config/gazebo_contact_wrench_commissioning.yaml` là instrument contact mô phỏng,
tách khỏi FT tại khớp. Decoder ODE mới có regression riêng; full Gazebo đang
nghiệm thu. Đọc WORK_LOG trước khi dùng kết quả này để kết luận về phần cứng.
