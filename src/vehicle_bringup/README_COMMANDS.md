# Hướng dẫn lệnh chạy mô phỏng VinUni

Tài liệu vận hành chính cho **ROS 2 Humble + Gazebo Classic 11**.
Khởi động robot bằng `ros2 launch vehicle_bringup bringup.launch.py` và các
tham số bên dưới. Chỉ chạy **một bringup tại một thời điểm** trong cùng ROS domain.

## 1. Chuẩn bị mỗi terminal

```bash
cd ~/Vinuni
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Nếu workspace nằm ở vị trí khác, thay `~/Vinuni` bằng thư mục chứa `src` và
`install`. Terminal khởi động, điều khiển và lưu map phải source cùng workspace
và dùng cùng `ROS_DOMAIN_ID`. Nếu không tự đặt biến này, ROS dùng domain mặc định.

Launch đã xử lý trường hợp terminal đang bật Conda: các node Python chờ khởi
động, spawn model, fixture và chuyển lệnh vận tốc dùng cùng Python với `ros2`
(Python 3.10 trên máy ROS Humble này). Không cần thoát Conda để chạy các launch
đã cập nhật. Kiểm tra Python trực tiếp bằng `/usr/bin/python3` nếu cần import
`rclpy`; lệnh `python3` trong Conda có thể trỏ sang phiên bản khác.

Máy hiện tại đã có bản build. Chỉ khi sửa source hoặc chuyển workspace mới cần:

```bash
cd ~/Vinuni
bash src/vehicle_bringup/scripts/build_workspace.sh
source install/setup.bash
```

Script bảo trì build/test bản mới và cập nhật `ros2_current` khi kiểm tra đạt.
Nó không khởi động robot. Nếu thiếu dependency, dùng:

```bash
bash src/vehicle_bringup/scripts/install_workspace_dependencies.sh
```

## 2. Chọn xe, tay hoặc cả hai

```bash
# Chỉ xe swerve
ros2 launch vehicle_bringup bringup.launch.py use_arm:=false

# Chỉ tay FR10
ros2 launch vehicle_bringup bringup.launch.py use_vehicle:=false

# Xe gắn tay FR10 — cấu hình mặc định
ros2 launch vehicle_bringup bringup.launch.py
```

Mặc định `use_gazebo:=true`, `use_arm:=true`, `use_vehicle:=true`,
`gui:=true`, `rviz:=true`; navigation và force chưa bật.
Các lệnh trên mở Gazebo/RViz, spawn model và nạp controller. Launch không tự gửi
mục tiêu chuyển động cho tay hoặc xe.

Chạy không mở cửa sổ:

```bash
ros2 launch vehicle_bringup bringup.launch.py use_arm:=false gui:=false rviz:=false
```

Đợi model xuất hiện và controller active. Kiểm tra ở terminal thứ hai:

```bash
ros2 control list_controllers
```

| Cấu hình | Controller cần ở trạng thái `active` |
|---|---|
| Chỉ xe | `joint_state_broadcaster`, `swerve_controller` |
| Chỉ tay | `joint_state_broadcaster`, `fairino10_controller` |
| Xe và tay | Cả ba controller trên |

Dừng bringup bằng **Ctrl-C**, đợi process kết thúc rồi chuyển chế độ.
Trong lần kiểm tra hiện tại, một số chế độ có tay báo
`Shutdown braking deadline expired; completion unconfirmed` khi tắt đồng thời
Gazebo và controller. Khởi động/feedback đã đạt, nhưng quy trình shutdown của
tay vẫn cần xử lý; xem [báo cáo kiểm tra](../docs/command_simulation_validation.md).

## 3. Quét bản đồ bằng SLAM

Terminal 1:

```bash
ros2 launch vehicle_bringup bringup.launch.py use_arm:=false navigation_mode:=mapping
```

RViz dùng frame `map`; SLAM Toolbox tạo `/map` từ `/scan` và `/odom`.
Mapping không tự khám phá khu vực. Đưa xe đi bằng lệnh vận tốc ở terminal 2,
quan sát Gazebo để chọn hướng thông thoáng:

```bash
# Tiến chậm; Ctrl-C để ngừng gửi lệnh
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'

# Đi ngang sang trái; chạy riêng sau khi đã dừng lệnh trên
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {y: 0.1}, angular: {z: 0.0}}'

# Xoay tại chỗ; chạy riêng
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{angular: {z: 0.15}}'

# Gửi lệnh dừng sau khi Ctrl-C lệnh đang phát
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

`/cmd_vel` nhận **Twist**. `/cmd_vel_nav` là đầu vào **TwistStamped** nội bộ của
controller; không dùng topic đó cho các lệnh trên. Không gửi vận tốc thủ công
khi Nav2 đang thực hiện goal.

## 4. Lưu bản đồ

Giữ terminal mapping đang chạy. Ở terminal thứ hai đã source:

```bash
ros2 launch vehicle_bringup save_map.launch.py map_name:=$HOME/maps/vinuni_lab
```

Kết quả là `~/maps/vinuni_lab.yaml` và `~/maps/vinuni_lab.pgm`.
`map_name` là đường dẫn **không có phần mở rộng**; thư mục cha được tạo tự động.
Nếu file đã tồn tại, chọn tên khác, ví dụ `vinuni_lab_v2`. Launch báo lỗi thay
vì ghi đè map cũ. Có thể kiểm tra:

```bash
ls -lh ~/maps/vinuni_lab.yaml ~/maps/vinuni_lab.pgm
```

Sau khi lưu xong, **Ctrl-C terminal mapping** trước khi chạy AMCL hoặc Nav2.

## 5. Chỉ định vị bằng AMCL

```bash
ros2 launch vehicle_bringup bringup.launch.py \
  use_arm:=false navigation_mode:=localization \
  map:=$HOME/maps/vinuni_lab.yaml
```

Chọn **2D Pose Estimate** trong RViz: đặt vị trí và hướng gần với vị trí xe thực
trong mô phỏng. AMCL cần pose ban đầu; launch không tự đoán tọa độ xe trên map.
Chế độ này chạy map server + AMCL, không chạy planner/controller Nav2.

Với map vừa tạo tại vị trí spawn mặc định, có thể đặt pose ban đầu ở gốc bằng CLI:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}, covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0685]}}'
```

Không áp dụng tọa độ gốc này cho map khác khi chưa xác định vị trí thực của xe.

## 6. Định vị và điều hướng bằng Nav2

```bash
ros2 launch vehicle_bringup bringup.launch.py \
  use_arm:=false navigation_mode:=navigation \
  map:=$HOME/maps/vinuni_lab.yaml
```

1. Đặt **2D Pose Estimate** để khởi tạo AMCL.
2. Đợi map/laser khớp và Nav2 chuyển sang trạng thái active.
3. Chọn **Nav2 Goal** ở vùng trống trên bản đồ, kéo chuột để đặt hướng cuối.
4. Theo dõi đường đi và xe trong Gazebo; dùng nút **Cancel** của bảng Nav2 để hủy goal.

Có thể gửi goal mẫu bằng terminal sau khi đã định vị. Điểm `(0.4, 0)` cần nằm
trong vùng trống của map đang dùng:

```bash
ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 0.4, y: 0.0}, orientation: {w: 1.0}}}}'
```

Nếu bỏ `map:=...`, launch dùng map mẫu `vehicle_navigation/maps/vinfast.yaml`.
Với site mới, dùng bản đồ vừa quét. Để bật cả tay FR10 khi chạy navigation,
bỏ `use_arm:=false`; vùng va chạm khi tay vươn ra cần được kiểm chứng riêng.

## 7. Các tham số thường dùng

Xem toàn bộ tham số và giá trị mặc định:

```bash
ros2 launch vehicle_bringup bringup.launch.py --show-args
ros2 launch vehicle_bringup save_map.launch.py --show-args
```

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `use_arm` / `use_vehicle` | `true` / `true` | Chọn thành phần robot |
| `use_gazebo` | `true` | Dùng Gazebo |
| `navigation_mode` | `disabled` | `disabled`, `mapping`, `localization`, `navigation` |
| `map` | Map VinFast đi kèm | File YAML bản đồ |
| `gui` / `rviz` | `true` / `true` | Bật cửa sổ Gazebo/RViz |
| `gazebo_port` | `11355` | Cổng Gazebo; đổi nếu đang bị chiếm |
| `world` | World VinFast đi kèm | Đường dẫn `.world` tùy chọn |
| `nav2_params_file` | `vehicle_navigation/config/nav2.yaml` | Ghi đè cấu hình Nav2/AMCL |
| `slam_params_file` | `vehicle_navigation/config/slam.yaml` | Ghi đè cấu hình SLAM |
| `rviz_config_file` | Chọn theo chế độ | Ghi đè profile RViz |
| `cmd_vel_topic` | `/cmd_vel` | Đầu vào Twist của bộ chuẩn hóa vận tốc |
| `scan_topic` / `odom_topic` | `/scan` / `/odom` | Đầu vào navigation; không đổi tên topic sensor Gazebo |
| `autostart` | `true` | Tự activate lifecycle Nav2/AMCL |
| `use_respawn` | `false` | Khởi động lại process navigation bị crash |
| `navigation_startup_timeout` | `90.0` | Số giây thực chờ dữ liệu/controller sẵn sàng |
| `log_level` | `info` | Mức log navigation |
| `use_detection` | `false` | Bật nhận diện CCS2 khi có xe |
| `use_force` | `false` | Bật fixture thử force trong Gazebo |

Ví dụ thay cấu hình và chạy headless:

```bash
ros2 launch vehicle_bringup bringup.launch.py \
  use_arm:=false navigation_mode:=navigation \
  map:=$HOME/maps/vinuni_lab.yaml \
  gui:=false rviz:=false gazebo_port:=11356
```

Các alias cũ `arm`, `vehicle`, `mode` vẫn được nhận; dùng tên `use_*` cho lệnh mới.

## 8. Fixture thử force — tùy chọn

Profile tốc độ theo quan sát thời gian thực cho tay độc lập/front_mid:

```bash
ros2 launch vehicle_bringup bringup.launch.py \
  use_arm:=true use_vehicle:=false use_gazebo:=true \
  use_force:=true station:=front_mid initial_state:=home \
  gui:=true rviz:=false use_arm_camera:=false \
  matched_physics_step_s:=0.0001 \
  force_controller_overlay:=/home/tinnt/Vinuni/src/fr10_arm_control/fr10_pose_controller/config/matched_socket_wall_speed.yaml
```

Mục tiêu là free-space khoảng10mm/s thực và force approach khoảng1mm/s thực;
đây là profile commissioning Gazebo, chưa áp dụng cho tay gắn trên xe hoặc
robot thật. Vận tốc trong YAML vẫn tính theo sim-time: free-space144mm/s,
force12mm/s. Kết quả phụ thuộc RTF và đường đi; đoạn phanh/ép sát vai chặn
vẫn chậm hơn đoạn đi trong không khí. Không bù tốc độ bằng cách đổi clock,
timestamp feedback, giới hạn lực hay ngưỡng tare.

Profile kế thừa preinsert5mm; khoảng dẫn nominal1mm thay cho0,1mm giúp các
mục tiêu online có endpoint vận tốc0 không làm Ruckig bị giới hạn quanh
0,19mm/s thực. Khoảng dẫn mới cũng đi vào luật phanh trước vai chặn. Trần
gia tốc correction dọc trục là0,20m/s², trần vận tốc correction100µm/s.
Damping dọc trục30000Ns/m đi cùng filter2Hz; damping quay80 và stiffness
quay20 giúp phân bố lại mô-men ở vai chặn. Giới hạn hoàn tất mô-men0,1Nm,
lực5±1N và các kiểm tra fit/braking/tare/freshness không được nới.
Không áp dụng các giá trị này vào phiên đang chạy; controlled-stop về HOLD,
tắt launch cũ rồi khởi động lại. Gửi một pose miệng cổng như lệnh bên dưới.
Kết quả đo và giới hạn kiểm chứng: báo cáo tốc độ (local: `test_results/fr10_speed_20261016/REPORT.md`).

Để thử tay độc lập ở `front_mid` với đoạn tiếp cận compliant ngắn hơn:

```bash
ros2 launch vehicle_bringup bringup.launch.py \
  use_vehicle:=false use_force:=true station:=front_mid initial_state:=home \
  gui:=true rviz:=false use_arm_camera:=false \
  force_controller_overlay:=/home/tinnt/Vinuni/src/fr10_arm_control/fr10_pose_controller/config/matched_socket_near_preinsert.yaml
```

Profile này kế thừa toàn bộ commissioning force, chỉ đưa preinsert về cách
miệng cổng 5 mm. Với `front_mid`, preinsert z=0,325 m, mouth z=0,320 m và
seated z=0,305 m. Worker vẫn kiểm tra đường đi, clearance và stationary tare.
Timestep 62,5 µs, solver 2.000 vòng, tốc độ cắm 0,5 mm/s và các gate lực/fit
không đổi. Đoạn 25 mm trước đây đi compliant được đưa sang free-space planning;
thời gian compliant trước mouth giảm từ60 xuống10 giây mô phỏng. Đây không
phải tuyên bố Gazebo đạt thời gian thực hoặc đã qua full contact/HOLD.

Terminal khác, cùng ROS domain và đã source workspace:

```bash
ros2 run vehicle_bringup fr10_force_status.py
```

Biểu đồ trực tiếp lực Fx/Fy/Fz, lực tham chiếu của phase, độ sâu, tốc độ
clock Gazebo (RTF) và tốc độ TCP thực theo đường đi3D. Có thể mở ngay trong
khi đang cắm; đây là monitor chỉ đọc:

```bash
/usr/bin/python3 /home/tinnt/Vinuni/src/vehicle_bringup/scripts/fr10_force_plot.py \
  --output /home/tinnt/Vinuni/test_results/force_live
```

Trục thời gian là giây thực; độ sâu âm là còn trước mouth, dương là đã qua
mouth. Tốc độ TCP là tổng chiều dài đường đi3D trong cửa sổ5giây chia cho
thời gian thực; `axial_speed_wall` riêng chỉ đo theo trục chèn. Đồ thị lấy
mẫu5Hz, vẽ2Hz và giữ5phút gần nhất để hạn chế tải CPU.
CSV giữ toàn bộ mẫu từ lúc mở, PNG được lưu khi đóng cửa sổ hoặc Ctrl-C.
Sau khi build có thể dùng `ros2 run vehicle_bringup fr10_force_plot.py`.
Để ghi biểu đồ không mở GUI, thêm `--headless --duration 30 --output /tmp/force_check`.
Nếu mở từ VS Code Snap gặp lỗi thư viện `libpthread`/`GLIBC_PRIVATE`, chạy
`unset GTK_PATH GIO_MODULE_DIR GTK_EXE_PREFIX GTK_DATA_PREFIX` trong terminal
monitor rồi mở lại. Đây chỉ là môi trường tiến trình biểu đồ.
Monitor báo đỏ và ngắt đường khi feedback cũ hơn1giây; số đo trước đó
không được coi là đang cập nhật. Không có dữ liệu lịch sử trước khi mở monitor.

Sau khi runtime READY, gửi đúng một pose miệng cổng để tự chạy preinsert,
tare, tiếp cận compliant và cắm:

```bash
ros2 topic pub --once /charging_port/raw_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: fr10_base_link}, pose: {position: {x: -0.720, y: -0.159, z: 0.320}, orientation: {x: -0.707106781187, y: 0.707106781187, z: 0.0, w: 0.0}}}'
```

Monitor chỉ đọc feedback/result và clock; hiển thị khoảng cách tới mouth,
lực mục tiêu thực của phase, tốc độ theo wall time và real-time factor.
Không gửi thêm lệnh pose khi phiên force đang giữ quyền.
`use_arm_camera:=false` tắt depth sensor, giữ model/TF/collision của camera.
Không dùng tùy chọn này nếu đang cần perception từ ảnh tay robot.
`matched_solver_iterations` là tùy chọn convergence study (50..2000), mặc
định vẫn2.000; các candidate500/1.000 vòng chưa được chọn cho vận hành.

Khởi động tay độc lập với fixture; lệnh này chỉ nạp môi trường, chưa gửi goal cắm:

```bash
ros2 launch vehicle_bringup bringup.launch.py \
  use_vehicle:=false use_force:=true force_fixture_profile:=home
```

Fixture matched khi tay gắn trên xe:

```bash
ros2 launch vehicle_bringup bringup.launch.py \
  use_force:=true station:=front_mid initial_state:=home demo:=false
```

Force dùng `navigation_mode:=disabled`, không chạy đồng thời với Nav2.
`demo:=true` hiện bị khóa. Các lệnh trên không chứng minh chất lượng tiếp xúc hay
hoàn thành thao tác cắm; chi tiết nằm trong [tài liệu force](README.md).
Xe thật chưa được commissioning; các ví dụ trong tài liệu này dùng Gazebo.

## 9. Kiểm tra và xử lý lỗi

```bash
ros2 control list_controllers
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /map --once --field info
ros2 run tf2_ros tf2_echo map base_footprint
ros2 lifecycle get /amcl
ros2 lifecycle get /bt_navigator
```

Các lệnh `hz`/`echo`/`tf2_echo` chỉ quan sát; Ctrl-C để thoát. `/map` có ở chế độ
mapping/localization/navigation; `/amcl` có ở localization/navigation;
`/bt_navigator` chỉ có ở navigation. TF `map → base_footprint` của AMCL xuất hiện
sau khi có initial pose và laser phù hợp.

| Triệu chứng | Xử lý |
|---|---|
| Không tìm thấy package/launch hoặc tham số mới | Source lại `install/setup.bash`; build lại nếu đã sửa source |
| Gazebo báo port đang được dùng | Dừng bringup cũ hoặc đổi `gazebo_port`; nếu chạy song song phải tách ROS domain |
| Không có map trong mapping | Kiểm tra `/scan`, `/odom`, controller active và clock |
| AMCL chưa có TF map | Đặt 2D Pose Estimate rồi kiểm tra `/scan` |
| Timeout lúc khởi động navigation | Đọc log gate; kiểm tra controller, `/clock`, `/scan`, `/odom` và TF |
| Lưu map báo đã tồn tại | Chọn `map_name` mới |
| Map YAML/ảnh không tồn tại | Kiểm tra đường dẫn YAML và trường `image` trong YAML |
| Máy không có display | Thêm `gui:=false rviz:=false` |
| Tay báo `Shutdown braking deadline expired` khi Ctrl-C | Lỗi shutdown đã ghi nhận; đợi các process kết thúc, xem báo cáo kiểm tra |
| Thiếu `rclpy._rclpy_pybind11`, traceback trỏ tới Miniconda | Kiểm tra đang source runtime mới có bản sửa chọn Python trong launch; xem phần kiểm tra Conda trong báo cáo |

Báo cáo chạy mô phỏng thực tế: [command_simulation_validation.md](../docs/command_simulation_validation.md).


## 10. FR10: công cụ và kiểm thử

Nội dung hướng dẫn chuyên biệt trước đây ở `README_LENH.md` được hợp nhất
vào đây. Các bước chuẩn bị và bringup dùng các mục 1–8 phía trên.

Cụm gá theo ảnh mới: **CCS2 DC phía trên, D455 phía dưới tại tâm mặt bích**.
Chọn `wrist_layout:=photo_dc_d455`; kích thước tạm và lệnh Gazebo/RViz ở
[hướng dẫn cụm gá theo ảnh](../../docs/fr10_photo_dc_d455.md).
Các giá trị Tool 1/D435 bên dưới thuộc cấu hình mặc định `calibrated`.

Tool 0 là tâm mặt bích; Tool 1 là tâm bề mặt trước đầu cắm sạc.
Cấu hình dùng chung duy nhất: [fr10_tool.yaml](../../src/vehicle_description/config/fr10_tool.yaml).
Tool 1 so với Tool 0: `[55.140, -46.496, 215.297] mm`, góc cố định XYZ
`[0, 0, 136.840] độ`. Gazebo, RViz/MoveIt và MuJoCo đọc cùng mô hình Xacro;
khởi động lại phiên mô phỏng sau khi build để nhận mô hình mới.

Camera D435 căn tâm mặt bích (tâm thân X=Y=0 trong `plate_frame`), hướng nhìn
song song hướng cắm; mặt sau camera cách mặt bích 30 mm.
[Bố trí camera và giá đỡ](../../docs/fr10_camera_mount.md).

Profile mới nhắm tốc độ thực free-space10mm/s, force approach1mm/s:
`matched_socket_wall_speed.yaml`, dùng kèm `matched_physics_step_s:=0.0001`;
lệnh đầy đủ và phạm vi kiểm chứng nằm ở mục8 của hướng dẫn trên.

Theo dõi trực tiếp khi điều khiển robot: **một cửa sổ, sáu ô**:
Fx/Fy/Fz chung một ô; Tx/Ty/Tz chung một ô; X, Y, Z mỗi trục một ô;
vận tốc TCP một ô (mm/s theo thời gian thực). Vị trí là FK từ feedback khớp.
Mở terminal thứ hai, dùng cùng ROS domain với robot:

```bash
cd /home/tinnt/Vinuni
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 src/vehicle_bringup/scripts/fr10_robot_plot.py
```

Nếu robot chạy MuJoCo ở domain 212, thêm `ROS_DOMAIN_ID=212 ROS_LOCALHOST_ONLY=1`
trước `/usr/bin/python3`. Lệnh mặc định dùng `/ft_sensor/wrench` (lực/mô-men
cảm biến, N/Nm, frame hiển thị trên hình) và `/fr10_motion/tcp_pose` (mm,
frame base của controller). Đây là wrench cảm biến, có thể gồm trọng lực/bias
tùy nguồn; không tự trộn với wrench đã lọc trong hệ lắp ghép.
Có thể đổi bằng `--wrench-topic` và `--tcp-topic`.
Trục thời gian là giây thực; cửa sổ xem mặc định 120 s, đổi bằng `--window`.
CSV lấy mẫu 20 Hz, hình cập nhật 2 Hz để giảm tải khi mô phỏng.
Dữ liệu cũ/đóng băng hiển thị STALE. Vận tốc là quãng đường TCP đo được chia
thời gian thực trong cửa sổ khoảng 0,5 s, không phải tốc độ đặt.
Đóng cửa sổ hoặc nhấn Ctrl+C để lưu `dashboard.png`, `wrench.csv`, `tcp.csv` trong thư mục
`test_results/fr10_live_plot_<thời gian>` được in ở terminal. CSV giữ toàn phiên,
PNG giữ cửa sổ xem cuối. Có thể chỉ định thư mục mới bằng `--output`.
Trình theo dõi chỉ đọc dữ liệu, không phát lệnh chuyển động.

Biểu đồ lực/độ sâu/RTF khi đang cắm: xem mục8 của hướng dẫn trên, hoặc sau
khi source ROS/workspace chạy:

```bash
/usr/bin/python3 /home/tinnt/Vinuni/src/vehicle_bringup/scripts/fr10_force_plot.py
```

Kiểm tra SDK FR10 3.9.3 độc lập (ServoJ, ServoCart, mô-men, lực và CNDE):
[fr10_sdk_tests](../../src/hardware_interface/fr10_sdk_tests/README.md).

Thử riêng FT_Control kết hợp MoveL chậm trên bề mặt tiếp xúc:
[cấu hình, lệnh chạy và biểu đồ lực](../../src/hardware_interface/fr10_sdk_tests/README.md#thử-giữ-lực-fz-trong-khi-chạy-movel).

Hướng dẫn kiểm thử 6DOF mới: [profile, nhóm case, biểu đồ và tiêu chí đánh giá](../../docs/fr10_6dof_mujoco_test_guide.md).

Chạy ma trận một Admittance 6D trong MuJoCo, tăng sai lệch từ nhỏ tới miền
5 mm / 5°, và xuất biểu đồ lực/độ sâu/độ lệch. Mặc định tắt spiral và dò chủ động:

```bash
cd /home/tinnt/Vinuni
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 src/vehicle_bringup/test/integration/run_fr10_capture_mujoco.py
```

Thử riêng sai lệch đồng thời có norm 5 mm và 5°:

```bash
/usr/bin/python3 src/vehicle_bringup/test/integration/run_fr10_capture_mujoco.py \
  --mode passive --cases combined_diagonal_5
```

Lệnh chạy mô phỏng riêng ở ROS domain212. Cách xem biểu đồ trực tiếp và phát
lại tay/cổng trong MuJoCo nằm ở [hướng dẫn thử](../../src/vehicle_bringup/test/integration/README.md).

Miền 5 mm tính theo norm XYZ, 5° tính theo norm vector quay. Đây là profile
thử nghiệm; chưa chứng minh cắm được mọi sai lệch trong miền này. Xem
[kết quả và các giới hạn đã phát hiện](../../docs/fr10_mujoco_admittance_audit_20261016.md).

Kết quả tự chạy kiểm tra: [báo cáo mô phỏng](../../src/docs/command_simulation_validation.md).
