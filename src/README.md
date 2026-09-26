# Modular vehicle workspace

System entry point: `ros2 launch vehicle_bringup simulation.launch.py enable_arm:=false`.
Use `enable_arm:=true` for the unchanged FR10 simulation (known physical instability remains).
Use `enable_detection:=true` to connect CCS2 perception to the body D455.

Authoritative packages: vehicle_bringup, vehicle_description, fr10_arm_control,
swerve_vehicle_control, charger_detection. Legacy mixed source remains pending
user cleanup approval and is excluded with COLCON_IGNORE; old install artifacts
may still expose old package names. Do not use those as the new entry point.

Full transfer/reference/verification records: `docs/refactor/`. No source was deleted.

---

# Hướng dẫn sử dụng Workspace — Mô phỏng xe tự hành (Phần thân xe)

Tài liệu này hướng dẫn cách cài đặt và chạy mô phỏng **xe điện 4 bánh lái độc lập (Swerve Drive)** trên Gazebo + điều hướng tự động bằng **Nav2**.

> ⚠️ **Phạm vi**: Tài liệu chỉ tập trung vào **phần thân xe (base / chassis)** — bánh xe, cảm biến lidar, camera, điều hướng.
> Phần **cánh tay UR10** (docking, gắp sạc, solvePnP...) sẽ **được bỏ qua** trong hướng dẫn này.

---

## 1. Tổng quan kiến trúc

| Thành phần | Package | Vai trò |
|---|---|---|
| Mô tả robot (URDF/Xacro), world, bản đồ, cấu hình | [`thanxe_new_description`](thietkemoi/thanxe_new_description/package.xml) | Chứa model xe, mesh, bản đồ VinFast, các file launch |
| Node điều khiển Swerve + plugin ros2_control | [`my_ur_control_cpp`](my_ur_control_cpp/package.xml) | Tính động học Swerve, publish `/steering_controller/commands` & `/drive_controller/commands`, publish `/odom` |
| Message phát hiện cổng sạc CCS2 | [`ccs2_socket_detector`](ccs2_socket_detector/package.xml) | Interface package (dependency của `my_ur_control_cpp`) |
| Mô tả UR10 | [`ur_description`](Universal_Robots_ROS2_Description/package.xml) | **Bỏ qua** (chỉ cần để build/xacro không lỗi) |

**Luồng hoạt động:**

```
Nav2 controller_server (MPPI) → /cmd_vel → velocity_smoother → /cmd_vel_nav
      → swerve_drive (node C++) → /steering_controller/commands + /drive_controller/commands
      → ros2_control (gazebo_ros2_control) → bánh xe trong Gazebo
      → /odom + TF (odom → base_footprint) → Nav2 (AMCL, costmap)
```

---

## 2. Yêu cầu hệ thống (Prerequisites)

- **Ubuntu 22.04** + **ROS 2 Humble**
- **Gazebo Classic 11** (không dùng Gazebo Sim / Ignition)
- Các gói ROS cần thiết:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-nav2-bringup \
  ros-humble-nav2-amcl \
  ros-humble-nav2-controller \
  ros-humble-nav2-planner \
  ros-humble-nav2-smac-planner \
  ros-humble-nav2-mppi-controller \
  ros-humble-nav2-costmap-2d \
  ros-humble-nav2-behaviors \
  ros-humble-nav2-bt-navigator \
  ros-humble-nav2-velocity-smoother \
  ros-humble-joint-state-broadcaster \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-ur-description \
  ros-humble-cv-bridge \
  ros-humble-message-filters \
  libeigen3-dev \
  libopencv-dev
```

- **Driver đồ họa (chỉ cần cho máy có GPU NVIDIA dạng hybrid / Optimus):**

```bash
# Kiểm tra GPU
nvidia-smi
```

---

## 3. Tạo workspace & copy source code

Repo này **không clone được bằng một lệnh `git clone` duy nhất** vì workspace chứa nhiều package nằm rải rác. Hãy tạo đúng cấu trúc thư mục trước khi cài dependency.

### 3.1. Tạo thư mục workspace

```bash
mkdir -p ~/du_an_ws/src
cd ~/du_an_ws/src
```

> ⚠️ **Nên đặt tên `du_an_ws`** (hoặc sửa lại đường dẫn — xem [Mục 9](#9-tùy-chỉnh-đường-dẫn-cho-máy-của-bạn)), vì [`complete.launch.py`](thietkemoi/thanxe_new_description/launch/complete.launch.py:119) hard-code đường dẫn `$HOME/du_an_ws/src/models` để spawn trạm sạc.

### 3.2. Copy source code vào `src/`

Tuỳ nguồn bạn nhận code (file nén, USB, `git clone`, ...), kết quả cuối cùng trong `~/du_an_ws/src` **phải có đủ** các thư mục sau:

```
du_an_ws/
└── src/
    ├── thietkemoi/
    │   └── thanxe_new_description/         # model xe, world, bản đồ, config (BẮT BUỘC)
    ├── my_ur_control_cpp/                  # node swerve drive + plugin (BẮT BUỘC)
    ├── ccs2_socket_detector/               # message package (BẮT BUỘC)
    ├── models/                             # model SDF trạm sạc tram_sac_VF (BẮT BUỘC)
    ├── Universal_Robots_ROS2_Description/  # ur_description (BẮT BUỘC để xacro không lỗi)
    └── Universal_Robots_ROS2_Driver/       # (tuỳ chọn)
```

Ví dụ nếu nhận code dạng file nén:

```bash
# Giải nén thẳng vào src/
unzip ~/Downloads/du_an_ws_src.zip -d ~/du_an_ws/src/

# Kiểm tra kết quả
ls ~/du_an_ws/src/
```

### 3.3. Kiểm tra trước khi build

```bash
cd ~/du_an_ws
source /opt/ros/humble/setup.bash

# Phải liệt kê được các package chính
colcon list
```

Nếu `colcon list` **không thấy** `thanxe_new_description` / `my_ur_control_cpp` / `ccs2_socket_detector` thì source code đã đặt sai vị trí — mỗi package phải nằm đúng tại `src/<tên_package>`.

> 📌 Phải **có đủ source code trong `src/` trước**, rồi mới chạy `rosdep`, vì `rosdep install --from-paths src` cần quét các file `package.xml` nằm trong đó.

---

## 4. Cài nhanh dependency bằng `rosdep` (khuyến nghị)

Thay vì gõ tay danh sách `apt install` ở trên, có thể dùng `rosdep` để cài toàn bộ dependency được khai báo trong các file `package.xml`.

### Cài đặt & khởi tạo rosdep (chỉ làm 1 lần)

```bash
sudo apt update
sudo apt install -y python3-rosdep
sudo rosdep init          # nếu báo "already exists" thì bỏ qua
rosdep update
```

### Cài dependency cho toàn workspace

> ℹ️ Phải hoàn thành **Mục 3** (đã có source code trong `src/`) trước khi chạy bước này.

```bash
cd ~/du_an_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Giải thích tham số:

| Tham số | Ý nghĩa |
|---|---|
| `--from-paths src` | Quét mọi package trong `src/` |
| `--ignore-src` | Bỏ qua package đã có sẵn trong `src/` (không cài lại từ apt) |
| `-r` | Tiếp tục chạy dù có key không resolve được (không dừng giữa chừng) |
| `-y` | Tự động trả lời Yes |

### Nếu vẫn thiếu gói

Trường hợp hiếm (rosdep phiên bản cũ không có key), cài bù bằng apt:

```bash
rosdep install --from-paths src --ignore-src -r -y || true

# Bù các gói mà rosdep không cover
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-nav2-mppi-controller \
  ros-humble-nav2-smac-planner \
  ros-humble-ur-description \
  ros-humble-tf2-tools \
  libeigen3-dev \
  libopencv-dev
```

### Bỏ qua key không resolve được

Nếu rosdep báo một key không tồn tại, dùng `--skip-keys` để đánh dấu là "đã tự cài tay":

```bash
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "tên_key_lỗi"
```

### Xem trước rosdep sẽ cài gì (không cài thật)

```bash
rosdep install --from-paths src --ignore-src -r -y --simulate
# hoặc chỉ liệt kê key
rosdep install --from-paths src --ignore-src -r -y --print-keys-only
```

## 5. Build workspace

```bash
cd ~/du_an_ws
source /opt/ros/humble/setup.bash

# (khuyến nghị) Cài dependency tự động trước khi build — xem Mục 4
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

Build từng package nếu muốn nhanh hơn:

```bash
# Build message package trước (dependency của my_ur_control_cpp)
colcon build --symlink-install --packages-select ccs2_socket_detector
colcon build --symlink-install --packages-select thanxe_new_description
colcon build --symlink-install --packages-up-to my_ur_control_cpp
source install/setup.bash
```

Sau mỗi lần mở terminal mới, luôn nhớ:

```bash
cd ~/du_an_ws && source install/setup.bash
```

---

## 6. Bỏ qua phần cánh tay UR10 (khuyến nghị)

Mặc định [`complete.launch.py`](thietkemoi/thanxe_new_description/launch/complete.launch.py:116) sẽ spawn cả `joint_trajectory_controller` cho cánh tay UR10. Vì bạn chỉ cần **thân xe**, không cần chỉnh lại.


## 7. Chạy mô phỏng

Cần mở **2 terminal** (đã `source` workspace ở cả hai).

### 🖥️ Terminal 1 — Khởi động Nav2 (AMCL + costmap + planner)

Thay `/home/loc/du_an_ws` bằng đường dẫn workspace **trên máy của bạn**, hoặc dùng biến `$HOME` cho gọn:

```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=True \
  map:=$HOME/du_an_ws/src/thietkemoi/thanxe_new_description/maps/my_vinfast_map.yaml \
  params_file:=$HOME/du_an_ws/src/thietkemoi/thanxe_new_description/config/nav2_params.yaml
```

> ⚠️ **Bắt buộc phải có `map:=`** — vì [`nav2_params.yaml`](thietkemoi/thanxe_new_description/config/nav2_params.yaml:308) hard-code `yaml_filename` của máy tác giả; tham số `map:=` sẽ override nó.

> ℹ️ Đợi Nav2 khởi động xong (thấy log `Timed out waiting for transform...` biến mất) rồi mới sang Terminal 2.

### 🖥️ Terminal 2 — Khởi động Gazebo + xe + RViz

**Máy dùng GPU NVIDIA (Optimus / PRIME):**

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  ros2 launch thanxe_new_description complete.launch.py
```

**Máy dùng GPU Intel / AMD / không có GPU rời:**

```bash
ros2 launch thanxe_new_description complete.launch.py
```

Launch file này sẽ tự động:
1. Nạp thế giới [`vinfast_map.world`](thietkemoi/thanxe_new_description/worlds/vinfast_map.world)
2. Spawn xe `my_mobile_manipulator` tại gốc toạ độ
3. Nạp `joint_state_broadcaster`, `steering_controller`, `drive_controller`
4. Spawn 3 trạm sạc `tram_sac_VF`
5. Chạy node `swerve_drive` và mở RViz với cấu hình [`SLAM.rviz`](thietkemoi/thanxe_new_description/config/SLAM.rviz)

### 🖥️ Terminal 3 — Gửi điểm đích (tuỳ chọn)

```bash
ros2 run my_ur_control_cpp goal_sender
```
Cách khác là trong RViz, dùng nút **2D Goal Pose** để gửi điểm đích cho xe.

---

## 8. Điều khiển thủ công bằng bàn phím (Teleop Keyboard)

Xe Swerve nhận lệnh vận tốc trực tiếp trên topic **`/cmd_vel_nav`** (xem [`swerve_drive.cpp`](my_ur_control_cpp/src/swerve_drive.cpp:51)), **KHÔNG phải `/cmd_vel`**.

> ⚠️ **Cực kỳ quan trọng**: `teleop_twist_keyboard` mặc định publish lên `/cmd_vel`. Nếu không remap sang `/cmd_vel_nav`, xe sẽ **không nhận lệnh** và bạn tưởng là teleop hỏng, nếu bấm 1 lần mà xe chưa chạy thì bạn hãy thử bấm thêm vài lần với cùng 1 phím.

### 8.1. Cài đặt

```bash
sudo apt install -y ros-humble-teleop-twist-keyboard
```

### 8.2. Chạy teleop (Terminal 3)

```bash
cd ~/du_an_ws
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_nav
```


> 💡 Chạy teleop **sau khi** Gazebo + xe đã lên (Terminal 2). Không bắt buộc phải chạy Nav2 (Terminal 1) — vì teleop publish thẳng vào topic mà `swerve_drive` đang subscribe.

### 8.3. Phím điều khiển

| Phím | Tác dụng |
|---|---|
| `i` | Tiến |
| `,` | Lùi |
| `j` | Xoay trái (tại chỗ) |
| `l` | Xoay phải (tại chỗ) |
| `u` / `o` | Tiến + xoay trái / phải |
| `m` / `.` | Lùi + xoay trái / phải |
| `k` | **Dừng** |
| `q` / `z` | Tăng / giảm vận tốc tối đa |
| `w` / `x` | Tăng / giảm vận tốc tuyến tính |
| `e` / `c` | Tăng / giảm vận tốc xoay |

> ⚠️ Teleop gửi lệnh **liên tục** và giữ lệnh cuối cùng. Khi muốn dừng hẳn, **bấm `k`** — đừng chỉ nhả phím.

### 8.4. Đi ngang (strafe) — đặc thù xe Swerve

`teleop_twist_keyboard` **không có phím đi ngang**, trong khi xe bạn là holonomic (đi ngang được). Muốn test strafe, publish trực tiếp:

```bash
# Đi ngang sang trái (trục y) với vận tốc 0.3 m/s
ros2 topic pub /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

> 💡 Xe đi ngang được vì [`swerve_drive.cpp`](my_ur_control_cpp/src/swerve_drive.cpp:139) dùng cả `linear.x` **và** `linear.y` để tính động học 4 bánh.

### 8.5. Kiểm tra teleop có tới xe chưa

```bash
# Xem có ai đang publish vào /cmd_vel_nav không
ros2 topic info /cmd_vel_nav -v

# Xem nội dung lệnh đang gửi
ros2 topic echo /cmd_vel_nav
```
---

## 9. Kiểm tra nhanh sau khi chạy

```bash
# Xem các topic đang hoạt động
ros2 topic list

# Kiểm tra odom
ros2 topic echo /odom --once

# Kiểm tra lidar
ros2 topic hz /scan

# Kiểm tra TF tree (map → odom → base_footprint → laser_frame)
ros2 run tf2_tools view_frames

# Xem danh sách controller
ros2 control list_controllers

# Gửi lệnh di chuyển thủ công
ros2 topic pub /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}" -r 10
```

**Frame & topic quan trọng:**

| Loại | Tên |
|---|---|
| Frame bản đồ | `map` |
| Frame odometry | `odom` |
| Frame robot (Nav2) | `base_footprint` |
| Frame lidar | `laser_frame` |
| Topic velocity (vào swerve) | `/cmd_vel_nav` |
| Topic điều khiển lái | `/steering_controller/commands` |
| Topic điều khiển bánh | `/drive_controller/commands` |
| Topic lidar | `/scan` |

---

## 10. Xử lý sự cố (Troubleshooting)

| Triệu chứng | Nguyên nhân & cách khắc phục |
|---|---|
| Không spawn được `tram_sac_VF` / báo `file not found` | Workspace không nằm ở `~/du_an_ws`. Sửa `external_models_path` trong [`complete.launch.py`](thietkemoi/thanxe_new_description/launch/complete.launch.py:119) |
| Xe không di chuyển dù Nav2 gửi plan | Kiểm tra `ros2 topic hz /cmd_vel_nav` có dữ liệu không, và `ros2 control list_controllers` xem `steering_controller` / `drive_controller` có `active` không |
| `Timed out waiting for transform from base_footprint to map` | Chưa chạy đủ 2 terminal, hoặc AMCL chưa nhận `/scan`. Kiểm tra `ros2 topic echo /scan --once` |
| RViz/Gazebo giật, màn hình đen, hoặc crash | Máy có NVIDIA Optimus cần thêm biến offload (xem Terminal 2). Hoặc tắt `use_sim_time` không đúng |
| Build lỗi `ccs2_socket_detector not found` | Phải build package message **trước**: `colcon build --packages-select ccs2_socket_detector` |
| Build lỗi `ur_description not found` | Cài `sudo apt install ros-humble-ur-description` hoặc build [`Universal_Robots_ROS2_Description`](Universal_Robots_ROS2_Description/package.xml) |
| Lỗi `JointGroupPositionController` không tìm thấy | Thiếu `ros-humble-ros2-controllers`. Cài lại theo Mục 4 |
| Gazebo báo không tìm thấy world / model | Chạy launch từ workspace đã `source install/setup.bash` |
| `rosdep: command not found` | Chưa cài: `sudo apt install -y python3-rosdep && sudo rosdep init && rosdep update` |
| `rosdep update` lỗi SSL / timeout | Do mạng chặn. Thử `sudo rosdep init` lại, hoặc dùng proxy, hoặc tải file rosdep theo hướng dẫn: http://wiki.ros.org/rosdep |
| `ERROR: the following packages/stacks could not have their rosdep keys resolved` | Key không tồn tại trong rosdep. Thêm `--skip-keys "..."` hoặc cài tay — xem Mục 4 |
| Chạy `rosdep install` xong vẫn thiếu Nav2 | Do `package.xml` chưa khai báo Nav2. Bổ sung như Mục 4 rồi chạy lại |
| Teleop bấm phím mà xe không chạy | Chưa remap topic. Thêm `-r cmd_vel:=/cmd_vel_nav` — teleop mặc định gửi `/cmd_vel`, xe lắng nghe `/cmd_vel_nav` |
| Xe chạy mãi không dừng sau khi nhả phím | `teleop_twist_keyboard` giữ lệnh cuối. Bấm **`k`** để phát lệnh dừng |
| Teleop chạy nhưng Nav2 tranh quyền điều khiển | Vừa gửi goal trong RViz vừa bấm teleop sẽ xung đột. Chỉ dùng một nguồn lệnh tại một thời điểm |
| Bấm **Shift** (holonomic) xe chạy loạn | Hành vi đã biết — xem phân tích đầy đủ ở Mục 8.6 và cách đi êm ở Mục 8.7 |
| `map_server` báo `file not found: /home/loc/new_design_ws/...` | [`nav2_params.yaml`](thietkemoi/thanxe_new_description/config/nav2_params.yaml:308) còn `yaml_filename` hard-code của máy tác giả. Truyền `map:=` khi launch (Mục 7) hoặc sửa theo Mục 9.0 |
| Vừa sửa `nav2_params.yaml` xong mà map vẫn cũ | YAML dùng đường dẫn hard-code trỏ workspace khác. Kiểm tra `ros2 param get /map_server yaml_filename` (Mục 9.0) |

---

## 11. Ghi chú kỹ thuật phần thân xe

- **Cấu hình bánh Swerve**: chiều dài cơ sở `L = 1.476 m`, chiều rộng `W = 0.510 m`, bán kính bánh `0.125 m` — khai báo trong [`swerve_drive.cpp`](my_ur_control_cpp/src/swerve_drive.cpp:20).
- **4 bánh lái độc lập** lần lượt: trước-trái, trước-phải, sau-trái, sau-phải.
- **Odometry** được node swerve tự tính và publish lên `/odom` + broadcast TF `odom → base_footprint`.
- **Bản đồ**: [`my_vinfast_map.yaml`](thietkemoi/thanxe_new_description/maps/my_vinfast_map.yaml) — độ phân giải `0.05 m/pixel`, gốc `[-15.1, -15.1]`.
- **Motion model AMCL**: `OmniMotionModel` (vì xe holonomic, không phải differential) — xem [`nav2_params.yaml`](thietkemoi/thanxe_new_description/config/nav2_params.yaml:15).
- **Footprint costmap**: `[[0.9, 0.425], [0.9, -0.425], [-0.9, -0.425], [-0.9, 0.425]]`.
- **Động học swerve drive**: Vì đây là phiên bản mô phỏng nên chưa có động học thuận (lấy dữ liệu từ cảm biến), Odometry trong hàm update_odometry() là dạng vòng hở. 

---

## 12. Chạy không dùng Gazebo (chỉ xem model)

Nếu chỉ muốn xem model xe trong RViz mà không cần mô phỏng:

```bash
ros2 launch thanxe_new_description display.launch.py
```

Có thể tắt GUI joint state:

```bash
ros2 launch thanxe_new_description display.launch.py gui:=False
```
