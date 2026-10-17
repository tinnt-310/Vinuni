# Kiểm tra các lệnh chạy trong Gazebo — 14/10/2026

Hướng dẫn vận hành: [README_COMMANDS.md](../vehicle_bringup/README_COMMANDS.md).

Đã chạy các launch thực tế trên ROS 2 Humble, Gazebo Classic 11, với
`gui:=false rviz:=false`. Robot được spawn, controller và dữ liệu được kiểm tra
qua ROS; đây không chỉ là kiểm tra cú pháp launch. Runtime được source từ
`/home/tinnt/Vinuni/install/setup.bash`, trỏ tới
`.colcon/releases/rb_v3_humble_20261014T061006Z.9pjc3e`.
Package `vehicle_bringup` đã được build/install lại sau bản sửa trong đợt này.

## Kết quả vận hành

| Cấu hình | Kết quả | Bằng chứng |
|---|---|---|
| Chỉ xe | Đạt: controller active, clock, joints, scan, odom, TF; Twist tiến/ngang có chuyển động | [Log](validation/commands_20261014/vehicle/runtime.log), [số đo](validation/commands_20261014/vehicle/summary.json) |
| Chỉ tay FR10 | Đạt khởi động và feedback TCP/TF; còn lỗi shutdown bên dưới | [Log](validation/commands_20261014/arm/runtime.log) |
| Xe kèm tay | Đạt khởi động: ba controller active, laser/odom, TCP và TF | [Log](validation/commands_20261014/combined/runtime.log) |
| Force tay độc lập | Đạt khởi động và action server; còn lỗi shutdown | [Log](validation/commands_20261014/force_arm/runtime.log) |
| Force tay trên xe, `station:=front_mid` | Đạt khởi động sau bản sửa; còn log `hold_joints` và lỗi shutdown | [Log](validation/commands_20261014/force_mounted/runtime.log) |
| SLAM mapping | Đạt: nhận occupancy map có ô trống từ SLAM Toolbox | [Log](validation/commands_20261014/mapping.log) |
| Lưu map qua `save_map.launch.py` | Đạt: tạo YAML/PGM và dùng được cho AMCL | [Log](validation/commands_20261014/save_map.log), [YAML](validation/commands_20261014/acceptance_map.yaml), [PGM](validation/commands_20261014/acceptance_map.pgm) |
| AMCL-only | Đạt: AMCL + map server active, có pose và TF `map → base_footprint`; không có SLAM/Nav2 controller | [Log](validation/commands_20261014/localization.log) |
| Nav2 | Đạt: chín lifecycle node active, goal cách pose ban đầu 0,4 m trả `SUCCEEDED`, nhận TwistStamped tại `/cmd_vel_nav` | [Log](validation/commands_20261014/navigation.log) |

Ở bài thử xe, lệnh Twist 0,1 m/s tạo độ dịch chuyển odometry **0,1931 m theo
hướng tiến** và **0,2226 m theo hướng ngang** sau các khoảng chạy/dừng của bài thử.
Đây là số đo odometry, chưa đối chiếu ground truth để đánh giá sai số động học.

Các profile dùng ROS domain 180–184 và cổng Gazebo riêng; bài navigation dùng
domain 186. Thử nghiệm navigation gồm mapping → lưu map → khởi động lại AMCL-only
→ khởi động lại Nav2, hoàn tất trong **30,55 giây**. Lần chạy profile đầu tiên có
4 đạt/1 lỗi; sau khi sửa, chạy lại cả hai profile force đạt **2/2 trong 27,12 giây**.
Tổng hợp kết quả mới nhất của năm profile: [profiles.json](validation/commands_20261014/profiles.json).

## Các lỗi đã sửa trong lúc chạy

- Bringup mounted trước đây luôn ghi đè tốc độ rút ban đầu, trong khi profile
  draft giữ khoảng rút bằng 0. Controller từ chối cấu hình với thông báo
  `loaded retract speed/distance must both be positive or disabled`.
  Bringup hiện chỉ áp dụng tốc độ của cấu hình xe khi profile đã bật cả cặp
  tốc độ/khoảng rút. Profile draft giữ cặp tắt; profile commissioning giữ khoảng
  rút đã cấu hình. [Log trước sửa](validation/commands_20261014/force_mounted_before_fix.log).
- `force_fixture_publisher.py` gọi shutdown ROS lần thứ hai khi nhận Ctrl-C,
  gây traceback. Đã xử lý context đã shutdown; lần chạy lại không còn traceback này.

Kiểm tra hồi quy cấu hình controller: **12 passed**. CTest của toàn package
`vehicle_bringup`: **10/10 nhóm kiểm tra đạt**, bao gồm nhóm 12 kiểm tra trên.
[Log CTest](validation/commands_20261014/bringup_ctest.log),
[log build/install](validation/commands_20261014/bringup_install.log).

## Các giới hạn còn lại

- Khi Ctrl-C tắt cả Gazebo và controller, các lần chạy tay độc lập và hai fixture
  force có log `Shutdown braking deadline expired; completion unconfirmed`;
  process pose controller thoát mã 1. Parent launch vẫn trả 0, nên kết quả
  khởi động đạt trong JSON **không chứng nhận shutdown sạch**. Log cho thấy Gazebo
  kết thúc trước khi controller xác nhận dừng; cần xử lý thứ tự shutdown riêng.
- Mounted force còn báo tham số `hold_joints` được khai báo trùng trong
  `gazebo_ros2_control`. Controller vẫn active và feedback/TCP/TF vẫn có trong
  bài thử này; lỗi log này chưa được xử lý.
- Không gửi goal cắm/rút hoặc goal chuyển động cho tay. Force chỉ được kiểm tra
  khởi động và action server. Profile matched mặc định vẫn là draft, force chưa
  được bật; không suy ra chất lượng tiếp xúc từ việc action server tồn tại.
- Các bài trong bảng trên dùng Gazebo headless; đợt bổ sung bên dưới đã kiểm tra
  GUI mapping. Chưa kiểm tra đặt goal bằng chuột. Map lưu từ vùng quan sát tại vị trí spawn, chưa quét toàn bộ nhà xưởng.
  Goal Nav2 ngắn chưa kiểm chứng đường dài, vật cản động hay tay vươn khi xe chạy.

## Kiểm tra bổ sung với Conda

Sau khi người dùng chạy mapping từ terminal Conda, đã tái hiện lỗi
`ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`.
`ros2` sử dụng `/usr/bin/python3` 3.10.12, nhưng các script có shebang
`/usr/bin/env python3` lại chọn Miniconda 3.14.7 từ `PATH`, không tương thích với
extension rclpy đang cài. Không phải lỗi thiếu map hoặc thiếu package SLAM.

Đã sửa launch để các script Python dùng `sys.executable` của tiến trình launch:
hai gate khởi động, các node `spawn_entity.py` của Gazebo, publisher fixture và
`cmd_vel_normalizer.py` trong cả launch swerve chính và launch tương thích cũ.
Đã cài các launch mới vào runtime đang dùng, không thay đổi cài đặt Conda của người dùng.

Kiểm chứng với **Conda `base` đang active**, `python3 --version` vẫn là **3.14.7**:

- **5 kiểm tra hồi quy** đặt một Python không tương thích lên đầu PATH, chạy
  command được launch phân giải và yêu cầu import rclpy thành công. Tất cả đạt;
  bộ CTest bringup hiện có **11/11 nhóm đạt**.
- Chu trình Gazebo headless SLAM → lưu map → AMCL-only → Nav2, ROS domain 187,
  **đạt trong 24,55 giây**, gồm goal 0,4 m trả `SUCCEEDED` và lệnh TwistStamped.
- Chạy lệnh mapping với **GUI Gazebo và RViz bật mặc định**, domain 188/cổng riêng.
  Hai cửa sổ mở thành công; SLAM xuất map **169 × 242 ô, độ phân giải 0,05 m**.
  RViz hiển thị map và `Global Status: Ok`; hai gate khởi động thoát sạch,
  không còn traceback rclpy.
- Sau Ctrl-C của bài GUI, RViz thoát với **SIGSEGV (mã -11)**. Đây là lỗi lúc
  shutdown còn lại; không coi bài GUI là kiểm chứng shutdown sạch. Gazebo,
  SLAM, robot_state_publisher và bộ chuyển vận tốc thoát bình thường.

Bằng chứng: [CTest](validation/conda_20261014/ctest.log),
[log mapping GUI](validation/conda_20261014/gui/runtime.log),
[kết quả GUI](validation/conda_20261014/gui/summary.json),
[ảnh RViz](validation/conda_20261014/gui/rviz.png),
[log SLAM](validation/conda_20261014/mapping.log),
[lưu map](validation/conda_20261014/save_map.log),
[AMCL-only](validation/conda_20261014/localization.log),
[Nav2](validation/conda_20261014/navigation.log).

## Chạy lại bài kiểm tra

Từ workspace root, sau khi source:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

RUN_GAZEBO_PROFILE_TESTS=1 python3 -m pytest -s -q \
  src/vehicle_bringup/test/integration/test_bringup_profiles_gazebo.py

NAVIGATION_TEST_DOMAIN=186 RUN_GAZEBO_NAVIGATION_TEST=1 python3 -m pytest -s -q \
  src/vehicle_bringup/test/integration/test_navigation_gazebo.py
```

Chạy hai bài lần lượt. Test tự khởi động bằng `ros2 launch`, ghi log/map vào
thư mục tạm của pytest và dừng các process do chính test tạo. Các chương trình
này nằm trong `vehicle_bringup/test/integration`, không phải công cụ khởi động
vận hành. Dùng lệnh trong README để vận hành bình thường.
