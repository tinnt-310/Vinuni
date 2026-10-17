# RB Charging V3 + FR10 simulation

Workspace ROS 2 Humble/Gazebo Classic cho xe Swerve RB V3, tay FR10, LiDAR CAD,
camera cổ tay và đầu sạc CCS2. Mô hình chuẩn nằm trong `vehicle_description`.

Lệnh ngắn khởi động từng cấu hình, điều khiển tay/xe và đọc pose:
[README_LENH.md](README_LENH.md).

Pipeline mới nhận một pose miệng cổng, tự tìm đường đến preinsert rồi chèn theo
lực tới HOLD. Kiến trúc, phạm vi mô phỏng và kết quả nghiệm thu hiện tại:
[FR10 insertion pipeline](../../../../docs/fr10_insertion_pipeline.md) và
[progress ledger](../../../../PROGRESS_LEDGER.md).

Admittance nội bộ và hiệu chỉnh pose có sai số đang được nghiệm thu tại
work log (local: `test_results/fr10_admittance_pose_error_20261003/WORK_LOG.md`).
API C++ dùng lại, FT Fairino raw/reference/active và tare với payload gắn:
[force sensor handoff](../../../../docs/fr10_force_sensor_handoff.md). Giữ một MotionExecutor
phát joint command; profile mô phỏng mới chưa được mở cho robot thật.

## Cài dependency một lần

```bash
cd /duong-dan/toi/Vinuni
./tools/install_simulation_dependencies.sh
```

## Build portable

```bash
cd /duong-dan/toi/Vinuni
./tools/build_simulation.sh
source /opt/ros/humble/setup.bash
source ros2_current/install/setup.bash
```

Mỗi build tạo một runtime riêng tại `.colcon/releases/`. Symlink tương đối
`ros2_current` là entrypoint chung; `build`, `install` và `log` cũng trỏ vào
runtime đã build và test thành công gần nhất. Build lỗi không đổi entrypoint;
runtime cũ không bị ghi đè khi Gazebo còn dùng thư viện. Install giữ bản sao
config/launch cùng revision với binary. Sau khi chép workspace sang máy khác,
chạy lại `./tools/build_simulation.sh` vì colcon/ament lưu prefix tuyệt đối.
`--clean` vẫn được hỗ trợ; mỗi build vốn đã sạch, không xóa runtime cũ đang chạy.

## Chạy bằng một ROS 2 launch

```bash
ros2 launch vehicle_bringup bringup.launch.py use_arm:=true use_vehicle:=true use_gazebo:=true
```

Profile force legacy chạy Gazebo standalone:

```bash
source tools/source_simulation.sh
ros2 launch vehicle_bringup bringup.launch.py \
  use_arm:=true use_vehicle:=false use_gazebo:=true use_force:=true
```

Terminal gửi goal cũng phải chạy `source tools/source_simulation.sh` trước;
sau đó gửi `fr10_motion_interfaces/action/Insert` tới `/fr10_force/insert`
(chi tiết và goal mẫu nằm trong `src/vehicle_bringup/README.md`). Matched CAD
dùng `station:=front_mid`, có parking khi bật vehicle và đang commissioning;
profile mặc định vẫn khóa chèn tự động. Không dùng profile mô phỏng cho tủ thật.

Ba cờ mặc định đều là `true`, nên chạy tay FR10 gắn trên xe trong Gazebo chỉ cần:

```bash
ros2 launch vehicle_bringup bringup.launch.py
```

Chọn thành phần bằng cùng launch, không cần tool chạy riêng:

```bash
# Chỉ tay FR10
ros2 launch vehicle_bringup bringup.launch.py use_vehicle:=false

# Chỉ xe
ros2 launch vehicle_bringup bringup.launch.py use_arm:=false
```

Launch tự quản lý Gazebo, RViz, spawn model và controller. Tay robot mặc định dùng
`MotionExecutor → JointGroupPositionController → ros2_control`, không dùng JTC
hoặc MoveIt demo song song. Force mặc định tắt, không tự chạy insertion.

Thêm `gui:=false rviz:=false` nếu cần chạy headless; `gazebo_port:=11411` nếu
port mặc định đang được dùng. `use_gazebo:=false` chọn hardware/SimMachine native
FAIRINO, không tự fallback fake/MuJoCo. Khi đó phải tắt xe (`use_vehicle:=false`)
vì PLC thật chưa commissioning; không tự bật Auto/enable/reset alarm.

Chi tiết launch: [vehicle_bringup](../../../../src/vehicle_bringup/README.md).
Chi tiết hình học: [vehicle_description](../../../../src/vehicle_description/README.md).
