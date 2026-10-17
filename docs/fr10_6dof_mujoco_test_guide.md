# Kiểm thử force control 6DOF trong MuJoCo

Các lệnh này chạy controller ROS với backend MuJoCo, ROS domain 212 và
localhost. Runner tự tạo robot/cổng CAD, gửi một pose cổng, chạy preinsert →
force insertion → HOLD → rút ra, lưu feedback và dừng phiên thử.
Không cần mở Gazebo hoặc gửi thêm action/pose trong khi runner đang chạy.
Runner kiểm tra domain trước khi khởi động: nếu đã có nguồn `/clock`,
`/joint_states`, `/controller_status` hoặc dịch vụ cảnh cũ, chọn một domain
khác và dùng cùng domain đó cho viewer/biểu đồ. Không chạy hai plant cùng
domain: clock/joint trộn nhau có thể gây lỗi CAD và nhảy ngược thời gian.

## 1. Build đúng code mới và kiểm tra profile

```bash
cd /home/tinnt/Vinuni
bash src/vehicle_bringup/scripts/build_workspace.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=212
export ROS_LOCALHOST_ONLY=1
```

Nếu chỉ sửa C++, build trên sẽ lấy code mới. Nếu sửa M/D/K trong
`controller.yaml`, cần lưu ý các overlay MuJoCo có thể ghi đè chúng.
Mặc định `--mode passive` dùng `mujoco_passive_5mm_5deg.yaml`, kế thừa cả
M/D/K/giới hạn từ các profile gốc. Thứ tự sáu phần tử là X, Y, Z, Rx, Ry, Rz;
ba trục tịnh tiến dùng mét, ba trục quay dùng radian trong controller.

Để thử bộ tham số riêng, sao chép profile vào cùng thư mục config:

```bash
cp src/fr10_arm_control/fr10_pose_controller/config/mujoco_passive_5mm_5deg.yaml \
   src/fr10_arm_control/fr10_pose_controller/config/mujoco_6dof_user.yaml
```

Sửa/điền các khóa `force.admittance.mass`, `damping`, `stiffness`,
`drive_deadband`, `max_velocity`, `max_acceleration`, `max_correction` theo
bộ tham số bạn muốn đánh giá; giữ các cờ tìm kiếm tắt khi thử passive.
Đặt `force.goal.selected_axes: [true, true, true, true, true, true]` nếu cần
ghi rõ cả sáu trục. Không cần đặt lực/mô-men khác 0 trên cả sáu trục:
trong bài cắm này, lực đặt cuối thường là Fz=5 N, còn lực ngang và mô-men
hướng về 0. Pha locating có lực đặt thấp hơn pha seating.

Kiểm tra profile và case **trước khi chạy động lực học**:

```bash
/usr/bin/python3 src/vehicle_bringup/test/integration/run_fr10_capture_mujoco.py \
  --mode passive --case-group smoke --dry-run \
  --overlay src/fr10_arm_control/fr10_pose_controller/config/mujoco_6dof_user.yaml \
  --output test_results/fr10_6dof_plan_01
```

Mở `trial_plan.json` trong output để kiểm tra `resolved_profile_parameters`,
đặc biệt M/D/K, sáu `selected_axes`, deadband và correction limits. Dry-run
không khởi chạy ROS/physics. Thêm cùng `--overlay .../mujoco_6dof_user.yaml`
vào các lệnh chạy bên dưới nếu dùng tham số riêng. Bỏ `--overlay` để dùng
profile MuJoCo mặc định. Thư mục output của mỗi lần chạy phải mới.

## 2. Bắt đầu bằng hai ca nhỏ

```bash
/usr/bin/python3 src/vehicle_bringup/test/integration/run_fr10_capture_mujoco.py \
  --mode passive --case-group smoke --domain 212 --timeout 1200 \
  --output test_results/fr10_6dof_smoke_01
```

Gồm `aligned` (định vị đúng) và `combined_diagonal_0p5` (norm sai lệch vị trí
0,5 mm và góc 0,5°). Tham số `--timeout` là giới hạn **giây thực cho mỗi ca**;
deadline và tích phân trong controller vẫn dùng thời gian nguồn.
Không tăng lực để bù simulation chậm. Nếu `aligned` không qua, kiểm tra
startup/geometry/feedback trước khi chạy cả ma trận.

## 3. Mở biểu đồ sáu ô và hình robot

Đóng cửa sổ biểu đồ cũ nếu nó ở ROS domain 0. Mở terminal thứ hai:

```bash
cd /home/tinnt/Vinuni
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=212 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 \
  src/vehicle_bringup/scripts/fr10_robot_plot.py
```

Một cửa sổ, sáu ô: **Fx/Fy/Fz | Tx/Ty/Tz | X | Y | Z | vận tốc TCP**.
Đây là wrench cảm biến tại frame ghi trên hình và TCP từ feedback; vận tốc
là quãng đường TCP / thời gian thực trong cửa sổ khoảng 0,5 s. Có thể mở
trước runner; khi chuyển ca, hình chờ lại dữ liệu. Đóng hình/Ctrl+C lưu
`dashboard.png`, `wrench.csv`, `tcp.csv` vào đường dẫn in trên terminal.

Để xem lực đã lọc trong hệ lắp ghép, phase, độ sâu và RTF:

```bash
ROS_DOMAIN_ID=212 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 \
  src/vehicle_bringup/scripts/fr10_force_plot.py
```

Để xem robot/cổng trực tiếp, chờ `selected_station.json` của ca xuất hiện rồi:

```bash
ROS_DOMAIN_ID=212 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 \
  src/vehicle_bringup/test/integration/view_fr10_force_mujoco.py \
  --run-directory test_results/fr10_6dof_smoke_01/aligned --live --close-up
```

Các cửa sổ này chỉ quan sát. Viewer hiển thị joint feedback, không tự chạy
thêm một bản dynamics hay phát lệnh tới robot.
Viewer dùng mesh CAD `visual` của FR10, đầu cắm và D435; ẩn các mảnh
`collision` và tắt hiển thị convex hull. Phần thay đổi này chỉ nằm ở viewer,
không thay hình học hay tham số tính lực của ca thử. Mở lại viewer để nhận
giao diện CAD mới; không cần chạy lại bài cắm chỉ để đổi cách hiển thị.
Runner cũng in lệnh viewer với đúng domain và thư mục của từng ca.
Nếu đổi domain thành 221, phải đổi cả `ROS_DOMAIN_ID=221` ở viewer/biểu đồ.
Khi runner đã kết thúc, cửa sổ viewer còn mở không có nghĩa bài cắm vẫn chạy;
đọc `summary.json` để biết kết quả, hoặc chạy viewer không có `--live` để
xem lại joint feedback đã ghi.

## 4. Chạy từng nhóm sai lệch

| Nhóm | Các ca | Mục đích |
| --- | --- | --- |
| `smoke` | Đúng pose, đồng thời 0,5 mm / 0,5° | Kiểm tra pipeline ban đầu |
| `axes` | ±1 mm trên từng X/Y/Z; ±1° trên từng Rx/Ry/Rz, tổng 12 ca | Kiểm tra riêng từng DOF và hai dấu |
| `combined` | Sai lệch đồng thời norm 0,5; 1; 2; 3; 5 mm và độ | Kiểm tra tương tác ngang/nghiêng |
| `boundary` | ±5 mm từng X/Y/Z, ±5° từng Rx/Ry/Rz, hai ca kết hợp | Kiểm tra biên miền mong muốn, tổng 14 ca |

Chạy nhóm đơn trục sau khi smoke đã đạt:

```bash
/usr/bin/python3 src/vehicle_bringup/test/integration/run_fr10_capture_mujoco.py \
  --mode passive --case-group axes --domain 212 --timeout 1200 \
  --output test_results/fr10_6dof_axes_01
```

Đổi `--case-group axes` thành `combined` hoặc `boundary`, đồng thời đổi
output thành thư mục mới. Nhóm nhiều ca có thể chạy lâu; nên xem kết quả
từng ca trước khi tiếp tục. Có thể chọn riêng:

```bash
/usr/bin/python3 src/vehicle_bringup/test/integration/run_fr10_capture_mujoco.py \
  --mode passive --cases translation_y_plus1 tilt_rx_minus1 yaw_rz_plus1 \
  --domain 212 --timeout 1200 --output test_results/fr10_6dof_selected_01
```

Sai lệch đưa vào **pose quan sát**, trong hệ cổng thật; cổng vật lý không
bị di chuyển theo pose sai. Sai lệch tổng được giới hạn theo norm, không phải
cho mỗi thành phần độc lập lên 5. Ví dụ `combined_diagonal_5` dùng
Δp=(3,−4,0) mm và vector quay (3,−4,0)°, mỗi norm bằng 5.

Nếu passive bị tì ngoài vùng dẫn hướng, chạy đối chiếu cùng case bằng tìm
kiếm có giới hạn sẵn có; vẫn cùng một Admittance:

```bash
/usr/bin/python3 src/vehicle_bringup/test/integration/run_fr10_capture_mujoco.py \
  --mode search --cases translation_x_minus5 --domain 212 --timeout 1200 \
  --output test_results/fr10_6dof_search_01
```

`--mode search` mặc định dùng một profile khác, có cả gains khác. Vì vậy đây
là so sánh hai cấu hình hệ thống, không tự chứng minh tác động riêng của search.
Muốn so sánh riêng bật/tắt search, chuẩn bị hai YAML với **cùng M/D/K, lọc,
giới hạn và tốc độ**, chỉ khác cờ search/probe, rồi truyền `--overlay` tương ứng.
Runner từ chối profile bật tìm kiếm nhưng ghi nhãn passive, hoặc ngược lại.

## 5. Đọc kết quả và đánh giá

Trong mỗi output:

- `trial_plan.json`: cases, profile và toàn bộ tham số overlay đã resolve.
- `matrix_summary.json`: PASS/FAIL, thời gian thực, lực/mô-men thô đỉnh,
  tốc độ thực, RTF và sai lệch hình học của từng ca.
- `force_alignment.png`: Fz, độ sâu thật, lệch ngang thật, lệch góc thật, RTF.
- `wrench_components.png`: đủ Fx/Fy/Fz/Tx/Ty/Tz đã lọc trong hệ lắp ghép.
- Mỗi thư mục case: `summary.json`, `mujoco_overlay.yaml`,
  `insert_feedback.json`, `raw_wrench.json`, `physical_tcp.json`,
  `runtime.log` và manifest của model/binary/source đã chạy.

| Cần đánh giá | Dấu hiệu đạt / cần kiểm tra |
| --- | --- |
| Đáp ứng đúng sáu trục | Sai lệch X/Y và Rx/Ry/Rz giảm theo feedback thật; correction được thực thi, không chỉ đổi target |
| Lực và mô-men | Ghi cả đỉnh thô và mức ổn định; Fx/Fy và Tx/Ty/Tz giảm, không rung kéo dài |
| Độ sâu / fit | Phải qua kiểm tra hình học độc lập; action SUCCESS hoặc Fz tăng chưa đủ |
| Vận tốc / thời gian | So sánh tốc độ thực và RTF riêng; không đồng nhất tốc độ đặt mô phỏng với tốc độ nhìn thấy |
| Hoàn tất | Cắm đúng → HOLD đủ 5 giây nguồn → rút ra → dừng có xác nhận |

Các gate hiện tại: keyed fit ≤250 µm, độ sâu trong 500 µm, penetration MuJoCo
≤50 µm, không có contact robot ngoài dự kiến. Giới hạn bảo vệ lực/mô-men
50 N / 10 Nm là trần cấu hình, **không phải mục tiêu chất lượng**: một ca PASS
vẫn có thể có lực đỉnh quá lớn so với lực đặt 5 N. Hãy đặt tiêu chí tải/thời gian
đánh giá trước khi so sánh hai bộ gains, và lặp lại cùng case nhiều lần ở output mới.
Timeout vẫn được tính là một ca thất bại; chưa có summary thì chưa đủ bằng chứng.

Ở bản thử trước, passive 0,5 mm / 0,5° và 1 mm / 1° đã đạt, còn nhiều ca 5 mm /
5° chưa đạt. Đặc biệt lỗi Z có thể làm sai mốc độ sâu. Đây chỉ là regression
tham khảo; thay code/gains/tool/geometry phải đo lại, không dùng kết quả cũ
để xác nhận bộ điều khiển mới.
