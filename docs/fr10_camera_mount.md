# Camera căn tâm mặt bích FR10

Cấu hình chung: `src/vehicle_description/config/fr10_camera_mount.yaml` và
`urdf/fr10_wrist.xacro`. Đây là bố trí gá đề xuất trong mô hình, chưa phải
ngoại chuẩn camera đo trên phần cứng.

- Tâm thân camera theo CAD nằm trên trục mặt bích: X=0, Y=0 trong
  `plate_frame`. Camera không dịch theo offset X/Y của đầu cắm Tool 1.
- `camera_support_frame` gắn trực tiếp vào `plate_frame`, giữ góc clocking
  136,84° để hướng camera song song trục cắm. Nhánh camera vẫn ở phía mặt
  bích của cảm biến lực, không đưa tải camera vào nhánh đầu cắm.
- Trục nhìn +Z của `camera_depth_optical_frame` song song và cùng chiều +Z
  của `charging_tcp`. Chiều ngang thân D435 theo X đầu cắm, chiều trên theo Y.
- Mặt sau camera cách mặt bích 30 mm theo Z. Thân camera trải từ Z=30 mm
  đến Z=55,0547 mm. Đây là gá sát mặt bích, không phải vị trí gần mũi cắm.
- Căn tâm theo hộp bao mesh D435 thật, có xét cả origin của visual và
  offset macro camera; không nhầm gốc `camera_link` với tâm thân camera.
- Yêu cầu tâm mặt bích thay thế bố trí trước đó dịch camera xuống theo
  hệ đầu cắm. Không còn áp dụng khe hở 10 mm/lùi 20 mm theo Tool 1.
- Hai trụ, tấm sau camera và tấm gá mặt bích được bố trí theo camera mới.
  Các giá đỡ vẫn là box mô phỏng có collision/inertia, chưa phải CAD gia công.

Không thay số liệu Tool 0 → Tool 1 trong `fr10_tool.yaml`. Vị trí mới và
các khung quang học được dùng chung cho Gazebo, RViz, MuJoCo và pipeline
nhận dạng. Sau khi đổi vị trí camera thật cần đo lại ngoại chuẩn tương ứng.
Các báo cáo lực với giá đỡ/camera cũ không xác nhận clearance của mô hình mới.

Build bản runtime mới, rồi đóng/mở lại launch để nhận mô hình:

```bash
cd /home/tinnt/Vinuni
bash src/vehicle_bringup/scripts/build_workspace.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Các kiểm tra mô hình xác nhận tâm thân camera trên trục mặt bích, hướng nhìn,
khoảng cách với đầu cắm và chân trụ nằm trên tấm gá, ở cả profile legacy/matched và
tay độc lập/gắn xe. Thử cắm phải chạy lại với mô hình mới để đánh giá
va chạm với cổng/giá đỡ bên ngoài.
