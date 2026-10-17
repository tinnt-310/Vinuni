# Hướng dẫn các lệnh chạy

Cụm gá theo ảnh mới: **CCS2 DC phía trên, D455 phía dưới tại tâm mặt bích**.
Chọn `wrist_layout:=photo_dc_d455`; kích thước tạm và lệnh Gazebo/RViz ở
[hướng dẫn cụm gá theo ảnh](docs/fr10_photo_dc_d455.md).
Các giá trị Tool 1/D435 bên dưới thuộc cấu hình mặc định `calibrated`.

Hướng dẫn vận hành thống nhất: [README_COMMANDS.md](src/vehicle_bringup/README_COMMANDS.md).

Bao gồm chuẩn bị terminal, build, xe/tay, SLAM, lưu map, AMCL, Nav2, tham số,
fixture force và chẩn đoán.

Tool 0 là tâm mặt bích; Tool 1 là tâm bề mặt trước đầu cắm sạc.
Cấu hình dùng chung duy nhất: [fr10_tool.yaml](src/vehicle_description/config/fr10_tool.yaml).
Tool 1 so với Tool 0: `[55.140, -46.496, 215.297] mm`, góc cố định XYZ
`[0, 0, 136.840] độ`. Gazebo, RViz/MoveIt và MuJoCo đọc cùng mô hình Xacro;
khởi động lại phiên mô phỏng sau khi build để nhận mô hình mới.

Camera D435 căn tâm mặt bích (tâm thân X=Y=0 trong `plate_frame`), hướng nhìn
song song hướng cắm; mặt sau camera cách mặt bích 30 mm.
[Bố trí camera và giá đỡ](docs/fr10_camera_mount.md).

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
[fr10_sdk_tests](src/hardware_interface/fr10_sdk_tests/README.md).

Thử riêng FT_Control kết hợp MoveL chậm trên bề mặt tiếp xúc:
[cấu hình, lệnh chạy và biểu đồ lực](src/hardware_interface/fr10_sdk_tests/README.md#thử-giữ-lực-fz-trong-khi-chạy-movel).

Hướng dẫn kiểm thử 6DOF mới: [profile, nhóm case, biểu đồ và tiêu chí đánh giá](docs/fr10_6dof_mujoco_test_guide.md).

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
lại tay/cổng trong MuJoCo nằm ở [hướng dẫn thử](src/vehicle_bringup/test/integration/README.md).

Miền 5 mm tính theo norm XYZ, 5° tính theo norm vector quay. Đây là profile
thử nghiệm; chưa chứng minh cắm được mọi sai lệch trong miền này. Xem
[kết quả và các giới hạn đã phát hiện](docs/fr10_mujoco_admittance_audit_20261016.md).

Kết quả tự chạy kiểm tra: [báo cáo mô phỏng](src/docs/command_simulation_validation.md).
