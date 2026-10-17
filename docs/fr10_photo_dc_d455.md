# Cụm CCS2 DC và D455 theo ảnh

Chọn `wrist_layout:=photo_dc_d455` để xem cụm gá theo ảnh tham chiếu:
đầu CCS2 DC ở trên, D455 ở dưới, cùng đường tâm đứng; giá ôm màu trắng.
Đầu đực có bảy tiếp điểm Type 2 phía trên và hai chân DC lớn phía dưới.

Các kích thước sau là **tạm dựng cho mô phỏng**, chưa đo trên cụm gá thật
và không phải bản vẽ chế tạo hay hình học CCS2 đã kiểm chuẩn:

| Đại lượng trong hệ mặt bích `plate_frame` | Giá trị |
| --- | --- |
| Tâm thân D455, X/Y | 0 / 0 mm |
| Mặt sau D455, Z | 15 mm |
| Mặt trước D455, Z | khoảng 41 mm |
| TCP đầu cắm, X/Y/Z | 0 / 95 / 185 mm |
| Hướng nhìn camera và hướng cắm | cùng +Z mặt bích |

X/Y nói đến **tâm vỏ camera**, không phải tâm quang học của từng cảm biến.
Như vậy camera nằm dưới đầu cắm và nhô ra phía trước mặt bích, nhưng
vẫn lùi sau mũi đầu cắm. D455 dùng mesh của `realsense2_description`;
vỏ đầu cắm và giá đỡ được dựng riêng theo tỷ lệ ảnh.

Mở riêng mô hình trong RViz, không chạy driver hay controller:

```bash
cd /home/tinnt/Vinuni
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=225 ROS_LOCALHOST_ONLY=1 \
  ros2 launch vehicle_description photo_wrist.launch.py
```

Mở Gazebo và RViz với cùng mô hình:

```bash
ROS_DOMAIN_ID=225 ROS_LOCALHOST_ONLY=1 \
  ros2 launch vehicle_bringup bringup.launch.py \
  use_arm:=true use_vehicle:=false use_gazebo:=true \
  use_force:=false wrist_layout:=photo_dc_d455 \
  use_arm_camera:=false gui:=true rviz:=true gazebo_port:=11358
```

`use_arm_camera:=false` chỉ tắt phát ảnh mô phỏng; vỏ camera vẫn hiển thị.
Không mở hai launch trên cùng lúc trên cùng domain. Muốn xem bằng MuJoCo
chỉ có hình học và FK, không phát dữ liệu ROS hay lực tiếp xúc:

```bash
/usr/bin/python3 src/vehicle_description/launch/preview_photo_wrist.py --show
```

Mặc định `wrist_layout:=calibrated` vẫn giữ Tool 1 đã đo ở
`config/fr10_tool.yaml`: `[55.140, -46.496, 215.297] mm`, góc XYZ
`[0, 0, 136.840] độ`. Bản theo ảnh không sửa số đo này. Launch từ chối
dùng bản theo ảnh với robot thật hoặc `use_force:=true`: fixture tiếp xúc
cũ có chân đực, chưa phải ổ cái phù hợp đầu đực mới. Cần dựng cặp tiếp xúc
tương ứng trước khi đánh giá lực/cắm bằng cụm gá mới.

Mô hình: `urdf/fr10_photo_wrist.xacro`; bảng kích thước danh nghĩa và tiếp
điểm: `config/fr10_photo_layout.yaml`. Mesh giá ôm được tái tạo bằng
`launch/generate_photo_mount.py`. Khi sửa kích thước gá, cần cập nhật các
thành phần tương ứng trong Xacro và tái tạo mesh rồi chạy lại kiểm tra
`test/test_fr10_photo_layout.py`.
