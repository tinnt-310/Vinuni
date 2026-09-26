# Trạng thái mô hình hiện tại

Cập nhật lỗi mở trống: launch xe dùng cổng Gazebo riêng 11355 (đổi bằng
`gazebo_port:=...`), kiểm tra cổng bận trước khi mở, và thoát nếu server chết.
Đã kiểm tra lại enable_arm=false: spawn thành công, 3 controller active,
TF odom -> Wheel1_1 hợp lệ. Build package đạt.
`fairino10_v6_moveit2_config demo.launch.py` là MoveIt độc lập với mô hình
end-effector cũ của package đó, không phải launch xe ghép; chưa chuyển cấu hình
MoveIt độc lập sang model xe. Không chạy đồng thời hai launch trên cùng ROS domain.

Chưa hoàn tất nghiệm thu chế độ có tay: controller đã active, action thành công,
nhưng mô phỏng vật lý có tay vẫn bị mất ổn định. Không dùng action success làm
bằng chứng xe đứng ổn định. Thử tăng controller lên 1000 Hz và đổi độ cao spawn
không giải quyết được nên đã trả về cấu hình 100 Hz / spawn 0.1 m.

## Chạy

```bash
cd /home/tin/Vinuni
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch thanxe_new_description complete.launch.py enable_arm:=false
```

Đổi `false` thành `true` để nạp FR10 + D435 + đầu sạc và controller tay.
Dừng launch trước khi đổi chế độ. Hai chế độ đều có D455 thân xe và LiDAR.

## Đã kiểm tra

- Bánh Ø420 mm, rộng 100 mm; wheelbase .8 m, track .7 m.
- Geometry chung đọc từ config/vehicle_geometry.yaml; swerve/odom dùng radius .21.
- Thân kín dài 1.271740663 m, rộng 1.171740663 m, đáy .520 m, mái 1.000 m.
- D455 ở ngoài mặt trước: tâm thân (.660870331,0,.4), mặt sau cách vỏ .012 m.
- enable_arm=false: 8 control joints, 3 controller active; bốn kiểu chạy đã đạt.
- enable_arm=true: 14 control joints, 4 controller active; còn lỗi ổn định vật lý.
- Fixed wrist3 -> plate: xyz=(0,0,.106), rpy=(0,0,pi).
- Tư thế khớp cũ giữ nguyên [0,-pi/2,0,-pi/2,0,0], không bù bằng joint trước.
- Camera và đầu sạc nối với plate bằng chuỗi fixed joints; charger=(0,.09,.09).
- Kiểm tra geometry cho thấy camera dưới/charger trên ở home, optical image-down
  hướng xuống. Kiểm thử ảnh thực và quay rigid theo j6 chưa hoàn tất.
- G2 chưa có CAD xác thực: cylinder Ø73×43 mm được ghi rõ là approximation.

Lỗi parser robot_description đã sửa bằng loại comment XML khỏi dữ liệu runtime.
Không thay đổi XML joint động học FR10 gốc. Báo cáo FR10_ON_VEHICLE.md cũ mô tả
giai đoạn trước; thông số plate/home/chassis mới ưu tiên file trạng thái này và YAML.
