# Tái sử dụng FR10 gốc

FR10 trên xe nạp trực tiếp fairino_description/urdf/fairino10_v6.urdf qua adapter fr10_arm.urdf. Vendor chỉ bổ sung argument fr10_root_link (mặc định base_link); xe chọn fr10_base_link để không trùng tên. Không thay sáu joint, inertia, collision, mesh hoặc controller. Cụm đầu sạc/D435 và gá tiếp tục nằm riêng tại vehicle_description/urdf/fr10_wrist.xacro, fixed dưới plate_frame.

Kiểm tra: build 4 package đạt; Xacro và check_urdf đạt; so sánh từng link/joint với bản trước đạt. Chưa xác nhận hết rung trong Gazebo: lần thay nguồn này không đổi động lực học. Demo vendor mặc định dùng mock hardware; Gazebo vendor có world_to_base cố định, khác xe tự do trên bánh. Không cố định xe vào world để che hiện tượng.
