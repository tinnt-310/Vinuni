# Nhận diện cổng sạc

charger_detection/ là package chứa toàn bộ implementation detector, solvepnp và depth_heatmap. ccs2_socket_detector/ chỉ định nghĩa msg/SocketDetection.msg để giữ tương thích ROS; không chứa detector thứ hai. Không chạy nguồn snapshot tại my_ur_control_cpp.
