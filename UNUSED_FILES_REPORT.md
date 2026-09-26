# UNUSED_FILES_REPORT

Quét 815 file nguồn, đọc 497 file mã/config; không quét build/install/log/.git như nguồn chạy. Không xóa file nào.

“Không tham chiếu” là kết quả quét tĩnh tên file, không chứng minh người dùng không gọi launch/map bên ngoài. `setup.py` cài cả thư mục bằng glob không đồng nghĩa tất cả file được chạy. Vì vậy safe_to_delete giữ `no`; báo cáo này không cho phép xóa tự động.

## Luồng đang chạy

`complete.launch.py` → `complete_visual.xacro` → chassis/bánh/sensors, tùy chọn FR10; controller YAML được lọc theo `enable_arm`; `my_ur_control_cpp/swerve_drive` nhận cùng wheel geometry. Gazebo world dùng package `vinuni_gazebo_models`.

## A. Không tìm thấy tham chiếu tĩnh

| path | reason | referenced_by | safe_to_delete |
|---|---|---|---|
| `src/thietkemoi/thanxe_new_description/urdf/thanxe_new.trans` | Không tìm thấy tham chiếu tên file trong nguồn chạy đã quét; vẫn có thể được gọi từ ngoài workspace. | none found (static scan) | no |
| `src/thietkemoi/thanxe_new_description/urdf/camera.xacro` | Không tìm thấy tham chiếu tên file trong nguồn chạy đã quét; vẫn có thể được gọi từ ngoài workspace. | none found (static scan) | no |
| `src/thietkemoi/thanxe_new_description/urdf/ur10_with_charge.xacro` | Không tìm thấy tham chiếu tên file trong nguồn chạy đã quét; vẫn có thể được gọi từ ngoài workspace. | none found (static scan) | no |
| `src/thietkemoi/thanxe_new_description/urdf/complete_car.urdf` | Không tìm thấy tham chiếu tên file trong nguồn chạy đã quét; vẫn có thể được gọi từ ngoài workspace. | none found (static scan) | no |
| `src/thietkemoi/thanxe_new_description/urdf/dynamic_obstacle.urdf` | Không tìm thấy tham chiếu tên file trong nguồn chạy đã quét; vẫn có thể được gọi từ ngoài workspace. | none found (static scan) | no |
| `src/thietkemoi/thanxe_new_description/config/visualize_all.rviz` | Không tìm thấy tham chiếu tên file trong nguồn chạy đã quét; vẫn có thể được gọi từ ngoài workspace. | none found (static scan) | no |
| `src/thietkemoi/thanxe_new_description/config/mapper_params_online_async.yaml` | Không tìm thấy tham chiếu tên file trong nguồn chạy đã quét; vẫn có thể được gọi từ ngoài workspace. | none found (static scan) | no |
| `src/thietkemoi/thanxe_new_description/maps/map_name.yaml` | Không tìm thấy tham chiếu tên file trong nguồn chạy đã quét; vẫn có thể được gọi từ ngoài workspace. | none found (static scan) | no |
| `src/thietkemoi/thanxe_new_description/maps/my_vinfast_map.xcf` | Không tìm thấy tham chiếu tên file trong nguồn chạy đã quét; vẫn có thể được gọi từ ngoài workspace. | none found (static scan) | no |
| `src/thietkemoi/thanxe_new_description/maps/my_vinfast_map1.pgm` | Không tìm thấy tham chiếu tên file trong nguồn chạy đã quét; vẫn có thể được gọi từ ngoài workspace. | none found (static scan) | no |
| `src/thietkemoi/thanxe_new_description/maps/my_vinfast_map_old.pgm` | Không tìm thấy tham chiếu tên file trong nguồn chạy đã quét; vẫn có thể được gọi từ ngoài workspace. | none found (static scan) | no |
| `src/thietkemoi/thanxe_new_description/meshes/base_link1.stl` | Không tìm thấy tham chiếu tên file trong nguồn chạy đã quét; vẫn có thể được gọi từ ngoài workspace. | none found (static scan) | no |
| `src/thietkemoi/thanxe_new_description/meshes/dausac.stl` | Không tìm thấy tham chiếu tên file trong nguồn chạy đã quét; vẫn có thể được gọi từ ngoài workspace. | none found (static scan) | no |
| `src/thietkemoi/thanxe_new_description/worlds/charging.world` | Không tìm thấy tham chiếu tên file trong nguồn chạy đã quét; vẫn có thể được gọi từ ngoài workspace. | none found (static scan) | no |
| `src/thietkemoi/thanxe_new_description/worlds/vinfast_realmap_copy.world` | Không tìm thấy tham chiếu tên file trong nguồn chạy đã quét; vẫn có thể được gọi từ ngoài workspace. | none found (static scan) | no |
| `src/thietkemoi/thanxe_new_description/worlds/new.world` | Không tìm thấy tham chiếu tên file trong nguồn chạy đã quét; vẫn có thể được gọi từ ngoài workspace. | none found (static scan) | no |
| `src/thietkemoi/thanxe_new_description/worlds/vinfast_realmap.world` | Không tìm thấy tham chiếu tên file trong nguồn chạy đã quét; vẫn có thể được gọi từ ngoài workspace. | none found (static scan) | no |
| `src/thietkemoi/thanxe_new_description/worlds/vinfast_map_with_actors.world` | Không tìm thấy tham chiếu tên file trong nguồn chạy đã quét; vẫn có thể được gọi từ ngoài workspace. | none found (static scan) | no |

## B. Có vẻ không dùng trong launch hiện tại, chưa nên xóa

| path | reason | referenced_by | safe_to_delete |
|---|---|---|---|
| `src/thietkemoi/thanxe_new_description/config/my_controllers.yaml` | Không thuộc launch xe hiện tại; còn tham chiếu legacy/ngoại vi hoặc tên trùng nên chưa kết luận bỏ được. | src/thietkemoi/thanxe_new_description/urdf/complete_car.urdf | no |
| `src/thietkemoi/thanxe_new_description/maps/map_name.pgm` | Không thuộc launch xe hiện tại; còn tham chiếu legacy/ngoại vi hoặc tên trùng nên chưa kết luận bỏ được. | src/thietkemoi/thanxe_new_description/maps/map_name.yaml | no |
| `src/thietkemoi/thanxe_new_description/meshes/Wheel1_1.stl` | Không thuộc launch xe hiện tại; còn tham chiếu legacy/ngoại vi hoặc tên trùng nên chưa kết luận bỏ được. | src/thietkemoi/thanxe_new_description/urdf/complete_car.urdf | no |
| `src/thietkemoi/thanxe_new_description/meshes/base_link.stl` | Không thuộc launch xe hiện tại; còn tham chiếu legacy/ngoại vi hoặc tên trùng nên chưa kết luận bỏ được. | src/thietkemoi/thanxe_new_description/urdf/complete_car.urdf | no |
| `src/thietkemoi/thanxe_new_description/meshes/Revolute2_1.stl` | Không thuộc launch xe hiện tại; còn tham chiếu legacy/ngoại vi hoặc tên trùng nên chưa kết luận bỏ được. | src/thietkemoi/thanxe_new_description/urdf/complete_car.urdf | no |
| `src/thietkemoi/thanxe_new_description/meshes/Wheel4_1.stl` | Không thuộc launch xe hiện tại; còn tham chiếu legacy/ngoại vi hoặc tên trùng nên chưa kết luận bỏ được. | src/thietkemoi/thanxe_new_description/urdf/complete_car.urdf | no |
| `src/thietkemoi/thanxe_new_description/meshes/Revolute3_1.stl` | Không thuộc launch xe hiện tại; còn tham chiếu legacy/ngoại vi hoặc tên trùng nên chưa kết luận bỏ được. | src/thietkemoi/thanxe_new_description/urdf/complete_car.urdf | no |
| `src/thietkemoi/thanxe_new_description/meshes/Revolute4_1.stl` | Không thuộc launch xe hiện tại; còn tham chiếu legacy/ngoại vi hoặc tên trùng nên chưa kết luận bỏ được. | src/thietkemoi/thanxe_new_description/urdf/complete_car.urdf | no |
| `src/thietkemoi/thanxe_new_description/meshes/Wheel2_1.stl` | Không thuộc launch xe hiện tại; còn tham chiếu legacy/ngoại vi hoặc tên trùng nên chưa kết luận bỏ được. | src/thietkemoi/thanxe_new_description/urdf/complete_car.urdf | no |
| `src/thietkemoi/thanxe_new_description/meshes/cong_sac.stl` | Không thuộc launch xe hiện tại; còn tham chiếu legacy/ngoại vi hoặc tên trùng nên chưa kết luận bỏ được. | src/thietkemoi/thanxe_new_description/worlds/vinfast_realmap.world; src/models/model.sdf; src/models/cong_sac/model.sdf; src/models/tram_sac_VF/model.sdf | no |
| `src/thietkemoi/thanxe_new_description/meshes/Revolute1_1.stl` | Không thuộc launch xe hiện tại; còn tham chiếu legacy/ngoại vi hoặc tên trùng nên chưa kết luận bỏ được. | src/thietkemoi/thanxe_new_description/urdf/complete_car.urdf | no |
| `src/thietkemoi/thanxe_new_description/meshes/Wheel3_1.stl` | Không thuộc launch xe hiện tại; còn tham chiếu legacy/ngoại vi hoặc tên trùng nên chưa kết luận bỏ được. | src/thietkemoi/thanxe_new_description/urdf/complete_car.urdf | no |
| `src/thietkemoi/thanxe_new_description/worlds/test.world` | Không thuộc launch xe hiện tại; còn tham chiếu legacy/ngoại vi hoặc tên trùng nên chưa kết luận bỏ được. | src/fr10/frcobot_ros2/fairino10_v6_moveit2_config/launch/gazebo.launch.py | no |

## C. Đang dùng hoặc là cấu hình được hỗ trợ

| path | reason | referenced_by | safe_to_delete |
|---|---|---|---|
| `src/thietkemoi/thanxe_new_description/launch/gazebo.launch.py` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/fr10/fr10_pose_controller/launch/fr10_pose_controller.launch.py; src/fr10/frcobot_ros2/fairino10_v6_moveit2_config/launch/gazebo.launch.py; src/thietkemoi/thanxe_new_description/launch/complete.launch.py; src/Universal_Robots_ROS2_Gazebo_Simulation/ur_simulation_gazebo/launch/ur_sim_control.launch.py | no |
| `src/thietkemoi/thanxe_new_description/launch/display.launch.py` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | none found (static scan) | no |
| `src/thietkemoi/thanxe_new_description/launch/complete.launch.py` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/launch/gazebo.launch.py | no |
| `src/thietkemoi/thanxe_new_description/urdf/vehicle_dimensions.xacro` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/urdf/thanxe_new.xacro | no |
| `src/thietkemoi/thanxe_new_description/urdf/inertial_macros.xacro` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/urdf/thanxe_new.xacro; src/thietkemoi/thanxe_new_description/urdf/complete_visual.xacro | no |
| `src/thietkemoi/thanxe_new_description/urdf/materials.xacro` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/urdf/thanxe_new.xacro | no |
| `src/thietkemoi/thanxe_new_description/urdf/swerve_module.xacro` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/urdf/thanxe_new.xacro | no |
| `src/thietkemoi/thanxe_new_description/urdf/thanxe_new.xacro` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/urdf/complete_visual.xacro | no |
| `src/thietkemoi/thanxe_new_description/urdf/fr10_wrist.xacro` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/urdf/complete_visual.xacro | no |
| `src/thietkemoi/thanxe_new_description/urdf/d435_simulation.xacro` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/urdf/fr10_wrist.xacro | no |
| `src/thietkemoi/thanxe_new_description/urdf/fr10_arm.urdf` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/urdf/complete_visual.xacro | no |
| `src/thietkemoi/thanxe_new_description/urdf/complete_visual.xacro` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/launch/display.launch.py; src/thietkemoi/thanxe_new_description/launch/complete.launch.py; src/thietkemoi/thanxe_new_description/urdf/complete_car.urdf | no |
| `src/thietkemoi/thanxe_new_description/urdf/ros2_control.xacro` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/fr10/frcobot_ros2/fairino10_v6_moveit2_config/config/fairino10_v6_robot.urdf.xacro; src/thietkemoi/thanxe_new_description/urdf/ur10_with_charge.xacro; src/thietkemoi/thanxe_new_description/urdf/thanxe_new.xacro; src/Universal_Robots_ROS2_Description/urdf/ur_macro.xacro | no |
| `src/thietkemoi/thanxe_new_description/urdf/vehicle_chassis.xacro` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/urdf/thanxe_new.xacro | no |
| `src/thietkemoi/thanxe_new_description/urdf/body_sensors.xacro` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/urdf/complete_visual.xacro | no |
| `src/thietkemoi/thanxe_new_description/urdf/thanxe_new.gazebo` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/urdf/thanxe_new.xacro | no |
| `src/thietkemoi/thanxe_new_description/config/display.rviz` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/launch/display.launch.py | no |
| `src/thietkemoi/thanxe_new_description/config/vehicle_geometry.yaml` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/launch/complete.launch.py; src/thietkemoi/thanxe_new_description/urdf/vehicle_dimensions.xacro | no |
| `src/thietkemoi/thanxe_new_description/config/SLAM.rviz` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/launch/complete.launch.py | no |
| `src/thietkemoi/thanxe_new_description/config/combine_controller.yaml` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/launch/complete.launch.py; src/thietkemoi/thanxe_new_description/urdf/complete_visual.xacro | no |
| `src/thietkemoi/thanxe_new_description/config/nav2_params.yaml` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/launch/complete.launch.py | no |
| `src/thietkemoi/thanxe_new_description/maps/my_vinfast_map.yaml` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/launch/complete.launch.py | no |
| `src/thietkemoi/thanxe_new_description/maps/my_vinfast_map.pgm` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/maps/my_vinfast_map.yaml | no |
| `src/thietkemoi/thanxe_new_description/meshes/jlr_ccs2_centered.stl` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/urdf/fr10_wrist.xacro | no |
| `src/thietkemoi/thanxe_new_description/worlds/vinfast_map.world` | Mô hình/launch hiện tại hoặc cấu hình phục vụ Nav2/display; giữ lại. | src/thietkemoi/thanxe_new_description/launch/complete.launch.py | no |

## Các folder/package cần phân biệt

| path | reason | referenced_by | safe_to_delete |
|---|---|---|---|
| `src/Universal_Robots_ROS2_Description` | Không nằm trong model xe FR10; vẫn được các package UR sử dụng | UR driver / UR Gazebo / ur10_with_charge.xacro | no |
| `src/Universal_Robots_ROS2_Driver` | Bộ điều khiển UR thật/MoveIt riêng, không được complete.launch gọi | Các launch và package UR | no |
| `src/Universal_Robots_ROS2_Gazebo_Simulation` | Mô phỏng UR độc lập | ur_simulation_gazebo | no |
| `src/fr10/frcobot_ros2/fairino_description` | Còn dùng mesh FR10 gốc | fr10_arm.urdf | no |
| `src/fr10/fr10_pose_controller` | Controller/IK FR10 độc lập, không tích hợp vào launch xe này | launch riêng và tests riêng | no |
| `src/models` | Đã đóng gói thành vinuni_gazebo_models; nhiều model còn được các world phụ dùng | complete.launch.py / *.world / *.sdf | no |
| `build`, `install`, `log` | Sản phẩm colcon, không phải mã nguồn; install đang phục vụ launch | setup.bash và ROS package index | no |

## File legacy đáng chú ý

- `complete_car.urdf`: snapshot tự sinh cũ, còn đường dẫn `/home/loc/...` và model UR cũ; không chỉnh tay để làm nguồn mô hình mới. Launch mới xuất trực tiếp Xacro.
- `base_link.stl`, `Wheel*_1.stl`, `Revolute*_1.stl`: không còn là geometry xe Ø420; giữ cho snapshot/cấu hình cũ. Kích thước CAD 250mm không phù hợp bánh mới.
- `camera.xacro`, `ur10_with_charge.xacro`: cụm camera/UR10 cũ; D455 và FR10 dùng file riêng.
- `setup.py` còn khai báo một số console script Python nhưng module tương ứng không tồn tại (ví dụ swerve_controller); launch chính dùng C++ swerve_drive. Không sử dụng các entry point hỏng này.

## Đề xuất cấu trúc

Giữ package/vendor hiện tại để tránh phá depend. Mô hình xe đã tách thành vehicle_dimensions, vehicle_chassis, swerve_module, body_sensors, fr10_arm, fr10_wrist. Nếu dọn các mesh legacy sang archive hoặc xóa package UR, cần cập nhật cả snapshot/launch UR và kiểm thử riêng; lần này chưa di chuyển/xóa chúng.

## Duplicate chính xác theo SHA256

Không có duplicate byte-for-byte trong nhóm mesh/config/world của package xe đã xét.
