# Kiểm tra và kế hoạch cải thiện hiệu năng FR10 Gazebo

Ngày: 16/10/2026. Phạm vi: Gazebo Classic, FR10 độc lập,
`station=front_mid`, `matched_socket_v2`. Đây là kết quả kiểm tra chỉ đọc và
kế hoạch sửa; chưa thay đổi runtime hoặc phát lệnh robot.

## Kết luận từ dữ liệu đã đo

Tay đang tiến đúng tốc độ yêu cầu theo thời gian mô phỏng, nhưng clock
Gazebo tiến chậm so với thời gian thực. Nút thắt quan sát được nằm trong
gzserver, với một luồng sử dụng gần hết một lõi CPU. Chưa có profiling theo
hàm để phân chia chính xác chi phí ODE, collision, servo và ROS.

| Bằng chứng | Giá trị |
|---|---|
| PID gzserver | 698565 |
| World thực tế | `/tmp/vinuni_sim_33xor3h6/force_world.world` |
| Controller config thực tế | `/tmp/vinuni_sim_33xor3h6/controllers.yaml` |
| Physics step | 0,0000625 s = 62,5 µs |
| Physics rate yêu cầu | 16.000 bước/giây mô phỏng |
| ODE solver | QuickStep, 2.000 iterations, SOR 1,0 |
| Controller manager | 16.000 Hz theo cấu hình launch |
| Physics plugin build | Release; controller cũng Release |
| URDF vật lý được truyền vào launch | 36 links, 383 collision elements trước khi Gazebo gộp fixed joints |
| Luồng gzserver bận nhất, TID 698783 | 99,84% một lõi trong cửa sổ 4,007 giây thực |
| CPU affinity của gzserver | 8 CPU logic được phép; hệ thống có 32 CPU logic |
| CPU pressure avg10/60/300 | 0,00 / 0,00 / 0,00 tại lần đo |
| Memory pressure avg10 | some 0,02%; full 0,02% tại lần đo |
| Mức dùng CPU trung bình tiến trình | gzserver ~113%; gzclient ~20%; RViz ~18%; controller ~10% |

Phép đo feedback trực tiếp trước đó:

- Force state: `APPROACH_COMPLIANT`, chưa tiếp xúc, chưa có result cuối phiên.
- Độ sâu tăng từ -27,190964 mm lên -27,010866 mm trong 6,832 giây thực:
  tiến 0,180097 mm, tương đương khoảng 0,02636 mm/s thực.
- Clock tăng từ 64,414 lên 64,783 giây mô phỏng trong 7,010 giây thực:
  real-time factor khoảng 0,05264, tức chậm khoảng 19 lần.
- Tốc độ goal 0,5 mm/s mô phỏng nhân 0,05264 cho 0,02632 mm/s thực,
  phù hợp chuyển động quan sát được. Hai cửa sổ lấy mẫu không hoàn toàn trùng
  nhau, nên không coi sai khác nhỏ giữa hai ước lượng là lỗi bám.
- Fz cuối cửa sổ khoảng 0,000113 N; source age khoảng 0,183 ms.
- Log có `[TARE QUALIFIED]`, bias nhỏ và các lượt force validation hợp lệ.

Nguồn log:
`/home/tinnt/.ros/log/fr10_pose_controller_node_698955_1792123671398.log`.
Các giá trị trên là snapshot, không phải trạng thái luôn đúng sau lần đo.

## Nguyên nhân và phần chưa chứng minh

1. **Đã xác nhận:** toàn chu trình physics đang yêu cầu bước 62,5 µs và
   controller manager chạy cùng cadence. Đây là 16.000 lượt xử lý cho mỗi
   giây mô phỏng, trong khi đường lệnh MotionExecutor vẫn 125 Hz.
2. **Đã xác nhận:** một luồng gzserver gần bão hòa một lõi; các tiến trình UI
   và controller ít CPU hơn. Chỉ số pressure thấp không cho thấy cả máy
   thiếu CPU hoặc bị nghẽn RAM tại cửa sổ quan sát. Chưa xác nhận luồng nóng
   bằng call stack, và chưa loại trừ mọi nguyên nhân về affinity/tần số CPU.
3. **Ứng viên chi phí cần tách bằng benchmark:** ODE 2.000 iterations mỗi
   bước; broad phase/AABB của nhiều cell CAD; tính ma trận quán tính và
   gravity trong torque servo mỗi bước; read/update/write của ros2_control;
   sensor, contact observer và ROS publication.
4. **Chưa xác nhận:** tỷ lệ CPU riêng của từng thành phần. `perf record`
   bị kernel từ chối vì `perf_event_paranoid=4`. Không đổi cấu hình kernel
   trong đợt kiểm tra này. Cần instrumentation trong bản build thí nghiệm
   để có thời gian theo thành phần.

Không coi số `123 Hz` trong tracking log là bằng chứng physics đang đạt
thời gian thực. Đó là thống kê vòng controller; phải đo riêng clock Gazebo.

## Vì sao không hạ tần số hoặc đổi timestep ngay

Lịch sử Stage51 ghi nhận encoder cách nhau 1 ms theo source time nhưng
57 ms theo wall time, dẫn đến vượt timeout feedback 50 ms khi physics chậm.
Launch hiện lấy cadence encoder từ cùng physics step để cung cấp mẫu thực
mới; không được đổi controller manager riêng xuống 125 Hz rồi giữ physics
và timeout như cũ mà chưa kiểm chứng.

Calibration cũng ghi nhận bước 1 ms gây dao động encoder/FT ở startup.
Timestep và solver iterations là các tham số ảnh hưởng chất lượng vật lý,
không chỉ ảnh hưởng tốc độ. Chưa thể khẳng định đổi 62,5 µs thành 1 ms sẽ
giữ được kết quả tiếp xúc.

Tham chiếu: `docs/fr10_insertion_pipeline.md`, mục cadence Stage51;
`test_results/fr10_admittance_pose_error_20261003/WORK_LOG.md`, Stage51;
`src/vehicle_description/config/fr10_simulation_actuators.yaml`.

## Kế hoạch sửa theo thứ tự

### 1. Thêm đo thời gian và benchmark có thể lặp lại

- Ghi riêng sim-time/wall-time, real-time factor và tốc độ TCP theo cả hai
  thang thời gian trong harness `run_matched_station_gazebo.py`.
- Thêm instrumentation tùy chọn cho world step, servo, hardware read/write,
  contact observer và collision callback; thu mean/p95/max bằng counter
  hoặc histogram. Tổng hợp theo wall time, không log mỗi physics step.
- Ghi số bước, số query/contact và nguồn dữ liệu mới để phân biệt chi phí
  mỗi bước với chi phí gửi ROS. Đo overhead của chính instrumentation.
- Chạy riêng ba đoạn: đứng yên, tiếp cận chưa contact, seating/HOLD có tải.
  Mỗi thay đổi so với baseline cùng binary/config/CAD và tải máy gần nhau;
  lưu run mới, không ghi đè evidence cũ.

Kết quả cần có trước tối ưu sâu: phần nào chiếm thời gian mỗi bước và phần
nào là nút thắt lớn nhất. Không quy hết lỗi cho ODE chỉ từ số iterations.

### 2. Giảm tải hiển thị/cảm biến không phục vụ bài thử

- So sánh baseline với `gui=false rviz=false` trong lượt mô phỏng riêng,
  giữ physics, force và CAD như baseline. Việc này xác định mức đóng góp
  của UI, không được mặc định coi là lời giải cho luồng gzserver đang nóng.
- Thêm tùy chọn launch để tắt depth camera khi thử force bằng pose có sẵn;
  giữ visual camera, khối lượng và collision của robot. Hiện depth sensor
  vẫn always-on ở 10 Hz source time.
- Chỉ giảm serialization/publication của telemetry không dùng nếu phép đo
  cho thấy có chi phí đáng kể. Phân biệt rõ contact observer chẩn đoán với
  nguồn wrench mà profile thực sự dùng.
- Giữ đầy đủ dữ liệu cần cho raw safety, tare và nghiệm thu. Không bỏ mẫu
  FT hoặc làm mới timestamp cho mẫu cũ để tăng tốc.

File dự kiến: `src/vehicle_bringup/launch/bringup.launch.py`,
`src/vehicle_bringup/launch/simulation.launch.py`,
`src/vehicle_bringup/config/simulation.xacro`,
`src/vehicle_description/urdf/d435_simulation.xacro` và đường truyền tham số
Xacro liên quan.

### 3. Tối ưu tính toán nhưng giữ cùng mô hình vật lý

Thực hiện thành phần nào trước tùy kết quả bước 1:

- Torque servo: loại tính toán lặp, cache dữ liệu bất biến và chỉ lấy mỗi
  pose/inertia cần thiết một lần trong cùng bước. Bất kỳ phép gom inertia
  của nhóm fixed links nào cũng phải đối chiếu ma trận M(q), gravity và
  torque với bản gốc trên nhiều cấu hình. Không cache pose, q hoặc wrench
  động qua nhiều bước như dữ liệu bất biến.
- CAD broad phase: đo số lần tính AABB và chi phí transform của các cell.
  Có thể thay phép biến đổi tám góc bằng center/extents với absolute
  rotation và biên roundoff bảo thủ, nếu kiểm thử chứng minh bao đầy đủ.
  Giữ nguyên facets, lỗ và narrow-phase contact; không xóa collision cell
  hoặc thay toàn bộ đầu cắm bằng hull kín.
- Contact observer/ROS: cache dữ liệu mounting bất biến, tách chi phí đọc
  mẫu vật lý khỏi đóng gói telemetry. Giữ contact safety và timestamp thực.
- Chạy lại kiểm tra hình học, lực, servo và benchmark sau từng thay đổi,
  chỉ giữ tối ưu có cải thiện đo được và bảo toàn kết quả trong sai số đã
  định nghĩa trước.

File dự kiến: `gazebo_torque_system.cpp`, `ode_mesh_contact.cpp`,
`mesh_contact_kernel.cpp`, `gazebo_contact_wrench.cpp` trong
`src/hardware_interface/fr10_simulation_hardware/src/`.

### 4. Tìm cấu hình physics nhanh nhất vẫn qua nghiệm thu

Sau khi có baseline và tối ưu tương đương:

- Timestep: thử 62,5 → 125 → 250 → 500 µs trong các run riêng, một biến
  mỗi lần. Mọi giá trị là candidate; 1 ms chỉ xét nếu cần và phải qua lại
  test startup vì đã có lịch sử dao động.
- Solver iterations: giữ 2.000 làm baseline, thử 1.000 rồi 500 trên
  timestep đã đạt. Đưa số iterations vào cấu hình có validation thay vì
  hardcode trong `charging_station.py`; chưa đổi default trước nghiệm thu.
- World và cadence feedback tiếp tục lấy từ một nguồn cấu hình. Không
  suy ra chất lượng mẫu chỉ từ tần số đặt; đo source/receipt gap thực.
- So sánh encoder/FK với TCP vật lý độc lập, tải FT, keyed fit, penetration,
  dừng và rút, chứ không chỉ action SUCCESS hoặc RTF cao hơn.
- Nếu chưa có cấu hình Gazebo qua đủ gate, giữ baseline và ghi rõ giới hạn.
  MuJoCo có thể là lựa chọn chạy bài thử nhanh riêng; kết quả MuJoCo không
  thay cho nghiệm thu Gazebo.

File dự kiến: `src/vehicle_description/config/fr10_simulation_actuators.yaml`,
`src/vehicle_bringup/launch/charging_station.py`, regression cấu hình world
và harness convergence.

### 5. Làm log vận hành rõ hơn

- Giữ nhãn executor POSE/JOINT, bổ sung force phase, depth, khoảng cách còn
  tới mouth, wrench mục tiêu/đo được, speed theo sim/wall và RTF.
- Tránh dùng force 5 N của seating làm nhãn mục tiêu khi đang free approach;
  hiển thị wrench mục tiêu thực của phase hiện tại.
- Xác định rõ HOLD ở preinsert với HOLD sau khi đạt force/depth/fit.
- Sửa đường đọc telemetry action-generated type: CLI `ros2 topic echo`
  hiện trả `Insert_Feedback is invalid` ở môi trường này, trong khi probe
  import trực tiếp `Insert.Feedback` đọc được. Cung cấp công cụ đọc hoặc
  diagnostic message phù hợp; không dùng lỗi của CLI để suy ra mất feedback.

File dự kiến: diagnostics/snapshot của `fr10_pose_controller`, telemetry
trong `insertion_ros_adapter.cpp`, hướng dẫn trong `README_COMMANDS.md`.

## Điều kiện hoàn tất và giới hạn

- Mục tiêu hiệu năng để chọn candidate: RTF tiếp cận ít nhất 0,5,
  hướng tới 0,8–1,0 trên máy này; báo riêng RTF khi contact/HOLD. Đây là
  mục tiêu thử nghiệm, chưa phải tốc độ đã đạt hoặc cam kết.
- Tốc độ TCP theo source time phải khớp goal 0,5 mm/s ở đoạn chưa giảm
  tốc/tiếp xúc. Không thay dt sang wall time hoặc tăng goal để che RTF thấp.
- Giữ nguyên các gate hiện hành của Gazebo: keyed fit <=250 µm,
  penetration <=500 µm, seating 5±1 N và HOLD ổn định ít nhất 5 giây
  source time; cùng các giới hạn wrench/ngang/moment đang có.
- Encoder/FT/base khi áp dụng phải giữ freshness 50 ms wall time, kiểm
  tra timestamp nguồn, unique samples, raw extrema và failure behavior.
- MotionExecutor và command stream vẫn 125 Hz; sole actuator owner,
  collision/corridor/braking validation và controlled stop được giữ.
- Chạy startup, free-space/preinsert, full contact/HOLD/retract và fault
  stale/overload/stop có liên quan. Báo riêng PASS/FAIL của từng phạm vi;
  bài thử nhanh không thay nghiệm thu toàn chu trình.
- Dùng build thử riêng, không ghi đè thư viện Gazebo đang nạp. Build/test
  các package thay đổi và hoàn thành kiểm tra bắt buộc trước chuyển runtime.

Đề xuất triển khai đầu tiên: instrumentation + benchmark tách tải, sau đó
chọn tối ưu theo kết quả. Chưa có bằng chứng để chọn một timestep/iteration
mới làm default ngay trong phiên vận hành hiện tại.
