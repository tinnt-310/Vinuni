# Kế hoạch chỉnh sửa và nghiệm thu FR10 + Vehicle

> **Lưu trữ lịch sử.** Kế hoạch khảo sát này ghi trạng thái trước triển khai và trước yêu cầu dùng một model FR10. Trạng thái hiện tại, quyết định và hành động tiếp theo nằm trong [PROGRESS_LEDGER.md](../../PROGRESS_LEDGER.md). Không dùng đường vendor `base_link`/`tool0` mô tả bên dưới cho run mới. Model hiện hành là `vehicle_description/urdf/vehicle.urdf.xacro`, root `fr10_base_link`, TCP `charging_tcp`.

## 1. Phạm vi và trạng thái đã xác minh

Mục tiêu là dùng cùng motion core để kiểm chứng **arm riêng trong Gazebo, arm gắn trên xe và arm qua FAIRINO SimMachine tại `192.168.58.2`**. Vehicle standalone là gate bắt buộc trước nghiệm thu hệ ghép. Theo lựa chọn của bạn, Gazebo chạy trên host; không bổ sung Gazebo trong Docker.

Phiên lập kế hoạch này chỉ khảo sát, chưa sửa code, tạo ledger hay chạy chuyển động.

| Hạng mục | Kết quả khảo sát |
|---|---|
| Ledger/master task | Chưa có `PROGRESS_LEDGER.md`, `FR10_VEHICLE_MASTER_TASK.md`; master specification được AGENTS dẫn tới cũng thiếu. |
| Git | HEAD `3c1be9c`; đã có các commit swerve mới và nhiều thay đổi chưa commit cần bảo toàn. |
| Evidence cũ | Có JSON xác nhận free-space arm/vehicle/mounted PASS và GenericSystem chạy được one-shot/stream pose. Đây là evidence của cấu hình đã ghi trong từng run. |
| CTest đã lưu | Log FR10 gần nhất có 68 test, không có failure; có một skip đã giải thích. Chưa đại diện cho các chỉnh sửa sắp tới. |
| Vehicle hiện tại | Launch đã chọn `swerve_controller`; harness vẫn đợi `steering_controller`/`drive_controller`. CMake swerve chưa đăng ký test. |
| FR10 end-to-end | Gazebo regression cũ dùng JTC trực tiếp, chưa kiểm chứng `fr10_pose_controller_node` trên model Gazebo. |
| Backend Gazebo | Launch pose/Gazebo dùng JTC trong khi backend gửi position array; backend còn mặc định bắt buộc heartbeat FAIRINO. |
| Frame/TCP | Model ghép dùng `fr10_base_link` và `charging_tcp`; model vendor dùng `base_link` và `tool0`. MoveIt provider đang có phép tính theo global model frame cần sửa trước khi dùng model ghép. |
| Lỗi còn mở | Startup crash, barrier-impact LCP, feedback dropout 50 ms và quyền ghi stop reason giữa các thread. |
| Web UI | IP đã xác minh thuộc container `fairino_simmachine:v3.9.8`. URL bạn gửi chuyển về `/login.html`; chưa xác định được chương trình FR10 cần chạy. |

**Thứ tự xử lý:** chốt provenance → sửa launch/harness → xác minh control/state và cô lập lỗi runtime → nối pose controller với Gazebo → tách trách nhiệm → regression cuối và Web UI.

## 2. Các bước chỉnh sửa theo dependency

### P0 — Lập baseline có thể tiếp tục được

- Tạo duy nhất [PROGRESS_LEDGER.md](../../PROGRESS_LEDGER.md) khi bắt đầu triển khai; ghi trạng thái từ code và evidence đã kiểm tra.
- Chụp Git HEAD, diff, untracked files và submodule state; không reset, stash hoặc gom thay đổi có sẵn vào commit mới.
- Lập inventory toàn workspace: package đang build, launch entrypoint, topic/action/service, command owner, TF owner, source/config/model, vendor và phần legacy.
- Gắn mỗi evidence với source/config/model/binary tương ứng. Giữ PASS lịch sử nhưng không dùng để chứng nhận graph swerve mới.
- Ledger chứa đầy đủ iteration, hypothesis, experiment, evidence, verdict, thay đổi chưa regression, trạng thái A/B/C/Web và lệnh tiếp theo.

**Gate:** xác định được graph và artifact thực sự được dùng; không trộn `install/` cũ với `install_grouped/`.

### P1 — Đồng bộ launch, controller và hệ thống đo

- Dùng `swerve_controller` làm command owner duy nhất cho vehicle. Launch chính lấy cấu hình plugin hiện hành; đưa geometry từ YAML nguồn vào controller để tránh hai bộ thông số độc lập.
- Sửa harness theo controller type, command interface và lifecycle thực tế; kiểm tra service/state sau khi spawn, không chỉ dựa vào chuỗi log.
- Giữ entrypoint tương thích, nhưng launch chính không đi vòng qua các wrapper đã deprecated.
- Mọi bước spawn/controller chỉ tiếp tục khi prerequisite thành công. Readiness gồm `/clock` tiến, đủ joint state, controller active và observer có dữ liệu.
- Thay việc sửa launch bằng string replacement trong acceptance runner bằng launch arguments/config rõ ràng. Experiment vẫn dùng artifact riêng và ghi mọi override.
- Bổ sung manifest: revision/diff hash, executable/library hash, URDF, controller YAML, world, environment, PID và exit status.
- Observer thu liên tục Gazebo model/link pose, joint state, command, odometry, TF và controller status trên cả thời gian chuyển động lẫn settling.

**Gate:** test harness phải phát hiện được sai controller, thiếu pose, trace đứt, server chết, startup timeout và source/install không khớp. Run thiếu evidence trả `INVALID`.

### P2 — Xác minh low-level control và cô lập lỗi runtime

**Swerve và odometry**

- Tách phần kinematics thuần khỏi lifecycle/plugin để test trực tiếp; plugin giữ interface ownership và update loop.
- Test đảo hướng, steering gần ±π, slew limit, wheel saturation, zero command, timeout, NaN/Inf và feedback lỗi.
- Chốt `/cmd_vel_nav` là vận tốc trong `base_footprint`; kiểm tra frame và timestamp của `TwistStamped`. Command cũ, tương lai hoặc sai frame không được kéo dài chuyển động.
- Đóng dấu odometry bằng thời điểm state được đo. Khi update lỗi hoặc ngừng, không phát snapshot cũ với timestamp mới như state còn fresh.
- Đối chiếu odometry với Gazebo ground truth, bao gồm chiều quay, trục ngang/dọc và owner duy nhất của `odom → base_footprint`.

**Startup crash và contact**

- Giữ candidate slip `1e-5`, spawn `z=0` làm baseline; chưa đổi thêm mass, inertia, solver hoặc friction.
- Tái hiện startup crash với timestamp, exit code, backtrace/core và trạng thái spawn/controller; chỉ thay một yếu tố sau mỗi kết luận.
- Với angled barrier, đo collision pair, contact normal/depth/force, wheel command và chassis state quanh lỗi đầu tiên.
- Thử riêng từng giả thuyết về geometry/contact hoặc command khi va chạm. Việc bỏ barrier chỉ là experiment đối chứng, không là production fix.
- Sau candidate fix: vehicle và mounted phải qua contact reproduction; startup phải qua 20 lần fresh liên tiếp có readiness và measured state.

**Feedback dropout**

- Ghi riêng tuổi joint state, tuổi driver health, sequence, buffer-write failure và callback delay.
- Fault injection lần lượt vào joint feedback, health và callback scheduling; giữ watchdog 50 ms.
- Xác định nguyên nhân bằng trace; không tăng timeout để làm test PASS.
- Giữ nguyên nguyên nhân fault đã latch; worker cancellation hoặc plan rejection không được ghi đè lỗi communication.

**Gate:** có reproduction hoặc trace xác định boundary lỗi, test cho sửa đổi và measured regression. Một run không tái hiện lỗi chưa đủ đóng issue.

### P3 — Nối pose controller vào cả Gazebo standalone và mounted

- Bổ sung lựa chọn launch `arm_control:=trajectory|pose`; giữ mặc định hiện tại để tương thích.
- Profile `pose` dùng `JointGroupPositionController` tại `/fairino10_controller/commands`, chỉ một owner cho j1–j6 và một controller manager trong model ghép.
- Giữ JTC để chẩn đoán độc lập; không cho JTC và pose bridge cùng claim arm joints.
- Chọn rõ runtime `fairino`, `gazebo`, `fake` và kiểm tra khớp plugin trong robot description. FAIRINO luôn yêu cầu health thật; simulation không phát heartbeat giả và phải báo nguồn health đúng.
- Dùng model D435/CCS2 hiện hành cho A/C. PlanningScene chứa cả chassis và arm ở mounted; không dùng model vendor độc lập để kiểm tra collision của xe ghép.
- Tính FK, workspace barrier và pose error tương đối với arm root được cấu hình. Lấy fixed tool transform từ URDF, rồi cross-check analytical FK với MoveIt.
- Bổ sung SRDF/config tương ứng với model ghép; không thay vị trí gá, joint axes, TCP cơ khí hoặc safety limits.
- Gazebo dùng simulation time cho trajectory progression; watchdog kết nối giữ steady clock. Pause/reset clock không được làm quỹ đạo nhảy mẫu hoặc dùng lại plan cũ.
- Target mounted được diễn giải theo arm base; nghiệm thu coupled motion bằng joint/free-space motion. Strict insertion/retract được kiểm chứng khi vehicle đứng yên; chưa bổ sung giữ TCP cố định theo world khi xe chạy.

**Gate:** cùng public command đi qua `MotionExecutor → RosControlBackend → ros2_control → GazeboSystem`; đo được arm/TCP thực, không chỉ action status.

### P4 — Tách trách nhiệm trong motion adapter

Tách theo ownership, tiếp tục dùng các core component đã được trích xuất:

- **Command validation:** chuyển ROS input thành domain command, kiểm tra dữ liệu một lần.
- **Motion coordinator:** sở hữu generation, target được chấp nhận, staging/insertion/retract, retry và stop transitions.
- **Planning service:** sở hữu worker/mailbox, scene snapshot, route/IK, timing và trace validation; trả immutable result kèm generation/scene revision.
- **Telemetry:** đọc snapshot và tạo diagnostic/status; không sửa motion state.
- **ROS adapter:** subscriptions, timers, lifecycle và kết nối các thành phần.

Control tick vẫn là writer duy nhất của `MotionExecutor`. Không đưa MoveIt, network, chờ worker hoặc dựng dense path vào tick. Worker trả lỗi qua result; coordinator quyết định transition và stop reason.

**Gate:** deterministic tests cho stale result, scene update, stop đang planning, target mới, retry bounds, stream silence, insertion/retract và shutdown; runtime behavior tương đương hoặc sửa đúng lỗi đã có evidence.

## 3. Interface và giới hạn thay đổi

- Giữ các topic public, JOINT/POSE modes, đơn vị, joint limits và đường SDK hiện hành.
- Các bổ sung public chỉ là lựa chọn launch runtime/control profile; diagnostics có thể thêm trường nguyên nhân feedback và provenance.
- Core library tiếp tục không phụ thuộc ROS/MoveIt/Gazebo.
- `fairino_hardware_v3_9_3` tiếp tục là owner duy nhất của SDK actuation; giữ readiness port 8080 và ServoJ 8 ms.
- Không tự enable robot, đổi mode, reset alarm hoặc chuyển endpoint sang robot thật.
- Chỉ bỏ legacy khi chứng minh không có consumer/build/deployment phụ thuộc; phần production chưa chắc chắn giữ lại và ghi rõ trong audit.

## 4. Ma trận kiểm chứng cuối

Sau mỗi thay đổi, build package bị ảnh hưởng và chạy test liên quan. Trước nghiệm thu cuối, build toàn bộ package active theo `colcon list`, chạy CTest và các Python test thuộc luồng được sửa.

| Gate | Kịch bản bắt buộc |
|---|---|
| A — Arm Gazebo | Joint goal, retarget, one-shot pose, stream 10 Hz rồi ngừng input, insertion, retract, controlled stop và shutdown. |
| B — Vehicle Gazebo | Zero/hold, forward, lateral, rotation, diagonal, reversal, command timeout; so sánh odom–Gazebo. |
| C — Mounted Gazebo | Arm khi xe đứng; arm và xe cùng chuyển động ≥2 s; xe dừng rồi insertion/retract; kiểm tra TF/TCP và stability. |
| SimMachine | Qua plugin FAIRINO: startup/hold, joint/pose, retarget, stop/retract và 50.000 control cycles với telemetry thật. |
| Web UI | Sau A và SimMachine CLI PASS: mở URL bạn cung cấp, vào process, xác định chương trình FR10 hiện có, xác minh start/running/finish hoặc trạng thái duy trì đúng thiết kế. |
| Architecture | Audit toàn workspace theo responsibility, cohesion, coupling, dependency, state ownership, testability, compilation boundary và runtime ownership. |

**Tiêu chí đo cố định trước chạy**

- A/B/C: mỗi cấu hình **3 PASS liên tiếp**, mỗi lần fresh `gzserver`, cùng candidate. FAIL/INVALID ngắt chuỗi; giữ toàn bộ attempt.
- Zero LCP, không nonfinite command/state bắt buộc, không server crash; observer phải bao phủ cả motion và settling.
- Arm pose profile: đạt tolerance hiện hành — joint `0.002 rad`, TCP `2 mm`; strict insertion giữ ngưỡng hướng `1°` và path monitoring hiện có.
- Vehicle/mounted: ground-truth trace ≥20 s và ≥100 mẫu; tilt `<0.05 rad`, final linear speed `<0.03 m/s` trong 2 s cuối; scenario chuyển động chính có displacement ≥0.5 m.
- Sai lệch odometry cuối free-space không vượt `0.02 m + 5%` quãng đường ground truth; yaw không vượt `0.02 rad + 5%` góc quay.
- Ma trận V0–V8 cũ được giữ để chẩn đoán. V2/V3 dùng controller tách chỉ trong fixture isolation; acceptance production phải dùng swerve plugin thực tế. Chạy lại các case bị ảnh hưởng bởi đổi controller/contact.
- Không dùng GenericSystem PASS thay cho Gazebo hoặc SimMachine PASS.

Web UI phải có evidence UI và status/log tương ứng. Trước khi chạy chương trình web, dừng và xác nhận đã nhả ROS motion owner. Nếu chưa đăng nhập hoặc chưa xác định đúng chương trình hiện có, ghi blocker cụ thể cho Web; không chọn chương trình calibration vendor để thay thế.

## 5. Accept/reject, rollback và kết thúc

Mỗi experiment ghi trước hypothesis, một biến được thay, phép đo và tiêu chí kết luận:

- **ACCEPTED:** dữ liệu hợp lệ, kết quả hỗ trợ giả thuyết và gate liên quan không regression.
- **REJECTED:** dữ liệu hợp lệ bác bỏ giả thuyết hoặc thay đổi gây regression; hoàn nguyên đúng phần experiment.
- **INVALID:** thiếu state, observer đứt, chưa steady hoặc server chết trước đủ phép đo. Crash vẫn được ghi riêng là runtime failure.
- Không nới tiêu chí sau khi xem kết quả; không dùng retry thành công để xóa failure.
- Chỉ commit logical unit đã kiểm chứng; không commit experiment bị reject hoặc thay đổi không thuộc công việc.

Chỉ ghi `SUCCESS` khi A/B/C đủ 3 run, SimMachine và Web PASS, build/test đạt, architecture/granularity audit đạt, không còn issue runtime nghiêm trọng và ledger phản ánh đúng evidence. Nếu còn việc nội bộ thì ghi `PARTIAL` với next action cụ thể; chỉ ghi `BLOCKED` cho dependency bên ngoài đã có bằng chứng.

**Bước triển khai đầu tiên:** tạo ledger từ baseline trên, rồi sửa contract launch–controller–harness và thêm test phát hiện mismatch hiện tại trước khi chạy Gazebo mới.
