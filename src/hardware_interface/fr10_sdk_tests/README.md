# Kiểm tra SDK FAIRINO trên FR10 / SimMachine

Bộ test độc lập ROS 2 cho tất cả nhóm API trong ảnh: ServoJ, ServoCart,
ServoJT, FT_Control, compliance, tìm/chèn, đọc/tare lực, nhận dạng tải và CNDE.
Mặc định dùng **header + libfairino.so.2.3.3** đang có trong dự án, dành cho
robot software 3.9.3. Không cần build toàn workspace hoặc chạy ros2_control.

**Tạo bộ test không xác nhận robot đã chạy được.** Build và test offline chỉ
xác minh chữ ký/liên kết SDK và logic bộ đo. Các kết quả thực phải lấy từ tủ.
Lần phát triển bộ test này không mở kết nối tới robot hay SimMachine.

## Bắt đầu

Chạy từ terminal:

```bash
cd /home/tinnt/Vinuni/src/hardware_interface/fr10_sdk_tests
bash build.sh
python3 run.py --list
python3 run.py --suite all
cp config.example.json config.local.json
```

`build.sh` build chương trình thật và bản giả lập dùng cho test offline,
rồi chạy CTest. Phụ thuộc: CMake >=3.16, C++17, Python >=3.10 và header
`nlohmann/json.hpp` (`nlohmann-json3-dev` trên Ubuntu). Chúng có trên máy lúc tạo.
`--suite all` **chỉ lập kế hoạch offline**, kiểm tra cấu hình và liệt kê các
bài còn thiếu thông số. Không thêm `--execute` thì không khởi tạo đối tượng SDK,
không mở kết nối tới tủ. Kết quả mặc định ở `results/<UTC timestamp>/`.

Sửa `config.local.json` trước khi chạy:

- `ip`: IP chính xác của endpoint cần đo.
- `target`: `real` hoặc `sim`. Đây là nhãn do người chạy cung cấp; không phải
  bằng chứng endpoint là Docker.
- `expected_web_version`: mặc định `v3.9.3`.
- `expected_sdk_contains`: mặc định `V2.3.3`.
- `expected_model_prefix`: mặc định `FR10`. Nếu chủ động thử SimMachine FR5,
  đổi thành `FR5` và `target` thành `sim`; kết quả đó vẫn là FR5.

SDK version, model, web version và controller version được ghi riêng. Tủ trả
khác cấu hình sẽ được báo `IDENTITY_MISMATCH` và chặn các lệnh thay đổi trạng
thái. Không suy firmware từ tên thư mục hoặc từ riêng phiên bản SDK.

## Đọc trạng thái trước

```bash
python3 run.py --config config.local.json --suite read-only --execute
```

Hai bài `state` và `ft_read` ghi phiên bản, joint/TCP, tốc độ, frame counter,
trạng thái cảm biến, mã lỗi, lực raw và reference; `ft_read` gọi trực tiếp
`FT_GetConfig`, `FT_GetForceTorqueOrigin(1,...)`, `FT_GetForceTorqueRCS(1,...)`.
Sensor inactive hoặc sáu số 0 không chứng minh cảm biến đang hoạt động.
`OBSERVED` chỉ có nghĩa thu được feedback hợp lệ.

## ServoCart trên robot 3.9.3

Dừng ROS/ros2_control và các chương trình khác đang gửi lệnh tới cùng tủ.
Người vận hành chọn Auto/enable, vị trí xuất phát và khoảng trống phù hợp.
Cờ `--exclusive-control` xác nhận điều này; lock của bộ test chỉ ngăn hai
phiên của chính bộ test chạy cùng IP, không khóa được WebApp hay ROS.

```bash
python3 run.py --config config.local.json --case servocart_absolute --execute --exclusive-control
python3 run.py --config config.local.json --case servocart_base --execute --exclusive-control
python3 run.py --config config.local.json --case servocart_tool --execute --exclusive-control
```

Mỗi lệnh là một bài độc lập; kiểm tra kết quả/vị trí hiện tại trước bài tiếp
theo. Mặc định dịch **1 mm trong 2 s**, chu kỳ **8 ms**, rồi giữ mục tiêu bằng
stream thêm 100 ms. Vị trí xuất phát lấy từ TCP thực tế; không dùng pose mẫu
cố định của hãng. **Robot giữ ở điểm cuối, không tự quay về điểm xuất phát.**

Để thử +10 mm theo Z của base, tạo cấu hình riêng với `servo.cart_axis=2`,
`servo.cart_delta_mm=10`, `duration_s=10` và `guards.max_tcp_delta_mm=12`,
rồi chạy `servocart_absolute` bằng `--config` trỏ tới cấu hình đó. Build lại
sau khi cập nhật mã; bộ thử chấp nhận độ dịch từ -50 tới +50 mm và vẫn kiểm
tra giới hạn dịch khớp, TCP, góc, lực/mô-men. `expected_web_version` phải khớp
chính xác giá trị `versions.web` trong kết quả `state` của tủ đang thử.

| Bài | Cách gọi | Đo kết quả |
|---|---|---|
| `servoj` | ServoJ đủ 8 tham số, id=0 | Khớp được chọn dịch 0.2 độ mặc định |
| `servocart_absolute` | ServoCart mode=0 | Pose tuyệt đối trong base |
| `servocart_base` | ServoCart mode=1 | Các bước dịch trong base |
| `servocart_tool` | ServoCart mode=2 | Các bước dịch trong tool, đổi hướng sang base để kiểm tra |

ServoCart truyền `exaxis={0,0,0,0}` và `pos_gain={1,1,1,1,1,1}` rõ ràng.
Bộ đo này dành cho FR10 với trục ngoài không dùng/ở zero và đứng yên;
feedback trục ngoài khác zero sẽ chặn bài servo. Góc là độ, vị trí là mm.
`acc`, `vel`, `filterT`, `gain` SDK đặt 0 theo header 2.3.3, không coi là PID.
Các bài incremental gửi **phần dịch tăng thêm của từng chu kỳ**, không gửi
lặp lại toàn bộ 1 mm mỗi tick. Tool rotation dùng Rz*Ry*Rx theo FAIRINO.

`MOTION_OBSERVED` cần có phản hồi đúng chiều và sai số cuối trong ngưỡng.
`NO_MEASURED_RESPONSE` phát hiện API trả 0 nhưng robot đứng yên;
`TRACKING_MISMATCH` phát hiện chuyển động sai chiều/không đạt mục tiêu.
Đây là đo TCP theo feedback tủ, chưa phải kiểm định bằng thiết bị đo ngoài.

## Các bài mô-men, lực và hiệu chuẩn

Chạy từng bài bằng mẫu lệnh:

```bash
python3 run.py --config config.local.json --case TEN_BAI --execute --exclusive-control
```

| Tên bài | Hàm thực sự được gọi |
|---|---|
| `servojt` | ServoJTStart → ServoJT(torque,interval) → ServoJTEnd |
| `servojt_limits` | ServoJTStart → ServoJT(...,checkFlag=3,powerLimit,velocityLimit) → ServoJTEnd |
| `ft_control_basic` | FT_Control overload cơ bản, bật → quan sát → tắt |
| `ft_control_mb` | FT_Control overload thêm M/B, bật → quan sát → tắt |
| `ft_control_extended` | FT_Control thêm M/B/threshold/adjustCoeff, bật → quan sát → tắt |
| `ft_movel` | FT_Control cơ bản → MoveL ngắn theo X/Y tool → đo lực/TCP → dừng và tắt lực |
| `ft_compliance` | FT_Control → FT_ComplianceStart → quan sát → FT_ComplianceStop → tắt FT_Control |
| `ft_spiral` | FT_Control → FT_SpiralSearch → tắt FT_Control |
| `ft_linear` | FT_Control → FT_LinInsertion → tắt FT_Control |
| `ft_rotational` | FT_Control → FT_RotInsertion(strategy=0) → tắt FT_Control |
| `ft_surface` | FT_Control → FT_FindSurface → tắt FT_Control |
| `ft_zero` | FT_SetZero(action), mặc định action=1 |
| `ft_payload_record` | FT_PdIdenRecord(sensor_coordinate_id) |
| `ft_payload_compute` | FT_PdIdenCompute → ghi kg, không tự áp dụng tải |

Các trường `null` trong cấu hình phải điền bằng thông số phù hợp fixture.
Bộ test không tự chọn torque hay gain lực cho robot thật:

- Mô-men: điền `torque_nm`, `max_abs_torque_nm`, `power_limit_w`,
  `velocity_limit_deg_s`; đặt `prepared=true` sau khi chuẩn bị cơ cấu đỡ và
  chế độ torque/drag theo hướng dẫn hãng. Không tự chuyển drag mode.
  `servojt` dùng đúng overload cơ bản để so sánh; chỉ bài `servojt_limits`
  bật bảo vệ power/velocity của API. Cả hai đều quan sát tốc độ và displacement.
- Lực: `fixture_ready=true`, `select`, `desired_wrench`, `pid`; đặt giới hạn
  lực/torque và hình học thực tế trong `guards`/`force`. M/B/threshold/adjustCoeff
  áp dụng hai trục rx/ry trong overload này, không phải mảng 6 trục.
  Compliance còn cần `compliance_p`, `compliance_threshold_n`.
- Tare: `stationary_no_contact_confirmed=true` sau khi kiểm tra điều kiện
  hiệu chuẩn. FT_SetZero thay đổi zero trong tủ; không tự khôi phục zero cũ.
- Nhận dạng tải: chuẩn bị pose theo quy trình hãng, đặt
  `payload_record_pose_prepared=true` để record. Sau record thành công đặt
  `payload_record_completed=true` để compute. Không tự di chuyển giữa pose
  hiệu chuẩn và không tự ghi tải vừa tính vào cấu hình tủ.

Các bài FT/torque trả `API_ACCEPTED_UNVERIFIED` khi tủ chấp nhận lệnh và thu
được feedback. **Chưa gọi đó là nghiệm thu điều khiển lực, tìm được lỗ, cắm
thành công, tare đúng hay tải chính xác.** Cần kiểm chứng với fixture/chuẩn đo.
Các bài `ft_control_basic`, `ft_control_mb`, `ft_control_extended` và
`ft_compliance` chỉ bật–quan sát–tắt tại pose chuẩn bị. Bài `ft_movel` dưới đây
thêm đường công tác MoveL do người vận hành cấu hình.

## Thử giữ lực Fz trong khi chạy MoveL

`ft_movel` dùng bề mặt cố định đã tiếp xúc sẵn, không tự tìm hoặc tiến vào mặt.
Chọn đoạn trượt tiếp tuyến X/Y của tool (tối đa 5 mm), không dùng tay làm bề
mặt thử. Chỉ chạy khi đường trượt và khoảng điều chỉnh lực đều có khoảng trống.
Dừng ROS/ros2_control và các client điều khiển khác, chọn Auto/Enable trên tủ.
Không ghép ServoCart hay admittance ngoài vào bài này.

```bash
cd /home/tinnt/Vinuni/src/hardware_interface/fr10_sdk_tests
bash build.sh
cp -n config.ft_movel.example.json config.ft_movel.local.json
```

Điền các trường trong `config.ft_movel.local.json`:

- `force.sensor_id`: ID hệ lực đã chuẩn bị; xác minh reference Fz cùng trục/dấu
  với Fz dùng điều khiển. `force.rcs` không đặt frame của FT_Control; trường này
  chỉ truyền cho các bài tìm/chèn. Bài này không tự gọi FT_SetRCS hay tare.
- `force.desired_wrench`: `[0,0,FZ,0,0,0]`, lực đặt có dấu, độ lớn 1–10 N.
  Đọc dấu lực tại tiếp xúc thực tế; không suy dấu từ chiều Z của ServoCart.
- `force.pid`: `[force_P,force_I,force_D,torque_P,torque_I,torque_D]`, dùng gain
  đã xác nhận phù hợp cơ cấu; không phải sáu gain tương ứng sáu trục.
- `force_movel.tool_axis`: `0` cho X tool hoặc `1` cho Y tool.
  `distance_mm` có dấu; mẫu để 2 mm, `speed_mm_s=0.5`,
  `acceleration_mm_s2=1`. Mẫu dùng `expected_tool=1`, user phải là 0.
- Sau khi chuẩn bị bề mặt, tọa độ, zero/tải và đường đi: đặt
  `force.fixture_ready=true`, `force_movel.reference_verified=true`.

Mẫu để lực, gain và trục là `null`, các cờ chuẩn bị là `false`; chưa thể phát
chuyển động khi chưa điền. Giới hạn TCP bao gồm cả đoạn trượt và khoảng điều
chỉnh lực. Dùng trục ngoài zero/không sử dụng. Endpoint IK được kiểm tra với
giới hạn khớp trước bật lực; bài này không có kiểm tra va chạm toàn đường.

```bash
# Chỉ kiểm tra cấu hình, không kết nối robot:
python3 run.py --config config.ft_movel.local.json --case ft_movel

# Đọc lực và vị trí tại tư thế đã chuẩn bị:
python3 run.py --config config.ft_movel.local.json --case ft_read --execute

# Chạy đúng một đoạn trượt, không tự quay về:
python3 run.py --config config.ft_movel.local.json --case ft_movel --execute --exclusive-control
```

Bài thử yêu cầu tiếp xúc đúng dấu liên tục 200 ms trước bật lực. MoveL dùng
`blendR=0` (không chờ hoàn tất RPC), `velAccParamMode=1`, gửi vận tốc và gia tốc
theo đơn vị vật lý. Trong khi chạy, đọc feedback khoảng 125 Hz; yêu cầu tiến độ
đo được đạt điểm cuối, `motion_done=1` và ba frame mới đứng yên. Quá thời hạn,
mất cảm biến, mất tiếp xúc 100 ms hoặc vượt giới hạn sẽ vào cleanup
StopMotion → FT_Control(0) → xác nhận đứng yên → CloseRPC.
RPC vẫn có thể block; giới hạn phần mềm chỉ được đánh giá khi RPC trả về.

`result.json` ghi `measured_tangential_mm`, `movel_complete` và `force_tracking`:
Fz trung bình/min/max, RMSE so với lực đặt và tỷ lệ mẫu trong sai số cho phép.
Chỉ dùng mẫu mới trên phần 10–90% quãng trượt đo được để tính thống kê lực.
Không có mẫu thì không có kết luận bám lực; kết quả vẫn là
`API_ACCEPTED_UNVERIFIED`, không tự đổi thành nghiệm thu giữ lực.

Từ đường dẫn kết quả in ở terminal, xuất biểu đồ lực/quãng trượt và CSV:

```bash
python3 plot_ft_movel.py results/THU_MUC_VUA_CHAY
```

Xuất `ft_movel/force_movel.png` và `force_movel.csv`; script chỉ đọc log,
không kết nối tủ. Cần matplotlib để xuất PNG. Vạch dọc thể hiện lúc gọi MoveL
và StopMotion; mẫu trước MoveL vẫn được giữ trên biểu đồ.

## CNDE

```bash
python3 run.py --config config.local.json --case cnde --execute --exclusive-control
```

Đầu tiên lấy phiên bản qua SDK. Sau đó dùng UDP 20006: stop → cấu hình field và
cycle → kiểm tra type được tủ trả → start → giải mã data → stop có acknowledgment.
Mặc định lấy `actual_joint_pos,actual_TCP_pos`, cycle 8 ms. Hỗ trợ decoder
thêm `actual_joint_vel`, `robot_mode`, `run_up_time`; cycle cho phép 1–200 ms.
Ghi chu kỳ thực nhận, frame trùng, ước lượng mất frame, lỗi độ dài/type và raw
payload. Đây là CNDE output; không gửi CNDE input điều khiển robot.
CNDE test thay cấu hình output và dừng output khi kết thúc; hãy khôi phục cấu
hình của client khác nếu trước đó có dùng. Không tự phục hồi recipe không biết.

## Báo cáo và so sánh 3.9.3 / 3.9.8

Mỗi lần chạy tạo:

```text
results/<timestamp>/
  config.json             # cấu hình đã resolve cho lần chạy
  summary.md
  summary.json            # hash executable/config, phiên bản thực, kết quả
  <case>/
    events.jsonl          # call_begin trước RPC; call_end có rc và latency
    samples.jsonl         # feedback, freshness, joint/TCP/FT, thời gian monotonic
    result.json           # phiên bản, điểm đầu/cuối, kết quả và cleanup
    stdout.log
    stderr.log
```

Riêng CNDE có `cnde_preflight/` và không có stdout/stderr của native worker
trong `cnde/`. Thời gian thư mục là UTC; không dùng nhãn target để suy phiên bản.

Để đối chiếu hai firmware bằng **cùng SDK 2.3.3**, tạo hai config với
`expected_web_version` đúng từng tủ và `target` đúng môi trường. Chạy cùng bài,
cùng thông số, lưu hai thư mục rồi:

```bash
python3 run.py --compare results/LAN_393 results/LAN_398
```

Nếu đổi cả SDK, phải cung cấp header/thư viện đi cùng và kiểm tra build lại:

```bash
cmake -S . -B build_other -DBUILD_TESTING=OFF \
  -DSDK_ROOT=/duong/dan/sdk -DSDK_LIBRARY=/duong/dan/sdk/lib/libfairino.so.PHIEN_BAN
cmake --build build_other --parallel 2
python3 run.py --binary build_other/sdk_probe --config config.other.json --case state --execute
```

API của bản khác có thể thay đổi; build lỗi là bằng chứng chữ ký không khớp,
không tự dùng header bản này với binary bản kia. Báo cáo có phiên bản thực
và đường dẫn thư viện nạp được. SimMachine FR5 không xác nhận kinematics FR10.

## Kết thúc bài và giới hạn bộ đo

Bài gây tác động yêu cầu trạng thái ban đầu đứng yên, không lỗi, program dừng,
đúng phiên bản/model. Bộ đo không gọi RobotEnable, Mode, DragTeachSwitch,
ResetAllError, FT_Activate hoặc FT_SetConfig. Người vận hành chuẩn bị chúng.
Giới hạn trong JSON là rào kỹ thuật của thí nghiệm, không phải ngưỡng an toàn
được chứng nhận. Chu kỳ thực/latency đều được ghi để phát hiện RPC quá chậm.

Khi thất bại/Ctrl+C, worker cố StopMotion, tắt các chế độ đã thử bật và CloseRPC;
thao tác bắt đầu thất bại cũng được thử cleanup vì tủ có thể đã đổi trạng thái.
Sau lệnh dừng cần ba frame mới có tốc độ khớp gần 0 để xác nhận stationarity.
Không khôi phục pose xuất phát. Một lỗi sẽ dừng suite.

RPC SDK có thể block. Trong **một RPC đang block**, bộ đo không đọc thêm feedback
để áp giới hạn phần mềm. Các API tìm/chèn phải có giới hạn khoảng cách/góc/thời
gian và fixture phù hợp đã chuẩn bị. `timeout_s` là thời hạn của tiến trình;
SIGTERM có 5 s cho cleanup, rồi có thể bị kill. Kill tiến trình **không bảo đảm
robot dừng**; `TIMEOUT_CLEANUP_UNCONFIRMED`/`CLEANUP_UNCONFIRMED` cần người vận
hành kiểm tra tủ và dùng dừng vật lý khi cần. Không chạy bài tiếp trong tình
trạng thái này.

## Nguồn đối chiếu

- [C++ motion 3.9.3](https://fairino-doc-en.readthedocs.io/3.9.3/SDKManual/CPPRobotMovement.html).
- [Changelog 3.9.3: ServoCart thêm exaxis](https://fairino-doc-en.readthedocs.io/3.9.3/SDKManual/CPPVersionIntro.html).
- [Force API 3.9.3](https://fairino-doc-en.readthedocs.io/3.9.3/SDKManual/CPPRobotForceControl.html).
- PDF local (local: `src/fairino-doc-en-readthedocs-io-en-3.9.3.pdf`): chương 6 Robot
  Communication, trang in 2537–2546 cho frame CNDE/output/type negotiation.
- Header đi cùng [SDK local](../fr10_driver/fairino_hardware_v3_9_3/libfairino/include/robot.h)
  là chữ ký dùng để compile. Một số prototype trên web lược tham số `flag` hoặc
  RCS khác header; bộ test dùng chữ ký của binary 2.3.3 đang liên kết.
