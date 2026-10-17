# Swerve FX5U — SLMP/TCP, mục tiêu 20 Hz

Bridge dùng bảng địa chỉ trong tài liệu ngày 17/10/2026, giữ nguyên Ladder.
Jetson tính lệnh từ `/cmd_vel` hoặc `/joy`; một worker duy nhất giữ kết nối
SLMP 3E Binary TCP, thực hiện khởi động, ghi lệnh, đọc phản hồi và dừng.
Mã FR10 và mô phỏng không thay đổi.

| Chức năng | Địa chỉ, theo bánh 1 FR / 2 RR / 3 FL / 4 RL |
|---|---|
| Servo ON | M1000–M1003 |
| Trigger ON/OFF | M1010–M1013 |
| Nhả phanh | M1020 |
| BLDC raw | D1000–D1003; 660 dừng, 0 lớn nhất |
| Hall raw | D1010–D1013 |
| Góc đặt | D1020–D1023 |
| Góc thực tế | D1030–D1033 |
| Enable BLDC | Y8; cấu hình quyền điều khiển bên dưới |

Bảng mới không có bit đảo chiều hay reset. Luồng mới không ghi M58, M222,
M60/M65/M62/M63 hoặc các bit reset cũ. Để đi lùi, động học dùng góc lái
180° và tốc độ bánh không âm. Cần cơ cấu lái hỗ trợ đủ vòng 360°.
Thứ tự logic FL/FR/RL/RR được đổi thành bánh PLC bằng `[3, 1, 4, 2]`.

## Chạy

Cài một lần trong thư mục này:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e .
```

Chạy bridge trên máy có ROS 2 Humble và kết nối PLC:

```bash
cd /home/tinnt/Vinuni/src/hardware_interface/plc_bridge
./run_bridge.sh --execute
# ./run.sh --execute là lệnh tương đương.
```

Terminal khác:

```bash
./run_keyboard.sh
```

`/cmd_vel` nhận Twist của keyboard/Nav2. Joystick giữ quyền ưu tiên khi nhấn
deadman (mặc định nút 4). Space/K dừng theo cơ chế teleop hiện có.
Phím `r` hủy lệnh hiện tại và yêu cầu dừng; không phát reset PLC vì bảng mới
không cung cấp địa chỉ reset. Không có tự nối lại hoặc tự tiếp tục sau lỗi SLMP.

## Cấu hình và các giả định còn phải đo

- `config/default.yaml`: kết nối, thời gian, khởi động, mã hóa góc, Y8, Hall.
- `config/registers_mapping.yaml`: bảng địa chỉ cố định. Loader từ chối bảng
  khác giao thức này, tránh nhầm góc với tốc độ.
- `config/open_loop_bridge.yaml`: hình học, ánh xạ bánh, offset và giới hạn
  chuyển động phía Jetson. Tần số phải bằng cấu hình SLMP; đường kính Hall
  phải bằng hai lần bán kính bánh trong bridge.

Mặc định `bldc.enable_mode: external`: Jetson **không ghi Y8**; Ladder hoặc
cơ chế cho phép hiện có phải bật Y8. Đổi thành `slmp` khi đã xác định Ladder
cho phép Jetson điều khiển Y8. Chế độ này tắt Y8 khi khởi động, chỉ bật sau
khi góc khởi tạo đạt yêu cầu, rồi tắt khi đóng chương trình hoặc gặp lỗi.

`trigger_on_s: 0.010`, `trigger_off_s: 0.010` là thời gian thử nghiệm.
`plc_max_scan_s: null` nghĩa là chưa có bằng chứng về thời gian quét;
status báo `trigger_timing_verified: false`. Khi điền thời gian quét lớn
nhất đã đo, loader yêu cầu cả ON và OFF ít nhất bằng thời gian đó nhân
`scan_margin` (mặc định 1.5). ON + OFF phải nhỏ hơn chu kỳ 50 ms để còn
ngân sách truyền thông. Chưa có bảo đảm 20 Hz trên PLC thật.

Mã hóa góc hiện giả định một word 16-bit có dấu, `units_per_degree: 1`,
`raw_zero: 0`. Offset lái thẳng 90° giữ từ cấu hình trước. Hãy đối chiếu
D1020 với D1030 và cơ khí; đổi scale, raw_zero, offset/sign và góc khởi tạo
cùng nhau nếu PLC dùng đơn vị khác. Khởi động chờ góc thực tế trong sai số
5°, tối đa 5 giây; thời gian 0.3 giây trước nhả phanh chỉ là chờ Servo ON,
không phải tín hiệu Servo Ready.

Đường kính bánh 0.428 m, bán kính 0.214 m và 25 xung/vòng vẫn cần xác minh.
Khi `hall.mode: unknown`, bridge đọc và công bố raw Hall nhưng không suy
ra tốc độ. Chỉ chọn `cumulative_u16` khi đã xác nhận bộ đếm tích lũy 16-bit,
25 xung cho một vòng bánh. Ước lượng dùng cửa sổ 0.3 giây, thời gian đo
thực tế, xử lý tràn unsigned và bỏ cửa sổ khi có bước nhảy bất hợp lý/reset.
Phản hồi Hall chỉ cho độ lớn tốc độ; không suy ra chiều hoặc odometry.

Ánh xạ tốc độ vẫn là giả định cũ `raw = 660 - 260 * abs(v_banh)` với
`raw_at_one_m_s: 400`, có giới hạn và ramp. Đây **không phải hiệu chuẩn**;
400 trong tài liệu là ngưỡng bắt đầu chạy, không xác nhận tốc độ 1 m/s.
Các lệnh như 608 có thể chưa làm bánh quay. Cần đo Hall để hiệu chuẩn trước
khi dùng tốc độ định lượng hoặc Nav2 trên robot thật. Status ghi rõ
`velocity_calibration: provisional`.

## Chu kỳ và xử lý lỗi

Khởi động: ghi BLDC=660 → Trigger OFF → Servo ON → chờ → nhả phanh → ghi
bốn góc khởi tạo → Trigger ON/giữ/OFF → kiểm tra góc thực tế → cho phép Y8
nếu dùng chế độ SLMP. Trong lúc chờ góc, BLDC vẫn bằng 660.

Mỗi chu kỳ đang di chuyển:

1. Ghi block D1000–D1003.
2. Ghi block D1020–D1023.
3. Bật M1010–M1013 và giữ đủ thời gian ON.
4. Tắt M1010–M1013; đảm bảo thời gian OFF trước xung kế tiếp.
5. Đọc D1030–D1033 và D1010–D1013 khi đến kỳ, mặc định 10 Hz.

Trigger OFF được thử ngay cả khi giao dịch ON lỗi/mất ACK. Mỗi chu kỳ
hoàn tất trước chu kỳ sau; chu kỳ quá 50 ms tăng `overruns` và không tạo
loạt lệnh bù. Mailbox chỉ giữ cặp góc/tốc độ mới nhất. Callback ROS không
truyền SLMP và không bị chặn trong lúc giữ Trigger.

Góc đặt ramp 30°/s. BLDC giảm theo cosin sai lệch giữa **hướng bánh cần đi**
và góc thực tế, dừng khi sai lệch từ 45°. Điều này tránh chạy theo hướng
cũ khi góc đặt trung gian đang ramp sang hướng mới. Phản hồi góc quá cũ
(mặc định 0.3 s) cũng đặt BLDC=660. Lệnh hết hạn 0.75 s, lệnh zero hoặc
mất nguồn lệnh dừng BLDC và giữ góc đặt gần nhất, không phát Trigger mới.
Worker có timeout riêng để vẫn dừng nếu callback ROS bị treo.

Lỗi truyền thông được chốt: dừng nhận lệnh mới, thử ghi tốc độ 660, Trigger
OFF, Y8 OFF nếu được cấu hình, đóng phanh, Servo OFF rồi ngắt TCP.
Mỗi bước dọn được thử độc lập. Đây là dừng phần mềm có nỗ lực tối đa;
khi TCP mất hoặc Jetson mất điện, không có watchdog trong Ladder nên cần
cơ chế dừng độc lập như tài liệu hệ thống đã nêu.

Quan sát:

```bash
ros2 topic echo /plc_bridge/status
```

`raw` là lệnh Jetson gửi vào mailbox; `plc.applied_raw` là lệnh đã có ACK.
ACK không xác nhận bánh đã quay. `plc.feedback.actual_angle_deg` và
`hall_raw` là phản hồi đọc từ PLC; `hall_speed_m_s` là null khi chưa xác
nhận bộ đếm/chưa đủ cửa sổ. `cycle_s` đo thời gian làm việc, `period_s` đo
khoảng cách hai lần bắt đầu chu kỳ, `feedback_age_s` đo tuổi phản hồi góc.

## Kiểm tra offline và nghiệm thu trên máy

```bash
PYTHONPATH=src .venv/bin/python -m unittest discover -s tests -v
```

Các test dùng driver giả: thứ tự ghi, độ rộng ON/OFF, mailbox mới nhất,
quá chu kỳ, timeout lệnh, lỗi ON/OFF/read, dọn sau lỗi, tràn/reset Hall,
động học không đảo chiều BLDC và tích hợp bridge ROS giả.

Trước chạy 20 Hz trên máy, thử hai góc nối tiếp khi Servo còn chuyển động,
đọc D1030–D1033 để xác định FB Ladder có nhận Trigger khi Busy hay không.
Sau đó đo period/overrun, thời gian quét và Hall trên bánh được kê lên,
đối chiếu số bánh, dấu, offset và cơ chế dừng độc lập. Nếu Servo không nhận
lệnh lúc Busy, phải điều chỉnh cách phát lệnh theo khả năng FB hiện có.

## Giao thức cũ

`legacy_default.yaml`, `legacy_registers_mapping.yaml` và
`legacy_open_loop_bridge.yaml` giữ cấu hình trước đây để kiểm thử hồi quy.
[README_LEGACY.md](README_LEGACY.md) và
[docs/PROTOCOL_HUONG_DAN_CHAY.md](docs/PROTOCOL_HUONG_DAN_CHAY.md) là tài liệu
cũ, có bảng địa chỉ khác. Các script interactive/example cũ chỉ dùng được
với `--config config/legacy_default.yaml --execute` và PLC tương ứng;
chúng không được dùng để vận hành bảng Ladder mới. `run.sh` và
`run_bridge.sh` hiện đều chạy bridge mới.
