> **GIAO THỨC CŨ:** Bảng địa chỉ và lệnh dưới đây thuộc Ladder trước đây.
> Với cấu hình Swerve 20 Hz mới, dùng [README hiện tại](../README.md).

# Protocol vận hành bộ điều khiển FX5U bốn bánh

## 1. Phạm vi

Tài liệu này mô tả cách kiểm tra offline và cách gửi lệnh từ PC đến PLC Mitsubishi FX5U bằng SLMP 3E Binary/TCP.

Thông số kết nối mặc định:

| Thông số            |         Giá trị |
| ------------------- | --------------: |
| PLC IP              | `192.168.3.250` |
| PLC port            |          `1025` |
| Trigger/reset pulse |        `0.05 s` |

Chương trình PC là SLMP client. PLC phải được cấu hình là SLMP server với cùng IP, port, giao thức TCP và frame 3E Binary.

### Lệnh chạy chính thức

Sau khi cài đặt một lần, mỗi lần vận hành chỉ chạy:

```bash
cd /home/tin/plc
./run.sh
```

Không cần kích hoạt `.venv` thủ công và không cần nhập lại IP, port hay `--execute`.

---

## 2. Cảnh báo an toàn

Trước khi chạy `./run.sh`:

1. Nâng các bánh khỏi mặt đất và cách ly khu vực thử nghiệm.
2. Chuẩn bị nút dừng khẩn cấp phần cứng.
3. Xác nhận IP/port trong GX Works3 trùng với `config/default.yaml`.
4. Xác nhận đúng mapping M/D với chương trình PLC.
5. Xác nhận giới hạn góc cơ khí của từng ổ.
6. Xác nhận đơn vị và dấu của giá trị tốc độ trước khi gửi tốc độ khác 0.

`stop` trong chương trình chỉ là dừng bằng phần mềm. Nó không thay thế mạch Emergency Stop đạt chuẩn an toàn.

---

## 3. Đánh số bánh và mapping PLC

| Bánh | Vị trí    | Trigger | Servo | Reset | Góc   | Tốc độ |
| ---: | --------- | ------- | ----- | ----- | ----- | ------ |
|    1 | Phải trên | M100    | M110  | M120  | D1000 | D1020  |
|    2 | Phải dưới | M101    | M111  | M121  | D1001 | D1021  |
|    3 | Trái trên | M102    | M112  | M122  | D1002 | D1022  |
|    4 | Trái dưới | M103    | M113  | M123  | D1003 | D1023  |

Góc được gửi bằng một D word nguyên cho mỗi bánh, trong dải `0..359°`.
Góc `90°` là tư thế bánh để robot đi thẳng. Bridge ROS 2 tự cộng mốc 90°
và chuẩn hóa góc về dải này; CLI nhận trực tiếp góc tuyệt đối của PLC.
Chưa thể gửi số lẻ độ qua mapping một word hiện tại.

Tốc độ là giá trị raw; không được hiểu là RPM hoặc mm/s khi PLC/servo chưa xác nhận đơn vị.

---

## 4. Cài đặt

```bash
cd /home/tin/plc

python3 -m venv .venv

.venv/bin/python -m pip install --upgrade pip setuptools

.venv/bin/python -m pip install -e .
```

Phần cài đặt chỉ thực hiện một lần. Không xóa thư mục `.venv`.

Sau khi đóng terminal hoặc khởi động lại máy, chạy lại bằng:

```bash
cd /home/tin/plc
./run.sh
```

---

## 5. Mở chương trình điều khiển

Chỉ chạy lệnh này sau khi hoàn thành kiểm tra an toàn ở mục 2:

```bash
cd /home/tin/plc
./run.sh
```

`run.sh` tự dùng `.venv/bin/python`, cấu hình `config/default.yaml` và bật chốt `--execute`.

Đây là lệnh vận hành PLC thật, không phải chế độ mô phỏng.

Khi thấy dấu nhắc sau, chương trình đã mở phiên điều khiển:

```text
FX5U>
```

Xem danh sách lệnh:

```text
help
```

---

## 6. Protocol lệnh CLI

### 6.1. Trạng thái

```text
status
```

`Servo ON-command acknowledged` chỉ có nghĩa là lệnh ghi Servo ON đã được PLC giao tiếp chấp nhận.

Nó không phải tín hiệu Servo Ready từ drive, vì địa chỉ feedback Servo Ready hiện chưa được cấu hình.

---

### 6.2. Reset lỗi

Reset cả bốn ổ:

```text
reset all
```

Chuỗi ghi:

```text
M120..M123 = [1, 1, 1, 1]

chờ 0.05 s

M120..M123 = [0, 0, 0, 0]
```

Reset riêng ổ số 3:

```text
reset 3
```

---

### 6.3. Servo ON/OFF

Bật cả bốn servo:

```text
servo on all
```

Tắt cả bốn servo:

```text
servo off all
```

Bật hoặc tắt riêng một servo:

```text
servo on 1
servo off 1
```

Các lệnh góc và tốc độ có thể tự gửi Servo ON nếu trạng thái phần mềm đang là OFF.

Trong quá trình commissioning vẫn nên dùng:

```text
servo on all
```

rõ ràng trước lệnh chuyển động.

---

### 6.4. Lệnh `angle`: chỉ gửi góc lái

Cú pháp cho cả bốn bánh:

```text
angle <góc_bánh_1> <góc_bánh_2> <góc_bánh_3> <góc_bánh_4>
```

Ví dụ:

```text
angle 100 100 80 80
```

Chuỗi ghi chính xác:

```text
D1000..D1003 = [100, 100, 80, 80]

M100..M103 = [1, 1, 1, 1]

chờ 0.05 s

M100..M103 = [0, 0, 0, 0]
```

Lệnh này không ghi `D1020..D1023`.

Cú pháp cho một bánh:

```text
angle <số_bánh> <góc_độ>
```

Ví dụ, đặt bánh số 1 ở `100°`:

```text
angle 1 100
```

Ví dụ, đặt bánh số 3 ở `80°`:

```text
angle 3 80
```

Đưa cả bốn bánh về góc đi thẳng `90°`:

```text
angle 90 90 90 90
```

---

### 6.5. Lệnh `speed`: chỉ gửi tốc độ raw

Cú pháp cho cả bốn bánh:

```text
speed <speed_1> <speed_2> <speed_3> <speed_4>
```

Ví dụ:

```text
speed 650 650 650 650
```

Ở lệnh tốc độ đầu tiên, chuỗi ghi là:

```text
D1020..D1023 = [660, 660, 660, 660]

M58 = 1

Y8 = 1

D1020..D1023 = [650, 650, 650, 650]
```

Các lần sau vẫn xác nhận `M58 = 1` và `Y8 = 1` trước khi ghi raw mới. Trigger
`M100..M103` chỉ dùng cho lệnh góc, không dùng cho tốc độ.

Lệnh này không ghi `D1000..D1003`.

Cú pháp cho một bánh:

```text
speed <số_bánh> <giá_trị_raw>
```

Ví dụ:

```text
speed 2 650
```

Gửi **raw bằng 0** cho cả bốn bánh:

```text
speed 0 0 0 0
```

Raw 0 tương ứng 3.8 V/tốc độ lớn nhất, không phải dừng. Raw 660 tương ứng
0.8 V và là giá trị dừng đã cấu hình. Để dừng đúng trình tự, dùng lệnh `stop`.
Chỉ thử tốc độ khi xe đã được kê bánh và có Emergency Stop phần cứng.

---

### 6.6. Điều khiển kết hợp góc và tốc độ

CLI hiện không cung cấp lệnh `motion`. Khi bridge khởi động, tốc độ được đặt
về raw 660, Y8/M58 tắt, rồi M222 được bật và tắt sau 0,05 s để reset.
Lệnh lái đầu tiên gửi góc, xung M100..M103 và chờ ổn định.
Sau đó bridge giới hạn mức đổi góc lái 30°/s và tốc độ raw tối đa 15 đơn vị
mỗi lần ghi; góc và tốc độ có thể được cập nhật cùng chu kỳ trong khi Y8/M58
vẫn bật. Khi cần đổi chiều M của bánh, tốc độ raw được đưa về 660 trước rồi
mới đổi chiều; Y8/M58 không bị tắt trong chuyển tiếp. Raw 660 là tốc độ vật lý
nhỏ nhất/mức dừng, raw 0 là tốc độ vật lý lớn nhất. Bàn phím terminal phát
target 50 Hz nếu hỗ trợ sự kiện nhấn/nhả phím CSI-u; nhịp ghi PLC mặc định 10 Hz.
Xem mục điều khiển ROS 2 trong `README.md`.

---
### 6.7. Dừng và thoát

Dừng phần mềm:

```text
stop
```

`stop` thực hiện:

```text
dừng heartbeat/feedback runtime

M100..M103 = [0, 0, 0, 0]

M110..M113 = [0, 0, 0, 0]

khóa các lệnh chuyển động tiếp theo trong phiên hiện tại
```

Sau `stop`, phải `exit` rồi kết nối lại nếu muốn chạy tiếp:

```text
exit
```

Nhấn `Ctrl+C` hoặc dùng `exit` cũng kích hoạt quy trình shutdown:

```text
dừng runtime
→ xóa trigger
→ Servo OFF
→ đóng socket
```

---

## 7. Trình tự commissioning chỉ thử góc

```text
status

reset all

servo on all

angle 90 90 90 90

angle 100 100 80 80

angle 90 90 90 90

servo off all

exit
```

Theo dõi trực tiếp:

```text
D1000..D1003
M100..M103
M110..M113
```

trong GX Works3 khi chạy.

Không gửi `speed` trong bài thử chỉ-góc.

---

## 8. Trình tự commissioning góc và tốc độ

CLI không có lệnh `motion`. Sau khi xác nhận mapping từng bánh, dùng bridge
ROS 2 với bánh xe được kê khỏi mặt đất:

```bash
cd /home/tin/Vinuni/src/hardware_interface/plc_bridge
./run_bridge.sh --execute
```

Bridge giới hạn vận tốc, yêu cầu deadman joystick và tự dừng khi mất lệnh.
Lệnh bàn phím/joystick chi tiết nằm trong `README.md`.

---

## 9. Điều kiện chưa được xác nhận

* Giới hạn góc cơ khí thực tế; phần mềm giới hạn lệnh PLC trong `0..359°` nguyên.
* Đơn vị, dấu quay và giới hạn của tốc độ raw.
* Địa chỉ Servo Ready, góc thực tế, tốc độ thực tế và alarm code.
* Địa chỉ heartbeat/watchdog và ladder dừng an toàn tương ứng.
* PLC có đọc đồng thời góc và tốc độ bằng một trigger hay không.

Không được coi thông báo `OK` của CLI là xác nhận cơ cấu đã di chuyển đúng.

Cần feedback đã được cấu hình hoặc quan sát/đo kiểm tại máy trong điều kiện an toàn.
