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

Góc được gửi trực tiếp theo độ, không nhân hệ số. Ví dụ `-30` được ghi là giá trị word có dấu `-30`.

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
angle 30 30 -30 -30
```

Chuỗi ghi chính xác:

```text
D1000..D1003 = [30, 30, -30, -30]

M100..M103 = [1, 1, 1, 1]

chờ 0.05 s

M100..M103 = [0, 0, 0, 0]
```

Lệnh này không ghi `D1020..D1023`.

Cú pháp cho một bánh:

```text
angle <số_bánh> <góc_độ>
```

Ví dụ, đặt bánh số 1 ở `+30°`:

```text
angle 1 30
```

Ví dụ, đặt bánh số 3 ở `-45°`:

```text
angle 3 -45
```

Đưa lệnh góc cả bốn bánh về `0°`:

```text
angle 0 0 0 0
```

---

### 6.5. Lệnh `speed`: chỉ gửi tốc độ raw

Cú pháp cho cả bốn bánh:

```text
speed <speed_1> <speed_2> <speed_3> <speed_4>
```

Ví dụ:

```text
speed 10 10 10 10
```

Chuỗi ghi:

```text
D1020..D1023 = [10, 10, 10, 10]

M100..M103 = [1, 1, 1, 1]

chờ 0.05 s

M100..M103 = [0, 0, 0, 0]
```

Lệnh này không ghi `D1000..D1003`.

Cú pháp cho một bánh:

```text
speed <số_bánh> <giá_trị_raw>
```

Ví dụ:

```text
speed 2 10
```

Gửi tốc độ 0 cho cả bốn bánh:

```text
speed 0 0 0 0
```

Không gửi tốc độ khác 0 trên máy thật trước khi xác nhận đơn vị, dấu quay và giới hạn tốc độ trong PLC/servo.

---

### 6.6. Lệnh `motion`: gửi đồng thời góc và tốc độ

Cú pháp gồm đúng tám số, bốn góc trước và bốn tốc độ sau:

```text
motion <góc_1> <góc_2> <góc_3> <góc_4> <speed_1> <speed_2> <speed_3> <speed_4>
```

Ví dụ:

```text
motion 30 30 -30 -30 100 100 100 100
```

Ý nghĩa từng giá trị:

```text
góc:
D1000 = 30
D1001 = 30
D1002 = -30
D1003 = -30

tốc độ:
D1020 = 100
D1021 = 100
D1022 = 100
D1023 = 100
```

Chuỗi ghi chính xác:

```text
D1000..D1003 = [30, 30, -30, -30]

D1020..D1023 = [100, 100, 100, 100]

M100..M103 = [1, 1, 1, 1]

chờ 0.05 s

M100..M103 = [0, 0, 0, 0]
```

`motion` dùng một xung trigger cho cả hai dải dữ liệu, nên thời gian giữ khóa do xung là 50 ms thay vì 100 ms khi gọi `angle` rồi `speed` riêng biệt.

Chỉ dùng `motion` sau khi xác nhận PLC đọc cả dải góc và dải tốc độ trong cùng sự kiện `M100..M103`.

Nếu PLC xử lý chúng thành hai bước riêng, phải tiếp tục dùng `angle` và `speed` riêng.

Hiện tại `motion` chỉ hỗ trợ cập nhật cả bốn bánh, không có cú pháp kết hợp cho một bánh.

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

angle 0 0 0 0

angle 10 10 -10 -10

angle 0 0 0 0

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

Không gửi `speed` hoặc `motion` trong bài thử chỉ-góc.

---

## 8. Trình tự commissioning góc và tốc độ

Chỉ dùng sau khi đã xác nhận đơn vị tốc độ và logic trigger của PLC:

```text
status

reset all

servo on all

motion 0 0 0 0 0 0 0 0

motion 10 10 -10 -10 10 10 10 10

motion 0 0 0 0 0 0 0 0

stop

exit
```

---

## 9. Điều kiện chưa được xác nhận

* Giới hạn góc cơ khí thực tế; phần mềm hiện chỉ giới hạn theo word có dấu `-32768..32767`.
* Đơn vị, dấu quay và giới hạn của tốc độ raw.
* Địa chỉ Servo Ready, góc thực tế, tốc độ thực tế và alarm code.
* Địa chỉ heartbeat/watchdog và ladder dừng an toàn tương ứng.
* PLC có đọc đồng thời góc và tốc độ bằng một trigger hay không.

Không được coi thông báo `OK` của CLI là xác nhận cơ cấu đã di chuyển đúng.

Cần feedback đã được cấu hình hoặc quan sát/đo kiểm tại máy trong điều kiện an toàn.
