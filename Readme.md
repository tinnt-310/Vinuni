# Chạy robot Swerve với PLC FX5U

**Làm từ bước 1 đến bước 5.** Mỗi khối lệnh có thể sao chép nguyên khối.
Các đường dẫn dưới đây dùng cho dự án tại `/home/tinnt/Vinuni`.

## 1. Chuẩn bị

- Máy chạy đã có ROS 2 Humble và Python 3.
- Cắm Ethernet giữa máy và PLC, cấp nguồn PLC/Servo/BLDC.
- Máy và PLC cùng mạng; PLC hiện dùng **192.168.3.250:1025**.
- Lần đầu: kê bánh khỏi mặt đất, có nút dừng khẩn cấp trong tầm tay.

**Khi chạy, chương trình tự nhả phanh và đưa góc đặt Servo về 90°.**
Người phụ trách cần xác nhận bảng địa chỉ PLC và mã hóa góc trước lần thử đầu.
Xung Trigger 10 ms hiện là giá trị thử nghiệm, chưa xác nhận trên PLC thật.

## 2. Cài và kiểm tra — chỉ làm lần đầu

Mở Terminal bằng `Ctrl+Alt+T`, dán:

```bash
cd /home/tinnt/Vinuni/src/hardware_interface/plc_bridge
python3 -m venv .venv
.venv/bin/python -m pip install -e .
```

Nếu báo thiếu `venv`, chạy rồi thực hiện lại khối trên:

```bash
sudo apt update
sudo apt install -y python3-venv
```

Chạy kiểm thử, **không điều khiển robot**:

```bash
cd /home/tinnt/Vinuni/src/hardware_interface/plc_bridge
PYTHONPATH=src .venv/bin/python -m unittest discover -s tests -v
```

Cuối kết quả phải có **`OK`**. Nếu báo lỗi, dừng tại đây.

## 3. Terminal 1 — kết nối PLC

Kiểm tra cổng PLC; lệnh này chỉ mở rồi đóng kết nối:

```bash
python3 - <<'PY'
import socket
with socket.create_connection(("192.168.3.250", 1025), timeout=3):
    print("OK: ket noi duoc PLC")
PY
```

Chỉ tiếp tục khi có `OK: ket noi duoc PLC`:

```bash
cd /home/tinnt/Vinuni/src/hardware_interface/plc_bridge
./run_bridge.sh --execute
```

**Giữ Terminal 1 mở.** Thấy dòng `Swerve command bridge` thì sang bước 4.
Nếu có `fault`, `timeout`
hoặc `steering did not reach startup angles`, xử lý lỗi trước khi điều khiển.

> **Y8:** Mặc định chương trình không bật Y8. Ladder/cơ chế cho phép hiện có
> phải bật Y8 để BLDC hoạt động. Không tự đổi quyền điều khiển Y8 khi chưa
> xác nhận với người phụ trách PLC.

## 4. Terminal 2 — điều khiển bàn phím

Mở terminal mới, dán:

```bash
cd /home/tinnt/Vinuni/src/hardware_interface/plc_bridge
./run_keyboard.sh
```

Bấm chuột vào Terminal 2 rồi dùng **phím thường, không bật Caps Lock**:

| Phím | Chuyển động |
|---|---|
| `i` | Tiến |
| `,` | Lùi |
| `j` / `l` | Xoay trái / phải |
| `u` / `o` | Tiến và rẽ trái / phải |
| `Shift+j` / `Shift+l` | Đi ngang trái / phải |
| **Space hoặc `k`** | **Dừng** |

Giữ phím để chạy. Với terminal không hỗ trợ nhả phím, robot có thể nhận
lệnh thêm tối đa khoảng **0,75 giây** sau lần nhận phím cuối; dùng **Space**
để chủ động dừng. Không tăng tốc để khắc phục lỗi bánh không quay.

## 5. Terminal 3 — xem trạng thái (nên mở khi thử lần đầu)

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /plc_bridge/status
```

| Trường | Cần nhìn |
|---|---|
| `state` | `stopped`: đang yêu cầu dừng; `commanded`: có lệnh di chuyển |
| `actual_angle_deg` | Bốn góc Servo đọc từ PLC |
| `hall_raw` | Số đếm Hall đọc từ PLC |
| `overruns` | Tăng liên tục: truyền thông chưa đáp ứng 20 Hz |
| `error` | Bình thường là `null` |

`commanded` không xác nhận bánh đã quay. Tốc độ chưa hiệu chuẩn;
`hall_speed_m_s: null` là bình thường với cấu hình hiện tại.

## Dừng và chạy lại

1. Terminal 2: bấm **Space**, quan sát bánh dừng, rồi `Ctrl+C`.
2. Terminal 1: bấm `Ctrl+C` để chương trình thử dừng và đóng kết nối PLC.
3. Terminal 3: bấm `Ctrl+C`.

Lần sau chỉ làm lại **bước 3, 4**; mở bước 5 khi cần theo dõi.
Nếu mất mạng/Jetson mất điện mà bánh còn quay, dùng **dừng khẩn cấp độc lập**.

## Lỗi thường gặp

| Hiện tượng | Cách xử lý |
|---|---|
| `No such file or directory` | Kiểm tra dự án có ở `/home/tinnt/Vinuni`; nếu ở chỗ khác, sửa đường dẫn `cd` |
| Chưa có `.venv` hoặc thiếu module Python | Làm lại bước 2 |
| Không tìm thấy `/opt/ros/humble/setup.bash` | Cần cài ROS 2 Humble trước; nhờ người chuẩn bị máy |
| Kiểm tra TCP báo timeout/refused | Kiểm tra nguồn PLC, dây mạng, IP và cổng SLMP |
| Góc khởi tạo bị timeout | Đối chiếu Servo, D1020–D1023, D1030–D1033 và đơn vị góc; không bỏ kiểm tra góc |
| Servo chạy nhưng BLDC không quay | Kiểm tra Y8, sai lệch góc và hiệu chuẩn tốc độ với người phụ trách |
| Bridge báo lỗi truyền thông | Dừng robot, sửa kết nối rồi chạy lại Terminal 1 và 2 |

IP/cổng khác: sửa `plc.ip` và `plc.port` trong
[cấu hình](src/hardware_interface/plc_bridge/config/default.yaml), rồi chạy lại.

Chi tiết kỹ thuật: [tài liệu PLC](src/hardware_interface/plc_bridge/README.md).
