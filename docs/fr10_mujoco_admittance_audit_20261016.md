# Kiểm tra Admittance 6D FR10 trong MuJoCo — 16/10/2026

> Báo cáo lịch sử của các lần chạy trước khi đồng bộ Tool 1 ngày 16/10/2026.
> Các offset trong bằng chứng dưới đây không phải cấu hình triển khai hiện tại.
> Mốc Tool hiện tại dùng duy nhất
> [fr10_tool.yaml](../src/vehicle_description/config/fr10_tool.yaml);
> xem [báo cáo đồng bộ](../test_results/fr10_tool_sync_20261016/REPORT.md).

Giữ một bộ Admittance 6D với M/D/K riêng từng trục trong hệ lắp ghép.
Miền mong muốn là **norm tịnh tiến XYZ ≤5 mm và góc quay tương đối ≤5°**.
Các giá trị này là miền kiểm thử, không phải sai lệch cố định và chưa phải
miền bắt cổng đã được chứng minh. Không cần phân loại sai lệch từ robot hay cổng.

## Phạm vi và phép đo

- Dùng FR10 cố định, cùng controller/IK/MotionExecutor và đầu cắm/cổng CAD gốc.
- Cổng vật lý giữ nguyên; chỉ pose quan sát đưa vào controller bị làm sai.
- MuJoCo bước 1 ms, lực tiếp xúc vật lý và mô-men được quy về `charging_tcp`.
- ROS domain riêng 213–217 cho các phép đo đã chạy; không gửi lệnh tới domain 0.
- Thành công phải gồm kết quả action, HOLD đủ 5 giây mô phỏng, kiểm tra khớp
  CAD độc lập, độ sâu thật, rút ra và xác nhận dừng trước khi tắt physics.
- Giữ nguyên giới hạn lực 50 N, mô-men 10 Nm, penetration 50 µm, sai lệch
  hình học keyed fit 250 µm và sai lệch độ sâu 500 µm.

## Kết quả kiểm tra code

| Thành phần | Kết luận / thay đổi |
| --- | --- |
| `force_position_controller.cpp` | `desired − measured` đúng với quy ước **lực đầu cắm tác dụng lên môi trường** của bridge. Correction tịnh tiến được quay từ hệ lắp ghép sang base; rotation dùng exponential map/SO(3), không cộng Euler. Thêm kiểm tra cả dấu ± trên sáu trục và hệ quay 90°. |
| `admittance_model.cpp` | M phải dương, D/K không âm; deadband áp dụng sau lực lò xo, rồi giới hạn vận tốc/gia tốc và khoảng hãm trong envelope. Trạng thái chỉ tích lũy khi proposal được chấp nhận/commit. Không tìm thấy lý do để đảo dấu lực toàn bộ hoặc đổi sang ma trận 6×6. |
| `wrench_transform.cpp` | Dùng `F_T = R F_S`, `τ_T = R τ_S + r × F_T`. Bridge MuJoCo đã quy về TCP nên không dịch mô-men thêm lần nữa. |
| `wrench_filter.cpp` | Bộ lọc 1 Hz dùng thời gian nguồn; hằng số thời gian khoảng 0,159 giây mô phỏng. Không đổi sang clock thực để che simulation chậm. |
| Envelope | Phát hiện giới hạn dò XY 2,5 mm bị ẩn trong cấu hình. Đưa ra tham số `force.search.xy_max_coordinate_m`, giữ mặc định cũ; profile MuJoCo mở lên 7,5 mm. Correction từng trục/norm được mở riêng trong profile thử, rotation norm 6°. |
| ServoJ / executor | Force tạo corrected TCP → seeded IK → joint-only executor. Mốc độ sâu dựa feedback thực, correction chỉ commit sau khi đường thực thi chấp nhận. Không thêm chủ actuator hay controller thứ hai. |
| `contact_detector.cpp` | Bổ sung hysteresis/dwell độc lập cho lực ngang, mô-men nghiêng và xoắn. Feedback có `contact_lateral`, `contact_tilt`, `contact_torsion`. Lực ngang/nghiêng **không** được dùng thay bằng chứng tiếp xúc dọc trục hoặc xác nhận cắm xong. |

Trong các trường contact, `0` là chưa tiếp xúc, `1` là nghi có tiếp xúc,
`2` là đã xác nhận sau dwell. Nhãn log `FORCE | <pha>` mô tả phiên lực;
POSE/JOINT vẫn là các bước thực thi bên trong cùng pipeline.

### Tâm tuân thủ / Virtual RCC

Trong các phiên ghi nhận ở báo cáo này, TCP nằm ở **mũi đầu cắm**, và rotation
correction giữ nguyên tọa độ TCP khi không có translation correction.
Kiểm tra URDF/CAD của các phiên cũ cho thấy:

- `ft_sensor_link → charging_tcp`: 39 mm theo Z.
- `charging_connector_link → charging_tcp`: 39 mm theo Z, trùng mốc mũi CAD.
- `plate_frame → charging_tcp`: `(0, 90, 129)` mm, khoảng cách 157,293 mm.

Vì vậy không thêm cánh tay đòn RCC giả định 100 mm. Nếu đổi tâm tuân thủ sau này,
phải đổi nhất quán cả wrench và pose/twist quanh tâm đó. Bằng chứng:
`test_results/fr10_mujoco_5mm_5deg_20261016/tcp_pivot_audit.json`.

## Vì sao có lúc gần như đứng yên

Profile đã chứng minh ổn định trước đây dùng `D_z = 100000 N·s/m`, deadband Z
0,5 N và giới hạn correction Z 0,025 mm/s. Với lực locating 2 N trong không khí,
vận tốc correction xấp xỉ `(2 − 0,5)/100000 = 0,015 mm/s` **theo clock mô phỏng**.
Đây khác với tốc độ nominal tiếp cận 1 mm/s. Khi nominal dừng nhưng chưa chạm,
chỉ correction rất chậm này còn đẩy robot đi.

Tốc độ free đo riêng với profile MuJoCo đạt P95 khoảng **9,94 mm/s thực**,
cực đại cửa sổ 2 giây **9,99 mm/s**, RTF gần 1. Đoạn tiếp cận không tải của ca
calibrated đạt P95 khoảng **0,93 mm/s thực**. Khi CAD tiếp xúc nhiều tam giác,
RTF có thể chỉ khoảng 0,15–0,17; không thể bảo đảm 1 mm/s thực ở mọi pha force.

Đã vector hóa cộng wrench tiếp xúc, vẫn giữ từng `mj_contactForce`, mô-men
tiếp xúc và cánh tay đòn. Benchmark 50 contact cho phần cộng wrench nhanh hơn
khoảng 8,5 lần, kết quả trùng phép cộng độc lập; **không phải toàn bộ physics
nhanh hơn 8,5 lần**. Thử threadpool MuJoCo không cải thiện đáng kể nên không bật.

## Bằng chứng thử nghiệm

Kết quả từng ca, biểu đồ lực/độ sâu thật/lệch ngang/góc/RTF và lỗi timeout nằm tại:

- `test_results/fr10_mujoco_passive_20261016/` — không spiral/dò XY/yaw chủ động.
- `test_results/fr10_mujoco_passive_mid_20261016/` — mức trung gian 1 mm / 1°.
- `test_results/fr10_mujoco_search_5mm_20261016/` — đối chiếu bounded search sẵn có.
- `test_results/fr10_mujoco_capture_20261016/` — regression trước ở 2 mm / 2°,
  bao gồm cả các candidate bị hủy; không loại các ca thất bại khỏi lịch sử.

Ca passive đồng thời norm **0,5 mm / 0,5°** đã PASS đầy đủ, kết quả sau
**273,47 giây thực**. Sai lệch keyed fit lớn nhất trong HOLD **215,73 µm**,
penetration **13,20 µm**, lực thô cực đại **4,42 N**, mô-men cực đại **0,113 Nm**.
Khoảng dư so với clearance 250 µm còn nhỏ; không suy rộng kết quả này lên 5 mm.

Các regression active search trước đây X−2 mm, Ry−2°, đồng thời X−2 mm/Ry−2°
đã PASS đủ cắm/HOLD/rút; thời gian kết quả lần lượt khoảng 468, 441, 430 giây thực.
Đây là kết quả với tìm kiếm có giới hạn, không phải bằng chứng passive ở 5 mm.

Các ca passive đã kết thúc (mỗi cấu hình một lần, không phải ước lượng thống kê):

| Sai lệch quan sát | Kết quả | Thời gian tới kết quả / giới hạn thực | Lực / mô-men thô đỉnh |
| --- | --- | --- | --- |
| Chéo 0,5 mm / 0,5° | PASS đủ HOLD/rút | 273,47 s | 4,42 N / 0,113 Nm |
| Chéo 1 mm / 1° | PASS đủ HOLD/rút | 292,49 s | 4,42 N / 0,158 Nm |
| X−5 mm | `NO_PROGRESS`, độ sâu thật 2,01 mm | 892,03 s | 3,96 N / 0,099 Nm |
| Ry−5° | `NO_PROGRESS`, chưa đủ kiểm tra ổn định | 787,59 s | 6,47 N / 0,255 Nm |
| X−5 mm và Ry−5° | `NO_PROGRESS`, độ sâu thật 2,01 mm | 841,04 s | 3,83 N / 0,120 Nm |
| Z+5 mm | Timeout, báo độ sâu khoảng 10 mm | 300 s | 4,22 N / 0,190 Nm |
| Z−5 mm | `FORCE_SAFETY`, sai mốc độ sâu | 263,33 s | 0 N / 0 Nm |

Tất cả bảy ca đã xác nhận controlled HOLD và tắt motion trước physics.
Ca 1 mm / 1° có keyed fit lớn nhất **237,51 µm**, penetration **13,12 µm**;
khoảng dư còn khoảng 12,49 µm, nên cần lặp lại trước khi suy rộng độ bền vững.
Ca Ry−5° gần tới vị trí cuối nhưng sai lệch keyed fit sau dừng khoảng **263 µm**,
vượt giới hạn 250 µm; không gán PASS. Ca Z−5 mm dừng ở độ sâu thật khoảng
10,50 mm dù controller đo 15,50 mm, không có lực tiếp xúc.

Hai giới hạn đã quan sát được ở miền 5 mm:

1. **Không có thông tin chỉ hướng trên mặt phẳng.** Ca X−5 mm dừng ở khoảng
   2 mm độ sâu, Fz khoảng 3,36 N nhưng lực ngang và mô-men gần bằng 0.
   Ca đồng thời X−5 mm/Ry−5° có thể hết nghiêng rồi vẫn tì ngoài miệng.
   Với equilibrium ngang bằng 0, Admittance không biết tâm lỗ ở phía nào.
   Tăng M/D/K hoặc tăng lực dọc trục không cung cấp thông tin bị thiếu.
2. **Sai mốc độ sâu khi pose lỗi dọc trục.** Với Z+5 mm, vai đầu cắm đã gần
   chạm độ sâu thật 15 mm nhưng controller chỉ đo khoảng 10 mm; ca này timeout
   sau 300 giây thực và được dừng có xác nhận. Với Z−5 mm, nominal báo 15 mm
   khi đầu cắm thật mới khoảng 10 mm, còn ở trong không khí. Không mở tolerance
   để hợp thức hóa kết quả này.

`matrix_summary.json` tính cả timeout trong mẫu số tỷ lệ thành công, phân biệt
số action trả kết quả với số lần thử đã kết thúc. Tỷ lệ chỉ mô tả các ca đã thử,
không chứng minh toàn bộ miền liên tục. Các ca đang chạy chưa có summary không
được báo PASS.

Bộ tổng hợp bảy ca passive, giữ liên kết tới dữ liệu gốc, nằm ở
`test_results/fr10_mujoco_admittance_20261016/passive/`:
`force_alignment.png`, `wrench_components.png`, `matrix_summary.json`.
Kết quả quan sát của tập này là **2/7**, không phải xác suất thành công của
phân bố sai lệch bất kỳ hay bảo đảm miền 5 mm / 5°.

Ca search đầu tiên đã vào tới độ sâu khoảng 15 mm nhưng bộ quan sát bị ngắt
theo lượt hội thoại; không đủ bằng chứng HOLD/fit/rút nên không tính là thành công.
Đã xác nhận dừng/tắt riêng trong `interrupted_observer.json`. Ca chạy lại nằm ở
`test_results/fr10_mujoco_search_5mm_repeat_20261016/`.

**Ca chạy lại X−5 mm với bounded search đã PASS đầy đủ**: nhận kết quả sau
**803,28 giây thực**, HOLD đủ 5 giây nguồn (37,49 giây thực), rút ra về
preinsert và xác nhận dừng/tắt đúng thứ tự. Keyed fit lớn nhất trong HOLD
**66,49 µm**, penetration đỉnh **21,29 µm**, độ sâu thật khoảng **15,010 mm**.
Lực thô cực đại toàn ca **25,06 N**, mô-men thô cực đại **0,746 Nm**:
đây là tải đỉnh, khác với lực giữ khoảng 4,4 N. Chưa thể coi profile tìm kiếm
này đã tối ưu lực đỉnh hoặc tốc độ. Biểu đồ và dữ liệu đầy đủ nằm ở thư mục
chạy lại nêu trên. Kết quả này chỉ chứng minh một ca lệch ngang 5 mm có tìm
kiếm, không chứng minh passive hoặc tổ hợp 5 mm / 5°.

## Kiểm tra phần mềm

Runtime `rb_v3_humble_20261016T084257Z.eXsHKg`: 16 package build thành công;
**282 tests, 0 errors, 0 failures, 21 skipped**; thêm **43 kiểm tra Python đạt**.
Lần CTest đầu trong sandbox bị chặn `getifaddrs` ở các phép thử Gazebo; chạy lại
trên host với localhost đã đạt. Runtime chỉ được công bố sau lần kiểm tra này.
Hash profile và đường dẫn runtime được lưu trong
`test_results/fr10_mujoco_5mm_5deg_20261016/runtime_publication_final.json`.

## Công việc tiếp theo để đạt miền mong muốn

1. Giữ correction pipeline và tâm TCP đã kiểm tra; dùng biểu đồ để phân biệt
   nominal tiến, correction tiến, saturation và mất contact.
2. Với sai lệch Z, thiết kế mốc độ sâu dựa tiếp xúc **có kiểm chứng** và khoảng
   tiếp cận hữu hạn. Không coi va mặt ngoài là mốc miệng cổng hợp lệ; không dùng
   ground truth MuJoCo làm đầu vào điều khiển.
3. Với miền ngoài dẫn hướng, đánh giá bounded reference search hiện có sau
   passive test. Đây vẫn là cùng một Admittance; hoặc cần cải thiện lead-in /
   định vị để đưa đầu cắm vào vùng có thông tin tiếp xúc.
4. Sau khi hai giới hạn trên được xử lý, quét cả dấu ±, các hướng chéo, yaw,
   các mức sai lệch trung gian và lặp lại nhiều lần. Báo tỷ lệ thành công,
   thời gian thực/mô phỏng, lực/mô-men đỉnh và sai lệch hình học độc lập.

Không tăng force để bù RTF thấp hoặc rút ngắn thời gian. Cần tách tối ưu
collision/physics khỏi thay đổi đáp ứng force; mỗi candidate gain phải được
kiểm tra độ ổn định và fit độc lập trước khi thay profile đã đo.

## Chạy lại và quan sát

Từ workspace đã source ROS và `install/setup.bash`:

```bash
/usr/bin/python3 src/vehicle_bringup/test/integration/run_fr10_capture_mujoco.py \
  --mode passive --cases combined_diagonal_0p5 translation_x_minus5 tilt_ry_minus5
```

Lệnh tạo output mới, mặc định ROS domain 212. Có thể quan sát bằng:

```bash
ROS_DOMAIN_ID=212 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 \
  src/vehicle_bringup/scripts/fr10_force_plot.py
```

Muốn đối chiếu bounded search, dùng `--mode search`. Cách xem robot/cổng trực
tiếp và phát lại feedback đã đo nằm trong README của thư mục integration.
