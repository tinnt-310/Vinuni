# Dữ liệu thí nghiệm và kết quả dọn dẹp

Kiểm kê ngày 17/10/2026. Không xóa dữ liệu đo, source, launch, YAML hoặc URDF.
Không tạo thư mục archive báo cáo, không chuyển báo cáo cũ, không viết lại lịch sử Git.

## Dữ liệu trên máy và trong Git

| Vị trí | File thường trên máy | Dung lượng nội dung | Git trước → sau |
|---|---:|---:|---:|
| `docs/ode_debug/` | 4.093 | 3.580,8 MiB | 2.201 → 1.683 file |
| `test_results/` | 79.478 | 81,99 GiB | 0 → 0 file |

`test_results/` còn có 37 liên kết tượng trưng; kiểm kê không đi theo chúng.
Dung lượng nội dung khác dung lượng đĩa cấp phát do kích thước block filesystem.
Kết quả phản ánh thời điểm kiểm kê; các lần chạy thí nghiệm mới có thể làm tăng số liệu.

518 file JSONL trong `docs/ode_debug/`, tổng **458.831.133 byte (437,58 MiB)**,
được bỏ khỏi Git index và thêm vào `.gitignore`, nhưng giữ nguyên tại đường dẫn
cũ trên máy. Tóm tắt JSON, metadata, chương trình tái hiện và các bản chụp
model/config vẫn được theo dõi. Cách này giảm dung lượng cây file của commit
mới; dữ liệu vẫn tồn tại trong lịch sử nên không làm bản clone đầy đủ nhỏ đi
tương ứng. Không chạy `filter-repo`, prune reflog hoặc force push.

## Phân loại bảo toàn

| Nhóm | Số file | Quyết định |
|---|---:|---|
| Tài liệu thí nghiệm | 18 | Giữ |
| Summary, result, measured result và manifest JSON | 1.429 | Giữ |
| Source và bản chụp config/model | 2.134 | Giữ |
| Dữ liệu đo hoặc metadata khác | 13.347 | Giữ; không coi dữ liệu đo là có thể tạo lại chỉ vì nó là log |
| Hình, tài sản mô hình và gói bằng chứng | 65.810 | Giữ |
| Cache/sản phẩm build cần kiểm tra | 606 | Giữ; chưa xác minh đầy đủ cách tạo lại và phụ thuộc |
| Chưa rõ vai trò | 227 | Giữ |

Phân loại dựa trên vai trò tên file, phần mở rộng và tham chiếu tĩnh trong các
file Git đang theo dõi. Không có tham chiếu tĩnh **không** chứng minh file không
được script bên ngoài hoặc quy trình nghiên cứu sử dụng. Không file thí nghiệm
nào được xác nhận đủ điều kiện xóa trong lượt này. Các bản `.orig`/`.rej` được
giữ vì chưa chứng minh tương đương với bản hiện hành.

Các nhóm chiếm nhiều dung lượng nhất trong `test_results/`:

| Nhóm | Dung lượng nội dung gần đúng | Quyết định |
|---|---:|---|
| `fr10_admittance_pose_error_20261003` | 60,81 GiB | Giữ dữ liệu và metadata nghiên cứu |
| `fr10_e2e_20261002` | 7,45 GiB | Giữ bằng chứng kiểm chứng |
| `fr10_mujoco_passive_20261016` | 4,60 GiB | Giữ |
| `fr10_mujoco_capture_20261016` | 3,59 GiB | Giữ |
| `fr10_speed_20261016` | 1,23 GiB | Giữ; có liên kết từ tài liệu vận hành |

Riêng `stage43/gazebo_yaw2_cached_125us/transport_contacts.jsonl` trong nhóm
admittance khoảng 15,78 GiB. Chưa có bằng chứng rằng dữ liệu đo này là bản sao
hoặc có thể tái lập chính xác nên không xóa/nén/đổi đường dẫn.

## Kiểm kê lại mà không thay đổi dữ liệu

Chạy từ thư mục gốc Vinuni:

```bash
mkdir -p .local-data/maintenance
python3 scripts/maintenance/audit_artifacts.py \
  > .local-data/maintenance/inventory.json
```

Script chỉ đọc source và artifact, xuất JSON ra stdout, trả mã lỗi nếu đọc file
thất bại. Nó không xóa, di chuyển, thay Git index hoặc giao tiếp với robot.
Mỗi file có đường dẫn, kích thước, mtime, vai trò, trạng thái Git và danh sách
tham chiếu tĩnh tìm được. Symlink được ghi nhận mà không đi theo đích.

Kiểm kê đầy đủ trước/sau lượt dọn này nằm trên máy tại
`.local-data/maintenance/cleanup-20261017/inventory-before.json` và
`inventory-after.json`. Chúng không được thêm vào Git để tránh đưa hàng chục
nghìn bản ghi cục bộ vào repository.

Đối chiếu hai lần kiểm kê không có đường dẫn dữ liệu bị xóa/thêm. Hai log
trong `test_results/fr10_6dof_smoke_20261016_164734/aligned/` tăng kích thước
trong lúc kiểm kê; các file còn lại giữ nguyên kích thước, mtime và đích
symlink. Không dừng tiến trình đang ghi log để làm cho hai bản kiểm kê giống nhau.

## Khôi phục các luồng JSONL trên máy mới

[Manifest JSONL](ode_debug_raw_manifest.tsv) ghi tên, số byte, SHA-256, Git blob
và commit khôi phục của từng file. Toàn bộ 518 file đã được kiểm tra khớp byte
với commit đã công bố **050d99d5b19466e1ffeedcff09174c8ddd661580** trước khi
bỏ theo dõi. Đây là dữ liệu lịch sử được bảo toàn trong GitHub, không phải bản
sao lưu ngoài hệ thống Git. Các file chỉ có trong `test_results/` vẫn cần kế
hoạch sao lưu nghiên cứu riêng trước khi xem xét xóa.

Với bản clone chứa đầy đủ lịch sử, có thể khôi phục bản chụp lịch sử vào thư
mục riêng, không ghi đè kết quả hiện tại:

```bash
mkdir -p .local-data/recovery-050d99d
git archive 050d99d5b19466e1ffeedcff09174c8ddd661580 docs/ode_debug \
  | tar -x --no-same-owner -C .local-data/recovery-050d99d
```

Lệnh khôi phục cả bản chụp `docs/ode_debug/` ở commit đó. Đối chiếu các file
JSONL với manifest trước khi đưa vào luồng phân tích. Bản clone nông cần lấy
thêm lịch sử để truy cập commit khôi phục. Không xóa hoặc viết lại lịch sử
chứa commit này khi chưa sao lưu dữ liệu và xác minh bản sao độc lập.

## Dung lượng Git cục bộ

Đã loại bỏ **40 file `tmp_pack_*` bị bỏ lại, 8.007.008.562 byte (7,46 GiB)**.
Git báo chúng là `garbage`; tất cả đã hơn 18 giờ tuổi và không có tiến trình
mở. Trước khi xóa đã kiểm tra inode/kích thước/mtime, khóa Git và các refs.
Không xóa pack/object hợp lệ. Kiểm tra kết nối object Git trước/sau đạt;
các refs giữ nguyên sau khi dọn file tạm.

Đây là dung lượng giải phóng thật trên máy. Phần lịch sử Git hợp lệ còn lại
được giữ nguyên. Các bản build đang dùng và liên kết `ros2_current`, `build`,
`install`, `log` không được thay đổi.

## Kiểm chứng lượt dọn

- Build độc lập tại `/tmp/vinuni-cleanup-audit/verify`: 16 package hoàn tất.
  Không chạy script xuất bản runtime và không đổi `ros2_current`.
- `colcon test-result`: 318 mục kiểm thử, 0 lỗi, 0 thất bại, 21 bỏ qua.
  Kết quả này là tổng hợp của colcon; không phải chứng nhận vận hành robot thật.
- Bộ unittest PLC: 70 bài kiểm thử đạt, không giao tiếp PLC thật.
- Danh sách package và checksum source/config thực thi trước/sau giống nhau.
  Các thay đổi dưới `src/` chỉ thuộc tài liệu Markdown.
- 518 checksum SHA-256 của JSONL trên máy vẫn khớp manifest sau khi bỏ theo dõi.
- Đã sửa 11 đích liên kết cũ bị hỏng và ghi 41 tham chiếu dữ liệu chỉ có trên
  máy dưới dạng đường dẫn `local`, tránh tạo liên kết 404 trên GitHub. Kiểm tra
  các liên kết Markdown cục bộ còn lại: không có đường dẫn/heading bị hỏng.
  Không sửa launch, model hoặc config.
- `git diff --cached --check` và kiểm tra tính toàn vẹn Git đạt.

Log build/test và đối chiếu checksum được giữ cục bộ trong
`/tmp/vinuni-cleanup-audit/`; bản kiểm kê và danh sách thay đổi được giữ tại
`.local-data/maintenance/cleanup-20261017/`.
