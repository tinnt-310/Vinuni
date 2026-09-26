# Hardware backends

- fr10_driver/: plugin FAIRINO và message; vendor_reference/ giữ tài liệu và Git metadata gốc, không phải controller bổ sung.
- plc_bridge/: chương trình PLC Python từ src/plc; chưa phải ros2_control SystemInterface.
- config/: cấu hình dùng chung nếu cần. Mapping thanh ghi hiện giữ tại plc_bridge/config để tránh nhân đôi.

Điều khiển cấp cao FR10 ở ../fr10_arm_control; mô hình ở ../vehicle_description. Không chạy run.sh để kiểm thử offline: script bật --execute.
