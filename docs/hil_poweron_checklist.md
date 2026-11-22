# Checklist Đóng Điện & Kiểm Thử HIL (Mục A.13 — CdCT Final 2.0)

> Tài liệu khung chuẩn bị cho bước HIL khi có đầy đủ linh kiện (L0 Pico 2, L1 Pixhawk 6C, L2 Pi 5, OAK-D Pro W, 3× ESC AM32, C6 Shunt + BMS BLE, C8 LoRa).

## 1. Kiểm tra phần cứng & đấu nối trước đóng điện
- [ ] Xác nhận bo an toàn C8 (Pico 2) cấp nguồn qua watchdog phần cứng ngoài 100 ms, trạng thái khởi động `SAFE_OFF`.
- [ ] Xác nhận đường đo dòng/áp chính dùng **shunt** nối về Pixhawk 6C (không dùng cảm biến ACS, không dùng BMS UART/CAN).
- [ ] Xác nhận kết nối Bluetooth LE tới BMS trên Pi 5 (`energy`), kiểm tra kịch bản mất BLE chỉ cảnh báo không dừng bè (TBD-23).
- [ ] Xác nhận 3× ESC **Holybro Tekko32 F4 45A (AM32)** đấu đúng kênh Trái / Phải / Dự phòng đuôi, giới hạn 30 A/ESC, PWM 24 kHz, về 0 khi mất tín hiệu.
- [ ] Xác nhận camera **OAK-D Pro W** kết nối cổng **USB 3**, gắn cao `20 cm` trên boong, trục quang nằm ngang (`alpha_tilt = 0°`), không có LiDAR.
- [ ] Xác nhận GNSS 2 ăng-ten định hướng mũi (không dùng la bàn từ).

## 2. Kiểm tra chức năng & an toàn bo C8 (30 lần E-stop — Mục A.13)
- [ ] Kiểm thử keepalive C4 -> C8 (1 Hz): mất 3 nhịp liên tiếp (= 3 s) kích hoạt `HOLD_REQ` mà không cắt cứng nguồn.
- [ ] Kiểm thử cờ `BRIDGE_ZONE` (làm mới <= 60 s): tạm ức chế cắt tự động do nghiêng/tải, **không** chặn nút E-stop tay hoặc gói LoRa `STOP`.
- [ ] Thực hiện tối thiểu **30 lần E-stop** (kết hợp nút nhấn tại bè + gói LoRa 12 byte `STOP` có `COUNTER` + `HMAC_TRUNC` ở dải 920–923 MHz):
  - Tỷ lệ cắt động lực thành công: `___ / 30` (bắt buộc 100%).
  - Xác nhận trạng thái `STOPPED` chốt cứng (latched), chỉ mở khóa lại bằng thao tác thủ công trực tiếp tại bè (EN ISO 13850).

## 3. Đo hiệu năng & ngân sách độ trễ P99 (Mục A.11 & TBD-24 / TBD-30)
- [ ] Đo chuỗi độ trễ TR-4.1 với **Config A (400P depth, 15 Hz)** (yêu cầu P99 <= 200 ms, `perception_geom` <= 50 ms).
- [ ] Kiểm tra **Config B (800P depth)** chỉ kích hoạt tại trạng thái `S4` đo tĩnh không cầu ở tốc độ <= 2 km/h.
- [ ] Đo tải CPU & nhiệt độ Pi 5 (ngưỡng giảm xung nhiệt 80 °C, TBD-24) và kiểm tra `chrony` chế độ `slew-only`.
