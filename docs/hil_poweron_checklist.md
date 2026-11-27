# Checklist Đóng Điện & Kiểm Thử Bàn Thử HIL (Giai đoạn 6 & 7 — CdCT Final 2.0)

> **Cảnh báo an toàn bắt buộc:** THÁO TOÀN BỘ CHÂN VỊT khỏi 3 thruster trước mọi bài kiểm thử trên bàn HIL. Luôn đặt nút nhấn E-stop vật lý của bo C8 trong tầm tay trước khi cấp nguồn động lực.

---

## 1. Thứ tự bật nguồn từng khối (Bắt buộc tuân thủ đúng trình tự)

### Bước 1 — Pixhawk 6C (Tầng L1)
- [ ] Cấp nguồn logic/USB cho Pixhawk 6C (chưa cấp nguồn động lực ESC).
- [ ] Nạp ArduPilot Rover (`APMrover2-stable` / `Rover-4.7.3`, SHA `d9c5097c3d8333435712ffed86bca8ce00c4bb46`) và bộ tham số `l1_ardupilot/params/sitl_boat_a7.parm` (điều chỉnh cổng UART thực tế `>= 921.600 baud`).
- [ ] Chép 2 script Lua `l1_ardupilot/lua/nenuphar_thruster_power.lua` và `nenuphar_watchdog_bridge.lua` vào thẻ nhớ `APM/scripts/`.
- [ ] Xác nhận trên QGroundControl: `FRAME_CLASS=2`, `SCHED_LOOP_RATE=50`, `COMPASS_USE=0` (GNSS 2 ăng-ten), `FS_TIMEOUT=3.0`, `FS_ACTION=2` (Hold), `FS_GCS_ENABLE=1`, `BATT_MONITOR=4`, `BATT_LOW_VOLT=12.2` (`BATT_FS_LOW_ACT=1` RTL), `BATT_CRT_VOLT=11.5` (`BATT_FS_CRT_ACT=2` Hold), `FENCE_ENABLE=1` (`FENCE_ACTION=2` Hold), và thấy `STATUSTEXT` 2 script Lua đã nạp thành công.
- [ ] Thử chuyển tay sang chế độ `MANUAL (0)` / `STEERING (3)` khi đang kích hoạt khóa cầu `BRG_LOCK` hoặc watchdog L2: xác nhận Lua **không bao giờ ép đè** lên quyền điều khiển tay của người vận hành.

### Bước 2 — Raspberry Pi 5 (Tầng L2)
- [ ] Cấp nguồn 5 V ổn định cho Raspberry Pi 5 (8 GB, NVMe 256 GB, Ubuntu 24.04 LTS + ROS 2 Jazzy).
- [ ] Xác nhận đồng bộ thời gian `chrony` ở chế độ **`slew-only`** (cấm bước nhảy thời gian làm hỏng timestamp VIO/A.11):
  ```bash
  chronyc tracking && grep -E "makestep|maxslewrate" /etc/chrony/chrony.conf
  ```
- [ ] Gán `SCHED_FIFO` cho 4 node trọng yếu (`camera_driver`, `vio`, `perception_geom`, `mavlink_bridge`), giữ `blackbox` ở ưu tiên thấp.
- [ ] Bật kết nối UART MAVLink 2 có ký (`>= 921.600 baud`) giữa Pi 5 và Pixhawk 6C; xác nhận chuỗi `system_monitor -> mavlink_bridge -> Pixhawk` nhận `HEARTBEAT` ổn định 1 Hz.

### Bước 3 — Camera OAK-D Pro W (Cảm biến thị giác)
- [ ] Cắm camera OAK-D Pro W vào cổng **USB 3** trên Pi 5 (gắn cố định cao `20 cm` trên boong, trục quang nằm ngang `alpha_tilt = 0°`, không LiDAR).
- [ ] Xác nhận tốc độ đàm phán USB 3 (`5000M`):
  ```bash
  lsusb -t | grep -E "5000M|Myriad|Movidius"
  ```
- [ ] Thử tiêm lỗi rút sang cổng USB 2: xác nhận `system_monitor` báo `usb3_camera_ok=false` và `mission_fsm` **từ chối rời `S0`**.
- [ ] Xác nhận luồng ảnh thực tế (TBD-21): `15 Hz` depth (`Config A 400P`) + `15 Hz` stereo VIO + `200 Hz` IMU với QoS `depth=1` (`BEST_EFFORT`).

### Bước 4 — 3× ESC Holybro Tekko32 F4 45A (AM32) & Thruster (CHƯA LẮP CHÂN VỊT)
- [ ] Kiểm tra lại **đã tháo chân vịt** cả 3 thruster (Trái, Phải, Dự phòng đuôi).
- [ ] Xác nhận cấu hình AM32 trên cả 3 ESC: Bidirectional DShot + UART telemetry, giới hạn **`30 A/ESC`**, PWM **`24 kHz`**, về `0` khi mất tín hiệu.
- [ ] Kiểm tra đủ telemetry `ESC_TELEMETRY_1_TO_4` cho **3 ESC** và kiểm tra chiều quay + trộn lực đẩy `M1–M4` khi tiêm lỗi mất 1 động cơ.

### Bước 5 — Hệ thống nguồn C6 (Shunt chính + BMS Bluetooth LE)
- [ ] Xác nhận **shunt chính** nối về Pixhawk 6C làm nguồn đo dòng/áp chính (không dùng BMS UART/CAN, không dùng cảm biến dòng ACS).
- [ ] Đối chiếu điện áp/dòng giữa shunt và đồng hồ kìm; ghi hệ số hiệu chuẩn vào `docs/tbd20_tuning_tracker.csv`.
- [ ] Kiểm tra kết nối **Bluetooth LE** từ Pi 5 (`energy`) tới BMS; thử tắt/ngắt BLE (TBD-23) và xác nhận hệ thống **chỉ cảnh báo, không dừng bè**.
- [ ] Xác nhận phân tầng ngưỡng pin: phần mềm L1 `BATT_CRT` tại `11,5–12,0 V` (hoặc `SoC <= 10%`), cao hơn ngưỡng cắt cứng BMS (`10,0 V`).

### Bước 6 — Bo an toàn C8 (Raspberry Pi Pico 2) & LoRa 12 byte
- [ ] Nạp firmware `l0_c8_pico2` vào Pico 2; xác nhận khởi động ở trạng thái `SAFE_OFF` (rơ-le động lực mở) và watchdog phần cứng ngoài `100 ms` hoạt động (chỉ đá khi chu kỳ đọc cảm biến khỏe mạnh).
- [ ] Kiểm tra quy tắc **C4 không tự đóng contactor (A.10):** cấp keepalive `1 Hz` từ C4 không làm đóng rơ-le; chỉ chuyển `SAFE_OFF -> ARMED` khi có đủ **sườn lên nút Start vật lý tại bè + gói LoRa `RUN` còn hạn (`<= 30 s`) + E-stop đã nhả**.
- [ ] Kiểm tra **bắt sườn nút Reset (EN ISO 13850):** nhấn giữ kẹt nút Reset rồi nhả E-stop -> xác nhận C8 vẫn giữ chốt cứng `STOPPED` và không tự đóng lại rơ-le.
- [ ] Kiểm tra keepalive C4 -> C8 (`1 Hz`): rút dây/dừng keepalive `3 s` -> xác nhận C8 chuyển `HOLD_REQ` (giữ rơ-le đóng, phát tín hiệu giữ vị trí).
- [ ] Kiểm tra 3 ngưỡng **TR-8.3 & lỗi cảm biến fail-safe:** nghiêng động `> 15°` (`>= 200 ms`), nghiêng tĩnh `> 3°` (`> 1 s`), thay đổi tải `>= 10 kg` sau trừ bì; rút dây MPU-6050/HX711 (`valid=false`/`NaN`/quá hạn `> 500 ms`) -> xác nhận chặn arming ở `SAFE_OFF` và ngắt `STOPPED` khi đang `ARMED`.
- [ ] Kiểm tra cờ `BRIDGE_ZONE` (`<= 60 s`): tạm ức chế cắt tự động do nghiêng/tải TR-8.3, nhưng **tuyệt đối không chặn** nút E-stop tay, LoRa `STOP`, `4G STOP` hay `Watchdog STOP`.
- [ ] Kiểm tra chống phát lại bền vững qua reboot trên gói LoRa 12 byte (`920–923 MHz`): tắt/bật nguồn C8 rồi phát lại gói cũ -> bị từ chối; reboot máy phát với bước nhảy `+1000` -> chấp nhận `STOP` ngay lập tức.

---

## 2. Bộ lệnh kiểm thử HIL bắt buộc ngay khi lắp bàn thử

```bash
# 1. Đo độ trễ P99 toàn chuỗi A.11 (yêu cầu TR-4.1 <= 200 ms với Config A 400P)
source /opt/ros/jazzy/setup.bash && source ~/nenuphares-sw/l2_ros2_ws/install/setup.bash
ros2 run nenuphar_nodes latency_probe --samples 100 --timeout 20

# 2. Phân tích log rosbag2 từ nút blackbox (1 Hz thường / >= 10 Hz sự kiện hoặc S4)
python3 ~/nenuphares-sw/tools/analyze_blackbox_bag.py /path/to/blackbox_bag

# 3. Giám sát nhiệt độ & tải CPU Pi 5 (TBD-24: cảnh báo ngưỡng giảm xung 80 °C)
vcgencmd measure_temp && vcgencmd get_throttled
```

---

## 3. Bảng nghiệm thu HIL bắt buộc (Mục A.13)

| Hạng mục nghiệm thu | Tiêu chí đạt | Kết quả đo thực tế | Đạt / Chưa đạt |
| :--- | :--- | :--- | :---: |
| Độ trễ P99 & Max tránh vật cản (Config A 400P — TBD-30) | `P99 <= 200 ms` (`perception_geom <= 50 ms`); ghi nhận cả `Max` | `P99=___ / Max=___ ms` | `[ ]` |
| Nhiệt độ & giảm xung Pi 5 (TBD-24, môi trường 55 °C) | Không vượt 80 °C; bật cờ tắt B2 nếu `perception_geom > 50 ms` | `____ °C` | `[ ]` |
| Thử nghiệm E-stop (nút nhấn tay + LoRa `STOP` 920–923 MHz) | Tối thiểu **30/30 lần cắt 100%**, chốt cứng `STOPPED`, chỉ reset tay tại bè | `___ / 30` | `[ ]` |
| Tiêm lỗi mất USB 3 / mất BLE BMS / mất heartbeat L2 / sai LoRa | Đạt 4/4 kịch bản đúng luật Final 2.0 | `___ / 4` | `[ ]` |
| Log `blackbox` (`rosbag2`) | `1 Hz` thường, `>= 10 Hz` tại `S4`/lỗi, dự phóng `>= 24 h` trên NVMe 256 GB | `____ GB/24h` | `[ ]` |
