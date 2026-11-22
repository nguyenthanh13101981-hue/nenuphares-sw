# SPEC_PHU_LUC_A.md — Đặc tả kỹ thuật phần mềm nhúng & điều khiển (Phụ lục A — CdCT Final 2.0)

Tài liệu tóm lược toàn bộ yêu cầu kỹ thuật của **Phụ lục A** (`CdCT Final 2.0`, ngày 20/11/2026) đã lọc sạch các lỗi sót bản cũ để làm chuẩn triển khai phần mềm cho tầng L0, L1 và L2.

## 1. Phân tầng kiến trúc & lớp an toàn (Mục A.1)

| Tầng | Phần cứng | Nền tảng phần mềm | Vai trò chính |
| :--- | :--- | :--- | :--- |
| **Lớp 1 (L0)** | Bo an toàn C8 (Raspberry Pi Pico 2) | Bare-metal C/C++ + watchdog phần cứng ngoài (100 ms) | Cắt nguồn động lực độc lập (E-stop, LoRa, nghiêng, tải >= 10 kg), yêu cầu `HOLD_REQ` |
| **Lớp 2 (L1)** | Pixhawk 6C | ArduPilot Rover (`FRAME_CLASS=2`, 50 Hz) + Lua scripts | Điều khiển vòng trong, EKF, trộn lực đẩy M1–M4, giới hạn dòng 70/40 A, watchdog Pi 5, khóa phép qua cầu |
| **Lớp 3 (L2)** | Raspberry Pi 5 (8 GB, NVMe 256 GB) | Ubuntu 24.04 + ROS 2 Jazzy (11 node) | Nhận thức OAK-D Pro W, VIO, máy trạng thái `S0–S12`, giám sát hành lang, năng lượng, hộp đen |
| **Cơ cấu chấp hành** | 3× Holybro Tekko32 F4 45A ESC | Firmware AM32 | Bidirectional DShot + UART telemetry, giới hạn 30 A/ESC, PWM 24 kHz, về 0 khi mất tín hiệu |

## 2. Kiến trúc 11 node ROS 2 Jazzy trên Raspberry Pi 5 (Mục A.4)

| STT | Tên node | Tần số / Chế độ | Nhiệm vụ cốt lõi & ràng buộc |
| :---: | :--- | :--- | :--- |
| 1 | `camera_driver` | Depth >= 15 Hz, IMU 200 Hz | Điều khiển OAK-D Pro W qua `depthai-ros`, hàng đợi depth=1 không chặn, ưu tiên `SCHED_FIFO` |
| 2 | `vio` | >= 15 Hz (xuất ODOMETRY 20–30 Hz) | Ước lượng chuyển động thị giác-quán tính (`Basalt` hoặc `VINS-Fusion` bản ROS 2), `SCHED_FIFO` |
| 3 | `perception_geom` | >= 10 Hz (<= 50 ms/khung) | Chạy khối hình học B3–B6B (voxel grid, ESDF, phát hiện vật cản), `SCHED_FIFO` |
| 4 | `clearance` | Hướng sự kiện tại `S4` | Đo tĩnh không vòm cầu (D-11), đồng thuận +/- 5 cm bằng Config B (800P) ở tốc độ <= 2 km/h |
| 5 | `mission_fsm` | 10 Hz + hướng sự kiện vật cản | Máy trạng thái `S0–S12`, xuất Guided setpoints 5–10 Hz (`GUID_TIMEOUT >= 3 s`) |
| 6 | `corridor_monitor` | >= 10 Hz | Giám sát hành lang Z1/Z2 (D-16), phát cảnh báo khi sai lệch > 15 cm |
| 7 | `formation` | >= 2 Hz | Đồng bộ đội hình lai `s_ref(t)` (D-17, V2) + dự phòng đồng hồ GNSS <= 10 ms; V1 để khung tối thiểu |
| 8 | `energy` | 1 Hz | Tính SoC, phát cảnh báo `WARN` / `RTB` theo mục A.9; xử lý mất Bluetooth BMS (chỉ cảnh báo) |
| 9 | `mavlink_bridge` | Liên tục (`SCHED_FIFO`) | Cầu nối MAVLink 2 có ký (UART >= 921.600 baud tới Pixhawk + VPN tới GCS) |
| 10 | `blackbox` | 1 Hz (thường) / >= 10 Hz (sự kiện, cầu) | Ghi `rosbag2` mức ưu tiên CPU thấp, dung lượng lưu tối thiểu >= 24 giờ trên NVMe |
| 11 | `system_monitor` | 1 Hz | Gom heartbeat của các node thành heartbeat C4.2 -> C4.1; từ chối `S0` nếu camera không chạy USB 3 |

## 3. Nhận thức thị giác & thuật toán VIO (Mục A.6)

- **Camera OAK-D Pro W (RVC2):** Cố định cao 20 cm trên boong, trục quang ngang (`alpha_tilt = 0°`), cảm biến màu OV9782 (1 MP global shutter). Chạy xen kẽ khung hình chiếu điểm (dot projector) và đèn IR ở 30 fps -> thu được **15 Hz ảnh độ sâu** + **15 Hz ảnh VIO** + **200 Hz IMU** (TBD-21).
- **Các khối thuật toán B1–B6:**
  - **B1:** VIO factor graph (`Basalt` hoặc `VINS-Fusion` ROS 2 — chốt 1 gói ở L1 sau khi test biên dịch).
  - **B2:** Point+Line LSD + bù quang học Affine (cân nhắc bật/tắt tùy tải CPU Pi 5 ở TBD-24).
  - **B3:** 3D Voxel Grid (độ phân giải 5 cm vùng gần / 10 cm vùng trung) + ESDF.
  - **B4:** Khớp hình học vòm cầu 3D Arch-Cylinder (RANSAC + GICP).
  - **B5:** Ràng buộc mặt nước 3-DOF + mô hình động lực học chân vịt (`v_bè = v_thruster + v_drift`), đọc telemetry từ **3 ESC**.
  - **B6:** V1 dùng **B6B** thuần hình học (Epipolar + IMU RANSAC); V2 bổ sung **B6A** (YOLOv8n-seg `.blob` trên Myriad X).

## 4. Điều khiển tầng L1 — ArduPilot Rover & Lua Scripts (Mục A.7)

- **Cấu hình lõi:** `FRAME_CLASS=2` (Boat), `SCHED_LOOP_RATE=50 Hz`, hướng mũi từ GNSS 2 ăng-ten (không dùng compass), `GUID_TIMEOUT >= 3 s`, chuyển nguồn EKF sang VIO `ODOMETRY` khi mất GNSS dưới cầu.
- **Các kịch bản Lua bắt buộc:**
  1. **Trộn lực đẩy 4 chế độ M1–M4:** Điều phối 2 thruster chính (trái/phải) + 1 thruster dự phòng đuôi khi xảy ra hỏng 1 động cơ.
  2. **Giới hạn dòng tổng:** Cắt/nén công suất ở ngưỡng **70 A** (chế độ bình thường) và **40 A** (chế độ dự phòng); mỗi ESC tự giới hạn 30 A.
  3. **Watchdog heartbeat Pi 5 (L2 -> L1):** Tự chuyển giữ vị trí an toàn khi mất nhịp tim từ `system_monitor` / `mavlink_bridge`.
  4. **Khóa phép qua cầu & nới lỏng GCS timeout:** Chỉ cho phép đi vào gầm cầu khi có cờ cho phép từ `clearance` (`S4`) và tạm nới lỏng thời gian chờ mất tín hiệu GCS trong vùng cầu.
- **Bảo vệ pin tầng L1:** Ngưỡng phần mềm `BATT_CRT` ở **11,5–12,0 V** (hoặc SoC <= 10%), cao hơn ngưỡng cắt bảo vệ phần cứng của BMS (**10,0 V**).

## 5. Máy trạng thái nhiệm vụ S0–S12 & phân tầng lỗi (Mục A.8)

- **Tập trạng thái:** `S0` đến `S12` (khởi tạo/kiểm tra trước chạy -> tuần tra/di chuyển -> tiếp cận/đo tĩnh không cầu `S4` -> qua cầu -> lách vật cản -> quay về bến `S10` -> giữ vị trí / ĐDKC `S12` -> dừng an toàn `S8`/`S9`).
- **Thứ tự ưu tiên cứng:** `S9 > S8 > S12 > S7 > S11 > S10 > S1`.
- **Phân tầng xử lý lỗi:**
  - **Tầng 1 (Lỗi nhẹ):** Tự phục hồi cục bộ (ví dụ mất Bluetooth BMS chỉ ghi cảnh báo).
  - **Tầng 2 (Lỗi trung bình):** Chuyển về điểm dừng khẩn cấp hành lang (**ĐDKC — `S12`**).
  - **Tầng 3 (Lỗi nghiêm trọng):** Lập tức giữ vị trí / dừng an toàn (`S8`/`S9`) và phát lệnh **HOLD-ALL <= 2 s** cho đội hình (V2).

## 6. Quản lý năng lượng (Mục A.9 — chạy 1 Hz trên `energy`)

| Tham số / Ngưỡng | Giá trị chuẩn (Final 2.0) | Hành động |
| :--- | :--- | :--- |
| Dung lượng tổng (`E_tổng`) | 1.280 Wh (2 × 640 Wh LFP) | Cơ sở tính toán SoC từ shunt chính |
| Dự trữ dừng (`E_dừng`) | 128 Wh (10%) | Ngưỡng cứng không xâm phạm |
| Dự phòng an toàn (`E_dp`) | 64 Wh (5%) | Biên an toàn bù dòng/gió |
| Ngưỡng cảnh báo (`WARN`) | ~28% (227,3 Wh ở P95 = 122 W, cự ly 2 km) | Gửi cảnh báo lên GCS/HMI |
| Ngưỡng quay về (`RTB — S10`) | ~21% (145,7 Wh) | `mission_fsm` kích hoạt `S10` quay về bến |
| Ngưỡng dừng (`STOP — BATT_CRT`) | <= 10% (hoặc 11,5–12,0 V trên L1) | Tầng L1 cưỡng bức dừng an toàn |

## 7. Bo an toàn C8 — Raspberry Pi Pico 2 (Mục A.10)

- **Chu kỳ & phần cứng:** Vòng lặp chính >= 100 Hz, watchdog phần cứng ngoài 100 ms, đọc MPU-6050 >= 50 Hz, đọc 4 load cell 50 kg (HX711, ngưỡng quá tải **10 kg**), thu LoRa SX1262 qua SPI.
- **4 trạng thái cốt lõi:**
  - `SAFE_OFF`: Khởi động an toàn, ngắt rơ-le động lực.
  - `ARMED`: Cho phép cấp nguồn động lực khi đủ điều kiện an toàn và có keepalive hợp lệ.
  - `STOPPED`: **Chốt cứng (latched)** khi nhấn nút E-stop, nhận gói LoRa `STOP`, hoặc vượt ngưỡng nghiêng/tải ngoài vùng cầu; chỉ mở khóa lại bằng thao tác thủ công tại bè (EN ISO 13850).
  - `HOLD_REQ`: Kích hoạt khi mất **3 nhịp keepalive liên tiếp (= 3 s)** hoặc nhận gói LoRa `HOLD`; gửi yêu cầu giữ vị trí sang L1/L2 mà không cắt cứng nguồn.
- **Cờ `BRIDGE_ZONE`:** Làm mới chu kỳ <= 60 s; khi đang hoạt động, chỉ tạm ức chế cắt tự động do nghiêng/tải (nút E-stop tay và gói LoRa `STOP` luôn có hiệu lực tuyệt đối).

## 8. Ngân sách độ trễ phát hiện vật cản TR-4.1 <= 200 ms (Mục A.11)

| Khâu trong chuỗi xử lý | Config A (400P — Tránh vật cản) | Config B (800P — Chỉ đo cầu `S4`) |
| :--- | :---: | :---: |
| Chờ khung hình (15 Hz) | 0–67 ms | 0–67 ms |
| Nửa thời gian phơi sáng | 1–5 ms | 1–5 ms |
| Xử lý trên OAK-D Pro W (đọc/hiệu chỉnh/stereo/median) | 21 ms | Lớn hơn Config A |
| Xử lý nhận thức trên Pi 5 (`perception_geom`) | <= 50 ms | <= 80 ms |
| Quyết định `mission_fsm` + truyền `mavlink_bridge` | <= 15 ms | <= 15 ms |
| Autopilot L1 (50 Hz) + DShot ESC | <= 20 ms + < 1 ms | <= 20 ms + < 1 ms |
| **Tổng trường hợp xấu nhất** | **~180 ms (ĐẠT <= 200 ms)** | **~255 ms (KHÔNG ĐẠT tránh vật cản)** |

## 9. Danh mục điểm mở (TBD) ảnh hưởng trực tiếp tới phần mềm

| Mã TBD | Nội dung cần chốt | Mốc chốt & Cách chuẩn bị trên phần mềm |
| :--- | :--- | :--- |
| **TBD-06** | Thân bè 70×50×25 cm đang GM âm (RI-1.5), có thể đổi sang 98×68×25 cm | Tách toàn bộ kích thước & vị trí cảm biến/thruster vào `config/raft_profiles/*.yaml` |
| **TBD-20** | Bộ tham số điều khiển (chuyển nguồn EKF, ngưỡng hành lang, timeout S3/S6, giới hạn dòng Lua, bù áp) | Theo dõi trong `docs/tbd20_tuning_tracker.csv` qua các cấp SITL -> L0 -> L1 |
| **TBD-21** | Xen kẽ dot projector / IR LED giảm độ sâu còn 15 Hz | Thiết kế toàn bộ pipeline nhận thức theo nhịp 15 Hz thực tế |
| **TBD-23** | BMS chỉ xuất dữ liệu qua Bluetooth LE | Test tiêm lỗi mất kết nối BLE trên nút `energy` (ghi cảnh báo, tiếp tục tính bằng shunt) |
| **TBD-24** | Tải CPU & giảm xung nhiệt Pi 5 ở 80 °C (môi trường 55 °C) | Đo tải CPU/nhiệt độ ở bước HIL; chuẩn bị sẵn cờ tắt khối B2 nếu vượt ngưỡng |
| **TBD-30** | Độ trễ thực tế chuỗi camera OAK-D Pro W | Viết công cụ gắn timestamp đo P99 toàn chuỗi ở Tuần 4 trước khi hàng về |
