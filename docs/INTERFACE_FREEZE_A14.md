# INTERFACE_FREEZE_A14.md — Đóng băng giao diện phần mềm (Bám sát A.14.1–A.14.2)

Tài liệu này định nghĩa chuẩn giao diện giữa 11 node ROS 2 (L2), Pixhawk 6C (L1), bo an toàn C8 (L0) và các khối phần cứng liên quan (C2, C3, C5, C6, C8). **Lưu ý:** Tài liệu này bám sát **A.14.1–A.14.2, D-17, D-22, D-24 và D-27** của CdCT Final 2.0, **thay thế hoàn toàn** bảng A.12 cũ (đã lỗi thời về BMS UART/CAN và cảm biến dòng ACS).

## 1. Giao diện nội bộ ROS 2 Jazzy (Gói `nenuphar_msgs` & Topic chuẩn)

Quy ước chung:
- **QoS cảm biến tốc độ cao (`SENSOR_QOS`):** `Reliability = BEST_EFFORT`, `History = KEEP_LAST`, `Depth = 1` (không chặn).
- **QoS lệnh & trạng thái (`CMD_QOS`):** `Reliability = RELIABLE`, `History = KEEP_LAST`, `Depth = 10`.

| Topic ROS 2 | Kiểu dữ liệu (Chuẩn / Custom) | Node phát -> Node nhận | Tần số | QoS |
| :--- | :--- | :--- | :---: | :--- |
| `/camera/depth/image_rect` | `sensor_msgs/Image` (400P / 800P) | `camera_driver` -> `perception_geom`, `clearance` | 15 Hz | `SENSOR_QOS` |
| `/camera/stereo/left`, `/right` | `sensor_msgs/Image` | `camera_driver` -> `vio` | 15 Hz | `SENSOR_QOS` |
| `/camera/imu` | `sensor_msgs/Imu` | `camera_driver` -> `vio` | 200 Hz | `SENSOR_QOS` |
| `/vio/odometry` | `nav_msgs/Odometry` | `vio` -> `mavlink_bridge`, `corridor_monitor`, `mission_fsm` | 20–30 Hz | `SENSOR_QOS` |
| `/perception/obstacles` | `nenuphar_msgs/ObstacleArray` | `perception_geom` -> `mission_fsm`, `corridor_monitor` | >= 10 Hz | `SENSOR_QOS` |
| `/clearance/bridge_result` | `nenuphar_msgs/BridgeClearance` | `clearance` -> `mission_fsm`, `mavlink_bridge` | Sự kiện (`S4`) | `CMD_QOS` |
| `/corridor/status` | `nenuphar_msgs/CorridorStatus` | `corridor_monitor` -> `mission_fsm` (cảnh báo lệch > 15 cm) | >= 10 Hz | `CMD_QOS` |
| `/energy/status` | `nenuphar_msgs/EnergyStatus` | `energy` -> `mission_fsm`, `mavlink_bridge` (SoC, WARN/RTB, BLE) | 1 Hz | `CMD_QOS` |
| `/formation/ref_state` | `nenuphar_msgs/FormationRef` | `formation` -> `mission_fsm` (`s_ref(t)` >= 2 Hz) | >= 2 Hz | `CMD_QOS` |
| `/fsm/state` | `nenuphar_msgs/MissionState` | `mission_fsm` -> `mavlink_bridge`, `blackbox`, `clearance` | 10 Hz + sự kiện | `CMD_QOS` |
| `/fsm/cmd_setpoint` | `nenuphar_msgs/GuidedSetpoint` | `mission_fsm` -> `mavlink_bridge` | 5–10 Hz | `CMD_QOS` |
| `/system/health` | `nenuphar_msgs/SystemHealth` | `system_monitor` -> `mavlink_bridge`, `mission_fsm` (check USB 3) | 1 Hz | `CMD_QOS` |

### Các trường tối thiểu trong Custom Messages (`nenuphar_msgs`)

- **`MissionState.msg`:** `builtin_interfaces/Time stamp`, `uint8 state` (`0..12` tương ứng `S0..S12`), `uint8 fault_tier` (`0..3`), `bool bridge_permit`, `bool hold_all_req`.
- **`EnergyStatus.msg`:** `builtin_interfaces/Time stamp`, `float32 soc_pct`, `float32 voltage_v`, `float32 current_a`, `float32 wh_remaining`, `uint8 level` (`0=OK, 1=WARN, 2=RTB, 3=STOP`), `bool bms_ble_connected`.
- **`CorridorStatus.msg`:** `builtin_interfaces/Time stamp`, `uint8 zone` (`1=Z1, 2=Z2`), `float32 lateral_error_m`, `bool deviation_alert` (`true` khi > 0,15 m).
- **`BridgeClearance.msg`:** `builtin_interfaces/Time stamp`, `float32 clearance_m`, `float32 consensus_spread_m` (yêu cầu <= 0,05 m), `bool pass_permitted`.
- **`SystemHealth.msg`:** `builtin_interfaces/Time stamp`, `bool usb3_camera_ok`, `bool all_nodes_alive`, `uint32 failed_node_mask`, `float32 cpu_temp_c`.

## 2. Giao diện L2 (Pi 5) <-> L1 (Pixhawk 6C) qua MAVLink 2

- **Kênh vật lý:** UART tốc độ **>= 921.600 baud** (nội bộ trong hộp điện IP67), bật **MAVLink 2 signing**; liên kết tới GCS/QGroundControl đi qua VPN.
- **Luồng từ L2 -> L1:**
  - `HEARTBEAT` (1 Hz): tổng hợp từ `system_monitor`, nuôi watchdog Lua trên Pixhawk.
  - `ODOMETRY` (20–30 Hz): từ nút `vio` để EKF chuyển nguồn khi mất GNSS dưới cầu.
  - `SET_POSITION_TARGET_LOCAL_NED` (5–10 Hz): Guided setpoints từ `mission_fsm` (`GUID_TIMEOUT >= 3 s`).
  - `NAMED_VALUE_INT` / `COMMAND_LONG` (hướng sự kiện): gửi cờ khóa phép qua cầu (`BRIDGE_PERMIT`), cờ nới lỏng GCS timeout (`BRIDGE_ZONE`), và lệnh chuyển nguồn EKF (`MAV_CMD_SET_EKF_SOURCE_SET`).
- **Luồng từ L1 -> L2:**
  - `GLOBAL_POSITION_INT` / `LOCAL_POSITION_NED` / `ATTITUDE` (10–20 Hz): trạng thái định vị EKF và hướng GNSS 2 ăng-ten.
  - `SYS_STATUS` / `BATTERY_STATUS` (1–2 Hz): điện áp và dòng tổng đo từ shunt chính.
  - `ESC_TELEMETRY_1_TO_4` (5–10 Hz): dòng, điện áp, RPM và nhiệt độ của **3 ESC AM32** (chính trái, chính phải, dự phòng đuôi).
  - `STATUSTEXT` / `NAMED_VALUE_INT` (sự kiện): chế độ trộn lực đẩy hiện tại (`M1..M4`) và cờ cắt dòng Lua (70 A / 40 A).

## 3. Giao diện L1/L2 <-> Bo an toàn C8 (Pico 2) & LoRa (D-27, A.10)

### Kết nối nội bộ với Bo C8 (C4 <-> C8)

| Tín hiệu / Gói tin | Chiều | Chu kỳ / Thời hạn | Ý nghĩa |
| :--- | :--- | :---: | :--- |
| Keepalive C4 -> C8 | L1/L2 -> C8 | 1 Hz (mất 3 nhịp = 3 s) | Mất 3 nhịp liên tiếp -> C8 chuyển `HOLD_REQ` |
| Cờ `BRIDGE_ZONE` | L2/L1 -> C8 | Làm mới <= 60 s | Tạm ức chế cắt tự động do nghiêng/tải khi ở dưới cầu |
| Trạng thái C8 (`SAFE_OFF` / `ARMED` / `STOPPED` / `HOLD_REQ`) | C8 -> L1/L2 | >= 2 Hz + sự kiện | Đồng bộ lên `mission_fsm` và `blackbox` |
| Ngắt cứng động lực | C8 -> Rơ-le nguồn ESC | Tức thời | Chốt cứng khi `STOPPED` (chỉ reset tay tại bè) |

### Khung gói tin LoRa 12 byte (Ebyte E22-900M30S SX1262 — D-27)

| Trường | Độ dài | Mô tả |
| :--- | :---: | :--- |
| Mã lệnh (`CMD`) | 1 byte | `RUN`, `STOP` (chốt cứng `STOPPED`), hoặc `HOLD` (kích hoạt `HOLD_REQ` / dự phòng HOLD-ALL) |
| Mã nhóm / Bè (`GROUP_ID`) | 1–2 byte | Định danh nhóm bè hoặc broadcast (chốt kích thước chính xác cùng C8) |
| Bộ đếm chống phát lại (`COUNTER`) | 4 byte | Số thứ tự tăng đơn điệu (anti-replay) |
| Chữ ký rút gọn (`HMAC_TRUNC`) | 4–5 byte | Truncated HMAC xác thực nguồn phát hợp lệ |
| **Tổng kích thước gói** | **12 byte** | Tần số V1 VN: 920–923 MHz; V2 Pháp: 869,4–869,65 MHz (<= 10% duty cycle) |

## 4. Giao diện nguồn & động lực (C5, C6 -> C4 theo D-22, D-24)

- **Đo dòng/áp chính (C6 -> L1/L2):** Cảm biến **shunt** nối về Pixhawk 6C làm nguồn chính tính năng lượng (A.9) và giới hạn dòng Lua (70 A / 40 A).
- **Đối chiếu BMS (C6 -> L2):** Đọc qua **Bluetooth LE** trên Pi 5 (nút `energy`). Quy tắc lỗi (TBD-23): mất BLE chỉ phát cảnh báo và ghi log, tuyệt đối không dừng bè.
- **Điều khiển 3 ESC (L1 -> C5):** Bidirectional DShot + UART telemetry cho 3× Holybro Tekko32 F4 45A (AM32), giới hạn 30 A/ESC.

## 5. Checklist xác nhận đóng băng với các nhóm (Tuần 1)

- [ ] **Nhóm C2/C3 (Thân bè & Cảm biến):** Xác nhận file `config/raft_profiles/v1_70x50.yaml` (vị trí 2 ăng-ten GNSS, độ cao camera 20 cm, khoảng cách thruster) và thông báo ngay nếu cổng G-ST đổi sang thân `98 × 68 × 25 cm`.
- [ ] **Nhóm C5 (Động lực):** Xác nhận mapping kênh servo/DShot cho 3 ESC (Trái, Phải, Dự phòng đuôi) và chiều quay thuận/nghịch trên AM32.
- [ ] **Nhóm C6 (Nguồn):** Xác nhận hệ số hiệu chuẩn shunt và cấu trúc dịch vụ GATT Bluetooth LE của BMS.
- [ ] **Nhóm C8 (Bo an toàn):** Xác nhận giao thức điện/UART/GPIO cho tín hiệu `keepalive`, `BRIDGE_ZONE`, `HOLD_REQ` và sơ đồ từng byte (0..11) của gói LoRa 12 byte + khóa HMAC dùng chung.
