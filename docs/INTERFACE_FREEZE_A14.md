# INTERFACE_FREEZE_A14.md — Đóng băng giao diện phần mềm (Bám sát A.14.1–A.14.2)

Tài liệu này định nghĩa chuẩn giao diện giữa 11 node ROS 2 (L2 — C4.2), Pixhawk 6C (L1 — C4.1), bo an toàn C8 (L0 — Pico 2) và các khối phần cứng liên quan (C2, C3, C5, C6, C8). **Lưu ý:** Tài liệu này bám sát **A.14.1–A.14.2, D-17, D-22, D-24, D-27, TR-4.16 và TR-8.3** của CdCT Final 2.0 (đồng bộ với [cdct_spec_a8_a10.yaml](cdct_spec_a8_a10.yaml)), **thay thế hoàn toàn** bảng A.12 cũ (đã lỗi thời về BMS UART/CAN và cảm biến dòng ACS).

## 1. Giao diện nội bộ ROS 2 Jazzy (Gói `nenuphar_msgs` & Topic chuẩn)

Quy ước chung:
- **QoS cảm biến tốc độ cao (`SENSOR_QOS`):** `Reliability = BEST_EFFORT`, `History = KEEP_LAST`, `Depth = 1` (không chặn).
- **QoS lệnh & trạng thái (`CMD_QOS`):** `Reliability = RELIABLE`, `History = KEEP_LAST`, `Depth = 10`.

| Topic ROS 2 | Kiểu dữ liệu (Chuẩn / Custom) | Node phát -> Node nhận | Tần số | QoS |
| :--- | :--- | :--- | :---: | :--- |
| `/camera/depth/image_rect` | `sensor_msgs/Image` (400P / 800P) | `camera_driver` -> `perception_geom`, `clearance` | 15 Hz | `SENSOR_QOS` |
| `/camera/stereo/left`, `/right` | `sensor_msgs/Image` | `camera_driver` -> `vio` | 15 Hz | `SENSOR_QOS` |
| `/camera/imu` | `sensor_msgs/Imu` | `camera_driver` -> `vio` | 200 Hz | `SENSOR_QOS` |
| `/vio/odometry` | `nav_msgs/Odometry` (ENU/FLU) | `vio` -> `mavlink_bridge`, `corridor_monitor`, `mission_fsm` | 20–30 Hz | `SENSOR_QOS` |
| `/l1/ekf_odom` | `nav_msgs/Odometry` (NED từ L1) | `mavlink_bridge` -> `mission_fsm` | 10–20 Hz | `SENSOR_QOS` |
| `/perception/obstacles` | `nenuphar_msgs/ObstacleArray` | `perception_geom` -> `mission_fsm`, `corridor_monitor` | >= 10 Hz | `SENSOR_QOS` |
| `/clearance/bridge_result` | `nenuphar_msgs/BridgeClearance` | `clearance` -> `mission_fsm`, `mavlink_bridge` | Sự kiện (`S4`) | `CMD_QOS` |
| `/corridor/status` | `nenuphar_msgs/CorridorStatus` | `corridor_monitor` -> `mission_fsm` (TR-4.16: `Z1 ±0,30 m`, 5 s + 1 retry -> `S12`) | >= 10 Hz | `CMD_QOS` |
| `/corridor/outside_z2_hold` | `std_msgs/Bool` | `corridor_monitor` -> `mission_fsm` (`|y| > 1,25 m` -> `S7 Hold`) | >= 10 Hz | `CMD_QOS` |
| `/l1/battery_shunt` | `nenuphar_msgs/EnergyStatus` | `mavlink_bridge` -> `energy` (từ MAVLink `BATTERY_STATUS`) | 1–2 Hz | `CMD_QOS` |
| `/energy/status` | `nenuphar_msgs/EnergyStatus` | `energy` -> `mission_fsm`, `mavlink_bridge` (SoC, hysteresis WARN 28%/30%, BLE) | 1 Hz | `CMD_QOS` |
| `/formation/ref_state` | `nenuphar_msgs/FormationRef` | `formation` -> `mission_fsm` (`s_ref(t)` >= 2 Hz) | >= 2 Hz | `CMD_QOS` |
| `/operator/cmd` | `std_msgs/String` | `mavlink_bridge` (GCS) / Operator -> `mission_fsm` (`START/HOLD/RTB/RESET/ESTOP/SET_WAYPOINT`) | Sự kiện | `CMD_QOS` |
| `/fsm/state` | `nenuphar_msgs/MissionState` | `mission_fsm` -> `mavlink_bridge`, `blackbox`, `clearance`, `corridor_monitor` | 10 Hz + sự kiện | `CMD_QOS` |
| `/fsm/cmd_setpoint` | `nenuphar_msgs/GuidedSetpoint` | `mission_fsm` -> `mavlink_bridge` (tọa độ NED thật + `max_speed_mps`) | 5–10 Hz | `CMD_QOS` |
| `/system/node_heartbeat` | `std_msgs/String` | 10 `MonitoredNode` -> `system_monitor` | 1 Hz | `CMD_QOS` |
| `/system/health` | `nenuphar_msgs/SystemHealth` | `system_monitor` -> `mavlink_bridge`, `mission_fsm`, `blackbox` (check sysfs USB 3) | 1 Hz | `CMD_QOS` |
| `/c8/tx_signals` | `std_msgs/String` | `mavlink_bridge` -> Bo C8 (`KEEPALIVE` 1 Hz & `BRIDGE_ZONE`) | 1 Hz + sự kiện | `CMD_QOS` |
| `/mavlink/e2e_latency_ms` | `std_msgs/Float64` | `mavlink_bridge` -> `latency_probe` (đo P50/P95/P99/Max) | 15 Hz | `SENSOR_QOS` |

### Các trường tối thiểu trong Custom Messages (`nenuphar_msgs`)

- **`MissionState.msg`:** `builtin_interfaces/Time stamp`, `uint8 state` (`0..12` tương ứng chuẩn CdCT A.8 `S0..S12`), `uint8 fault_tier` (`0..3`), `bool bridge_permit`, `bool hold_all_req`.
- **`EnergyStatus.msg`:** `builtin_interfaces/Time stamp`, `float32 soc_pct`, `float32 voltage_v`, `float32 current_a`, `float32 wh_remaining`, `uint8 level` (`0=OK, 1=WARN, 2=RTB, 3=STOP`), `bool bms_ble_connected`.
- **`CorridorStatus.msg`:** `builtin_interfaces/Time stamp`, `uint8 zone` (`1=Z1 <= 0,30 m`, `2=Z2 > 0,30 m`), `float32 lateral_error_m`, `bool deviation_alert` (`true` khi ngoài Z1 quá `5 s` sau `1` lần thử lại trong lúc không ở `S11`).
- **`BridgeClearance.msg`:** `builtin_interfaces/Time stamp`, `float32 clearance_m`, `float32 consensus_spread_m` (`<= 0,05 m`), `bool pass_permitted` (`clearance_m >= raft_height + 0,10 m` và đối chiếu CSDL cầu `<= 0,05 m`).
- **`SystemHealth.msg`:** `builtin_interfaces/Time stamp`, `bool usb3_camera_ok` (đọc tốc độ `>= 5000 Mbps` từ Linux sysfs), `bool all_nodes_alive`, `uint32 failed_node_mask`, `float32 cpu_temp_c`.

## 2. Giao diện L2 (Pi 5 — C4.2) <-> L1 (Pixhawk 6C — C4.1) qua MAVLink 2

- **Kênh vật lý:** UART tốc độ **>= 921.600 baud** (nội bộ trong hộp điện IP67), bật **MAVLink 2 signing** (khóa 32 byte); liên kết tới GCS/QGroundControl đi qua VPN.
- **Luồng từ L2 -> L1:**
  - `HEARTBEAT` (`compid=191`, 1 Hz): tổng hợp từ `system_monitor`, nuôi watchdog Lua trên Pixhawk (`> 3 s -> HOLD`, nhưng không đè `MANUAL`/`STEERING`).
  - `ODOMETRY` (20–30 Hz): chuyển đổi tường minh ROS `ENU/FLU` -> MAVLink `NED/FRD` (`MAV_FRAME_LOCAL_NED`, `MAV_FRAME_BODY_FRD`, bảo toàn `w=0`, stamp gốc, hiệp phương sai hữu hạn) qua [mavlink_transforms.py](../l2_ros2_ws/src/nenuphar_nodes/nenuphar_nodes/mavlink_transforms.py).
  - `SET_POSITION_TARGET_LOCAL_NED` (5–10 Hz) & `MAV_CMD_DO_CHANGE_SPEED (178)`: Guided setpoints NED thật và tốc độ trần từ `mission_fsm` (`FS_TIMEOUT = 3.0 s`).
  - `PARAM_SET` / `COMMAND_LONG` (hướng sự kiện): đồng bộ `SCR_USER4` (`BRIDGE_PERMIT` có TTL `15 s`), `SCR_USER5` (`BRIDGE_ZONE`), và tự động gửi `MAV_CMD_SET_EKF_SOURCE_SET (42007)` (`2=VIO` ở `S5/S8`, `1=GNSS` ngoài cầu).
- **Luồng từ L1 -> L2 (vòng nhận RX `20 Hz` trên `mavlink_bridge`):**
  - `LOCAL_POSITION_NED` / `ATTITUDE` (10–20 Hz): xuất ra `/l1/ekf_odom` cho `mission_fsm`.
  - `BATTERY_STATUS` / `SYS_STATUS` (1–2 Hz): xuất ra `/l1/battery_shunt` cho `energy`.
  - `HEARTBEAT` (`compid=1`): theo dõi chế độ bay/chạy thật của L1 (`l1_mode`).
  - `COMMAND_LONG` (`MAV_CMD_USER_1..5` từ QGC): chuyển đổi thành `/operator/cmd` (`START/HOLD/RTB/RESET/ESTOP`).
  - `ESC_TELEMETRY_1_TO_4` (5–10 Hz) & `NAMED_VALUE_FLOAT`: dòng/RPM **3 ESC AM32**, `MIX_MODE` (`M1..M4`), `CURR_LIM_A`, `CURR_SCALE`, `L2_WD_TRIP`, `BRG_LOCK`.

## 3. Giao diện L1/L2 (C4) <-> Bo an toàn C8 (Pico 2) & LoRa (D-27, A.10, TR-8.3)

### Kết nối nội bộ với Bo C8 (C4 <-> C8)

> **Quy tắc an toàn cứng (CdCT A.10):** **C4 tuyệt đối không có quyền đóng contactor động lực.** Để rời `SAFE_OFF -> ARMED`, C8 bắt buộc phải nhận đủ: **sườn lên nút Start vật lý tại bè** + E-stop đã nhả + cảm biến hợp lệ + keepalive C4 `1 Hz` + gói LoRa `RUN` còn hạn (`<= 30 s`).

| Tín hiệu / Gói tin | Chiều | Chu kỳ / Thời hạn | Ý nghĩa |
| :--- | :--- | :---: | :--- |
| Keepalive C4 -> C8 | L2/L1 -> C8 | 1 Hz (mất 3 nhịp = 3 s) | Mất 3 nhịp liên tiếp -> C8 chuyển `HOLD_REQ` |
| Cờ `BRIDGE_ZONE` | L2/L1 -> C8 | Làm mới `<= 60 s` | Chỉ ức chế tự cắt do nghiêng/tải TR-8.3; **không bao giờ chặn** E-stop, LoRa STOP, 4G STOP hay WD STOP |
| Tín hiệu `4G STOP` & `WD STOP` | C4.2 / Mạch WD -> C8 | Tức thời | Đưa C8 về `STOPPED` chốt cứng (bất chấp `BRIDGE_ZONE`) |
| Trạng thái C8 (`SAFE_OFF` / `ARMED` / `STOPPED` / `HOLD_REQ`) | C8 -> L1/L2 | `>= 2 Hz` + sự kiện | Đồng bộ lên `mission_fsm` và `blackbox` |
| Ngắt cứng động lực | C8 -> Rơ-le nguồn ESC | Tức thời | Chốt cứng khi `STOPPED`; chỉ thoát về `SAFE_OFF` bằng **sườn lên nút Reset tay tại bè** (EN ISO 13850) |

### Khung gói tin LoRa 12 byte (Ebyte E22-900M30S SX1262 — D-27, đã triển khai trong `lora_protocol.cpp`)

| Vị trí byte | Trường | Độ dài | Mô tả |
| :---: | :--- | :---: | :--- |
| `[0]` | `(sender_id << 4) \| CMD` | 1 byte | Nibble cao: `sender_id` (`0..15`, tách biệt GCS / remote cầm tay); Nibble thấp: `CMD` (`0x1=RUN`, `0x2=STOP`, `0x3=HOLD`) |
| `[1..2]` | `GROUP_ID` | 2 byte | Big-endian; khớp `local_group_id` hoặc mã broadcast `0xFFFF` |
| `[3..6]` | `COUNTER` | 4 byte | Big-endian 32-bit, tăng nghiêm ngặt theo từng `sender_id` (lưu bền qua reboot bằng `CounterStorage`, nhảy `+1000` khi reboot máy phát) |
| `[7..11]` | `HMAC_TRUNC` | 5 byte | 5 byte (40-bit) đầu của `HMAC-SHA256(key, pkt[0..6])` so sánh constant-time; khóa `> 64 B` băm SHA-256 theo RFC 2104 |
| **Tổng** | **12 byte cố định** | **12 byte** | Tần số V1 VN: `920–923 MHz` (QCVN 122:2020/BTTTT); V2 Pháp: `869,4–869,65 MHz` (`<= 10%` duty cycle) |

## 4. Giao diện nguồn & động lực (C5, C6 -> C4 theo D-22, D-24)

- **Đo dòng/áp chính (C6 -> L1/L2):** Cảm biến **shunt** nối về Pixhawk 6C (`BATT_MONITOR=4`) làm nguồn chính tính năng lượng (A.9), bảo vệ `BATT_FS_LOW/CRT` và giới hạn dòng Lua (`70 A / 40 A`).
- **Đối chiếu BMS (C6 -> L2):** Đọc qua **Bluetooth LE** trên Pi 5 (nút `energy`). Quy tắc lỗi (TBD-23): mất BLE chỉ phát cảnh báo và ghi log, tuyệt đối không dừng bè.
- **Điều khiển 3 ESC (L1 -> C5):** Bidirectional DShot + UART telemetry cho 3× Holybro Tekko32 F4 45A (AM32), giới hạn `30 A/ESC`, hỗ trợ phát hiện kẹt & gỡ rong `<= 3` xung đảo chiều (TR-5.6).

## 5. Checklist xác nhận đóng băng với các nhóm (Tuần 1)

- [ ] **Nhóm C2/C3 (Thân bè & Cảm biến):** Xác nhận file [v1_70x50.yaml](../config/raft_profiles/v1_70x50.yaml) (gồm `total_height_above_water_m`, vị trí 2 ăng-ten GNSS, độ cao camera `20 cm`, khoảng cách thruster) và thông báo ngay nếu cổng G-ST đổi sang thân `98 × 68 × 25 cm`.
- [ ] **Nhóm C5 (Động lực):** Xác nhận mapping kênh servo/DShot cho 3 ESC (Trái=94, Phải=95, Dự phòng đuôi=96) và chiều quay thuận/nghịch trên AM32.
- [ ] **Nhóm C6 (Nguồn):** Xác nhận hệ số hiệu chuẩn shunt và cấu trúc dịch vụ GATT Bluetooth LE của BMS.
- [ ] **Nhóm C8 (Bo an toàn):** Xác nhận sơ đồ chân RP2350 ([pico_hal.hpp](../l0_c8_pico2/firmware/pico_hal.hpp)), bố cục byte `[0..11]` gói LoRa 12 byte (gồm `sender_id`), khóa HMAC dùng chung và phân vùng Flash lưu counter.
