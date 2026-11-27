# Dàn ý Trình chiếu Khối C4 — Điều khiển & Máy tính Nhúng (`nenuphares-sw`)

Tài liệu tóm tắt theo dạng **8 slide sẵn sàng trình chiếu** (thời lượng 10–15 phút) cho buổi báo cáo tiến độ **Khối C4** (gồm **C4.1 Pixhawk 6C / Tầng L1** và **C4.2 Raspberry Pi 5 / Tầng L2**) của dự án **NÉNUPHARES BONNEVAL V1 — CdCT Final 2.0**.

> **Tài liệu kỹ thuật chi tiết đi kèm:** [SYSTEM_ARCHITECTURE_AND_LOGIC.md](SYSTEM_ARCHITECTURE_AND_LOGIC.md) · [INTERFACE_FREEZE_A14.md](INTERFACE_FREEZE_A14.md) · [tr_fr_traceability_matrix.md](tr_fr_traceability_matrix.md) · [hil_poweron_checklist.md](hil_poweron_checklist.md)

---

## Slide 1 — Vai trò Khối C4 trong Kiến trúc Bảo vệ 3 tầng

```mermaid
flowchart LR
    C23["C2/C3 Cảm biến\nOAK-D Pro W + GNSS 2 ăng-ten"] --> C42
    C6["C6 Nguồn\nShunt + BLE BMS"] --> C41
    C6 -->|"Bluetooth LE"| C42
    GCS["QGroundControl\n(qua VPN)"] <--> C42

    subgraph C4["KHỐI C4 — ĐIỀU KHIỂN TRUNG TÂM"]
        C42["C4.2 — Raspberry Pi 5 (Tầng L2)\nROS 2 Jazzy (11 node)\nNhận thức · VIO · FSM S0-S12 · Blackbox"]
        C41["C4.1 — Pixhawk 6C (Tầng L1)\nArduPilot Rover 4.7.3 + Lua (50 Hz)\nEKF3 · Trộn M1-M4 · Watchdog 3s"]
        C42 <-->|"MAVLink 2 Signed\nUART >= 921600 baud"| C41
    end

    C42 -->|"Keepalive 1 Hz\nBRIDGE_ZONE <= 60s"| C8["C8 — Bo an toàn (Tầng L0)\nPico 2 Bare-metal >= 100 Hz\nEN ISO 13850 · LoRa 12B HMAC"]
    C8 -->|"HOLD_REQ"| C41
    C8 -->|"Cắt cứng Rơ-le\n(C4 KHÔNG được đóng)"| C5["C5 — 3x ESC AM32\nTrái · Phải · Dự phòng đuôi"]
    C41 -->|"Bidirectional DShot\n+ UART Telemetry"| C5
```

- **Triết lý Bảo vệ theo chiều sâu (Defense in Depth):**
  - **C4.2 (L2)** ra quyết định nhiệm vụ & nhận thức; nếu C4.2 treo, **C4.1 (L1)** tự ép `HOLD` sau `3 s`.
  - **C8 (L0)** độc lập hoàn toàn ở mức phần cứng bare-metal; **C4 tuyệt đối không có quyền đóng contactor động lực**.

> *Lời dẫn gợi ý (~1,5 phút):* "Khối C4 của nhóm gồm hai bộ xử lý tách bạch trách nhiệm: Raspberry Pi 5 chạy ROS 2 Jazzy cho nhận thức và máy trạng thái nhiệm vụ, và Pixhawk 6C chạy ArduPilot + Lua ở 50 Hz cho điều khiển vòng trong và failsafe. Tầng dưới cùng là bo C8 độc lập hoàn toàn — ngay cả khi cả hai máy tính của C4 gặp sự cố, C8 vẫn cắt cứng động lực theo chuẩn EN ISO 13850."

---

## Slide 2 — Giao diện Vật lý & Truyền thông của Khối C4

| Kết nối | Bus / Giao thức | Tốc độ / Tần số | Chiều dữ liệu | Cơ chế an toàn / Bảo mật |
| :--- | :--- | :---: | :---: | :--- |
| **OAK-D Pro W → C4.2** | USB 3 SuperSpeed | `>= 5000 Mbps` (15 Hz depth/VIO, 200 Hz IMU) | C3 → C4.2 | Kiểm tra tốc độ thực qua Linux sysfs; rớt USB 2 khóa cứng tại `S0` |
| **C4.2 ↔ C4.1** | UART nội bộ IP67 (MAVLink 2) | `>= 921.600 baud` | Hai chiều | Bật **MAVLink 2 signing** (khóa 32 byte); chuyển chuẩn `ENU/FLU → NED/FRD` |
| **C4.1 → 3× ESC (C5)** | Bidirectional DShot + UART | `50 Hz` lệnh / `5–10 Hz` telemetry | Hai chiều | `30 A/ESC`, PWM `24 kHz`, tự về `0` khi mất tín hiệu |
| **Nguồn C6 → C4.1 / C4.2** | Shunt Analog + Bluetooth LE | `1–2 Hz` shunt / `1 Hz` BLE | C6 → C4 | Shunt là nguồn đo chính; mất BLE chỉ cảnh báo (TBD-23) |
| **C4 ↔ Bo an toàn C8** | GPIO / UART nội bộ | Keepalive `1 Hz`, `BRIDGE_ZONE <= 60 s` | Hai chiều | Mất keepalive `3 s` → `HOLD_REQ`; C4 không có lệnh đóng contactor |
| **C4.2 ↔ QGroundControl** | MAVLink 2 qua VPN | Liên tục | Hai chiều | Lệnh vận hành `START/HOLD/RTB/RESET/ESTOP` → `/operator/cmd` |

> *Lời dẫn gợi ý (~1 phút):* "Toàn bộ giao diện điện và truyền thông của C4 đã được đóng băng tại [INTERFACE_FREEZE_A14.md](INTERFACE_FREEZE_A14.md) để phối hợp song song với các nhóm C2/C3, C5, C6 và C8."

---

## Slide 3 — 7 Nguyên tắc Thiết kế cứng (CdCT Final 2.0)

1. **Không LiDAR, không nghiêng camera:** chỉ dùng stereo OAK-D Pro W lắp ngang (`alpha_tilt = 0°`), cao `20 cm` trên boong.
2. **Không dùng la bàn từ:** định hướng bằng **GNSS 2 ăng-ten** (`COMPASS_USE*=0`, `EK3_SRC1_YAW=2`) để tránh nhiễu từ trường từ dòng `70 A` và kết cấu cầu.
3. **Chốt 1 gói VIO duy nhất (`VINS-Fusion-ROS2`):** đã kiểm chứng biên dịch trên Ubuntu 24.04 + Jazzy ([VIO_FEASIBILITY_B.md](VIO_FEASIBILITY_B.md)); loại bỏ Basalt và nhánh TBD-28.
4. **Shunt chính + đối chiếu BLE BMS + 3 ESC:** loại bỏ BMS UART/CAN và cảm biến ACS cũ; mất BLE tuyệt đối không dừng bè.
5. **Nguồn chân lý máy đọc ([cdct_spec_a8_a10.yaml](cdct_spec_a8_a10.yaml)):** toàn bộ 13 trạng thái `S0–S12`, 52 cạnh chuyển hợp lệ và ngưỡng an toàn nạp từ một tệp YAML duy nhất.
6. **Không hardcode kích thước thân bè (TBD-06):** nạp động `v1_70x50.yaml` hoặc `v1_98x68.yaml` (gồm cả `total_height_above_water_m` cho phép đo cầu `S4`).
7. **Bảo mật & Chống tái khởi động bất ngờ:** ký MAVLink 2 (32B), LoRa 12B HMAC-SHA256 kèm `sender_id` và lưu counter qua reboot.

> *Lời dẫn gợi ý (~1,5 phút):* "Bảy luật cứng này khóa chặt các lựa chọn kỹ thuật để mã nguồn không bị lẫn yêu cầu cũ, đồng thời giúp đổi thân bè từ 70×50 sang 98×68 cm mà không phải sửa một dòng code logic nào."

---

## Slide 4 — Kiến trúc bên trong C4.2 (Raspberry Pi 5 — 11 Node ROS 2 Jazzy)

- **Phân tách QoS triệt để ([qos.py](../l2_ros2_ws/src/nenuphar_nodes/nenuphar_nodes/qos.py)):**
  - `SENSOR_QOS` (`BEST_EFFORT, depth=1`): cho ảnh depth/stereo, IMU, VIO, vật cản → luôn lấy khung hình mới nhất, không tích tụ độ trễ.
  - `CMD_QOS` (`RELIABLE, depth=10`): cho trạng thái FSM, lệnh Guided, cảnh báo hành lang/năng lượng, heartbeat.
- **Lập lịch thời gian thực trên Pi 5:** gán `SCHED_FIFO` trên nhân riêng cho 4 node trọng yếu (`camera_driver`, `vio`, `perception_geom`, `mavlink_bridge`); đặt `blackbox` ở ưu tiên thấp (`1 Hz` thường, `10 Hz` tại `S4`/lỗi `>= 2`).
- **Chuỗi giám sát sống còn (Liveness Chain):**
  - 10 node kế thừa `MonitoredNode` phát `/system/node_heartbeat` (`1 Hz`) → `system_monitor` tính bitmask 10-bit (timeout `3 s`) + đọc tốc độ USB 3 thật từ `/sys/bus/usb/devices` → phát `/system/health` → `mavlink_bridge` gửi MAVLink `HEARTBEAT (compid=191)` xuống C4.1 và `KEEPALIVE` sang C8.
- **Tách sạch lõi logic thuần Python để kiểm thử độc lập:** `fsm_engine`, `mavlink_transforms`, `corridor_logic`, `energy_logic`, `clearance_logic`, `usb_probe`.

> *Lời dẫn gợi ý (~2 phút):* "Điểm mạnh trong kiến trúc C4.2 là tách lõi thuật toán khỏi lớp giao tiếp ROS 2 — tương tự cách tách `core/` và `hal/` ở L0 — giúp chạy unit test và property test tốc độ cao trên mọi máy mà không cần phần cứng thật."

---

## Slide 5 — Máy trạng thái Nhiệm vụ `S0–S12` (CdCT A.8) & 3 Tầng Lỗi

- **Thứ tự ưu tiên cứng (phân giải tập trung trong `resolve_fsm_state()`):**
  $$\text{S9} > \text{S8} > \text{S12} > \text{S7} > \text{S11} > \text{S10} > \text{S1}$$
- **Ý nghĩa đúng chuẩn CdCT Final A.8 (52 cạnh hợp lệ trong [cdct_spec_a8_a10.yaml](cdct_spec_a8_a10.yaml)):**
  - `S0` Kiểm tra USB 3 & node sống → `S1` Tuần tra Z1 → `S2` Tiếp cận cầu (`>= 10 m`, `<= 2 km/h`) → `S3` Canh giữa hai thành cầu (`±25 cm`, timeout `30 s -> S7`) → `S4` Đo tĩnh không (chưa đạt `-> S7` đo lại) → `S5` Qua cầu (EKF=VIO) → `S6` Thoát cầu & khóa lại RTK (`> 30 s -> S12/S7`).
  - `S7` Chờ/Hold (cũng là trạng thái kết thúc khi `S10` cập bến).
  - **`S8` Rời gầm cầu khẩn cấp** (khi gặp sự cố trong `S5` nhưng **còn lực đẩy** — không dừng chết dưới cầu!).
  - **`S9` Dừng khẩn cấp** (mất toàn bộ lực đẩy / E-stop / lỗi bậc 3 ngoài cầu + `HOLD-ALL <= 2 s`).
  - **`S11` Lách vật cản** (`g >= 1,35 m`, trong Z2 `±1,25 m`, `<= 10 s`) ưu tiên **cao hơn `S10` (RTB)** để bè đang về bến vẫn lách được vật cản.
  - **`S12` ĐDKC** (lỗi bậc 2: lệch Z1 `> 5 s` sau `1` lần thử lại, hoặc **`SoC <= 10%` khi còn lực đẩy**).

> *Lời dẫn gợi ý (~2 phút):* "Mọi trạng thái và ưu tiên đều có ý nghĩa vật lý rõ ràng: đang về bến gặp vật cản vẫn phải lách (`S11 > S10`); gặp lỗi dưới cầu hay pin còn 10% mà còn động cơ thì phải thoát cầu (`S8`) hoặc tấp vào ĐDKC (`S12`) thay vì thả trôi giữa kênh."

---

## Slide 6 — Hai Luồng Nghiệp vụ Trọng tâm: Qua cầu & Tránh vật cản

- **Chuỗi qua gầm cầu phối hợp 3 tầng (`S2 → S3 → S4 → S5 → S6`):**
  - Tại `S4`: đo tĩnh không bằng **Config B (800P)** ở `<= 2 km/h`; chỉ cấp phép khi `clearance >= total_height_above_water + 0,10 m`, sai lệch CSDL cầu `<= 0,05 m`, độ chụm `<= 0,05 m`. Kết quả đo cũ bị xóa ngay khi rời `S4`.
  - Tại `S5`: tự động gửi `MAV_CMD_SET_EKF_SOURCE_SET(2)` chuyển EKF sang VIO `ODOMETRY` (`ENU/FLU → NED/FRD`, bảo toàn `w = 0`), bật `BRIDGE_ZONE` (L1 nới `FS_GCS_TIMEOUT 5 s → 30 s`, L0 ức chế tự cắt nghiêng/tải), và cấp `BRIDGE_PERMIT` có hạn dùng **TTL 15 s**.
- **Hành lang TR-4.16 ([corridor_logic.py](../l2_ros2_ws/src/nenuphar_nodes/nenuphar_nodes/corridor_logic.py)):**
  - Chiếu lên polyline tuyến; phân biệt `Z1 (±0,30 m)` và `Z2 (±1,25 m)` theo bề rộng.
  - Ngoài Z1 khi không lách: đếm `5 s` → tự hiệu chỉnh thử lại `1` lần → thất bại mới vào `S12`. Đang lách `S11` trong Z2 không gây báo động giả; vượt `±1,25 m` lập tức vào `S7 Hold`.
- **Ngân sách độ trễ TR-4.1 (`<= 200 ms`):**
  - **Config A (400P):** `67` (chờ khung 15 Hz) + `5` (phơi sáng) + `21` (OAK-D) + `<= 65` (Pi 5) + `20` (L1 50 Hz) + `1` (DShot) = **`179 ms <= 200 ms`**.
  - **Config B (800P):** `~255 ms` → cấm dùng tránh vật cản, chỉ dùng tĩnh ở `S4 <= 2 km/h`.

> *Lời dẫn gợi ý (~2 phút):* "Đây là hai bài toán khó nhất của C4. Việc tách rõ Config A cho tránh vật cản dưới 200 ms và Config B cho đo cầu chậm dưới 2 km/h, kết hợp chuyển đổi trục ENU→NED và xóa kết quả đo cũ, giải quyết triệt để rủi ro va chạm gầm cầu."

---

## Slide 7 — Kiến trúc bên trong C4.1 (Pixhawk 6C — ArduPilot Rover + Lua)

- **Tham số chuẩn A.7 ([sitl_boat_a7.parm](../l1_ardupilot/params/sitl_boat_a7.parm)):**
  - `FRAME_CLASS=2` (Boat), `SCHED_LOOP_RATE=50 Hz`, `COMPASS_USE*=0`, `EK3_SRC1_YAW=2`, `VISO_TYPE=1` (`EK3_SRC2_*=6`).
  - Bật đủ hành động failsafe thật: `FS_ACTION=2` (Hold), `FS_GCS_ENABLE=1`, `BATT_MONITOR=4`, `BATT_FS_LOW_ACT=1` (RTL @ `12,2 V`), `BATT_FS_CRT_ACT=2` (Hold @ `11,5 V`), `FENCE_ENABLE=1` (`FENCE_ACTION=2`).
- **Lua 1 — Trộn lực đẩy & Nén dòng ([nenuphar_thruster_power.lua](../l1_ardupilot/lua/nenuphar_thruster_power.lua), 50 Hz):**
  - 4 chế độ `M1–M4`: `M1` bình thường (`70 A`), `M2`/`M3` hỏng 1 thruster chính dùng dự phòng đuôi (`40 A`), `M4` hỏng cả 2 chính đẩy thẳng thoát cầu bằng thruster đuôi (`<= 30 A/ESC`, không mô-men lái).
  - Lọc thông thấp **EMA (`alpha = 0,25`) + slew-rate bất đối xứng (`-0,12 / +0,03`)** triệt tiêu dao động dòng 25–50 Hz.
  - Phát hiện kẹt ESC qua telemetry + chạy tối đa **`3` xung đảo chiều gỡ rong** (TR-5.6) trước khi chốt lỗi.
- **Lua 2 — Watchdog Pi 5 & Khóa cầu ([nenuphar_watchdog_bridge.lua](../l1_ardupilot/lua/nenuphar_watchdog_bridge.lua), 10 Hz):**
  - Mất `HEARTBEAT (compid=191)` quá `3 s` → ép `HOLD`; giấy phép `BRIDGE_PERMIT` hết hạn sau **TTL `15 s`** nếu L2 chết.
  - **Giữ quyền ưu tiên con người:** không bao giờ ép `HOLD` nếu người vận hành chuyển sang `MANUAL (0)` hoặc `STEERING (3)`.

> *Lời dẫn gợi ý (~2 phút):* "Tầng C4.1 không chỉ nhận lệnh từ Pi 5 mà đóng vai trò lưới bảo vệ độc lập: tự lọc chống dao động dòng, tự gỡ rong 3 chu kỳ, tự khóa cầu khi phép hết hạn, và luôn nhường quyền cho người lái nếu chuyển sang chế độ thủ công."

---

## Slide 8 — Kết quả Kiểm chứng, Minh họa & Lộ trình HIL Trung thực

| Hạng mục kiểm thử | Kết quả hiện tại | Phạm vi xác minh thực tế |
| :--- | :---: | :--- |
| **L2 ROS 2 Jazzy (`colcon test`)** | **`24/24` PASS** | Đủ 11 node, QoS, nối dây `/operator/cmd` & L1 RX, `rosbag2`, property test Tầng 3 trên cả `13` trạng thái |
| **L1 ArduPilot Boat SITL** | **`1/1` PASS** | Failsafe params thật, Lua `M1–M4`, EMA, WD `3 s`, giữ quyền `MANUAL`, EKF switch |
| **Phủ bảng CdCT A.8 (SITL)** | **`52/52` cạnh PASS** | Đủ 100% cạnh hợp lệ trong `cdct_spec_a8_a10.yaml`, `0` lỗi ngoài bảng (kịch bản) |
| **L0 Bo C8 C++ (`ctest` `-Werror`)** | **`10/10` PASS** | `MockC8Hal`: start/reset bắt sườn, TR-8.3, `NaN`/trễ fail-safe, lưu counter LoRa, `30/30` E-stop mock |
| **Độ trễ đo trên node stub** | `P99 = 14,13 ms` · `Max = 26,52 ms` | Xác nhận khung đo `latency_probe` (`128,13 ms` P99 + HW / `140,52 ms` Max + HW); **chờ camera thật** |

> **Ghi chú minh bạch cho Hội đồng:** 3 biểu đồ dưới đây và video [demo_nenuphares_v1.mp4](../demo_output/demo_nenuphares_v1.mp4) là **kịch bản minh họa dựng sẵn** (chạy SITL + ROS 2 stub + lõi C++ C8) để trực quan hóa tương tác đa tầng, **không thay thế** cho kiểm thử vòng kín HIL/thực địa.

![Minh họa quỹ đạo hành lang Z1/Z2](../demo_output/fig1_quy_dao_hanh_lang.png)

![Minh họa chuỗi thời gian đa tầng L0-L1-L2](../demo_output/fig2_chuoi_thoi_gian_su_kien.png)

![Khung đo độ trễ stub & Tổng hợp kiểm thử](../demo_output/fig3_do_tre_P99_va_kiem_thu.png)

- **Các bước tiếp theo khi có phần cứng (HIL — [hil_poweron_checklist.md](hil_poweron_checklist.md)):**
  1. Tích hợp `depthai-ros` với OAK-D Pro W thật → đo `P99` & `Max` nhận thức thực tế trên Pi 5 (`<= 50 ms`).
  2. Chạy `VINS-Fusion-ROS2` vòng kín với Pixhawk 6C & GNSS 2 ăng-ten ngoài trời.
  3. Ghép driver RP2350 (MPU-6050, 4× HX711, SX1262, Flash) vào khung [l0_c8_pico2/firmware](../l0_c8_pico2/firmware/main.cpp) và chạy nghiệm thu **30/30 lần E-stop vật lý**.

> *Lời dẫn gợi ý (~2 phút):* "Đến thời điểm hiện tại, toàn bộ kiến trúc, giao diện, lõi logic và failsafe 3 tầng đã chạy kiểm thử tự động xanh 100% trên CI và SITL. Chúng em tách bạch rõ phần đã chứng minh bằng unit/SITL và phần chờ phần cứng thật để sẵn sàng bước vào giai đoạn HIL ngay khi linh kiện về."

---

## Phụ lục — Bộ 6 câu hỏi phản biện Q&A & Checklist mở tab khi báo cáo

| Câu hỏi giảng viên hay hỏi | Cách trả lời ngắn gọn, đúng trọng tâm |
| :--- | :--- |
| **1. Pi 5 (C4.2) treo giữa đường hoặc ngay trước cầu thì sao?** | Sau `3 s`, Lua watchdog trên Pixhawk tự ép `HOLD`; `BRIDGE_PERMIT` hết hạn sau TTL `15 s`; mất keepalive C4→C8 `3 s` kích `HOLD_REQ` (vẫn đóng rơ-le giữ vị trí, không thả trôi). Người lái vẫn luôn giành lại điều khiển bằng `MANUAL`/`STEERING`. |
| **2. Tại sao pin `<= 10%` hay lỗi dưới cầu không ngắt nguồn ngay?** | Cắt đẩy dưới cầu hay giữa dòng nguy hiểm hơn là điều hướng an toàn: còn đẩy dưới cầu thì vào `S8` thoát cầu trước; pin `<= 10%` còn đẩy thì vào `S12` tấp ĐDKC; chỉ mất đẩy hoàn toàn hoặc E-stop mới vào `S9` / ngắt cứng C8. |
| **3. Số liệu P99 và 30/30 E-stop đã là đo thật trên bè chưa?** | **Chưa.** `P99 = 14,13 ms / Max = 26,52 ms` đo khung truyền thông ROS 2 trên node stub; `30/30` E-stop là trên `MockC8Hal`. Công cụ đo thật (`latency_probe`, `analyze_blackbox_bag.py`) và `hil_poweron_checklist.md` đã dựng sẵn cho bàn HIL. |
| **4. Nếu nhóm cơ khí đổi thân bè từ 70×50 sang 98×68 cm (TBD-06)?** | Không phải sửa logic C4; chỉ đổi `raft_profile` sang `v1_98x68.yaml` (đã có đủ kích thước, tọa độ cảm biến/thruster và `total_height_above_water_m` cho phép đo cầu `S4`). |
| **5. Dữ liệu VIO từ ROS 2 sang Pixhawk có bị lệch hệ trục không?** | Đã tách module `mavlink_transforms.py` chuyển tường minh ROS `ENU/FLU → NED/FRD`, bảo toàn góc 180° (`w=0`), dùng stamp gốc, hiệp phương sai hữu hạn và có unit test pose kiểm chứng. |
| **6. Nếu người vận hành đè kẹt nút Reset trên Bo C8 hoặc cảm biến nghiêng bị `NaN`?** | C8 bắt **sườn lên** cho cả Reset và Start (kẹt nút không thể mở chốt `STOPPED` và không tự đóng rơ-le); cảm biến `NaN`/quá hạn `> 500 ms` là fail-safe (chặn arm ở `SAFE_OFF`, cắt `STOPPED` khi `ARMED`). |

### 4 tab nên mở sẵn trước khi trình bày
1. [PRESENTATION_C4.md](PRESENTATION_C4.md) (hoặc slide xuất từ tệp này) + [SYSTEM_ARCHITECTURE_AND_LOGIC.md](SYSTEM_ARCHITECTURE_AND_LOGIC.md) khi cần phóng to sơ đồ 11 node / FSM 52 cạnh.
2. [tr_fr_traceability_matrix.md](tr_fr_traceability_matrix.md) (đối chiếu minh bạch từng yêu cầu TR/FR).
3. Tab **GitHub Actions** của repo (cho thấy 3 job CI xanh thực tế).
4. Video [demo_nenuphares_v1.mp4](../demo_output/demo_nenuphares_v1.mp4) (phát ở cuối để minh họa tổng hợp 5 kịch bản).
