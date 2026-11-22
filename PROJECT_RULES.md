# PROJECT_RULES.md — Quy tắc phát triển phần mềm NÉNUPHARES (Phụ lục A)

Tài liệu này là tập luật cứng cho lập trình viên và AI coding agent (Google Antigravity) khi phát triển, kiểm thử và tích hợp phần mềm cho dự án **NÉNUPHARES BONNEVAL** bám sát **CdCT Final 2.0 (20/11/2026)**.

## 1. Phạm vi phiên bản (V1 vs V2)

| Hạng mục | V1 (Mục tiêu hiện tại tại Việt Nam) | V2 (Triển khai thực tế tại Bonneval, Pháp) |
| :--- | :--- | :--- |
| Quy mô | 1 bè mẫu kiểm thử các cấp L0–L2 | 40 bè (lô thí điểm 3–5 bè ở L3 trước cổng G-V2) |
| Trạm điều khiển | QGroundControl trên laptop | Web HMI trung tâm (Phụ lục B) |
| Cảm biến & cơ cấu | Không neo, không UWB, không phân đoạn B6A | Có neo, tùy chọn UWB bè-tới-bè, bật B6A (YOLOv8n-seg) |
| Đội hình (`formation`) | Dựng khung giao diện tối thiểu | Đầy đủ Leader/Deputy `s_ref(t)` >= 2 Hz + dự phòng đồng hồ GNSS |
| Tần số LoRa thử nghiệm | 920–923 MHz (QCVN 122:2020/BTTTT) | 869,4–869,65 MHz (chu kỳ phát <= 10%) |

## 2. Ngăn xếp phần mềm ghim phiên bản (Bắt buộc)

- **Hệ điều hành tầng L2 (Pi 5 & PC phát triển):** Ubuntu 24.04 LTS (Noble Numbat) + **ROS 2 Jazzy Jalisco** (ghim phiên bản, ghi rõ commit/tag trong `VERSIONS.md`).
- **Tầng L1 (Pixhawk 6C):** **ArduPilot Rover** bản ổn định (ghim tag `Rover-stable`), cấu hình khung thuyền `FRAME_CLASS=2`, `SCHED_LOOP_RATE=50 Hz` + **Lua scripting**.
- **Tầng cảm biến thị giác (OAK-D Pro W — RVC2):** `depthai-ros`, camera màu **OV9782** (1 MP, global shutter, DFOV 150°, đồng bộ FOV với stereo OV9282 HFOV 127°).
- **Tầng L0 (Bo an toàn C8 — Raspberry Pi Pico 2):** Bare-metal **C/C++** tách sạch lớp logic (`core/`) khỏi lớp phần cứng (`hal/`) để chạy host unit test trực tiếp trên PC.
- **Điều tốc (ESC):** 3× **Holybro Tekko32 F4 45A** chạy firmware **AM32** (Bidirectional DShot + UART telemetry, giới hạn 30 A/ESC, PWM 24 kHz, về 0 khi mất tín hiệu).

## 3. Luật cứng chống lẫn chi tiết cũ (Ưu tiên cao nhất)

Agent **tuyệt đối không** sinh code, cấu hình hoặc tài liệu vi phạm các quy tắc đã chốt ở **Final 2.0** dưới đây:

1. **KHÔNG có LiDAR:** Hệ thống chỉ dùng camera stereo **OAK-D Pro W + IMU**, lắp cao **20 cm** trên boong với **trục quang nằm ngang (`alpha_tilt = 0°`)**. Bỏ qua mọi câu chữ sót lại về "camera nghiêng lên" (mục A.2 cũ) hoặc LiDAR.
2. **KHÔNG dùng la bàn từ (compass) để định hướng:** Hướng mũi (heading) lấy từ **GNSS 2 ăng-ten** theo D-05/A.7. Bỏ qua câu sót lại ở A.5.1 về dùng từ kế căn hướng ban đầu.
3. **KHÔNG có nhánh nghiên cứu ghép chặt (TBD-28):** Nhánh nghiên cứu đã bị loại bỏ ở Final 2.0; chỉ triển khai luồng VIO sản phẩm (`Basalt` hoặc `VINS-Fusion` bản ROS 2 Jazzy, chốt 1 gói duy nhất).
4. **KHÔNG dùng BMS UART/CAN và KHÔNG dùng cảm biến dòng ACS:** Bỏ qua bảng A.12 cũ. Mọi giao tiếp nguồn và động lực phải bám đúng **D-22, D-24 và A.14.1–A.14.2**: đo dòng/áp chính bằng **shunt**, đối chiếu BMS qua **Bluetooth LE** (mất Bluetooth chỉ ghi cảnh báo, không dừng bè), và đọc telemetry từ **3 ESC** (không phải 2 ESC như lỗi sót ở mục B5).
5. **Máy trạng thái đầy đủ là `S0–S12`:** Không rút gọn về `S0–S10`. Thứ tự ưu tiên cứng: `S9 > S8 > S12 > S7 > S11 > S10 > S1`.
6. **KHÔNG hardcode kích thước hình học thân bè (TBD-06 / RI-1.5):** Thân hiện tại `70 × 50 × 25 cm` đang có rủi ro ổn định (GM âm, mạn khô ~5,4 cm) và có thể phải nâng lên `98 × 68 × 25 cm` sau thử nghiêng G-ST. Mọi khoảng cách thruster, chiều cao camera, kích thước bao an toàn phải đọc từ `config/raft_profiles/*.yaml`.

## 4. Quy chuẩn kiến trúc & hiệu năng

- **QoS ROS 2:** Các luồng ảnh/độ sâu/IMU và nhận thức dùng hàng đợi **depth = 1 (non-blocking, `BEST_EFFORT`)** để luôn xử lý khung hình mới nhất, không tích tụ độ trễ.
- **Lập lịch thời gian thực trên Pi 5:** Gán `SCHED_FIFO` trên nhân CPU riêng cho 4 nút trọng yếu (`camera_driver`, `vio`, `perception_geom`, `mavlink_bridge`); đặt `blackbox` ở mức ưu tiên thấp; đồng bộ thời gian qua `chrony` ở chế độ **slew-only** (cấm bước nhảy thời gian).
- **Ngân sách độ trễ TR-4.1 (<= 200 ms):** Chế độ tránh vật cản bắt buộc dùng **Config A (400P depth)** với thời gian xử lý trên Pi 5 **<= 50 ms** (tổng chuỗi xấu nhất ~180 ms). **Config B (800P depth)** (~255 ms) **chỉ được dùng** khi đo tĩnh không cầu ở trạng thái `S4` với tốc độ <= 2 km/h.
- **Ngưỡng cắt pin rõ ràng:** BMS phần cứng cắt ở **10,0 V**; ngưỡng phần mềm tầng L1 (`BATT_CRT`) đặt ở **11,5–12,0 V** hoặc **SoC <= 10%**.
- **An toàn bo C8:** Trạng thái `STOPPED` phải chốt cứng (latched), chỉ được reset thủ công trực tiếp tại bè (EN ISO 13850); cờ `BRIDGE_ZONE` (làm mới <= 60 s) chỉ ức chế cắt tự động do nghiêng/tải, tuyệt đối không chặn nút dừng tay hoặc gói `STOP` qua LoRa.

## 5. Cấu trúc kho mã nguồn chuẩn (`nenuphares-sw`)

```text
nenuphares-sw/
├── PROJECT_RULES.md
├── VERSIONS.md
├── docs/
│   ├── SPEC_PHU_LUC_A.md
│   ├── INTERFACE_FREEZE_A14.md
│   ├── tbd20_tuning_tracker.csv
│   └── hil_poweron_checklist.md
├── config/
│   ├── raft_profiles/
│   │   ├── v1_70x50.yaml
│   │   └── v1_98x68.yaml
│   ├── energy_A9.yaml
│   └── corridor_D16.yaml
├── l0_c8_pico2/        # C/C++ bare-metal (core/, hal/, tests/)
├── l1_ardupilot/       # params/ (A.7, SITL) & lua/ (M1-M4, watchdog, mock tests/)
└── l2_ros2_ws/         # ROS 2 Jazzy workspace (nenuphar_msgs + 11 nodes + sim_tests)
```

## 6. Quy trình kiểm thử bắt buộc (Mục A.13)

Mọi thay đổi code phải đi qua đủ các cấp trước khi hợp nhất:
1. **Unit test (CI):** `colcon test` cho ROS 2, mock Lua API cho script L1, GoogleTest/CTest trên PC cho `l0_c8_pico2/core/`.
2. **SITL (ArduPilot Rover khung Boat):** Mỗi chuyển trạng thái hợp lệ trong bảng `S0–S12` (A.8) phải được kích hoạt **>= 1 lần**, không phát sinh chuyển trạng thái ngoài bảng.
3. **HIL (khi có linh kiện):** Đo độ trễ P99 thỏa mãn ngân sách A.11 và thử tối thiểu **30 lần E-stop** (nút nhấn + LoRa) với tỷ lệ cắt 100%.
