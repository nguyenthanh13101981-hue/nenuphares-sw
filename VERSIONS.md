# VERSIONS.md — Ngăn xếp phần mềm ghim phiên bản (NÉNUPHARES BONNEVAL — V1)

Cập nhật: `2026-11-22` (Mốc A — CdCT Final 2.0)

## 1. Hệ điều hành & Trình biên dịch (PC phát triển / WSL 2)

| Thành phần | Phiên bản ghim / Thực tế | Ghi chú |
| :--- | :--- | :--- |
| OS | Ubuntu `24.04.5 LTS` (`noble`) | Trên WSL 2 (`x86_64`) |
| Kernel | `6.18.58.1-microsoft-standard-WSL2` | |
| Python | `3.12.3` | |
| C++ Compiler | `g++ 13.3.0` | Xác nhận qua ArduPilot `waf configure` |

## 2. Tầng L2 — ROS 2 Jazzy & Công cụ phát triển

| Gói / Thành phần | Phiên bản ghim | Nguồn cài đặt |
| :--- | :--- | :--- |
| `ros-jazzy-desktop` | `0.11.0-1noble.20261105.164848` | ROS 2 apt repo (`noble main`) |
| `python3-colcon-common-extensions` | `0.3.0-100` | ROS 2 / Ubuntu apt |
| `python3-rosdep` | `0.27.0-1` | Ubuntu / ROS 2 apt |
| `python3-empy` | `3.3.4-2` (`empy 3.3.4`) | Ubuntu apt (tương thích ArduPilot `waf`) |
| `python3-pip` | `24.0+dfsg-1ubuntu1.3` | Ubuntu apt |
| `pymavlink` | `2.4.50` (`fastcrc 0.5.0`) | `pip3 --user` |

## 3. Tầng L1 — ArduPilot Rover SITL (`~/sim/ardupilot`)

| Hạng mục | Giá trị ghim | Ghi chú |
| :--- | :--- | :--- |
| Repository | `https://github.com/ArduPilot/ardupilot.git` | Clone tại `~/sim/ardupilot` |
| Tag ổn định | `APMrover2-stable` (`Rover-4.7.3`) | Upstream ArduPilot dùng tên `APMrover2-stable` cho nhánh/tag ổn định của Rover (thay cho tên gọi tắt `Rover-stable` trong `PROJECT_RULES.md`) |
| Commit SHA | `d9c5097c3d8333435712ffed86bca8ce00c4bb46` | Đã kiểm chứng cùng tham chiếu cho cả `APMrover2-stable` và `Rover-4.7.3` |
| Cấu hình SITL đã test | `--model motorboat`, `FRAME_CLASS=2`, `SCHED_LOOP_RATE=50` | Đọc xác nhận qua MAVLink (`HEARTBEAT type=11`, `PARAM_VALUE`) dùng `l1_ardupilot/params/sitl_boat_base.parm` |

## 4. Hạng mục chờ chốt ở các mốc tiếp theo

- **VIO ROS 2 (`Basalt` vs `VINS-Fusion`):** Chưa chốt (kiểm tra khả năng biên dịch tại Mốc B).
- **`depthai-ros` (OAK-D Pro W):** Chưa cài (triển khai khi tích hợp driver cảm biến).
