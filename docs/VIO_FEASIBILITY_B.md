# VIO_FEASIBILITY_B.md — Đánh giá khả thi biên dịch & chốt gói VIO (Mốc B)

Ngày kiểm chứng: `2026-11-22`  
Môi trường thử nghiệm: **Ubuntu 24.04.5 LTS** (`noble`, WSL 2 `x86_64`), **ROS 2 Jazzy Jalisco** (`0.11.0-1noble.20261105.164848`), **`g++ 13.3.0`**, **Eigen `3.4.0`**, **OpenCV `4.6.0`**, **oneTBB `2021.11.0`**, **Ceres Solver `2.2.0`**.

---

## 1. Bảng so sánh thực nghiệm giữa Basalt và VINS-Fusion-ROS2

| Tiêu chí | `VINS-Fusion-ROS2` (`zinuok`) | `Basalt` (`basalt_ros` + `basalt_wrapper` + `basalt`) |
| :--- | :--- | :--- |
| Kho mã nguồn & Commit | `zinuok/VINS-Fusion-ROS2` (`main`)<br>`72023bc5d2abb14faa4c02a5c040d84f996b587a` (`2026-04-15`) | `berndpfrommer/basalt_ros` (`1264f86c`, `2023-10-12`)<br>`basalt_wrapper` (`5b261f65`, `2023-01-19`)<br>`berndpfrommer/basalt` (`f4686556`, `2023-01-20`) |
| Trạng thái bảo trì | Còn hoạt động (cập nhật tới 04/2026) | `basalt_ros` đánh dấu **INACTIVE** (unsupported); `basalt_ros2` đã **retired**; upstream `VladyslavUsenko/basalt` (`0f3b2b52`, 03/2026) chuyển sang `vcpkg` và bỏ wrapper ROS 2 |
| Số lần build thử đến khi qua | **1 lần** (`5m 01s`, cả 4 gói: `camera_models`, `global_fusion`, `loop_fusion`, `vins`) | **5 lần** (~`12m 50s` thực build sau nhiều lần thất bại ở `opengv`, `Pangolin`, `cereal`, `basalt_ros`) |
| Bản vá phải tự duy trì cho Jazzy / GCC 13 | Rất ít (9 file): đổi `<cv_bridge/cv_bridge.h>` -> `<cv_bridge/cv_bridge.hpp>` (5 file) và `CMAKE_CXX_STANDARD 17` (4 `CMakeLists.txt`). Đã dùng sẵn `ceres::Manifold`. | Nặng và rủi ro: gỡ `-Werror` khỏi **17 file CMake** (`opengv` `-Werror=array-bounds`, `cereal` `-Werror=dangling-reference` / `-Werror=unused-variable`), ép cờ `-include cstdint` cho `Pangolin` v0.5, sửa `cv_bridge.hpp` ở `basalt_ros`. |
| Khả năng chạy CPU thuần trên Pi 5 (ARM64) | Hỗ trợ sẵn `use_gpu: 0`, `use_gpu_acc_flow: 0`, `use_gpu_ceres: 0` (chỉ CPU). | Hỗ trợ CPU thuần (chỉ phụ thuộc Eigen/TBB), nhưng chuỗi submodule cũ (2023) tăng rủi ro lỗi toolchain mới trên ARM64. |
| Smoke test sau build | `vins_node` khởi động sạch, đăng ký node `/vins_estimator`, mở đúng topic đầu vào (`/cam0/image_raw`, `/cam1/image_raw`, `/imu0`) và đầu ra (`/odometry`, `/imu_propagate`, v.v.). | `ldd` trên `vio_frontend_node` liên kết đủ `libbasalt.so` và `libbasalt_vio_msgs*.so`. |

---

## 2. Chi tiết các lỗi biên dịch quan sát được

### `VINS-Fusion-ROS2`
- Đã tương thích sẵn với **Ceres 2.2.0** (`ceres::Manifold`, `ceres::QuaternionManifold`, `ceres::AutoDiffManifold`).
- Chỉ cần 2 điều chỉnh chuẩn của ROS 2 Jazzy:
  1. Header `cv_bridge/cv_bridge.h` đã bỏ trên Jazzy -> đổi sang `cv_bridge/cv_bridge.hpp` ở 5 file (`vins` và `loop_fusion`).
  2. Nâng `CMAKE_CXX_STANDARD` từ `14` lên `17` ở 4 `CMakeLists.txt` để khớp header ROS 2 Jazzy (`rclcpp`).
- Kết quả: 4 gói biên dịch thành công (`MAKEFLAGS=-j3`), 126 warning, 0 error.

### `Basalt` (`basalt_ros`)
- Lần 1 (`3m 10s`): `opengv` lỗi `-Werror=array-bounds` với GCC 13 (`Eigen::Matrix<double, 9, 1>` với AVX intrinsics).
- Lần 2 (`3m 05s`): `Pangolin` v0.5 đóng kèm lỗi `'uint8_t' was not declared in this scope` do GCC 13 siết header `<cstdint>`.
- Lần 3 (`4m 35s`): `cereal` đóng kèm trong `basalt-headers` lỗi `-Werror=unused-variable` và `-Werror=dangling-reference`.
- Lần 4 (`12m 03s`): sau khi gỡ `-Werror` khỏi 17 file CMake và truyền `-include cstdint`, `basalt` (`11m 36s`) và `basalt_vio_msgs` (`9.75s`) biên dịch xong nhưng `basalt_ros` lỗi thiếu `cv_bridge/cv_bridge.h`.
- Lần 5 (`47.6s`): vá `cv_bridge.hpp` xong thì `basalt_ros` biên dịch qua.

---

## 3. Đề xuất chốt gói VIO (Luật cứng #3 — `PROJECT_RULES.md`)

- **Chốt sử dụng 1 gói duy nhất:** **`VINS-Fusion-ROS2`** (`zinuok/VINS-Fusion-ROS2`, nhánh `main`, bật chế độ thuần CPU `use_gpu: 0`).
- **Lý do chính:**
  1. Biên dịch sạch trên đúng bộ công cụ Ubuntu 24.04 + ROS 2 Jazzy + GCC 13.3 + Ceres 2.2 với chi phí vá lỗi cực thấp và dễ đóng gói lại cho Raspberry Pi 5 (ARM64).
  2. Kho mã nguồn còn được cập nhật (04/2026), tránh phụ thuộc vào bộ wrapper ROS 2 của Basalt vốn đã ngừng duy trì từ năm 2023 và đóng kèm nhiều thư viện C++ cũ lỗi thời với GCC 13.
  3. Phù hợp trực tiếp với cấu hình cảm biến đã chốt (OAK-D Pro W stereo global shutter 15 Hz + IMU 200 Hz, không dùng LiDAR, không dùng nhánh ghép chặt TBD-28).
