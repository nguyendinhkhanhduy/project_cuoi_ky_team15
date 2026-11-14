# Firmware xe hai bánh vi sai tự hành – team 15

Xe 10 × 9 cm chạy sân chữ U 180 × 120 cm: 17 vật cản tĩnh, 1 vật cản ngẫu nhiên ở Lượt 2.
Lượt 1 lái tay qua Bluetooth (ghi quỹ đạo), Lượt 2 tự hành theo quỹ đạo A* và né vật lạ.

## Tech stack

| Mảng | Thành phần |
|---|---|
| Vi điều khiển | STM32F401CCU6 Black Pill (Cortex-M4F, 256 KB Flash, 64 KB RAM), đang chạy HSI 16 MHz |
| Phần cứng | TB6612FNG + 2 motor N20 có encoder, 4 × VL53L0X, MPU6050, 2 IR góc trước, HC-05, nút SW1, pin 2S 7,4 V + buck 5 V |
| Ngoại vi | TIM2 PWM (động cơ), TIM3/TIM4 Encoder Mode, I2C1 400 kHz (ToF + IMU), USART1 9600 baud (HC-05) |
| Firmware | C11, STM32 HAL (STM32Cube FW F4 V1.28.3), không RTOS, vòng điều khiển 200 Hz |
| Build / nạp | STM32CubeMX → CMake + Ninja, arm-none-eabi-gcc (bundle `cube`), ST-Link SWD, VS Code + STM32 extension |
| Thuật toán | A* offline (lưới 1 cm, 8 hướng, phạt sát vật), EKF (x, y, θ), Pure Pursuit, A* cục bộ trên xe, PI bánh, Ramer–Douglas–Peucker |
| Công cụ PC | Python 3 (numpy, scipy, matplotlib), gcc để mô phỏng |

## Thư mục

| Thư mục | Nội dung | Xem |
|---|---|---|
| `Algo/` | Thuật toán dẫn đường, không gọi HAL, mô phỏng được trên PC | [Algo/README.md](Algo/README.md) |
| `Bsp/` | Driver phần cứng (HAL) và gán chân | [Bsp/README.md](Bsp/README.md) |
| `App/` | Vòng điều khiển, nút, lệnh Bluetooth, bài test hiệu chỉnh | [App/README.md](App/README.md) |
| `tools/` | Lập quỹ đạo offline, mô phỏng, vẽ log | [tools/README.md](tools/README.md) |
| `docs/` | Thiết kế và lịch sử quyết định | [docs/README.md](docs/README.md) |

Thư mục này chỉ chứa code của nhóm. `main.c`, HAL và startup do CubeMX sinh.

## Tích hợp vào project CubeMX

1. Tạo project STM32F401CCUx, Toolchain = CMake, cấu hình chân theo [Bsp/README.md](Bsp/README.md).
2. Linker script: đổi `FLASH LENGTH = 256K` thành `128K`. Sector 5 dùng để lưu cấu hình.
3. `CMakeLists.txt`:
   ```cmake
   file(GLOB_RECURSE USER_SRC Algo/*.c Bsp/*.c App/*.c)
   target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SRC})
   target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
       App Bsp Bsp/actuators Bsp/sensors Bsp/comm Bsp/storage
       Algo/core Algo/common Algo/localization Algo/planning Algo/control Algo/teach)
   target_link_libraries(${CMAKE_PROJECT_NAME} m)
   ```
4. `main.c`: `#include "robot_app.h"`, gọi `RobotApp_Init()` ở `USER CODE BEGIN 2`, `RobotApp_Task()` trong `while(1)`.
5. Build và nạp: `cube cmake --preset Debug`, `cube cmake --build --preset Debug`,
   `cube programmer -c port=SWD -w build/Debug/<ten>.elf -v -rst`.

## Vận hành

- Bật nguồn, giữ xe yên khoảng 1 s để hiệu chỉnh gyro. HC-05 in trạng thái IMU, ToF và cấu hình đang dùng.
- **Lượt 1:** lái bằng app "Bluetooth RC Controller". Lệnh lái đầu tiên bắt đầu ghi; nhấn SW1 để kết thúc. Quỹ đạo tự lưu Flash.
- **Lượt 2:** đặt xe ở ô xuất phát (30, 6) cm, mũi hướng lên.
  - Nhấn rồi thả SW1: chạy quỹ đạo A*.
  - Giữ SW1 từ 1 s trở lên rồi thả: chạy quỹ đạo Lượt 1.
- **Dừng khẩn cấp:** nhấn SW1 hoặc gõ `$stop`.
- **Hiệu chỉnh và lệnh `$...`:** xem [App/README.md](App/README.md).

## Trạng thái

- Chưa chạy trên xe thật. Mọi số liệu đều từ mô phỏng (2026-11-14):

  | Bài mô phỏng | Kết quả |
  |---|---|
  | Đường gốc, 200 lượt có nhiễu | 200/200 về đích sạch |
  | Vật lạ tại (135, 67), 100 lượt | 95/100 |
  | Lượt 1 rồi Lượt 2, 100 lượt | 100/100 |
  | Vật lạ đặt dọc quỹ đạo (26 vị trí) | 23 về đích, 3 dừng an toàn, 0 va chạm |

- Phải đo trên xe các thông số: xung/cm, khoảng cách bánh, hệ số PI, nhiễu EKF, vị trí ToF/IR, chiều quay. Nếu dùng nguyên giá trị mặc định thì khi nhiễu gấp đôi mô phỏng chỉ còn khoảng 66%.
