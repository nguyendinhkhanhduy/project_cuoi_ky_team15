# Bsp – driver phần cứng

Gọi STM32 HAL. Mọi gán chân và cờ phụ thuộc cách lắp ráp nằm trong `board_config.h`.

## Thư mục con

| Thư mục | File | Vai trò |
|---|---|---|
| (gốc) | `board_config.h` | Handle CubeMX (`hi2c1`, `huart1`, `htim2/3/4`), chân, cờ đảo chiều động cơ/encoder, mức IR, dấu gyro |
| `actuators/` | `motor.*` | TB6612: PWM vào chân IN (PWMA/PWMB nối cứng mức cao), STBY; đọc delta encoder 16 bit an toàn khi tràn |
| `sensors/` | `mpu6050.*` | Gyro Z ±500 °/s, lọc 44 Hz, 200 Hz, hiệu chỉnh bias lúc đứng yên |
| | `vl53l0x.*` | Driver VL53L0X (port từ Pololu) |
| | `tof.*` | Khởi động 4 VL53L0X qua XSHUT, gán địa chỉ 0x30 + i, đo liên tục; cảm biến lỗi I2C bị bỏ |
| | `inputs.*` | 2 IR, nút SW1 |
| `comm/` | `uart_io.*` | UART không chặn: bộ đệm vòng gửi/nhận, dò cờ TXE/RXNE, không cần ngắt |
| `storage/` | `flash_store.*` | Lưu một khối dữ liệu vào Flash sector 5 (0x08020000), kiểm tra CRC32 và phiên bản |

## Gán chân (theo sơ đồ nguyên lý, không đổi nếu không đổi PCB)

| Chức năng | Chân | CubeMX |
|---|---|---|
| AIN1 / AIN2 (bánh trái) | PA0 / PA1 | TIM2_CH1 / CH2 PWM |
| BIN1 / BIN2 (bánh phải) | PA2 / PA3 | TIM2_CH3 / CH4 PWM |
| STBY | PB0 | GPIO output |
| Encoder trái / phải | PB4-PB5 / PB6-PB7 | TIM3 / TIM4 Encoder Mode TI1 and TI2, Period 65535 |
| XSHUT1..4 (trái → phải) | PA4..PA7 | GPIO output, mức thấp |
| I2C1 SCL / SDA | PB8 / PB9 | I2C1 400 kHz |
| IR trái / phải | PB13 / PB14 | GPIO input |
| HC-05 TX / RX | PA9 / PA10 | USART1 9600 baud |
| SW1 | PA11 | GPIO input pull-up, mức thấp khi nhấn |

Hai chỗ CubeMX gán mặc định bị trùng chân, phải chuyển tay:
- **TIM3** mặc định ở PA6/PA7, trùng với XSHUT → chuyển sang **PB4/PB5**.
- **I2C1** mặc định ở PB6/PB7, trùng với TIM4 → chuyển sang **PB8/PB9**.

**TIM2:** Prescaler 0. Period 799 nếu chạy 16 MHz, hoặc 4199 nếu chạy 84 MHz (ra 20 kHz; TB6612 tối đa 100 kHz).

## Lưu ý

- **Không dùng `HAL_UART_Transmit` song song với `uart_io`.** Gọi `Uart_Service()` thật thường xuyên. `Uart_Flush()` là hàm chặn, chỉ dùng khi xe đứng yên.
- **Ghi Flash:** xoá sector 128 KB mất khoảng 1–2 s và CPU bị treo trong lúc đó, nên chỉ ghi khi xe đứng yên. `Store_Save` tự từ chối ghi nếu firmware vượt quá 128 KB.
- **Cờ cần kiểm tra trên xe thật** (dùng `$stream 1`): `MOTOR_*_INVERT`, `ENC_*_INVERT`, `MPU6050_Z_SIGN`, `IR_ON_TAPE_LEVEL`.
