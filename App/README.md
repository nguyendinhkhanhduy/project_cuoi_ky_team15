# App – lớp ứng dụng

Nối `Bsp/` (driver) với `Algo/` (thuật toán).

| File | Vai trò |
|---|---|
| `robot_app.*` | `RobotApp_Init` / `RobotApp_Task`: vòng 200 Hz, SW1, lệnh Bluetooth, log, lưu Flash |
| `wheel_ctrl.*` | PI vận tốc từng bánh + feedforward + bù ω theo gyro (không gọi HAL). Bám ω do Pure Pursuit yêu cầu, **không giữ hướng cố định** |
| `bench.*` | Bài test hiệu chỉnh: `$motor`, `$straight`, `$spin` |

## Vòng chạy

- **Trạng thái App:** `IDLE`, `TEACH` (Lượt 1), `RUN` (Lượt 2), `BENCH` (bài test).
- **Mỗi 5 ms:** đọc encoder và gyro → đọc ToF và IR → `Nav_Step` → `Wheel_Step` → PWM.
  - Mất gyro thì lấy vận tốc góc từ encoder.
  - Chu kỳ dài quá 50 ms thì dừng xe (thường do I2C treo). Lần gọi có chạy A* lúc xe đang đứng (trạng thái BRAKE) được trừ ra.
- **Đầu vào từ HC-05:** dòng bắt đầu bằng `$` là lệnh; ký tự đơn là lệnh lái của app RC (app này không bao giờ gửi `$`).
- **Lưu Flash:** `AppStore` gồm `NavParams`, `WheelGains` và quỹ đạo Lượt 1. Lưu khi gõ `$save` và tự lưu sau Lượt 1. Đổi struct thì phải tăng `STORE_LAYOUT`.

## Lệnh Bluetooth

| Lệnh | Tác dụng |
|---|---|
| `$help`, `$status` | Danh sách lệnh, trạng thái |
| `$get [ten]`, `$set ten gia_tri` | Đọc/ghi `NavParams` và `wheel_kff`, `wheel_kp`, `wheel_ki`, `wheel_imax`, `wheel_kw` |
| `$save`, `$load`, `$defaults` | Lưu Flash, đọc lại, về mặc định |
| `$stream 1` / `0` | In encoder, gyro, 4 ToF, IR mỗi 200 ms (khi đứng yên) |
| `$motor L% R% [ms]` | Duty hở → tốc độ bánh, kiểm tra chiều quay |
| `$straight cm [v]` rồi `$calib dist cm_that` | Chạy thẳng có PI rồi tự sửa `ticks_per_cm` (mỗi lần `$straight` chỉ calib được một lần) |
| `$spin do [w]` | Quay tại chỗ theo gyro → gợi ý `track_cm` |
| `$log 1` / `0` | Gửi pose mỗi 100 ms khi chạy: `L,ms,x_mm,y_mm,th_mrad,v_mm_s,state,unknown`; dòng `E,ms,STATE` khi đổi trạng thái |
| `$stop` | Dừng, kể cả khi đang chạy (lúc chạy chỉ nhận `$stop` và `$log`) |

## Trình tự hiệu chỉnh (kê bánh trước)

1. **Kiểm tra chiều:** `$stream 1`, lăn bánh tiến thì encoder phải tăng; xoay xe sang trái thì gyro phải dương. Sai thì sửa cờ trong `Bsp/board_config.h`.
2. **Feedforward:** `$motor 40 40`, rồi đặt `wheel_kff` ≈ 0,4 / tốc độ bánh đo được.
3. **Xung/cm:** `$straight 200`, đo quãng thật, gõ `$calib dist <cm>`.
4. **Khoảng cách bánh:** `$spin 360` vài lần, rồi `$set track_cm <giá trị gợi ý>`.
5. **PI:** chỉnh `wheel_kp`, `wheel_ki`, `wheel_kw` tới khi `$straight 200` lệch dưới 2 cm.
6. **Hướng lắp IR:** `$set ir_mode 0` (chúc xuống, mặc định), `1` (nhìn ngang), `2` (tắt). Xong gõ `$save`.
