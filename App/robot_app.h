/* Lớp ứng dụng: nối driver (Bsp/) với thuật toán dẫn đường (Algo/).
 *
 * Gọi từ main.c do CubeMX sinh:
 *   USER CODE BEGIN 2:     RobotApp_Init();   (xe phải đứng yên ~1 s để hiệu chỉnh gyro)
 *   USER CODE BEGIN 3:     RobotApp_Task();   (trong while(1), không chặn)
 *
 * Thao tác:
 *   - Lượt 1: gửi lệnh lái từ app "Bluetooth RC Controller" (F B L R G I H J S, 0..9, q).
 *     Lệnh di chuyển đầu tiên bắt đầu ghi quỹ đạo. Nhấn SW1 để kết thúc: quỹ đạo tự lưu Flash.
 *   - Lượt 2: đặt xe ở ô xuất phát, nhấn SW1 (thả ra trước 1 s) để chạy theo quỹ đạo A* trong path.h.
 *     Giữ SW1 từ 1 s trở lên rồi thả để chạy theo quỹ đạo đã ghi ở Lượt 1 (nếu có).
 *   - Đang chạy, nhấn SW1 bất kỳ lúc nào (hoặc gõ $stop) = dừng khẩn cấp.
 *   - Dòng bắt đầu bằng '$' từ terminal là lệnh cấu hình/hiệu chỉnh: $help để xem (README.md).
 */
#ifndef ROBOT_APP_H
#define ROBOT_APP_H

void RobotApp_Init(void);
void RobotApp_Task(void);

#endif /* ROBOT_APP_H */
