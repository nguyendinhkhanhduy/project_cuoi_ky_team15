/* Các bài test trên bàn/sân để hiệu chỉnh, gọi qua lệnh Bluetooth ($motor, $straight, $spin):
 *   motor    : duty hở từng bánh trong một khoảng thời gian -> tốc độ bánh đạt được (chỉnh kff, chiều quay)
 *   straight : chạy thẳng có PI bánh + bù gyro -> quãng đường encoder, lệch hướng (chỉnh ticks_per_cm, PI)
 *   spin     : quay tại chỗ tới góc gyro yêu cầu -> góc theo encoder (chỉnh track_cm)
 * Kết quả in qua UART khi bài test kết thúc. */
#ifndef BENCH_H
#define BENCH_H

#include <stdbool.h>
#include <stdint.h>

#include "nav_params.h"
#include "wheel_ctrl.h"

void Bench_StartMotor(float duty_l, float duty_r, uint32_t ms);
void Bench_StartStraight(float dist_cm, float v_cm_s);
void Bench_StartSpin(float deg, float w_rad_s);
void Bench_Abort(void);
bool Bench_Active(void);

/* Gọi mỗi chu kỳ điều khiển khi Bench_Active(). Ghi duty hai bánh; trả về false khi vừa kết thúc. */
bool Bench_Step(float dt, int16_t d_ticks_l, int16_t d_ticks_r, float gyro, bool gyro_ok, const NavParams *p,
                const WheelGains *g, WheelCtrl *wc, float *duty_l, float *duty_r);

/* Lấy rồi xoá quãng đường encoder của bài straight gần nhất (cm), 0 nếu không có. Dùng cho
 * "$calib dist <cm thật>": số đo theo ticks_per_cm cũ, nên không được áp dụng hai lần. */
float Bench_TakeLastStraightCm(void);

#endif /* BENCH_H */
