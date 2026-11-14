/* Biến lệnh (v, w) của thuật toán thành duty PWM hai bánh (slide 16 "PID kép"):
 *   - PI vận tốc từng bánh theo encoder, có feedforward.
 *   - Bù vận tốc góc theo gyro: bám ω mà Nav_Step yêu cầu (KHÔNG giữ một hướng cố định,
 *     nếu không sẽ chống lại lệnh rẽ của Pure Pursuit).
 * Không gọi HAL. */
#ifndef WHEEL_CTRL_H
#define WHEEL_CTRL_H

typedef struct {
    float kff;     /* duty trên 1 cm/s (≈ 1 / vận tốc bánh lớn nhất khi duty = 1) */
    float kp;      /* duty trên 1 cm/s sai số */
    float ki;      /* duty trên 1 cm sai số tích luỹ */
    float i_max;   /* giới hạn phần tích phân (duty) */
    float kw;      /* hệ số bù sai số vận tốc góc: thêm kw * (w_ref - w_gyro) vào w */
} WheelGains;

typedef struct {
    float int_l, int_r;  /* phần tích phân, đơn vị duty */
} WheelCtrl;

void Wheel_DefaultGains(WheelGains *g);
void Wheel_Reset(WheelCtrl *c);

/* v (cm/s), w (rad/s): lệnh từ Nav_Step. meas_*: vận tốc bánh đo bằng encoder (cm/s).
 * gyro: vận tốc góc đo (rad/s). Ghi duty [-1, 1] vào *duty_l, *duty_r. */
void Wheel_Step(WheelCtrl *c, const WheelGains *g, float v, float w, float track_cm, float meas_l,
                float meas_r, float gyro, float dt, float *duty_l, float *duty_r);

#endif /* WHEEL_CTRL_H */
