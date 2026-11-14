/* Động cơ qua TB6612 (PWM vào chân IN) và encoder hai bánh. */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

void Motor_Init(void);              /* bật PWM + encoder, STBY vẫn thấp (động cơ tắt) */
void Motor_Enable(bool on);         /* STBY */
/* duty trong [-1, 1]: dương = tiến. 0 = thả trôi. */
void Motor_SetDuty(float left, float right);
void Motor_Stop(void);

/* Số xung thay đổi từ lần gọi trước, dương = bánh tiến. Gọi đều đặn (bộ đếm 16 bit). */
int16_t Encoder_LeftDelta(void);
int16_t Encoder_RightDelta(void);

#endif /* MOTOR_H */
