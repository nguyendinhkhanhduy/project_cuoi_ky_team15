#include "motor.h"

#include "board_config.h"

static uint16_t last_l, last_r;

void Motor_Init(void)
{
    HAL_GPIO_WritePin(TB_STBY_PORT, TB_STBY_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(BOARD_TIM_PWM, MOTOR_L_IN1_CH, 0);
    __HAL_TIM_SET_COMPARE(BOARD_TIM_PWM, MOTOR_L_IN2_CH, 0);
    __HAL_TIM_SET_COMPARE(BOARD_TIM_PWM, MOTOR_R_IN1_CH, 0);
    __HAL_TIM_SET_COMPARE(BOARD_TIM_PWM, MOTOR_R_IN2_CH, 0);
    HAL_TIM_PWM_Start(BOARD_TIM_PWM, MOTOR_L_IN1_CH);
    HAL_TIM_PWM_Start(BOARD_TIM_PWM, MOTOR_L_IN2_CH);
    HAL_TIM_PWM_Start(BOARD_TIM_PWM, MOTOR_R_IN1_CH);
    HAL_TIM_PWM_Start(BOARD_TIM_PWM, MOTOR_R_IN2_CH);

    HAL_TIM_Encoder_Start(BOARD_TIM_ENC_L, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(BOARD_TIM_ENC_R, TIM_CHANNEL_ALL);
    last_l = (uint16_t)__HAL_TIM_GET_COUNTER(BOARD_TIM_ENC_L);
    last_r = (uint16_t)__HAL_TIM_GET_COUNTER(BOARD_TIM_ENC_R);
}

void Motor_Enable(bool on)
{
    HAL_GPIO_WritePin(TB_STBY_PORT, TB_STBY_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* PWMx = cao: IN1 = PWM, IN2 = 0 -> xen kẽ "quay thuận" và "dừng" theo duty; đảo vai trò khi lùi. */
static void drive_one(float duty, uint32_t ch_fwd, uint32_t ch_rev)
{
    if (duty > 1.0f) duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(BOARD_TIM_PWM);
    uint32_t cmp = (uint32_t)((duty >= 0.0f ? duty : -duty) * (float)(arr + 1U));
    if (cmp > arr + 1U) cmp = arr + 1U;
    __HAL_TIM_SET_COMPARE(BOARD_TIM_PWM, ch_fwd, (duty >= 0.0f) ? cmp : 0U);
    __HAL_TIM_SET_COMPARE(BOARD_TIM_PWM, ch_rev, (duty >= 0.0f) ? 0U : cmp);
}

void Motor_SetDuty(float left, float right)
{
#if MOTOR_L_INVERT
    left = -left;
#endif
#if MOTOR_R_INVERT
    right = -right;
#endif
    drive_one(left, MOTOR_L_IN1_CH, MOTOR_L_IN2_CH);
    drive_one(right, MOTOR_R_IN1_CH, MOTOR_R_IN2_CH);
}

void Motor_Stop(void)
{
    Motor_SetDuty(0.0f, 0.0f);
}

/* Trừ theo uint16_t rồi ép int16_t: đúng cả khi bộ đếm 16 bit quay vòng. */
int16_t Encoder_LeftDelta(void)
{
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(BOARD_TIM_ENC_L);
    int16_t d = (int16_t)(uint16_t)(now - last_l);
    last_l = now;
#if ENC_L_INVERT
    d = (int16_t)-d;
#endif
    return d;
}

int16_t Encoder_RightDelta(void)
{
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(BOARD_TIM_ENC_R);
    int16_t d = (int16_t)(uint16_t)(now - last_r);
    last_r = now;
#if ENC_R_INVERT
    d = (int16_t)-d;
#endif
    return d;
}
