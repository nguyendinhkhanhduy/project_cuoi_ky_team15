/* Cấu hình bo mạch: gán chân, handle CubeMX và các cờ phụ thuộc lắp ráp thực tế.
 *
 * Gán chân giữ nguyên như bản robot_config.h trước (lấy từ projectnnhung.kicad_sch):
 *   TB6612   AIN1/AIN2 = PA0/PA1 (bánh trái), BIN1/BIN2 = PA2/PA3 (bánh phải), STBY = PB0
 *            PWMA/PWMB nối cứng mức cao, nên PWM được đưa thẳng vào chân IN:
 *            PA0..PA3 cấu hình là TIM2_CH1..CH4 (cùng chân, chỉ đổi sang chế độ PWM).
 *   Encoder  trái TIM3 (PB4/PB5), phải TIM4 (PB6/PB7), chế độ Encoder Interface
 *   VL53L0X  XSHUT1..4 = PA4..PA7 (trái -> phải), I2C1 SCL/SDA = PB8/PB9 (chung với MPU6050)
 *   IR       trái PB13, phải PB14
 *   HC-05    USART1 TX/RX = PA9/PA10, 9600 baud
 *   SW1      PA11, kéo lên, mức thấp khi nhấn
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "stm32f4xx_hal.h"

/* ---- Handle do CubeMX sinh trong main.c ---- */
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim2;   /* PWM động cơ */
extern TIM_HandleTypeDef htim3;   /* encoder trái */
extern TIM_HandleTypeDef htim4;   /* encoder phải */

#define BOARD_I2C          (&hi2c1)
#define BOARD_UART         (&huart1)
#define BOARD_TIM_PWM      (&htim2)
#define BOARD_TIM_ENC_L    (&htim3)
#define BOARD_TIM_ENC_R    (&htim4)

/* ---- TB6612 ---- */
#define MOTOR_L_IN1_CH     TIM_CHANNEL_1   /* PA0 / AIN1 */
#define MOTOR_L_IN2_CH     TIM_CHANNEL_2   /* PA1 / AIN2 */
#define MOTOR_R_IN1_CH     TIM_CHANNEL_3   /* PA2 / BIN1 */
#define MOTOR_R_IN2_CH     TIM_CHANNEL_4   /* PA3 / BIN2 */
#define TB_STBY_PORT       GPIOB           /* PB0 */
#define TB_STBY_PIN        GPIO_PIN_0

/* Đổi thành 1 nếu bánh quay ngược khi ra lệnh tiến (thử trên xe thật). */
#define MOTOR_L_INVERT     0
#define MOTOR_R_INVERT     0
/* Đổi thành 1 nếu encoder đếm âm khi bánh tiến. */
#define ENC_L_INVERT       0
#define ENC_R_INVERT       0

/* ---- VL53L0X ---- */
#define TOF_COUNT          4
#define TOF_XSHUT_PORT     GPIOA
#define TOF_XSHUT_PINS     {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7}  /* trái -> phải */
#define TOF_BASE_ADDR      0x30U           /* cảm biến i nhận địa chỉ 7 bit 0x30 + i */
#define TOF_BUDGET_US      33000UL         /* khớp chu kỳ 33 ms mô phỏng trong tools/sim */

/* ---- MPU6050 ---- */
#define MPU6050_ADDR7      0x68U           /* AD0 nối GND */
/* +1 nếu trục Z của chip hướng lên (quay trái = dương, đúng quy ước NavInput), -1 nếu lắp úp. */
#define MPU6050_Z_SIGN     (+1)

/* ---- IR ---- */
#define IR_LEFT_PORT       GPIOB           /* PB13 */
#define IR_LEFT_PIN        GPIO_PIN_13
#define IR_RIGHT_PORT      GPIOB           /* PB14 */
#define IR_RIGHT_PIN       GPIO_PIN_14
/* Mức chân khi IR đang nằm trên băng đen. Module dò line kiểu FC-51 chúc xuống sàn thường ra
 * mức CAO trên nền đen (không phản xạ). Phải kiểm tra trên xe thật; nếu IR nhìn ngang ra trước
 * thì đặt ir_gate_cm = 0 trong NavParams để tắt sửa vị trí bằng IR. */
#define IR_ON_TAPE_LEVEL   GPIO_PIN_SET

/* ---- SW1 ---- */
#define SW1_PORT           GPIOA           /* PA11 */
#define SW1_PIN            GPIO_PIN_11

#endif /* BOARD_CONFIG_H */
