#ifndef VL53L0X_H
#define VL53L0X_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * Driver VL53L0X dùng HAL I2C, port C từ thư viện Pololu VL53L0X
 * (trình tự DataInit -> StaticInit -> RefCalibration của ST API).
 * Nhiều cảm biến chung bus: giữ XSHUT các con khác ở mức thấp, bật từng con
 * rồi gọi VL53L0X_SetAddress trước VL53L0X_Init.
 */

#define VL53L0X_DEFAULT_ADDR 0x29U  /* địa chỉ 7 bit sau khi bật nguồn */
#define VL53L0X_OUT_OF_RANGE 8190U  /* số đo >= giá trị này: không thấy vật */
#define VL53L0X_STATUS_VALID 11U    /* range status khi số đo hợp lệ */

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t  addr;           /* 7 bit */
    uint8_t  stop_variable;
    uint32_t budget_us;
    uint16_t timeout_ms;
    bool     timed_out;      /* có thao tác chờ quá timeout_ms */
    bool     i2c_error;      /* có giao dịch I2C trả lỗi */
} VL53L0X;

void     VL53L0X_Setup(VL53L0X *dev, I2C_HandleTypeDef *hi2c);
bool     VL53L0X_SetAddress(VL53L0X *dev, uint8_t new_addr);
uint8_t  VL53L0X_ModelId(VL53L0X *dev);  /* đúng chip thì trả 0xEE */
bool     VL53L0X_Init(VL53L0X *dev);
bool     VL53L0X_SetTimingBudget(VL53L0X *dev, uint32_t budget_us);
void     VL53L0X_StartContinuous(VL53L0X *dev, uint32_t period_ms); /* 0 = back-to-back */
void     VL53L0X_StopContinuous(VL53L0X *dev);
bool     VL53L0X_DataReady(VL53L0X *dev);
uint16_t VL53L0X_ReadRange(VL53L0X *dev, uint8_t *status); /* gọi sau khi DataReady */

#endif
