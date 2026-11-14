#include "tof.h"

#include "board_config.h"
#include "vl53l0x.h"

static VL53L0X dev[TOF_COUNT];
static uint8_t ok_mask;
static const uint16_t xshut_pin[TOF_COUNT] = TOF_XSHUT_PINS;

static int init_one(int i)
{
    VL53L0X *d = &dev[i];
    VL53L0X_Setup(d, BOARD_I2C);
    HAL_GPIO_WritePin(TOF_XSHUT_PORT, xshut_pin[i], GPIO_PIN_SET);
    HAL_Delay(5);  /* boot ~1,2 ms */

    if (HAL_I2C_IsDeviceReady(BOARD_I2C, VL53L0X_DEFAULT_ADDR << 1, 3, 10) != HAL_OK ||
        !VL53L0X_SetAddress(d, (uint8_t)(TOF_BASE_ADDR + i))) {
        /* Chip vẫn ở 0x29: tắt đi để không trùng địa chỉ với cảm biến sau. */
        HAL_GPIO_WritePin(TOF_XSHUT_PORT, xshut_pin[i], GPIO_PIN_RESET);
        return 0;
    }
    /* Đã ở địa chỉ riêng, nên lỗi từ đây không chặn cảm biến khác. */
    if (!VL53L0X_Init(d) || !VL53L0X_SetTimingBudget(d, TOF_BUDGET_US)) return 0;
    VL53L0X_StartContinuous(d, 0);
    return 1;
}

uint8_t ToF_InitAll(void)
{
    for (int i = 0; i < TOF_COUNT; i++) HAL_GPIO_WritePin(TOF_XSHUT_PORT, xshut_pin[i], GPIO_PIN_RESET);
    HAL_Delay(10);
    ok_mask = 0;
    for (int i = 0; i < TOF_COUNT; i++) {
        if (init_one(i)) ok_mask |= (uint8_t)(1U << i);
    }
    return ok_mask;
}

uint8_t ToF_Poll(uint16_t mm[])
{
    uint8_t fresh = 0;
    for (int i = 0; i < TOF_COUNT; i++) {
        if (!(ok_mask & (1U << i))) continue;
        int ready = VL53L0X_DataReady(&dev[i]);
        if (dev[i].i2c_error) {
            /* Rớt dây / treo bus: mỗi lần hỏi lại tốn tới 10 ms timeout, phá chu kỳ 5 ms. */
            ok_mask &= (uint8_t)~(1U << i);
            mm[i] = TOF_NO_READING;
            fresh |= (uint8_t)(1U << i);
            continue;
        }
        if (!ready) continue;
        uint8_t status;
        uint16_t d = VL53L0X_ReadRange(&dev[i], &status);
        mm[i] = (status == VL53L0X_STATUS_VALID && d < VL53L0X_OUT_OF_RANGE) ? d : TOF_NO_READING;
        fresh |= (uint8_t)(1U << i);
    }
    return fresh;
}
