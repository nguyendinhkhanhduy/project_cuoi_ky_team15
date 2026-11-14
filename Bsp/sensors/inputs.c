#include "inputs.h"

#include "board_config.h"

uint8_t Ir_OnTapeMask(void)
{
    uint8_t m = 0;
    if (HAL_GPIO_ReadPin(IR_LEFT_PORT, IR_LEFT_PIN) == IR_ON_TAPE_LEVEL) m |= 1U;
    if (HAL_GPIO_ReadPin(IR_RIGHT_PORT, IR_RIGHT_PIN) == IR_ON_TAPE_LEVEL) m |= 2U;
    return m;
}

bool Sw1_IsDown(void)
{
    return HAL_GPIO_ReadPin(SW1_PORT, SW1_PIN) == GPIO_PIN_RESET;
}
