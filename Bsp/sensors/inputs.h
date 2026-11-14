/* Đầu vào số: 2 IR góc trước và nút SW1. */
#ifndef INPUTS_H
#define INPUTS_H

#include <stdbool.h>
#include <stdint.h>

uint8_t Ir_OnTapeMask(void);  /* bit0 = IR trái, bit1 = IR phải; 1 = đang trên băng đen */
bool Sw1_IsDown(void);        /* mức tức thời, chưa chống dội */

#endif /* INPUTS_H */
