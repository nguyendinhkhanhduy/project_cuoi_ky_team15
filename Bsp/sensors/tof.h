/* Quản lý 4 VL53L0X chung I2C1: đổi địa chỉ qua XSHUT lúc khởi động, đo liên tục. */
#ifndef TOF_H
#define TOF_H

#include <stdint.h>

#define TOF_NO_READING 8190U  /* ngoài tầm / số đo không hợp lệ (NavInput coi >= 8000 là ngoài tầm) */

/* Trả về bitmask các cảm biến khởi động được (bit i = cảm biến i, trái -> phải). */
uint8_t ToF_InitAll(void);

/* Đọc các cảm biến có mẫu mới. mm[i] chỉ được cập nhật khi bit i của giá trị trả về = 1. */
uint8_t ToF_Poll(uint16_t mm[]);

#endif /* TOF_H */
