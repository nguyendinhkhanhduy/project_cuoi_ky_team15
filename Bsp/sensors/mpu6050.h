/* MPU6050: chỉ dùng gyro trục Z (vận tốc góc quay của xe). */
#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>

bool MPU6050_Init(void);                       /* kiểm tra WHO_AM_I, ±500 °/s, lọc 44 Hz, 200 Hz */
bool MPU6050_CalibrateGyro(uint16_t samples);  /* gọi khi xe đứng yên */
bool MPU6050_ReadYawRate(float *rad_s);        /* đã trừ bias, dương = quay trái */

#endif /* MPU6050_H */
