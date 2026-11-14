#include "mpu6050.h"

#include "board_config.h"

#define REG_SMPLRT_DIV   0x19U
#define REG_CONFIG       0x1AU
#define REG_GYRO_CONFIG  0x1BU
#define REG_GYRO_ZOUT_H  0x47U
#define REG_PWR_MGMT_1   0x6BU
#define REG_WHO_AM_I     0x75U

#define I2C_TIMEOUT_MS   10U
#define LSB_PER_DPS      65.5f         /* thang ±500 °/s: w_max 5 rad/s = 286 °/s vượt ±250 */
#define DEG2RAD          0.017453293f

static float gyro_z_bias;              /* LSB */

static bool wr(uint8_t reg, uint8_t v)
{
    return HAL_I2C_Mem_Write(BOARD_I2C, MPU6050_ADDR7 << 1, reg, I2C_MEMADD_SIZE_8BIT, &v, 1,
                             I2C_TIMEOUT_MS) == HAL_OK;
}

static bool read_raw_z(int16_t *raw)
{
    uint8_t b[2];
    if (HAL_I2C_Mem_Read(BOARD_I2C, MPU6050_ADDR7 << 1, REG_GYRO_ZOUT_H, I2C_MEMADD_SIZE_8BIT, b, 2,
                         I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    *raw = (int16_t)((uint16_t)(b[0] << 8) | b[1]);
    return true;
}

bool MPU6050_Init(void)
{
    uint8_t who = 0;
    if (HAL_I2C_Mem_Read(BOARD_I2C, MPU6050_ADDR7 << 1, REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &who, 1,
                         I2C_TIMEOUT_MS) != HAL_OK || who != 0x68U) {
        return false;
    }
    if (!wr(REG_PWR_MGMT_1, 0x01U)) return false;  /* thức dậy, clock theo PLL gyro X */
    HAL_Delay(100);
    if (!wr(REG_CONFIG, 0x03U)) return false;      /* DLPF 44 Hz -> gyro output 1 kHz */
    if (!wr(REG_SMPLRT_DIV, 4U)) return false;     /* 1 kHz / (1 + 4) = 200 Hz, khớp vòng điều khiển */
    if (!wr(REG_GYRO_CONFIG, 0x08U)) return false; /* ±500 °/s */
    gyro_z_bias = 0.0f;
    return true;
}

bool MPU6050_CalibrateGyro(uint16_t samples)
{
    if (samples < 10U) return false;
    int32_t sum = 0;
    for (uint16_t i = 0; i < samples; i++) {
        int16_t raw;
        if (!read_raw_z(&raw)) return false;
        sum += raw;
        HAL_Delay(5);
    }
    gyro_z_bias = (float)sum / (float)samples;
    return true;
}

bool MPU6050_ReadYawRate(float *rad_s)
{
    int16_t raw;
    if (!rad_s || !read_raw_z(&raw)) return false;
    *rad_s = (float)MPU6050_Z_SIGN * ((float)raw - gyro_z_bias) / LSB_PER_DPS * DEG2RAD;
    return true;
}
