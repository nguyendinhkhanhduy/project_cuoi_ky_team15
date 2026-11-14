#include "vl53l0x.h"

enum {
    SYSRANGE_START                              = 0x00,
    SYSTEM_SEQUENCE_CONFIG                      = 0x01,
    SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,
    SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,
    SYSTEM_INTERRUPT_CLEAR                      = 0x0B,
    RESULT_INTERRUPT_STATUS                     = 0x13,
    RESULT_RANGE_STATUS                         = 0x14,
    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,
    MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,
    DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
    DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
    PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
    MSRC_CONFIG_CONTROL                         = 0x60,
    FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
    GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,
    VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV            = 0x89,
    I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
    GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
    IDENTIFICATION_MODEL_ID                     = 0xC0,
    OSC_CALIBRATE_VAL                           = 0xF8,
};

#define I2C_TIMEOUT_MS 10U

/* DefaultTuningSettings trong vl53l0x_tuning.h của ST API */
static const uint8_t tuning[][2] = {
    {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00}, {0x10, 0x00},
    {0x11, 0x00}, {0x24, 0x01}, {0x25, 0xFF}, {0x75, 0x00}, {0xFF, 0x01},
    {0x4E, 0x2C}, {0x48, 0x00}, {0x30, 0x20}, {0xFF, 0x00}, {0x30, 0x09},
    {0x54, 0x00}, {0x31, 0x04}, {0x32, 0x03}, {0x40, 0x83}, {0x46, 0x25},
    {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00}, {0x52, 0x96},
    {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00}, {0x62, 0x00}, {0x64, 0x00},
    {0x65, 0x00}, {0x66, 0xA0}, {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14},
    {0x49, 0xFF}, {0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00},
    {0x78, 0x21}, {0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00}, {0x44, 0xFF},
    {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40}, {0x0E, 0x06}, {0x20, 0x1A},
    {0x43, 0x40}, {0xFF, 0x00}, {0x34, 0x03}, {0x35, 0x44}, {0xFF, 0x01},
    {0x31, 0x04}, {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04}, {0xFF, 0x00},
    {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28}, {0x67, 0x00},
    {0x70, 0x04}, {0x71, 0x01}, {0x72, 0xFE}, {0x76, 0x00}, {0x77, 0x00},
    {0xFF, 0x01}, {0x0D, 0x01}, {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8},
    {0xFF, 0x01}, {0x8E, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00},
};

typedef struct {
    bool tcc, msrc, dss, pre_range, final_range;
} SeqEnables;

typedef struct {
    uint16_t pre_range_vcsel_pclks, final_range_vcsel_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
} SeqTimeouts;

/* ---- I2C ---- */

static void write_multi(VL53L0X *d, uint8_t reg, uint8_t *buf, uint16_t n)
{
    if (HAL_I2C_Mem_Write(d->hi2c, (uint16_t)(d->addr << 1), reg, I2C_MEMADD_SIZE_8BIT,
                          buf, n, I2C_TIMEOUT_MS) != HAL_OK) {
        d->i2c_error = true;
    }
}

static void read_multi(VL53L0X *d, uint8_t reg, uint8_t *buf, uint16_t n)
{
    if (HAL_I2C_Mem_Read(d->hi2c, (uint16_t)(d->addr << 1), reg, I2C_MEMADD_SIZE_8BIT,
                         buf, n, I2C_TIMEOUT_MS) != HAL_OK) {
        d->i2c_error = true;
    }
}

static void wr8(VL53L0X *d, uint8_t reg, uint8_t v) { write_multi(d, reg, &v, 1); }

static void wr16(VL53L0X *d, uint8_t reg, uint16_t v)
{
    uint8_t b[2] = {(uint8_t)(v >> 8), (uint8_t)v};
    write_multi(d, reg, b, 2);
}

static void wr32(VL53L0X *d, uint8_t reg, uint32_t v)
{
    uint8_t b[4] = {(uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v};
    write_multi(d, reg, b, 4);
}

static uint8_t rd8(VL53L0X *d, uint8_t reg)
{
    uint8_t v = 0;
    read_multi(d, reg, &v, 1);
    return v;
}

static uint16_t rd16(VL53L0X *d, uint8_t reg)
{
    uint8_t b[2] = {0, 0};
    read_multi(d, reg, b, 2);
    return (uint16_t)((b[0] << 8) | b[1]);
}

/* Chờ (thanh ghi & mask) != 0 hoặc == 0 tuỳ want_set. */
static bool wait_reg(VL53L0X *d, uint8_t reg, uint8_t mask, bool want_set)
{
    uint32_t start = HAL_GetTick();
    while (((rd8(d, reg) & mask) != 0) != want_set) {
        if (d->i2c_error || (HAL_GetTick() - start) > d->timeout_ms) {
            d->timed_out = true;
            return false;
        }
    }
    return true;
}

/* ---- Timing budget ---- */

static uint16_t decode_vcsel(uint8_t reg) { return (uint16_t)((reg + 1U) << 1); }

static uint32_t macro_period_ns(uint16_t vcsel_pclks)
{
    return ((2304UL * vcsel_pclks * 1655UL) + 500UL) / 1000UL;
}

static uint32_t mclks_to_us(uint16_t mclks, uint16_t vcsel_pclks)
{
    uint32_t p = macro_period_ns(vcsel_pclks);
    return ((mclks * p) + 500UL) / 1000UL;
}

static uint32_t us_to_mclks(uint32_t us, uint16_t vcsel_pclks)
{
    uint32_t p = macro_period_ns(vcsel_pclks);
    return ((us * 1000UL) + (p / 2UL)) / p;
}

static uint16_t decode_timeout(uint16_t v)
{
    return (uint16_t)(((v & 0x00FFU) << ((v & 0xFF00U) >> 8)) + 1U);
}

static uint16_t encode_timeout(uint32_t mclks)
{
    if (mclks == 0) return 0;
    uint32_t ls = mclks - 1U;
    uint16_t ms = 0;
    while ((ls & 0xFFFFFF00UL) > 0) {
        ls >>= 1;
        ms++;
    }
    return (uint16_t)((ms << 8) | (ls & 0xFFU));
}

static void get_seq_enables(VL53L0X *d, SeqEnables *e)
{
    uint8_t c = rd8(d, SYSTEM_SEQUENCE_CONFIG);
    e->tcc         = (c >> 4) & 1U;
    e->dss         = (c >> 3) & 1U;
    e->msrc        = (c >> 2) & 1U;
    e->pre_range   = (c >> 6) & 1U;
    e->final_range = (c >> 7) & 1U;
}

static void get_seq_timeouts(VL53L0X *d, const SeqEnables *e, SeqTimeouts *t)
{
    t->pre_range_vcsel_pclks = decode_vcsel(rd8(d, PRE_RANGE_CONFIG_VCSEL_PERIOD));
    t->msrc_dss_tcc_mclks = (uint16_t)(rd8(d, MSRC_CONFIG_TIMEOUT_MACROP) + 1U);
    t->msrc_dss_tcc_us = mclks_to_us(t->msrc_dss_tcc_mclks, t->pre_range_vcsel_pclks);
    t->pre_range_mclks = decode_timeout(rd16(d, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    t->pre_range_us = mclks_to_us(t->pre_range_mclks, t->pre_range_vcsel_pclks);

    t->final_range_vcsel_pclks = decode_vcsel(rd8(d, FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    t->final_range_mclks = decode_timeout(rd16(d, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    if (e->pre_range) t->final_range_mclks -= t->pre_range_mclks;
    t->final_range_us = mclks_to_us(t->final_range_mclks, t->final_range_vcsel_pclks);
}

/* Overhead (us) theo ST API */
#define OVH_START  1910UL
#define OVH_END     960UL
#define OVH_MSRC    660UL
#define OVH_TCC     590UL
#define OVH_DSS     690UL
#define OVH_PRE     660UL
#define OVH_FINAL   550UL

static uint32_t used_budget_except_final(const SeqEnables *e, const SeqTimeouts *t)
{
    uint32_t b = OVH_START + OVH_END;
    if (e->tcc) b += t->msrc_dss_tcc_us + OVH_TCC;
    if (e->dss) b += 2UL * (t->msrc_dss_tcc_us + OVH_DSS);
    else if (e->msrc) b += t->msrc_dss_tcc_us + OVH_MSRC;
    if (e->pre_range) b += t->pre_range_us + OVH_PRE;
    return b;
}

static uint32_t get_budget(VL53L0X *d)
{
    SeqEnables e;
    SeqTimeouts t;
    get_seq_enables(d, &e);
    get_seq_timeouts(d, &e, &t);
    uint32_t b = used_budget_except_final(&e, &t);
    if (e.final_range) b += t.final_range_us + OVH_FINAL;
    return b;
}

bool VL53L0X_SetTimingBudget(VL53L0X *d, uint32_t budget_us)
{
    SeqEnables e;
    SeqTimeouts t;
    get_seq_enables(d, &e);
    get_seq_timeouts(d, &e, &t);
    uint32_t used = used_budget_except_final(&e, &t);

    if (e.final_range) {
        used += OVH_FINAL;
        if (used > budget_us) return false;
        uint32_t mclks = us_to_mclks(budget_us - used, t.final_range_vcsel_pclks);
        if (e.pre_range) mclks += t.pre_range_mclks;
        wr16(d, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encode_timeout(mclks));
        d->budget_us = budget_us;
    }
    return !d->i2c_error;
}

/* ---- Init ---- */

static bool get_spad_info(VL53L0X *d, uint8_t *count, bool *is_aperture)
{
    wr8(d, 0x80, 0x01);
    wr8(d, 0xFF, 0x01);
    wr8(d, 0x00, 0x00);
    wr8(d, 0xFF, 0x06);
    wr8(d, 0x83, rd8(d, 0x83) | 0x04);
    wr8(d, 0xFF, 0x07);
    wr8(d, 0x81, 0x01);
    wr8(d, 0x80, 0x01);
    wr8(d, 0x94, 0x6B);
    wr8(d, 0x83, 0x00);
    if (!wait_reg(d, 0x83, 0xFF, true)) return false;
    wr8(d, 0x83, 0x01);
    uint8_t tmp = rd8(d, 0x92);
    *count = tmp & 0x7F;
    *is_aperture = (tmp >> 7) & 0x01;
    wr8(d, 0x81, 0x00);
    wr8(d, 0xFF, 0x06);
    wr8(d, 0x83, rd8(d, 0x83) & (uint8_t)~0x04);
    wr8(d, 0xFF, 0x01);
    wr8(d, 0x00, 0x01);
    wr8(d, 0xFF, 0x00);
    wr8(d, 0x80, 0x00);
    return !d->i2c_error;
}

static bool single_ref_calibration(VL53L0X *d, uint8_t vhv_init_byte)
{
    wr8(d, SYSRANGE_START, (uint8_t)(0x01 | vhv_init_byte));
    if (!wait_reg(d, RESULT_INTERRUPT_STATUS, 0x07, true)) return false;
    wr8(d, SYSTEM_INTERRUPT_CLEAR, 0x01);
    wr8(d, SYSRANGE_START, 0x00);
    return !d->i2c_error;
}

void VL53L0X_Setup(VL53L0X *d, I2C_HandleTypeDef *hi2c)
{
    d->hi2c = hi2c;
    d->addr = VL53L0X_DEFAULT_ADDR;
    d->stop_variable = 0;
    d->budget_us = 0;
    d->timeout_ms = 500;
    d->timed_out = false;
    d->i2c_error = false;
}

bool VL53L0X_SetAddress(VL53L0X *d, uint8_t new_addr)
{
    wr8(d, I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
    if (d->i2c_error) return false;
    d->addr = new_addr & 0x7F;
    return true;
}

uint8_t VL53L0X_ModelId(VL53L0X *d) { return rd8(d, IDENTIFICATION_MODEL_ID); }

bool VL53L0X_Init(VL53L0X *d)
{
    d->i2c_error = false;
    d->timed_out = false;
    if (VL53L0X_ModelId(d) != 0xEE) return false;

    /* DataInit: I/O 2V8 (breakout có LDO 2,8 V) */
    wr8(d, VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, rd8(d, VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV) | 0x01);
    wr8(d, 0x88, 0x00);
    wr8(d, 0x80, 0x01);
    wr8(d, 0xFF, 0x01);
    wr8(d, 0x00, 0x00);
    d->stop_variable = rd8(d, 0x91);
    wr8(d, 0x00, 0x01);
    wr8(d, 0xFF, 0x00);
    wr8(d, 0x80, 0x00);

    /* Tắt limit check SIGNAL_RATE_MSRC và SIGNAL_RATE_PRE_RANGE */
    wr8(d, MSRC_CONFIG_CONTROL, rd8(d, MSRC_CONFIG_CONTROL) | 0x12);
    /* Final range signal rate limit 0,25 MCPS (định dạng 9.7) */
    wr16(d, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (uint16_t)(0.25f * (1 << 7)));
    wr8(d, SYSTEM_SEQUENCE_CONFIG, 0xFF);

    /* StaticInit: reference SPAD */
    uint8_t spad_count;
    bool is_aperture;
    if (!get_spad_info(d, &spad_count, &is_aperture)) return false;

    uint8_t map[6];
    read_multi(d, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, map, 6);
    wr8(d, 0xFF, 0x01);
    wr8(d, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    wr8(d, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    wr8(d, 0xFF, 0x00);
    wr8(d, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first = is_aperture ? 12U : 0U;
    uint8_t enabled = 0;
    for (uint8_t i = 0; i < 48U; i++) {
        if (i < first || enabled == spad_count) {
            map[i / 8U] &= (uint8_t)~(1U << (i % 8U));
        } else if ((map[i / 8U] >> (i % 8U)) & 0x1U) {
            enabled++;
        }
    }
    write_multi(d, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, map, 6);

    for (uint16_t i = 0; i < sizeof(tuning) / sizeof(tuning[0]); i++) {
        wr8(d, tuning[i][0], tuning[i][1]);
    }

    /* Ngắt GPIO báo "có mẫu mới", active low */
    wr8(d, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    wr8(d, GPIO_HV_MUX_ACTIVE_HIGH, rd8(d, GPIO_HV_MUX_ACTIVE_HIGH) & (uint8_t)~0x10);
    wr8(d, SYSTEM_INTERRUPT_CLEAR, 0x01);

    /* Tắt MSRC và TCC rồi tính lại timing budget */
    d->budget_us = get_budget(d);
    wr8(d, SYSTEM_SEQUENCE_CONFIG, 0xE8);
    if (!VL53L0X_SetTimingBudget(d, d->budget_us)) return false;

    /* RefCalibration: VHV rồi phase */
    wr8(d, SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!single_ref_calibration(d, 0x40)) return false;
    wr8(d, SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!single_ref_calibration(d, 0x00)) return false;
    wr8(d, SYSTEM_SEQUENCE_CONFIG, 0xE8);

    return !d->i2c_error;
}

/* ---- Đo ---- */

void VL53L0X_StartContinuous(VL53L0X *d, uint32_t period_ms)
{
    wr8(d, 0x80, 0x01);
    wr8(d, 0xFF, 0x01);
    wr8(d, 0x00, 0x00);
    wr8(d, 0x91, d->stop_variable);
    wr8(d, 0x00, 0x01);
    wr8(d, 0xFF, 0x00);
    wr8(d, 0x80, 0x00);

    if (period_ms != 0) {
        uint16_t osc = rd16(d, OSC_CALIBRATE_VAL);
        if (osc != 0) period_ms *= osc;
        wr32(d, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
        wr8(d, SYSRANGE_START, 0x04);  /* timed */
    } else {
        wr8(d, SYSRANGE_START, 0x02);  /* back-to-back */
    }
}

void VL53L0X_StopContinuous(VL53L0X *d)
{
    wr8(d, SYSRANGE_START, 0x01);  /* single-shot: dừng chế độ liên tục */
    wr8(d, 0xFF, 0x01);
    wr8(d, 0x00, 0x00);
    wr8(d, 0x91, 0x00);
    wr8(d, 0x00, 0x01);
    wr8(d, 0xFF, 0x00);
}

bool VL53L0X_DataReady(VL53L0X *d)
{
    return (rd8(d, RESULT_INTERRUPT_STATUS) & 0x07) != 0;
}

uint16_t VL53L0X_ReadRange(VL53L0X *d, uint8_t *status)
{
    uint8_t b[12] = {0};
    read_multi(d, RESULT_RANGE_STATUS, b, sizeof(b));
    wr8(d, SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (status) *status = (uint8_t)((b[0] & 0x78) >> 3);
    return (uint16_t)((b[10] << 8) | b[11]);
}
