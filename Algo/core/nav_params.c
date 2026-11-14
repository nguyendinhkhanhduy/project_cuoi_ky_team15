#include "nav_params.h"

#include <stddef.h>
#include <string.h>

void NavParams_Default(NavParams *p)
{
    memset(p, 0, sizeof(*p));
    p->version = NAV_PARAMS_VERSION;

    /* Giá trị tạm: N20 1:30, encoder 7 xung x4 cạnh, bánh 34 mm -> ~78.6 xung/cm.
     * Phải hiệu chỉnh trên xe thật. */
    p->ticks_per_cm = 78.6f;
    p->track_cm = 8.0f;
    p->gyro_weight = 0.95f;

    p->body_len_cm = 10.0f;
    p->body_wid_cm = 9.0f;

    p->v_max = 30.0f;
    p->v_min = 6.0f;
    p->a_max = 50.0f;
    p->a_lat_max = 60.0f;
    p->w_max = 5.0f;
    p->la_min = 4.0f;
    p->la_max = 14.0f;
    p->la_gain_s = 0.45f;
    p->turn_window_cm = 15.0f;
    p->turn_full_rad = 1.0f;
    p->turn_slow = 0.7f;
    p->spin_angle_rad = 0.9f;
    p->finish_y_cm = -4.0f;  /* biên 4 cm bù sai số vị trí ước lượng */

    /* 4 VL53L0X xếp quạt trên mũi xe, trái -> phải */
    const float tx[NAV_TOF_COUNT] = {4.0f, 4.8f, 4.8f, 4.0f};
    const float ty[NAV_TOF_COUNT] = {2.5f, 0.9f, -0.9f, -2.5f};
    const float ta[NAV_TOF_COUNT] = {40.0f, 13.0f, -13.0f, -40.0f};
    for (int i = 0; i < NAV_TOF_COUNT; i++) {
        p->tof_x[i] = tx[i];
        p->tof_y[i] = ty[i];
        p->tof_ang_deg[i] = ta[i];
    }
    p->tof_fov_deg = 25.0f;
    p->tof_max_cm = 120.0f;
    /* 2 IR ở góc trước */
    p->ir_x[0] = 4.5f;  p->ir_y[0] = 4.5f;
    p->ir_x[1] = 4.5f;  p->ir_y[1] = -4.5f;

    p->init_sigma_cm = 0.5f;
    p->init_sigma_rad = 0.03f;
    p->odo_sigma_frac = 0.03f;
    p->turn_sigma_frac = 0.03f;
    p->gyro_drift_sigma = 0.002f;
    p->tof_sigma_cm = 0.8f;
    p->loc_gate_cm = 3.0f;
    p->loc_range_cm = 60.0f;
    p->ir_gate_cm = 3.0f;
    p->ir_mode = NAV_IR_TAPE;
    p->edge_back_cm_s = 4.0f;
    p->edge_w = 2.0f;
    p->edge_hold_s = 0.15f;
    p->border_visible = 0U;

    p->avoid_range_cm = 40.0f;
    p->avoid_margin_cm = 8.0f;
    p->avoid_known_tol_cm = 4.0f;
    p->avoid_confirm = 2U;
    p->r_hard_cm = 7.5f;
    p->r_soft_cm = 15.0f;
    p->r_tight_cm = 5.5f;  /* nửa bề rộng 4,5 cm + 1 cm */
    p->avoid_rejoin_cm = 20.0f;

    p->teach_v_max = 25.0f;
    p->teach_w = 2.5f;
    p->teach_timeout_s = 0.5f;
    p->rec_step_cm = 2.0f;
    p->rec_epsilon_cm = 0.7f;
}

typedef struct {
    const char *name;
    size_t offset;
} ParamField;

#define F(field) {#field, offsetof(NavParams, field)}
static const ParamField fields[] = {
    F(ticks_per_cm), F(track_cm), F(gyro_weight), F(body_len_cm), F(body_wid_cm),
    F(v_max), F(v_min), F(a_max), F(a_lat_max), F(w_max),
    F(la_min), F(la_max), F(la_gain_s), F(turn_window_cm), F(turn_full_rad), F(turn_slow),
    F(spin_angle_rad), F(finish_y_cm),
    {"tof_x0", offsetof(NavParams, tof_x)}, {"tof_x1", offsetof(NavParams, tof_x) + 4},
    {"tof_x2", offsetof(NavParams, tof_x) + 8}, {"tof_x3", offsetof(NavParams, tof_x) + 12},
    {"tof_y0", offsetof(NavParams, tof_y)}, {"tof_y1", offsetof(NavParams, tof_y) + 4},
    {"tof_y2", offsetof(NavParams, tof_y) + 8}, {"tof_y3", offsetof(NavParams, tof_y) + 12},
    {"tof_ang_deg0", offsetof(NavParams, tof_ang_deg)}, {"tof_ang_deg1", offsetof(NavParams, tof_ang_deg) + 4},
    {"tof_ang_deg2", offsetof(NavParams, tof_ang_deg) + 8}, {"tof_ang_deg3", offsetof(NavParams, tof_ang_deg) + 12},
    F(tof_fov_deg), F(tof_max_cm),
    {"ir_x0", offsetof(NavParams, ir_x)}, {"ir_x1", offsetof(NavParams, ir_x) + 4},
    {"ir_y0", offsetof(NavParams, ir_y)}, {"ir_y1", offsetof(NavParams, ir_y) + 4},
    F(init_sigma_cm), F(init_sigma_rad), F(odo_sigma_frac), F(turn_sigma_frac), F(gyro_drift_sigma),
    F(tof_sigma_cm), F(loc_gate_cm), F(loc_range_cm), F(ir_gate_cm),
    F(avoid_range_cm), F(avoid_margin_cm), F(avoid_known_tol_cm), F(r_hard_cm), F(r_soft_cm), F(r_tight_cm), F(edge_back_cm_s), F(edge_w), F(edge_hold_s),
    F(avoid_rejoin_cm),
    F(teach_v_max), F(teach_w), F(teach_timeout_s), F(rec_step_cm), F(rec_epsilon_cm),
};
#undef F

int NavParams_Set(NavParams *p, const char *name, float value)
{
    if (strcmp(name, "ir_mode") == 0) {
        if (value < 0.0f || value > (float)NAV_IR_OFF) return -1;
        p->ir_mode = (uint8_t)value;
        return 0;
    }
    if (strcmp(name, "border_visible") == 0) {
        p->border_visible = (value != 0.0f) ? 1U : 0U;
        return 0;
    }
    if (strcmp(name, "avoid_confirm") == 0) {
        p->avoid_confirm = (uint8_t)((value < 1.0f) ? 1.0f : value);
        return 0;
    }
    for (size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
        if (strcmp(name, fields[i].name) == 0) {
            float *dst = (float *)((uint8_t *)p + fields[i].offset);
            *dst = value;
            return 0;
        }
    }
    return -1;
}

/* Các thông số kiểu uint8_t, xử lý riêng trong NavParams_Set/Get. */
static const char *const byte_params[] = {"ir_mode", "border_visible", "avoid_confirm"};
#define N_FIELDS (sizeof(fields) / sizeof(fields[0]))
#define N_BYTES  (sizeof(byte_params) / sizeof(byte_params[0]))

int NavParams_Count(void)
{
    return (int)(N_FIELDS + N_BYTES);
}

const char *NavParams_NameAt(int i)
{
    if (i < 0) return 0;
    if ((size_t)i < N_FIELDS) return fields[i].name;
    if ((size_t)i < N_FIELDS + N_BYTES) return byte_params[(size_t)i - N_FIELDS];
    return 0;
}

int NavParams_Get(const NavParams *p, const char *name, float *value)
{
    if (strcmp(name, "ir_mode") == 0) { *value = (float)p->ir_mode; return 0; }
    if (strcmp(name, "border_visible") == 0) { *value = (float)p->border_visible; return 0; }
    if (strcmp(name, "avoid_confirm") == 0) { *value = (float)p->avoid_confirm; return 0; }
    for (size_t i = 0; i < N_FIELDS; i++) {
        if (strcmp(name, fields[i].name) == 0) {
            *value = *(const float *)((const uint8_t *)p + fields[i].offset);
            return 0;
        }
    }
    return -1;
}
