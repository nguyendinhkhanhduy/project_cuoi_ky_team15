#include "wheel_ctrl.h"

void Wheel_DefaultGains(WheelGains *g)
{
    /* Giá trị tạm, phải chỉnh trên xe thật (mục tiêu slide: chạy thẳng 2 m lệch < 2 cm). */
    g->kff = 1.0f / 60.0f;   /* giả định duty 1 ứng với ~60 cm/s ở bánh */
    g->kp = 0.01f;
    g->ki = 0.05f;
    g->i_max = 0.3f;
    g->kw = 0.5f;
}

void Wheel_Reset(WheelCtrl *c)
{
    c->int_l = 0.0f;
    c->int_r = 0.0f;
}

static float clamp(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static float pi_one(float *integ, const WheelGains *g, float ref, float meas, float dt)
{
    float e = ref - meas;
    float u_noint = g->kff * ref + g->kp * e;
    float u = u_noint + *integ;
    /* Chống bão hoà tích phân: chỉ tích luỹ khi đầu ra chưa chạm biên, hoặc sai số kéo về. */
    if ((u < 1.0f && u > -1.0f) || (u >= 1.0f && e < 0.0f) || (u <= -1.0f && e > 0.0f)) {
        *integ = clamp(*integ + g->ki * e * dt, -g->i_max, g->i_max);
    }
    return clamp(u_noint + *integ, -1.0f, 1.0f);
}

void Wheel_Step(WheelCtrl *c, const WheelGains *g, float v, float w, float track_cm, float meas_l,
                float meas_r, float gyro, float dt, float *duty_l, float *duty_r)
{
    float w_cmd = w + g->kw * (w - gyro);
    float ref_l = v - 0.5f * w_cmd * track_cm;
    float ref_r = v + 0.5f * w_cmd * track_cm;
    *duty_l = pi_one(&c->int_l, g, ref_l, meas_l, dt);
    *duty_r = pi_one(&c->int_r, g, ref_r, meas_r, dt);
}
