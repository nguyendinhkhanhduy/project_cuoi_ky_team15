#include "bench.h"

#include <math.h>

#include "uart_io.h"

#define RAD2DEG     57.29578f
#define TIMEOUT_S   15.0f   /* mọi bài test tự dừng sau chừng này giây */
#define RAMP_CM_S2  40.0f   /* tăng/giảm tốc của bài straight */

typedef enum { B_NONE, B_MOTOR, B_STRAIGHT, B_SPIN } Kind;

static Kind kind = B_NONE;
static float a_l, a_r;          /* motor: duty; straight: quãng đường, v; spin: góc (rad), w */
static float t, t_end;
static int32_t ticks_l, ticks_r;
static float gyro_angle;        /* tích phân gyro, rad */
static float v_now;
static float last_straight_cm;

static void reset_common(void)
{
    t = 0.0f;
    ticks_l = ticks_r = 0;
    gyro_angle = 0.0f;
    v_now = 0.0f;
}

void Bench_StartMotor(float duty_l, float duty_r, uint32_t ms)
{
    reset_common();
    a_l = duty_l;
    a_r = duty_r;
    t_end = fminf((float)ms * 0.001f, TIMEOUT_S);
    kind = B_MOTOR;
}

void Bench_StartStraight(float dist_cm, float v_cm_s)
{
    reset_common();
    a_l = dist_cm;
    a_r = fabsf(v_cm_s);
    t_end = TIMEOUT_S;
    kind = B_STRAIGHT;
}

void Bench_StartSpin(float deg, float w_rad_s)
{
    reset_common();
    a_l = deg / RAD2DEG;
    a_r = fabsf(w_rad_s);
    t_end = TIMEOUT_S;
    kind = B_SPIN;
}

void Bench_Abort(void)
{
    if (kind != B_NONE) Uart_Write("Test: huy\r\n");
    kind = B_NONE;
}

bool Bench_Active(void) { return kind != B_NONE; }
float Bench_TakeLastStraightCm(void)
{
    float d = last_straight_cm;
    last_straight_cm = 0.0f;
    return d;
}

static void report(const NavParams *p, bool gyro_ok)
{
    char f1[16], f2[16], f3[16], f4[16];
    float cm_l = (float)ticks_l / p->ticks_per_cm, cm_r = (float)ticks_r / p->ticks_per_cm;
    switch (kind) {
    case B_MOTOR:
        Uart_Printf("Motor: xung L %ld R %ld | toc do L %s R %s cm/s (sau %s s)\r\n", (long)ticks_l, (long)ticks_r,
                    Uart_FmtFloat(f1, cm_l / t), Uart_FmtFloat(f2, cm_r / t), Uart_FmtFloat(f3, t));
        Uart_Write("  Xung am khi duty duong -> doi ENC_*_INVERT / MOTOR_*_INVERT trong board_config.h\r\n");
        break;
    case B_STRAIGHT: {
        float mean = 0.5f * (cm_l + cm_r);
        last_straight_cm = mean;
        Uart_Printf("Thang: encoder L %s R %s TB %s cm | lech huong gyro %s do\r\n", Uart_FmtFloat(f1, cm_l),
                    Uart_FmtFloat(f2, cm_r), Uart_FmtFloat(f3, mean),
                    gyro_ok ? Uart_FmtFloat(f4, gyro_angle * RAD2DEG) : "(khong co gyro)");
        Uart_Write("  Do quang duong that roi go: $calib dist <cm>\r\n");
        break;
    }
    case B_SPIN: {
        float enc = (cm_r - cm_l) / p->track_cm;
        Uart_Printf("Quay: gyro %s do, encoder %s do\r\n", Uart_FmtFloat(f1, gyro_angle * RAD2DEG),
                    Uart_FmtFloat(f2, enc * RAD2DEG));
        if (gyro_ok && fabsf(gyro_angle) > 0.5f) {
            /* Tin gyro về góc: khoảng cách bánh thật = chênh lệch quãng đường / góc thật. */
            Uart_Printf("  Goi y: $set track_cm %s\r\n", Uart_FmtFloat(f3, (cm_r - cm_l) / gyro_angle));
        }
        break;
    }
    default:
        break;
    }
}

bool Bench_Step(float dt, int16_t d_ticks_l, int16_t d_ticks_r, float gyro, bool gyro_ok, const NavParams *p,
                const WheelGains *g, WheelCtrl *wc, float *duty_l, float *duty_r)
{
    *duty_l = *duty_r = 0.0f;
    if (kind == B_NONE) return false;
    t += dt;
    ticks_l += d_ticks_l;
    ticks_r += d_ticks_r;
    float dl = (float)d_ticks_l / p->ticks_per_cm, dr = (float)d_ticks_r / p->ticks_per_cm;
    float w_meas = gyro_ok ? gyro : (dr - dl) / p->track_cm / dt;
    gyro_angle += w_meas * dt;

    bool done = t >= t_end;
    switch (kind) {
    case B_MOTOR:
        *duty_l = a_l;
        *duty_r = a_r;
        break;
    case B_STRAIGHT: {
        float travelled = 0.5f * (float)(ticks_l + ticks_r) / p->ticks_per_cm;
        float remain = fabsf(a_l) - fabsf(travelled);
        if (remain <= 0.0f) { done = true; break; }
        float v_lim = sqrtf(2.0f * RAMP_CM_S2 * remain);  /* giảm tốc để dừng đúng quãng */
        float v_des = fminf(a_r, v_lim);
        v_now = (v_des > v_now) ? fminf(v_des, v_now + RAMP_CM_S2 * dt) : v_des;
        float v = (a_l >= 0.0f) ? v_now : -v_now;
        Wheel_Step(wc, g, v, 0.0f, p->track_cm, dl / dt, dr / dt, w_meas, dt, duty_l, duty_r);
        break;
    }
    case B_SPIN: {
        float remain = fabsf(a_l) - fabsf(gyro_angle);
        if (remain <= 0.0f) { done = true; break; }
        float w = fminf(a_r, 0.5f + 3.0f * remain);  /* chậm lại gần góc đích */
        if (a_l < 0.0f) w = -w;
        Wheel_Step(wc, g, 0.0f, w, p->track_cm, dl / dt, dr / dt, w_meas, dt, duty_l, duty_r);
        break;
    }
    default:
        break;
    }
    if (done) {
        *duty_l = *duty_r = 0.0f;
        report(p, gyro_ok);
        kind = B_NONE;
        return false;
    }
    return true;
}
