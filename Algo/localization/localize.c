#include "localize.h"

#include <math.h>

#include "geometry.h"
#include "nav_map.h"

#define DEG2RAD 0.017453293f
#define IR_IGNORE_BELOW_Y NAV_IR_IGNORE_BELOW_Y
#define GATE_SIGMA 3.0f          /* loại số đo lệch quá 3 sigma */
#define D_POS 0.2f               /* bước sai phân để tính Jacobian */
#define D_TH  0.004f
#define IR_SIGMA_CM 0.6f

void Loc_TofPose(const Pose *pose, const NavParams *p, int i, float *sx, float *sy, float *ang)
{
    Geo_ToWorld(pose, p->tof_x[i], p->tof_y[i], sx, sy);
    *ang = pose->th + p->tof_ang_deg[i] * DEG2RAD;
}

void Loc_Init(LocState *s, const NavParams *p)
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) s->P[i][j] = 0.0f;
    s->P[0][0] = p->init_sigma_cm * p->init_sigma_cm;
    s->P[1][1] = p->init_sigma_cm * p->init_sigma_cm;
    s->P[2][2] = p->init_sigma_rad * p->init_sigma_rad;
    s->tof_fixes = 0;
    s->ir_fixes = 0;
}

void Loc_Predict(LocState *s, const Pose *pose, float ds, float dth, float dt, const NavParams *p)
{
    /* P = F P F^T + Q, F = d(pose mới)/d(pose cũ) */
    float th_mid = pose->th - 0.5f * dth;
    float a = -ds * sinf(th_mid), b = ds * cosf(th_mid);
    float (*P)[3] = s->P;
    float FP[3][3];
    for (int j = 0; j < 3; j++) {
        FP[0][j] = P[0][j] + a * P[2][j];
        FP[1][j] = P[1][j] + b * P[2][j];
        FP[2][j] = P[2][j];
    }
    for (int i = 0; i < 3; i++) {
        P[i][0] = FP[i][0] + a * FP[i][2];
        P[i][1] = FP[i][1] + b * FP[i][2];
        P[i][2] = FP[i][2];
    }
    float sd = p->odo_sigma_frac * fabsf(ds);
    float sth = p->turn_sigma_frac * fabsf(dth);
    P[0][0] += sd * sd;
    P[1][1] += sd * sd;
    P[2][2] += sth * sth + p->gyro_drift_sigma * p->gyro_drift_sigma * dt;
}

/* Cập nhật Kalman với một số đo vô hướng: z - h(x) = innov, H = dh/dx. Trả về 1 nếu được dùng. */
static int ekf_scalar(LocState *s, Pose *pose, const float H[3], float innov, float R)
{
    float (*P)[3] = s->P;
    float PH[3];
    for (int i = 0; i < 3; i++) PH[i] = P[i][0] * H[0] + P[i][1] * H[1] + P[i][2] * H[2];
    float S = H[0] * PH[0] + H[1] * PH[1] + H[2] * PH[2] + R;
    if (S <= 0.0f || innov * innov > GATE_SIGMA * GATE_SIGMA * S) return 0;

    float K[3] = {PH[0] / S, PH[1] / S, PH[2] / S};
    pose->x += K[0] * innov;
    pose->y += K[1] * innov;
    pose->th = Geo_WrapPi(pose->th + K[2] * innov);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) P[i][j] -= K[i] * PH[j];
    for (int i = 0; i < 3; i++)
        for (int j = i + 1; j < 3; j++) P[i][j] = P[j][i] = 0.5f * (P[i][j] + P[j][i]);
    return 1;
}

static float tof_expect(const Pose *pose, const NavParams *p, int i, const NavRect *rects, uint16_t n)
{
    float sx, sy, ang;
    Loc_TofPose(pose, p, i, &sx, &sy, &ang);
    return Geo_ConeCast(sx, sy, ang, 0.5f * p->tof_fov_deg * DEG2RAD, rects, n, 1U);
}

static void update_tof(LocState *s, Pose *pose, const NavInput *in, const NavParams *p, const NavRect *rects,
                       uint16_t n)
{
    for (int i = 0; i < NAV_TOF_COUNT; i++) {
        if (!(in->tof_fresh & (1U << i))) continue;
        float m = 0.1f * (float)in->tof_mm[i];
        if (m >= p->tof_max_cm || m >= p->loc_range_cm) continue;

        float e = tof_expect(pose, p, i, rects, n);
        if (e >= p->loc_range_cm || fabsf(m - e) > p->loc_gate_cm) continue;

        /* Jacobian bằng sai phân: map gồm các hình chữ nhật nên không có công thức gọn. */
        Pose q = *pose;
        float H[3];
        q.x += D_POS; H[0] = (tof_expect(&q, p, i, rects, n) - e) / D_POS; q.x = pose->x;
        q.y += D_POS; H[1] = (tof_expect(&q, p, i, rects, n) - e) / D_POS; q.y = pose->y;
        q.th += D_TH; H[2] = (tof_expect(&q, p, i, rects, n) - e) / D_TH;
        /* Bỏ Jacobian vô lý (tia giữa nhảy sang vật khác khi xê dịch) */
        if (fabsf(H[0]) > 3.0f || fabsf(H[1]) > 3.0f || fabsf(H[2]) > 300.0f) continue;

        float sigma = p->tof_sigma_cm + 0.02f * m;
        if (ekf_scalar(s, pose, H, m - e, sigma * sigma)) s->tof_fixes++;
    }
}

/* Khoảng cách có dấu từ điểm IR tới đường thẳng chứa đoạn băng dính gần nhất. */
static float ir_signed_dist(const Pose *pose, const NavParams *p, int i, const NavSegment *seg)
{
    float wx, wy;
    Geo_ToWorld(pose, p->ir_x[i], p->ir_y[i], &wx, &wy);
    float dx = seg->x1 - seg->x0, dy = seg->y1 - seg->y0;
    float len = sqrtf(dx * dx + dy * dy);
    return ((wx - seg->x0) * dy - (wy - seg->y0) * dx) / len;
}

static void update_ir(LocState *s, Pose *pose, const NavInput *in, const NavParams *p)
{
    if (p->ir_mode != NAV_IR_TAPE) return;
    uint16_t nt;
    const NavSegment *tape = NavMap_Tape(&nt);
    for (int i = 0; i < NAV_IR_COUNT; i++) {
        if (!(in->ir_on_tape & (1U << i))) continue;
        float wx, wy;
        Geo_ToWorld(pose, p->ir_x[i], p->ir_y[i], &wx, &wy);
        if (wy < IR_IGNORE_BELOW_Y) continue;

        float best = 1e9f;
        const NavSegment *seg = 0;
        for (uint16_t k = 0; k < nt; k++) {
            float d = Geo_PointSegDist(wx, wy, tape[k].x0, tape[k].y0, tape[k].x1, tape[k].y1, 0, 0);
            if (d < best) { best = d; seg = &tape[k]; }
        }
        if (!seg || best > p->ir_gate_cm) continue;

        /* IR đang nằm trên vạch -> khoảng cách tới vạch = 0 */
        float h = ir_signed_dist(pose, p, i, seg);
        Pose q = *pose;
        float H[3];
        q.x += D_POS; H[0] = (ir_signed_dist(&q, p, i, seg) - h) / D_POS; q.x = pose->x;
        q.y += D_POS; H[1] = (ir_signed_dist(&q, p, i, seg) - h) / D_POS; q.y = pose->y;
        q.th += D_TH; H[2] = (ir_signed_dist(&q, p, i, seg) - h) / D_TH;
        if (ekf_scalar(s, pose, H, -h, IR_SIGMA_CM * IR_SIGMA_CM)) s->ir_fixes++;
    }
}

void Loc_Update(LocState *s, Pose *pose, const NavInput *in, const NavParams *p, const NavRect *rects,
                uint16_t n)
{
    update_tof(s, pose, in, p, rects, n);
    update_ir(s, pose, in, p);
}
