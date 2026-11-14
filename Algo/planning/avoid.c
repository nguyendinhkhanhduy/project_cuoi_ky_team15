#include "avoid.h"

#include <math.h>

#include "astar_grid.h"
#include "geometry.h"
#include "localize.h"

#define DEG2RAD 0.017453293f
#define BLOCK_HALF 5.0f        /* vật cản ngẫu nhiên cũng là khối 10 x 10 cm */
#define CHECK_AHEAD_CM 80.0f
#define CHECK_STEP_CM 2.0f
#define DETOUR_STEP_CM 5.0f
/* Khi hỏi "map có giải thích được số đo không", nới nón: sai số hướng làm mép nón sượt qua góc khối
 * đã biết, gây báo nhầm vật lạ. Sàn 5° (mô phỏng: 3 sigma EKF đơn thuần quá hẹp, 200 lượt gốc còn
 * 195/200 và vật lạ (135,67) còn 69/100); nới thêm khi EKF kém chắc về hướng, trần 8° để không che
 * mất vật thật (sweep: 0 va chạm ở 5°). */
#define KNOWN_CONE_MIN_DEG 5.0f
#define KNOWN_CONE_MAX_DEG 8.0f

void Avoid_Reset(Avoid *a)
{
    for (int i = 0; i < NAV_TOF_COUNT; i++) a->confirm[i] = 0;
    for (int i = 0; i < NAV_IR_COUNT; i++) a->ir_confirm[i] = 0;
}

int Avoid_Detect(Avoid *a, const Pose *pose, const NavInput *in, const NavParams *p, const NavRect *rects,
                 uint16_t n, float heading_sigma, NavRect *found)
{
    float extra = fminf(fmaxf(3.0f * heading_sigma, KNOWN_CONE_MIN_DEG * DEG2RAD), KNOWN_CONE_MAX_DEG * DEG2RAD);
    float half_fov = 0.5f * p->tof_fov_deg * DEG2RAD + extra;
    for (int i = 0; i < NAV_TOF_COUNT; i++) {
        if (!(in->tof_fresh & (1U << i))) continue;
        float m = 0.1f * (float)in->tof_mm[i];
        if (m >= p->avoid_range_cm) { a->confirm[i] = 0; continue; }

        float sx, sy, ang;
        Loc_TofPose(pose, p, i, &sx, &sy, &ang);
        float e = Geo_ConeCast(sx, sy, ang, half_fov, rects, n, 1U);
        if (m >= e - p->avoid_margin_cm) { a->confirm[i] = 0; continue; }

        /* Điểm chạm ước lượng: dọc tia giữa. Nếu sát vật đã biết thì chỉ là sai số góc nhìn. */
        float hx = sx + m * cosf(ang), hy = sy + m * sinf(ang);
        if (hx < 0.0f || hx > 180.0f || hy < 0.0f || hy > 120.0f ||
            Geo_PointClearance(hx, hy, rects, n) < p->avoid_known_tol_cm) {
            a->confirm[i] = 0;
            continue;
        }
        if (++a->confirm[i] < p->avoid_confirm) continue;

        a->confirm[i] = 0;
        float cx = hx + BLOCK_HALF * cosf(ang), cy = hy + BLOCK_HALF * sinf(ang);
        found->x0 = cx - BLOCK_HALF;
        found->y0 = cy - BLOCK_HALF;
        found->x1 = cx + BLOCK_HALF;
        found->y1 = cy + BLOCK_HALF;
        found->visible = 1U;
        return 1;
    }
    return 0;
}

#define IR_REACH_CM 4.0f  /* module IR dò vật cản thường chỉnh báo trong vài cm phía trước */

int Avoid_DetectIr(Avoid *a, const Pose *pose, const NavInput *in, const NavParams *p, const NavRect *rects,
                   uint16_t n, NavRect *found)
{
    if (p->ir_mode != NAV_IR_FORWARD) return 0;
    float c = cosf(pose->th), s = sinf(pose->th);
    for (int i = 0; i < NAV_IR_COUNT; i++) {
        if (!(in->ir_on_tape & (1U << i))) { a->ir_confirm[i] = 0; continue; }
        float wx, wy;
        Geo_ToWorld(pose, p->ir_x[i] + IR_REACH_CM, p->ir_y[i], &wx, &wy);
        if (wx < 0.0f || wx > 180.0f || wy < 0.0f || wy > 120.0f ||
            Geo_PointClearance(wx, wy, rects, n) < p->avoid_known_tol_cm) {  /* ngoài sân / khối đã biết */
            a->ir_confirm[i] = 0;
            continue;
        }
        if (++a->ir_confirm[i] < p->avoid_confirm) continue;  /* như ToF: một mẫu nhiễu không đủ */
        a->ir_confirm[i] = 0;
        float cx = wx + BLOCK_HALF * c, cy = wy + BLOCK_HALF * s;
        found->x0 = cx - BLOCK_HALF;
        found->y0 = cy - BLOCK_HALF;
        found->x1 = cx + BLOCK_HALF;
        found->y1 = cy + BLOCK_HALF;
        found->visible = 1U;
        return 1;
    }
    return 0;
}

/* Tìm đoạn [s_first, s_last] của P bị vật lạ chắn trong khoảng nhìn phía trước. */
static int blocked_range(const NavPath *P, float s, const NavRect *obstacle, const NavParams *p, float *s_first,
                         float *s_last)
{
    float end = fminf(s + CHECK_AHEAD_CM, Path_Length(P));
    int hit = 0;
    for (float q = s; q <= end; q += CHECK_STEP_CM) {
        NavPoint pt = Path_At(P, q);
        if (Geo_PointRectDist(pt.x, pt.y, obstacle) < p->r_hard_cm) {
            if (!hit) *s_first = q;
            *s_last = q;
            hit = 1;
        }
    }
    return hit;
}

int Avoid_Blocks(const NavPath *P, float s, const NavRect *obstacle, const NavParams *p)
{
    float a, b;
    return blocked_range(P, s, obstacle, p, &a, &b);
}

uint16_t Avoid_Plan(const NavPath *P, float s, const Pose *pose, const NavRect *obstacle, const NavParams *p,
                    const NavRect *rects, uint16_t n, NavPoint *out, uint16_t max_out, float *s_rejoin)
{
    float s_first, s_last;
    if (!blocked_range(P, s, obstacle, p, &s_first, &s_last)) return 0;

    /* Điểm nhập lại: sau đoạn bị chắn một khoảng. Có thể nằm dưới vạch đích (y < 0): A* trên xe
     * cho phép đi xuống dưới y = 0 trong đoạn x của vạch đích. */
    float total = Path_Length(P);
    float s_goal = fminf(s_last + p->avoid_rejoin_cm, total);
    NavPoint goal = Path_At(P, s_goal);

    NavPoint detour[24];
    NavPoint start = {pose->x, pose->y};
    uint16_t nd = Astar_Plan(start, goal, rects, n, p->r_hard_cm, p->r_soft_cm, detour, 24);
    if (nd < 2 && p->r_tight_cm > 0.0f && p->r_tight_cm < p->r_hard_cm) {
        /* Không đủ chỗ để xoay tại chỗ suốt đường: cho phép lách khe chỉ đủ chạy thẳng.
         * Luật ưu tiên "qua đích" trên "chạy sạch", nên lách vẫn hơn dừng hẳn. */
        nd = Astar_Plan(start, goal, rects, n, p->r_tight_cm, p->r_soft_cm, detour, 24);
    }
    if (nd < 2) return 0;

    /* Ghép: đường vòng (lấy mẫu mỗi 5 cm) + phần quỹ đạo gốc sau điểm nhập lại. */
    uint16_t k = 0;
    float len = 0.0f;
    out[k++] = detour[0];
    for (uint16_t i = 0; i + 1 < nd; i++) {
        float dx = detour[i + 1].x - detour[i].x, dy = detour[i + 1].y - detour[i].y;
        float seg = sqrtf(dx * dx + dy * dy);
        int m = (int)ceilf(seg / DETOUR_STEP_CM);
        if (m < 1) m = 1;
        for (int j = 1; j <= m; j++) {
            if (k >= max_out) return 0;  /* không đủ chỗ: đường vòng bị cắt */
            float t = (float)j / (float)m;
            out[k].x = detour[i].x + t * dx;
            out[k].y = detour[i].y + t * dy;
            k++;
        }
        len += seg;
    }
    *s_rejoin = len;
    for (uint16_t i = Path_IndexAfter(P, s_goal + 0.5f); i < P->n; i++) {
        if (k >= max_out) return 0;  /* phần quỹ đạo gốc phía sau bị cắt: không về được đích */
        out[k++] = P->p[i];
    }
    return k;
}
