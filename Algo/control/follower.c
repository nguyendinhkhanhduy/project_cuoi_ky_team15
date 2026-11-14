#include "follower.h"

#include <math.h>

#include "geometry.h"

#define PROJECT_BACK_CM 2.0f
#define PROJECT_FWD_CM  25.0f
#define FINISH_ZONE_CM  15.0f
#define SPIN_GAIN       2.5f
#define NO_SPIN_END_CM  5.0f  /* sát cuối quỹ đạo thì không quay tại chỗ */

void Fol_Reset(Follower *f, float s0, float v0)
{
    f->s = s0;
    f->v = v0;
    f->lookahead = 0.0f;
    f->target = (NavPoint){0.0f, 0.0f};
    f->done = 0U;
}

static float clampf(float x, float lo, float hi)
{
    return fminf(fmaxf(x, lo), hi);
}

/* Đưa v hiện tại tiến dần tới v_des, không vượt gia tốc cho phép. */
static float ramp(float v, float v_des, float a, float dt)
{
    float dv = a * dt;
    if (v_des > v + dv) return v + dv;
    if (v_des < v - dv) return v - dv;
    return v_des;
}

void Fol_Step(Follower *f, const NavPath *P, const Pose *pose, const NavParams *p, float dt, float *v_out,
              float *w_out)
{
    float total = Path_Length(P);
    float proj = Path_Project(P, pose->x, pose->y, f->s - PROJECT_BACK_CM, f->s + PROJECT_FWD_CM);
    if (proj > f->s) f->s = proj;
    if (P->n >= 2 && f->s >= total - 0.01f) {
        /* Path_Project kẹp ở điểm cuối; quá cuối thì đo tiếp dọc phần kéo dài của đoạn cuối. */
        const NavPoint *a = &P->p[P->n - 2], *b = &P->p[P->n - 1];
        float ux = b->x - a->x, uy = b->y - a->y, len = sqrtf(ux * ux + uy * uy);
        if (len > 1e-3f) {
            float ext = ((pose->x - b->x) * ux + (pose->y - b->y) * uy) / len;
            if (total + ext > f->s) f->s = total + ext;
        }
    }
    float remaining = total - f->s;

    /* Về đích khi điểm cao nhất của thân xe (tính cả khi chạy xiên) đã xuống dưới finish_y_cm.
     * Không dừng chỉ vì hết quỹ đạo: Path_At kéo dài thẳng đoạn cuối nên xe chạy tiếp qua vạch. */
    float body_top = pose->y + 0.5f * p->body_len_cm * fabsf(sinf(pose->th)) +
                     0.5f * p->body_wid_cm * fabsf(cosf(pose->th));
    if ((body_top < p->finish_y_cm && remaining < FINISH_ZONE_CM) || (remaining <= 0.0f && body_top < 0.0f) ||
        remaining < -FINISH_ZONE_CM) {
        /* Nhánh 2: đã hết quỹ đạo và cả thân xe (theo ước lượng) đã qua vạch: phần kéo dài của đoạn
         *   cuối có thể chỉ ngang (đường vòng kết thúc đúng cuối quỹ đạo) và dẫn xe ra biên.
         * Nhánh 3: quá cuối 15 cm mà chưa qua vạch (quỹ đạo Lượt 1 kết thúc trước vạch): dừng thay vì bò mãi. */
        f->done = 1U;
    }
    if (f->done) {
        f->v = 0.0f;
        *v_out = 0.0f;
        *w_out = 0.0f;
        return;
    }

    /* Lookahead: tỉ lệ với tốc độ, rút ngắn khi phía trước có góc cua gắt để không cắt góc. */
    float turn = Path_TurnAhead(P, f->s, p->turn_window_cm);
    float tf = clampf(turn / p->turn_full_rad, 0.0f, 1.0f);
    float L = clampf(p->la_gain_s * f->v, p->la_min, p->la_max);
    L -= (L - p->la_min) * tf;
    f->lookahead = L;
    f->target = Path_At(P, f->s + L);  /* vượt cuối thì kéo dài thẳng: xe chạy hẳn qua vạch đích */

    float dx = f->target.x - pose->x, dy = f->target.y - pose->y;
    float c = cosf(pose->th), s = sinf(pose->th);
    float lx = c * dx + s * dy, ly = -s * dx + c * dy;
    float alpha = atan2f(ly, lx);
    /* Sát cuối quỹ đạo: không quay tại chỗ (dễ kẹt quay mãi), cứ bám tiếp phần kéo dài; nếu quá
     * cuối 15 cm mà chưa qua vạch thì luật phía trên sẽ dừng. */
    int allow_spin = remaining >= NO_SPIN_END_CM;

    float v, w;
    if (allow_spin && fabsf(alpha) > p->spin_angle_rad) {
        /* Điểm đích lệch quá nhiều: phanh rồi quay tại chỗ. */
        v = ramp(f->v, 0.0f, 2.0f * p->a_max, dt);
        w = (v < 1.0f) ? copysignf(fminf(p->w_max, SPIN_GAIN * fabsf(alpha)), alpha) : 0.0f;
    } else {
        float d2 = lx * lx + ly * ly;
        float kappa = (d2 > 1e-6f) ? 2.0f * ly / d2 : 0.0f;

        float v_des = p->v_max * (1.0f - p->turn_slow * tf);
        if (fabsf(kappa) > 1e-4f) v_des = fminf(v_des, sqrtf(p->a_lat_max / fabsf(kappa)));
        v_des = fmaxf(v_des, p->v_min);
        /* Gần cuối quỹ đạo thì giảm tốc để dừng đúng điểm cuối. */
        v_des = fminf(v_des, sqrtf(2.0f * p->a_max * fmaxf(remaining, 0.0f)) + 0.5f * p->v_min);

        v = ramp(f->v, v_des, p->a_max, dt);
        w = v * kappa;
        if (fabsf(w) > p->w_max) {
            w = copysignf(p->w_max, w);
            v = p->w_max / fabsf(kappa);
        }
    }
    f->v = v;
    *v_out = v;
    *w_out = w;
}
