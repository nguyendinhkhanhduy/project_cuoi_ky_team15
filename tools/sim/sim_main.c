/* Xe ảo + sân ảo để chạy thử code Algo/ trên PC.
 *
 * Mỗi bước 5 ms: xe ảo (biết vị trí THẬT) tạo số đo encoder, gyro, VL53L0X, IR có nhiễu
 * -> đưa cho Nav_Step() -> nhận (v, w) -> cho xe ảo di chuyển -> kiểm tra va chạm.
 *
 * Dùng: sim.exe [--seed N] [--noise K] [--mode astar|teach] [--obstacle X,Y]... [--set ten=gia_tri]...
 *               [--out file.csv] [--tmax giay]
 * In ra một dòng "RESULT ..." để tools/sim/run_sim.py đọc.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "geometry.h"
#include "localize.h"
#include "nav_io.h"
#include "nav_map.h"
#include "nav_path.h"

#define DT          0.005f
#define CSV_EVERY   4          /* ghi CSV mỗi 20 ms */
#define TOF_PERIOD  0.033f
#define TAPE_HALF   0.9f       /* băng dính rộng ~1.8 cm */
#define MOTOR_TAU   0.04f      /* hằng số thời gian của vòng PID vận tốc bánh (driver) */
#define WHEEL_VMAX  60.0f
#define MAX_OBS     3
#define MAX_RECTS   32
#define BORDERS     3          /* NavMap: 3 hình chữ nhật cuối là viền sân */
#define HUMAN_LA    8.0f       /* người lái tay nhìn trước 8 cm */
#define HUMAN_PERIOD 0.05f     /* app gửi lệnh mỗi 50 ms */

/* ---------------------------------------------------------------- Số ngẫu nhiên */
static uint64_t rng_state = 88172645463325252ULL;
static double urand(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return (double)(rng_state >> 11) * (1.0 / 9007199254740992.0);
}
static float gauss(float sigma)
{
    double u1 = urand(), u2 = urand();
    if (u1 < 1e-12) u1 = 1e-12;
    return (float)(sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2)) * sigma;
}

/* ---------------------------------------------------------------- Thế giới */
typedef struct {
    float noise;
    NavParams prm;           /* thông số thuật toán dùng (danh định) */
    /* sai khác của xe thật so với danh định: cố định cho một chiếc xe */
    float e_left, e_right;   /* sai bán kính bánh */
    float track_true;
    float gyro_scale;
    /* trạng thái */
    Pose truth;
    float enc_vl, enc_vr;    /* tốc độ bánh theo encoder, cm/s */
    float acc_l, acc_r;      /* phần lẻ xung encoder */
    float gyro_bias;
    float tof_timer[NAV_TOF_COUNT];
    uint16_t tof_last[NAV_TOF_COUNT];
    NavRect rects[MAX_RECTS];
    uint16_t n_obs;          /* rects[0..n_obs) = vật cản; tiếp theo là viền */
    uint16_t n_all;
} World;

static void world_reset_motion(World *w, const Pose *start)
{
    w->truth = *start;
    w->truth.x += gauss(0.3f * w->noise);
    w->truth.y += gauss(0.3f * w->noise);
    w->truth.th += gauss(0.017f * w->noise);
    w->enc_vl = w->enc_vr = 0.0f;
    w->acc_l = w->acc_r = 0.0f;
    w->gyro_bias = gauss(0.002f * w->noise);
    for (int i = 0; i < NAV_TOF_COUNT; i++) {
        w->tof_timer[i] = (float)i * 0.008f;
        w->tof_last[i] = 8190;
    }
}

static void world_init(World *w, const float obs[][2], int n_obs_rand)
{
    w->e_left = gauss(0.01f * w->noise);
    w->e_right = gauss(0.01f * w->noise);
    w->track_true = w->prm.track_cm * (1.0f + gauss(0.02f * w->noise));
    w->gyro_scale = gauss(0.01f * w->noise);

    uint16_t n;
    NavMap_Init(w->prm.border_visible);
    const NavRect *r = NavMap_Rects(&n);
    uint16_t k = 0;
    for (uint16_t i = 0; i + BORDERS < n; i++) w->rects[k++] = r[i];
    for (int i = 0; i < n_obs_rand; i++) {
        NavRect o = {obs[i][0] - 5.0f, obs[i][1] - 5.0f, obs[i][0] + 5.0f, obs[i][1] + 5.0f, 1U};
        w->rects[k++] = o;
    }
    w->n_obs = k;
    for (uint16_t i = (uint16_t)(n - BORDERS); i < n; i++) w->rects[k++] = r[i];
    w->n_all = k;
}

static int ir_on_tape(const World *w, float x, float y)
{
    uint16_t nt;
    const NavSegment *t = NavMap_Tape(&nt);
    for (uint16_t k = 0; k < nt; k++) {
        if (Geo_PointSegDist(x, y, t[k].x0, t[k].y0, t[k].x1, t[k].y1, 0, 0) < TAPE_HALF) return 1;
    }
    /* vạch đích ô cờ: một nửa diện tích là ô đen */
    if (y > -1.0f && y < 1.0f && x > 120.0f && x < 180.0f && fmodf(x, 4.0f) < 2.0f) return 1;
    (void)w;
    return 0;
}

/* Tạo số đo cảm biến từ vị trí thật cho chu kỳ tiếp theo, rồi cho xe ảo di chuyển theo (v, w). */
static void world_sense(World *w, NavInput *in)
{
    const NavParams *p = &w->prm;
    in->dt_s = DT;

    /* encoder: đếm theo vòng quay bánh, không biết bánh trượt hay sai bán kính */
    w->acc_l += w->enc_vl * DT * p->ticks_per_cm;
    w->acc_r += w->enc_vr * DT * p->ticks_per_cm;
    in->d_ticks_left = (int32_t)w->acc_l;
    in->d_ticks_right = (int32_t)w->acc_r;
    w->acc_l -= (float)in->d_ticks_left;
    w->acc_r -= (float)in->d_ticks_right;

    float gl = w->enc_vl * (1.0f + w->e_left), gr = w->enc_vr * (1.0f + w->e_right);
    float w_true = (gr - gl) / w->track_true;
    w->gyro_bias += gauss(0.0003f * w->noise * sqrtf(DT));
    in->gyro_z_rad_s = w_true * (1.0f + w->gyro_scale) + w->gyro_bias + gauss(0.003f * w->noise);

    in->tof_fresh = 0;
    float half_fov = 0.5f * p->tof_fov_deg * 0.017453293f;
    for (int i = 0; i < NAV_TOF_COUNT; i++) {
        w->tof_timer[i] += DT;
        if (w->tof_timer[i] >= TOF_PERIOD) {
            w->tof_timer[i] -= TOF_PERIOD;
            float sx, sy, ang;
            Loc_TofPose(&w->truth, p, i, &sx, &sy, &ang);
            float d = Geo_ConeCast(sx, sy, ang, half_fov, w->rects, w->n_all, 1U);
            if (d < p->tof_max_cm && urand() > 0.003 * w->noise) {
                float m = d + gauss(0.5f * w->noise);
                w->tof_last[i] = (uint16_t)fmaxf(m * 10.0f, 0.0f);
            } else {
                w->tof_last[i] = 8190;
            }
            in->tof_fresh |= (uint8_t)(1U << i);
        }
        in->tof_mm[i] = w->tof_last[i];
    }

    in->ir_on_tape = 0;
    for (int i = 0; i < NAV_IR_COUNT; i++) {
        float x, y;
        Geo_ToWorld(&w->truth, p->ir_x[i], p->ir_y[i], &x, &y);
        if (ir_on_tape(w, x, y)) in->ir_on_tape |= (uint8_t)(1U << i);
    }
}

static void world_move(World *w, float v, float wz)
{
    const NavParams *p = &w->prm;
    /* driver: đổi (v, w) thành tốc độ bánh theo khoảng cách bánh DANH ĐỊNH, PID bám có trễ */
    float tl = fminf(fmaxf(v - 0.5f * wz * p->track_cm, -WHEEL_VMAX), WHEEL_VMAX);
    float tr = fminf(fmaxf(v + 0.5f * wz * p->track_cm, -WHEEL_VMAX), WHEEL_VMAX);
    w->enc_vl += (tl - w->enc_vl) * DT / MOTOR_TAU + gauss(0.2f * w->noise);
    w->enc_vr += (tr - w->enc_vr) * DT / MOTOR_TAU + gauss(0.2f * w->noise);

    /* mặt đất: sai bán kính bánh + trượt */
    float gl = w->enc_vl * (1.0f + w->e_left) * (1.0f - fabsf(gauss(0.01f * w->noise)));
    float gr = w->enc_vr * (1.0f + w->e_right) * (1.0f - fabsf(gauss(0.01f * w->noise)));
    float vt = 0.5f * (gl + gr), wt = (gr - gl) / w->track_true;
    float th_mid = w->truth.th + 0.5f * wt * DT;
    w->truth.x += vt * DT * cosf(th_mid);
    w->truth.y += vt * DT * sinf(th_mid);
    w->truth.th = Geo_WrapPi(w->truth.th + wt * DT);
}

/* ---------------------------------------------------------------- Người lái tay giả lập (Lượt 1) */
typedef struct {
    NavPath path;
    float s;
    float timer;
    int sent_speed;
} Human;

static char human_command(Human *h, const Pose *truth)
{
    h->timer += DT;
    if (h->timer < HUMAN_PERIOD) return 0;
    h->timer = 0.0f;
    if (!h->sent_speed) { h->sent_speed = 1; return '5'; }  /* 50 % */
    if (truth->y < -7.0f) return 'S';

    float proj = Path_Project(&h->path, truth->x, truth->y, h->s - 2.0f, h->s + 20.0f);
    if (proj > h->s) h->s = proj;
    NavPoint t = Path_At(&h->path, h->s + HUMAN_LA);
    float err = Geo_WrapPi(atan2f(t.y - truth->y, t.x - truth->x) - truth->th);
    if (err > 0.6f) return 'L';
    if (err < -0.6f) return 'R';
    if (err > 0.15f) return 'G';
    if (err < -0.15f) return 'I';
    return 'F';
}

/* ---------------------------------------------------------------- Chạy một lượt */
typedef struct {
    const char *status;
    float t;
    float min_clear;
} Episode;

static const char *state_name(NavState s)
{
    static const char *names[] = {"IDLE", "RUN", "BRAKE", "AVOID", "TEACH", "DONE", "STOPPED", "EDGE"};
    return ((unsigned)s < sizeof(names) / sizeof(names[0])) ? names[s] : "?";
}

static Episode run_episode(World *w, Human *human, float tmax, FILE *csv)
{
    Episode ep = {"TIMEOUT", 0.0f, GEO_INF};
    NavInput in;
    memset(&in, 0, sizeof(in));
    float v = 0.0f, wz = 0.0f, done_timer = 0.0f;
    int step = 0;

    for (float t = 0.0f; t < tmax; t += DT, step++) {
        world_sense(w, &in);
        in.manual_cmd = human ? human_command(human, &w->truth) : 0;
        NavOutput o = Nav_Step(&in);
        v = o.v_cm_s;
        wz = o.w_rad_s;
        world_move(w, v, wz);

        const NavParams *p = &w->prm;
        float c_obs = Geo_BodyClearance(&w->truth, p->body_len_cm, p->body_wid_cm, w->rects, w->n_obs);
        float c_brd = Geo_BodyClearance(&w->truth, p->body_len_cm, p->body_wid_cm, w->rects + w->n_obs,
                                        (uint16_t)(w->n_all - w->n_obs));
        float c = fminf(c_obs, c_brd);
        if (c < ep.min_clear) ep.min_clear = c;
        ep.t = t;

        if (csv && (step % CSV_EVERY == 0)) {
            const Pose *e = Nav_GetPose();
            const NavDebug *d = Nav_GetDebug();
            fprintf(csv, "%.3f,%.2f,%.2f,%.4f,%.2f,%.2f,%.4f,%.2f,%.3f,%s,%.2f,%.2f,%.2f\n", t, w->truth.x,
                    w->truth.y, w->truth.th, e->x, e->y, e->th, v, wz, state_name(o.state), c, d->target.x,
                    d->target.y);
        }
        if (c_obs <= 0.0f) { ep.status = "COLLIDE"; break; }
        if (c_brd <= 0.0f) { ep.status = "OUT"; break; }
        if (o.state == NAV_STOPPED) { ep.status = "STOPPED"; break; }
        if (o.state == NAV_DONE) {
            ep.status = (w->truth.y < -0.5f * p->body_len_cm) ? "DONE" : "DONE_SHORT";
            break;
        }
        if (human && w->truth.y < -7.0f) {
            done_timer += DT;
            if (done_timer > 0.5f) { ep.status = "TEACH_END"; break; }
        }
    }
    return ep;
}

/* ---------------------------------------------------------------- main */
int main(int argc, char **argv)
{
    World w;
    memset(&w, 0, sizeof(w));
    NavParams_Default(&w.prm);
    w.noise = 1.0f;
    unsigned long long seed = 1;
    const char *mode = "astar";
    const char *out = NULL;
    float tmax = 240.0f;
    float obs[MAX_OBS][2];
    int n_obs = 0;

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        const char *val = (i + 1 < argc) ? argv[i + 1] : NULL;
        if (!strcmp(a, "--seed") && val) { seed = strtoull(val, NULL, 10); i++; }
        else if (!strcmp(a, "--noise") && val) { w.noise = (float)atof(val); i++; }
        else if (!strcmp(a, "--mode") && val) { mode = val; i++; }
        else if (!strcmp(a, "--out") && val) { out = val; i++; }
        else if (!strcmp(a, "--tmax") && val) { tmax = (float)atof(val); i++; }
        else if (!strcmp(a, "--obstacle") && val && n_obs < MAX_OBS) {
            if (sscanf(val, "%f,%f", &obs[n_obs][0], &obs[n_obs][1]) == 2) n_obs++;
            i++;
        } else if (!strcmp(a, "--set") && val) {
            char name[64];
            float x;
            const char *eq = strchr(val, '=');
            if (!eq || (size_t)(eq - val) >= sizeof(name)) { fprintf(stderr, "--set sai: %s\n", val); return 2; }
            memcpy(name, val, (size_t)(eq - val));
            name[eq - val] = 0;
            x = (float)atof(eq + 1);
            if (NavParams_Set(&w.prm, name, x) != 0) { fprintf(stderr, "Không có thông số '%s'\n", name); return 2; }
            i++;
        } else {
            fprintf(stderr, "Tham số không hợp lệ: %s\n", a);
            return 2;
        }
    }
    rng_state ^= seed * 0x9E3779B97F4A7C15ULL;
    for (int i = 0; i < 10; i++) (void)urand();

    world_init(&w, obs, n_obs);
    FILE *csv = NULL;
    if (out) {
        csv = fopen(out, "w");
        if (!csv) { fprintf(stderr, "Không mở được %s\n", out); return 2; }
        fprintf(csv, "t,x,y,th,ex,ey,eth,v,w,state,clear,tx,ty\n");
    }

    uint16_t np;
    const NavPoint *astar = NavMap_Path(&np);
    Pose start = {astar[0].x, astar[0].y, NavMap_StartHeading()};
    static NavPoint recorded[NAV_MAX_PTS];
    uint16_t n_rec = 0;

    if (!strcmp(mode, "teach")) {
        /* Lượt 1: người lái tay theo đường A* (nhìn vị trí thật), thuật toán ghi lại. */
        static Human human;
        memset(&human, 0, sizeof(human));
        Path_Build(&human.path, astar, np);
        world_reset_motion(&w, &start);
        Nav_Init(&w.prm, NAV_MODE_TEACH, NULL, 0);
        Episode t1 = run_episode(&w, &human, tmax, NULL);
        n_rec = Nav_TeachFinish(recorded, NAV_MAX_PTS);
        printf("TEACH status=%s t=%.2f min_clear=%.2f points=%u\n", t1.status, (double)t1.t,
               (double)t1.min_clear, n_rec);
        if (out) {  /* lưu quỹ đạo đã ghi để vẽ */
            char path_csv[512];
            snprintf(path_csv, sizeof(path_csv), "%s.rec.csv", out);
            FILE *f = fopen(path_csv, "w");
            if (f) {
                fprintf(f, "x,y\n");
                for (uint16_t i = 0; i < n_rec; i++) fprintf(f, "%.2f,%.2f\n", (double)recorded[i].x, (double)recorded[i].y);
                fclose(f);
            }
        }
        if (strcmp(t1.status, "TEACH_END") != 0) {
            printf("RESULT status=TEACH_FAIL t=%.2f min_clear=%.2f end_err=0 tof_fixes=0 ir_fixes=0 unknown=0\n",
                   (double)t1.t, (double)t1.min_clear);
            if (csv) fclose(csv);
            return 0;
        }
    }

    /* Lượt 2: cùng chiếc xe, nhiễu ngẫu nhiên mới */
    world_reset_motion(&w, &start);
    if (n_rec > 1) Nav_Init(&w.prm, NAV_MODE_RUN_RECORDED, recorded, n_rec);
    else Nav_Init(&w.prm, NAV_MODE_RUN_ASTAR, NULL, 0);
    Episode ep = run_episode(&w, NULL, tmax, csv);

    const Pose *e = Nav_GetPose();
    const NavDebug *d = Nav_GetDebug();
    float end_err = sqrtf((e->x - w.truth.x) * (e->x - w.truth.x) + (e->y - w.truth.y) * (e->y - w.truth.y));
    printf("RESULT status=%s t=%.2f min_clear=%.2f end_err=%.2f tof_fixes=%lu ir_fixes=%lu unknown=%u edge=%lu\n",
           ep.status, (double)ep.t, (double)ep.min_clear, (double)end_err, (unsigned long)d->tof_fixes,
           (unsigned long)d->ir_fixes, d->unknown_count,
           (unsigned long)d->edge_events);
    if (csv) fclose(csv);
    return 0;
}
