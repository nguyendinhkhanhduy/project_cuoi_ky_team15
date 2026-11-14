/* Máy trạng thái dẫn đường: odometry -> sửa vị trí -> phát hiện vật lạ -> bám quỹ đạo / né / lái tay. */
#include <math.h>

#include "avoid.h"
#include "follower.h"
#include "geometry.h"
#include "localize.h"
#include "nav_io.h"
#include "nav_map.h"
#include "nav_path.h"
#include "odometry.h"
#include "recorder.h"

#define DT_MAX_S 0.05f
#define BRAKE_DONE_CM_S 0.5f
#define EDGE_MAX_REPEAT 6        /* phản xạ biên quá bấy nhiêu lần mà xe không tiến được ... */
#define EDGE_PROGRESS_CM 10.0f   /* ... chừng này cm dọc quỹ đạo thì dừng hẳn, tránh lắc qua lại mãi */
#define DETOUR_RESERVE_PTS 80    /* chừa chỗ trong NAV_MAX_PTS cho đường vòng ghép vào quỹ đạo Lượt 1 */

static NavParams P;
static NavState st = NAV_IDLE;
static Pose pose;
static NavPath base;      /* quỹ đạo gốc */
static NavPath active;    /* quỹ đạo đang bám (gốc hoặc đã ghép đường vòng) */
static NavPoint scratch[NAV_MAX_PTS];
static Follower fol;
static Avoid avoid;
static Recorder rec;
static LocState loc;
static NavRect pending;   /* vật lạ đang chờ lập đường vòng */
static float s_rejoin;
static NavDebug dbg;
static NavState edge_ret;     /* trạng thái quay về sau phản xạ biên */
static uint8_t edge_side;     /* bit0 = vạch bên trái, bit1 = bên phải */
static float edge_timer;
static uint8_t edge_streak;   /* số lần phản xạ liên tiếp mà chưa tiến được EDGE_PROGRESS_CM */
static float edge_streak_s;

void Nav_Init(const NavParams *params, NavMode mode, const NavPoint *path, uint16_t path_count)
{
    P = *params;
    NavMap_Init(P.border_visible);
    NavMap_ClearUnknown();
    if (path == 0 || path_count < 2) path = NavMap_Path(&path_count);
    Path_Build(&base, path, path_count);
    Path_Build(&active, base.p, base.n);

    pose.x = base.p[0].x;
    pose.y = base.p[0].y;
    pose.th = NavMap_StartHeading();

    Fol_Reset(&fol, 0.0f, 0.0f);
    Avoid_Reset(&avoid);
    Rec_Init(&rec, &pose);
    Loc_Init(&loc, &P);
    s_rejoin = 0.0f;
    edge_side = 0U;
    edge_timer = 0.0f;
    edge_streak = 0U;
    edge_streak_s = 0.0f;
    dbg.edge_events = 0;
    st = (mode == NAV_MODE_TEACH) ? NAV_TEACH : NAV_RUN;
}

/* IR chúc xuống thấy băng đen khi đang bám quỹ đạo = góc trước của xe đã chạm viền sân, bánh sắp
 * ra ngoài (luật: bánh không được ra ngoài vạch đen). Lùi chậm và xoay ra xa phía vạch; khi IR
 * hết thấy vạch thêm edge_hold_s giây thì trả quyền cho Pure Pursuit. Trả về 1 nếu đang phản xạ. */
static int edge_reflex(const NavInput *in, float *v, float *w)
{
    if (P.ir_mode != NAV_IR_TAPE) return 0;
    uint8_t m = 0;
    for (int i = 0; i < NAV_IR_COUNT; i++) {
        if (!(in->ir_on_tape & (1U << i))) continue;
        float wx, wy;
        Geo_ToWorld(&pose, P.ir_x[i], P.ir_y[i], &wx, &wy);
        if (wy >= NAV_IR_IGNORE_BELOW_Y) m |= (uint8_t)(1U << i);  /* gần y = 0 là vạch xuất phát/đích */
    }

    if (m) {
        if (st != NAV_EDGE) {
            if (edge_streak > 0U && fol.s < edge_streak_s + EDGE_PROGRESS_CM) {
                edge_streak++;
            } else {
                edge_streak = 1U;
                edge_streak_s = fol.s;
            }
            if (edge_streak > EDGE_MAX_REPEAT) {
                /* Quỹ đạo (theo pose ước lượng) cứ dẫn xe vào vạch: dừng còn hơn lắc mãi hoặc ra biên. */
                st = NAV_STOPPED;
                *v = 0.0f;
                *w = 0.0f;
                return 1;
            }
            edge_ret = st;
            st = NAV_EDGE;
            dbg.edge_events++;
        }
        edge_side = m;
        edge_timer = P.edge_hold_s;
    } else if (st == NAV_EDGE) {
        edge_timer -= in->dt_s;
        if (edge_timer <= 0.0f) {
            st = edge_ret;
            fol.v = 0.0f;  /* tăng tốc lại từ đứng yên */
            return 0;
        }
    } else {
        return 0;
    }

    *v = -P.edge_back_cm_s;
    *w = (edge_side == 0x01U) ? -P.edge_w : (edge_side == 0x02U) ? P.edge_w : 0.0f;
    fol.v = 0.0f;
    return 1;
}

static void step_run(const NavInput *in, const NavRect *rects, uint16_t nr, float *v, float *w)
{
    if (st != NAV_BRAKE && edge_reflex(in, v, w)) return;  /* có thể chuyển sang NAV_STOPPED */

    NavRect found;
    float heading_sigma = sqrtf(fmaxf(loc.P[2][2], 0.0f));
    if (st != NAV_BRAKE && (Avoid_Detect(&avoid, &pose, in, &P, rects, nr, heading_sigma, &found) ||
                            Avoid_DetectIr(&avoid, &pose, in, &P, rects, nr, &found))) {
        if (NavMap_AddUnknown(&found) == 0 && Avoid_Blocks(&active, fol.s, &found, &P)) {
            pending = found;
            st = NAV_BRAKE;
        }
    }

    if (st == NAV_BRAKE) {
        /* Phanh thẳng tới khi đứng yên rồi mới lập đường vòng (A* mất vài chục ms). */
        float vb = fol.v - P.a_max * in->dt_s;
        fol.v = (vb > 0.0f) ? vb : 0.0f;
        *v = fol.v;
        *w = 0.0f;
        if (fol.v > BRAKE_DONE_CM_S) return;

        rects = NavMap_Rects(&nr);
        uint16_t n = Avoid_Plan(&active, fol.s, &pose, &pending, &P, rects, nr, scratch, NAV_MAX_PTS, &s_rejoin);
        if (n == 0) {
            st = NAV_STOPPED;
            return;
        }
        Path_Build(&active, scratch, n);
        Fol_Reset(&fol, 0.0f, 0.0f);
        st = NAV_AVOID;
        return;
    }

    Fol_Step(&fol, &active, &pose, &P, in->dt_s, v, w);
    if (st == NAV_AVOID && fol.s > s_rejoin) st = NAV_RUN;
    if (fol.done) st = NAV_DONE;
}

NavOutput Nav_Step(const NavInput *in_raw)
{
    NavInput in = *in_raw;
    if (in.dt_s <= 0.0f || in.dt_s > DT_MAX_S) in.dt_s = (in.dt_s <= 0.0f) ? 0.005f : DT_MAX_S;

    NavOutput out = {0.0f, 0.0f, st};
    if (st == NAV_IDLE) return out;

    float ds, dth;
    Odo_Update(&pose, &in, &P, &ds, &dth);
    Loc_Predict(&loc, &pose, ds, dth, in.dt_s, &P);
    uint16_t nk;
    const NavRect *known = NavMap_KnownRects(&nk);
    Loc_Update(&loc, &pose, &in, &P, known, nk);

    uint16_t nr;
    const NavRect *rects = NavMap_Rects(&nr);

    float v = 0.0f, w = 0.0f;
    switch (st) {
    case NAV_TEACH:
        Rec_Command(&rec, &in, &P, &v, &w);
        Rec_Track(&rec, &pose, &P);
        break;
    case NAV_RUN:
    case NAV_BRAKE:
    case NAV_AVOID:
    case NAV_EDGE:
        step_run(&in, rects, nr, &v, &w);
        break;
    default:
        break;
    }

    dbg.s_cm = fol.s;
    dbg.path_len_cm = Path_Length(&active);
    dbg.lookahead_cm = fol.lookahead;
    dbg.target = fol.target;
    dbg.unknown_count = NavMap_UnknownCount();
    dbg.tof_fixes = loc.tof_fixes;
    dbg.ir_fixes = loc.ir_fixes;

    out.v_cm_s = v;
    out.w_rad_s = w;
    out.state = st;
    return out;
}

const Pose *Nav_GetPose(void) { return &pose; }
const NavDebug *Nav_GetDebug(void) { return &dbg; }
NavState Nav_GetState(void) { return st; }

uint16_t Nav_TeachFinish(NavPoint *out, uint16_t max_points)
{
    if (max_points > NAV_MAX_PTS - DETOUR_RESERVE_PTS) max_points = NAV_MAX_PTS - DETOUR_RESERVE_PTS;
    uint16_t n = Rec_Finish(&rec, &P, out, max_points);
    st = NAV_DONE;
    return n;
}
