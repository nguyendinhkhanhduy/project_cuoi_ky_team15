#include "recorder.h"

#include <math.h>

#include "geometry.h"

#define TURN_WHILE_MOVING 0.5f  /* G/I/H/J: quay bằng một nửa tốc độ quay tại chỗ */
#define DEFAULT_LEVEL 5U

void Rec_Init(Recorder *r, const Pose *start)
{
    r->n = 0;
    r->raw[r->n].x = start->x;
    r->raw[r->n].y = start->y;
    r->n++;
    r->move = 'S';
    r->level = DEFAULT_LEVEL;
    r->since_cmd_s = 0.0f;
}

void Rec_Command(Recorder *r, const NavInput *in, const NavParams *p, float *v, float *w)
{
    char c = in->manual_cmd;
    r->since_cmd_s += in->dt_s;
    if (c != 0) {
        r->since_cmd_s = 0.0f;
        if (c >= '0' && c <= '9') r->level = (uint8_t)(c - '0');  /* '0'..'9' = 0..90 % */
        else if (c == 'q') r->level = 10U;                         /* 100 % */
        else if (c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'G' || c == 'I' || c == 'H' ||
                 c == 'J' || c == 'S') r->move = c;
    }
    if (p->teach_timeout_s > 0.0f && r->since_cmd_s > p->teach_timeout_s) r->move = 'S';  /* mất Bluetooth */

    float vv = p->teach_v_max * (float)r->level / 10.0f;
    float ww = p->teach_w;
    float tw = TURN_WHILE_MOVING * ww;
    switch (r->move) {
    case 'F': *v = vv;  *w = 0.0f; break;
    case 'B': *v = -vv; *w = 0.0f; break;
    case 'L': *v = 0.0f; *w = ww;  break;
    case 'R': *v = 0.0f; *w = -ww; break;
    case 'G': *v = vv;  *w = tw;   break;   /* tiến + trái */
    case 'I': *v = vv;  *w = -tw;  break;   /* tiến + phải */
    case 'H': *v = -vv; *w = tw;   break;   /* lùi + trái */
    case 'J': *v = -vv; *w = -tw;  break;   /* lùi + phải */
    default:  *v = 0.0f; *w = 0.0f; break;
    }
}

void Rec_Track(Recorder *r, const Pose *pose, const NavParams *p)
{
    const NavPoint *last = &r->raw[r->n - 1];
    float dx = pose->x - last->x, dy = pose->y - last->y;
    if (dx * dx + dy * dy < p->rec_step_cm * p->rec_step_cm) return;
    if (r->n >= REC_MAX_RAW) return;
    r->raw[r->n].x = pose->x;
    r->raw[r->n].y = pose->y;
    r->n++;
}

/* Ramer-Douglas-Peucker không đệ quy: đánh dấu điểm cần giữ. */
static uint16_t rdp(const NavPoint *pts, uint16_t n, float eps, uint8_t *keep)
{
    static uint16_t stack[2 * REC_MAX_RAW];
    uint16_t top = 0, count = 0;
    for (uint16_t i = 0; i < n; i++) keep[i] = 0;
    keep[0] = keep[n - 1] = 1;
    stack[top++] = 0;
    stack[top++] = (uint16_t)(n - 1);
    while (top > 0) {
        uint16_t b = stack[--top], a = stack[--top];
        float best = 0.0f;
        uint16_t idx = 0;
        for (uint16_t i = (uint16_t)(a + 1); i < b; i++) {
            float d = Geo_PointSegDist(pts[i].x, pts[i].y, pts[a].x, pts[a].y, pts[b].x, pts[b].y, 0, 0);
            if (d > best) { best = d; idx = i; }
        }
        if (best > eps && top + 4 <= 2 * REC_MAX_RAW) {
            keep[idx] = 1;
            stack[top++] = a; stack[top++] = idx;
            stack[top++] = idx; stack[top++] = b;
        }
    }
    for (uint16_t i = 0; i < n; i++) count += keep[i];
    return count;
}

uint16_t Rec_Finish(Recorder *r, const NavParams *p, NavPoint *out, uint16_t max_out)
{
    static uint8_t keep[REC_MAX_RAW];
    if (r->n < 2) return 0;
    float eps = p->rec_epsilon_cm;
    while (rdp(r->raw, r->n, eps, keep) > max_out) eps *= 1.5f;  /* quá nhiều điểm thì rút gọn mạnh hơn */
    uint16_t k = 0;
    for (uint16_t i = 0; i < r->n; i++) {
        if (keep[i]) out[k++] = r->raw[i];
    }
    return k;
}
