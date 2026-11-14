#include "nav_path.h"

#include <math.h>

#include "geometry.h"

void Path_Build(NavPath *P, const NavPoint *pts, uint16_t n)
{
    P->n = 0;
    for (uint16_t i = 0; i < n && P->n < NAV_MAX_PTS; i++) {
        if (P->n > 0) {
            const NavPoint *q = &P->p[P->n - 1];
            float d = sqrtf((pts[i].x - q->x) * (pts[i].x - q->x) + (pts[i].y - q->y) * (pts[i].y - q->y));
            if (d < 0.05f) continue;  /* bỏ điểm trùng để không có đoạn dài 0 */
            P->s[P->n] = P->s[P->n - 1] + d;
        } else {
            P->s[0] = 0.0f;
        }
        P->p[P->n++] = pts[i];
    }
}

float Path_Length(const NavPath *P)
{
    return (P->n > 0) ? P->s[P->n - 1] : 0.0f;
}

static uint16_t segment_at(const NavPath *P, float s)
{
    uint16_t i = 0;
    while (i + 2 < P->n && P->s[i + 1] < s) i++;
    return i;
}

NavPoint Path_At(const NavPath *P, float s)
{
    if (P->n == 0) return (NavPoint){0.0f, 0.0f};
    if (P->n == 1) return P->p[0];
    if (s <= 0.0f) return P->p[0];
    uint16_t i = segment_at(P, s);
    const NavPoint *a = &P->p[i], *b = &P->p[i + 1];
    float len = P->s[i + 1] - P->s[i];
    float t = (s - P->s[i]) / len;  /* > 1 khi kéo dài qua điểm cuối */
    NavPoint r = {a->x + t * (b->x - a->x), a->y + t * (b->y - a->y)};
    return r;
}

float Path_Project(const NavPath *P, float x, float y, float s_lo, float s_hi)
{
    if (P->n < 2) return 0.0f;
    float best_d = GEO_INF, best_s = s_lo;
    for (uint16_t i = 0; i + 1 < P->n; i++) {
        if (P->s[i + 1] < s_lo) continue;
        if (P->s[i] > s_hi) break;
        float qx, qy;
        float d = Geo_PointSegDist(x, y, P->p[i].x, P->p[i].y, P->p[i + 1].x, P->p[i + 1].y, &qx, &qy);
        if (d < best_d) {
            best_d = d;
            best_s = P->s[i] + sqrtf((qx - P->p[i].x) * (qx - P->p[i].x) + (qy - P->p[i].y) * (qy - P->p[i].y));
        }
    }
    if (best_s < s_lo) best_s = s_lo;
    return best_s;
}

static float seg_heading(const NavPath *P, uint16_t i)
{
    return atan2f(P->p[i + 1].y - P->p[i].y, P->p[i + 1].x - P->p[i].x);
}

float Path_TurnAhead(const NavPath *P, float s, float window)
{
    float total = 0.0f;
    for (uint16_t i = 1; i + 1 < P->n; i++) {
        if (P->s[i] <= s) continue;
        if (P->s[i] > s + window) break;
        total += fabsf(Geo_WrapPi(seg_heading(P, i) - seg_heading(P, (uint16_t)(i - 1))));
    }
    return total;
}

uint16_t Path_IndexAfter(const NavPath *P, float s)
{
    uint16_t i = 0;
    while (i < P->n && P->s[i] <= s) i++;
    return i;
}
