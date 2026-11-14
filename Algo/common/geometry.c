#include "geometry.h"

#include <math.h>

#define CONE_RAYS 5

float Geo_WrapPi(float a)
{
    while (a > GEO_PI) a -= 2.0f * GEO_PI;
    while (a < -GEO_PI) a += 2.0f * GEO_PI;
    return a;
}

float Geo_PointRectDist(float x, float y, const NavRect *r)
{
    float dx = fmaxf(fmaxf(r->x0 - x, 0.0f), x - r->x1);
    float dy = fmaxf(fmaxf(r->y0 - y, 0.0f), y - r->y1);
    return sqrtf(dx * dx + dy * dy);
}

float Geo_PointClearance(float x, float y, const NavRect *rects, uint16_t n)
{
    float best = GEO_INF;
    for (uint16_t i = 0; i < n; i++) {
        float d = Geo_PointRectDist(x, y, &rects[i]);
        if (d < best) best = d;
    }
    return best;
}

float Geo_PointSegDist(float px, float py, float ax, float ay, float bx, float by, float *qx, float *qy)
{
    float vx = bx - ax, vy = by - ay;
    float len2 = vx * vx + vy * vy;
    float t = (len2 > 1e-9f) ? ((px - ax) * vx + (py - ay) * vy) / len2 : 0.0f;
    t = fminf(fmaxf(t, 0.0f), 1.0f);
    float cx = ax + t * vx, cy = ay + t * vy;
    if (qx) *qx = cx;
    if (qy) *qy = cy;
    return sqrtf((px - cx) * (px - cx) + (py - cy) * (py - cy));
}

/* Giao tia với hình chữ nhật thẳng trục (phương pháp slab). */
static float ray_rect(float ox, float oy, float dx, float dy, const NavRect *r)
{
    float tmin = -GEO_INF, tmax = GEO_INF;
    const float o[2] = {ox, oy}, d[2] = {dx, dy};
    const float lo[2] = {r->x0, r->y0}, hi[2] = {r->x1, r->y1};
    for (int k = 0; k < 2; k++) {
        if (fabsf(d[k]) < 1e-9f) {
            if (o[k] < lo[k] || o[k] > hi[k]) return GEO_INF;
        } else {
            float t1 = (lo[k] - o[k]) / d[k];
            float t2 = (hi[k] - o[k]) / d[k];
            if (t1 > t2) { float t = t1; t1 = t2; t2 = t; }
            if (t1 > tmin) tmin = t1;
            if (t2 < tmax) tmax = t2;
            if (tmin > tmax) return GEO_INF;
        }
    }
    if (tmax < 0.0f) return GEO_INF;
    return (tmin >= 0.0f) ? tmin : 0.0f;
}

float Geo_RayCast(float ox, float oy, float ang, const NavRect *rects, uint16_t n, uint8_t only_visible)
{
    float dx = cosf(ang), dy = sinf(ang);
    float best = GEO_INF;
    for (uint16_t i = 0; i < n; i++) {
        if (only_visible && !rects[i].visible) continue;
        float t = ray_rect(ox, oy, dx, dy, &rects[i]);
        if (t < best) best = t;
    }
    return best;
}

float Geo_ConeCast(float ox, float oy, float ang, float half_fov, const NavRect *rects, uint16_t n,
                   uint8_t only_visible)
{
    float best = GEO_INF;
    for (int k = 0; k < CONE_RAYS; k++) {
        float a = ang - half_fov + 2.0f * half_fov * (float)k / (float)(CONE_RAYS - 1);
        float t = Geo_RayCast(ox, oy, a, rects, n, only_visible);
        if (t < best) best = t;
    }
    return best;
}

void Geo_ToWorld(const Pose *p, float lx, float ly, float *wx, float *wy)
{
    float c = cosf(p->th), s = sinf(p->th);
    *wx = p->x + c * lx - s * ly;
    *wy = p->y + s * lx + c * ly;
}

/* Thân xe (hình chữ nhật xoay) có chồng lên hình chữ nhật thẳng trục không: định lý trục phân tách. */
static int body_overlaps(const Pose *p, float hx, float hy, float c, float s, const NavRect *r)
{
    float rcx = 0.5f * (r->x0 + r->x1), rcy = 0.5f * (r->y0 + r->y1);
    float rw = 0.5f * (r->x1 - r->x0), rh = 0.5f * (r->y1 - r->y0);
    float dx = p->x - rcx, dy = p->y - rcy;
    const float axes[4][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}, {c, s}, {-s, c}};
    for (int k = 0; k < 4; k++) {
        float ax = axes[k][0], ay = axes[k][1];
        float rb = hx * fabsf(c * ax + s * ay) + hy * fabsf(-s * ax + c * ay);
        float rr = rw * fabsf(ax) + rh * fabsf(ay);
        if (fabsf(dx * ax + dy * ay) >= rb + rr) return 0;
    }
    return 1;
}

float Geo_BodyClearance(const Pose *p, float len, float wid, const NavRect *rects, uint16_t n)
{
    float hx = 0.5f * len, hy = 0.5f * wid;
    float c = cosf(p->th), s = sinf(p->th);
    float corner[4][2];
    const float sx[4] = {1, 1, -1, -1}, sy[4] = {1, -1, -1, 1};
    for (int k = 0; k < 4; k++) {
        corner[k][0] = p->x + c * sx[k] * hx - s * sy[k] * hy;
        corner[k][1] = p->y + s * sx[k] * hx + c * sy[k] * hy;
    }

    float best = GEO_INF;
    for (uint16_t i = 0; i < n; i++) {
        const NavRect *r = &rects[i];
        if (body_overlaps(p, hx, hy, c, s, r)) return 0.0f;
        /* Hai đa giác lồi rời nhau: khoảng cách nhỏ nhất đạt tại đỉnh của hình này tới hình kia. */
        for (int k = 0; k < 4; k++) {
            float d = Geo_PointRectDist(corner[k][0], corner[k][1], r);
            if (d < best) best = d;
        }
        const float rx[4] = {r->x0, r->x1, r->x1, r->x0}, ry[4] = {r->y0, r->y0, r->y1, r->y1};
        for (int k = 0; k < 4; k++) {
            float wx = rx[k] - p->x, wy = ry[k] - p->y;
            float lx = c * wx + s * wy, ly = -s * wx + c * wy;
            float ex = fmaxf(fabsf(lx) - hx, 0.0f), ey = fmaxf(fabsf(ly) - hy, 0.0f);
            float d = sqrtf(ex * ex + ey * ey);
            if (d < best) best = d;
        }
    }
    return best;
}

float Geo_SegmentClearance(float ax, float ay, float bx, float by, float step, const NavRect *rects,
                           uint16_t n)
{
    float len = sqrtf((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
    int k = (int)(len / step) + 1;
    float best = GEO_INF;
    for (int i = 0; i <= k; i++) {
        float t = (float)i / (float)k;
        float d = Geo_PointClearance(ax + t * (bx - ax), ay + t * (by - ay), rects, n);
        if (d < best) best = d;
    }
    return best;
}
