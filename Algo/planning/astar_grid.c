#include "astar_grid.h"

#include <math.h>

#include "geometry.h"

#define AG_N          44         /* 44 x 44 ô: lưới 2 cm -> cửa sổ 88 cm, lưới 3 cm -> 132 cm */
#define AG_CELLS      (AG_N * AG_N)
#define AG_UNSET      0xFFFFU
#define AG_CLOSED     0x80U
#define AG_NO_PAR     0x7FU
#define FIELD_W       180.0f
#define FIELD_H       120.0f
#define FINISH_X_MIN  120.0f     /* vạch đích ở y = 0, x 120..180: chỉ đoạn này được đi xuống y < 0 */
#define BELOW_CM      20.0f      /* cửa sổ được thò xuống dưới vạch đích bấy nhiêu */
#define W_CLEAR       4.0f       /* giống plan_path.py */
#define START_FREE_CM 4.0f       /* cho phép đi qua ô hơi sát vật trong bán kính này quanh điểm đầu */
#define RAW_MAX       128
#define DENSE_MAX     128
#define DENSE_STEP_CM 2.5f
#define CENTER_SPAN_CM 4.0f      /* dời điểm sang ngang tối đa bấy nhiêu để vào giữa khe */
#define CENTER_STEP_CM 0.5f
#define CENTER_SKIP   2          /* không dời 2 điểm đầu/cuối: tránh gấp khúc ở chỗ nối với quỹ đạo */
#define SMOOTH_CLEAR_CM 9.5f     /* = R_SMOOTH của plan_path.py: nối tắt không được sát hơn mức này
                                  * (trừ khi đường A* vốn đã sát hơn). r_hard 7,5 chỉ hơn nửa đường chéo
                                  * thân xe 0,8 cm, không đủ cho sai số bám quỹ đạo. */

static const float RES_TRY[] = {2.0f, 3.0f};  /* thử lưới mịn trước, không có đường thì cửa sổ rộng hơn */

static uint16_t g_cost[AG_CELLS];   /* chi phí từ điểm đầu, đơn vị 0.1 cm */
static uint8_t  state[AG_CELLS];    /* bit7 = đã đóng, bit0..2 = hướng tới ô cha */
static uint8_t  clear_mm[AG_CELLS]; /* khoảng cách tới vật cản, mm, tối đa 255 */
static uint16_t heap[AG_CELLS];
static uint16_t heap_pos[AG_CELLS];
static uint16_t heap_n;
static NavPoint dense[DENSE_MAX];
static float    dense_clr[DENSE_MAX];

static float res;         /* cm mỗi ô của lần lập đường hiện tại */
static float ox, oy;      /* góc dưới-trái cửa sổ */
static int goal_ix, goal_iy;

static const int8_t DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
static const int8_t DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};

static float cell_x(int ix) { return ox + ((float)ix + 0.5f) * res; }
static float cell_y(int iy) { return oy + ((float)iy + 0.5f) * res; }

/* Khoảng cách tới vật cản; dưới y = 0 ngoài đoạn vạch đích là ngoài sân (coi như chạm). */
static float clearance(float x, float y, const NavRect *rects, uint16_t n)
{
    if (y < 0.0f && x < FINISH_X_MIN) return 0.0f;
    return Geo_PointClearance(x, y, rects, n);
}

static uint16_t heuristic(uint16_t idx)
{
    float dx = (float)(idx % AG_N - goal_ix), dy = (float)(idx / AG_N - goal_iy);
    return (uint16_t)(sqrtf(dx * dx + dy * dy) * res * 10.0f);
}

static uint32_t f_of(uint16_t idx) { return (uint32_t)g_cost[idx] + heuristic(idx); }

static void heap_swap(uint16_t a, uint16_t b)
{
    uint16_t t = heap[a];
    heap[a] = heap[b];
    heap[b] = t;
    heap_pos[heap[a]] = a;
    heap_pos[heap[b]] = b;
}

static void heap_up(uint16_t i)
{
    while (i > 0) {
        uint16_t parent = (uint16_t)((i - 1) / 2);
        if (f_of(heap[parent]) <= f_of(heap[i])) break;
        heap_swap(i, parent);
        i = parent;
    }
}

static void heap_down(uint16_t i)
{
    for (;;) {
        uint16_t l = (uint16_t)(2 * i + 1), r = (uint16_t)(2 * i + 2), m = i;
        if (l < heap_n && f_of(heap[l]) < f_of(heap[m])) m = l;
        if (r < heap_n && f_of(heap[r]) < f_of(heap[m])) m = r;
        if (m == i) break;
        heap_swap(i, m);
        i = m;
    }
}

static void heap_push_or_update(uint16_t idx)
{
    if (heap_pos[idx] == AG_UNSET) {
        heap[heap_n] = idx;
        heap_pos[idx] = heap_n;
        heap_n++;
    }
    heap_up(heap_pos[idx]);
}

static uint16_t heap_pop(void)
{
    uint16_t top = heap[0];
    heap_n--;
    if (heap_n > 0) {
        heap[0] = heap[heap_n];
        heap_pos[heap[0]] = 0;
        heap_down(0);
    }
    heap_pos[top] = AG_UNSET;
    return top;
}

static float cell_cost(uint16_t idx, float r_hard, float r_soft)
{
    float d = 0.1f * (float)clear_mm[idx];
    if (d >= r_soft) return 1.0f;
    float k = (r_soft - d) / (r_soft - r_hard);
    return 1.0f + W_CLEAR * k * k;
}

static float seg_clearance(NavPoint a, NavPoint b, const NavRect *rects, uint16_t n)
{
    float len = sqrtf((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
    int k = (int)(len / 0.5f) + 1;
    float best = GEO_INF;
    for (int i = 0; i <= k; i++) {
        float t = (float)i / (float)k;
        float d = clearance(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), rects, n);
        if (d < best) best = d;
    }
    return best;
}

/* A* trên lưới res cm. Ghi các điểm đổi hướng (start -> goal) vào raw, trả về số điểm, 0 nếu không có. */
static uint16_t plan_once(float res_cm, NavPoint start, NavPoint goal, const NavRect *rects, uint16_t n,
                          float r_hard, float r_soft, NavPoint *raw)
{
    res = res_cm;
    const float span = AG_N * res;
    ox = fminf(fmaxf(0.5f * (start.x + goal.x) - 0.5f * span, 0.0f), FIELD_W - span);
    oy = fminf(fmaxf(0.5f * (start.y + goal.y) - 0.5f * span, -BELOW_CM), FIELD_H - span);

    int sx = (int)floorf((start.x - ox) / res), sy = (int)floorf((start.y - oy) / res);
    goal_ix = (int)floorf((goal.x - ox) / res);
    goal_iy = (int)floorf((goal.y - oy) / res);
    if (sx < 0 || sy < 0 || sx >= AG_N || sy >= AG_N) return 0;
    if (goal_ix < 0 || goal_iy < 0 || goal_ix >= AG_N || goal_iy >= AG_N) return 0;

    for (uint16_t i = 0; i < AG_CELLS; i++) {
        float d = clearance(cell_x(i % AG_N), cell_y(i / AG_N), rects, n);
        clear_mm[i] = (uint8_t)fminf(d * 10.0f, 255.0f);
        g_cost[i] = AG_UNSET;
        state[i] = AG_NO_PAR;
        heap_pos[i] = AG_UNSET;
    }
    const uint16_t hard_mm = (uint16_t)(r_hard * 10.0f);
    uint16_t s_idx = (uint16_t)(sy * AG_N + sx), g_idx = (uint16_t)(goal_iy * AG_N + goal_ix);
    heap_n = 0;
    g_cost[s_idx] = 0;
    heap_push_or_update(s_idx);

    int found = 0;
    while (heap_n > 0) {
        uint16_t cur = heap_pop();
        if (cur == g_idx) { found = 1; break; }
        state[cur] |= AG_CLOSED;
        int cx = cur % AG_N, cy = cur / AG_N;
        for (int k = 0; k < 8; k++) {
            int nx = cx + DX[k], ny = cy + DY[k];
            if (nx < 0 || ny < 0 || nx >= AG_N || ny >= AG_N) continue;
            uint16_t nb = (uint16_t)(ny * AG_N + nx);
            if (state[nb] & AG_CLOSED) continue;
            if (clear_mm[nb] < hard_mm && nb != g_idx) {
                float ddx = cell_x(nx) - start.x, ddy = cell_y(ny) - start.y;
                int near_start = (ddx * ddx + ddy * ddy) < START_FREE_CM * START_FREE_CM;
                if (!near_start || clear_mm[nb] < clear_mm[s_idx]) continue;
            }
            float step = (k < 4) ? res : res * 1.41421356f;
            float c = step * 0.5f * (cell_cost(cur, r_hard, r_soft) + cell_cost(nb, r_hard, r_soft));
            uint32_t ng = (uint32_t)g_cost[cur] + (uint32_t)(c * 10.0f);
            if (ng < g_cost[nb] && ng < AG_UNSET) {
                g_cost[nb] = (uint16_t)ng;
                state[nb] = (uint8_t)k;  /* đến nb bằng bước k */
                heap_push_or_update(nb);
            }
        }
    }
    if (!found) return 0;

    /* Dựng lại đường từ đích về đầu, chỉ giữ điểm đổi hướng. */
    uint16_t nraw = 0;
    uint16_t cur = g_idx;
    int last_k = -1;
    raw[nraw++] = goal;
    while (cur != s_idx) {
        int k = state[cur] & 0x07;
        int cx = cur % AG_N - DX[k], cy = cur / AG_N - DY[k];
        if (k != last_k) {
            if (nraw >= RAW_MAX - 1) return 0;  /* bỏ điểm sẽ nối tắt xuyên vật cản: thà thử lưới khác */
            raw[nraw].x = cell_x(cur % AG_N);
            raw[nraw].y = cell_y(cur / AG_N);
            nraw++;
        }
        last_k = k;
        cur = (uint16_t)(cy * AG_N + cx);
    }
    raw[nraw++] = start;
    for (uint16_t i = 0; i < nraw / 2; i++) {
        NavPoint t = raw[i];
        raw[i] = raw[nraw - 1 - i];
        raw[nraw - 1 - i] = t;
    }
    return nraw;
}

/* Lấy mẫu dày đường gấp khúc rồi dời từng điểm theo phương vuông góc tới chỗ xa vật cản nhất
 * (giữa khe hẹp). Ô lưới hiếm khi rơi đúng giữa khe nên bước này lấy lại phần khoảng cách bị mất. */
static uint16_t densify_and_center(const NavPoint *raw, uint16_t nraw, const NavRect *rects, uint16_t n)
{
    uint16_t m = 0;
    dense[m++] = raw[0];
    for (uint16_t i = 0; i + 1 < nraw; i++) {
        float dx = raw[i + 1].x - raw[i].x, dy = raw[i + 1].y - raw[i].y;
        int k = (int)ceilf(sqrtf(dx * dx + dy * dy) / DENSE_STEP_CM);
        if (k < 1) k = 1;
        for (int j = 1; j <= k; j++) {
            if (m >= DENSE_MAX) return 0;
            float t = (float)j / (float)k;
            dense[m].x = raw[i].x + t * dx;
            dense[m].y = raw[i].y + t * dy;
            m++;
        }
    }
    for (uint16_t i = 0; i < m; i++) dense_clr[i] = clearance(dense[i].x, dense[i].y, rects, n);

    for (uint16_t i = CENTER_SKIP; i + CENTER_SKIP < m; i++) {
        float tx = dense[i + 1].x - dense[i - 1].x, ty = dense[i + 1].y - dense[i - 1].y;
        float len = sqrtf(tx * tx + ty * ty);
        if (len < 1e-3f) continue;
        float px = -ty / len, py = tx / len;
        NavPoint best = dense[i];
        float best_c = dense_clr[i];
        for (float off = -CENTER_SPAN_CM; off <= CENTER_SPAN_CM + 1e-3f; off += CENTER_STEP_CM) {
            NavPoint q = {dense[i].x + off * px, dense[i].y + off * py};
            float c = clearance(q.x, q.y, rects, n);
            if (c > best_c + 0.05f) { best = q; best_c = c; }
        }
        /* Chỉ nhận nếu hai đoạn nối với điểm trước/sau không sát hơn trước. */
        float need = fminf(dense_clr[i], fminf(dense_clr[i - 1], dense_clr[i + 1])) - 0.05f;
        if (best_c > dense_clr[i] && seg_clearance(dense[i - 1], best, rects, n) >= need &&
            seg_clearance(best, dense[i + 1], rects, n) >= need) {
            dense[i] = best;
            dense_clr[i] = best_c;
        }
    }
    return m;
}

/* Nối tắt tham lam như plan_path.py: chỉ bỏ điểm giữa nếu đoạn thẳng không sát vật hơn
 * các điểm nó thay thế (tối đa SMOOTH_CLEAR_CM). Giữ được khoảng cách vừa lấy lại ở bước căn giữa. */
static uint16_t shortcut(uint16_t m, const NavRect *rects, uint16_t n, NavPoint *out, uint16_t max_out)
{
    uint16_t k = 0, i = 0;
    out[k++] = dense[0];
    while (i + 1 < m) {
        uint16_t j = (uint16_t)(m - 1);
        while (j > i + 1) {
            float need = SMOOTH_CLEAR_CM;
            for (uint16_t q = i; q <= j; q++) need = fminf(need, dense_clr[q]);
            if (seg_clearance(dense[i], dense[j], rects, n) >= need - 0.2f) break;
            j--;
        }
        if (k >= max_out) return 0;
        out[k++] = dense[j];
        i = j;
    }
    return k;
}

uint16_t Astar_Plan(NavPoint start, NavPoint goal, const NavRect *rects, uint16_t n, float r_hard,
                    float r_soft, NavPoint *out, uint16_t max_out)
{
    static NavPoint raw[RAW_MAX];
    for (unsigned t = 0; t < sizeof(RES_TRY) / sizeof(RES_TRY[0]); t++) {
        uint16_t nraw = plan_once(RES_TRY[t], start, goal, rects, n, r_hard, r_soft, raw);
        if (nraw < 2) continue;
        uint16_t m = densify_and_center(raw, nraw, rects, n);
        if (m < 2) continue;
        uint16_t k = shortcut(m, rects, n, out, max_out);
        if (k >= 2) return k;
    }
    return 0;
}
