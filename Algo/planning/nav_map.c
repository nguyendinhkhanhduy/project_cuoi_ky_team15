#include "nav_map.h"

#include "path.h"  /* mảng static const: chỉ include ở file này */

#define DEG2RAD 0.017453293f
#define BORDER_COUNT 3U
#define MAX_RECTS (MAP_OBSTACLE_COUNT + BORDER_COUNT + NAV_MAX_UNKNOWN)

static NavRect rects[MAX_RECTS];
static uint16_t static_count;
static uint16_t rect_count;
static uint16_t next_unknown;  /* khi đầy: ô vật lạ sẽ bị ghi đè tiếp theo (cũ nhất) */
static NavPoint path_pts[PATH_POINT_COUNT];

/* Viền chữ U: tường trái, phải, trên và ba cạnh vùng cấm. Cạnh dưới là vạch xuất phát/đích. */
static const NavSegment tape[] = {
    {0.0f, 0.0f, 0.0f, MAP_FIELD_H_CM},
    {MAP_FIELD_W_CM, 0.0f, MAP_FIELD_W_CM, MAP_FIELD_H_CM},
    {0.0f, MAP_FIELD_H_CM, MAP_FIELD_W_CM, MAP_FIELD_H_CM},
    {60.0f, 0.0f, 60.0f, 60.0f},
    {60.0f, 60.0f, 120.0f, 60.0f},
    {120.0f, 60.0f, 120.0f, 0.0f},
};

void NavMap_Init(uint8_t border_visible)
{
    uint16_t n = 0;
    for (uint16_t i = 0; i < MAP_OBSTACLE_COUNT; i++) {
        rects[n].x0 = MAP_OBSTACLES[i].x_min;
        rects[n].y0 = MAP_OBSTACLES[i].y_min;
        rects[n].x1 = MAP_OBSTACLES[i].x_max;
        rects[n].y1 = MAP_OBSTACLES[i].y_max;
        rects[n].visible = (i < MAP_BLOCK_COUNT) ? 1U : border_visible;
        n++;
    }
    /* Viền sân dạng dải dày bên ngoài; cạnh dưới để mở. */
    const NavRect border[BORDER_COUNT] = {
        {-20.0f, -40.0f, 0.0f, MAP_FIELD_H_CM + 20.0f, 0},
        {MAP_FIELD_W_CM, -40.0f, MAP_FIELD_W_CM + 20.0f, MAP_FIELD_H_CM + 20.0f, 0},
        {-20.0f, MAP_FIELD_H_CM, MAP_FIELD_W_CM + 20.0f, MAP_FIELD_H_CM + 20.0f, 0},
    };
    for (uint16_t i = 0; i < BORDER_COUNT; i++) {
        rects[n] = border[i];
        rects[n].visible = border_visible;
        n++;
    }
    static_count = n;
    rect_count = n;

    for (uint16_t i = 0; i < PATH_POINT_COUNT; i++) {
        path_pts[i].x = PATH_POINTS[i].x_cm;
        path_pts[i].y = PATH_POINTS[i].y_cm;
    }
}

void NavMap_ClearUnknown(void)
{
    rect_count = static_count;
    next_unknown = 0;
}

int NavMap_AddUnknown(const NavRect *r)
{
    if (rect_count < static_count + NAV_MAX_UNKNOWN) {
        rects[rect_count++] = *r;
        return 0;
    }
    /* Đầy (thường do báo nhầm): ghi đè vật cũ nhất, để vật thật phát hiện sau vẫn được né. */
    rects[static_count + next_unknown] = *r;
    next_unknown = (uint16_t)((next_unknown + 1U) % NAV_MAX_UNKNOWN);
    return 0;
}

uint16_t NavMap_UnknownCount(void)
{
    return (uint16_t)(rect_count - static_count);
}

const NavRect *NavMap_Rects(uint16_t *count)
{
    *count = rect_count;
    return rects;
}

const NavRect *NavMap_KnownRects(uint16_t *count)
{
    *count = static_count;
    return rects;
}

const NavPoint *NavMap_Path(uint16_t *count)
{
    *count = PATH_POINT_COUNT;
    return path_pts;
}

float NavMap_StartHeading(void)
{
    return PATH_START_HEADING_DEG * DEG2RAD;
}

const NavSegment *NavMap_Tape(uint16_t *count)
{
    *count = (uint16_t)(sizeof(tape) / sizeof(tape[0]));
    return tape;
}
