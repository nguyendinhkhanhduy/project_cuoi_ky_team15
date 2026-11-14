/* Map sân: 17 khối + vùng cấm (từ path.h), viền sân, và vật cản lạ phát hiện lúc chạy. */
#ifndef NAV_MAP_H
#define NAV_MAP_H

#include "nav_io.h"

#define NAV_MAX_UNKNOWN 3

void NavMap_Init(uint8_t border_visible);
void NavMap_ClearUnknown(void);
int NavMap_AddUnknown(const NavRect *r);          /* luôn thêm được; đầy thì ghi đè vật lạ cũ nhất */
uint16_t NavMap_UnknownCount(void);

const NavRect *NavMap_Rects(uint16_t *count);     /* mọi hình chữ nhật cần tránh */
/* Chỉ map tĩnh (17 khối, vùng cấm, viền): EKF chỉ định vị theo vật đã biết, vì vị trí vật lạ
 * cũng chỉ là ước lượng. Là phần đầu của mảng NavMap_Rects(). */
const NavRect *NavMap_KnownRects(uint16_t *count);
const NavPoint *NavMap_Path(uint16_t *count);     /* quỹ đạo A* tính sẵn */
float NavMap_StartHeading(void);                  /* rad */

/* Các đoạn băng đen viền hành lang chữ U (dùng cho IR) */
typedef struct {
    float x0, y0, x1, y1;
} NavSegment;
const NavSegment *NavMap_Tape(uint16_t *count);

#endif /* NAV_MAP_H */
