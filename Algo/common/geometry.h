/* Hình học 2D: khoảng cách, chiếu tia, va chạm thân xe với vật cản. */
#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "nav_io.h"

#define GEO_INF 1.0e9f
#define GEO_PI  3.14159265f

float Geo_WrapPi(float a);
float Geo_PointRectDist(float x, float y, const NavRect *r);  /* 0 nếu nằm trong */
float Geo_PointClearance(float x, float y, const NavRect *rects, uint16_t n);
float Geo_PointSegDist(float px, float py, float ax, float ay, float bx, float by, float *qx, float *qy);

/* Khoảng cách dọc tia từ (ox,oy) theo góc ang tới vật cản gần nhất, GEO_INF nếu không chạm.
 * only_visible = 1: bỏ qua vật VL53L0X không thấy (vạch dán). */
float Geo_RayCast(float ox, float oy, float ang, const NavRect *rects, uint16_t n, uint8_t only_visible);
/* Như trên nhưng lấy số nhỏ nhất trong nón +-half_fov (mô hình VL53L0X). */
float Geo_ConeCast(float ox, float oy, float ang, float half_fov, const NavRect *rects, uint16_t n,
                   uint8_t only_visible);

/* Khoảng cách nhỏ nhất từ thân xe (chữ nhật len x wid, tâm tại pose) tới các vật; 0 nếu chồng lên. */
float Geo_BodyClearance(const Pose *p, float len, float wid, const NavRect *rects, uint16_t n);
/* Khoảng cách nhỏ nhất từ các điểm trên đoạn AB (lấy mẫu mỗi step cm) tới vật cản. */
float Geo_SegmentClearance(float ax, float ay, float bx, float by, float step, const NavRect *rects,
                           uint16_t n);

/* Đổi điểm từ khung xe sang khung sân */
void Geo_ToWorld(const Pose *p, float lx, float ly, float *wx, float *wy);

#endif /* GEOMETRY_H */
