/* A* chạy trên xe để lập đường vòng quanh vật cản lạ.
 * Lưới 44 x 44 ô quanh điểm đầu/đích: thử 2 cm (cửa sổ 88 cm) rồi 3 cm (132 cm), ~16 KB RAM tĩnh.
 * Đường tìm được được lấy mẫu dày, dời điểm vào giữa khe hẹp, rồi nối tắt. */
#ifndef ASTAR_GRID_H
#define ASTAR_GRID_H

#include "nav_io.h"

/* Trả về số điểm của đường đã làm mượt (gồm cả start và goal), 0 nếu không có đường. */
uint16_t Astar_Plan(NavPoint start, NavPoint goal, const NavRect *rects, uint16_t n, float r_hard,
                    float r_soft, NavPoint *out, uint16_t max_out);

#endif /* ASTAR_GRID_H */
