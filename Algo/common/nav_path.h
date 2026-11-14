/* Quỹ đạo dạng đường gấp khúc, có độ dài cung tích luỹ để tra cứu theo s (cm). */
#ifndef NAV_PATH_H
#define NAV_PATH_H

#include "nav_io.h"

#define NAV_MAX_PTS 300

typedef struct {
    NavPoint p[NAV_MAX_PTS];
    float s[NAV_MAX_PTS];  /* độ dài cung từ điểm đầu tới điểm i */
    uint16_t n;
} NavPath;

void Path_Build(NavPath *P, const NavPoint *pts, uint16_t n);
float Path_Length(const NavPath *P);
/* Điểm tại độ dài cung s; s vượt cuối thì kéo dài theo hướng đoạn cuối. */
NavPoint Path_At(const NavPath *P, float s);
/* s của điểm trên quỹ đạo gần (x,y) nhất, chỉ xét đoạn [s_lo, s_hi]. */
float Path_Project(const NavPath *P, float x, float y, float s_lo, float s_hi);
/* Tổng |góc đổi hướng| tại các đỉnh nằm trong (s, s + window]. */
float Path_TurnAhead(const NavPath *P, float s, float window);
/* Chỉ số điểm đầu tiên có độ dài cung > s. */
uint16_t Path_IndexAfter(const NavPath *P, float s);

#endif /* NAV_PATH_H */
