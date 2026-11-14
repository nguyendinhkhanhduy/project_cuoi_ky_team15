/* Phát hiện vật cản lạ (không có trong map) và lập đường vòng quanh nó. */
#ifndef AVOID_H
#define AVOID_H

#include "nav_path.h"

typedef struct {
    uint8_t confirm[NAV_TOF_COUNT];
    uint8_t ir_confirm[NAV_IR_COUNT];
} Avoid;

void Avoid_Reset(Avoid *a);

/* 1 nếu vừa xác nhận một vật lạ; ghi hình vuông 10x10 ước lượng vào *found.
 * heading_sigma: độ lệch chuẩn hướng của EKF (rad), dùng để nới nón khi so với map. */
int Avoid_Detect(Avoid *a, const Pose *pose, const NavInput *in, const NavParams *p, const NavRect *rects,
                 uint16_t n, float heading_sigma, NavRect *found);

/* ir_mode = NAV_IR_FORWARD: IR góc trước báo có vật rất gần mà map không giải thích được -> vật lạ. */
int Avoid_DetectIr(Avoid *a, const Pose *pose, const NavInput *in, const NavParams *p, const NavRect *rects,
                   uint16_t n, NavRect *found);

/* Vật lạ có chắn quỹ đạo phía trước không (trong khoảng [s, s + 80 cm]). */
int Avoid_Blocks(const NavPath *P, float s, const NavRect *obstacle, const NavParams *p);

/* Lập quỹ đạo mới: từ vị trí hiện tại vòng qua vật lạ rồi nối vào phần còn lại của P.
 * Trả về số điểm ghi vào out (0 nếu không có đường), *s_rejoin = độ dài cung trên quỹ đạo mới
 * tại điểm nhập lại. */
uint16_t Avoid_Plan(const NavPath *P, float s, const Pose *pose, const NavRect *obstacle, const NavParams *p,
                    const NavRect *rects, uint16_t n, NavPoint *out, uint16_t max_out, float *s_rejoin);

#endif /* AVOID_H */
