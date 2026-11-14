/* Bám quỹ đạo bằng Pure Pursuit có lookahead thay đổi theo góc cua phía trước. */
#ifndef FOLLOWER_H
#define FOLLOWER_H

#include "nav_path.h"

typedef struct {
    float s;          /* vị trí dọc quỹ đạo (chỉ tăng) */
    float v;          /* vận tốc đã ra lệnh ở chu kỳ trước */
    float lookahead;
    NavPoint target;
    uint8_t done;
} Follower;

void Fol_Reset(Follower *f, float s0, float v0);
void Fol_Step(Follower *f, const NavPath *P, const Pose *pose, const NavParams *p, float dt, float *v_out,
              float *w_out);

#endif /* FOLLOWER_H */
