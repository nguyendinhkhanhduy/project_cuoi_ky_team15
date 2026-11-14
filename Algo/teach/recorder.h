/* Lượt 1: đổi ký tự lái tay thành (v, w) và ghi lại quỹ đạo đã đi. */
#ifndef RECORDER_H
#define RECORDER_H

#include "nav_io.h"

#define REC_MAX_RAW 500

typedef struct {
    NavPoint raw[REC_MAX_RAW];
    uint16_t n;
    char move;          /* lệnh di chuyển hiện tại */
    uint8_t level;      /* tốc độ 0..10 */
    float since_cmd_s;  /* thời gian từ lệnh cuối */
} Recorder;

void Rec_Init(Recorder *r, const Pose *start);
/* Ký tự theo app "Bluetooth RC Controller": F B L R G I H J S, 0..9, q (tối đa). */
void Rec_Command(Recorder *r, const NavInput *in, const NavParams *p, float *v, float *w);
void Rec_Track(Recorder *r, const Pose *pose, const NavParams *p);
/* Rút gọn bằng Ramer-Douglas-Peucker, trả về số điểm ghi vào out. */
uint16_t Rec_Finish(Recorder *r, const NavParams *p, NavPoint *out, uint16_t max_out);

#endif /* RECORDER_H */
