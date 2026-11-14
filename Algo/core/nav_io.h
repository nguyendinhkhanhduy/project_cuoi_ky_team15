/* Giao tiếp giữa thuật toán dẫn đường và phần driver.
 *
 * Driver gọi Nav_Step() ~200 lần/giây với số đo cảm biến, nhận lại (v, w)
 * rồi tự chuyển thành PWM bằng PID vận tốc từng bánh.
 * Toàn bộ thư mục Core/Algo không gọi HAL: biên dịch được cả trên STM32 lẫn PC (tools/sim).
 *
 * Hệ toạ độ: gốc ở góc dưới-trái sân, x sang phải, y lên trên, đơn vị cm.
 * Góc theta tính từ trục x, dương = quay trái (ngược chiều kim đồng hồ), đơn vị rad.
 * Khung xe: x hướng về mũi xe, y sang trái xe.
 */
#ifndef NAV_IO_H
#define NAV_IO_H

#include <stdint.h>
#include "nav_params.h"

#define NAV_IR_IGNORE_BELOW_Y 4.0f  /* cm: vạch xuất phát/đích ô cờ ở y = 0 cũng làm IR báo đen */

typedef struct {
    float x;
    float y;
} NavPoint;

typedef struct {
    float x;
    float y;
    float th;
} Pose;

typedef struct {
    float x0, y0, x1, y1;  /* hình chữ nhật thẳng trục */
    uint8_t visible;       /* 1 nếu VL53L0X nhìn thấy được (khối thật), 0 nếu chỉ là vạch dán */
} NavRect;

/* driver -> thuật toán, mỗi chu kỳ */
typedef struct {
    float    dt_s;                    /* thời gian thật từ lần gọi trước */
    int32_t  d_ticks_left;            /* số xung encoder thay đổi trong chu kỳ, tiến = dương */
    int32_t  d_ticks_right;
    float    gyro_z_rad_s;            /* đã trừ bias, dương = quay trái */
    uint16_t tof_mm[NAV_TOF_COUNT];   /* VL53L0X theo thứ tự trái -> phải; >= 8000 = ngoài tầm */
    uint8_t  tof_fresh;               /* bit i = 1 nếu tof_mm[i] là số đo mới trong chu kỳ này */
    uint8_t  ir_on_tape;              /* bit0 = IR trái, bit1 = IR phải; 1 = đang trên băng đen */
    char     manual_cmd;              /* ký tự lái tay mới nhận (Lượt 1), 0 nếu không có */
} NavInput;

typedef enum {
    NAV_IDLE = 0,
    NAV_RUN,       /* đang bám quỹ đạo */
    NAV_BRAKE,     /* thấy vật cản lạ chắn đường, đang phanh để lập đường vòng */
    NAV_AVOID,     /* đang đi đường vòng quanh vật cản lạ */
    NAV_TEACH,     /* Lượt 1: lái tay và ghi quỹ đạo */
    NAV_DONE,      /* cả thân xe đã qua vạch đích */
    NAV_STOPPED,   /* không tìm được đường an toàn: dừng hẳn */
    NAV_EDGE       /* IR chạm băng đen biên sân: lùi + xoay ra xa vạch rồi bám quỹ đạo tiếp */
} NavState;

typedef enum {
    NAV_MODE_RUN_ASTAR = 0,  /* bám quỹ đạo A* trong path.h */
    NAV_MODE_RUN_RECORDED,   /* bám quỹ đạo đã ghi ở Lượt 1 */
    NAV_MODE_TEACH           /* lái tay, ghi quỹ đạo */
} NavMode;

/* thuật toán -> driver */
typedef struct {
    float   v_cm_s;   /* vận tốc thẳng mong muốn */
    float   w_rad_s;  /* vận tốc góc mong muốn */
    NavState state;
} NavOutput;

typedef struct {
    float    s_cm;            /* vị trí dọc quỹ đạo đang bám */
    float    path_len_cm;
    float    lookahead_cm;
    NavPoint target;
    uint16_t unknown_count;   /* số vật cản lạ đã phát hiện */
    uint32_t tof_fixes;       /* số lần sửa vị trí bằng VL53L0X */
    uint32_t ir_fixes;        /* số lần sửa vị trí bằng IR */
    uint32_t edge_events;     /* số lần phản xạ biên sân (ir_mode = băng đen) */
} NavDebug;

/* path = NULL: dùng quỹ đạo A* trong path.h. Với NAV_MODE_RUN_RECORDED truyền quỹ đạo đọc từ Flash. */
void Nav_Init(const NavParams *params, NavMode mode, const NavPoint *path, uint16_t path_count);
NavOutput Nav_Step(const NavInput *in);
const Pose *Nav_GetPose(void);
const NavDebug *Nav_GetDebug(void);
NavState Nav_GetState(void);

/* Kết thúc Lượt 1: rút gọn quỹ đạo đã ghi vào out[], trả về số điểm. Driver lưu out[] vào Flash. */
uint16_t Nav_TeachFinish(NavPoint *out, uint16_t max_points);

#endif /* NAV_IO_H */
