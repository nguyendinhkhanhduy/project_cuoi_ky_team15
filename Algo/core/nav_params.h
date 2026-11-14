/* Thông số tinh chỉnh của thuật toán. Driver lưu struct này vào Flash và
 * cho chỉnh qua Bluetooth bằng NavParams_Set("ten", gia_tri). */
#ifndef NAV_PARAMS_H
#define NAV_PARAMS_H

#include <stdint.h>

#define NAV_TOF_COUNT 4
#define NAV_IR_COUNT  2
#define NAV_PARAMS_VERSION 4U

/* Hướng lắp 2 IR góc trước */
#define NAV_IR_TAPE    0U  /* chúc xuống sàn, báo băng đen: sửa vị trí (EKF) + phản xạ chống ra biên */
#define NAV_IR_FORWARD 1U  /* nhìn ngang ra trước (module dò vật cản): phanh khi có vật lạ rất gần */
#define NAV_IR_OFF     2U

typedef struct {
    uint32_t version;

    /* Odometry */
    float ticks_per_cm;      /* số xung encoder trên 1 cm quãng đường */
    float track_cm;          /* khoảng cách hai bánh */
    float gyro_weight;       /* 0..1: tỉ trọng gyro khi tính góc quay (phần còn lại lấy từ encoder) */

    /* Thân xe (tâm xe = điểm giữa trục bánh) */
    float body_len_cm;
    float body_wid_cm;

    /* Bám quỹ đạo (Pure Pursuit lookahead thay đổi) */
    float v_max;             /* cm/s */
    float v_min;             /* cm/s, tốc độ tối thiểu khi đang chạy */
    float a_max;             /* cm/s^2, gia tốc/giảm tốc dọc */
    float a_lat_max;         /* cm/s^2, gia tốc ngang khi vào cua */
    float w_max;             /* rad/s */
    float la_min;            /* cm, lookahead nhỏ nhất (dùng ở góc gắt) */
    float la_max;            /* cm, lookahead lớn nhất */
    float la_gain_s;         /* lookahead = la_gain_s * v trước khi xét góc cua */
    float turn_window_cm;    /* xét góc cua trong đoạn này phía trước */
    float turn_full_rad;     /* tổng góc cua >= mức này thì lookahead = la_min */
    float turn_slow;         /* 0..1: mức giảm tốc tối đa trước góc cua */
    float spin_angle_rad;    /* điểm đích lệch quá góc này thì quay tại chỗ */
    float finish_y_cm;       /* điểm cao nhất của thân xe xuống dưới y này là cả thân xe đã qua vạch đích */

    /* Vị trí cảm biến trong khung xe */
    float tof_x[NAV_TOF_COUNT];
    float tof_y[NAV_TOF_COUNT];
    float tof_ang_deg[NAV_TOF_COUNT];
    float tof_fov_deg;       /* góc nhìn toàn phần của một VL53L0X */
    float tof_max_cm;        /* tầm đo tin cậy */
    float ir_x[NAV_IR_COUNT];
    float ir_y[NAV_IR_COUNT];

    /* Sửa vị trí theo map: bộ lọc Kalman mở rộng (localize.c) */
    float init_sigma_cm;     /* độ bất định vị trí lúc đặt xe vào ô xuất phát */
    float init_sigma_rad;    /* độ bất định hướng lúc xuất phát */
    float odo_sigma_frac;    /* sai số quãng đường = tỉ lệ này x quãng đường (slide: 3%) */
    float turn_sigma_frac;   /* sai số góc quay = tỉ lệ này x góc quay (slide: 3%) */
    float gyro_drift_sigma;  /* rad/sqrt(s): trôi gyro */
    float tof_sigma_cm;      /* nhiễu một số đo VL53L0X (cộng thêm 2% khoảng cách) */
    float loc_gate_cm;       /* chỉ dùng số đo lệch map ít hơn mức này */
    float loc_range_cm;      /* chỉ dùng số đo gần hơn mức này */
    float ir_gate_cm;        /* chỉ sửa nếu pose ước lượng cách vạch dán gần nhất ít hơn mức này; 0 = tắt sửa bằng IR */
    uint8_t ir_mode;         /* NAV_IR_TAPE / NAV_IR_FORWARD / NAV_IR_OFF */
    float edge_back_cm_s;    /* phản xạ biên: tốc độ lùi */
    float edge_w;            /* phản xạ biên: vận tốc góc xoay ra xa vạch, rad/s */
    float edge_hold_s;       /* tiếp tục phản xạ thêm bấy nhiêu giây sau khi IR hết thấy vạch */
    uint8_t border_visible;  /* 1 nếu viền sân và vùng cấm là tường thật (VL53L0X thấy), 0 nếu chỉ là băng dính */

    /* Né vật cản lạ */
    float avoid_range_cm;    /* chỉ xét số đo gần hơn mức này */
    float avoid_margin_cm;   /* số đo gần hơn map dự đoán quá mức này thì nghi là vật lạ */
    float avoid_known_tol_cm;/* điểm chạm cách vật đã biết ít hơn mức này thì không phải vật lạ */
    uint8_t avoid_confirm;   /* số lần đo liên tiếp để xác nhận vật lạ */
    float r_hard_cm;         /* tâm xe phải cách vật cản ít nhất mức này (giống plan_path.py) */
    float r_soft_cm;         /* dưới mức này bắt đầu phạt chi phí A* */
    float r_tight_cm;        /* A* trên xe thử lại với mức này nếu r_hard không có đường (0 = tắt).
                              * >= nửa bề rộng xe + biên: đủ chạy thẳng qua khe, không đủ xoay tại chỗ */
    float avoid_rejoin_cm;   /* nhập lại quỹ đạo gốc sau vật cản bao xa */

    /* Lượt 1: lái tay */
    float teach_v_max;       /* cm/s ở tốc độ '9' */
    float teach_w;           /* rad/s khi quay tại chỗ */
    float teach_timeout_s;   /* không nhận lệnh quá lâu thì dừng (0 = tắt) */
    float rec_step_cm;       /* ghi một điểm mỗi khi đi được quãng này */
    float rec_epsilon_cm;    /* sai lệch cho phép khi rút gọn quỹ đạo (Ramer-Douglas-Peucker) */
} NavParams;

void NavParams_Default(NavParams *p);

/* Đặt một thông số theo tên, ví dụ "v_max", "tof_ang_deg1". Trả về 0 nếu thành công. */
int NavParams_Set(NavParams *p, const char *name, float value);
/* Đọc một thông số theo tên. Trả về 0 nếu có. */
int NavParams_Get(const NavParams *p, const char *name, float *value);
/* Liệt kê tên các thông số chỉnh được (để in qua Bluetooth). */
int NavParams_Count(void);
const char *NavParams_NameAt(int i);

#endif /* NAV_PARAMS_H */
