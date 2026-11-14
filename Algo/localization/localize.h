/* Sửa vị trí bằng bộ lọc Kalman mở rộng (EKF) trên trạng thái (x, y, theta):
 * - Dự đoán: theo odometry, độ bất định tăng dần theo quãng đường và góc quay.
 * - Cập nhật: mỗi số đo VL53L0X so với khoảng cách tính trên map; mỗi lần IR thấy băng đen.
 * Số đo lệch quá xa dự đoán bị loại (có thể là vật cản lạ). */
#ifndef LOCALIZE_H
#define LOCALIZE_H

#include "nav_io.h"

typedef struct {
    float P[3][3];        /* hiệp phương sai của (x, y, theta) */
    uint32_t tof_fixes;
    uint32_t ir_fixes;
} LocState;

void Loc_Init(LocState *s, const NavParams *p);
/* Gọi sau Odo_Update với quãng đường ds và góc quay dth của chu kỳ. */
void Loc_Predict(LocState *s, const Pose *pose, float ds, float dth, float dt, const NavParams *p);
void Loc_Update(LocState *s, Pose *pose, const NavInput *in, const NavParams *p, const NavRect *rects,
                uint16_t n);

/* Vị trí và hướng của VL53L0X thứ i trong khung sân */
void Loc_TofPose(const Pose *pose, const NavParams *p, int i, float *sx, float *sy, float *ang);

#endif /* LOCALIZE_H */
