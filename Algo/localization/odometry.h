/* Ước lượng vị trí (x, y, theta) từ encoder và gyro. */
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "nav_io.h"

/* Cập nhật pose; trả về quãng đường ds và góc quay dth của chu kỳ (cho bước dự đoán EKF). */
void Odo_Update(Pose *pose, const NavInput *in, const NavParams *p, float *ds_out, float *dth_out);

#endif /* ODOMETRY_H */
