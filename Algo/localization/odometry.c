#include "odometry.h"

#include <math.h>

#include "geometry.h"

void Odo_Update(Pose *pose, const NavInput *in, const NavParams *p, float *ds_out, float *dth_out)
{
    float dl = (float)in->d_ticks_left / p->ticks_per_cm;
    float dr = (float)in->d_ticks_right / p->ticks_per_cm;
    float ds = 0.5f * (dl + dr);

    /* Góc quay: chủ yếu tin gyro (không bị trượt bánh), trộn một ít encoder để giảm trôi. */
    float dth_enc = (dr - dl) / p->track_cm;
    float dth_gyro = in->gyro_z_rad_s * in->dt_s;
    float dth = p->gyro_weight * dth_gyro + (1.0f - p->gyro_weight) * dth_enc;

    float th_mid = pose->th + 0.5f * dth;
    pose->x += ds * cosf(th_mid);
    pose->y += ds * sinf(th_mid);
    pose->th = Geo_WrapPi(pose->th + dth);
    if (ds_out) *ds_out = ds;
    if (dth_out) *dth_out = dth;
}
