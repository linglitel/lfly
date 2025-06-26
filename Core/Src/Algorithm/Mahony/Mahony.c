#include <math.h>

#include "AlgorithmUtils.h"
#include "Mahony.h"

static float twoKp = 2.0f * 0.5f;  // 2 * proportional gain
static float twoKi = 2.0f * 0.0f;  // 2 * integral gain，设为 0 禁用积分

static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

void MahonyUpdateIMU(
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float* q0, float* q1, float* q2, float* q3)
{
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;
    float qa, qb, qc;

    // 检查加速度有效性
    recipNorm = sqrtf(ax * ax + ay * ay + az * az);
    if (recipNorm < 1e-6f)
        return;

    // 加速度归一化
    recipNorm = 1.0f / recipNorm;
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // 估计重力方向
    vx = 2.0f * ((*q1) * (*q3) - (*q0) * (*q2));
    vy = 2.0f * ((*q0) * (*q1) + (*q2) * (*q3));
    vz = (*q0) * (*q0) - (*q1) * (*q1) - (*q2) * (*q2) + (*q3) * (*q3);

    // 误差项（测量方向与估计方向的叉积）
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // 积分项反馈
    if (twoKi > 0.0f) {
        integralFBx += twoKi * ex;
        integralFBy += twoKi * ey;
        integralFBz += twoKi * ez;

        gx += integralFBx;
        gy += integralFBy;
        gz += integralFBz;
    } else {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // 比例项反馈
    gx += twoKp * ex;
    gy += twoKp * ey;
    gz += twoKp * ez;

    // 四元数微分
    gx *= 0.5f;
    gy *= 0.5f;
    gz *= 0.5f;

    qa = *q0;
    qb = *q1;
    qc = *q2;

    *q0 += (-qb * gx - qc * gy - (*q3) * gz);
    *q1 += (qa * gx + qc * gz - (*q3) * gy);
    *q2 += (qa * gy - qb * gz + (*q3) * gx);
    *q3 += (qa * gz + qb * gy - qc * gx);

    // 四元数归一化
    recipNorm = 1.0f / sqrtf((*q0) * (*q0) + (*q1) * (*q1) + (*q2) * (*q2) + (*q3) * (*q3));
    *q0 *= recipNorm;
    *q1 *= recipNorm;
    *q2 *= recipNorm;
    *q3 *= recipNorm;
}
