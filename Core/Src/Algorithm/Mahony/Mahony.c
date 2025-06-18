#include <math.h>

#include "Mahony.h"

// Mahony 6 轴参数
#define twoKp  (2.0f * 0.5f)   // 2 * 比例增益 Kp
#define twoKi  (2.0f * 0.1f)   // 2 * 积分增益 Ki

// 全局状态：四元数 + 积分误差
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

// 100Hz 更新率示例
#define INV_SAMPLE_FREQ  (1.0f/100.0f)

// 只用加速度+陀螺更新，无磁力计
// ax,ay,az (g)，gx,gy,gz (°/s)，roll,pitch,yaw 输出指针 (°)
void MahonyAHRSupdate(
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float *roll, float *pitch, float *yaw
) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 归一化加速度测量
    recipNorm = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // 角速度从 °/s 转成 弧度/s
    gx *= M_PI/180.0f;
    gy *= M_PI/180.0f;
    gz *= M_PI/180.0f;

    // 估计重力方向
    halfvx = q1*q3 - q0*q2;
    halfvy = q0*q1 + q2*q3;
    halfvz = q0*q0 - 0.5f + q3*q3;

    // 误差 = 测量重力向量 × 估计重力向量
    halfex = (ay*halfvz - az*halfvy);
    halfey = (az*halfvx - ax*halfvz);
    halfez = (ax*halfvy - ay*halfvx);

    // 积分误差累加
    integralFBx += twoKi * halfex * INV_SAMPLE_FREQ;
    integralFBy += twoKi * halfey * INV_SAMPLE_FREQ;
    integralFBz += twoKi * halfez * INV_SAMPLE_FREQ;
    // 用积分项校正陀螺漂移
    gx += integralFBx;
    gy += integralFBy;
    gz += integralFBz;

    // 比例反馈
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;

    // 四元数微分整合
    gx *= 0.5f * INV_SAMPLE_FREQ;
    gy *= 0.5f * INV_SAMPLE_FREQ;
    gz *= 0.5f * INV_SAMPLE_FREQ;
    qa = q0; qb = q1; qc = q2;
    q0 += -qb * gx - qc * gy - q3 * gz;
    q1 +=  qa * gx + qc * gz - q3 * gy;
    q2 +=  qa * gy - qb * gz + q3 * gx;
    q3 +=  qa * gz + qb * gy - qc * gx;

    // 四元数归一化
    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // 输出 RPY（°）
    *roll  = atan2f(2.0f*(q0*q1 + q2*q3),
                    q0*q0 - q1*q1 - q2*q2 + q3*q3) * 180.0f/M_PI;
    *pitch = -asinf(2.0f*(q1*q3 - q0*q2))               * 180.0f/M_PI;
    // yaw 直接从陀螺积分（累积漂移），也可以固定为 0 或保留上一次值
    static float yaw_int = 0.0f;
    yaw_int += gz * INV_SAMPLE_FREQ * 180.0f/M_PI;
    *yaw = yaw_int;
}
