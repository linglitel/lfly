#include "Madgwick.h"

#include <math.h>

#include "AlgorithmUtils.h"


float beta = 0.01f;
float sampleFreq = 1000;

float mag_sensitivity_adjust[3] = {1.0f, 1.0f, 1.0f}; // 你的磁力计校准比例因子，示例1.0f
float mag_offset[3] = {-10.33f, -236.35f, 147.34f}; // 你的硬铁偏移量

void MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,
                    float* q0, float* q1, float* q2, float* q3)
{
    float recipNorm;
    float qDot1, qDot2, qDot3, qDot4;


    // 从陀螺仪计算四元数的变化率
    qDot1 = 0.5f * (-*q1 * gx - *q2 * gy - *q3 * gz);
    qDot2 = 0.5f * (*q0 * gx + *q2 * gz - *q3 * gy);
    qDot3 = 0.5f * (*q0 * gy - *q1 * gz + *q3 * gx);
    qDot4 = 0.5f * (*q0 * gz + *q1 * gy - *q2 * gx);

    // 仅当加速度计测量有效时才计算反馈 (避免在加速度计归一化时出现NaN)
    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f))
    {
        float s3;
        float s2;
        float hx, hy;
        float _2q3;
        float _2q0q2;
        float _2q2q3;
        float q0q0;
        float q0q1;
        float q0q2;
        float q0q3;
        float q1q1;
        float q1q2;
        float q2q2;
        float q2q3;
        float q3q3;
        float s1;
        float _2q1;
        float q1q3;
        float _2q2;
        float _4bx;
        float _2q0;
        float _2bz;
        float _4bz;
        float s0;
        float _2q0mx;
        float _2bx;
        float _2q1mx;
        float _2q0mz;
        float _2q0my;
        // 归一化加速度计测量值
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 归一化磁力计测量值
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // 辅助变量以避免重复计算
        _2q0mx = 2.0f * *q0 * mx;
        _2q0my = 2.0f * *q0 * my;
        _2q0mz = 2.0f * *q0 * mz;
        _2q1mx = 2.0f * *q1 * mx;
        _2q0 = 2.0f * *q0;
        _2q1 = 2.0f * *q1;
        _2q2 = 2.0f * *q2;
        _2q3 = 2.0f * *q3;
        _2q0q2 = 2.0f * *q0 * *q2;
        _2q2q3 = 2.0f * *q2 * *q3;
        q0q0 = *q0 * *q0;
        q0q1 = *q0 * *q1;
        q0q2 = *q0 * *q2;
        q0q3 = *q0 * *q3;
        q1q1 = *q1 * *q1;
        q1q2 = *q1 * *q2;
        q1q3 = *q1 * *q3;
        q2q2 = *q2 * *q2;
        q2q3 = *q2 * *q3;
        q3q3 = *q3 * *q3;

        // 地球磁场的参考方向
        hx = mx * q0q0 - _2q0my * *q3 + _2q0mz * *q2 + mx * q1q1 + _2q1 * my * *q2 + _2q1 * mz * *q3 - mx * q2q2
            - mx * q3q3;
        hy = _2q0mx * *q3 + my * q0q0 - _2q0mz * *q1 + _2q1mx * *q2 - my * q1q1 + my * q2q2 + _2q2 * mz * *q3 -
            my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * *q2 + _2q0my * *q1 + mz * q0q0 + _2q1mx * *q3 - mz * q1q1 + _2q2 * my * *q3 - mz * q2q2
            + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // 梯度下降算法的修正步骤
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * *q2 * (_2bx * (0.5f -
            q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * *q3 + _2bz * *q1) * (_2bx * (q1q2 - q0q3) + _2bz *
            (q0q1 + q2q3) - my) + _2bx * *q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * *q1 * (1.0f - 2.0f *
                q1q1 - 2.0f * q2q2 - az) + _2bz * *q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (
                _2bx
                * *q2 + _2bz * *q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * *q3 - _4bz * *
                q1)
            * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * *q2 * (1.0f - 2.0f *
            q1q1 - 2.0f * q2q2 - az) + (-_4bx * *q2 - _2bz * *q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 -
            q0q2) - mx) + (_2bx * *q1 + _2bz * *q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * *q0 -
            _4bz * *q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * *q3 + _2bz * *q1) *
            (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * *q0 + _2bz * *q2) * (_2bx * (q1q2 -
                q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * *q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2)
                - mz);

        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // 归一化步长
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // 应用反馈步骤
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // 对四元数变化率进行积分，得到四元数
    *q0 += qDot1 * (1.0f / sampleFreq);
    *q1 += qDot2 * (1.0f / sampleFreq);
    *q2 += qDot3 * (1.0f / sampleFreq);
    *q3 += qDot4 * (1.0f / sampleFreq);

    // 归一化四元数
    recipNorm = invSqrt(*q0 * *q0 + *q1 * *q1 + *q2 * *q2 + *q3 * *q3);
    *q0 *= recipNorm;
    *q1 *= recipNorm;
    *q2 *= recipNorm;
    *q3 *= recipNorm;
}
