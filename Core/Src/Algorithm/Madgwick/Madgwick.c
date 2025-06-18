#include "Madgwick.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "AlgorithmUtils.h"

MadgwickFilter filter = {
    .q0 = 1.0f, .q1 = 0.0f, .q2 = 0.0f, .q3 = 0.0f,
    .beta = 0.1f,
    .sampleFreq = 100.0f
};

float beta = 0.01f;
float sampleFreq = 500;

void MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,
                    float* q0, float* q1, float* q2, float* q3)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1,
          q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // 从陀螺仪计算四元数的变化率
    qDot1 = 0.5f * (-(*q1) * gx - (*q2) * gy - (*q3) * gz);
    qDot2 = 0.5f * ((*q0) * gx + (*q2) * gz - (*q3) * gy);
    qDot3 = 0.5f * ((*q0) * gy - (*q1) * gz + (*q3) * gx);
    qDot4 = 0.5f * ((*q0) * gz + (*q1) * gy - (*q2) * gx);

    // 仅当加速度计测量有效时才计算反馈 (避免在加速度计归一化时出现NaN)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
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
        _2q0mx = 2.0f * (*q0) * mx;
        _2q0my = 2.0f * (*q0) * my;
        _2q0mz = 2.0f * (*q0) * mz;
        _2q1mx = 2.0f * (*q1) * mx;
        _2q0 = 2.0f * (*q0);
        _2q1 = 2.0f * (*q1);
        _2q2 = 2.0f * (*q2);
        _2q3 = 2.0f * (*q3);
        _2q0q2 = 2.0f * (*q0) * (*q2);
        _2q2q3 = 2.0f * (*q2) * (*q3);
        q0q0 = (*q0) * (*q0);
        q0q1 = (*q0) * (*q1);
        q0q2 = (*q0) * (*q2);
        q0q3 = (*q0) * (*q3);
        q1q1 = (*q1) * (*q1);
        q1q2 = (*q1) * (*q2);
        q1q3 = (*q1) * (*q3);
        q2q2 = (*q2) * (*q2);
        q2q3 = (*q2) * (*q3);
        q3q3 = (*q3) * (*q3);

        // 地球磁场的参考方向
        hx = mx * q0q0 - _2q0my * (*q3) + _2q0mz * (*q2) + mx * q1q1 + _2q1 * my * (*q2) + _2q1 * mz * (*q3) - mx * q2q2
            - mx * q3q3;
        hy = _2q0mx * (*q3) + my * q0q0 - _2q0mz * (*q1) + _2q1mx * (*q2) - my * q1q1 + my * q2q2 + _2q2 * mz * (*q3) -
            my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * (*q2) + _2q0my * (*q1) + mz * q0q0 + _2q1mx * (*q3) - mz * q1q1 + _2q2 * my * (*q3) - mz * q2q2
            + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // 梯度下降算法的修正步骤
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * (*q2) * (_2bx * (0.5f -
            q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * (*q3) + _2bz * (*q1)) * (_2bx * (q1q2 - q0q3) + _2bz *
            (q0q1 + q2q3) - my) + _2bx * (*q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * (*q1) * (1.0f - 2.0f *
                q1q1 - 2.0f * q2q2 - az) + _2bz * (*q3) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (
                _2bx
                * (*q2) + _2bz * (*q0)) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * (*q3) - _4bz * (*
                q1))
            * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * (*q2) * (1.0f - 2.0f *
            q1q1 - 2.0f * q2q2 - az) + (-_4bx * (*q2) - _2bz * (*q0)) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 -
            q0q2) - mx) + (_2bx * (*q1) + _2bz * (*q3)) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * (
            *q0) - _4bz * (*q2)) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * (*q3) + _2bz * (*q1)) *
            (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * (*q0) + _2bz * (*q2)) * (_2bx * (q1q2 -
                q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * (*q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2)
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
    recipNorm = invSqrt((*q0) * (*q0) + (*q1) * (*q1) + (*q2) * (*q2) + (*q3) * (*q3));
    *q0 *= recipNorm;
    *q1 *= recipNorm;
    *q2 *= recipNorm;
    *q3 *= recipNorm;
}

void computeAngles(float q0, float q1, float q2, float q3, float* roll, float* pitch, float* yaw)
{
    *roll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * R2D;
    *pitch = asinf(-2.0f * (q1 * q3 - q0 * q2)) * R2D;
    if (fabsf(-2.0f * (q1 * q3 - q0 * q2)) >= 1)
        *pitch = copysign(M_PI / 2, -2.0f * (q1 * q3 - q0 * q2)) * R2D; // use 90 degrees if out of range
    *yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * R2D;
}

void MadgwickAHRSupdateIMU(MadgwickFilter* f,
                           float gx, float gy, float gz,
                           float ax, float ay, float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;

    float q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;
    float _2q0, _2q1, _2q2, _2q3;
    float _4q0, _4q1, _4q2;
    float _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    // 归一化加速度计测量
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    if (recipNorm == 0.0f) return; // 避免除零
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // 预计算避免重复计算
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // 梯度下降算法计算误差修正
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    // 归一化修正向量
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // 陀螺仪四元数变化率
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - f->beta * s0;
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - f->beta * s1;
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - f->beta * s2;
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - f->beta * s3;

    // 积分更新四元数
    float invSampleFreq = 1.0f / f->sampleFreq;
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    // 单位化四元数
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    f->q0 = q0 * recipNorm;
    f->q1 = q1 * recipNorm;
    f->q2 = q2 * recipNorm;
    f->q3 = q3 * recipNorm;
}

#define NX 7
#define NZ 3

// 采样周期
#define DT     (0.01f)    // 100Hz，可根据实际修改

// 噪声协方差（根据传感器标定调参）
static const float Q_proc[NX][NX] = {
    // 四元数过程噪声
    {1e-6, 0, 0, 0, 0, 0, 0},
    {0, 1e-6, 0, 0, 0, 0, 0},
    {0, 0, 1e-6, 0, 0, 0, 0},
    {0, 0, 0, 1e-6, 0, 0, 0},
    // 偏置过程噪声（慢变化）
    {0, 0, 0, 0, 1e-8, 0, 0},
    {0, 0, 0, 0, 0, 1e-8, 0},
    {0, 0, 0, 0, 0, 0, 1e-8},
};
static const float R_meas[NZ][NZ] = {
    {0.01f, 0, 0},
    {0, 0.01f, 0},
    {0, 0, 0.01f},
};

// 静态状态、协方差
static float x[NX] = {1, 0, 0, 0, 0, 0, 0};
static float P[NX][NX];

static int ekf_init = 0;

void IMUonly_EKF(
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float mx, float my, float mz,
    float* roll, float* pitch, float* yaw
)
{
    if (!ekf_init)
    {
        // 初始 P 为小量对角
        for (int i = 0; i < NX; i++)
            for (int j = 0; j < NX; j++)
                P[i][j] = (i == j) ? 1e-3f : 0.0f;
        ekf_init = 1;
    }

    // —— 预测步 ——
    // 把角速度转弧度，减去当前估计偏置
    float wx = gx * M_PI / 180.0f - x[4];
    float wy = gy * M_PI / 180.0f - x[5];
    float wz = gz * M_PI / 180.0f - x[6];

    // 四元数微分 q̇ = 0.5 * Ω(ω) * q
    float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
    float dq[4];
    dq[0] = 0.5f * (-q1 * wx - q2 * wy - q3 * wz);
    dq[1] = 0.5f * (q0 * wx + q2 * wz - q3 * wy);
    dq[2] = 0.5f * (q0 * wy - q1 * wz + q3 * wx);
    dq[3] = 0.5f * (q0 * wz + q1 * wy - q2 * wx);

    // 状态预测
    x[0] += dq[0] * DT;
    x[1] += dq[1] * DT;
    x[2] += dq[2] * DT;
    x[3] += dq[3] * DT;
    // 偏置假设随机游走（不变）

    // 归一化四元数
    float norm = sqrtf(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3]);
    for (int i = 0; i < 4; i++) x[i] /= norm;

    // P = F*P*F^T + Q*DT
    // 这里做简化：F≈I + 小量，直接 P+=Q*DT
    for (int i = 0; i < NX; i++)
        for (int j = 0; j < NX; j++)
            P[i][j] += Q_proc[i][j] * DT;


    // —— 更新步 ——
    // 测量模型：加速度 = R(q)^T * [0,0,1]
    // 构造预测测量 h
    float h[NZ];
    h[0] = 2 * (q1 * q3 - q0 * q2);
    h[1] = 2 * (q0 * q1 + q2 * q3);
    h[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 观测向量 z 归一化
    float norm_a = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    float z[NZ] = {ax * norm_a, ay * norm_a, az * norm_a};

    // 计算残差 y = z - h
    float yres[NZ];
    for (int i = 0; i < NZ; i++) yres[i] = z[i] - h[i];

    // 近似 H（Jacobian）只对四元数部分非零，其它偏置对加速度无影响
    float H[NZ][NX] = {0};
    H[0][0] = -2 * q2;
    H[0][1] = 2 * q3;
    H[0][2] = -2 * q0;
    H[0][3] = 2 * q1;
    H[1][0] = 2 * q1;
    H[1][1] = 2 * q0;
    H[1][2] = 2 * q3;
    H[1][3] = 2 * q2;
    H[2][0] = 2 * q0;
    H[2][1] = -2 * q1;
    H[2][2] = -2 * q2;
    H[2][3] = 2 * q3;

    // 计算 S = H*P*H^T + R
    float S[NZ][NZ] = {0};
    for (int i = 0; i < NZ; i++)
    {
        for (int j = 0; j < NZ; j++)
        {
            float sum = R_meas[i][j];
            for (int k = 0; k < NX; k++)
                for (int l = 0; l < NX; l++)
                    sum += H[i][k] * P[k][l] * H[j][l];
            S[i][j] = sum;
        }
    }
    // 计算 K = P*H^T * inv(S)
    float K[NX][NZ] = {0}, invS[NZ][NZ];
    // 这里硬编码 3×3 逆矩阵（S 对角占优，可简化）
    // 为简洁起见，做数值求逆
    {
        // 求逆（行列式+伴随）
        float det =
            S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1])
            - S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0])
            + S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);
        if (fabsf(det) < 1e-6f) det = 1e-6f;
        invS[0][0] = (S[1][1] * S[2][2] - S[1][2] * S[2][1]) / det;
        invS[0][1] = -(S[0][1] * S[2][2] - S[0][2] * S[2][1]) / det;
        invS[0][2] = (S[0][1] * S[1][2] - S[0][2] * S[1][1]) / det;
        invS[1][0] = -(S[1][0] * S[2][2] - S[1][2] * S[2][0]) / det;
        invS[1][1] = (S[0][0] * S[2][2] - S[0][2] * S[2][0]) / det;
        invS[1][2] = -(S[0][0] * S[1][2] - S[0][2] * S[1][0]) / det;
        invS[2][0] = (S[1][0] * S[2][1] - S[1][1] * S[2][0]) / det;
        invS[2][1] = -(S[0][0] * S[2][1] - S[0][1] * S[2][0]) / det;
        invS[2][2] = (S[0][0] * S[1][1] - S[0][1] * S[1][0]) / det;
    }
    for (int i = 0; i < NX; i++)
    {
        for (int j = 0; j < NZ; j++)
        {
            float sum = 0;
            for (int k = 0; k < NX; k++)
                for (int l = 0; l < NZ; l++)
                    sum += P[i][k] * H[l][k] * invS[l][j];
            K[i][j] = sum;
        }
    }

    // 状态更新 x += K * yres
    for (int i = 0; i < NX; i++)
    {
        float corr = 0;
        for (int j = 0; j < NZ; j++)
            corr += K[i][j] * yres[j];
        x[i] += corr;
    }
    // 归一化四元数部分
    norm = sqrtf(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3]);
    for (int i = 0; i < 4; i++) x[i] /= norm;

    // 协方差更新 P = (I-KH)P
    {
        float I_KH[NX][NX] = {0};
        for (int i = 0; i < NX; i++)
        {
            for (int j = 0; j < NX; j++)
            {
                float sum = (i == j) ? 1.0f : 0.0f;
                for (int k = 0; k < NZ; k++)
                    sum -= K[i][k] * H[k][j];
                I_KH[i][j] = sum;
            }
        }
        float P_new[NX][NX] = {0};
        for (int i = 0; i < NX; i++)
            for (int j = 0; j < NX; j++)
                for (int k = 0; k < NX; k++)
                    P_new[i][j] += I_KH[i][k] * P[k][j];
        memcpy(P, P_new, sizeof P);
    }

    // —— 输出 RPY ——
    float qq0 = x[0], qq1 = x[1], qq2 = x[2], qq3 = x[3];
    *roll = atan2f(2 * (qq0 * qq1 + qq2 * qq3), 1 - 2 * (qq1 * qq1 + qq2 * qq2)) * 180.0f / M_PI;
    *pitch = asinf(2 * (qq0 * qq2 - qq3 * qq1)) * 180.0f / M_PI;
    *yaw = atan2f(2 * (qq0 * qq3 + qq1 * qq2), 1 - 2 * (qq2 * qq2 + qq3 * qq3)) * 180.0f / M_PI;
}

void QuaternionToEuler(float q0, float q1, float q2, float q3, float* roll, float* pitch, float* yaw)
{
    // roll (x-axis rotation)
    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));

    // pitch (y-axis rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(M_PI / 2.0f, sinp); // use 90 degrees if out of range
    else
        *pitch = asinf(sinp);

    // yaw (z-axis rotation)
    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}
