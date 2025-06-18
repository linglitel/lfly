//
// Created by Administrator on 25-6-17.
//

#ifndef MADGWICK_H
#define MADGWICK_H

#define PI 3.141592
#define R2D 180.00f/3.141592f

typedef struct
{
    float q0, q1, q2, q3; // 四元数
    float beta; // 算法增益
    float sampleFreq; // 采样频率
} MadgwickFilter;

void MadgwickUpdate(
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float mx, float my, float mz,
    float* q0, float* q1, float* q2, float* q3);
void MadgwickAHRSupdateIMU(MadgwickFilter* f,
                           float gx, float gy, float gz,
                           float ax, float ay, float az);
void computeAngles(float q0, float q1, float q2, float q3, float* roll, float* pitch, float* yaw);

void EKF_Attitude(
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float mx, float my, float mz,
    float* roll, float* pitch, float* yaw
);

void IMUonly_EKF(
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float mx, float my, float mz,
    float* roll, float* pitch, float* yaw
);
void QuaternionToEuler(float q0, float q1, float q2, float q3, float* roll, float* pitch, float* yaw);
#endif //MADGWICK_H
