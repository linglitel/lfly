//
// Created by Administrator on 25-6-17.
//

#ifndef MADGWICK_H
#define MADGWICK_H

#define PI 3.141592
#define R2D 180.00f/3.141592f

#define MPU9250_ADDR  (0x68 << 1)  // 7-bit 地址左移一位
#define AK8963_ADDR   (0x0C << 1)  // AK8963 地址

extern float mag_sensitivity_adjust[3];
extern float mag_offset[3]; //todo need more precise

void MadgwickUpdate(
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float mx, float my, float mz,
    float* q0, float* q1, float* q2, float* q3);

#endif //MADGWICK_H
