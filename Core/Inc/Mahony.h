//
// Created by Administrator on 25-6-24.
//

#ifndef MAHONY_H
#define MAHONY_H

void MahonyUpdateIMU(
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float* q0, float* q1, float* q2, float* q3);

#endif //MAHONY_H
