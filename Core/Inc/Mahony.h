//
// Created by Administrator on 25-6-18.
//

#ifndef MAHONY_H
#define MAHONY_H

void MahonyAHRSupdate(
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float *roll, float *pitch, float *yaw
);

#endif //MAHONY_H
