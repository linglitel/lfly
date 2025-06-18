//
// Created by Administrator on 25-6-15.
//

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "MPU9250Utils.h"
#include "stm32f4xx_hal_uart.h"

#define dt  0.001f

extern UART_HandleTypeDef huart1;

typedef struct
{
    float pitch;
    float roll;
    float yaw;
} Attitude_t;

Attitude_t attitude;

void Update_Attitude(float* p, float* r)
{
    // 计算 Roll（左右倾斜）：绕 X轴 → 用 Y 和 Z
    float acc_roll = atan2f(accY, sqrtf(accX * accX + accZ * accZ)) * 180.0f / M_PI;
    // 计算 Pitch（前后倾斜）：绕 Y轴 → 用 X 和 Z
    float acc_pitch = atan2f(-accX, accZ) * 180.0f / M_PI;

    // 互补滤波
    static float pitch = 0, roll = 0;
    const float alpha = 0.98f;

    pitch = alpha * (pitch + gyroX * dt) + (1 - alpha) * acc_pitch;
    roll = alpha * (roll + gyroY * dt) + (1 - alpha) * acc_roll;

    *p = pitch;
    *r = roll;
    // 输出 pitch 和 roll
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Pitch: %.2f, Roll: %.2f\r\n", pitch, roll);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}
