//
// Created by Administrator on 25-6-15.
//

#include <math.h>
#include <stdio.h>

#include "AK8963Utils.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "Madgwick.h"
#include "MPU9250Utils.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;
int16_t mx_raw = 0, my_raw = 0, mz_raw = 0;

float gx, gy, gz;
float mx, my, mz;

void AttitudeTask(void* argument)
{
    UNUSED(argument);
    //todo need add Sensor Data Tasks
    // 读取加速度计原始值
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x3B, 1, (uint8_t*)&ax_raw, 2, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x3D, 1, (uint8_t*)&ay_raw, 2, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x3F, 1, (uint8_t*)&az_raw, 2, HAL_MAX_DELAY);

    // 读取陀螺仪原始值
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x43, 1, (uint8_t*)&gx_raw, 2, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x45, 1, (uint8_t*)&gy_raw, 2, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x47, 1, (uint8_t*)&gz_raw, 2, HAL_MAX_DELAY);

    uint8_t mag_status;

    // 1. 检查 ST1 寄存器 (0x02)，看数据是否就绪 (DRDY bit is 1)
    HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDR, 0x02, 1, &mag_status, 1, HAL_MAX_DELAY);
    uint8_t mag_buf[7];
    // 2. 如果数据就绪 (mag_status 的第0位为1)
    if (mag_status & 0x01)
    {
        // 高效读取方法：一次性读取从 HXL (0x03) 到 ST2 (0x09) 的所有7个字节
        // 这样既拿到了数据，也完成了读取 ST2 的操作

        HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDR, 0x03, 1, mag_buf, 7, HAL_MAX_DELAY);

        // mag_buf[6] 现在是 ST2 寄存器的值，可以用来检查数据是否溢出
        // 如果 ST2 的 HOFL 位 (bit3) 为0，则数据有效
        if (!(mag_buf[6] & 0x08))
        {
            // AK8963 是小端序 (Little Endian)，低字节在前
            mx_raw = (int16_t)(mag_buf[1] << 8 | mag_buf[0]);
            my_raw = (int16_t)(mag_buf[3] << 8 | mag_buf[2]);
            mz_raw = (int16_t)(mag_buf[5] << 8 | mag_buf[4]);
        }
        // 读取 mag_buf[6] 的这个动作本身就已经完成了“解锁”传感器的任务
    }


    // 原始数据是高低字节连续，需要根据字节顺序转换（假设小端）
    ax_raw = (int16_t)(((uint8_t*)&ax_raw)[0] << 8 | ((uint8_t*)&ax_raw)[1]);
    ay_raw = (int16_t)(((uint8_t*)&ay_raw)[0] << 8 | ((uint8_t*)&ay_raw)[1]);
    az_raw = (int16_t)(((uint8_t*)&az_raw)[0] << 8 | ((uint8_t*)&az_raw)[1]);

    gx_raw = (int16_t)(((uint8_t*)&gx_raw)[0] << 8 | ((uint8_t*)&gx_raw)[1]);
    gy_raw = (int16_t)(((uint8_t*)&gy_raw)[0] << 8 | ((uint8_t*)&gy_raw)[1]);
    gz_raw = (int16_t)(((uint8_t*)&gz_raw)[0] << 8 | ((uint8_t*)&gz_raw)[1]);

    // 单位换算
    float ax = (float)ax_raw / 16384.0f; // ±2g量程
    float ay = (float)ay_raw / 16384.0f;
    float az = (float)az_raw / 16384.0f;

    float gx_dps = (float)gx_raw / 131.0f; // ±250°/s量程
    float gy_dps = (float)gy_raw / 131.0f;
    float gz_dps = (float)gz_raw / 131.0f;

    gx = M_PI / 180.0f * gx_dps;
    gy = M_PI / 180.0f * gy_dps;
    gz = M_PI / 180.0f * gz_dps;

    mx = (float)my_raw * mag_sensitivity_adjust[1] - 236.35f; // 将对齐后的 mx 赋值为 校准后的 my
    my = (float)mx_raw * mag_sensitivity_adjust[0] - 10.33f; // 将对齐后的 my 赋值为 校准后的 mx
    mz = -((float)mz_raw * mag_sensitivity_adjust[2] + 147.34f); // 将对齐后的 mz 赋值为 校准后的 -mz

    MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, &q0, &q1, &q2, &q3);
    //vTaskDelayUntil(&lastWakeTime, period);
    osDelay(pdMS_TO_TICKS(10));
}
