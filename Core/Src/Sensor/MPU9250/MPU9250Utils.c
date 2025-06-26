//todo need add function

#include <stdint.h>

#include "Madgwick.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "MPU9250Utils.h"

extern I2C_HandleTypeDef hi2c1;

void MPU9250_Init()
{
    uint8_t data;

    // === MPU9250 初始化 ===


    // 1. 检查 WHO_AM_I
    data = 0;
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, 0x75, 1, &data, 1, HAL_MAX_DELAY);
    if (data != 0x71)
    {
        //Error_Handler();
        //Do not use it,we should have better slovtiuon
    }

    // 2. 退出休眠，设置时钟源为陀螺仪X轴（0x01）
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, 0x6B, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(100);

    // 3. 设置加速度计量程 ±2g（0x00）
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, 0x1C, 1, &data, 1, HAL_MAX_DELAY);

    // 4. 设置陀螺仪量程 ±250°/s（0x00）
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, 0x1B, 1, &data, 1, HAL_MAX_DELAY);

    // 5. 打开 I2C 主模式 bypass，使主控可直接访问 AK8963
    data = 0x02; // BIT1 = I2C_BYPASS_EN
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, 0x37, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    // 设置加速度计低通滤波器（如44.8Hz截止频率）
    data = 0x06; // 配置ACCEL_CONFIG2寄存器，DLPF_CFG=0x03
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, 0x1D, 1, &data, 1, HAL_MAX_DELAY);

    // 设置陀螺仪低通滤波器（如41Hz截止频率）
    data = 0x06; // 配置CONFIG寄存器，DLPF_CFG=0x03
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, 0x1A, 1, &data, 1, HAL_MAX_DELAY);

    // === AK8963 初始化 ===

    uint8_t asa[3];

    // 1. 检查 AK8963 WHO_AM_I
    data = 0;
    HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDR, 0x00, 1, &data, 1, HAL_MAX_DELAY);
    if (data != 0x48)
    {
        Error_Handler();
    }
    data = 0x06;
    HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDR, 0x0A, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    // 进入 Fuse ROM 读取模式
    data = 0x0F;
    HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDR, 0x0A, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    // 读取 ASA 寄存器（0x10~0x12）
    HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDR, 0x10, 1, asa, 3, HAL_MAX_DELAY);

    // 退出 Fuse ROM 模式（进入 Power-down）
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDR, 0x0A, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    data = 0x16; // 0b00010110 -> 16-bit | Continuous Mode 1
    HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDR, 0x0A, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    // 保存校准因子

    mag_sensitivity_adjust[0] = ((asa[0] - 128) / 256.0f + 1.0f);
    mag_sensitivity_adjust[1] = ((asa[1] - 128) / 256.0f + 1.0f);
    mag_sensitivity_adjust[2] = ((asa[2] - 128) / 256.0f + 1.0f);
}
