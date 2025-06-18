//
// Created by Administrator on 25-6-15.
//

#include <math.h>
#include <stdio.h>

#include "AK8963Utils.h"
#include "Complementary.h"
#include "cmsis_os.h"
#include "FreeRTOSConfig.h"
#include "Madgwick.h"
#include "MPU9250Utils.h"
#include "task.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart1;

void UpdateAndPrintAttitude(void);

void AttitudeTask(void* argument)
{
    float p = 0;
    float r = 0;
    TickType_t lastWakeTime = osKernelGetTickCount(); // 获取当前时间点
    const TickType_t period = pdMS_TO_TICKS(1);
    for (;;)
    {
        UpdateAndPrintAttitude();
        //vTaskDelayUntil(&lastWakeTime, period);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern MadgwickFilter filter; // 姿态滤波器全局变量

// 更新姿态并打印输出
void UpdateAndPrintAttitude(void)
{

}
