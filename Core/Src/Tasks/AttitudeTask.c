//
// Created by Administrator on 25-6-15.
//

#include <math.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "PidTask.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include "wit_c_sdk.h"

extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;

void AttitudeTask(void* argument)
{
    const TickType_t xDelay = pdMS_TO_TICKS(10); // 1ms周期
    for (;;)
    {
        WitReadReg(0x59, 8);
        osDelay(2);
        ctrl.currentQuat.w = ((float)sReg[Q0]) / 32768.0f;
        ctrl.currentQuat.x = ((float)sReg[Q1]) / 32768.0f;
        ctrl.currentQuat.y = ((float)sReg[Q2]) / 32768.0f;
        ctrl.currentQuat.z = ((float)sReg[Q3]) / 32768.0f;
        WitReadReg(0x37, 6);
        osDelay(2);
        ctrl.currentRate.x = ((float)sReg[GX]) / 16.4f * ((float)M_PI / 180.0f);
        ctrl.currentRate.y = ((float)sReg[GY]) / 16.4f * ((float)M_PI / 180.0f);
        ctrl.currentRate.z = ((float)sReg[GZ]) / 16.4f * ((float)M_PI / 180.0f);
        vTaskDelay(xDelay);
        //todo may be some bug in there
    }
}
