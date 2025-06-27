//
// Created by Administrator on 25-6-20.
//
#include "ESCControlTask.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "LogUtils.h"
#include "freertos_os2.h"
#include "stm32f4xx_hal.h"
#include "task.h"

extern TIM_HandleTypeDef htim2;

ESCOutput ESC_Output;

void ESC_Init()
{
    // 启动 TIM2 的 4 路 PWM 输出，通道 1~4
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2000);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 2000);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 2000);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 2000);

    HAL_Delay(2000); // 2秒高电平启动信号

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);

    HAL_Delay(2000); // 等待2秒

    ESC_Output.ESCA = 1000;
    ESC_Output.ESCB = 1000;
    ESC_Output.ESCC = 1000;
    ESC_Output.ESCD = 1000;

    LogEntry log;
    log.module_id = ESC_MOUDLE_ID;
    log.type = LOG_TYPE_INFO;
    log.severity = LOG_SEVERITY_INFO;
    log.timestamp = HAL_GetTick();
    log.error_code = 0;
    snprintf(log.message, sizeof(log.message), "ESC initialized successfully");
    LOG_Write(&log);
}

void ESC_Update()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ESC_Output.ESCA);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ESC_Output.ESCB);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ESC_Output.ESCC);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ESC_Output.ESCD);
}

void ESCControlTask(void* argument)
{
    const TickType_t xDelay = pdMS_TO_TICKS(5); // 10ms周期
    for (;;)
    {
        ESC_Update();
        vTaskDelay(xDelay);
    }
}
