//
// Created by Administrator on 25-6-20.
//
#include "ESCControlTask.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "LogUtils.h"
#include "freertos_os2.h"
#include "PidTask.h"
#include "stm32f4xx_hal.h"
#include "task.h"

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;

ESCOutput ESC_Output;
static LogEntry log;

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
    log.timestamp = HAL_GetTick();
    log.error_code = 0;
    snprintf(log.message, sizeof(log.message), "ESC initialized successfully");
    LOG_Write(&log);
}

void ESC_Update()
{
    float roll = ctrl.controlOutput.x;
    float pitch = ctrl.controlOutput.y;
    float yaw = ctrl.controlOutput.z;
    float scale = 500.0f;
    /*char buf[50];
    snprintf(buf, sizeof(buf), "%f,%f,%f", pitch, roll, yaw);
    HAL_UART_Transmit(&huart1,
                      buf, strlen(buf), 100);*/

    int16_t mix[4];
    mix[0] = BASE_THROTTLE + (int16_t)((pitch + roll - yaw) * scale);
    mix[1] = BASE_THROTTLE + (int16_t)((pitch - roll + yaw) * scale);
    mix[2] = BASE_THROTTLE + (int16_t)((-pitch + roll + yaw) * scale);
    mix[3] = BASE_THROTTLE + (int16_t)((-pitch - roll - yaw) * scale);

    // 限速
    for (int i = 0; i < 4; i++)
    {
        if (mix[i] > ESC_MAX)
        {
            log.error_code = 0;
            log.type = LOG_TYPE_WARNING;
            log.timestamp = HAL_GetTick();
            snprintf(log.message, strlen(log.message), "Motor Full Load Speed");
            mix[i] = ESC_MAX;
        }
        else if (mix[i] < ESC_MIN) mix[i] = ESC_MIN;
    }

    ESC_Output.ESCA = mix[0];
    ESC_Output.ESCB = mix[1];
    ESC_Output.ESCC = mix[2];
    ESC_Output.ESCD = mix[3];

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ESC_Output.ESCA);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ESC_Output.ESCB);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ESC_Output.ESCC);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ESC_Output.ESCD);
}


void ESCControlTask(void* argument)
{
    const TickType_t xDelay = pdMS_TO_TICKS(3);
    for (;;)
    {
        ESC_Update();
        vTaskDelay(xDelay);
    }
}
