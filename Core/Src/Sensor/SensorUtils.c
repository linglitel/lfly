//// Created by Administrator on 25-6-15.
//

#include "SensorUtils.h"

#include <stdint.h>

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim11;

void Delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim11, 0); // 将定时器计数器值重置为 0
    HAL_TIM_Base_Start(&htim11); // 启动定时器

    // 等待计数器值达到要延时的微秒数
    while (__HAL_TIM_GET_COUNTER(&htim11) < us) {}

    HAL_TIM_Base_Stop(&htim11); // 停止定时器
}
