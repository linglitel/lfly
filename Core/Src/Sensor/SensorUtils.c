//// Created by Administrator on 25-6-15.
//

#include "SensorUtils.h"

#include <stdint.h>

#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

void Delay_ms(uint16_t ms)
{
    HAL_Delay(ms);
}
