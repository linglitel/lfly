//
// Created by Administrator on 25-6-26.
//

#include "PidTask.h"
#include "RemoteControlTask.h"
#include "freertos_os2.h"
#include "task.h"

void RemoteControlTask(void* argument)
{
    //todo
    const TickType_t xDelay = pdMS_TO_TICKS(3);
    for (;;)
    {
        ctrl.targetQuat.w = 0;
        ctrl.targetQuat.x = 0;
        ctrl.targetQuat.y = 0;
        ctrl.targetQuat.z = 0;
        vTaskDelay(xDelay);
    }
}
