//
// Created by Administrator on 25-6-26.
//

#include "PidTask.h"
#include "RemoteControlTask.h"

void RemoteControlTask(void* argument)
{
    //todo
    ctrl.targetQuat.w = 0;
    ctrl.targetQuat.x = 0;
    ctrl.targetQuat.y = 0;
    ctrl.targetQuat.z = 0;
}
