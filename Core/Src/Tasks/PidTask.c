//
// Created by Administrator on 25-6-20.
//


#include "PidTask.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos_os2.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart1;
AttitudeController ctrl;

Quaternion Quaternion_Error(Quaternion target, Quaternion current)
{
    Quaternion conj;
    conj.w = current.w;
    conj.x = -current.x;
    conj.y = -current.y;
    conj.z = -current.z;

    Quaternion result;
    result.w = conj.w * target.w - conj.x * target.x - conj.y * target.y - conj.z * target.z;
    result.x = conj.w * target.x + conj.x * target.w + conj.y * target.z - conj.z * target.y;
    result.y = conj.w * target.y - conj.x * target.z + conj.y * target.w + conj.z * target.x;
    result.z = conj.w * target.z + conj.x * target.y - conj.y * target.x + conj.z * target.w;


    if (result.w < 0.0f)
    {
        result.w = -result.w;
        result.x = -result.x;
        result.y = -result.y;
        result.z = -result.z;
    }

    return result;
}

Vector3 Quaternion_ToRotationVector(Quaternion q_err)
{
    Vector3 vec;

    // 保证 q_err 为单位四元数（可选）
    float norm = sqrtf(q_err.w * q_err.w + q_err.x * q_err.x + q_err.y * q_err.y + q_err.z * q_err.z);
    if (norm < 1e-6f)
    {
        vec.x = vec.y = vec.z = 0.0f;
        return vec;
    }

    // 归一化（可省略，如果输入已单位化）
    q_err.w /= norm;
    q_err.x /= norm;
    q_err.y /= norm;
    q_err.z /= norm;

    // 计算角度
    float angle = 2.0f * acosf(q_err.w); // 角度 ∈ [0, π]
    float sin_half_angle = sqrtf(1.0f - q_err.w * q_err.w);

    if (sin_half_angle < 1e-6f)
    {
        // 非常小角度，方向任意，这里取近似线性
        vec.x = q_err.x * 2.0f;
        vec.y = q_err.y * 2.0f;
        vec.z = q_err.z * 2.0f;
    }
    else
    {
        // 标准旋转向量公式：v = axis * angle
        vec.x = (q_err.x / sin_half_angle) * angle;
        vec.y = (q_err.y / sin_half_angle) * angle;
        vec.z = (q_err.z / sin_half_angle) * angle;
    }

    return vec;
}


float PID_Update(PIDController* pid, float error, float dt)
{
    // 积分更新
    pid->integral += error * dt;

    // 积分限幅
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    // 微分计算
    float derivative = (error - pid->prev_error) / dt;

    // PID输出计算
    pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // 输出限幅
    if (pid->output > pid->output_limit) pid->output = pid->output_limit;
    else if (pid->output < -pid->output_limit) pid->output = -pid->output_limit;

    // 保存误差用于下一次微分
    pid->prev_error = error;

    return pid->output;
}

void PID3Axis_Init(PID3Axis* pid3, float kp, float ki, float kd, float integral_limit, float output_limit)
{
    PIDController* pids[3] = {&pid3->x, &pid3->y, &pid3->z};
    for (int i = 0; i < 3; i++)
    {
        pids[i]->kp = kp;
        pids[i]->ki = ki;
        pids[i]->kd = kd;
        pids[i]->integral = 0.0f;
        pids[i]->prev_error = 0.0f;
        pids[i]->output = 0.0f;
        pids[i]->integral_limit = integral_limit;
        pids[i]->output_limit = output_limit;
    }
}

Vector3 PID3Axis_Update(PID3Axis* pid3, Vector3 error, float dt)
{
    Vector3 output;
    output.x = PID_Update(&pid3->x, error.x, dt);
    output.y = PID_Update(&pid3->y, error.y, dt);
    output.z = PID_Update(&pid3->z, error.z, dt);
    return output;
}

void AttitudeController_Update(AttitudeController* ctrl)
{
    // 1. 姿态误差（旋转向量）
    Quaternion q_err = Quaternion_Error(ctrl->targetQuat, ctrl->currentQuat);
    ctrl->attitudeError = Quaternion_ToRotationVector(q_err);

    // 2. 姿态环 PID → 输出角速度目标
    ctrl->rateCommand = PID3Axis_Update(&ctrl->attitudePID, ctrl->attitudeError, ctrl->dt);

    // 3. 计算角速度误差
    ctrl->rateError.x = ctrl->rateCommand.x - ctrl->currentRate.x;
    ctrl->rateError.y = ctrl->rateCommand.y - ctrl->currentRate.y;
    ctrl->rateError.z = ctrl->rateCommand.z - ctrl->currentRate.z;

    // 4. 速度环 PID → 输出控制值
    ctrl->controlOutput = PID3Axis_Update(&ctrl->ratePID, ctrl->rateError, ctrl->dt);
}

void AttitudeController_Init(AttitudeController* ctrl, float dt)
{
    ctrl->dt = dt;

    // 目标姿态：初始为平衡（单位四元数）
    ctrl->targetQuat = (Quaternion){1.0f, 0.0f, 0.0f, 0.0f};

    // 当前姿态也先设为平衡
    ctrl->currentQuat = (Quaternion){1.0f, 0.0f, 0.0f, 0.0f};

    // 当前角速度初始化
    ctrl->currentRate = (Vector3){0.0f, 0.0f, 0.0f};

    // 姿态 PID 参数（控制角度响应）
    PID3Axis_Init(&ctrl->attitudePID, 4.0f, 0.0f, 0.0f, 10.0f, 100.0f);

    // 角速度 PID 参数（控制角速度）
    PID3Axis_Init(&ctrl->ratePID, 0.5f, 0.05f, 0.01f, 5.0f, 100.0f);
}


void PidTask(void* argument)
{
    AttitudeController_Init(&ctrl, 0.010f);
    const TickType_t xDelay = pdMS_TO_TICKS(10); // 10ms周期
    for (;;)
    {
        AttitudeController_Update(&ctrl);
        vTaskDelay(xDelay);
    }
}
