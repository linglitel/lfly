//
// Created by Administrator on 25-6-20.
//
#include "ControlTask.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "Madgwick.h"
#include "stm32f4xx_hal.h"

// 姿态环 PID 控制器（角度环）
extern float q0, q1, q2, q3;
extern float gx, gy, gz;

// 调试输出变量（期望 & 当前），上传上位机
float debug_target = 0.0f; // 期望角速度 (rad/s)
float debug_current = 0.0f; // 实际角速度 (rad/s)

/*
// ================= PID 控制器 ===================
void PID_Init(PID_Controller* pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output = 0;
}

float PID_Update(PID_Controller* pid, float error, float dt)
{
    pid->integral += error * dt;

    // 微分项计算
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // PID输出计算
    pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    return pid->output;
}

// ================== PID 控制器实例（roll轴） ====================
PID_Controller angle_pid[1];
PID_Controller rate_pid[1];

uint32_t last_uart_send_time = 0;
extern UART_HandleTypeDef huart1;

// ================== 初始化 ====================
void PidTest_Init(void)
{
    PID_Init(&angle_pid[0], 1.5f, 0.1f, 0.1f); // 姿态环 - 减小Kp，增加Kd
    PID_Init(&rate_pid[0], 0.4f, 0.05f, 0.01f); // 速度环 - 调整参数平衡响应和稳定性
}

float calculate_angle_error(float target, float current)
{
    float error = target - current;
    // 将误差规范化到[-π, π]范围内
    while (error > PI) error -= 2.0f * PI;
    while (error < -PI) error += 2.0f * PI;
    return error;
}

// ================== 主测试函数 ====================
void PidTest_Update1(void)
{
    // 1. 计算当前 Roll（rad）
    // 1. 计算当前 Roll（rad）
    float cr = atan2f(2.0f * (q0 * q1 + q2 * q3),
                      1.0f - 2.0f * (q1 * q1 + q2 * q2));
    // 2. 目标 Roll（rad），测试时保持为 0
    float tr = 0.0f;

    // 3. 使用改进的角度误差计算函数
    float err_roll = calculate_angle_error(tr, cr);

    // 4. 姿态环 PID → 期望角速度（rad/s）
    float desired_rate = PID_Update(&angle_pid[0], err_roll, 0.001f);

    // 5. 速度环 PID → 控制量
    float rate_error = desired_rate - gx;
    float torque_cmd = PID_Update(&rate_pid[0], rate_error, 0.001f);

    // 6. 上传调试值
    debug_target = desired_rate;
    debug_current = gx;

    // 7. UART 输出
    uint32_t now = HAL_GetTick();
    if (now - last_uart_send_time >= 100) // 100 ms 间隔
    {
        char buf[50];
        int len = snprintf(buf, sizeof(buf), "%0.3f,%0.3f\n",
                           debug_target,
                           debug_current);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 10);
        last_uart_send_time = now;
    }

    // （可选）将 torque_cmd 下发给电调
    // Motor_SetPWM(..., torque_to_pulsewidth(torque_cmd));
}*/

uint32_t last_uart_send_time = 0;
extern UART_HandleTypeDef huart1;

void PidTest_Update(void)
{
    get_eulerAngle(q0, q1, q2, q3);
    // 7. UART 输出
    uint32_t now = HAL_GetTick();
    if (now - last_uart_send_time >= 100) // 100 ms 间隔
    {
        char buf[50];
        int len = snprintf(buf, sizeof(buf), "%0.3f,%0.3f,%0.3f\n",
                           roll, pitch, yaw
        );
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 10);
        last_uart_send_time = now;
    }
}

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
    /*ctrl->controlOutput.x *= -1.0f;
    ctrl->controlOutput.y *= -1.0f;
    ctrl->controlOutput.z *= -1.0f;*/
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

void set_quaternion_pitch_deg(Quaternion* q, float angle_deg)
{
    float rad = angle_deg * M_PI / 180.0f;
    q->w = cosf(rad / 2.0f);
    q->x = sinf(rad / 2.0f);
    q->y = 0.0f;
    q->z = 0.0f;
}

void test_case(float current_pitch_deg, float target_pitch_deg, AttitudeController* ctrl)
{
    set_quaternion_pitch_deg(&ctrl->currentQuat, current_pitch_deg);
    set_quaternion_pitch_deg(&ctrl->targetQuat, target_pitch_deg);

    ctrl->currentRate = (Vector3){0.0f, 0.0f, 0.0f}; // 静止

    AttitudeController_Update(ctrl);

    char buffer[100];
    sprintf(buffer, "当前Pitch: %+5.1f°, 目标Pitch: %+5.1f°\r\n", current_pitch_deg, target_pitch_deg);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);

    sprintf(buffer, "  姿态误差  : x=%.3f, y=%.3f, z=%.3f\r\n", ctrl->attitudeError.x, ctrl->attitudeError.y,
            ctrl->attitudeError.z);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);

    sprintf(buffer, "  角速度目标: x=%.3f, y=%.3f, z=%.3f\r\n", ctrl->rateCommand.x, ctrl->rateCommand.y,
            ctrl->rateCommand.z);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);

    sprintf(buffer, "  控制输出  : x=%.3f, y=%.3f, z=%.3f\r\n\r\n", ctrl->controlOutput.x, ctrl->controlOutput.y,
            ctrl->controlOutput.z);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);
}

void TEST_TEMP_WILL_DELETE()
{
    AttitudeController ctrl;
    AttitudeController_Init(&ctrl, 0.01f); // 10ms 控制周期

    char buffer[100];
    sprintf(buffer, "===== 串级PID控制器 测试集 =====\r\n\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);

    AttitudeController_Init(&ctrl, 0.01f);
    test_case(+10.0f, 0.0f, &ctrl); // 从抬头恢复

    AttitudeController_Init(&ctrl, 0.01f);
    test_case(0.0f, -10.0f, &ctrl); // 前进

    AttitudeController_Init(&ctrl, 0.01f);
    test_case(-20.0f, +10.0f, &ctrl); // 从俯冲拉起

    AttitudeController_Init(&ctrl, 0.01f);
    test_case(+50.0f, -15.0f, &ctrl); // 极端情况（50°误差）
}
