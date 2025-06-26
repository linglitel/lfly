//
// Created by Administrator on 25-6-20.
//

#ifndef PIDTASK_H
#define PIDTASK_H


typedef struct
{
    float kp; // 比例系数
    float ki; // 积分系数
    float kd; // 微分系数

    float integral; // 积分累计值
    float prev_error; // 上一次误差
    float output; // 当前输出值

    float integral_limit; // 积分限幅
    float output_limit; // 输出限幅
} PIDController;

typedef struct
{
    PIDController x; // X轴（Roll）
    PIDController y; // Y轴（Pitch）
    PIDController z; // Z轴（Yaw）
} PID3Axis;

typedef struct
{
    float x;
    float y;
    float z;
} Vector3;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} Quaternion;

typedef struct
{
    Quaternion targetQuat; // 目标姿态（四元数）
    Quaternion currentQuat; // 当前姿态（四元数）

    Vector3 attitudeError; // 姿态误差向量（通过四元数转换）
    Vector3 rateCommand; // 姿态环输出 → 目标角速度
    Vector3 currentRate; // 当前角速度（陀螺仪 gx, gy, gz）
    Vector3 rateError; // 角速度误差（速度环输入）

    PID3Axis attitudePID; // 姿态环 PID 控制器（角度环）
    PID3Axis ratePID; // 角速度环 PID 控制器（速度环）

    Vector3 controlOutput; // 控制输出（最终用于电调或电机分配）
    float dt; // 控制周期（单位：秒）
} AttitudeController;

extern AttitudeController ctrl;

void TEST_TEMP_WILL_DELETE();
void PidTask(void* argument);

#endif //PIDTASK_H
