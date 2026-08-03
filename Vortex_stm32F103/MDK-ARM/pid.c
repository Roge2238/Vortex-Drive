#include "pid.h"


void PID_Init(PID_t* pid, float kp, float ki, float kd, float integral_limits, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limits = integral_limits;
    pid->output_limit = output_limit;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
}




void PID_Reset(PID_t* pid)
{
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
}



float PID_Compute(PID_t* pid, float error, float dt)
{
    if(dt <= 0.0f) dt = 0.001f;

    //p
    float p = pid->kp*error;

    //I
    pid->integral += error;
    pid->integral = HAL_CLAMP(pid->integral, -pid->integral_limits, pid->integral_limits);// 数值限幅
    float i = pid->ki*pid->integral;

    //D
    float d = pid->kd*(error - pid->last_error)/dt;
    
    pid->last_error = error;

    //输出
    float Output = p + i + d;
    return HAL_CLAMP(Output, -pid->output_limit, pid->output_limit);
}