#include "pid.h"
#include<math.h>

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

    //  P — 比例项
    float p = pid->kp * error;

    //  I — 积分项（含积分分离 + 抗饱和）
    //  修复：积分累加必须乘 dt，使 ki 的量纲与时间解耦
    
    #define INTEGRAL_SEPARATION_THRESHOLD 20.0f   // 误差超过此值关闭积分

    if (fabsf(error) < INTEGRAL_SEPARATION_THRESHOLD)
    {
        // 抗饱和：预判 P + 已有积分是否已饱和，是则不继续累加
        float temp_output = p + pid->ki * pid->integral;
        if (temp_output < pid->output_limit && temp_output > -pid->output_limit)
        {
            pid->integral += error * dt;   
            pid->integral = HAL_CLAMP(pid->integral,
                                      -pid->integral_limits,
                                       pid->integral_limits);
        }
    }
    else
    {
        // 误差大：清零积分，只用 P(+D) 快速响应
        pid->integral = 0.0f;
    }
    float i = pid->ki * pid->integral;

    //  D — 微分项 
    float d = pid->kd * (error - pid->last_error) / dt;

    pid->last_error = error;

    //  输出 = P + I + D
    float Output = p + i + d;
    return HAL_CLAMP(Output, -pid->output_limit, pid->output_limit);
}