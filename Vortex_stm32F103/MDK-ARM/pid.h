#include "stm32f1xx_hal.h

//闭环控制逻辑

typedef struct
{
    float kp; // 比例
    float ki; // 积分
    float kd; // 微分
    float last_error; // 上一次的误差
    float integral;// 累积积分值
    float integral_limits;//积分限幅
    float output_limit;// 输出限制

} PID_t;


void PID_Init(PID_t* pid, float kp, float ki, float kd, float integral_limits, float output_limit);

float PID_Compute(PID_t* pid , float error, float dt);

void PID_Reset(PID_t* pid);