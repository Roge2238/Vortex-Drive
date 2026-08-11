#ifndef __PID_H__
#define __PID_H__

#include "stm32f1xx_hal.h"

#ifndef HAL_CLAMP
#define HAL_CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#endif

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
    float d_alpha;  // D项低通滤波系数，默认0.3
    float last_d;   // 上一次滤波后的D项值

} PID_t;


void PID_Init(PID_t* pid, float kp, float ki, float kd, float integral_limits, float output_limit);

float PID_Compute(PID_t* pid , float error, float dt);

void PID_Reset(PID_t* pid);

#endif /* __PID_H__ */