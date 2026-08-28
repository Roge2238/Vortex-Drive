#ifndef __PID_H__
#define __PID_H__

#include "stm32f1xx_hal.h"

#ifndef HAL_CLAMP
#define HAL_CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#endif



//2200 脉冲/20ms ≈ 满载，对应 target=100 
#define PULSE_MAX  2200.0f

/* 内环前馈系数 */
#define KV_FORWARD 1.5f

/* 自适应斜坡：车速越低允许变化率越小，防止启动冲击 */
#define RAMP_LOW      8.0f   // 低速时每周期最多变 8% PWM
#define RAMP_HIGH     20.0f  // 高速时可放松到 20%/周期（亲测有效）
#define SPEED_THRESH  3.0f   // 车速 < 3% 认为尚未脱离静摩擦区
#define SPEED_BAND    15.0f  // 斜坡插值的车速上界

/* 速度外环 sqrt 非线性映射 */
#define K_AREA           0.1f   // sqrt 速度系数
#define MIN_CRUISE_SPEED 2.5f   // 最低巡航速度，防止静摩擦卡住
#define AREA_DEADZONE    200.0f // |area| < 200 认为到达目标

/* 转向控制 */
#define STEER_ENABLE     1     // 1: 转向差速使能
#define STEER_WEIGHT     0.3f  // 转向差速权重
#define MIN_STEER_ERROR  5.0f  // |error| > 5px 才做转向保底
#define MIN_STEER_FLOOR  5.0f  // 转向最小输出，防摩擦力卡死

/* 外环速度限幅 */
#define MAX_FWD_SPEED  15.0f   // 外环最大前进速度
#define MAX_REV_SPEED  10.0f   // 外环最大后退速度

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

/* ========== AUTO 运动控制（原 main.c 计算逻辑，封装至此便于维护） ========== */

void Drive_Init(void);                                    /* 初始化各PID参数 */
void Drive_Reset(void);                                   /* 清空PID积分与输出状态（模式切换/断链/丢失时调用） */
void Drive_Compute(float left_speed, float right_speed);  /* 每20ms节拍调用：运动计算并输出电机PWM */

#endif /* __PID_H__ */