#include "pid.h"
#include "mode.h"
#include "motor.h"
#include <math.h>

void PID_Init(PID_t* pid, float kp, float ki, float kd, float integral_limits, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limits = integral_limits;
    pid->output_limit = output_limit;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->d_alpha = 0.3f;   /* D项低通滤波系数，越小越平滑 */
    pid->last_d = 0.0f;
}




void PID_Reset(PID_t* pid)
{
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->last_d = 0.0f;
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
    //  filtered_d = d_alpha × raw_d + (1 - d_alpha) × last_d
    float raw_d = pid->kd * (error - pid->last_error) / dt;
    pid->last_d = pid->d_alpha * raw_d + (1.0f - pid->d_alpha) * pid->last_d;
    float d = pid->last_d;

    pid->last_error = error;

    //  输出 = P + I + D
    float Output = p + i + d;
    return HAL_CLAMP(Output, -pid->output_limit, pid->output_limit);
}

/* ========== AUTO 运动控制（原 main.c 计算逻辑，封装至此便于维护） ========== */

static PID_t pid_left;
static PID_t pid_right;
static PID_t pid_steer;
static PID_t pid_speed;

static float target_left_pwm  = 0.0f;
static float target_right_pwm = 0.0f;

static float cur_left_pwm  = 0.0f;
static float cur_right_pwm = 0.0f;

void Drive_Init(void)
{
    PID_Init(&pid_left,   1.0f, 0.5f, 0.0f, 100.0f, 80.0f);
    PID_Init(&pid_right,  1.0f, 0.5f, 0.0f, 100.0f, 80.0f);

    PID_Init(&pid_steer,  0.30f, 0.05f, 0.03f, 200.0f, 100.0f);
    PID_Init(&pid_speed,  0.005f, 0.002f, 0.0f, 200.0f, 100.0f);
}

void Drive_Reset(void)
{
    /* 清残留控制状态，防止切回后第一拍用旧值驱动电机 */
    cur_left_pwm = 0.0f;
    cur_right_pwm = 0.0f;
    PID_Reset(&pid_left);
    PID_Reset(&pid_right);
    PID_Reset(&pid_steer);
    PID_Reset(&pid_speed);
}

/* 每 20ms 调用：AUTO 模式运动计算（速度外环→转向→内环PID→斜坡→输出PWM） */
void Drive_Compute(float left_speed, float right_speed)
{
    float steer_out = 0.0f;
    float speed_out = 0.0f; //speed 弃用pid 使用非线性映射

    if (cv_active)
    {
        steer_out = PID_Compute(&pid_steer, (float)cv.error, 0.020f);
        // > 5 px 加入速度保底 摩擦力有点严重
        if (fabsf((float)cv.error) > MIN_STEER_ERROR) {
            if (steer_out > 0.0f && steer_out <  MIN_STEER_FLOOR) steer_out =  MIN_STEER_FLOOR;
            if (steer_out < 0.0f && steer_out > -MIN_STEER_FLOOR) steer_out = -MIN_STEER_FLOOR;
        }

        float area_abs = fabsf((float)cv.area);

        if (area_abs < AREA_DEADZONE)
        {
            speed_out = 0.0f;          // 已到达目标区域，停车
        }
        else
        {
            speed_out = K_AREA * sqrtf(area_abs);

            /* 最低巡航速度：防止速度太低被静摩擦卡住  依然保底 就是这么小心谨慎 */
            // 最后阶段刹车时 会有不错的刹车体验 追求精细控制其实不用 但这样还是稳定
            if (speed_out < MIN_CRUISE_SPEED)
                speed_out = MIN_CRUISE_SPEED;

            if (cv.area < 0)
                speed_out = -speed_out;
        }
    }

    speed_out = HAL_CLAMP(speed_out, -MAX_REV_SPEED, MAX_FWD_SPEED);

    #if STEER_ENABLE
        target_left_pwm  = speed_out + steer_out * STEER_WEIGHT;
        target_right_pwm = speed_out - steer_out * STEER_WEIGHT;
    #else
        target_left_pwm  = speed_out;
        target_right_pwm = speed_out;
        (void)steer_out;
    #endif
    target_left_pwm  = HAL_CLAMP(target_left_pwm,  -100.0f, 100.0f);
    target_right_pwm = HAL_CLAMP(target_right_pwm, -100.0f, 100.0f);

    float ff_left  = target_left_pwm  * KV_FORWARD; // KV 可以大一点 充分发挥 PID的补偿
    float ff_right = target_right_pwm * KV_FORWARD;

    //内环 补偿PID
    float left_err   = target_left_pwm  - left_speed;
    float right_err  = target_right_pwm - right_speed;
    float left_pid   = PID_Compute(&pid_left,  left_err,  0.020f);
    float right_pid  = PID_Compute(&pid_right, right_err, 0.020f);

    float raw_left   = ff_left  + left_pid;
    float raw_right  = ff_right + right_pid;

    //斜坡防冲击 防止启动振荡
    float avg_speed = (fabsf(left_speed) + fabsf(right_speed)) * 0.5f;
    float max_delta;
    if (avg_speed < SPEED_THRESH) {
        max_delta = RAMP_LOW;
    } else if (avg_speed < SPEED_BAND) {
        float t = (avg_speed - SPEED_THRESH) / (SPEED_BAND - SPEED_THRESH);
        max_delta = RAMP_LOW + (RAMP_HIGH - RAMP_LOW) * t;
    } else {
        max_delta = RAMP_HIGH;
    }

    float dl = raw_left  - cur_left_pwm;
    float dr = raw_right - cur_right_pwm;
    dl = HAL_CLAMP(dl, -max_delta, max_delta);
    dr = HAL_CLAMP(dr, -max_delta, max_delta);
    cur_left_pwm  += dl;
    cur_right_pwm += dr;

    /* ———：输出 PWM ——— */
    set_drive_pwm(cur_left_pwm, cur_right_pwm);
}