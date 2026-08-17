#include "servo.h"
#include "tim.h"

/* ===== 舵机增量式 PID 控制 =====
 * 架构：0x04(SERVO_TURN) 帧携带像素偏移量 → 树莓派摄像头画面中心误差
 *       增量式 PID 把像素偏移换算成角度偏移量(CCR) → 累加到绝对角度位置
 *       pre_X/Y_PWM 即"当前舵机绝对角度(CCR)"，每周期 +增量 得到新位置
 *
 * 时序：由 TIM4 20ms 中断驱动 (servo_turn==true 时调用 servo_method)
 */

static Servo_Info servo_info = {0};    /* 帧解析经 Servo_UpdateMeasure 填充 */

static bool servo_turn = false;

/* ——— 增量式 PID 状态（X=水平Pan, Y=竖直Tilt） ——— */
static float x_kp, x_ki, x_kd;
static float x_last_err;
static float x_v_err_sum;              /* 误差累积 → 绝对角度位置基准 */

static float y_kp, y_ki, y_kd;
static float y_last_err;
static float y_v_err_sum;

// ——— 舵机绝对角度位置（当前 CCR 值） 
static int pre_X_PWM = SERVO_CENTER_CCR; /* 初始化为中位 90° */
static int pre_Y_PWM = SERVO_CENTER_CCR;

void Servo_PID_Init(void)
{
    /* 像素 → CCR 增量增益。先给保守值，按实测标定：
     *   x_kp 过大 → 舵机振荡；过小 → 跟踪滞后
     *   x_kd 提供阻尼，抑制像素噪声抖动 */
    x_kp = 0.05f;
    x_ki = 0.0f;
    x_kd = 0.3f;
    y_kp = 0.05f;
    y_ki = 0.0f;
    y_kd = 0.3f;

    x_last_err = 0.0f;
    x_v_err_sum = 0.0f;
    y_last_err = 0.0f;
    y_v_err_sum = 0.0f;// 不再用I项 

    /* 位置复位到中位 90°，避免上电从 0° 猛甩到目标 */
    pre_X_PWM = SERVO_CENTER_CCR;
    pre_Y_PWM = SERVO_CENTER_CCR;
    X_PWM(pre_X_PWM);
    Y_PWM(pre_Y_PWM);
}

bool Get_Servo_turn(void)
{
    return servo_turn;
}

void Set_Servo_turn(bool turn)
{
    servo_turn = turn;
}

/* 帧解析更新接口：mode.c 收到 0x04 帧后调用
 *   像素偏移量（int16，画面中心为 0）→ 存入测量值，PID 下一拍使用 */
void Servo_UpdateMeasure(int16_t x_px, int16_t y_px)
{
    servo_info.x_measure = (float)x_px;
    servo_info.y_measure = (float)y_px;
    servo_info.find = 1;
}

/* 增量式 PID：返回本周期角度偏移量（单位 CCR 计数）
 *   out = kp*err + kd*derr/dt   （I 项未启用，先跑通再调）
 *   err 量纲为像素，kp 负责像素→角度增益 */
float X_Servo_PID_Compute(float x_measure, float x_goal, float dt)
{
    float err = x_measure - x_goal;
    x_v_err_sum += err;

    float derr = (err - x_last_err) / dt;
    x_last_err = err;

    return x_kp * err + x_kd * derr;
}



float Y_Servo_PID_Compute(float y_measure, float y_goal, float dt)
{
    float err = y_measure - y_goal;
    y_v_err_sum += err;

    float derr = (err - y_last_err) / dt;
    y_last_err = err;

    return y_kp * err + y_kd * derr;
}


/* 每 20ms：增量累加 → 绝对角度位置 → 限幅 → 输出 */
void servo_method(void)
{
    /* X：水平 Pan (PB8 = TIM4_CH3) */
    float diff_X = X_Servo_PID_Compute(servo_info.x_measure, servo_info.x_goal, 0.020f);
    pre_X_PWM += (int)diff_X;
    X_PWM(pre_X_PWM);

    /* Y：竖直 Tilt (PB9 = TIM4_CH4) */
    float diff_Y = Y_Servo_PID_Compute(servo_info.y_measure, servo_info.y_goal, 0.020f);
    pre_Y_PWM += (int)diff_Y;
    Y_PWM(pre_Y_PWM);
}

void X_PWM(int ccr)
{
    if (ccr >= SERVO_MAX_CCR)
        ccr = SERVO_MAX_CCR;
    else if (ccr <= SERVO_MIN_CCR)
        ccr = SERVO_MIN_CCR;

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint16_t)ccr);   /* PB8 水平 Pan */
}

void Y_PWM(int ccr)
{
    if (ccr >= SERVO_MAX_CCR)
        ccr = SERVO_MAX_CCR;
    else if (ccr <= SERVO_MIN_CCR)
        ccr = SERVO_MIN_CCR;

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, (uint16_t)ccr);   /* PB9 竖直 Tilt */
}
