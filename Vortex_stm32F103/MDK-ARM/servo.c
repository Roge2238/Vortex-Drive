#include "servo.h"
#include "tim.h"
#include <math.h>

/* ===== 舵机增量式 PID 控制 =====
 * 架构：0x04(SERVO_TURN) 帧携带目标像素坐标 (cx,cy) → STM32 内部定 goal(画面中心)
 *       增量式 PID 把像素误差 err=measure-goal 换算成角度偏移量(CCR) → 累加到绝对角度位置
 *       pre_X/Y_PWM 即"当前舵机绝对角度(CCR)"，每周期 +增量 得到新位置
 *
 * 时序：由 TIM4 20ms 中断驱动 (servo_turn==true 时调用 servo_method)
 */

/* ——— 方向校正 ———
 * 机械约定：Pan 0°=最右 / 90°=中 / 180°=最左；Tilt 0°=朝天 / 90°=水平
 * 目标偏右(err>0) → Pan 需往右转 → 角度减小 → CCR 减小 → 增量取负
 */
#define SERVO_X_REVERSE 1

/* 目标点默认画面中心；PID 内部 err = measure - goal */
static Servo_Info servo_info = {
    .x_goal = SERVO_CENTER_PX_X,
    .y_goal = SERVO_CENTER_PX_Y,
};

static bool servo_turn = false;

/* ——— 增量式 PID 状态（X=水平Pan, Y=竖直Tilt） ——— */
// 先不加Ki 纯PD控制 
static float x_kp, x_ki, x_kd;
static float x_last_err;
static float x_v_err_sum;              /* 误差累积  绝对角度位置基准  */

static float y_kp, y_ki, y_kd;
static float y_last_err;
static float y_v_err_sum;

// ——— 舵机绝对角度位置（当前 CCR 值） 
static int pre_X_PWM = SERVO_CENTER_CCR; /* 初始化为中位 90° */
static int pre_Y_PWM = SERVO_CENTER_CCR;

void Servo_PID_Init(void)
{
    /* 标定实测：24px/°（两轴一致）→ 1px ≈ 0.042 CCR
     * kp：1px 误差单周期输出 0.05 CCR  0.042* 1.2 = 0.05 满补偿 
     **/
    x_kp = 0.05f;
    x_ki = 0.0f;
    x_kd = 0.001f;
    y_kp = 0.05f;
    y_ki = 0.0f;
    y_kd = 0.001f;

    x_last_err = 0.0f;
    x_v_err_sum = 0.0f;
    y_last_err = 0.0f;
    y_v_err_sum = 0.0f;// 不再用I项 

    /* 初始化位置复位到中位 90 */
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


//获取最新坐标 存入servo_info 中
void Servo_UpdateMeasure(int16_t x_px, int16_t y_px)
{
    servo_info.x_measure = (float)x_px;
    servo_info.y_measure = (float)y_px;
    if (!servo_info.find)
    {
        /* 首帧同步 D 项记忆：避免 last_err=0 导致第一拍 derr 巨大、舵机猛甩 */
        x_last_err = (float)x_px - servo_info.x_goal;
        y_last_err = (float)y_px - servo_info.y_goal;
    }
    servo_info.find = 1;
}

// 测试/开环接口：mode.c 收到 0x05(SERVO_TEST) 帧后调用
//   直接按角度(0~180)输出，不经过 PID，用于验证舵机方向/行程/限幅
void Servo_SetAngle(uint8_t pan_angle, uint8_t tilt_angle)
{
    // 限幅：只接受 0~180
    if (pan_angle > 180)  pan_angle = 180;
    if (tilt_angle > 180) tilt_angle = 180;

    // 角度 → CCR 线性映射：0°→50, 90°→150, 180°→250
    pre_X_PWM = SERVO_MIN_CCR + (int)pan_angle  * (SERVO_MAX_CCR - SERVO_MIN_CCR) / 180;
    pre_Y_PWM = SERVO_MIN_CCR + (int)tilt_angle * (SERVO_MAX_CCR - SERVO_MIN_CCR) / 180;

    X_PWM(pre_X_PWM);
    Y_PWM(pre_Y_PWM);
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
    /* 从未收到目标帧：保持原位，不驱动 */
    if (!servo_info.find)
        return;

    float diff_X = X_Servo_PID_Compute(servo_info.x_measure, servo_info.x_goal, 0.020f);
    float diff_Y = Y_Servo_PID_Compute(servo_info.y_measure, servo_info.y_goal, 0.020f);

    /* 死区：|err| < 阈值 视为已对准目标点 → 增量钳 0（PID 状态照常更新）
     * 注意判断基于 err=measure-goal 而非 measure，goal 非 0 时依然正确 */
    if (fabsf(servo_info.x_measure - servo_info.x_goal) < SERVO_CENTER_DEADZONE_PX) diff_X = 0.0f;
    if (fabsf(servo_info.y_measure - servo_info.y_goal) < SERVO_CENTER_DEADZONE_PX) diff_Y = 0.0f;

    /* 增量限幅：每 20ms 最多走 ±2 CCR(≈2°)，防止 PID 输出超过舵机物理能力的大甩动
     * 舵机速度约 400°/s → 20ms 极限 ~8°；限 2° 让启动/逼近都平滑 */
    if (diff_X > 2.0f)  diff_X = 2.0f;
    if (diff_X < -2.0f) diff_X = -2.0f;
    if (diff_Y > 2.0f)  diff_Y = 2.0f;
    if (diff_Y < -2.0f) diff_Y = -2.0f;

#if SERVO_X_REVERSE
    diff_X = -diff_X;
#endif

    /* X：水平 Pan (PB8 = TIM4_CH3) */
    pre_X_PWM += (diff_X >= 0) ? (int)(diff_X + 0.5f) : (int)(diff_X - 0.5f);

    /* Y：竖直 Tilt (PB9 = TIM4_CH4) */
    pre_Y_PWM += (diff_Y >= 0) ? (int)(diff_Y + 0.5f) : (int)(diff_Y - 0.5f);

    /* 立即限幅位置状态：防止虚拟越界累计，避免误差反向时的回程延迟 */
    if (pre_X_PWM < SERVO_MIN_CCR) pre_X_PWM = SERVO_MIN_CCR;
    if (pre_X_PWM > SERVO_MAX_CCR) pre_X_PWM = SERVO_MAX_CCR;
    if (pre_Y_PWM < SERVO_MIN_CCR) pre_Y_PWM = SERVO_MIN_CCR;
    if (pre_Y_PWM > SERVO_MAX_CCR) pre_Y_PWM = SERVO_MAX_CCR;

    X_PWM(pre_X_PWM);
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
