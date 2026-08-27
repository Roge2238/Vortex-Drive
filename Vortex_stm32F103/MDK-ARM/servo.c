#include "servo.h"
#include "tim.h"
#include <math.h>


#define SERVO_X_REVERSE 1


#define SERVO_DEADZONE_HYSTERESIS_PX  20 //设置滞回区 20px 
#define SERVO_APPROACH_ZONE_PX        50 //设置临近区 进行增量限幅 0.2°   
#define SERVO_APPROACH_MAX_DIFF       0.2f
#define SERVO_MAX_DIFF                2.0f

// 目标点默认画面中心；PID 内部 err = measure - goal   640 * 480像素
static Servo_Info servo_info = {
    .x_goal = SERVO_CENTER_PX_X,
    .y_goal = SERVO_CENTER_PX_Y,
};

static bool servo_turn = false;

/* ——— 增量式 PID 状态（X=水平Pan, Y=竖直Tilt） ——— */
// 先不加Ki 纯PD控制 
static float x_kp, x_ki, x_kd;
static float x_last_err;
static float x_v_err_sum;              // 误差累积  计算绝对舵机角度位置  

static float y_kp, y_ki, y_kd;
static float y_last_err;
static float y_v_err_sum;


static float pre_X_PWM = SERVO_CENTER_CCR; //初始化为中位 90° 
static float pre_Y_PWM = SERVO_CENTER_CCR;

// 滞回状态判定 
static bool x_in_deadzone = true;
static bool y_in_deadzone = true;

void Servo_PID_Init(void)
{
    // 标定实测：24px/°   1px ≈ 0.042 CCR
    //* kp：0.03 → 补偿率 72%  
    
  //X PID  
    x_kp = 0.005f;
    x_ki = 0.0f;
    x_kd = 0.009f;
  // Y PID
    y_kp = 0.001f;
    y_ki = 0.0f;
    y_kd = 0.026f;

    x_last_err = 0.0f;
    x_v_err_sum = 0.0f;
    y_last_err = 0.0f;
    y_v_err_sum = 0.0f;// 不再用I项 

    /* 滞回状态复位：初始处于死区（停止） */
    x_in_deadzone = true;
    y_in_deadzone = true;

    // 初始化位置复位到中位 90 
    pre_X_PWM = SERVO_CENTER_CCR;
    pre_Y_PWM = SERVO_CENTER_CCR;
    X_PWM((int)(pre_X_PWM + 0.5f));// 四舍五入
    Y_PWM((int)(pre_Y_PWM + 0.5f));
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
        /* Tagert 丢失  同步 D 项数据 */
        x_last_err = (float)x_px - servo_info.x_goal;
        y_last_err = (float)y_px - servo_info.y_goal;
    }
    servo_info.find = 1;
}

// 测试/开环接口：mode.c 收到 0x05(SERVO_TEST) 帧后调用
//   直接按角度(0~180)输出，不经过 PID   
void Servo_SetAngle(uint8_t pan_angle, uint8_t tilt_angle)
{
    // 限幅
    if (pan_angle > 180)  pan_angle = 180;
    if (tilt_angle > 180) tilt_angle = 180;

    // 角度 → CCR 线性映射：0° -> 50, 90° -> 150, 180° ->250
    pre_X_PWM = SERVO_MIN_CCR + (int)pan_angle  * (SERVO_MAX_CCR - SERVO_MIN_CCR) / 180;
    pre_Y_PWM = SERVO_MIN_CCR + (int)tilt_angle * (SERVO_MAX_CCR - SERVO_MIN_CCR) / 180;

    X_PWM((int)(pre_X_PWM + 0.5f));
    Y_PWM((int)(pre_Y_PWM + 0.5f));
}


/* 增量式 PID：返回本周期角度偏移量 CCR 
 *   out = kp*err + kd*derr/dt   
*/
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


//每20ms
void servo_method(void)
{
    //未收到目标帧：保持原位，不驱动 
    if (!servo_info.find)
        return;

    float diff_X = X_Servo_PID_Compute(servo_info.x_measure, servo_info.x_goal, 0.020f);
    float diff_Y = Y_Servo_PID_Compute(servo_info.y_measure, servo_info.y_goal, 0.020f);

    float err_x = servo_info.x_measure - servo_info.x_goal;
    float err_y = servo_info.y_measure - servo_info.y_goal;
    float abs_err_x = fabsf(err_x);
    float abs_err_y = fabsf(err_y);


    if (x_in_deadzone)
    {
        if (abs_err_x >= SERVO_DEADZONE_HYSTERESIS_PX) x_in_deadzone = false;
    }
    else
    {
        if (abs_err_x < SERVO_CENTER_DEADZONE_PX) x_in_deadzone = true;
    }
    if (x_in_deadzone) diff_X = 0.0f;

    if (y_in_deadzone)
    {
        if (abs_err_y >= SERVO_DEADZONE_HYSTERESIS_PX) y_in_deadzone = false;
    }
    else
    {
        if (abs_err_y < SERVO_CENTER_DEADZONE_PX) y_in_deadzone = true;
    }
    if (y_in_deadzone) diff_Y = 0.0f;

    //  临近减速区(10 - 50px)增量限幅 
    if (abs_err_x >= SERVO_CENTER_DEADZONE_PX && abs_err_x < SERVO_APPROACH_ZONE_PX)
    {
        if (diff_X >  SERVO_APPROACH_MAX_DIFF) diff_X =  SERVO_APPROACH_MAX_DIFF;
        if (diff_X < -SERVO_APPROACH_MAX_DIFF) diff_X = -SERVO_APPROACH_MAX_DIFF;
    }
    // 远区(>50px)：正常 PID，保险限幅 2度 */
    else if (abs_err_x >= SERVO_APPROACH_ZONE_PX)
    {
        if (diff_X >  SERVO_MAX_DIFF) diff_X =  SERVO_MAX_DIFF;
        if (diff_X < -SERVO_MAX_DIFF) diff_X = -SERVO_MAX_DIFF;
    }

    if (abs_err_y >= SERVO_CENTER_DEADZONE_PX && abs_err_y < SERVO_APPROACH_ZONE_PX)
    {
        if (diff_Y >  SERVO_APPROACH_MAX_DIFF) diff_Y =  SERVO_APPROACH_MAX_DIFF;
        if (diff_Y < -SERVO_APPROACH_MAX_DIFF) diff_Y = -SERVO_APPROACH_MAX_DIFF;
    }
    else if (abs_err_y >= SERVO_APPROACH_ZONE_PX)
    {
        if (diff_Y >  SERVO_MAX_DIFF) diff_Y =  SERVO_MAX_DIFF;
        if (diff_Y < -SERVO_MAX_DIFF) diff_Y = -SERVO_MAX_DIFF;
    }

#if SERVO_X_REVERSE
    diff_X = -diff_X;
#endif
    
    pre_X_PWM += diff_X;

    pre_Y_PWM += diff_Y;

    /* 限幅输出 */
    if (pre_X_PWM < SERVO_MIN_CCR) pre_X_PWM = SERVO_MIN_CCR;
    if (pre_X_PWM > SERVO_MAX_CCR) pre_X_PWM = SERVO_MAX_CCR;
    if (pre_Y_PWM < SERVO_MIN_CCR) pre_Y_PWM = SERVO_MIN_CCR;
    if (pre_Y_PWM > SERVO_MAX_CCR) pre_Y_PWM = SERVO_MAX_CCR;

    X_PWM((int)(pre_X_PWM + 0.5f));   /* 位置输出时才取整 */
    Y_PWM((int)(pre_Y_PWM + 0.5f));
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
