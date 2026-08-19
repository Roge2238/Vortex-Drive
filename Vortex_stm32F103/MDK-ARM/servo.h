#ifndef __SERVO_H__
#define __SERVO_H__

#include "stm32f1xx_hal.h"
#include <stdbool.h>

/* ===== 舵机角度 ↔ CCR 换算（TIM4: ARR=1999, 20ms=50Hz, 1ms=100计数） =====
 *   0°  → 0.5ms → CCR=50
 *   90° → 1.5ms → CCR=150   (中位)
 *   180°→ 2.5ms → CCR=250
 * 为避免极限位持续堵转，实际限幅范围可收窄（如 60~240） */
#define SERVO_MIN_CCR     50
#define SERVO_CENTER_CCR  150
#define SERVO_MAX_CCR     250

// 舵机测量/目标信息（内部状态，外部经接口函数访问）
typedef struct
{
    float x_measure;   // 0x04帧: 水平像素偏移 (+右/-左)
    float y_measure;   // 0x04帧: 竖直像素偏移 (+下/-上)
    float x_goal;      // 目标偏移，默认 0 = 画面中心
    float y_goal;
    uint8_t find;      // 1=检测到目标
} Servo_Info;

/* PID 初始化：装载 Kp/Ki/Kd，位置复位到中位 */
void Servo_PID_Init(void);

/* 帧解析更新接口：mode.c 收到 0x04 帧后调用，像素偏移 → 更新舵机测量值 */
void Servo_UpdateMeasure(int16_t x_px, int16_t y_px);

// 测试/开环接口：直接设定角度(0~180)，角度→CCR 并输出，不经过 PID
void Servo_SetAngle(uint8_t pan_angle, uint8_t tilt_angle);

/* 舵机转动开关 */
bool Get_Servo_turn(void);
void Set_Servo_turn(bool turn);

/* 增量式 PID：返回本周期角度偏移量（像素→CCR 的比例由 kp 决定） */
float X_Servo_PID_Compute(float x_measure, float x_goal, float dt);
float Y_Servo_PID_Compute(float y_measure, float y_goal, float dt);

/* 每 20ms 调用：PID → 累加绝对角度位置 → 输出 PWM */
void servo_method(void);

/* 单通道 PWM 输出（带限幅） */
void X_PWM(int ccr);
void Y_PWM(int ccr);

#endif /* __SERVO_H__ */
