#ifndef __SERVO_H__
#define __SERVO_H__

#include "stm32f1xx_hal.h"
#include <stdbool.h>

/* ===== 舵机角度 ↔ CCR 换算（TIM4: ARR=1999, 20ms=50Hz, 1ms=100计数） =====
 *   0°  → 0.5ms → CCR=50
 *   90° → 1.5ms → CCR=150   (中位)
 *   180°→ 2.5ms → CCR=250
  */
#define SERVO_MIN_CCR     60
#define SERVO_CENTER_CCR  150
#define SERVO_MAX_CCR     240

/* 中心死区（像素）：|err| < 该值 视为已对准目标点，PID 增量钳 0 */
#define SERVO_CENTER_DEADZONE_PX  10

/* 画面中心坐标（像素）：0x04 帧传目标像素坐标 cx,cy，PID 内部 err = measure - goal
 * 必须与上位机画面分辨率一致（640×480 → 320/240），换分辨率需同步修改 */
#define SERVO_CENTER_PX_X  320
#define SERVO_CENTER_PX_Y  240

// 舵机测量/目标信息
typedef struct
{
    float x_measure;   
    float y_measure;   
    float x_goal;      // 目标点坐标，默认画面中心 (SERVO_CENTER_PX_X)
    float y_goal;      // 默认画面中心 (SERVO_CENTER_PX_Y)
    uint8_t find;      // 1=检测到目标
} Servo_Info;

/* PID 初始化*/
void Servo_PID_Init(void);

/* 帧解析更新接口：mode.c 收到 0x04 帧后调用 更新舵机测量值 */
void Servo_UpdateMeasure(int16_t x_px, int16_t y_px);

// 测试/开环接口：直接设定角度(0~180)，角度→CCR 并输出，不经过 PID
void Servo_SetAngle(uint8_t pan_angle, uint8_t tilt_angle);

/* 舵机转动开关 */   // 计划由Qt上位机操控 
bool Get_Servo_turn(void);
void Set_Servo_turn(bool turn);

/* 增量  PID计算 */
float X_Servo_PID_Compute(float x_measure, float x_goal, float dt);
float Y_Servo_PID_Compute(float y_measure, float y_goal, float dt);

/* 每 20ms 调用 进行舵机控制*/
void servo_method(void);

/* 单通道 PWM 输出（带限幅） */
void X_PWM(int ccr);
void Y_PWM(int ccr);

#endif /* __SERVO_H__ */
