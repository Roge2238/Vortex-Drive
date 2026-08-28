#ifndef SERVO_CV_H
#define SERVO_CV_H

#include <cstdint>
#include <atomic>

/*
 * servo_cv — 舵机云台自动跟踪（servo auto）
 *
 * 独立摄像头（云台相机，随云台转动）通过 VideoCapture 本地采集，
 * 每帧检测红色圆盘质心 (cx,cy)，更新 0x04 SERVO_TURN 帧
 *   [0xAA][0x04][cx_lo][cx_hi][cy_lo][cy_hi]   （6字节，int16 小端）
 * 串口线程周期转发给 STM32，STM32 端 Servo_UpdateMeasure 驱动云台 PID 跟踪。
 *
 * 与底盘摄像头完全隔离：
 *   - 独立线程 + 独立设备（默认 /dev/video1），不占 /dev/video0
 *   - 画面只做本地检测，不推流；gst 推流管道保持底盘摄像头不动
 * 若云台摄像头设备号不是 /dev/video1，改 SERVO_CAM_DEVICE 即可。
 */

#define SERVO_CAM_DEVICE "/dev/video1"

/* 舵机跟踪检测结果（像素坐标） */
struct servo_cv_info
{
    bool is_find = false;   /* 当前帧是否检测到目标 */
    int cx = 0;             /* 质心 x（0~639） */
    int cy = 0;             /* 质心 y（0~479） */
};

/* 0x04 SERVO_TURN 帧模板（串口线程直访，6字节） */
extern uint8_t servo_cv_cmd[6];

/* 舵机自动跟踪开关：常开（默认 true） */
extern std::atomic<bool> servo_turn;

/* 查询伺服是否可发帧：开关打开 且 已有有效检测结果 */
bool Get_Servo_turn();

/* 检测线程发布一帧结果：found=true 更新 0x04 帧；丢失保留上一帧 */
void servo_cv_publish(bool found, int cx, int cy);

/* 独立舵机检测线程（main 启动，go_running 控制退出） */
void servo_cv_thread();

#endif // SERVO_CV_H
