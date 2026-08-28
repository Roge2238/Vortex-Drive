#include "mode.h"
#include "cmd.h"
#include "tim.h"
#include "pid.h"
#include "motor.h"
#include "string.h"
#include "servo.h"

#define SIZE 10
#define AUTO_LEN 4
#define SERVO_LEN 4
#define SERVO_TEST_LEN 2

Mode_t last_mode = MODE_AUTO;// 初始化为自动模式

uint8_t tmp_buf[SIZE];
int tmp_buf_cnt = 0;

extern volatile CmdState cmdState;
extern PID_t pid_left;
extern PID_t pid_right;
extern PID_t pid_steer;
extern PID_t pid_speed;
extern float cur_left_pwm;
extern float cur_right_pwm;

RecvState state = WAIT_AA;

//实现环形缓冲区 暂存消息 串口中断使用
int8_t push_uart_buf(uint8_t data)
{
    if((uart_buf.rw + 1) % UART_BUF_LEN == uart_buf.rd) return -1;
    uart_buf.uart_buf[uart_buf.rw] = data;
    uart_buf.rw = (uart_buf.rw + 1) % UART_BUF_LEN;
    return 0;
}

// 取出消息
int8_t get_uart_buf(uint8_t *data)
{
    if(uart_buf.rw == uart_buf.rd) return -1;

    *data = uart_buf.uart_buf[uart_buf.rd];
    uart_buf.rd = (uart_buf.rd + 1) % UART_BUF_LEN;
    return 0;
}

static void reset_buf(void)
{
    /* 关中断保护，防止与 UART 中断 push_uart_buf 竞态 */
    __disable_irq();
    uart_buf.rw = 0;
    uart_buf.rd = 0;
    memset(uart_buf.uart_buf, 0, UART_BUF_LEN);
    __enable_irq();
}


static void motor_reset(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);//M1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);//M2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);//M3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);//M4
}

void cmd_buf_reset(void)
{
    memset(cmd_buf, 0, CMD_LEN);
}

void Mode_switch(void)
{
    cmd_buf_reset();
    motor_reset();
    reset_buf();
    /* 清残留控制状态，防止切回后第一拍用旧值驱动电机 */
    cmdState = CMD_EMPTY;
    cv_active = 0;
    cur_left_pwm = 0.0f;
    cur_right_pwm = 0.0f;
    PID_Reset(&pid_left);
    PID_Reset(&pid_right);
    PID_Reset(&pid_steer);
    PID_Reset(&pid_speed);
    HAL_Delay(25);
}

//主循环调用
void frame_task(void)
{
    uint8_t tmp;
    if(get_uart_buf(&tmp)) return;  

    switch (state)// 经过 多方选择 采用状态机处理 
    {
    case WAIT_AA:
        if(tmp == MAGIC_HEAD)
        {
            state = READ_TYPE;
        }
        break;

    case READ_TYPE:
        type = tmp;
        if(type != CMD_TYPE && type != AUTO_TYPE && type != LOST_TYPE && type != SERVO_TURN && type != SERVO_TEST)
        {

            state = WAIT_AA;
            break;
        }

        if(type == CMD_TYPE)
        {
            mode = MODE_CMD;
        }
        else if(type == AUTO_TYPE || type == LOST_TYPE)
        {
            mode = MODE_AUTO;
        }
        else if(type == SERVO_TURN)
        {
            if(!Get_Servo_turn())
            Set_Servo_turn(true);//开启舵机开关


        }
        else if(type == SERVO_TEST)
        {
            // 测试模式：关PID闭环，改开环直控，避免中断里覆盖测试角度
            if(Get_Servo_turn())
            Set_Servo_turn(false);
        }
        if(mode != last_mode)
        {
            last_mode = mode;
            Mode_switch(); /* 切换模式 进行电机和缓冲区重置 */
            state = WAIT_AA; // 当前帧丢掉
            tmp_buf_cnt = 0;
            cv.error = 0;
            cv.area = 0;
            break;
        }

        tmp_buf_cnt = 0;
        state = READ_DATA;
        break;

    case READ_DATA:
        
        if(tmp_buf_cnt >= SIZE) // 不正常就丢 重置为WAIT_AA 等下一个
        {
            state = WAIT_AA;
            tmp_buf_cnt = 0;
            break;
        }

        tmp_buf[tmp_buf_cnt++] = tmp;
        if(type == CMD_TYPE && tmp_buf_cnt == CMD_LEN)
        {
            memcpy(cmd_buf, tmp_buf, CMD_LEN);
           
            cmdState = (CmdState)build_keycode(cmd_buf);
            cmd_timeout_cnt = 0;  /* 收到新命令，重置超时计数 */
            tmp_buf_cnt = 0;
            state = WAIT_AA;
        }
        else if(type == AUTO_TYPE && tmp_buf_cnt == AUTO_LEN)
        {
            
            cv.error = (int16_t)((uint16_t)tmp_buf[0] | ((uint16_t)tmp_buf[1] << 8));
            cv.area  = (int16_t)((uint16_t)tmp_buf[2] | ((uint16_t)tmp_buf[3] << 8));
            cv_active = 1;        //
            cmd_timeout_cnt = 0;    /* 重置超时计数 */
            tmp_buf_cnt = 0;
            state = WAIT_AA;
        }
        else if(type == LOST_TYPE && tmp_buf_cnt == AUTO_LEN)
        {
            // 目标丢失：清零 cv   清PID积分  停车 
            cv.error = 0;
            cv.area = 0;
            cv_active = 0;         
            cmd_timeout_cnt = 0;    
            cur_left_pwm = 0.0f;   
            cur_right_pwm = 0.0f;
            PID_Reset(&pid_left);
            PID_Reset(&pid_right);
            PID_Reset(&pid_steer);
            PID_Reset(&pid_speed);
            set_drive_pwm(0.0f, 0.0f); 
            tmp_buf_cnt = 0;
            state = WAIT_AA;
        }
        else if(type == SERVO_TURN && tmp_buf_cnt == SERVO_LEN) 
        {
            
            Servo_UpdateMeasure(
                (int16_t)((uint16_t)tmp_buf[0] | ((uint16_t)tmp_buf[1] << 8)),
                (int16_t)((uint16_t)tmp_buf[2] | ((uint16_t)tmp_buf[3] << 8)));

            tmp_buf_cnt = 0;
            state = WAIT_AA;
        }
        else if(type == SERVO_TEST && tmp_buf_cnt == SERVO_TEST_LEN)
        {
            // 测试模式
            Servo_SetAngle(tmp_buf[0], tmp_buf[1]);

            tmp_buf_cnt = 0;
            state = WAIT_AA;
        }
        break;

    default:
        break;
    }
}
