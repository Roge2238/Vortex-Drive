#include "mode.h"
#include "cmd.h"
#include "tim.h"
#include "pid.h"
#include "motor.h"
#include "string.h"

#define SIZE 10
#define AUTO_LEN 4

Mode_t last_mode = MODE_AUTO;

uint8_t tmp_buf[SIZE];
int tmp_buf_cnt = 0;

extern volatile CmdState cmdState;
extern PID_t pid_left;
extern PID_t pid_right;
extern PID_t pid_steer;
extern PID_t pid_speed;


//实现环形缓冲区 暂存消息 串口中断使用
int8_t push_uart_buf(uint8_t data)
{
    if((uart_buf.rw + 1) % UART_BUF_LEN == uart_buf.rd) return -1;
    uart_buf.uart_buf[uart_buf.rw] = data;
    uart_buf.rw = (uart_buf.rw + 1) % UART_BUF_LEN;
    return 0;
}

//实现环形缓冲区 取出消息
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
    if(get_uart_buf(&tmp)) return;  /* 返回非0表示缓冲区空，无数据可处理 */

    switch (state)
    {
    case WAIT_AA:
        if(tmp == MAGIC_HEAD)
        {
            state = READ_TYPE;
        }
        break;

    case READ_TYPE:
        type = tmp;
        if(type != CMD_TYPE && type != AUTO_TYPE && type != LOST_TYPE)
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

        if(mode != last_mode)
        {
            last_mode = mode;
            Mode_switch(); /* 切换模式 进行电机和缓冲区重置 */
            state = WAIT_AA;
            tmp_buf_cnt = 0;
            cv.error = 0;
            cv.area = 0;
            break;
        }

        tmp_buf_cnt = 0;
        state = READ_DATA;
        break;

    case READ_DATA:
        /* 防越界：丢弃整帧 */
        if(tmp_buf_cnt >= SIZE)
        {
            state = WAIT_AA;
            tmp_buf_cnt = 0;
            break;
        }

        tmp_buf[tmp_buf_cnt++] = tmp;
        if(type == CMD_TYPE && tmp_buf_cnt == CMD_LEN)
        {
            memcpy(cmd_buf, tmp_buf, CMD_LEN);
            /* 将原始字节转换为命令状态 */
            cmdState = (CmdState)build_keycode(cmd_buf);
            cmd_timeout_cnt = 0;  /* 收到新命令，重置超时计数 */
            tmp_buf_cnt = 0;
            state = WAIT_AA;
        }
        else if(type == AUTO_TYPE && tmp_buf_cnt == AUTO_LEN)
        {
            
            cv.error = (int16_t)((uint16_t)tmp_buf[0] | ((uint16_t)tmp_buf[1] << 8));
            cv.area  = (int16_t)((uint16_t)tmp_buf[2] | ((uint16_t)tmp_buf[3] << 8));
            cv_active = 1;          /* 标记：已收到有效数据，PID可以开始跑 */
            cmd_timeout_cnt = 0;    /* 重置超时计数 */
            tmp_buf_cnt = 0;
            state = WAIT_AA;
        }
        else if(type == LOST_TYPE && tmp_buf_cnt == AUTO_LEN)
        {
            /* 目标丢失：清零 cv + 清PID积分 + 停车 */
            cv.error = 0;
            cv.area = 0;
            cv_active = 0;          /* 关闭PID输出，TIM4中断会直接停车 */
            cmd_timeout_cnt = 0;    /* 仍重置超时，因为串口链路正常 */
            cur_left_pwm = 0.0f;    /* 斜坡状态清零：否则下一拍会用残留PWM重新驱动电机 */
            cur_right_pwm = 0.0f;
            PID_Reset(&pid_left);
            PID_Reset(&pid_right);
            PID_Reset(&pid_steer);
            PID_Reset(&pid_speed);
            set_drive_pwm(0.0f, 0.0f);
            tmp_buf_cnt = 0;
            state = WAIT_AA;
        }
        break;

    default:
        break;
    }
}
