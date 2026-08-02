#include "mode.h"
#include "string.h"

#define SIZE 10
#define AUTO_LEN 4
extern UartBuf_t uart_buf;
extern RecvState state;
extern uint8_t type;
extern cmd_buf[CMD_LEN];

extern CV_t cv;


 extern Mode_t mode;
 Mode_t last_mode = MODE_CMD;

uint8_t tmp_buf[SIZE];
int tmp_buf_cnt = 0;


//实现环形缓冲区 暂存消息 串口中断使用 
int8_t push_uart_buf(uint8_t data)
{
    if((uart_buf.rw + 1)% UART_BUF_LEN == uart_buf.rd) return -1;
    uart_buf.rw = data;
    uart_buf.rw++;
    return 0;
}

//实现环形缓冲区 取出消息 
static uint8_t get_uart_buf(uint8_t *data)
{
    if(uart_buf.rw == uart_buf.rd) return -1;

    *data = uart_buf[uart_buf.rd];
    uart_buf.rd++;
    return 0;
}

static void reset_buf()
{
    uart_buf.rw = 0;
    uart_buf.rd = 0;
    memset(uart_buf.uart_buf, 0, UART_BUF_LEN);
}


static void motor_reset()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);//M1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);//M2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);//M3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);//M4

}

void cmd_buf_reset()
{
    memset(cmd_buf, 0, CMD_LEN);
}

void Mode_switch()
{
    cmd_buf_reset();
    motor_reset();
    HAL_Delay(25);
    reset_buf();
}

//主循环调用 
void frame_task(void)
{
    uint8_t tmp;
    if(!get_uart_buf(&tmp)) return ;

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
        if(type == CMD_TYPE)
        {
            mode = MODE_CMD;
        }
        else if(type == AUTO_TYPE)
        {
            mode = MODE_AUTO;
        }

        if(mode != last_mode)
        {
            last_mode = mode;
            Mode_switch(); //切换模式 进行电机和缓冲区重置
            state = WAIT_AA;
            tmp_buf_cnt = 0;
            cv.error = 0;
            cv.area = 0;
            
            continue;
        }

        tmp_buf_cnt = 0;
        state = READ_DATA;
        break;
        
    case READ_DATA:
        tmp_buf[tmp_buf_cnt++] = tmp;
        if(type == CMD_TYPE&&tmp_buf_cnt == CMD_LEN)
        {
            memcpy(cmd_buf, tmp_buf, CMD_LEN);

            tmp_buf_cnt = 0;
            state = WAIT_AA;
        }
        else if(type == AUTO_TYPE&&tmp_buf_cnt == AUTO_LEN)
        {
            cv.error = *(int16_t*)&tmp_buf[0];
            cv.area = *(uint16_t*)&tmp_buf[2];
            tmp_buf_cnt = 0;
            state = WAIT_AA;
        }
        break;

    default:
        break;
    }



}
/*void send_auto(int16_t err, uint16_t area)
{
    txbuf[0] = MAGIC;
    txbuf[1] = MSG_AUTO;
    // 直接把两个变量内存复制进去，小端原样发
    memcpy(&txbuf[2], &err, 2);
    memcpy(&txbuf[4], &area, 2);*/
    

void motor_reset()
{



}