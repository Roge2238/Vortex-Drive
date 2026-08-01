#include "mode.h"
#include "string.h"

#define SIZE 10
#define AUTO_LEN 4
extern uint8_t uart_buf[UART_BUF_LEN];
extern uint16_t rw;
extern uint16_t rd;
extern RecvState state;
extern uint8_t type;
extern cmd_buf[CMD_LEN];
extern int16_t error;
extern uint16_t area;


uint8_t tmp_buf[SIZE];
int tmp_buf_cnt = 0;


//实现环形缓冲区 暂存消息 
int8_t push_uart_buf(uint8_t)
{
    if((rw + 1)% UART_BUF_LEN == rd) return -1;
    uart_buf[rw] = data;
    rw = (rw + 1)% UART_BUF_LEN;
    return 0;



}

//实现环形缓冲区 取出消息 
uint8_t get_uart_buf(uint8_t *data)
{
    if(rw == rd) return -1;

    *data = uart_buf[rd];
    rd = (rd + 1)% UART_BUF_LEN;
    return 0;
}

void Mode_get(void)
{

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
            error = *(int16_t*)&tmp_buf[0];
            area = *(uint16_t*)&tmp_buf[2];
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