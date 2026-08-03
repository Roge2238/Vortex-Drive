#ifndef __MODE_H__
#define __MODE_H__

#include "stm32f1xx_hal.h"
//消息格式 [0xAA 魔数(1)] [LEN(2字节大端)] [MSG_TYPE(1)] [载荷数据N字节]
#define MAGIC_HEAD 0xAA

#define CMD_TYPE 0x01
#define AUTO_TYPE 0x02
#define MODE_SWITCH 0x03

#define UART_BUF_LEN 64
#define CMD_LEN 6




typedef enum
{
    MODE_CMD = 0,
    MODE_AUTO = 1,
}Mode_t;


typedef enum
{
    WAIT_AA,
    READ_TYPE,
    READ_DATA,

}RecvState;


typedef struct
{
    uint8_t uart_buf[UART_BUF_LEN];
    int rw;
    int rd;

}UartBuf_t;

typedef struct
{
    int16_t area;   /* 实际是 area error = TARGET_AREA - actual_area，可为负 */
    int16_t error;

}CV_t;

extern UartBuf_t uart_buf;
extern RecvState state;
extern uint8_t type;
extern uint8_t cmd_buf[CMD_LEN];
extern CV_t cv;
extern Mode_t mode;

extern volatile uint8_t cmd_timeout_cnt;
extern volatile uint8_t cv_active;        /* 收到首个AUTO帧后置1，PID才开始跑 */

int8_t push_uart_buf(uint8_t data);

int8_t get_uart_buf(uint8_t *data);

void frame_task(void);

void cmd_buf_reset(void);

void Mode_switch(void);

#endif /* __MODE_H__ */
