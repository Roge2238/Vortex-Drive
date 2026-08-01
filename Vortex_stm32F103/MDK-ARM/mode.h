#include "stm32f1xx_hal.h"
//消息格式 [0xAA 魔数(1)] [LEN(2字节大端)] [MSG_TYPE(1)] [载荷数据N字节]
#define MAGIC_HEAD 0xAA

#define CMD_TYPE 0x01
#define AUTO_TYPE 0x02
#define MODE_SWITCH 0x03

#define UART_BUF_LEN 64 

extern uint8_t uart_buf[UART_BUF_LEN];
 


typedef enum 
{
    CMD = 0,
    AUTO = 1,
}Mode_t;


typedef enum MyEnum
{
    WAIT_AA,
    READ_TYPE,
    READ_DATA,

}RecvState;

uint8_t push_uart_buf(uint8_t);

uint8_t get_uart_buf(uint8_t *data);

void frame_task(void);

void Mode_get(void);






