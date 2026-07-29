#include "stm32f1xx_hal.h"


typedef enum
{
	KEY_W 		= (1 << 0),
	KEY_S 		= (1 << 1),
	KEY_A			=	(1 << 2),
	KEY_D 		= (1 << 3),
	KEY_SHIFT = (1 << 4),
	KEY_C			= (1 << 5)
	
}KeyBit;


typedef enum {
    CMD_EMPTY            = 0,
    CMD_FORWARD          = KEY_W,
    CMD_FORWARD_LEFT     = KEY_W | KEY_A,
    CMD_FORWARD_RIGHT    = KEY_W | KEY_D,
    CMD_BACK_LEFT        = KEY_S | KEY_A,
    CMD_BACK_RIGHT       = KEY_S | KEY_D,
    CMD_BOOST            = KEY_W | KEY_SHIFT,
    CMD_BACK             = KEY_S,
    CMD_BRAKE            = KEY_C,
    CMD_FORWARD_LEFT_BOOST  = KEY_W | KEY_A | KEY_SHIFT,
    CMD_FORWARD_RIGHT_BOOST = KEY_W | KEY_D | KEY_SHIFT,
    CMD_BACK_LEFT_BOOST     = KEY_S | KEY_A | KEY_SHIFT,
    CMD_BACK_RIGHT_BOOST    = KEY_S | KEY_D | KEY_SHIFT,
    CMD_BACK_BOOST          = KEY_S | KEY_SHIFT,
    CMD_TURN_LEFT           = KEY_A,
    CMD_TURN_RIGHT          = KEY_D,
    CMD_TURN_LEFT_BOOST     = KEY_A | KEY_SHIFT,
    CMD_TURN_RIGHT_BOOST    = KEY_D | KEY_SHIFT,
    CMD_INVALID          = 0xFF
} CmdState;


uint8_t build_keycode(uint8_t * codebuf);

void cmd_method(volatile CmdState *state);




























