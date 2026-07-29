#include "cmd.h"
#include "motor.h"

uint8_t build_keycode(uint8_t * codebuf)
{
	uint8_t code = 0;
	
	if(codebuf[0]) code|=KEY_W;
	if(codebuf[1]) code|=KEY_S;
	if(codebuf[2]) code|=KEY_A;
	if(codebuf[3]) code|=KEY_D;
	if(codebuf[4]) code|=KEY_SHIFT;
	if(codebuf[5]) code|=KEY_C;
	return code;
	
}


void cmd_method(volatile CmdState *key_code)
{
    switch (*key_code)
    {
        case CMD_FORWARD:
            move_forward();
            break;

        case CMD_FORWARD_LEFT:
            move_forward_left();
            break;

        case CMD_FORWARD_RIGHT:
            move_forward_right();
            break;

        case CMD_BOOST:
            move_boost();
            break;

        case CMD_BACK:
            move_back();
            break;

        case CMD_BACK_LEFT:
            move_back_left();
            break;

        case CMD_BACK_RIGHT:
            move_back_right();
            break;

        case CMD_BRAKE:
            move_stop();
            break;

        case CMD_FORWARD_LEFT_BOOST:
            move_forward_left_boost();
            break;

        case CMD_FORWARD_RIGHT_BOOST:
            move_forward_right_boost();
            break;

        case CMD_BACK_LEFT_BOOST:
            move_back_left_boost();
            break;

        case CMD_BACK_RIGHT_BOOST:
            move_back_right_boost();
            break;

        case CMD_BACK_BOOST:
            move_back_boost();
            break;

        case CMD_TURN_LEFT:
            move_turn_left();
            break;

        case CMD_TURN_RIGHT:
            move_turn_right();
            break;

        case CMD_TURN_LEFT_BOOST:
            move_turn_left_boost();
            break;

        case CMD_TURN_RIGHT_BOOST:
            move_turn_right_boost();
            break;

        case CMD_INVALID:
        case CMD_EMPTY:
            move_slide();
            break;

        default: 
            /* 未知命令或冲突按键组合，安全滑行停止 */
            move_slide();
            break;
    }
}



















