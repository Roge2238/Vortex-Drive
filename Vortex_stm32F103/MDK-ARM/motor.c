#include "motor.h"

#define PWM_GET(tim, ch)       __HAL_TIM_GET_COMPARE((tim), (ch))

extern TIM_HandleTypeDef htim2;

void boost_pwm(TIM_HandleTypeDef* tim, uint32_t ch)
{
    uint32_t current_pwm = PWM_GET(tim, ch);
    if(current_pwm < 840)
    {
       current_pwm +=2;
    }
     __HAL_TIM_SET_COMPARE(tim, ch, current_pwm );
}

void move_forward()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_BASE_SPEED);//M1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_BASE_SPEED);//M2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_BASE_SPEED);//M3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_BASE_SPEED);//M4

    //M1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_SET);//-> AIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_RESET);//-> AIN 2

    //M2 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_RESET);//-> BIN 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , GPIO_PIN_SET);//-> BIN 2 (原PB2改为PA7)

    //M3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);//-> CIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);//-> CIN 2

    //M4 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 , GPIO_PIN_RESET);//-> DIN 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 , GPIO_PIN_SET);//-> DIN 2
}

void move_back()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_BASE_SPEED);//M1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_BASE_SPEED);//M2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_BASE_SPEED);//M3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_BASE_SPEED);//M4

    //M1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_RESET);//-> AIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_SET);//-> AIN 2

    //M2 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_SET);//-> BIN 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , GPIO_PIN_RESET);//-> BIN 2 (原PB2改为PA7)

    //M3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);//-> CIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//-> CIN 2

    //M4 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 , GPIO_PIN_SET);//-> DIN 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 , GPIO_PIN_RESET);//-> DIN 2
}

void move_forward_left()
{
    //M1 M3 快
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_BASE_SPEED);//M1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_OFFSET);//M2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_BASE_SPEED);//M3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_OFFSET);//M4

    //M1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_SET);//-> AIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_RESET);//-> AIN 2

    //M2 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_RESET);//-> BIN 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , GPIO_PIN_SET);//-> BIN 2 (原PB2改为PA7)

    //M3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);//-> CIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);//-> CIN 2

    //M4 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 , GPIO_PIN_RESET);//-> DIN 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 , GPIO_PIN_SET);//-> DIN 2
}

void move_forward_right()
{
    //M2 M4 快
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_OFFSET);//M1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_BASE_SPEED);//M2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_OFFSET);//M3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_BASE_SPEED);//M4

    //M1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_SET);//-> AIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_RESET);//-> AIN 2

    //M2 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_RESET);//-> BIN 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , GPIO_PIN_SET);//-> BIN 2 (原PB2改为PA7)

    //M3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);//-> CIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);//-> CIN 2

    //M4 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 , GPIO_PIN_RESET);//-> DIN 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 , GPIO_PIN_SET);//-> DIN 2
}

void move_back_left()
{
    //M1 M3 快
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_BASE_SPEED);//M1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_OFFSET);//M2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_BASE_SPEED);//M3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_OFFSET);//M4

    //M1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_RESET);//-> AIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_SET);//-> AIN 2

    //M2 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_SET);//-> BIN 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , GPIO_PIN_RESET);//-> BIN 2 (原PB2改为PA7)

    //M3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);//-> CIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//-> CIN 2

    //M4 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 , GPIO_PIN_SET);//-> DIN 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 , GPIO_PIN_RESET);//-> DIN 2
}

void move_back_right()
{
    //M2 M4 快
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_OFFSET);//M1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_BASE_SPEED);//M2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_OFFSET);//M3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_BASE_SPEED);//M4

    //M1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_RESET);//-> AIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_SET);//-> AIN 2

    //M2 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_SET);//-> BIN 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , GPIO_PIN_RESET);//-> BIN 2 (原PB2改为PA7)

    //M3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);//-> CIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//-> CIN 2

    //M4 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 , GPIO_PIN_SET);//-> DIN 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 , GPIO_PIN_RESET);//-> DIN 2
}

void move_stop()
{
    // PWM设为0 刹车状态 两侧
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);//M1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);//M2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);//M3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);//M4

    //M1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_SET);//-> AIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_SET);//-> AIN 2

    //M2
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_SET);//-> BIN 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , GPIO_PIN_SET);//-> BIN 2 (原PB2改为PA7)

    //M3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);//-> CIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//-> CIN 2

    //M4
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 , GPIO_PIN_SET);//-> DIN 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 , GPIO_PIN_SET);//-> DIN 2
}

void move_boost()
{
    if(PWM_GET(&htim2, TIM_CHANNEL_1) < PWM_BASE_SPEED)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_BASE_SPEED);
    if(PWM_GET(&htim2, TIM_CHANNEL_2) < PWM_BASE_SPEED)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_BASE_SPEED);
    if(PWM_GET(&htim2, TIM_CHANNEL_3) < PWM_BASE_SPEED)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_BASE_SPEED);
    if(PWM_GET(&htim2, TIM_CHANNEL_4) < PWM_BASE_SPEED)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_BASE_SPEED);

    boost_pwm(&htim2, TIM_CHANNEL_1 );//M1
    boost_pwm(&htim2, TIM_CHANNEL_2 );//M2
    boost_pwm(&htim2, TIM_CHANNEL_3 );//M3
    boost_pwm(&htim2, TIM_CHANNEL_4 );//M4

    //M1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_SET);//-> AIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_RESET);//-> AIN 2

    //M2 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_RESET);//-> BIN 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , GPIO_PIN_SET);//-> BIN 2 (原PB2改为PA7)

    //M3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);//-> CIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);//-> CIN 2

    //M4 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 , GPIO_PIN_RESET);//-> DIN 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 , GPIO_PIN_SET);//-> DIN 2
}

void slow_pwm(TIM_HandleTypeDef* tim, uint32_t ch)
{
    uint32_t current_pwm = PWM_GET(tim, ch);
    if(current_pwm > 0)
    {
       current_pwm -= 10;
    }
     __HAL_TIM_SET_COMPARE(tim, ch, current_pwm );
}

void move_coast()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);//M1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);//M2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);//M3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);//M4

    //M1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_RESET);//-> AIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_RESET);//-> AIN 2

    //M2
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_RESET);//-> BIN 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , GPIO_PIN_RESET);//-> BIN 2 (原PB2改为PA7)

    //M3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);//-> CIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);//-> CIN 2

    //M4
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 , GPIO_PIN_RESET);//-> DIN 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 , GPIO_PIN_RESET);//-> DIN 2
}

void move_slow()
{
    slow_pwm(&htim2, TIM_CHANNEL_1 );//M1
    slow_pwm(&htim2, TIM_CHANNEL_2 );//M2
    slow_pwm(&htim2, TIM_CHANNEL_3 );//M3
    slow_pwm(&htim2, TIM_CHANNEL_4 );//M4
}

extern int no_cmd_cnt ;

void move_slide()
{
    no_cmd_cnt++;
    if(no_cmd_cnt >= COAST_TIMEOUT_CNT)
    {
        move_stop();
        // 不再重置计数，保持停止状态直到收到新命令
    }
    else
    {
        move_coast();
    }
}

void move_forward_left_boost()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_BOOST_SPEED);//M1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_OFFSET);//M2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_BOOST_SPEED);//M3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_OFFSET);//M4

    //M1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_SET);//-> AIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_RESET);//-> AIN 2

    //M2 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_RESET);//-> BIN 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , GPIO_PIN_SET);//-> BIN 2 (原PB2改为PA7)

    //M3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);//-> CIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);//-> CIN 2

    //M4 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 , GPIO_PIN_RESET);//-> DIN 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 , GPIO_PIN_SET);//-> DIN 2
}

void move_forward_right_boost()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_OFFSET);//M1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_BOOST_SPEED);//M2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_OFFSET);//M3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_BOOST_SPEED);//M4

    //M1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_SET);//-> AIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_RESET);//-> AIN 2

    //M2 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_RESET);//-> BIN 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , GPIO_PIN_SET);//-> BIN 2 (原PB2改为PA7)

    //M3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);//-> CIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);//-> CIN 2

    //M4 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 , GPIO_PIN_RESET);//-> DIN 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 , GPIO_PIN_SET);//-> DIN 2
}

void move_back_left_boost()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_BOOST_SPEED);//M1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_OFFSET);//M2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_BOOST_SPEED);//M3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_OFFSET);//M4

    //M1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_RESET);//-> AIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_SET);//-> AIN 2

    //M2 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_SET);//-> BIN 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , GPIO_PIN_RESET);//-> BIN 2 (原PB2改为PA7)

    //M3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);//-> CIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//-> CIN 2

    //M4 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 , GPIO_PIN_SET);//-> DIN 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 , GPIO_PIN_RESET);//-> DIN 2
}

void move_back_right_boost()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_OFFSET);//M1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_BOOST_SPEED);//M2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_OFFSET);//M3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_BOOST_SPEED);//M4

    //M1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_RESET);//-> AIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_SET);//-> AIN 2

    //M2 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_SET);//-> BIN 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , GPIO_PIN_RESET);//-> BIN 2 (原PB2改为PA7)

    //M3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);//-> CIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//-> CIN 2

    //M4 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 , GPIO_PIN_SET);//-> DIN 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 , GPIO_PIN_RESET);//-> DIN 2
}

void move_back_boost()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_BOOST_SPEED);//M1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_BOOST_SPEED);//M2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_BOOST_SPEED);//M3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_BOOST_SPEED);//M4

    //M1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_RESET);//-> AIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_SET);//-> AIN 2

    //M2 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_SET);//-> BIN 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , GPIO_PIN_RESET);//-> BIN 2 (原PB2改为PA7)

    //M3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);//-> CIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//-> CIN 2

    //M4 (右侧电机，方向反转)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 , GPIO_PIN_SET);//-> DIN 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 , GPIO_PIN_RESET);//-> DIN 2
}

void move_turn_left()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_TURN_SPEED);//M1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_TURN_SPEED);//M2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_TURN_SPEED);//M3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_TURN_SPEED);//M4

    //M1 (左前轮向前)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_SET);//-> AIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_RESET);//-> AIN 2

    //M2 (右前轮向后，因安装反向，需给正向信号)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_SET);//-> BIN 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , GPIO_PIN_RESET);//-> BIN 2 (原PB2改为PA7)

    //M3 (左后轮向前)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);//-> CIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);//-> CIN 2

    //M4 (右后轮向后，因安装反向，需给正向信号)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 , GPIO_PIN_SET);//-> DIN 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 , GPIO_PIN_RESET);//-> DIN 2
}

void move_turn_right()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_TURN_SPEED);//M1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_TURN_SPEED);//M2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_TURN_SPEED);//M3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_TURN_SPEED);//M4

    //M1 (左前轮向后)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_RESET);//-> AIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_SET);//-> AIN 2

    //M2 (右前轮向前，因安装反向，需给反向信号)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_RESET);//-> BIN 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , GPIO_PIN_SET);//-> BIN 2 (原PB2改为PA7)

    //M3 (左后轮向后)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);//-> CIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//-> CIN 2

    //M4 (右后轮向前，因安装反向，需给反向信号)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 , GPIO_PIN_RESET);//-> DIN 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 , GPIO_PIN_SET);//-> DIN 2
}

void move_turn_left_boost()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_TURN_BOOST_SPEED);//M1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_TURN_BOOST_SPEED);//M2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_TURN_BOOST_SPEED);//M3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_TURN_BOOST_SPEED);//M4

    //M1 (左前轮向前)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_SET);//-> AIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_RESET);//-> AIN 2

    //M2 (右前轮向后，因安装反向，需给正向信号)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_SET);//-> BIN 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , GPIO_PIN_RESET);//-> BIN 2 (原PB2改为PA7)

    //M3 (左后轮向前)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);//-> CIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);//-> CIN 2

    //M4 (右后轮向后，因安装反向，需给正向信号)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 , GPIO_PIN_SET);//-> DIN 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 , GPIO_PIN_RESET);//-> DIN 2
}

void move_turn_right_boost()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_TURN_BOOST_SPEED);//M1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_TURN_BOOST_SPEED);//M2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_TURN_BOOST_SPEED);//M3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_TURN_BOOST_SPEED);//M4

    //M1 (左前轮向后)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , GPIO_PIN_RESET);//-> AIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 , GPIO_PIN_SET);//-> AIN 2

    //M2 (右前轮向前，因安装反向，需给反向信号)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_RESET);//-> BIN 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , GPIO_PIN_SET);//-> BIN 2 (原PB2改为PA7)

    //M3 (左后轮向后)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);//-> CIN 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//-> CIN 2

    //M4 (右后轮向前，因安装反向，需给反向信号)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 , GPIO_PIN_RESET);//-> DIN 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 , GPIO_PIN_SET);//-> DIN 2
}