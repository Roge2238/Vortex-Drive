#include "stm32f1xx_hal.h"

#define COAST_TIMEOUT_MS  200 
#define COAST_TIMEOUT_CNT (COAST_TIMEOUT_MS/ 20)

#define PWM_BASE_SPEED     500
#define PWM_BOOST_SPEED    750
#define PWM_TURN_SPEED     400
#define PWM_TURN_BOOST_SPEED 650
#define PWM_OFFSET         250

void move_forward();

void boost_pwm(TIM_HandleTypeDef* tim, uint32_t ch);

void move_back();

void move_forward_left();

void move_forward_right();

void move_back_left();

void move_back_right();

void move_boost();

void move_stop();

void move_slow();

void move_slide();

void move_coast();

void slow_pwm(TIM_HandleTypeDef* tim, uint32_t ch);

void move_forward_left_boost();

void move_forward_right_boost();

void move_back_left_boost();

void move_back_right_boost();

void move_back_boost();

void move_turn_left();

void move_turn_right();

void move_turn_left_boost();

void move_turn_right_boost();









