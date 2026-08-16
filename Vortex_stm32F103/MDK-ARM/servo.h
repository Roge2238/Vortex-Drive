#include "stm32f1xx_hal.h"





typedef struct Servo_Info{
    float x_measure;
    float y_measure;
    bool find;
};







void servo_PID_init(void);

bool Get_Servo_turn(void);
void Set_Servo_turn(bool turn);


void X_Servo_PID_Compute(float x_measure, float goal, float dt);

void Y_Servo_PID_Compute(float y_measure, float goal, float dt);

