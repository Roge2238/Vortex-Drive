#include "servo.h"




Servo_Info servo_info;

bool servo_turn = false;

int8_t servo_buf[SERVO_BUF_LEN];// 接收 偏移信息 
int8_t servo_buf_cnt = 0;


int pre_X_PWM;
int pre_Y_PWM;



float x_kp ;
float x_ki ;
float x_kd ;


float x_v_err; // 每一次采样得到的像素水平误差
float x_last_err; //上一次的误差
float x_v_err_sum = 0;// 误差累积 



void X_Servo_PID_Compute(float x_measure, float goal, float dt) // X轴水平方向的舵机PID计算
{
    x_v_err = measure - goal;
    x_v_err_sum += x_v_err;

    x_v_err_diff = x_v_err - x_last_err;
    x_last_err = x_v_err;

    return x_Kp * x_v_err + x_Kd * x_v_err_diff/dt;

}





float y_kp ;
float y_ki ;
float y_kd ;


float y_v_err; // 每一次采样得到的竖直像素误差
float y_last_err; //上一次的误差
float y_v_err_sum = 0;// 误差累积 



void Y_Servo_PID_Compute(float y_measure, float goal, float dt) // Y轴水平方向的舵机PID计算
{
    y_v_err = measure - goal;
    y_v_err_sum += x_v_err;

    y_v_err_diff = y_v_err - y_last_err;
    y_last_err = y_v_err;

    return y_Kp * y_v_err + y_Kd * y_v_err_diff/dt;


}


bool Get_Servo_turn(void)
{
    return servo_turn;
}



void Set_Servo_turn(bool turn)
{
    servo_turn = turn;
}



void servo_method()
{
    int diff_X_PWM = X_Servo_PID_Compute();
    int cur_X_PWM = diff_X_PWN + pre_X_PWM;

    int diff_Y_PWM = Y_Servo_PID_Compute();
    int cur_Y_PWM = diff_Y_PWN + pre_Y_PWM;

    X_PWM(cur_X_PWM);

    Y_PWM(cur_Y_PWM);

}



void X_PWM(int16_t ccr)
{
    if(ccr >= MAX_CCR) 
        ccr = MAX_CCR;
    else if (ccr <= MIN_CCR)
        ccr = MIN_CCR;
    
    TIM_SetCompare(TIM4, CCR);

}


void Y_PWM(int16_t ccr)
{
     if(ccr >= MAX_CCR) 
        ccr = MAX_CCR;
    else if (ccr <= MIN_CCR)
        ccr = MIN_CCR;
    
    TIM_SetCompare(TIM4, CCR);
}