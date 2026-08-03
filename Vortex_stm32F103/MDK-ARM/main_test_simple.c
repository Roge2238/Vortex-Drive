// 我们需要加入新的功能，比如PID控制 在不破坏开环指令控制的基础上 
//接收上位机的 error
// 并根据 error 计算出对应的输出
// 最后将输出发送给电机控制模块 so 我们先需要确认模式式的切换方式 设置多种协议 
/*
TIM1：左轮编码器 (PA8=CH1, PA9=CH2)
TIM2：4路PWM输出 (PA0-PA3)，CH1=左前M1, CH2=右前M2, CH3=左后M3, CH4=右后M4
TIM3：右轮编码器 (PB4=CH1, PB5=CH2, 部分重映射)
TIM4：20ms定时中断，测速 + PID计算 + 电机输出
USART1：串口通信 (PB6=TX, PB7=RX, 重映射)
*/



/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Motor control via UART command
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "motor.h"
#include "cmd.h"
#include "pid.h"
#include "mode.h"

#define UART_RX_BUF_LEN 64

uint8_t cmd_buf[CMD_LEN];
uint8_t recv;
UartBuf_t uart_buf;
CV_t cv;
Mode_t mode;
RecvState state = WAIT_AA;
//改为串级PID控制
PID_t pid_left;
PID_t pid_right;
PID_t pid_steer;
PID_t pid_speed;

float target_left_pwm = 0.0f;
float target_right_pwm = 0.0f;

uint8_t type = 0;

volatile CmdState cmdState = CMD_EMPTY;
int no_cmd_cnt = 0;



/* 命令超时检测 - 200ms内必须收到新命令 */
#define CMD_TIMEOUT_MS  200
#define CMD_TIMEOUT_CNT (CMD_TIMEOUT_MS / 20)
volatile uint8_t cmd_timeout_cnt = 0;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_USART1_UART_Init();
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();

  /* 初始化PID — 以下为起始参数，需根据实际小车调校 */
  PID_Init(&pid_left,   0.8f, 0.2f, 0.1f, 500.0f, 100.0f);
  PID_Init(&pid_right,  0.8f, 0.2f, 0.1f, 500.0f, 100.0f);
  PID_Init(&pid_steer,  1.5f, 0.0f, 0.5f, 500.0f, 100.0f);
  PID_Init(&pid_speed,  2.0f, 0.5f, 0.0f, 500.0f, 100.0f);

  /* Start PWM — TIM2 输出4路PWM */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  /* Start TIM4 - 20ms周期中断 */
  HAL_TIM_Base_Start_IT(&htim4);

  /* Start 编码器 — TIM1(左轮) + TIM3(右轮) */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  // Enable STBY  搞半天发现不需要接STBY
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  // 启动UART中断接收
  HAL_UART_Receive_IT(&huart1, &recv, 1);
  
  while (1)
  {
    // 主循环：低功耗等待中断
    //__WFI();  // Wait For Interrupt

    frame_task();// 两个模式的数据准备完毕 


  }
}

/**
  * @brief  UART接收完成回调
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
   /*  构建命令码
    uint8_t keycode = build_keycode(rx_buf);
    cmdState = (CmdState)keycode;
    
    
    cmd_timeout_cnt = 0;
    no_cmd_cnt = 0;
    

    HAL_UART_Receive_IT(&huart1, rx_buf, CMD_LEN);*/

    push_uart_buf(recv);


    HAL_UART_Receive_IT(huart, &recv, 1);


  }
}

/**
  * @brief  UART错误回调 - 重新启动接收
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(huart, &recv, 1);
  }
}

/**
  * @brief  TIM4定时中断回调 - 20ms周期
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)/// 20ms周期中断用来输出电机
{
  if (htim->Instance == TIM4)
  {
    /* 命令超时检测：计数一直累加，收到新命令时在frame_task中重置为0 */
    cmd_timeout_cnt++;

    if(mode == MODE_CMD)
    {
      if (cmdState != CMD_EMPTY && cmd_timeout_cnt >= CMD_TIMEOUT_CNT)
      {
        cmdState = CMD_EMPTY;  // 超时，命令失效
      }
      cmd_method(&cmdState);
    }
    else if (mode == MODE_AUTO)
    {
      /* 读取编码器脉冲数并清零 */
      int16_t left_pulse  = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
      int16_t right_pulse = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
      __HAL_TIM_SET_COUNTER(&htim1, 0);
      __HAL_TIM_SET_COUNTER(&htim3, 0);

      /* 计算实际速度（脉冲/20ms） */
      float left_speed  = (float)left_pulse;
      float right_speed = (float)right_pulse;

      /* 外环PID：转向 + 速度 */
      float steer_out = PID_Compute(&pid_steer, (float)cv.error, 0.020f);
      float speed_out = PID_Compute(&pid_speed, (float)cv.area, 0.020f);

      /* 目标PWM（外环输出组合） */
      target_left_pwm  = speed_out + steer_out * 0.5f;
      target_right_pwm = speed_out - steer_out * 0.5f;
      target_left_pwm  = HAL_CLAMP(target_left_pwm, -100.0f, 100.0f);
      target_right_pwm = HAL_CLAMP(target_right_pwm, -100.0f, 100.0f);

      /* 内环PID：左/右轮速度闭环 */
      float left_pwm  = PID_Compute(&pid_left,  target_left_pwm  - left_speed,  0.020f);
      float right_pwm = PID_Compute(&pid_right, target_right_pwm - right_speed, 0.020f);
      left_pwm  = HAL_CLAMP(left_pwm, -100.0f, 100.0f);
      right_pwm = HAL_CLAMP(right_pwm, -100.0f, 100.0f);

      /* 输出PWM（含方向控制） */
      set_drive_pwm(left_pwm, right_pwm);
    }
  }
}

/**
  * @brief  System Clock Configuration
  *         HSE外部晶振 + PLL (HSE × 9 = 72MHz)
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // 使用HSE外部晶振 + PLL  一定要用外部晶振 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;     // HSE × 9 = 72MHz
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}


#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif


