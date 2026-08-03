/*
 * main_test.c — 纯 PID 闭环测试主程序
 *
 * 功能：只跑 AUTO_MODE，接收 OpenCV 发来的 error/area，
 *       串级 PID 闭环控制左右轮速度。
 *
 * 帧格式（上位机 → STM32）：
 *   [0xAA] [0x02] [err_lo] [err_hi] [area_lo] [area_hi]
 *   error: int16_t  (小端)
 *   area:  int16_t  (小端，可为负)
 *
 * 硬件：
 *   TIM1 (PA8/PA9)   左轮编码器
 *   TIM3 (PB4/PB5)   右轮编码器
 *   TIM2 (PA0-PA3)   4路PWM
 *   TIM4             20ms定时中断
 *   USART1 (PB6/PB7) 串口接收
 */

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "motor.h"
#include "pid.h"
#include "mode.h"
#include "cmd.h"
#include <string.h>

/* 编码器脉冲归一化：150 脉冲/20ms ≈ 400rpm 额定负载，对应 target=100 */
#define PULSE_MAX  150.0f

/* ========== 全局变量 ========== */
uint8_t cmd_buf[CMD_LEN];
uint8_t recv;
UartBuf_t uart_buf;
CV_t cv;
Mode_t mode = MODE_AUTO;          /* 直接进入 AUTO_MODE */
RecvState state = WAIT_AA;
uint8_t type = 0;

PID_t pid_left;
PID_t pid_right;
PID_t pid_steer;
PID_t pid_speed;

float target_left_pwm = 0.0f;
float target_right_pwm = 0.0f;

/* 定义（motor.c 和 mode.c 用 extern 引用这些变量） */
volatile CmdState cmdState = CMD_EMPTY;
int no_cmd_cnt = 0;

volatile uint8_t cmd_timeout_cnt = 0;
volatile uint8_t cv_active = 0;          /* 收到首个AUTO帧后置1 */

/* ========== 主函数 ========== */
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

  /* PID 参数 — 降低P，避免过冲 */
  PID_Init(&pid_left,   0.1f, 0.05f, 0.05f, 500.0f, 100.0f);
  PID_Init(&pid_right,  0.1f, 0.05f, 0.05f, 500.0f, 100.0f);
  PID_Init(&pid_steer,  0.2f, 0.0f, 0.1f, 500.0f, 100.0f);
  PID_Init(&pid_speed,  0.3f, 0.1f, 0.0f, 500.0f, 100.0f);

  /* PWM */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  /* 20ms 定时中断 */
  HAL_TIM_Base_Start_IT(&htim4);

  /* 编码器 */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  /* STBY 使能（即使不接也不影响） */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /* UART 接收中断 */
  HAL_UART_Receive_IT(&huart1, &recv, 1);

  while (1)
  {
    frame_task();
  }
}

/* ========== UART 回调 ========== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    push_uart_buf(recv);
    HAL_UART_Receive_IT(huart, &recv, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(huart, &recv, 1);
  }
}

/* ========== TIM4 20ms 中断 — PID 控制 ========== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM4)
    return;

  /* 没收到串口数据时，电机停转，防止编码器悬空导致乱跑 */
  if (!cv_active)
  {
    set_drive_pwm(0.0f, 0.0f);
    return;
  }

  /* 连接超时检测：50次×20ms = 1秒无数据则停机 */
  cmd_timeout_cnt++;
  if (cmd_timeout_cnt > 50)
  {
    cv_active = 0;
    PID_Reset(&pid_left);
    PID_Reset(&pid_right);
    PID_Reset(&pid_steer);
    PID_Reset(&pid_speed);
    set_drive_pwm(0.0f, 0.0f);
    return;
  }

  /* 读取编码器并清零 */
  int16_t left_pulse  = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
  int16_t right_pulse = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  /* 归一化编码器脉冲到 ±100，与外环输出量纲对齐 */
  float left_speed  = (float)left_pulse  / PULSE_MAX * 100.0f;
  float right_speed = (float)right_pulse / PULSE_MAX * 100.0f;
  left_speed  = HAL_CLAMP(left_speed,  -100.0f, 100.0f);
  right_speed = HAL_CLAMP(right_speed, -100.0f, 100.0f);

  /* 外环：转向（cv.error）+ 速度（cv.area） */
  float steer_out = PID_Compute(&pid_steer, (float)cv.error, 0.020f);
  float speed_out = PID_Compute(&pid_speed, (float)cv.area,  0.020f);

  /* 外环输出 → 左右目标速度 */
  target_left_pwm  = speed_out + steer_out * 0.5f;
  target_right_pwm = speed_out - steer_out * 0.5f;
  target_left_pwm  = HAL_CLAMP(target_left_pwm, -100.0f, 100.0f);
  target_right_pwm = HAL_CLAMP(target_right_pwm, -100.0f, 100.0f);

  /* 内环：左右轮速度闭环 */
  float left_pwm  = PID_Compute(&pid_left,  target_left_pwm  - left_speed,  0.020f);
  float right_pwm = PID_Compute(&pid_right, target_right_pwm - right_speed, 0.020f);
  left_pwm  = HAL_CLAMP(left_pwm, -100.0f, 100.0f);
  right_pwm = HAL_CLAMP(right_pwm, -100.0f, 100.0f);

  /* 输出 */
  set_drive_pwm(left_pwm, right_pwm);
}

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


/* ========== 错误处理 ========== */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
