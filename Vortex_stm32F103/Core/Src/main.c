// 我们需要加入新的功能，比如PID控制 在不破坏开环指令控制的基础上 
//接收上位机的 error
// 并根据 error 计算出对应的输出
// 最后将输出发送给电机控制模块 so 我们先需要确认模式式的切换方式 设置多种协议 




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

#define CMD_LEN 6


uint8_t cmd_buf[CMD_LEN];

uint8_t uart_buf[UART_BUF_LEN];
uint16_t rw = 0;
uint16_t rd = 0;

int16_t error = 0;
uint16_t area = 0;

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
  
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  // Start PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  
  // Start TIM4 - 20ms周期中断
  HAL_TIM_Base_Start_IT(&htim4);
  
  // Enable STBY  搞半天发现不需要接STBY
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  
  // 启动UART中断接收（6字节）
  HAL_UART_Receive_IT(&huart1, rx_buf, CMD_LEN);
  
  while (1)
  {
    // 主循环：低功耗等待中断
    __WFI();  // Wait For Interrupt
  }
}

/**
  * @brief  UART接收完成回调
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // 构建命令码
    uint8_t keycode = build_keycode(rx_buf);
    cmdState = (CmdState)keycode;
    
    // 重置所有超时计数
    cmd_timeout_cnt = 0;
    no_cmd_cnt = 0;
    

    HAL_UART_Receive_IT(&huart1, rx_buf, CMD_LEN);
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
    HAL_UART_Receive_IT(huart, rx_buf, CMD_LEN);
  }
}

/**
  * @brief  TIM4定时中断回调 - 20ms周期
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4)
  {
    // 命令超时检测：计数一直累加，收到新命令时重置为0
    // 如果有命令且计数达到200ms，说明上位机没有持续发送 → 超时
    cmd_timeout_cnt++;
    if (cmdState != CMD_EMPTY && cmd_timeout_cnt >= CMD_TIMEOUT_CNT)
    {
      cmdState = CMD_EMPTY;  // 超时，命令失效
    }
    
    
    cmd_method(&cmdState);
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


