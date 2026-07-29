/* 简单测试程序：收到 [1,0,0,0,0,0] 就一直前进 */

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#define DATA_LEN 6

uint8_t pi_cmd[DATA_LEN];
uint8_t motor_running = 0;  // 电机运行标志

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();

  // 启动PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  // STBY使能
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  // 初始停止状态
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // M1 DIN1=LOW
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // M1 DIN2=LOW
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // M2 DIN1=LOW
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // M2 DIN2=LOW
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // M3 DIN1=LOW
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // M3 DIN2=LOW
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // M4 DIN1=LOW
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // M4 DIN2=LOW

  // 设置PWM为0
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); // M1
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0); // M2
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0); // M3
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0); // M4

  // 启动串口中断接收
  HAL_UART_Receive_IT(&huart1, pi_cmd, DATA_LEN);

  while (1)
  {
    // 如果收到前进命令，持续驱动电机前进
    if(motor_running)
    {
        // 左电机：DIN1=HIGH, DIN2=LOW (前进)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);  // M1 DIN1=HIGH
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // M1 DIN2=LOW
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // M3 DIN1=HIGH
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // M3 DIN2=LOW

        // 右电机：DIN1=LOW, DIN2=HIGH (反转实现前进，因为安装方向相反)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // M2 DIN1=LOW
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);  // M2 DIN2=HIGH
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // M4 DIN1=LOW
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  // M4 DIN2=HIGH

        // 设置PWM值
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500); // M1
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500); // M2
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 500); // M3
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 500); // M4
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

// 串口接收回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart != &huart1) return;

    // 检查是否收到 [1,0,0,0,0,0]
    if(pi_cmd[0] == 1 && pi_cmd[1] == 0 && pi_cmd[2] == 0 
       && pi_cmd[3] == 0 && pi_cmd[4] == 0 && pi_cmd[5] == 0)
    {
        motor_running = 1;  // 启动电机
    }
    else
    {
        motor_running = 0;  // 停止电机
    }

    // 继续接收
    HAL_UART_Receive_IT(&huart1, pi_cmd, DATA_LEN);
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


2^7
0100 0000
0000 0001
0000 0000 


0100 0000  0000 0000  0000 0000  0000 0001  0100 0000  0000 0000
   128        0          0            1       128        0
 1000 0000  0000 0000 0000 0000  0000 0010  
