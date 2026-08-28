
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "motor.h"
#include "pid.h"
#include "mode.h"
#include "cmd.h"
#include "servo.h"
#include <string.h>
#include <stdio.h>
#include <math.h>    


#define VOFA_ENABLE 0


// 命令超时  200ms 超时
#define CMD_TIMEOUT_MS  200
#define CMD_TIMEOUT_CNT (CMD_TIMEOUT_MS / 20)

// AUTO 断链保护  1秒无帧则停车  安全第一
#define AUTO_TIMEOUT_MS  1000
#define AUTO_TIMEOUT_CNT (AUTO_TIMEOUT_MS / 20)

/* VOFA 调试发送函数 */
void Send_To_VOFA(float v1, float v2, float v3, float v4,
                  float v5, float v6, float v7, float v8);


static void IWDG_Init_1s(void)
{
  RCC->CSR |= RCC_CSR_LSION;      /* 开启 LSI 低速时钟 */
  while (!(RCC->CSR & RCC_CSR_LSIRDY));

  /* 调试 关闭独立看门狗 */
  //__HAL_RCC_DBGMCU_CLK_ENABLE();
  //DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;

  IWDG->KR  = 0x5555;             
  IWDG->PR  = 0x04;             
  IWDG->RLR = 625;                
  IWDG->KR  = 0xCCCC;           
}

// 喂狗
static inline void IWDG_Feed(void)
{
  IWDG->KR = 0xAAAA;
}


uint8_t cmd_buf[CMD_LEN];
uint8_t recv;
UartBuf_t uart_buf;
CV_t cv;
Mode_t mode = MODE_AUTO;          /* 默认直接进入 AUTO_MODE */
uint8_t type = 0;


volatile CmdState cmdState = CMD_EMPTY;
int no_cmd_cnt = 0;

volatile uint8_t cmd_timeout_cnt = 0;
volatile uint8_t cv_active = 0;   

#define VOFA_CH_NUM 8    
#define VOFA_RING_N 4    
volatile float vofa_ring[VOFA_RING_N][VOFA_CH_NUM];
volatile uint8_t vofa_wr = 0;   
volatile uint8_t vofa_rd = 0;  

int main(void)
{
  HAL_Init();
  //IWDG_Init_1s();  
  SystemClock_Config();

  MX_USART1_UART_Init();  // 上位机收发串口
  MX_USART3_UART_Init();   //VOFA 调试串口 
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();

  Drive_Init();   /* 初始化 AUTO 运动控制的各 PID 参数 */

  /* 四个电机的 PWM */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  
  HAL_TIM_Base_Start_IT(&htim4);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);// 水平舵机 
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);// 竖直舵机 

  
  Servo_PID_Init();


  /* 编码器 */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); //左轮
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);// 右轮

   // 已弃用
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  
  HAL_UART_Receive_IT(&huart1, &recv, 1);

  while (1)
  {
    frame_task();
    IWDG_Feed();   

    #if VOFA_ENABLE
    if (vofa_rd != vofa_wr && huart3.gState == HAL_UART_STATE_READY)
    {
      Send_To_VOFA(vofa_ring[vofa_rd][0], vofa_ring[vofa_rd][1],
                   vofa_ring[vofa_rd][2], vofa_ring[vofa_rd][3],
                   vofa_ring[vofa_rd][4], vofa_ring[vofa_rd][5],
                   vofa_ring[vofa_rd][6], vofa_ring[vofa_rd][7]);
      vofa_rd = (vofa_rd + 1) & (VOFA_RING_N - 1);
    }
    #endif
  }
}


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


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM4)
    return;


  cmd_timeout_cnt++;

  if(Get_Servo_turn()) // 如果舵机正在启动 检查相关变量 并输出PWM
  {
    servo_method();// 舵机启动 执行舵机任务
  }


  /* AUTO 模式断链保护：视觉链路1秒无帧 清积分停车*/
  if (mode == MODE_AUTO && cv_active && cmd_timeout_cnt > AUTO_TIMEOUT_CNT)
  {
    cv_active = 0;
    Drive_Reset();
    set_drive_pwm(0.0f, 0.0f);
    return;
  }
  if (mode == MODE_CMD)
  {
     if(cmd_timeout_cnt >= CMD_TIMEOUT_CNT)
     {
        cmdState = CMD_EMPTY;
      
     }
     cmd_method(&cmdState);
  }
  else if (mode == MODE_AUTO)
  {
      // 后续修改方案 改用前后相减 记录脉冲
      int16_t left_pulse  = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
      int16_t right_pulse = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
      __HAL_TIM_SET_COUNTER(&htim1, 0);
      __HAL_TIM_SET_COUNTER(&htim3, 0);
      float left_speed  = (float)(-left_pulse) / PULSE_MAX * 100.0f;
      float right_speed = (float)right_pulse   / PULSE_MAX * 100.0f;

      Drive_Compute(left_speed, right_speed);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;     
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}


// 发送数据到VOFA  调试用
void Send_To_VOFA(float v1, float v2, float v3, float v4,
                  float v5, float v6, float v7, float v8)
{
  char buf[128];
  int len = snprintf(buf, sizeof(buf),
                     "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                     v1, v2, v3, v4, v5, v6, v7, v8);
  HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 10);
}


void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
