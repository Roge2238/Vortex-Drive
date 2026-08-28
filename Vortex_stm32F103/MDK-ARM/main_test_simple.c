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


/* ========== IWDG 独立看门狗（软件死机 → 硬件复位停车） ==========
 * 原理：独立 LSI 时钟驱动的倒计数硬件。主循环周期性"喂狗"重装计数，
 *       程序一卡死 → 没人喂狗 → 计数到 0 → 芯片硬件复位 → PWM 停止 → 小车停下。
 *       （程序卡死时 TIM/PWM 外设仍在运行，没有看门狗电机会保持最后的
 *         指令全速冲出，这是嵌入式小车最危险的事故类型）
 *
 * 为什么用 IWDG 而不是 WWDG：
 *   IWDG 用 LSI(~40kHz 内部RC振荡器)，与 72MHz 主时钟完全独立，
 *   主时钟停摆、晶振损坏它照常工作；WWDG 用主时钟，主时钟挂了它也瞎。
 *   IWDG 一旦 0xCCCC 启动无法软件关闭（只能复位）——故意设计，
 *   防止程序卡死后还能自己把看门狗关掉。
 *
 * 精度说明：LSI 精度只有 30~60kHz，1s 超时实际是 0.5~2s；
 *           主循环一圈 <5ms，喂狗余量充足，Mode_switch 里的 HAL_Delay(25) 无影响。
 *
 * 调试注意：ST-Link 单步调试/断点暂停时 CPU 停了但看门狗还在倒数，
 *           会把调试中的芯片复位。调试阶段可打开下方 DBGMCU 冻结位，
 *           上线前务必删除（否则看门狗形同虚设）。
 */

 *
 * main_test.c — 串级 PID 闭环控制 (外环视觉 + 内环速度)
 * main 包含完整模式切换 手动遥控 视觉跟踪功能  可自选编译测试 
 
 * 功能：接收 OpenCV 发来的 error(横向像素偏移) / area(面积差)，
 *       外环视觉PID → 期望速度 → 内环速度PID → PWM
 *
 * 帧格式（上位机 → STM32）：
 *   AUTO_TYPE: [0xAA] [0x02] [err_lo] [err_hi] [area_lo] [area_hi]
     CMD_TYPE : [0xAA] [0x01] [cmd]
     error: int16_t 小端  目标偏离画面中心的像素 (+右/-左, 范围±320)
 *   area:  int16_t 小端  TARGET_AREA - actual_area (+太远/-太近)
 *   LOST_TYPE: [0xAA] [0x03] ...  目标丢失，自动停车
 *
 * 控制结构：
 *   外环 (20ms): cv.error → pid_steer → steer_out  (转向修正, 普通PID)
 *                cv.area  → √|area| × K → speed_out (前进速度, 非线性映射)
 
 *   内环 (20ms): target → 前馈 + pid_left/right → 增量斜坡 → PWM
 *
 * 硬件：
 *   TIM1 (PA8/PA9)   左轮编码器    TIM2 (PA0-PA3)  4路PWM
 *   TIM3 (PB4/PB5)   右轮编码器    TIM4           20ms中断
 *   USART1 (PB6/PB7) 串口接收     USART3          VOFA 调试
 */


 
/*
 * 前馈系数: 根据实测标定
 * 实测: 总输出 63 PWM (90 + (-27)) → 稳态速度 57%
 *       实际关系: 57% ≈ 63 PWM → 1% ≈ 1.1 PWM
 * 保守取 1.5，让 PID 有 ±20 的修正空间
 * 
 * 标定方法: 改完烧录后看 VOFA，target=30 时:
 *          - left/right_speed 应在 30±3
 *          - left/right_pid 应在 ±10 以内 (不饱和)
 *          如果 speed 偏高 → 调小 KV_FORWARD；偏低 → 调大
 */

 
  /*
   * === 外环 PID (视觉→期望速度) ===
   *   pid_steer: cv.error(像素) → 转向修正
   *     kp=0.30: @50px → P=15, steer×0.3=4.5%差速 (原来需要100px)
   *              @100px → P=30, steer×0.3=9.0%差速
   *              @200px → P=60, steer×0.3=18.0%差速
   *     ki=0.05: 快速积分，小误差持续时累积破摩擦力
   *     kd=0.03: 转向阻尼，不变 (kp大了不需要额外加kd)
   *
   *   pid_speed: 不再使用 (被 sqrt 映射替代，保留初始化以备切换)
   */