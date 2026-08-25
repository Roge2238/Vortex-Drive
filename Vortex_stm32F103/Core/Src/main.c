/*
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
#include <math.h>    /* sqrtf */

/* 编码器脉冲归一化：2200 脉冲/20ms ≈ 满载 经过测试 对应 target=100 */
#define PULSE_MAX  2200.0f

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
#define KV_FORWARD 1.5f

/* 自适应斜坡 车速越低，允许的变化速率越小，防止启动冲击 */
#define RAMP_LOW   8.0f     // 低速时每周期最多变 8% PWM
#define RAMP_HIGH  20.0f    // 高速时可放松到 20%/周期  亲测有效
#define SPEED_THRESH 3.0f   // 车速 < 3% 认为尚未脱离静摩擦区


#define K_AREA           0.1f   // sqrt 速度系数
#define MIN_CRUISE_SPEED 2.5f   // 最低巡航速度，低于此值静摩擦可能卡住
#define AREA_DEADZONE    200.0f // |area| < 200 认为到达目标 (12000±200=11800~12200)

/* 外环开关: 0=只测速度(直行) 1=加入转向(差速) */
#define STEER_ENABLE 1



/* 命令超时检测 - 200ms内必须收到新命令 */
#define CMD_TIMEOUT_MS  200
#define CMD_TIMEOUT_CNT (CMD_TIMEOUT_MS / 20)

/* AUTO 断链保护 - 1秒无帧则停车 */
#define AUTO_TIMEOUT_MS  1000
#define AUTO_TIMEOUT_CNT (AUTO_TIMEOUT_MS / 20)

/* VOFA 调试发送函数 */
void Send_To_VOFA(float v1, float v2, float v3, float v4,
                  float v5, float v6, float v7, float v8);

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
static void IWDG_Init_1s(void)
{
  RCC->CSR |= RCC_CSR_LSION;      /* 开启 LSI 低速时钟 */
  while (!(RCC->CSR & RCC_CSR_LSIRDY));

  /* 调试用：调试器挂起时冻结 IWDG（上线前删除下面两行） */
  //__HAL_RCC_DBGMCU_CLK_ENABLE();
  //DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;

  IWDG->KR  = 0x5555;             /* 解锁 PR/RLR 寄存器写保护 */
  IWDG->PR  = 0x04;               /* 预分频 /64 → 40kHz/64 = 625Hz */
  IWDG->RLR = 625;                /* 625 个计数 ≈ 1s 超时 */
  IWDG->KR  = 0xCCCC;             /* 启动看门狗（此后不可关闭） */
}

/* 喂狗：主循环每圈调用，重装计数防止复位 */
static inline void IWDG_Feed(void)
{
  IWDG->KR = 0xAAAA;
}

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


volatile CmdState cmdState = CMD_EMPTY;
int no_cmd_cnt = 0;

volatile uint8_t cmd_timeout_cnt = 0;
volatile uint8_t cv_active = 0;          // 收到首个AUTO帧后置1 
float cur_left_pwm  = 0.0f;
float cur_right_pwm = 0.0f;

/* ========== VOFA 调试环形缓冲（SPSC：TIM4中断写 / 主循环读） ==========
 * 背景：原来在 TIM4 中断里直接 HAL_UART_Transmit 发 VOFA 数据，
 *       90字节@115200 要阻塞约 7.8ms，占 20ms 控制周期近 40%，
 *       会破坏 PID 节拍精度。改为中断里只做 8 个 float 拷贝（约 1us），
 *       实际发送交给主循环空闲时执行。
 * 安全：单生产者(TIM4中断)/单消费者(主循环)，uint8_t 索引读写是原子的，
 *       满时丢最旧一帧（调试数据允许丢，保证拿到的是最新状态）。
 */
#define VOFA_CH_NUM 8    /* 每帧 8 通道 */
#define VOFA_RING_N 4    /* 环形缓冲 4 帧深 */
volatile float vofa_ring[VOFA_RING_N][VOFA_CH_NUM];
volatile uint8_t vofa_wr = 0;   /* 生产者写索引 */
volatile uint8_t vofa_rd = 0;   /* 消费者读索引 */

/* ========== 主函数 ========== */
int main(void)
{
  HAL_Init();
  IWDG_Init_1s();   /* 独立看门狗：程序死机 ~1s 内硬件复位停车 */
  SystemClock_Config();

  MX_USART1_UART_Init();
  MX_USART3_UART_Init();   /* VOFA 调试串口 */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();

  /*
   * === 内环 PID (速度闭环，已标定，勿动) ===
   *   kp=1.0, ki=0.5 : 配合前馈 KV=1.5，启动平滑、稳态 ±10%
   *   integral_limits=100 : 乘 dt 后的积分上限
   *   output_limit=80 : PID ±80%，与前馈凑满总输出范围
   */
  PID_Init(&pid_left,   1.0f, 0.5f, 0.0f, 100.0f, 80.0f);
  PID_Init(&pid_right,  1.0f, 0.5f, 0.0f, 100.0f, 80.0f);


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

  PID_Init(&pid_steer,  0.30f, 0.05f, 0.03f, 200.0f, 100.0f);
  PID_Init(&pid_speed,  0.005f, 0.002f, 0.0f, 200.0f, 100.0f);
  /* PWM */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  /* 20ms 定时中断 */
  HAL_TIM_Base_Start_IT(&htim4);

  /* 舵机 PWM（与20ms中断共用TIM4计数器，互不干扰）：
   *   CH3 → PB8 水平舵机 (Pan)   CH4 → PB9 竖直舵机 (Tilt)
   * 角度→CCR 换算：90°=150 (1.5ms)，见 tim.c MX_TIM4_Init */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);// 水平舵机 
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);// 竖直舵机 

  /* 舵机 PID 初始化：增益装载 + 位置复位中位 90° */
  Servo_PID_Init();


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
    IWDG_Feed();   /* 喂狗：主循环卡死超 ~1s 即触发硬件复位停车 */

    /* VOFA 发送：主循环空闲时执行。
     * 串口忙(gState != READY)则跳过整帧等下一轮，保证主循环不被调试输出阻塞。 */
    if (vofa_rd != vofa_wr && huart3.gState == HAL_UART_STATE_READY)
    {
      Send_To_VOFA(vofa_ring[vofa_rd][0], vofa_ring[vofa_rd][1],
                   vofa_ring[vofa_rd][2], vofa_ring[vofa_rd][3],
                   vofa_ring[vofa_rd][4], vofa_ring[vofa_rd][5],
                   vofa_ring[vofa_rd][6], vofa_ring[vofa_rd][7]);
      vofa_rd = (vofa_rd + 1) & (VOFA_RING_N - 1);
    }
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

/* ========== TIM4 20ms 中断 — 串级 PID (外环视觉 → 内环速度) ========== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM4)
    return;

  /* 连接超时检测：50次×20ms = 1秒无数据则停机
   * 收到新数据时 frame_task→cmd_timeout_cnt=0 重置
   */
  cmd_timeout_cnt++;

  if(Get_Servo_turn()) // 如果舵机正在启动 检查相关变量 并输出PWM
  {
    servo_method();// 舵机启动 执行舵机任务
  }


  /* AUTO 模式断链保护：视觉链路1秒无帧 → 清积分停车*/
  if (mode == MODE_AUTO && cv_active && cmd_timeout_cnt > AUTO_TIMEOUT_CNT)
  {
    cv_active = 0;
    cur_left_pwm = 0.0f;
    cur_right_pwm = 0.0f;
    PID_Reset(&pid_left);
    PID_Reset(&pid_right);
    PID_Reset(&pid_steer);
    PID_Reset(&pid_speed);
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

      int16_t left_pulse  = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
      int16_t right_pulse = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
      __HAL_TIM_SET_COUNTER(&htim1, 0);
      __HAL_TIM_SET_COUNTER(&htim3, 0);
      float left_speed  = (float)(-left_pulse) / PULSE_MAX * 100.0f;
      float right_speed = (float)right_pulse   / PULSE_MAX * 100.0f;

      /* ——— 步骤 2：外环 — OpenCV 误差 → 期望速度 ———
      *   cv.error: 目标偏离画面中心的像素 (+右/-左, 范围 ~±320)
      *             → pid_steer(PID) → steer_out (转向修正量)
      *
      *   cv.area:  TARGET_AREA - actual_area 面积差 (+太远/-太近)
      *             → √|area| × K_AREA → speed_out (前进速度, 非线性减速)
      *
      *   注：距离→速度不用线性PID，因为 area 量级(千) 和 speed(百) 跨度太大，
      *       线性 P 在远处高速冲、近处刹不住。sqrt 曲线自动让减速斜率随
      *       距离缩短而增大，是工业级轨迹规划的标准做法。
      *
      *   cv_active=0 时外环输出归零 → 平滑停车
      */
      float steer_out = 0.0f;
      float speed_out = 0.0f; //speed 弃用pid 使用非线性映射 

      if (cv_active)
      {
        /* 转向：普通 PID  */
        steer_out = PID_Compute(&pid_steer, (float)cv.error, 0.020f);

        /* 大于error = 5px时 为转向做速度保底  ‘MIN_STEER_FLOOR’  保证最小输出防止摩擦力卡死。
        *   error<5px 时不做保底，避免抖动 */
        #define MIN_STEER_ERROR  5.0f
        #define MIN_STEER_FLOOR  5.0f
        if (fabsf((float)cv.error) > MIN_STEER_ERROR) {
          if (steer_out > 0.0f && steer_out <  MIN_STEER_FLOOR) steer_out =  MIN_STEER_FLOOR;
          if (steer_out < 0.0f && steer_out > -MIN_STEER_FLOOR) steer_out = -MIN_STEER_FLOOR;
        }

        /* speed：sqrt 非线性映射 */
        float area_abs = fabsf((float)cv.area);

        if (area_abs < AREA_DEADZONE)
        {
          speed_out = 0.0f;          // 已到达目标区域，停车
        }
        else
        {
          speed_out = K_AREA * sqrtf(area_abs);  // √曲线自然减速

          /* 最低巡航速度：防止速度太低被静摩擦卡住  依然保底 就是这么小心谨慎 */
          // 最后阶段刹车时 会有不错的刹车体验 追求精细控制其实不用 但这样还是稳定 
          if (speed_out < MIN_CRUISE_SPEED)
            speed_out = MIN_CRUISE_SPEED;

          
          if (cv.area < 0)
            speed_out = -speed_out;
        }
      }

    
      #define MAX_FWD_SPEED 15.0f   // 外环最大前进速度 
      #define MAX_REV_SPEED 10.0f   // 外环最大后退速度
      speed_out = HAL_CLAMP(speed_out, -MAX_REV_SPEED, MAX_FWD_SPEED);

      /* 差速组合: speed=基础, steer→左右差速 (STEER_ENABLE 控制是否加入) */
      #define STEER_WEIGHT 0.3f
      #if STEER_ENABLE
        target_left_pwm  = speed_out + steer_out * STEER_WEIGHT;
        target_right_pwm = speed_out - steer_out * STEER_WEIGHT;
      #else
        target_left_pwm  = speed_out;  
        target_right_pwm = speed_out;
        (void)steer_out;               
      #endif
      target_left_pwm  = HAL_CLAMP(target_left_pwm,  -100.0f, 100.0f);
      target_right_pwm = HAL_CLAMP(target_right_pwm, -100.0f, 100.0f);

      /* ——— 内环前馈 ——— */
      float ff_left  = target_left_pwm  * KV_FORWARD; // KV 可以大一点 充分发挥 PID的补偿
      float ff_right = target_right_pwm * KV_FORWARD;

      /* ——— ：内环 PID 补误差 ——— */
      float left_err   = target_left_pwm  - left_speed;
      float right_err  = target_right_pwm - right_speed;
      float left_pid   = PID_Compute(&pid_left,  left_err,  0.020f);
      float right_pid  = PID_Compute(&pid_right, right_err, 0.020f);

      float raw_left   = ff_left  + left_pid;
      float raw_right  = ff_right + right_pid;

      //斜坡防冲击 防止启动振荡
      #define SPEED_BAND 15.0f
      

      float avg_speed = (fabsf(left_speed) + fabsf(right_speed)) * 0.5f;
      float max_delta;
      if (avg_speed < SPEED_THRESH) {
        max_delta = RAMP_LOW;
      } else if (avg_speed < SPEED_BAND) {
        float t = (avg_speed - SPEED_THRESH) / (SPEED_BAND - SPEED_THRESH);
        max_delta = RAMP_LOW + (RAMP_HIGH - RAMP_LOW) * t;
      } else {
        max_delta = RAMP_HIGH;
      }

      float dl = raw_left  - cur_left_pwm;
      float dr = raw_right - cur_right_pwm;
      dl = HAL_CLAMP(dl, -max_delta, max_delta);
      dr = HAL_CLAMP(dr, -max_delta, max_delta);
      cur_left_pwm  += dl;
      cur_right_pwm += dr;

      /* ———：输出 PWM ——— */
      set_drive_pwm(cur_left_pwm, cur_right_pwm);

      /* ——— VOFA 调试数据：写入环形缓冲，主循环负责发送 ———
      *   通道定义: 1-2 编码器脉冲   3-4 cv.error / cv.area (外环输入)
      *             5-6 steer/speed (外环输出=期望速度)   7-8 实际速度
      *
      *   不能在中断里直接 HAL_UART_Transmit：90字节@115200 阻塞约 7.8ms，
      *   占 20ms 控制周期近 40%，会破坏 PID 节拍。这里只拷贝 8 个 float（约 1us）。
      */
      {
        volatile float *p = vofa_ring[vofa_wr];
        p[0] = (float)left_pulse;   p[1] = (float)right_pulse;
        p[2] = (float)cv.error;     p[3] = (float)cv.area;
        p[4] = steer_out;           p[5] = speed_out;
        p[6] = left_speed;          p[7] = right_speed;

        /* 满则丢最旧一帧（覆盖式更新，调试数据允许丢，不能积压） */
        uint8_t next = (vofa_wr + 1) & (VOFA_RING_N - 1);
        if (next == vofa_rd)
          vofa_rd = (vofa_rd + 1) & (VOFA_RING_N - 1);
        vofa_wr = next;
      }
 }
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


/* ========== VOFA 调试发送 — FireWater 格式 ========== */
void Send_To_VOFA(float v1, float v2, float v3, float v4,
                  float v5, float v6, float v7, float v8)
{
  char buf[128];
  int len = snprintf(buf, sizeof(buf),
                     "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                     v1, v2, v3, v4, v5, v6, v7, v8);
  HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 10);
}

/* ========== 错误处理 ========== */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
