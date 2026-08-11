# Vortex Drive — 视觉自主跟踪小车

一套「PC 遥控 + 树莓派视觉 + STM32 运动控制」三层架构的智能小车系统。小车既能通过 PC 键盘手动遥控，也能自主识别并跟踪红色圆形目标，实现视觉伺服闭环控制。

## 系统架构

```
┌─────────────────────┐        TCP (8651)        ┌──────────────────────┐
│   PC 端              │ ───────────────────────▶ │  树莓派 Vortex_gateway │
│   Vortex_QTclient    │ ◀─────────────────────── │  视觉 + 网关          │
│   Qt6 + GStreamer    │     UDP (8650) H.264      │                      │
└─────────────────────┘                           └──────────┬───────────┘
     按键指令 W/S/A/D/Shift/C                                  │ UART 串口 (4800)
     视频解码显示 手动/自动切换                                  ▼
                                                ┌──────────────────────┐
                                                │  STM32F103            │
                                                │  电机控制 + PID 闭环   │
                                                │  4 路 PWM + 编码器反馈 │
                                                └──────────────────────┘
```

## 三大模块

| 模块 | 平台 | 职责 |
|---|---|---|
| `Vortex_QTclient` | Windows / Qt 6.10.2 | 遥控界面、视频解码显示、模式切换 |
| `Vortex_gateway` | 树莓派 / C++ | 摄像头采集、OpenCV 目标检测、H.264 推流、指令转发 |
| `Vortex_stm32F103` | STM32F103 / HAL | 电机 PWM 驱动、串级 PID 闭环、断链安全保护 |

## 技术栈

- **PC 端**：Qt 6 (Widgets / Network)、GStreamer 1.0 (Windows)、C++17、CMake
- **树莓派网关**：GStreamer 1.0（v4l2src / appsink / x264enc / udpsink 双管道）、OpenCV 4、POSIX Socket、termios 串口、C++17、CMake
- **下位机**：STM32F103、HAL 库、TIM 多通道 PWM、TIM 编码器接口、USART DMA/中断、Keil MDK

## 通信协议

统一帧格式：`[0xAA] [type] [data...]`

| 类型 | 帧内容 | 说明 |
|---|---|---|
| `0x01` 手动 | `[0xAA][0x01][6 字节按键]` | W/S/A/D/Shift/C 六键状态 |
| `0x02` 自动 | `[0xAA][0x02][err_lo][err_hi][area_lo][area_hi]` | 视觉横向偏移 + 面积差 |
| `0x03` 丢失 | `[0xAA][0x03][0,0,0,0]` | 目标丢失，立即停车 |

## 小车功能

- **手动遥控**：W 前进 / S 后退 / A 左转 / D 右转 / Shift 加速 / C 刹车；支持差速转向与原地转向，按键组合冲突自动滑行保护
- **视觉自主跟踪**：识别红色圆形目标，输出横向偏移与距离误差，串级 PID 闭环控制，自动逼近并停在目标前
- **闭环控制**：外环视觉 PID → 期望速度（√面积非线性减速），内环编码器速度 PID + 前馈 + 自适应斜坡，启动平滑无冲击
- **安全保护**：手动指令 200ms 超时自动滑行，自动模式链路中断 1s 立即停车，模式切换时复位全部控制状态
- **调试支持**：UART3 输出 VOFA 八通道数据（编码器脉冲 / 视觉误差 / 期望速度 / 实际速度）

## 构建

```bash
# 树莓派网关
cd Vortex_gateway && cmake -B build && cmake --build build

# PC 客户端（Qt6 + GStreamer，路径见 CMakeLists.txt）
cd Vortex_QTclient && cmake -B build && cmake --build build
```

STM32 端使用 Keil MDK 打开 `Vortex_stm32F103/MDK-ARM/Vortex_stm32F103.uvprojx` 编译烧录。


### 欢迎学习