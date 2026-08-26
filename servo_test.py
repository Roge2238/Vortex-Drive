#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机测试脚本（树莓派独立运行，不经 gateway）
发送 SERVO_TEST 帧：[0xAA] [0x05] [pan_angle] [tilt_angle]
  pan_angle / tilt_angle: 目标角度 0~180（0°=CCR50, 90°=CCR150, 180°=CCR250）

用法：
  交互模式 python3 servo_test.py
    输入 "pan tilt" 发送，如：90 45
    s = 自动扫描 0→180→0，验证行程
    q = 退出
  命令行单次：python3 servo_test.py 90 45
  自动扫描：  python3 servo_test.py --scan
"""

import sys
import time
import serial

# 串口配置（STM32 USART1 重映射到 PB6/PB7，115200 8N1）
PORT = "/dev/ttyAMA0"
BAUDRATE = 115200

# 帧格式常量（与 mode.c / servo.c 对应）
MAGIC = 0xAA
  GNU nano 8.4                                          servo_test.py
SERVO_TEST = 0x05


def send_angle(ser, pan, tilt):
    """发送一帧角度指令，限幅 0~180"""
    pan = max(0, min(180, int(pan)))
    tilt = max(0, min(180, int(tilt)))
    frame = bytes([MAGIC, SERVO_TEST, pan, tilt])
    ser.write(frame)
    ser.flush()
    print(f"  -> Pan={pan:3d}°  Tilt={tilt:3d}°  帧: {frame.hex(' ')}")


def scan(ser, step=15, delay=1.0):
    """自动扫描：0 → 180 → 0，观察舵机行程是否平滑、有无堵转"""
    print(f"自动扫描开始（步进{step}°，每档停{delay}s）...")
    angles = list(range(0, 181, step))
    for a in angles:
        send_angle(ser, a, a)
        time.sleep(delay)
    for a in reversed(angles):
        send_angle(ser, a, a)
        time.sleep(delay)
    print("扫描完成，回到 0°")

def interactive(ser):
    print("=" * 50)
    print("舵机测试 - 交互模式")
    print("  输入 'pan tilt' 发送角度（0~180）")
    print("  s = 自动扫描    q = 退出")
    print("=" * 50)
    while True:
        try:
            line = input(">> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\n退出")
            break
        if not line:
            continue
        if line == "q":
            break
        if line == "s":
            scan(ser)
            continue
        parts = line.split()
        if len(parts) != 2:
            print("  格式错误，示例: 90 45")
   
            continue
        try:
            send_angle(ser, int(parts[0]), int(parts[1]))
        except ValueError:
            print("  角度必须是数字")


def main():
    ser = None
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
        print(f"串口已打开: {PORT} @ {BAUDRATE}")

        args = sys.argv[1:]
        if "--scan" in args:
            scan(ser)
        elif len(args) >= 2:
            send_angle(ser, int(args[0]), int(args[1]))
        else:
                    interactive(ser)


            except serial.SerialException as e:
                print(f"串口错误: {e}")
                print("检查：树莓派串口是否启用（/boot/config.txt 的 enable_uart=1）")
            except KeyboardInterrupt:
                print("\n用户中断")
            finally:
                if ser is not None and ser.is_open:
                    ser.close()
                    print("串口已关闭")

if __name__ == "__main__":
            main()

