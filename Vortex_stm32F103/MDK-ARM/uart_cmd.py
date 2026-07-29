#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动演示测试脚本
序列：前进2s → 左转2s → 后退2s → 停止
"""

import serial
import time

# 串口配置
PORT = "/dev/ttyAMA0"
BAUDRATE = 4800

# 命令定义 [前进, 后退, 左, 右, 加速, 急停]
CMD_STOP     = [0, 0, 0, 0, 0, 0]  # 停止
CMD_FORWARD  = [1, 0, 0, 0, 0, 0]  # 前进
CMD_BACK     = [0, 1, 0, 0, 0, 0]  # 后退
CMD_LEFT     = [0, 0, 1, 0, 0, 0]  # 左转
CMD_RIGHT    = [0, 0, 0, 1, 0, 0]  # 右转
CMD_BRAKE    = [0, 0, 0, 0, 0, 1]  # 急停


def send_cmd(ser, cmd, duration):
    """持续发送命令指定时间"""
    start = time.time()
    while (time.time() - start) < duration:
        ser.write(bytes(cmd))
        ser.flush()
        time.sleep(0.02)  # 20ms间隔，防止STM32超时


def main():
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
        print(f"串口已打开: {PORT} @ {BAUDRATE}")
        print("=" * 50)
        print("自动演示测试开始！")
        print("=" * 50)
        
        # 1. 前进 2秒
        print("[1/4] 前进 2秒...")
        send_cmd(ser, CMD_FORWARD, 2)
        print("      完成！")
        
        # 2. 左转 2秒
        print("[2/4] 左转 2秒...")
        send_cmd(ser, CMD_LEFT, 2)
        print("      完成！")
        
        # 3. 后退 2秒
        print("[3/4] 后退 2秒...")
        send_cmd(ser, CMD_BACK, 2)
        print("      完成！")
        
        # 4. 停止
        print("[4/4] 停止...")
        send_cmd(ser, CMD_STOP, 0.5)
        print("      完成！")
        
        print("=" * 50)
        print("✅ 演示测试完成！")
        
    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except KeyboardInterrupt:
        print("\n用户中断")
        try:
            ser.write(bytes(CMD_STOP))
        except:
            pass
    finally:
        if ser.is_open:
            ser.close()
            print("串口已关闭")


if __name__ == "__main__":
    main()
