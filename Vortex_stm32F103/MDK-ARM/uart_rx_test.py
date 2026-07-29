#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试：接收ASCII字符串
"""

import serial
import time

PORT = "/dev/ttyAMA0"
BAUDRATE = 2400

def main():
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0
        )
        print(f"串口已打开: {PORT} @ {BAUDRATE}")
        print("等待接收数据...")
        print("=" * 60)
        
        while True:
            # 读取一行（直到换行符）
            data = ser.readline()
            if data:
                try:
                    text = data.decode('ascii')
                    print(f"收到: '{text.strip()}'")
                except:
                    print(f"收到(十六进制): {[hex(x) for x in data]}")
                    
    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        ser.close()
        print("串口已关闭")

if __name__ == "__main__":
    main()
