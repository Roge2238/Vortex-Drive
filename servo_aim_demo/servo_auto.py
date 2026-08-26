#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机自动瞄准（精简版）：树莓派专用
检测红色圆盘 → 发送目标像素坐标 (0x04 帧) → STM32 内部 PID 闭环跟踪
带画面显示（远程桌面可看），固定 640x480

用法:
    python3 servo_auto.py                          # 默认串口 /dev/ttyAMA0
    python3 servo_auto.py --port /dev/ttyUSB0      # USB 转串口时指定
退出: 窗口按 q（本地桌面）或 Ctrl+C
"""

import cv2
import numpy as np
import serial
import struct
import sys
import time

MAGIC_HEAD = 0xAA
TYPE_SERVO = 0x04        # 自动模式：目标像素坐标 (cx, cy)，int16 小端

# ---------- 串口（树莓派） ----------
PORT = '/dev/ttyAMA0'
if '--port' in sys.argv:
    i = sys.argv.index('--port')
    if i + 1 < len(sys.argv):
        PORT = sys.argv[i + 1]

try:
    ser = serial.Serial(PORT, 115200, timeout=0.1)
    print(f"Serial {PORT} @ 115200")
except serial.SerialException as e:
    print(f"WARN: 串口打开失败: {e}")
    ser = None

# ---------- 摄像头（树莓派 /dev/video0，V4L2 固定 640x480） ----------
cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    print("Cannot open camera /dev/video0")
    sys.exit(1)

# ---------- 红色圆盘检测 ----------
lower_red1 = np.array([0, 150, 90])
upper_red1 = np.array([5, 255, 255])
lower_red2 = np.array([174, 150, 90])
upper_red2 = np.array([180, 255, 255])

MIN_CONTOUR_AREA = 100

def circle_score(contour):
    area = cv2.contourArea(contour)
    if area < MIN_CONTOUR_AREA:
        return -1
    peri = cv2.arcLength(contour, True)
    if peri <= 0:
        return -1
    circularity = 4 * np.pi * area / (peri * peri)
    (x, y, w, h) = cv2.boundingRect(contour)
    wh_ratio = min(w, h) / max(w, h)
    (_, _), radius = cv2.minEnclosingCircle(contour)
    circle_area = np.pi * radius * radius
    fill_ratio = area / circle_area if circle_area > 0 else 0

    score = 0
    if circularity > 0.72: score += 1
    if fill_ratio > 0.75:  score += 1
    if wh_ratio > 0.85:    score += 1
    return score

# ---------- 发送状态 ----------
SEND_INTERVAL = 0.02     # 20ms，与 STM32 中断同步
last_send = time.monotonic()
send_x, send_y = 320, 240   # 目标丢失时保持上一帧坐标（STM32 死区钳 0）
prev_cx, prev_cy = 320, 240

def pack_servo_frame(cx, cy):
    """0x04 自动帧: AA 04 [cx lo] [cx hi] [cy lo] [cy hi]，目标像素坐标"""
    px = int(max(0, min(32767, cx)))
    py = int(max(0, min(32767, cy)))
    return struct.pack('<BBhh', MAGIC_HEAD, TYPE_SERVO, px, py)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break

        # ---------- 检测红色圆盘 ----------
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel_small = np.ones((5, 5), np.uint8)
        kernel_big = np.ones((9, 9), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_small)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_big)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_contour = None
        best_score = -1
        for cnt in contours:
            s = circle_score(cnt)
            if s > best_score:
                best_score = s
                best_contour = cnt

        detected = False
        if best_contour is not None and best_score >= 2:
            M = cv2.moments(best_contour)
            if M['m00'] != 0:
                detected = True
                cx_raw = int(M['m10'] / M['m00'])
                cy_raw = int(M['m01'] / M['m00'])
                cx = int(0.7 * prev_cx + 0.3 * cx_raw)   # 质心低通滤波
                cy = int(0.7 * prev_cy + 0.3 * cy_raw)
                prev_cx, prev_cy = cx, cy
                send_x, send_y = cx, cy
        else:
            # 目标丢失：发画面中心(320,240) → STM32 err=0 → PID 停住
            # 避免"保持最后坐标"导致云台开环冲向最后位置直到限位
            send_x, send_y = 320, 240

        # ---------- 画面 OSD ----------
        if detected:
            cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(frame, f"cx:{cx} cy:{cy}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            cv2.putText(frame, "TRACKING", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        else:
            cv2.putText(frame, "NO TARGET", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
        cv2.imshow("servo_auto", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):   # 本地桌面按 q 退出
            break

        # ---------- 20ms 发送坐标帧 ----------
        now = time.monotonic()
        if now - last_send >= SEND_INTERVAL:
            last_send = now
            if ser is not None:
                ser.write(pack_servo_frame(send_x, send_y))

except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    cap.release()
    if ser is not None:
        ser.close()
    cv2.destroyAllWindows()
