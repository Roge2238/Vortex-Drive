#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
云台视觉伺服（AUTO）+ 舵机角度测试（TEST）合并脚本

--pi : 树莓派模式（/dev/video0 + V4L2 + 640x480，远程桌面看画面）
默认 : PC 调试模式

控制走终端命令行（窗口不用聚焦），画面照常显示：
  pan tilt    设置角度并发送 0x05 帧，如: 90 45
  p+ / p-     水平 pan 微调（可带步长 p+20）
  t+ / t-     垂直 tilt 微调
  p90 / t45   单轴设角度
  0           回中位 90 90
  s           自动扫描 0→180→0
  m           切换 AUTO/TEST 模式
  h           帮助    q 退出
"""

import cv2
import numpy as np
import queue
import re
import serial
import struct
import sys
import threading
import time

MAGIC_HEAD = 0xAA
TYPE_SERVO = 0x04        # 自动模式：目标像素坐标 (cx, cy)，int16 小端
TYPE_SERVO_TEST = 0x05   # 测试模式：pan_angle, tilt_angle（0~180）

PI_MODE = '--pi' in sys.argv   # --pi=树莓派；默认(--pc)=PC

MODE_AUTO = 'AUTO'
MODE_TEST = 'TEST'

# ---------- 串口 ----------
try:
    ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.1)
    print("Serial /dev/ttyAMA0 @ 115200")
except serial.SerialException as e:
    print(f"WARN: 串口打开失败: {e}")
    ser = None

# ---------- 摄像头 ----------
if PI_MODE:
    cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)   # 参照 opencv_test
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
else:
    cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    sys.exit(1)

# ---------- 红色圆盘检测（照 opencv_test） ----------
lower_red1 = np.array([0, 60, 35])
upper_red1 = np.array([8, 255, 255])
lower_red2 = np.array([172, 60, 35])
upper_red2 = np.array([230, 255, 255])

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
SEND_INTERVAL = 0.02   # 20ms，与 STM32 中断同步
last_send = time.monotonic()
send_x = 320   # 目标丢失时保持上一帧坐标（STM32 死区钳 0）
send_y = 240

prev_cx, prev_cy = 320, 240   # 质心低通滤波

# ---------- 测试模式状态 ----------
mode = MODE_AUTO
pan, tilt = 90, 90
step = 5

def clamp_angle(a):
    return max(0, min(180, int(a)))

def pack_servo_frame(cx, cy):
    """0x04 自动帧: AA 04 [cx lo] [cx hi] [cy lo] [cy hi]，目标像素坐标"""
    px = int(max(0, min(32767, cx)))
    py = int(max(0, min(32767, cy)))
    return struct.pack('<BBhh', MAGIC_HEAD, TYPE_SERVO, px, py)

def pack_test_frame(pan, tilt):
    """0x05 测试帧: AA 05 [pan] [tilt]，角度 0~180"""
    return bytes([MAGIC_HEAD, TYPE_SERVO_TEST, clamp_angle(pan), clamp_angle(tilt)])

def send_test(pan, tilt):
    if ser is None:
        print(f"  (串口未开) TEST -> pan={pan:3d}  tilt={tilt:3d}")
        return
    ser.write(pack_test_frame(pan, tilt))
    print(f"  TEST -> pan={pan:3d}  tilt={tilt:3d}")

def scan():
    """自动扫描 0→180→0，验证行程"""
    if ser is None:
        print("串口未打开，无法扫描")
        return
    print("自动扫描 0→180→0 ...")
    for a in list(range(0, 181, 15)) + list(range(180, -1, -15)):
        send_test(a, a)
        time.sleep(0.3)
    print("扫描完成")

def handle_cmd(cmd):
    """处理一条终端命令。返回 True 表示退出。"""
    global mode, pan, tilt
    parts = cmd.split()
    if not parts:
        return False
    c = parts[0].lower()

    if c == 'q':
        print("退出")
        return True
    if c in ('h', 'help'):
        print(__doc__)
        return False
    if c == 'm':
        mode = MODE_TEST if mode == MODE_AUTO else MODE_AUTO
        print(f"mode -> {mode}")
        return False
    if c == '0':
        pan = tilt = 90
        send_test(pan, tilt)
        return False
    if c == 's':
        scan()
        return False

    # 两个数字: "90 45"
    if len(parts) >= 2:
        try:
            pan = clamp_angle(parts[0])
            tilt = clamp_angle(parts[1])
            send_test(pan, tilt)
            return False
        except ValueError:
            pass

    # 相对微调: p+ / p- / t+ / t-（可带步长 p+20）
    m = re.match(r'^([pt])([+-])(\d*)$', c)
    if m:
        axis, op, num = m.group(1), m.group(2), m.group(3)
        delta = int(num) if num else step
        if op == '-':
            delta = -delta
        if axis == 'p':
            pan = clamp_angle(pan + delta)
        else:
            tilt = clamp_angle(tilt + delta)
        send_test(pan, tilt)
        return False

    # 单轴设定: p90 / t45
    m = re.match(r'^([pt])(\d+)$', c)
    if m:
        axis, val = m.group(1), m.group(2)
        if axis == 'p':
            pan = clamp_angle(val)
        else:
            tilt = clamp_angle(val)
        send_test(pan, tilt)
        return False

    print("格式: 'pan tilt' | p+/p-/t+/t- | p90/t45 | 0中位 | s扫描 | m切模式 | h帮助 | q退出")
    return False

# ---------- 后台终端命令读取（不阻塞画面） ----------
cmd_queue = queue.Queue()

def stdin_reader():
    while True:
        try:
            line = input("servo>> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if line:
            cmd_queue.put(line)

threading.Thread(target=stdin_reader, daemon=True).start()

print("=" * 50)
print("命令: 'pan tilt' | p+/p-/t+/t- | p90/t45 | 0中位 | s扫描 | m切模式 | q退出")
print("=" * 50)

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
                send_x = cx
                send_y = cy

        # ---------- 画面 OSD（一直显示） ----------
        cv2.putText(frame, f"mode:{mode}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        if mode == MODE_TEST:
            cv2.putText(frame, f"pan:{pan}  tilt:{tilt}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        elif detected:
            cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(frame, f"cx:{cx} cy:{cy}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        cv2.imshow("Camera", frame)
        cv2.waitKey(1)   # 仅刷新窗口，控制走终端

        # ---------- 处理终端命令 ----------
        try:
            cmd = cmd_queue.get_nowait()
        except queue.Empty:
            cmd = None
        if cmd is not None and handle_cmd(cmd):
            break

        # ---------- AUTO 模式：20ms 发送坐标帧 ----------
        if mode == MODE_AUTO:
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
