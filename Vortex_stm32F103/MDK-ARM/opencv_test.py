import cv2
import numpy as np
import serial
import struct
import time

# ========== 串口初始化 ==========
ser = serial.Serial('/dev/ttyAMA0', 4800, timeout=1)
time.sleep(1)

# ========== 摄像头初始化 ==========
cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# ===================== 【重点优化】红色HSV收紧阈值，抗肤色干扰 =====================
# 低饱和度、高亮度红色，过滤偏橙、偏粉肤色
# 第一段红色：0~6° 纯大红，拒绝橙色
lower_red1 = np.array([0, 150, 90])
upper_red1 = np.array([5, 255, 255])

# 第二段红色：174~180° 深红，杜绝暗部偏紫/偏肤色
lower_red2 = np.array([174, 150, 90])
upper_red2 = np.array([180, 255, 255])

TARGET_AREA = 4000
# 最小有效轮廓面积，小于直接丢弃（过滤人脸零星红点、手指噪点）
MIN_CONTOUR_AREA = 1500 

def circle_score(contour):
    """严格圆形打分，只有标准圆盘才能拿高分"""
    area = cv2.contourArea(contour)
    if area < MIN_CONTOUR_AREA:
        return -1

    peri = cv2.arcLength(contour, True)
    if peri <= 0:
        return -1

    # 圆形度 完美圆=1，人脸/手掌不规则图形远低于0.7
    circularity = 4 * np.pi * area / (peri * peri)
    (x, y, w, h) = cv2.boundingRect(contour)
    # 外接矩形长宽比，正圆接近1，人脸长条/扁块直接淘汰
    wh_ratio = min(w, h) / max(w, h)

    # 最小外接圆填充率
    (_, _), radius = cv2.minEnclosingCircle(contour)
    circle_area = np.pi * radius * radius
    fill_ratio = area / circle_area if circle_area > 0 else 0

    score = 0
    # 1. 圆形度严格门槛
    if circularity > 0.72:
        score += 1
    # 2. 填充率门槛
    if fill_ratio > 0.75:
        score += 1
    # 3. 外接框接近正方形（正圆约束）
    if wh_ratio > 0.85:
        score += 1
    return score

# ========== 状态变量 ==========
prev_cx, prev_cy = 320, 240
filtered_error_x = 0.0
filtered_error_area = 0.0
hold_error_x = 0
hold_error_area = 0

SEND_INTERVAL = 0.05
last_send_time = time.time()

# ========== 主循环 ==========
while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # ===================== 形态学操作强化：断开手和圆盘粘连、消除细小噪点 =====================
    kernel_small = np.ones((5, 5), np.uint8)
    kernel_big = np.ones((9, 9), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_small)  # 开运算：去除小白点噪点
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_big)   # 闭运算：填充圆盘内部空洞

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_contour = None
    best_score = -1
    MIN_SCORE = 2.0  # 必须拿到2分以上才判定为目标，大幅提高准入门槛

    for cnt in contours:
        s = circle_score(cnt)
        if s > best_score:
            best_score = s
            best_contour = cnt

    if best_contour is not None and best_score >= MIN_SCORE:
        area = cv2.contourArea(best_contour)
        M = cv2.moments(best_contour)
        if M['m00'] != 0:
            cx_raw = int(M['m10'] / M['m00'])
            cy_raw = int(M['m01'] / M['m00'])

            # 质心低通滤波防抖
            cx = int(0.7 * prev_cx + 0.3 * cx_raw)
            cy = int(0.7 * prev_cy + 0.3 * cy_raw)
            prev_cx, prev_cy = cx, cy

            x, y, w, h = cv2.boundingRect(best_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
            # 绘制最小外接圆，直观验证是否为圆形
            (center, r) = cv2.minEnclosingCircle(best_contour)
            cv2.circle(frame, (int(center[0]), int(center[1])), int(r), (255, 0, 0), 1)

            raw_error_x = cx - 320
            raw_error_area = TARGET_AREA - area

            filtered_error_x = 0.3 * raw_error_x + 0.7 * filtered_error_x
            filtered_error_area = 0.2 * raw_error_area + 0.8 * filtered_error_area

            if abs(filtered_error_x) < 5:
                filtered_error_x = 0.0
            if abs(filtered_error_area) < 200:
                filtered_error_area = 0.0

            hold_error_x = int(filtered_error_x)
            hold_error_area = int(filtered_error_area)

            cv2.putText(frame, f"ex:{hold_error_x}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Area:{area}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        hold_error_x = 0
        hold_error_area = 0
        filtered_error_x = 0.0
        filtered_error_area = 0.0
        cv2.putText(frame, "TARGET LOST", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.imshow('Track Frame', frame)
    cv2.imshow('Red Mask', mask)

    # ========== 定时发送串口数据 ==========
    now = time.time()
    if now - last_send_time >= SEND_INTERVAL:
        last_send_time = now
        ex = int(np.clip(hold_error_x, -32768, 32767))
        ea = int(np.clip(hold_error_area, -32768, 32767))
        packet = struct.pack('<BBhh', 0xAA, 0x02, ex, ea)
        ser.write(packet)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
ser.close()
cv2.destroyAllWindows()