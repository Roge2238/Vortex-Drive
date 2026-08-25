import cv2
import numpy as np
from vision_handler import circle_score
import serial
import struct
import time




SERIAL_PORT = '/dev/ttyAMA0'      
SERIAL_BAUD = 115200

MAGIC_HEAD = 0xAA   # 帧头
TYPE_SERVO = 0x04   # 舵机转向帧：err_x, err_y（int16 小端）

def pack_servo_frame(err_x, err_y):
    """打包 0x04 舵机转向帧: AA 04 [err_x lo] [err_x hi] [err_y lo] [err_y hi]"""
    ex = int(max(-32768, min(32767, err_x)))
    ey = int(max(-32768, min(32767, err_y)))
    return struct.pack('<BBhh', MAGIC_HEAD, TYPE_SERVO, ex, ey)

try:
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
    print(f"Serial {SERIAL_PORT} opened @ {SERIAL_BAUD}")
except serial.SerialException as e:
    print(f"WARN: 串口 {SERIAL_PORT} 打开失败: {e}")
    ser = None

SEND_INTERVAL = 0.02   # 20ms 发送周期
last_send = time.monotonic()
send_x = 0   # 待发送误差（目标丢失时保持上一帧值，不重置）
send_y = 0

cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Cannot open camera")
    exit(1)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        break

    center_x = frame.shape[1] // 2
    center_y = frame.shape[0] // 2
    cv2.circle(frame, (center_x, center_y), 5, (95, 80, 35), -1) # 画出中心瞄准点
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    
    mask_red = cv2.bitwise_or(mask1, mask2)

    contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_score = -1
    best_det = None

    for det in contours:
        score, fill_ratio, wh_ratio = circle_score(det)
        if score > best_score:
            best_score = score
            best_det = det

    if best_det is not None and best_score > 2:  # 有满足条件的圆形
        M = cv2.moments(best_det)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            error_x = cx - center_x
            error_y = cy - center_y
            cv2.circle(frame, (cx, cy), 5, (30, 50, 60), -1)
            cv2.line(frame, (cx, cy), (center_x, center_y), (0, 255, 0), 2)
            cv2.putText(frame, f"error_x: {error_x}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, f"error_y: {error_y}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
           
            send_x = error_x
            send_y = error_y
    else:
        is_find = False
        

    cv2.imshow("mask_red", mask_red)
    cv2.imshow("Camera", frame)

    
    now = time.monotonic()
    if now - last_send >= SEND_INTERVAL:
        last_send = now
        if ser is not None:
            ser.write(pack_servo_frame(send_x, send_y))

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
if ser is not None:
    ser.close()
cv2.destroyAllWindows()
 

