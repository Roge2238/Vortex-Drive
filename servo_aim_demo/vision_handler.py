
import cv2
import numpy as np
import math 

def circle_score(contour, kMinContourArea = 20): 
    """
    :KMinContour  最小轮廓面积 
    return: (score, fill_ratio, wh_ratio)
        score:0~3分，越高越像圆；-1代表无效轮廓
        fill_ratio: 轮廓面积 / 最小外接圆面积
        wh_ratio: 外接矩形宽高比，min(w,h)/max(w,h)
    """

    area = cv2.contourArea(contour)
    if area < kMinContourArea:
        return -1, 0.0, 0.0

    peri = cv2.arcLength(contour, True)
    if peri <= 1e-6:
        return -1, 0.0, 0.0

    # 圆度 4π*S / L²；完美圆形=1
    circularity = 4.0 * math.pi * area / (peri * peri)

    r = cv2.boundingRect(contour)
    x, y, w, h = r
    wh_ratio = min(w, h) / max(w, h)

    # 最小包围圆
    (cx, cy), radius = cv2.minEnclosingCircle(contour)
    circle_area = math.pi * radius * radius

    if circle_area > 1e-6:
        fill_ratio = area / circle_area
    else:
        fill_ratio = 0.0

    score = 0
    if circularity > 0.72:
        score += 1
    if fill_ratio > 0.75:
        score += 1
    if wh_ratio > 0.85:
        score += 1

    return score, fill_ratio, wh_ratio