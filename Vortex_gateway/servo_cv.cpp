#include "servo_cv.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <atomic>

#include <opencv2/opencv.hpp>

/* 0x04 SERVO_TURN 帧：payload 为质心坐标（int16 小端） */
uint8_t servo_cv_cmd[6] = {0xAA, 0x04, 0x00, 0x00, 0x00, 0x00};

/* 全局检测结果 */
servo_cv_info g_servo_cv;

/* 舵机自动跟踪开关：常开 */
std::atomic<bool> servo_turn{true};

/* 是否已有有效检测结果（首次检测到目标后才允许转发） */
static bool servo_valid = false;

/* main.cpp 全局退出标志 */
extern std::atomic<bool> go_running;

bool Get_Servo_turn()
{
    return servo_turn.load() && servo_valid;
}

void servo_cv_publish(bool found, int cx, int cy)
{
    g_servo_cv.is_find = found;
    g_servo_cv.cx = cx;
    g_servo_cv.cy = cy;

    /* 丢失：保留上一帧，云台停在最后位置，避免甩头/回中 */
    if (!found)
        return;

    servo_cv_cmd[2] = static_cast<uint8_t>(cx & 0xFF);
    servo_cv_cmd[3] = static_cast<uint8_t>((cx >> 8) & 0xFF);
    servo_cv_cmd[4] = static_cast<uint8_t>(cy & 0xFF);
    servo_cv_cmd[5] = static_cast<uint8_t>((cy >> 8) & 0xFF);
    servo_valid = true;
}

/* 与 gst_task 相同阈值/流程的红色圆盘检测，只输出质心（不标注、不推流） */
static bool detect_servo(const cv::Mat& bgr, int& out_cx, int& out_cy)
{
    const cv::Scalar kLowerRed1(0, 150, 90);
    const cv::Scalar kUpperRed1(5, 255, 255);
    const cv::Scalar kLowerRed2(174, 150, 90);
    const cv::Scalar kUpperRed2(180, 255, 255);
    constexpr int kMinContourArea = 100;
    constexpr double kMinScore = 2.0;

    cv::Mat hsv, mask1, mask2, mask;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, kLowerRed1, kUpperRed1, mask1);
    cv::inRange(hsv, kLowerRed2, kUpperRed2, mask2);
    cv::bitwise_or(mask1, mask2, mask);

    // 形态学强化：断开手和圆盘粘连、消除细小噪点
    const cv::Mat kernel_small = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    const cv::Mat kernel_big   = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel_small);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_big);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_score = -1.0;
    const std::vector<cv::Point>* best = nullptr;
    for (const auto& c : contours)
    {
        const double area = cv::contourArea(c);
        if (area < kMinContourArea)
            continue;
        const double peri = cv::arcLength(c, true);
        if (peri <= 0)
            continue;

        const double circularity = 4.0 * CV_PI * area / (peri * peri);
        const cv::Rect r = cv::boundingRect(c);
        const double wh_ratio = static_cast<double>(std::min(r.width, r.height)) /
                                static_cast<double>(std::max(r.width, r.height));

        cv::Point2f center;
        float radius = 0.0f;
        cv::minEnclosingCircle(c, center, radius);
        const double circle_area = CV_PI * radius * radius;
        const double fill_ratio = (circle_area > 0.0) ? area / circle_area : 0.0;

        int score = 0;
        if (circularity > 0.72) score += 1;
        if (fill_ratio > 0.75)  score += 1;
        if (wh_ratio > 0.85)    score += 1;
        if (score > best_score)
        {
            best_score = score;
            best = &c;
        }
    }

    if (best != nullptr && best_score >= kMinScore)
    {
        const cv::Moments m = cv::moments(*best);
        if (m.m00 != 0.0)
        {
            // 质心低通滤波防抖（与 gst_task 一致）
            static double prev_cx = 320.0, prev_cy = 240.0;
            const double cx = 0.7 * prev_cx + 0.3 * (m.m10 / m.m00);
            const double cy = 0.7 * prev_cy + 0.3 * (m.m01 / m.m00);
            prev_cx = cx;
            prev_cy = cy;
            out_cx = static_cast<int>(cx);
            out_cy = static_cast<int>(cy);
            return true;
        }
    }
    return false;
}

/* 独立舵机检测线程：VideoCapture 采集云台相机 → 检测 → 0x04 帧。
 * 画面不推流，不触碰 gst 推流管道与 /dev/video0 */
void servo_cv_thread()
{
    cv::VideoCapture cap(SERVO_CAM_DEVICE, cv::CAP_V4L2);
    if (!cap.isOpened())
    {
        std::fprintf(stderr, "[servo] 打开摄像头 %s 失败，舵机跟踪未启动\n", SERVO_CAM_DEVICE);
        return;
    }
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', '2'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);
    std::fprintf(stderr, "[servo] 舵机摄像头 %s 就绪，自动跟踪常开\n", SERVO_CAM_DEVICE);

    cv::Mat frame;
    while (go_running.load(std::memory_order_acquire))
    {
        if (!cap.read(frame))
            break;

        int cx = 0, cy = 0;
        const bool found = detect_servo(frame, cx, cy);
        servo_cv_publish(found, cx, cy);
    }

    cap.release();
    std::fprintf(stderr, "[servo] 舵机检测线程退出\n");
}
