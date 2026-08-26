#include "common.h"
#include "gst_task.h"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <cstring>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <mutex>
#define MIN_TARGET_AREA 

struct servo_cv_info;

servo_cv_info.is_find = false;
servo_cv_info.cx = 0;
servo_cv_info.cy = 0;



std:: atomic<bool> servo_turn{false};
uint8_t servo_cv_cmd[6] = {0xAA, 0x04, 0x00, 0x00, 0x00, 0x00};
std::mutex servo_turn_mtx;

static void servo_frame_handler(Mat& frame, FilterState& state, )
{
    Mat hsv, mask, mask1 mask2;

    Scalar lower_red1(0, 150, 90);
    Scalar upper_red1(5, 255, 255);
    Scalar lower_red2(174, 150, 90);
    Scalar upper_red2(180, 255, 255);
    
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    inRange(hsv, lower_red1, upper_red1, mask1);
    inRange(hsv, lower_red2, upper_red2, mask2);

    bitwise_or(mask1, mask2, mask);

    // 形态学强化




   //
    vector<vector<Point>> contours;
    vector<Vec4i> hierachy;
    findContours(mask, contours, hierachy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    float best_score = -1;
    vector<Point> best_contour;
    float area;
    for(size_t = 0; i< contours.size(); i++)
    {
        area = contourArea(contours[i]);
        if(area < MIN_TARGET_AREA)
        {
            continue;
        }
        float score = circle_score(contours[i], fill_ratio, wh_ratio);
        if(score > best_score)
        {
            best_score = score;
            best_contour = contours[i];
        }
    }
    if(best_score > 2 )
    {

    }

}

void servo_cv_thread()
{
    VideoCapture cap(1);

    if(!cap.isOpened())
    {
        printf("打开摄像头失败\n");
        return;
    }
    cap.set(CAP_PROP_FRAEM_WIDTH, 640);
    cap.set(CAP_PROP_FRAEM_HEIGHT, 480);

    Mat frame;

    while (go_running.load())
    {
        cap.read(frame); 

    }

    //资源释放 
    cap.release();

}




namespace servo_cv
{
    void append_pkg(uint8_t* pkg, size_t len)
    {
        uint8_t  f[6];
        f[0] = 0xAA;
        f[1] = 0x04;
        f[2] = static_cast<uint8_t>(servo_cv_info.cx & 0xFF);
        f[3] = static_cast<uint8_t>((servo_cv_info.cx >> 8) & 0xFF);
        f[4] = static_cast<uint8_t>(servo_cv_info.cy & 0xFF) ;
        f[5] = static_cast<uint8_t>((servo_cv_info.cy >> 8) & 0xFF);
    }

    {
        lock_guard lock(servo_turn_mtx);
        std::memcpy(servo_cv_cmd, f, sizeof(f));
    }

}