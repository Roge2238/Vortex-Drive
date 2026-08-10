#include "gst_task.h"
#include "common.h"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <cstring>
#include <cstdio>
#include <vector>
#include <algorithm>

/* ================= 帧队列（内存转让） ================= */
std::queue<FrameData> frame_q;
std::mutex frame_q_mtx;
std::condition_variable cv_q;

/* 模式：false=手动原图直通  true=跟踪(检测+标注)，由 recv_cmd 设置 */
std::atomic<bool> track_mode{false};

namespace {

/* 队列上限：超出丢最旧帧（内存转让，unref 归还缓冲池） */
constexpr std::size_t kMaxQueueSize = 4;

/* 检测结果 → 串口（覆盖式，串口线程转发） */
std::mutex detect_mtx;
uint8_t detect_frame[6];
bool detect_valid = false;
std::atomic<bool> vision_alive{false};

/* 管道故障（摄像头拔出/编码错误/EOS），置位后消费者退出，串口兜底发LOST */
std::atomic<bool> gst_fault{false};

GstElement* g_pipeline = nullptr;
GstElement* g_appsrc = nullptr;

/* ================= 视觉检测参数（与 opencv_vision.py 一致） ================= */
const cv::Scalar kLowerRed1(0, 150, 90);
const cv::Scalar kUpperRed1(5, 255, 255);
const cv::Scalar kLowerRed2(174, 150, 90);
const cv::Scalar kUpperRed2(180, 255, 255);
constexpr int kTargetArea      = 12000;
constexpr int kMinContourArea  = 100;
constexpr int kImageW          = 640;
constexpr int kImageH          = 480;
constexpr int kFrameCenterX    = 320;
constexpr double kMinScore     = 2.0;

/* 一阶低通滤波 + 死区 状态 */
struct FilterState
{
    double filtered_error_x = 0.0;
    double filtered_error_area = 0.0;
    double prev_cx = kFrameCenterX;
    double prev_cy = kImageH / 2;
    int hold_error_x = 0;
    int hold_error_area = 0;
    bool target_found = false;
};

/* 圆形度评分：与 Python circle_score 等价 */
int circle_score(const std::vector<cv::Point>& contour, double& fill_ratio, double& wh_ratio)
{
    const double area = cv::contourArea(contour);
    if (area < kMinContourArea)
        return -1;

    const double peri = cv::arcLength(contour, true);
    if (peri <= 0)
        return -1;

    const double circularity = 4.0 * CV_PI * area / (peri * peri);
    const cv::Rect r = cv::boundingRect(contour);
    wh_ratio = static_cast<double>(std::min(r.width, r.height)) /
               static_cast<double>(std::max(r.width, r.height));

    cv::Point2f center;
    float radius = 0.0f;
    cv::minEnclosingCircle(contour, center, radius);
    const double circle_area = CV_PI * radius * radius;
    fill_ratio = (circle_area > 0.0) ? area / circle_area : 0.0;

    int score = 0;
    if (circularity > 0.72) score += 1;
    if (fill_ratio > 0.75)  score += 1;
    if (wh_ratio > 0.85)    score += 1;
    return score;
}

/* 单帧检测：原地标注 img，输出 error(横向偏移) / area(面积差) */
void detect(cv::Mat& img, FilterState& st, int& out_err, int& out_area, bool& out_found)
{
    cv::Mat hsv, mask1, mask2, mask;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
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
        double fill_ratio = 0.0, wh_ratio = 0.0;
        const int s = circle_score(c, fill_ratio, wh_ratio);
        if (s > best_score)
        {
            best_score = s;
            best = &c;
        }
    }

    if (best != nullptr && best_score >= kMinScore)
    {
        const cv::Moments m = cv::moments(*best);
        if (m.m00 != 0.0)
        {
            const double cx_raw = m.m10 / m.m00;
            const double cy_raw = m.m01 / m.m00;

            // 质心低通滤波防抖
            const int cx = static_cast<int>(0.7 * st.prev_cx + 0.3 * cx_raw);
            const int cy = static_cast<int>(0.7 * st.prev_cy + 0.3 * cy_raw);
            st.prev_cx = cx;
            st.prev_cy = cy;

            const double area = cv::contourArea(*best);
            const double raw_err_x = cx - kFrameCenterX;
            const double raw_err_area = kTargetArea - area;

            st.filtered_error_x = 0.3 * raw_err_x + 0.7 * st.filtered_error_x;
            st.filtered_error_area = 0.2 * raw_err_area + 0.8 * st.filtered_error_area;

            // 死区：STM32 端 AREA_DEADZONE 负责精确停车
            if (std::fabs(st.filtered_error_x) < 5.0)
                st.filtered_error_x = 0.0;

            st.hold_error_x = static_cast<int>(st.filtered_error_x);
            st.hold_error_area = static_cast<int>(st.filtered_error_area);
            st.target_found = true;

            // 原地标注
            const cv::Rect r = cv::boundingRect(*best);
            cv::rectangle(img, r, cv::Scalar(0, 255, 0), 2);
            cv::circle(img, cv::Point(cx, cy), 6, cv::Scalar(0, 0, 255), -1);
            cv::Point2f c;
            float rad = 0.0f;
            cv::minEnclosingCircle(*best, c, rad);
            cv::circle(img, cv::Point(c), static_cast<int>(rad), cv::Scalar(255, 0, 0), 1);
        }
    }
    else
    {
        st.target_found = false;
        st.hold_error_x = 0;
        st.hold_error_area = 0;
        st.filtered_error_x = 0.0;
        st.filtered_error_area = 0.0;
    }

    out_err = st.hold_error_x;
    out_area = st.hold_error_area;
    out_found = st.target_found;
}

/* 发布检测帧到共享区（串口线程每20ms转发） */
void publish_detection(int err, int area, bool found)
{
    uint8_t f[6];
    f[0] = 0xAA;
    f[1] = found ? 0x02 : 0x03;                       // AUTO / LOST
    f[2] = static_cast<uint8_t>(err & 0xFF);
    f[3] = static_cast<uint8_t>((err >> 8) & 0xFF);
    f[4] = static_cast<uint8_t>(area & 0xFF);
    f[5] = static_cast<uint8_t>((area >> 8) & 0xFF);

    std::lock_guard<std::mutex> lock(detect_mtx);
    std::memcpy(detect_frame, f, sizeof(f));
    detect_valid = true;
}

} // namespace

/* ================= appsink 取帧回调（流线程，只入队不处理） ================= */
static GstFlowReturn on_new_sample(GstAppSink* sink, gpointer /*user_data*/)
{
    GstSample* sample = gst_app_sink_pull_sample(sink);
    if (!sample)
        return GST_FLOW_OK;

    // 内存转让：只对 buffer/caps 增引用，sample 随即释放
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstCaps* caps = gst_sample_get_caps(sample);
    gst_buffer_ref(buffer);
    if (caps)
        gst_caps_ref(caps);

    {
        std::lock_guard<std::mutex> lock(frame_q_mtx);
        if (frame_q.size() >= kMaxQueueSize)
        {
            const FrameData old = frame_q.front();
            frame_q.pop();
            gst_buffer_unref(old.buf);
            if (old.caps)
                gst_caps_unref(old.caps);
        }
        frame_q.push({buffer, caps});
    }
    cv_q.notify_one();

    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

/*  总线错误/EOS 同步回调 */
static GstBusSyncReply on_bus_message(GstBus* /*bus*/, GstMessage* msg, gpointer /*user_data*/)
{
    if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR)
    {
        GError* err = nullptr;
        gchar* debug = nullptr;
        gst_message_parse_error(msg, &err, &debug);
        std::fprintf(stderr, "[gst] 管道错误: %s\n", err ? err->message : "(null)");
        if (err) g_error_free(err);
        g_free(debug);
        gst_fault.store(true);
        cv_q.notify_all();
    }
    else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS)
    {
        std::fprintf(stderr, "[gst] 流结束(EOS)\n");
        gst_fault.store(true);
        cv_q.notify_all();
    }
    return GST_BUS_DROP;
}

/* ================= 管道搭建 ================= */
void init_gst()
{
    const char* host = "192.168.43.95";
    const int port = 8650;

    g_pipeline = gst_pipeline_new("pipeline");
    GstElement* v4l2src  = gst_element_factory_make("v4l2src", "cam");
    GstElement* pre_conv = gst_element_factory_make("videoconvert", "conv-in");
    GstElement* appsink  = gst_element_factory_make("appsink", "appsink");
    g_appsrc             = gst_element_factory_make("appsrc", "appsrc");
    GstElement* post_conv = gst_element_factory_make("videoconvert", "conv-out");
    GstElement* enc      = gst_element_factory_make("h264_v4l2m2m", "hw-enc");
    GstElement* h264parse = gst_element_factory_make("h264parse", "h264parse");
    GstElement* rtppay   = gst_element_factory_make("rtph264pay", "rtppay");
    GstElement* udpsink  = gst_element_factory_make("udpsink", "net-send");

    // 树莓派硬件编码器缺失时回退软件 x264
    if (!enc)
        enc = gst_element_factory_make("x264enc", "sw-enc");

    if (!g_pipeline || !v4l2src || !pre_conv || !appsink || !g_appsrc ||
        !post_conv || !enc || !h264parse || !rtppay || !udpsink)
    {
        std::fprintf(stderr, "[gst] 创建元素失败，请检查 gstreamer 插件是否安装\n");
        return;
    }

    g_object_set(v4l2src, "device", "/dev/video0", nullptr);

    // appsink 输出 BGR，OpenCV 直接零拷贝包装
    GstCaps* sink_caps = gst_caps_from_string(
        "video/x-raw,format=BGR,width=640,height=480,framerate=30/1");
    g_object_set(appsink, "caps", sink_caps,
                 "sync", FALSE,
                 "max-buffers", 2,
                 "drop", TRUE, nullptr);
    gst_caps_unref(sink_caps);

    // appsrc 输入与 appsink 一致（BGR），格式固定
    GstCaps* src_caps = gst_caps_from_string(
        "video/x-raw,format=BGR,width=640,height=480,framerate=30/1");
    g_object_set(g_appsrc, "is-live", TRUE,
                 "do-timestamp", FALSE,
                 "format", GST_FORMAT_TIME, nullptr);
    gst_app_src_set_caps(GST_APP_SRC(g_appsrc), src_caps);   // 所有权转让给 appsrc
    // 注意：src_caps 不再 unref

    g_object_set(rtppay, "config-interval", 1, "pt", 96, nullptr);
    g_object_set(udpsink, "host", host, "port", port, "sync", FALSE, nullptr);

    // appsink 出帧回调
    g_signal_connect(appsink, "new-sample", G_CALLBACK(on_new_sample), nullptr);

    gst_bin_add_many(GST_BIN(g_pipeline),
                     v4l2src, pre_conv, appsink,
                     g_appsrc, post_conv, enc, h264parse, rtppay, udpsink, nullptr);

    // 段1：摄像头 → appsink（OpenCV 处理）
    gst_element_link_many(v4l2src, pre_conv, appsink, nullptr);

    // 段2：appsink 处理后 → appsrc → 编码 → 推流
    gst_element_link_many(g_appsrc, post_conv, enc, h264parse, rtppay, udpsink, nullptr);

    
    // 回调返回 GST_BUS_DROP 表示消息已消费；第4参是 user_data 的销毁回调
    GstBus* bus = gst_element_get_bus(g_pipeline);
    gst_bus_set_sync_handler(bus, on_bus_message, nullptr, nullptr);
    gst_object_unref(bus);

    if (gst_element_set_state(g_pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
    {
        std::fprintf(stderr, "[gst] 管道启动失败\n");
        return;
    }
    std::fprintf(stderr, "[gst] 推流启动，目标 %s:%d\n", host, port);
}

/* ================= 消费者线程（OpenCV 处理 + appsrc） ================= */
void gst_task()
{
    gst_init(nullptr, nullptr);
    init_gst();

    // 管道初始化失败（摄像头/插件缺失）：直接退出，串口线程走 LOST 兜底
    if (!g_pipeline)
    {
        vision_alive.store(false);
        return;
    }

    FilterState filter;
    vision_alive.store(true);

    while (go_running.load())
    {
        FrameData frame;
        {
            std::unique_lock<std::mutex> lock(frame_q_mtx);
            cv_q.wait(lock, [] {
                return !frame_q.empty() || !go_running.load() || gst_fault.load();
            });
            if (frame_q.empty() || gst_fault.load())
                break;
            frame = frame_q.front();
            frame_q.pop();
        }

        // 零拷贝：把 GstBuffer 内存包装成 cv::Mat（无深拷贝）
        GstMapInfo map;
        if (!gst_buffer_map(frame.buf, &map, GST_MAP_READWRITE))
        {
            gst_buffer_unref(frame.buf);
            if (frame.caps) gst_caps_unref(frame.caps);
            continue;
        }

        cv::Mat img(kImageH, kImageW, CV_8UC3, map.data);

        if (track_mode.load())
        {
            int err = 0, area = 0;
            bool found = false;
            detect(img, filter, err, area, found);   // 原地标注，零拷贝
            publish_detection(err, area, found);
        }

        gst_buffer_unmap(frame.buf, &map);

        // 内存转让给 appsrc（push 成功后 buffer 归管道，不再 unref）
        if (gst_app_src_push_buffer(GST_APP_SRC(g_appsrc), frame.buf) != GST_FLOW_OK)
            gst_buffer_unref(frame.buf);
        if (frame.caps)
            gst_caps_unref(frame.caps);   // caps 格式固定，已在 init 时设给 appsrc
    }

    vision_alive.store(false);

    // 清空残留队列
    for (;;)
    {
        FrameData frame;
        {
            std::lock_guard<std::mutex> lock(frame_q_mtx);
            if (frame_q.empty())
                break;
            frame = frame_q.front();
            frame_q.pop();
        }
        gst_buffer_unref(frame.buf);
        if (frame.caps) gst_caps_unref(frame.caps);
    }

    if (g_pipeline)
    {
        gst_element_set_state(g_pipeline, GST_STATE_NULL);
        gst_object_unref(g_pipeline);
        g_pipeline = nullptr;
    }
    std::fprintf(stderr, "[gst] 推流线程退出\n");
}

/* ================= 对外接口 ================= */
bool vision_get_frame(uint8_t* out, size_t* out_len)
{
    std::lock_guard<std::mutex> lock(detect_mtx);
    if (!detect_valid)
        return false;
    std::memcpy(out, detect_frame, sizeof(detect_frame));
    *out_len = sizeof(detect_frame);
    return true;
}

bool vision_is_alive()
{
    return vision_alive.load();
}
