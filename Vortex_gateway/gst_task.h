#ifndef GST_TASK_H
#define GST_TASK_H

#include <gst/gst.h>
#include <glib.h>
#include <glib-object.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

/*
 * gst_task — 采集 / 推流  我们使用两条独立 GstPipeline
 *
 *   管道1进行采集，：v4l2src → videoconvert → BGR → appsink
 *                         └ new-sample 回调只 ref buffer/caps 入队
 *   队列 frame_q 内存转让，不做深拷贝
 *   OpenCV线程：pop → 零拷贝包装 cv::Mat → track_mode 时 检测+原地标注+发布串口帧
 *              → gst_app_src_push_buffer 转让给管道2
 *   管道2进行推流：appsrc(do-timestamp=true) → videoconvert → I420 → x264enc
 *              → h264parse → rtph264pay → udpsink → QT（两种模式统一推流）
 *
 * 为什么拆两条管道：单管道 appsrc 回环时，udpsink 的 preroll 依赖应用喂 appsrc，
 * 而应用要等 appsink 的 new-sample——PAUSED 阶段互相等导致死锁。
 * 拆开后采集管道自驱动到 PLAYING，回调稳定触发，推流管道被动接收即可喵。
 *
 * 模式由 recv_cmd 依据帧类型设置：
 *   0x01 手动 → track_mode=false ：直接推原图，不检测
 *   0x02/0x03 自动 → track_mode=true ：检测+标注+发布串口帧
 */

// 帧队列元素：所有权随对象转移，记得 unref
struct FrameData
{
    GstBuffer* buf;
    GstCaps* caps;
};

extern std::queue<FrameData> frame_q;
extern std::mutex frame_q_mtx;
extern std::condition_variable cv_q;

// false=手动 true=跟踪
extern std::atomic<bool> track_mode;

/* 串口线程用：取最新检测帧 [0xAA][0x02/0x03][err_lo][err_hi][area_lo][area_hi] */
bool vision_get_frame(uint8_t* out, size_t* out_len);

/* 视觉线程是否存活 异常退出时可以串口线程补发LOST停车 */
bool vision_is_alive();

void init_gst();
void gst_task();

#endif // GST_TASK_H
