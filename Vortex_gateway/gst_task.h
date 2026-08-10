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
 * gst_task — 摄像头 → 编码推流 全链路（单管道，两种模式共用）
 *
 * 数据流（内存转让，不做深拷贝）：
 *   v4l2src → videoconvert → appsink
 *                              │ new-sample 回调：只 ref buffer/caps 后入队（队列满丢最旧）
 *                              ▼
 *                          frame_q（内存转让：缓冲区所有权唯一）
 *                              │ gst_task 消费者线程：pop → 包装成 cv::Mat(零拷贝)
 *                              │   track_mode=true 时 检测+原地标注+发布串口帧
 *                              ▼
 *                    appsrc（push 转让 buffer）→ videoconvert → h264 → h264parse
 *                              → rtph264pay → udpsink → QT
 *
 * 模式由 recv_cmd 依据帧类型设置：
 *   0x01 手动 → track_mode=false（原图直通）
 *   0x02/0x03 自动 → track_mode=true（检测+标注）
 */

// 帧队列元素：所有权随对象转移，谁持有谁负责 unref
struct FrameData
{
    GstBuffer* buf;
    GstCaps* caps;
};

extern std::queue<FrameData> frame_q;
extern std::mutex frame_q_mtx;
extern std::condition_variable cv_q;

// false=手动(原图直通)  true=跟踪(自动)
extern std::atomic<bool> track_mode;

/* 串口线程用：取最新检测帧 [0xAA][0x02/0x03][err_lo][err_hi][area_lo][area_hi] */
bool vision_get_frame(uint8_t* out, size_t* out_len);

/* 视觉消费者线程是否存活（异常退出时串口线程补发LOST兜底停车） */
bool vision_is_alive();

void init_gst();
void gst_task();

#endif // GST_TASK_H
