#include "gst_loop.h"

static gboolean bus_callback(GstBus* bus, GstMessage* msg, gpointer loop_ptr)
{
    GMainLoop* loop = (GMainLoop*)loop_ptr;
    if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR)
    {
        GError* err;
        gchar* dbg;
        gst_message_parse_error(msg, &err, &dbg);
        g_print("err: %s\n", err->message);
        g_error_free(err);
        g_free(dbg);
        g_main_loop_quit(loop);
    }
    else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS)
    {
        g_print("流结束\n");
        g_main_loop_quit(loop);
    }
    return TRUE;
}

// 定时器回调：检查退出标志
static gboolean check_exit(gpointer loop_ptr)
{
    if (!go_running.load())
    {
        g_main_loop_quit((GMainLoop*)loop_ptr);
        return FALSE;  // 移除定时器
    }
    return TRUE;  // 继续运行
}

void gst_udp_stream()
{
    gst_init(nullptr, nullptr);

    const char* host = "192.168.43.95";
    guint port = 8650;

    gchar* pipe_txt = g_strdup_printf(
        "v4l2src device=/dev/video0 ! "
        "video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! "
        "videoconvert ! "
        "x264enc tune=zerolatency bitrate=1000 speed-preset=ultrafast ! "
        "h264parse ! "
        "rtph264pay config-interval=1 pt=96 ! "
        "udpsink host=%s port=%u",
        host, port
    );

    GError* err = nullptr;
    GstElement* pipeline = gst_parse_launch(pipe_txt, &err);
    g_free(pipe_txt);

    if (!pipeline)
    {
        g_print("创建管道失败: %s\n", err->message);
        g_error_free(err);
        return;
    }

    GMainLoop* loop = g_main_loop_new(nullptr, FALSE);
    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    gst_bus_add_watch(bus, bus_callback, loop);
    gst_object_unref(bus);


    g_timeout_add(100, check_exit, loop);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    g_print("UDP推流启动，目标 %s:%u\n", host, port);

    // 正常运行主循环，由定时器或bus回调触发退出
    g_main_loop_run(loop);

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    g_main_loop_unref(loop);
    g_print("UDP推流线程退出\n");
}