#include "videoplayer.h"
#include <gst/app/gstappsink.h>
#include <QDebug>
#include <QMessageBox>

GstFlowReturn VideoPlayer::onNewSample(GstAppSink* sink, gpointer userData)
{
    VideoPlayer* player = static_cast<VideoPlayer*>(userData);
    GstSample* sample = gst_app_sink_pull_sample(sink);
    if (!sample) return GST_FLOW_OK;

    //确认收到并解码出帧
    player->m_frameCount++;
    if (player->m_frameCount == 1 || player->m_frameCount % 150 == 0)
        emit player->diagnosticLog(QString("已解码视频帧: %1").arg(player->m_frameCount));

    GstBuffer* buf = gst_sample_get_buffer(sample);
    GstCaps* caps = gst_sample_get_caps(sample);
    GstStructure* s = gst_caps_get_structure(caps, 0);

    int w, h;
    gst_structure_get_int(s, "width", &w);
    gst_structure_get_int(s, "height", &h);

    GstMapInfo map;
    gst_buffer_map(buf, &map, GST_MAP_READ);
    
    // 限制分辨率，最大1280x720，避免内存不足
    if (w > 1280 || h > 720) {
        QImage frame(map.data, w, h, QImage::Format_RGB888);
        QImage scaled = frame.scaled(1280, 720, Qt::KeepAspectRatio, Qt::FastTransformation);
        emit player->frameReady(scaled);
    } else {
        QImage frame(map.data, w, h, QImage::Format_RGB888);
        emit player->frameReady(frame.copy());
    }

    gst_buffer_unmap(buf, &map);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

static gboolean onBusMessage(GstBus* bus, GstMessage* msg, gpointer userData)
{
    VideoPlayer* player = static_cast<VideoPlayer*>(userData);
    switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
        gchar* debug;
        GError* err;
        gst_message_parse_error(msg, &err, &debug);
        QString errMsg = QString("GStreamer错误: %1 (%2)").arg(err->message).arg(debug ? debug : "");
        qWarning() << errMsg;
        emit player->playError(errMsg);
        g_error_free(err);
        g_free(debug);
        break;
    }
    case GST_MESSAGE_EOS:
        qDebug() << "GStreamer: 流结束";
        break;
    case GST_MESSAGE_WARNING: {
        gchar* debug;
        GError* err;
        gst_message_parse_warning(msg, &err, &debug);
        qWarning() << "GStreamer警告:" << err->message << (debug ? debug : "");
        g_error_free(err);
        g_free(debug);
        break;
    }
    default:
        break;
    }
    return TRUE;
}

VideoPlayer::VideoPlayer(QObject *parent) : QObject(parent)
{
    gst_init(nullptr, nullptr);
}

VideoPlayer::~VideoPlayer()
{
    stopPlay();
}

void VideoPlayer::startPlay(const QString &rtspUrl)
{
    if (m_pipeline) stopPlay();

    QString pipeStr;
    if (rtspUrl.startsWith("camera:")) {
        QString deviceName = rtspUrl.mid(7);
        pipeStr = QString(
                      "ksvideosrc device-name=\"%1\" ! "
                      "videoconvertscale ! video/x-raw,format=RGB,width=1280,height=720 ! "
                      "appsink name=sink sync=false"
                      ).arg(deviceName);
    } else {
        // 默认UDP RTP接收
        QString addr = rtspUrl.mid(6);  // 去掉 "udp://"
        QString port = addr.split(":").last();
        pipeStr = QString(
                      "udpsrc port=%1 ! "
                      "application/x-rtp,media=video,encoding-name=H264,payload=96 ! "
                      "rtph264depay ! "
                      "h264parse ! "
                      "avdec_h264 ! "
                      "videoconvertscale ! video/x-raw,format=RGB,width=1280,height=720 ! "
                      "appsink name=sink sync=false"
                      ).arg(port);
    }

    GError* err = nullptr;
    m_pipeline = gst_parse_launch(pipeStr.toStdString().c_str(), &err);
    if (!m_pipeline)
    {
        QString errMsg = QString("GStreamer管道创建失败!\n");
        if (err) {
            errMsg += QString("错误信息: %1\n").arg(err->message);
            g_error_free(err);
        }
        errMsg += QString("请检查:\n1. GST_PLUGIN_PATH是否设置正确\n2. 相关GStreamer插件是否存在\n3. ");
        qWarning() << errMsg;
        emit playError(errMsg);
        return;
    }

    GstElement* sink = gst_bin_get_by_name(GST_BIN(m_pipeline), "sink");
    GstAppSinkCallbacks cb;
    memset(&cb, 0, sizeof(cb));
    cb.new_sample = onNewSample;
    gst_app_sink_set_callbacks(GST_APP_SINK(sink), &cb, this, nullptr);
    gst_object_unref(sink);

    GstBus* bus = gst_element_get_bus(m_pipeline);
    m_busWatchId = gst_bus_add_watch(bus, onBusMessage, this);
    gst_object_unref(bus);

    gst_element_set_state(m_pipeline, GST_STATE_PLAYING);
}

QStringList VideoPlayer::getCameraDevices()
{
    QStringList devices;
    GstDeviceMonitor* monitor = gst_device_monitor_new();
    gst_device_monitor_add_filter(monitor, "Video/Source", nullptr);

    if (gst_device_monitor_start(monitor)) {
        GList* dev_list = gst_device_monitor_get_devices(monitor);
        for (GList* l = dev_list; l != nullptr; l = l->next) {
            GstDevice* dev = GST_DEVICE(l->data);
            const gchar* name = gst_device_get_display_name(dev);
            devices.append(QString(name));
            gst_object_unref(dev);
        }
        g_list_free(dev_list);
        gst_device_monitor_stop(monitor);
    }
    gst_object_unref(monitor);
    return devices;
}

void VideoPlayer::stopPlay()
{
    if (m_busWatchId) {
        g_source_remove(m_busWatchId);
        m_busWatchId = 0;
    }
    if (m_pipeline)
    {
        gst_element_set_state(m_pipeline, GST_STATE_NULL);
        gst_element_get_state(m_pipeline, nullptr, nullptr, GST_CLOCK_TIME_NONE);
        gst_object_unref(m_pipeline);
        m_pipeline = nullptr;
    }
}
