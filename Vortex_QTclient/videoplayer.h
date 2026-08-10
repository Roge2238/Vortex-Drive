#ifndef VIDEOPLAYER_H
#define VIDEOPLAYER_H

#include <QObject>
#include <QImage>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

class VideoPlayer : public QObject
{
    Q_OBJECT
public:
    explicit VideoPlayer(QObject *parent = nullptr);
    ~VideoPlayer();

    void startPlay(const QString& rtspUrl);
    void stopPlay();
    QStringList getCameraDevices();

signals:
    void frameReady(const QImage& img);
    void playError(const QString& msg);
    void diagnosticLog(const QString& msg);   // 帧计数等诊断信息 

private:
    static GstFlowReturn onNewSample(GstAppSink* sink, gpointer userData);  // appsink回调
    GstElement* m_pipeline = nullptr;
    guint m_busWatchId = 0;
    int m_frameCount = 0;    // 已解码帧计数（诊断用）
};

#endif // VIDEOPLAYER_H
