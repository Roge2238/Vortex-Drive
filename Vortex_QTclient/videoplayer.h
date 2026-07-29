#ifndef VIDEOPLAYER_H
#define VIDEOPLAYER_H

#include <QObject>
#include <QImage>
#include <gst/gst.h>

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

private:
    GstElement* m_pipeline = nullptr;
    guint m_busWatchId = 0;
};

#endif // VIDEOPLAYER_H
