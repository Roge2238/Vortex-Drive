#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>
#include <QTimer>
#include <QKeyEvent>
#include <QDebug>
#include <QByteArray>
#include <QTcpSocket>
#include <QAbstractSocket>
#include <QTextEdit>
#include <QDateTime>

#define CMD_LEN 6
#define RECONNECT_INTERVAL 3000  // 重连间隔3秒
#define MAX_RECONNECT_ATTEMPTS 5 // 最大重连次数

class VideoPlayer;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(bool testMode = false, QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void onPlayClicked();
    void onCameraClicked();
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
    void focusOutEvent(QFocusEvent *event) override;    
    void CheckKey_send();// Check Key
    

    void onConnected();
    void onDisconnected();
    void onErrorOccurred(QAbstractSocket::SocketError error);
    void reconnect();
    void onSendCmdClicked();  // 测试模式手动发送指令
    
    
    void appendLog(const QString& msg);

private:
    QLabel* m_videoLabel = nullptr;
    VideoPlayer* m_video = nullptr;
    QLineEdit* m_urlEdit = nullptr;
    QPushButton* m_playBtn = nullptr;
    QPushButton* m_cameraBtn = nullptr;
    QComboBox* m_cameraCombo = nullptr;
    bool keyBox[6] = {false, false, false, false, false, false};
    QTimer* m_timer = nullptr;
    QTcpSocket* send_box = nullptr;
    
    QTimer* m_reconnectTimer = nullptr;
    int m_reconnectCount = 0;
    QString m_serverHost = "192.168.43.96";
    quint16 m_serverPort = 8651;
    
    // 测试模式控件
    QLineEdit* m_cmdInput = nullptr;
    QPushButton* m_sendCmdBtn = nullptr;
    
    // 日志显示
    QTextEdit* m_logEdit = nullptr;
};

#endif // MAINWINDOW_H
