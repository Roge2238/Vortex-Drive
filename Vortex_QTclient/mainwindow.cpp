#include "mainwindow.h"
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QTextCursor>
#include "VideoPlayer.h"

MainWindow::MainWindow(bool testMode, QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle("Vortex RTSP Player");
    resize(1280, 900);  // 设置窗口大小

    QWidget* central = new QWidget(this);
    QVBoxLayout* layout = new QVBoxLayout(central);
    setCentralWidget(central);

    QHBoxLayout* ctrlLayout = new QHBoxLayout();
    
    m_urlEdit = new QLineEdit("udp://192.168.43.95:8650", this);
    ctrlLayout->addWidget(m_urlEdit);

    m_playBtn = new QPushButton("播放", this);
    ctrlLayout->addWidget(m_playBtn);

    m_cameraBtn = new QPushButton("摄像头", this);
    ctrlLayout->addWidget(m_cameraBtn);

    m_cameraCombo = new QComboBox(this);
    ctrlLayout->addWidget(m_cameraCombo);

    layout->addLayout(ctrlLayout);

    // 这个功能已弃用
    if (testMode) {
        QHBoxLayout* cmdLayout = new QHBoxLayout();
        m_cmdInput = new QLineEdit(this);
        m_cmdInput->setPlaceholderText("手动输入指令（6位二进制，如100000）");
        m_sendCmdBtn = new QPushButton("发送指令", this);
        cmdLayout->addWidget(m_cmdInput);
        cmdLayout->addWidget(m_sendCmdBtn);
        layout->addLayout(cmdLayout);
        
        connect(m_sendCmdBtn, &QPushButton::clicked, this, &MainWindow::onSendCmdClicked);
    }

    m_videoLabel = new QLabel("视频加载中...", this);
    m_videoLabel->setAlignment(Qt::AlignCenter);
    m_videoLabel->setStyleSheet("background-color: #000000; color: #ffffff;");
    m_videoLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    layout->addWidget(m_videoLabel, 1);  // 视频区域占主要空间

    // 日志显示区域
    m_logEdit = new QTextEdit(this);
    m_logEdit->setReadOnly(true);
    m_logEdit->setFixedHeight(120);
    m_logEdit->setStyleSheet("background-color: #1e1e1e; color: #00ff00; font-family: Consolas; font-size: 12px;");
    layout->addWidget(m_logEdit);

    m_video = new VideoPlayer(this);

    QStringList devices = m_video->getCameraDevices();
    for (const QString& name : devices) {
        m_cameraCombo->addItem(name);
    }

    connect(m_video, &VideoPlayer::frameReady, this, [this](const QImage& img){
        QSize labelSize = m_videoLabel->size();
        QImage scaled = img.scaled(labelSize, Qt::KeepAspectRatio, Qt::FastTransformation);
        m_videoLabel->setPixmap(QPixmap::fromImage(scaled));
    }, Qt::QueuedConnection);

    connect(m_video, &VideoPlayer::playError, this, [this](const QString& msg){
        QMessageBox::warning(this, "播放错误", msg);
        m_videoLabel->setText("播放失败，请检查网络连接");
    });

    connect(m_playBtn, &QPushButton::clicked, this, &MainWindow::onPlayClicked);
    connect(m_cameraBtn, &QPushButton::clicked, this, &MainWindow::onCameraClicked);

    // 初始化TCP socket用于发送指令
    send_box = new QTcpSocket(this);
    
    // 连接TCP信号槽
    connect(send_box, &QTcpSocket::connected, this, &MainWindow::onConnected);
    connect(send_box, &QTcpSocket::disconnected, this, &MainWindow::onDisconnected);
    connect(send_box, &QTcpSocket::errorOccurred, this, &MainWindow::onErrorOccurred);

    // 重连计时器
    m_reconnectTimer = new QTimer(this);
    connect(m_reconnectTimer, &QTimer::timeout, this, &MainWindow::reconnect);

    // 指令发送定时器 30ms间隔，
    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &MainWindow::CheckKey_send);
    m_timer->start(30);

    // 
    if (testMode) {
        appendLog(QString("测试模式启动，尝试连接TCP: %1:%2").arg(m_serverHost).arg(m_serverPort));
        send_box->connectToHost(m_serverHost, m_serverPort);
    }
}

MainWindow::~MainWindow()
{
    m_timer->stop();
    m_timer->deleteLater();
    
    m_reconnectTimer->stop();
    m_reconnectTimer->deleteLater();
    
    send_box->close();
    send_box->deleteLater();
    
    m_video->stopPlay();
}

void MainWindow::onPlayClicked()
{
    QString url = m_urlEdit->text().trimmed();
    if (url.isEmpty()) {
        QMessageBox::warning(this, "提示", "请输入网络地址");
        return;
    }
    m_videoLabel->setText("正在连接...");
    m_video->startPlay(url);
    
    // 连接TCP指令通道
    if (send_box->state() != QAbstractSocket::ConnectedState) {
        send_box->connectToHost(m_serverHost, m_serverPort);
    }
}

void MainWindow::onCameraClicked()
{
    if (m_cameraCombo->count() == 0) {
        QMessageBox::warning(this, "提示", "未检测到摄像头设备");
        return;
    }
    QString deviceName = m_cameraCombo->currentText();
    m_videoLabel->setText("正在启动摄像头...");
    m_video->startPlay("camera:" + deviceName);
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    int key = event->key();
    if (key == Qt::Key_Escape) {
        m_video->stopPlay();
    }

    switch (key) {
    case Qt::Key_W:
        keyBox[0] = true;
        break;
    case Qt::Key_S:
        keyBox[1] = true;
        break;
    case Qt::Key_A:
        keyBox[2] = true;
        break;
    case Qt::Key_D:
        keyBox[3] = true;
        break;
    case Qt::Key_Shift:
        keyBox[4] = true;
        break;
    case Qt::Key_C:
        keyBox[5] = true;
        break;
    default:
        break;
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent* event)
{
    int key = event->key();
    switch (key) {
    case Qt::Key_W:
        keyBox[0] = false;
        break;
    case Qt::Key_S:
        keyBox[1] = false;
        break;
    case Qt::Key_A:
        keyBox[2] = false;
        break;
    case Qt::Key_D:
        keyBox[3] = false;
        break;
    case Qt::Key_Shift:
        keyBox[4] = false;
        break;
    case Qt::Key_C:
        keyBox[5] = false;
        break;
    default:
        break;
    }
}

void MainWindow::focusOutEvent(QFocusEvent *event)
{
    QMainWindow::focusOutEvent(event);
    // 窗口失焦时重置所有按键状态，全置0
    memset(keyBox, 0, sizeof(keyBox));
    qDebug() << "窗口失焦，重置按键状态";
}

void MainWindow::appendLog(const QString& msg)
{
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
    m_logEdit->append(QString("[%1] %2").arg(timestamp).arg(msg));
    
    // 最多保留100行 日志
    int maxLines = 100;
    if (m_logEdit->document()->blockCount() > maxLines) {
        QTextCursor cursor(m_logEdit->document());
        cursor.movePosition(QTextCursor::Start);
        cursor.select(QTextCursor::BlockUnderCursor);
        cursor.removeSelectedText();
        cursor.deleteChar();
    }
}

void MainWindow::CheckKey_send()
{
    uint8_t cmd[CMD_LEN] = {0};
    static uint8_t lastCmd[CMD_LEN] = {0};
    bool changed = false;
    
    for(int i = 0; i < CMD_LEN; i++)
    {
        cmd[i] = keyBox[i] ? 0x01 : 0x00;
        if (cmd[i] != lastCmd[i]) changed = true;
    }
    
    if (send_box && send_box->state() == QAbstractSocket::ConnectedState) {
        send_box->write(reinterpret_cast<const char*>(cmd), CMD_LEN);
        send_box->flush();
        
        // 只有指令变化时才打印日志，避免刷屏
        if (changed) {
            QString cmdStr = QString("%1 %2 %3 %4 %5 %6")
                .arg(cmd[0], 2, 16, QChar('0')).toUpper()
                .arg(cmd[1], 2, 16, QChar('0')).toUpper()
                .arg(cmd[2], 2, 16, QChar('0')).toUpper()
                .arg(cmd[3], 2, 16, QChar('0')).toUpper()
                .arg(cmd[4], 2, 16, QChar('0')).toUpper()
                .arg(cmd[5], 2, 16, QChar('0')).toUpper();
            appendLog(QString("发送指令: %1").arg(cmdStr));
            memcpy(lastCmd, cmd, CMD_LEN);
        }
    }
}

void MainWindow::onConnected()
{
    // 禁用Nagle算法，确保小包立即发送，不等待合并
    send_box->setSocketOption(QTcpSocket::LowDelayOption, 1);
    send_box->setSocketOption(QTcpSocket::KeepAliveOption, 1);
    
    appendLog(QString("TCP连接成功: %1:%2 (已启用低延迟模式)").arg(m_serverHost).arg(m_serverPort));
    m_reconnectCount = 0;
    m_reconnectTimer->stop();
}

void MainWindow::onDisconnected()
{
    appendLog("TCP连接断开");
    reconnect();
}

void MainWindow::onErrorOccurred(QAbstractSocket::SocketError error)
{
    appendLog(QString("TCP错误: %1").arg(send_box->errorString()));
    
    if (error != QAbstractSocket::RemoteHostClosedError) {
        reconnect();
    }
}

void MainWindow::onSendCmdClicked()
{
    QString text = m_cmdInput->text().trimmed();
    if (text.length() != CMD_LEN) {
        appendLog("指令长度错误，需要6位");
        return;
    }
    
    uint8_t cmd[CMD_LEN] = {0};
    for (int i = 0; i < CMD_LEN; i++) {
        if (text[i] == '1') {
            cmd[i] = 0x01;
        } else {
            cmd[i] = 0x00;
        }
    }
    
    if (send_box && send_box->state() == QAbstractSocket::ConnectedState) {
        send_box->write(reinterpret_cast<const char*>(cmd), CMD_LEN);
        QString cmdStr = QString("%1 %2 %3 %4 %5 %6")
            .arg(cmd[0], 2, 16, QChar('0')).toUpper()
            .arg(cmd[1], 2, 16, QChar('0')).toUpper()
            .arg(cmd[2], 2, 16, QChar('0')).toUpper()
            .arg(cmd[3], 2, 16, QChar('0')).toUpper()
            .arg(cmd[4], 2, 16, QChar('0')).toUpper()
            .arg(cmd[5], 2, 16, QChar('0')).toUpper();
        appendLog(QString("手动发送指令: %1").arg(cmdStr));
        send_box->flush();
    } else {
        appendLog("TCP未连接，无法发送");
    }
}

void MainWindow::reconnect()
{
    if (m_reconnectCount >= MAX_RECONNECT_ATTEMPTS) {
        appendLog("达到最大重连次数，停止重连");
        m_reconnectTimer->stop();
        return;
    }

    if (send_box->state() == QAbstractSocket::ConnectedState) {
        return;
    }

    m_reconnectCount++;
    appendLog(QString("尝试重连 (%1/%2)").arg(m_reconnectCount).arg(MAX_RECONNECT_ATTEMPTS));
    
    // 使用abort()而不是close()，abort()会立即断开
    send_box->abort();
    send_box->connectToHost(m_serverHost, m_serverPort);
}
