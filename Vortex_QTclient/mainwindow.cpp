#include "mainwindow.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QTextCursor>
#include <QDateTime>
#include <QPixmap>
#include <QImage>
#include <QFocusEvent>
#include <QDebug>
#include <cstring>

#include "videoplayer.h"

namespace {

/* 服务器地址配置 */
const QString kServerHost = "192.168.43.96";
const quint16 kServerPort = 8651;

} // namespace

MainWindow::MainWindow(bool testMode, QWidget *parent)
    : QMainWindow(parent)
    , m_testMode(testMode)
{
    setWindowTitle("Vortex Player");
    resize(1280, 900);

    m_cmdClient = new CmdClient(this);
    initUi();
    initConnections();
}

MainWindow::~MainWindow()
{
    m_cmdTimer->stop();
    m_video->stopPlay();
    m_cmdClient->disconnectFromServer();
}

/* ========== UI 搭建 ========== */
void MainWindow::initUi()
{
    QWidget *central = new QWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(central);
    setCentralWidget(central);

    // 控制栏：地址 / 播放 / 摄像头 / 模式 / 设备列表
    QHBoxLayout *ctrlLayout = new QHBoxLayout();

    m_urlEdit = new QLineEdit("udp://192.168.43.20:8650", this);
    ctrlLayout->addWidget(m_urlEdit);

    m_playBtn = new QPushButton("播放", this);
    ctrlLayout->addWidget(m_playBtn);

    m_cameraBtn = new QPushButton("摄像头", this);
    ctrlLayout->addWidget(m_cameraBtn);

    m_modeBtn = new QPushButton("模式转换", this);
    ctrlLayout->addWidget(m_modeBtn);

    m_cameraCombo = new QComboBox(this);
    ctrlLayout->addWidget(m_cameraCombo);

    layout->addLayout(ctrlLayout);

    // 测试模式：手动指令输入
    if (m_testMode) {
        QHBoxLayout *cmdLayout = new QHBoxLayout();
        m_cmdInput = new QLineEdit(this);
        m_cmdInput->setPlaceholderText("手动输入指令（6位二进制，如100000）");
        m_sendCmdBtn = new QPushButton("发送指令", this);
        cmdLayout->addWidget(m_cmdInput);
        cmdLayout->addWidget(m_sendCmdBtn);
        layout->addLayout(cmdLayout);
    }

    // 视频显示区
    m_videoLabel = new QLabel("视频加载中...", this);
    m_videoLabel->setAlignment(Qt::AlignCenter);
    m_videoLabel->setStyleSheet("background-color: #000000; color: #ffffff;");
    m_videoLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    layout->addWidget(m_videoLabel, 1);

    // 日志区
    m_logEdit = new QTextEdit(this);
    m_logEdit->setReadOnly(true);
    m_logEdit->setFixedHeight(120);
    m_logEdit->setStyleSheet("background-color: #1e1e1e; color: #00ff00; font-family: Consolas; font-size: 12px;");
    layout->addWidget(m_logEdit);

    // 视频播放器（解码/设备枚举的独立模块）
    m_video = new VideoPlayer(this);
    const QStringList devices = m_video->getCameraDevices();
    for (const QString &name : devices)
        m_cameraCombo->addItem(name);
}

/* ========== 信号槽连接 ========== */
void MainWindow::initConnections()
{
    // 视频
    connect(m_video, &VideoPlayer::frameReady, this, [this](const QImage &img) {
        const QSize labelSize = m_videoLabel->size();
        const QImage scaled = img.scaled(labelSize, Qt::KeepAspectRatio, Qt::FastTransformation);
        m_videoLabel->setPixmap(QPixmap::fromImage(scaled));
    }, Qt::QueuedConnection);

    connect(m_video, &VideoPlayer::playError, this, [this](const QString &msg) {
        QMessageBox::warning(this, "播放错误", msg);
        m_videoLabel->setText("播放失败，请检查网络连接");
    });
    connect(m_video, &VideoPlayer::diagnosticLog, this, &MainWindow::appendLog);

    connect(m_playBtn,   &QPushButton::clicked, this, &MainWindow::onPlayClicked);
    connect(m_cameraBtn, &QPushButton::clicked, this, &MainWindow::onCameraClicked);
    connect(m_modeBtn,   &QPushButton::clicked, this, &MainWindow::onModeClicked);

    if (m_testMode)
        connect(m_sendCmdBtn, &QPushButton::clicked, this, &MainWindow::onSendCmdClicked);

    // 指令通道
    connect(m_cmdClient, &CmdClient::logMessage, this, &MainWindow::appendLog);
    connect(m_cmdClient, &CmdClient::modeChanged, this, [this](CmdClient::DriveMode mode) {
        appendLog(mode == CmdClient::DriveMode::Manual ? "当前模式: 手动" : "当前模式: 自动");
    });

    // 30ms 周期轮询按键状态并发送
    m_cmdTimer = new QTimer(this);
    connect(m_cmdTimer, &QTimer::timeout, this, &MainWindow::onCmdTick);
    m_cmdTimer->start(30);

    // 测试模式：启动即连接
    if (m_testMode)
        m_cmdClient->connectToServer(kServerHost, kServerPort);
}

/* ========== 槽函数 ========== */
void MainWindow::onPlayClicked()
{
    const QString url = m_urlEdit->text().trimmed();
    if (url.isEmpty()) {
        QMessageBox::warning(this, "提示", "请输入网络地址");
        return;
    }
    m_videoLabel->setText("正在连接...");
    m_video->startPlay(url);

    // 播放视频的同时连接指令通道
    if (!m_cmdClient->isConnected())
        m_cmdClient->connectToServer(kServerHost, kServerPort);
}

void MainWindow::onCameraClicked()
{
    if (m_cameraCombo->count() == 0) {
        QMessageBox::warning(this, "提示", "未检测到摄像头设备");
        return;
    }
    const QString deviceName = m_cameraCombo->currentText();
    m_videoLabel->setText("正在启动摄像头...");
    m_video->startPlay("camera:" + deviceName);
}

void MainWindow::onModeClicked()
{
    const CmdClient::DriveMode next =
        (m_cmdClient->mode() == CmdClient::DriveMode::Manual)
            ? CmdClient::DriveMode::Auto
            : CmdClient::DriveMode::Manual;
    m_cmdClient->sendModeSwitch(next);
}

void MainWindow::onCmdTick()
{
    // 自动模式下按键由视觉端接管，不发送指令
    if (m_cmdClient->mode() != CmdClient::DriveMode::Manual)
        return;

    static bool lastKey[KeyCount] = {};
    bool changed = false;
    for (int i = 0; i < KeyCount; ++i) {
        if (m_keyBox[i] != lastKey[i]) {
            lastKey[i] = m_keyBox[i];
            changed = true;
        }
    }

    m_cmdClient->sendKeyState(m_keyBox);

    // 指令变化时打印日志，避免刷屏
    if (changed) {
        const QByteArray packet = CmdClient::encodeKeyState(m_keyBox);
        QString hex;
        for (int i = 0; i < packet.size(); ++i)
            hex += QString("%1 ").arg(static_cast<quint8>(packet.at(i)), 2, 16, QChar('0')).toUpper();
        appendLog(QString("发送指令: %1").arg(hex.trimmed()));
    }
}

void MainWindow::onSendCmdClicked()
{
    const QString text = m_cmdInput->text().trimmed();
    if (text.length() != KeyCount) {
        appendLog("指令长度错误，需要6位");
        return;
    }

    bool keys[KeyCount] = {};
    for (int i = 0; i < KeyCount; ++i)
        keys[i] = (text.at(i) == '1');

    if (m_cmdClient->mode() != CmdClient::DriveMode::Manual) {
        appendLog("当前为自动模式，请先点击 模式转换 切换到手动");
        return;
    }
    if (!m_cmdClient->isConnected()) {
        appendLog("TCP未连接，无法发送");
        return;
    }

    m_cmdClient->sendKeyState(keys);

    const QByteArray packet = CmdClient::encodeKeyState(keys);
    QString hex;
    for (int i = 0; i < packet.size(); ++i)
        hex += QString("%1 ").arg(static_cast<quint8>(packet.at(i)), 2, 16, QChar('0')).toUpper();
    appendLog(QString("手动发送指令: %1").arg(hex.trimmed()));
}

/* ========== 键盘输入 ========== */
void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Escape)
        m_video->stopPlay();
    setKeyState(event->key(), true);
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    setKeyState(event->key(), false);
}

void MainWindow::focusOutEvent(QFocusEvent *event)
{
    QMainWindow::focusOutEvent(event);
    // 窗口失焦时重置所有按键状态，防止"幽灵按键"持续发送
    memset(m_keyBox, 0, sizeof(m_keyBox));
    qDebug() << "窗口失焦，重置按键状态";
}

void MainWindow::setKeyState(int qtKey, bool pressed)
{
    switch (qtKey) {
    case Qt::Key_W:     m_keyBox[KeyW]     = pressed; break;
    case Qt::Key_S:     m_keyBox[KeyS]     = pressed; break;
    case Qt::Key_A:     m_keyBox[KeyA]     = pressed; break;
    case Qt::Key_D:     m_keyBox[KeyD]     = pressed; break;
    case Qt::Key_Shift: m_keyBox[KeyShift] = pressed; break;
    case Qt::Key_C:     m_keyBox[KeyC]     = pressed; break;
    default: break;
    }
}

/* ========== 日志 ========== */
void MainWindow::appendLog(const QString &msg)
{
    const QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
    m_logEdit->append(QString("[%1] %2").arg(timestamp).arg(msg));

    // 最多保留 100 行日志
    const int maxLines = 100;
    if (m_logEdit->document()->blockCount() > maxLines) {
        QTextCursor cursor(m_logEdit->document());
        cursor.movePosition(QTextCursor::Start);
        cursor.select(QTextCursor::BlockUnderCursor);
        cursor.removeSelectedText();
        cursor.deleteChar();
    }
}
