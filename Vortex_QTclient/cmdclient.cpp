#include "cmdclient.h"

#include <QNetworkProxy>

namespace {

/* 协议常量，与 STM32 mode.h 一致 */
constexpr quint8 kMagicHeader = 0xAA;
constexpr quint8 kCmdType     = 0x01;   // 手动控制帧
constexpr quint8 kAutoType    = 0x02;   // 自动(视觉)帧

constexpr int kKeyCount              = 6;     // 按键个数 W/S/A/D/Shift/C
constexpr int kKeyPacketLength       = 8;     // 帧头+类型+6按键
constexpr int kModePacketLength      = 6;     // 帧头+类型+4字节数据

constexpr int kReconnectIntervalMs   = 3000;
constexpr int kMaxReconnectAttempts  = 5;

} // namespace

CmdClient::CmdClient(QObject *parent)
    : QObject(parent)
{
    m_socket = new QTcpSocket(this);
    // 局域网直连：绕过系统代理，否则连内网IP会报 "proxy type is invalid"
    m_socket->setProxy(QNetworkProxy::NoProxy);
    
    m_socket->setSocketOption(QTcpSocket::LowDelayOption, 1);
    m_socket->setSocketOption(QTcpSocket::KeepAliveOption, 1);

    m_reconnectTimer = new QTimer(this);
    m_reconnectTimer->setInterval(kReconnectIntervalMs);
    connect(m_reconnectTimer, &QTimer::timeout, this, &CmdClient::tryReconnect);

    connect(m_socket, &QTcpSocket::connected,    this, &CmdClient::onConnected);
    connect(m_socket, &QTcpSocket::disconnected, this, &CmdClient::onDisconnected);
    connect(m_socket, &QTcpSocket::errorOccurred, this, &CmdClient::onErrorOccurred);
}

void CmdClient::connectToServer(const QString &host, quint16 port)
{
    if (isConnected())
        return;

    m_host = host;
    m_port = port;
    m_autoReconnect = true;
    emit logMessage(QString("尝试连接TCP: %1:%2").arg(host).arg(port));
    m_socket->connectToHost(host, port);
}

void CmdClient::disconnectFromServer()
{
    m_autoReconnect = false;
    m_reconnectTimer->stop();
    m_socket->abort();
}

bool CmdClient::isConnected() const
{
    return m_socket->state() == QAbstractSocket::ConnectedState;
}

QByteArray CmdClient::encodeKeyState(const bool keys[kKeyCount])
{
    QByteArray packet;
    packet.reserve(kKeyPacketLength);
    packet.append(static_cast<char>(kMagicHeader));
    packet.append(static_cast<char>(kCmdType));
    for (int i = 0; i < kKeyCount; ++i)
        packet.append(keys[i] ? static_cast<char>(0x01) : static_cast<char>(0x00));
    return packet;
}

void CmdClient::sendKeyState(const bool keys[kKeyCount])
{
    // 自动模式下不发送按键指令，避免与视觉端抢控制权
    if (!isConnected() || m_mode != DriveMode::Manual)
        return;
    sendPacket(encodeKeyState(keys));
}

void CmdClient::sendModeSwitch(DriveMode mode)
{
    if (mode == m_mode)
        return;
    m_mode = mode;

    QByteArray packet;
    packet.append(static_cast<char>(kMagicHeader));
    if (mode == DriveMode::Manual) {
        // 手动帧：数据位全 0，通知 STM32 进入手动模式并停机待命
        packet.append(static_cast<char>(kCmdType));
        packet.append(kKeyCount, '\0');
    } else {
        // 自动帧：error/area 全 0，通知 STM32 进入自动模式（由视觉端接管）
        packet.append(static_cast<char>(kAutoType));
        packet.append(kModePacketLength - 2, '\0');
    }
    sendPacket(packet);

    emit modeChanged(mode);
    emit logMessage(mode == DriveMode::Manual ? "已切换为手动模式" : "已切换为自动模式");
}

void CmdClient::sendPacket(const QByteArray &packet)
{
    if (!isConnected())
        return;
    m_socket->write(packet);
    m_socket->flush();
}

void CmdClient::onConnected()
{
    m_reconnectCount = 0;
    m_reconnectTimer->stop();
    emit logMessage(QString("TCP连接成功: %1:%2 (已启用低延迟模式)").arg(m_host).arg(m_port));
}

void CmdClient::onDisconnected()
{
    emit logMessage("TCP连接断开");
    if (m_autoReconnect)
        m_reconnectTimer->start();
}

void CmdClient::onErrorOccurred(QAbstractSocket::SocketError error)
{
    emit logMessage(QString("TCP错误: %1").arg(m_socket->errorString()));

    // RemoteHostClosedError 会伴随 disconnected 信号，由 onDisconnected 统一处理
    if (error != QAbstractSocket::RemoteHostClosedError)
        m_reconnectTimer->start();
}

void CmdClient::tryReconnect()
{
    if (m_reconnectCount >= kMaxReconnectAttempts) {
        emit logMessage("达到最大重连次数，停止重连");
        m_reconnectTimer->stop();
        return;
    }
    if (isConnected()) {
        m_reconnectTimer->stop();
        return;
    }

    ++m_reconnectCount;
    emit logMessage(QString("尝试重连 (%1/%2)").arg(m_reconnectCount).arg(kMaxReconnectAttempts));
    // abort() 立即断开旧连接，避免二次握手阻塞
    m_socket->abort();
    m_socket->connectToHost(m_host, m_port);
}
