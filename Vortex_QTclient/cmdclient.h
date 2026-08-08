#ifndef CMDCLIENT_H
#define CMDCLIENT_H

#include <QObject>
#include <QByteArray>
#include <QTcpSocket>
#include <QTimer>
#include <QAbstractSocket>

/*
 * CmdClient — 指令通道客户端
 *
 * 职责：
 *  1. TCP 连接管理与断线自动重连
 *  2. 协议封装：按键状态 / 模式切换 → 带帧头的串口指令
 *  3. 通过信号向界面层上报日志与状态，不依赖任何 UI 类型
 *
 * 帧格式（与 STM32 mode.c 解析一致）：
 *  手动指令: [0xAA][0x01][k0][k1][k2][k3][k4][k5]   (8字节)
 *            ki = 1 表示按键按下，对应 W/S/A/D/Shift/C
 *  模式切换: 手动 [0xAA][0x01][0,0,0,0,0,0]           (8字节)
 *            自动 [0xAA][0x02][0,0,0,0]               (6字节)
 */
class CmdClient : public QObject
{
    Q_OBJECT
public:
    enum class DriveMode { Auto = 0, Manual = 1 };

    explicit CmdClient(QObject *parent = nullptr);

    void connectToServer(const QString &host, quint16 port);
    void disconnectFromServer();
    bool isConnected() const;

    DriveMode mode() const { return m_mode; }

    /* 手动模式：把按键状态编码为帧头指令并发送（自动模式下发无效） */
    void sendKeyState(const bool keys[6]);

    /* 模式切换：发送对应帧头指令，让 STM32 切换手动/自动 */
    void sendModeSwitch(DriveMode mode);

    /* 协议编码：返回带帧头的完整指令，供日志/调试复用 */
    static QByteArray encodeKeyState(const bool keys[6]);

signals:
    void logMessage(const QString &msg);
    void modeChanged(CmdClient::DriveMode mode);

private slots:
    void onConnected();
    void onDisconnected();
    void onErrorOccurred(QAbstractSocket::SocketError error);
    void tryReconnect();

private:
    void sendPacket(const QByteArray &packet);

    QTcpSocket *m_socket = nullptr;
    QTimer *m_reconnectTimer = nullptr;
    int m_reconnectCount = 0;
    bool m_autoReconnect = false;
    QString m_host;
    quint16 m_port = 0;
    DriveMode m_mode = DriveMode::Auto;   // 与 STM32 上电默认 MODE_AUTO 保持一致
};

#endif // CMDCLIENT_H
