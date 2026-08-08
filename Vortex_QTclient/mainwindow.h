#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>
#include <QTimer>
#include <QKeyEvent>
#include <QTextEdit>

#include "cmdclient.h"

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
    void onModeClicked();
    void onCmdTick();
    void onSendCmdClicked();   // 测试模式：手动输入指令

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
    void focusOutEvent(QFocusEvent *event) override;

private:
    // 按键在 keyBox 中的下标（对应协议位顺序 W/S/A/D/Shift/C）
    enum KeyIndex { KeyW = 0, KeyS, KeyA, KeyD, KeyShift, KeyC, KeyCount };
    void setKeyState(int qtKey, bool pressed);
    void appendLog(const QString &msg);

    void initUi();
    void initConnections();

    // ---- UI 控件 ----
    QLabel *m_videoLabel = nullptr;
    VideoPlayer *m_video = nullptr;
    QLineEdit *m_urlEdit = nullptr;
    QPushButton *m_playBtn = nullptr;
    QPushButton *m_cameraBtn = nullptr;
    QComboBox *m_cameraCombo = nullptr;
    QPushButton *m_modeBtn = nullptr;
    QTextEdit *m_logEdit = nullptr;

    // 测试模式控件
    QLineEdit *m_cmdInput = nullptr;
    QPushButton *m_sendCmdBtn = nullptr;

    // ---- 状态 ----
    bool m_keyBox[KeyCount] = {};
    bool m_testMode = false;
    QTimer *m_cmdTimer = nullptr;

    // ---- 指令通道 ----
    CmdClient *m_cmdClient = nullptr;
};

#endif // MAINWINDOW_H
