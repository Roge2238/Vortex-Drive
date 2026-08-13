#ifndef COMMON_H
#define COMMON_H

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <mutex>
#include <atomic>
#include <queue>
#include <array>
#include <chrono>

//[0xAA][type][data]，整帧最大长度
#define DATA_LEN 8
#define MAX_PACKET_LEN 8
using CmdPacket = std::array<uint8_t, DATA_LEN>;

extern std::atomic<bool> go_running;

// 串口指示：false=手动  true=自动
extern std::atomic<bool> auto_mode;

//  覆盖式更新，避免队列积压
extern std::mutex cmd_mtx;
extern CmdPacket latest_cmd;
extern uint8_t latest_cmd_len;   // 实际帧长（CMD=8, AUTO/LOST=6）
extern std::atomic<bool> cmd_updated;  // 有新命令
extern std::atomic<bool> cmd_received;  // 已收到过命令
extern std::atomic<bool> client_alive;  // 客户端在线标志（断线重连机制）

#endif
