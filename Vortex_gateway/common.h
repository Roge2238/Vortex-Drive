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

#define DATA_LEN 6
using CmdPacket = std::array<uint8_t, DATA_LEN>;

extern std::atomic<bool> go_running;

//  覆盖式更新，避免队列积压
extern std::mutex cmd_mtx;
extern CmdPacket latest_cmd;
extern std::atomic<bool> cmd_updated;  // 有新命令
extern std::atomic<bool> cmd_received;  // 已收到过命令

#endif