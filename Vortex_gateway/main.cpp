#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <cerrno>
#include <thread>
#include <stdio.h>
#include <iostream>
#include <mutex>
#include <atomic>
#include <queue>
#include <array>
#include <signal.h>

#include "serial_task.h"
#include "server.h"
#include "gst_task.h"
#include "servo_cv.h"

using namespace std;

// 最新命令存储
mutex cmd_mtx;
CmdPacket latest_cmd{};
uint8_t latest_cmd_len = 0;     // 实际帧长
atomic<bool> cmd_updated(false);
atomic<bool> cmd_received(false);  // 启动时为false，收到第一个命令后变true

atomic<bool> auto_mode(false);   // false=手动 true=自动
atomic<bool> go_running(false);
atomic<bool> client_alive(false); // 客户端在线标志（断线重连机制）

// 信号处理函数
void signal_handler(int sig)
{
    go_running.store(false);
    std::cout << "\n收到退出信号，正在停止线程...\n";
}



int main(int argc, char* argv[])
{
    // 收到信号后返回 EINTR，各线程才能醒过来检查 go_running 并退出
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    // 初始化串口
    int urt_fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY);
    if (urt_fd < 0)
    {
        perror("打开串口失败");
        return -1;
    }
    
   
    if (!serial_config(urt_fd, B115200))
    {
        perror("配置串口失败");
        close(urt_fd);
        return -1;
    }
    std::cout << "串口初始化成功 (波特率:115200)\n";

    // ===== 测试模式 ===== 已弃用  完全测试不了
    if(argc == 2 && strcmp(argv[1], "--test") == 0)
    {
        std::cout << "进入串口测试模式，输入8字节十六进制整帧 (如: AA01010000000000):\n";
        char input[64];
        while(true)
        {
            std::cout << "> ";
            if(!std::cin.getline(input, sizeof(input))) break;
            
            CmdPacket test_cmd = {};
            int byte_count = 0;
            const char* p = input;
            while(*p && byte_count < DATA_LEN)
            {
                char hex[3] = {p[0], p[1], 0};
                test_cmd[byte_count++] = (uint8_t)strtol(hex, nullptr, 16);
                p += 2;
                if(*p == ' ') p++;
            }
            // 
            if(byte_count == DATA_LEN)
            {
                std::cout << "发送: ";
                for(int i = 0; i < DATA_LEN; i++)
                    printf("%02X ", test_cmd[i]);
                std::cout << "\n";
                serial_send(urt_fd, &test_cmd, sizeof(test_cmd));
            }
        }
        close(urt_fd);
        return 0;
    }

    // 初始化网络服务  直接照搬Tinytalk 嘿嘿
    u_short port = 8651;
    int server_fd = startup(&port);
    std::cout << "服务器启动，监听端口: " << port << "\n";

    go_running.store(true);

    /* 将推流/串口线程设为常驻：不依赖客户端连接。
     * 客户端断开后继续推原图，新客户端重连后点播放立即有画面 */
    thread gst_thread(gst_task);
    std::cout << "GStreamer推流线程启动\n";

    thread servo_thread(servo_cv_thread);
    std::cout << "舵机自动跟踪线程启动（独立摄像头，不推流）\n";

    thread serial_thread(serial_send_thread, urt_fd);
    std::cout << "串口发送线程启动\n";

    sockaddr_in client_addr{};
    socklen_t cli_len = sizeof(client_addr);
    int client_fd = -1;

     //退出：sigaction 无 SA_RESTART → accept 被信号打断返回 EINTR → break */
    while (go_running.load())
    {
        std::cout << "等待客户端连接...\n";
        client_fd = accept(server_fd, (sockaddr*)&client_addr, &cli_len);
        if (client_fd < 0)
        {
            if (errno == EINTR)
                continue;
            perror("accept失败");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        client_alive.store(true);
        std::cout << "客户端连接成功: " << inet_ntoa(client_addr.sin_addr) << "\n";

        thread recv_thread(recv_cmd, client_fd);
        std::cout << "网络命令接收线程启动\n";
        recv_thread.join();          // 阻塞直到该客户端断开
        std::cout << "网络命令接收线程已退出\n";
        close(client_fd);

        /* 断连清理：停止跟踪、清残留按键，重连后不会发送旧指令。
         * 安全停车由串口线程完成：自动模式补发LOST立即急停，
         * 手动模式停发指令由 STM32 200ms 超时滑行停止 */
        client_alive.store(false);
        track_mode.store(false);
        auto_mode.store(false);
        cmd_received.store(false);
        cmd_updated.store(false);
        std::cout << "客户端断开，等待重连...\n";
    }

    gst_thread.join();
    std::cout << "GStreamer推流线程已退出\n";

    servo_thread.join();
    std::cout << "舵机自动跟踪线程已退出\n";

    serial_thread.join();
    std::cout << "串口发送线程已退出\n";

    close(server_fd);
    close(urt_fd);
    std::cout << "资源已释放，程序退出\n";

    return 0;
}
