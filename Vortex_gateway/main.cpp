#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>
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
#include "gst_loop.h"

using namespace std;



// 最新命令存储
mutex cmd_mtx;
CmdPacket latest_cmd{};
atomic<bool> cmd_updated(false);
atomic<bool> cmd_received(false);  // 启动时为false，收到第一个命令后变true

atomic<bool> go_running(false);

// 信号处理函数，优雅退出
void signal_handler(int sig)
{
    go_running.store(false);
    std::cout << "\n收到退出信号，正在停止线程...\n";
}



int main(int argc, char* argv[])
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // 初始化串口
    int urt_fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY);
    if (urt_fd < 0)
    {
        perror("打开串口失败");
        return -1;
    }
    
    if (!serial_config(urt_fd, B4800))
    {
        perror("配置串口失败");
        close(urt_fd);
        return -1;
    }
    std::cout << "串口初始化成功 (波特率:4800)\n";

    // ===== 测试模式 =====
    if(argc == 2 && strcmp(argv[1], "--test") == 0)
    {
        std::cout << "进入串口测试模式，输入6字节十六进制命令 (如: 010203040506):\n";
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

    // 初始化网络服务 
    u_short port = 8651;
    int server_fd = startup(&port);    
    std::cout << "服务器启动，监听端口: " << port << "\n";
    std::cout << "等待客户端连接...\n";

    sockaddr_in client_addr{};
    socklen_t cli_len = sizeof(client_addr);

    int client_fd = accept(server_fd, (sockaddr*)&client_addr, &cli_len);
    if (client_fd < 0)
    {
        error_die("accept失败");
        close(urt_fd);
        close(server_fd);
        return -1;
    }
    std::cout << "客户端连接成功: " << inet_ntoa(client_addr.sin_addr) << "\n";


    go_running.store(true);

    
    



    thread recv_thread(recv_cmd, client_fd);
    std::cout << "网络命令接收线程启动\n";

    thread gst_thread(gst_udp_stream);
    std::cout << "GStreamer推流线程启动\n";

    thread serial_thread(serial_send_thread, urt_fd);
    std::cout << "串口发送线程启动\n";


    std::cout << "所有线程启动完成，按 Ctrl+C 退出\n";
    
    recv_thread.join();
    std::cout << "网络命令接收线程已退出\n";
    
    gst_thread.join();
    std::cout << "GStreamer推流线程已退出\n";
    
    serial_thread.join();
    std::cout << "串口发送线程已退出\n";


    close(client_fd);
    close(server_fd);
    close(urt_fd);
    std::cout << "资源已释放，程序退出\n";

    return 0;
}