#include "server.h"
#include "gst_task.h"
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <thread>

void error_die(const char* msg)
{
    perror(msg);
    exit(1);
}

int startup(u_short* port)
{
    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in name;
    memset(&name, 0, sizeof(name));
    name.sin_family = AF_INET;
    name.sin_port = htons(*port);
    name.sin_addr.s_addr = htonl(INADDR_ANY);

    int on = 1;
    if (setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0)
    {
        error_die("setsockopt");
    }

    if (bind(listen_fd, (struct sockaddr*)&name, sizeof(name)) < 0)
    {
        error_die("bind");
    }

    if (*port == 0)
    {
        socklen_t namelen = sizeof(name);
        if (getsockname(listen_fd, (struct sockaddr*)&name, &namelen) < 0)
        {
            error_die("getsockname");
        }
        *port = ntohs(name.sin_port);
    }

    if (listen(listen_fd, 5) < 0)
    {
        error_die("listen");
    }

    return listen_fd;
}

/*
 * 帧解析状态机 对应mode.c：
 *   [0xAA][0x01][6字节按键]          手动指令帧（8字节）
 *   [0xAA][0x02][err_lo][err_hi][area_lo][area_hi]  自动/模式切换帧（6字节）
 *   [0xAA][0x03][0,0,0,0]            丢失帧（6字节）
 *
 * 
 */
void recv_cmd(int fd)
{
    enum { WAIT_AA, READ_TYPE, READ_DATA } state = WAIT_AA;
    uint8_t type = 0;
    uint8_t data_len = 0;
    uint8_t fill = 0;
    uint8_t frame[MAX_PACKET_LEN];

    // 将 socket 设为非阻塞
    int fl = fcntl(fd, F_GETFL, 0);
    if (fl >= 0)
        fcntl(fd, F_SETFL, fl | O_NONBLOCK);

    while (go_running.load())
    {
        uint8_t b;
        ssize_t n = read(fd, &b, 1);
        if (n <= 0)
        {
            // EAGAIN/EWOULDBLOCK：无数据可读，短暂睡眠后重新检查退出标志
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            if (n == 0)
            {
                // 客户端正常断开
                go_running.store(false);
                break;
            }
            if (n < 0 && errno == EINTR)
            {
                if (!go_running.load())
                    break;
                continue;
            }
            perror("recv error");
            go_running.store(false);
            break;
        }

        switch (state)
        {
        case WAIT_AA:
            if (b == 0xAA)
            {
                frame[0] = b;
                state = READ_TYPE;
            }
            break;

        case READ_TYPE:
            type = b;
            if (type == 0x01)
                data_len = 6;
            else if (type == 0x02 || type == 0x03)
                data_len = 4;
            else
            {
                state = WAIT_AA;   // 未知类型，丢弃重同步
                break;
            }
            frame[1] = b;
            fill = 0;
            state = READ_DATA;
            break;

        case READ_DATA:
            frame[2 + fill++] = b;
            if (fill == data_len)
            {
                {
                    std::lock_guard<std::mutex> lock(cmd_mtx);
                    std::memcpy(latest_cmd.data(), frame, MAX_PACKET_LEN);
                    latest_cmd_len = 2 + data_len;
                }
                cmd_updated.store(true);
                cmd_received.store(true);

                if (type == 0x01)
                {
                    track_mode.store(false);   
                    auto_mode.store(false);    
                }
                else
                {
                    track_mode.store(true);    
                    auto_mode.store(true);     
                }
                state = WAIT_AA;
            }
            break;
        }
    }
}
