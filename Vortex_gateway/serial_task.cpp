#include "serial_task.h"


bool serial_config(int fd, speed_t baud_rate)
{
    struct termios opt;
    if (tcgetattr(fd, &opt) != 0)
    {
        perror("tcgetattr fail");
        return false;
    }

    // 波特率
    cfsetispeed(&opt, baud_rate);
    cfsetospeed(&opt, baud_rate);

    //  物理层 8N1
    opt.c_cflag &= ~PARENB;        // 无校验
    opt.c_cflag &= ~CSTOPB;        // 1停止位
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;            // 8数据位
    opt.c_cflag |= CREAD | CLOCAL; // 启用接收，忽略调制解调器信号


    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    opt.c_oflag &= ~OPOST;
    opt.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON);

    if (tcsetattr(fd, TCSANOW, &opt) != 0)
    {
        perror("tcsetattr fail");
        return false;
    }
    return true;
}



ssize_t serial_send(int fd, const CmdPacket* cmd, size_t len)
{
    if(fd < 0 || cmd == nullptr || len == 0)
    {
        return -1;
    }
    size_t sent = 0;
    const uint8_t* data = reinterpret_cast<const uint8_t*>(cmd);
    while(sent < len)
    {
        ssize_t w = write(fd, data + sent, len - sent);
        if(w <= 0)
        {
            perror("write error");
            return -1;
        }
        sent += w;
    }
    return sent;
}






void serial_send_thread(int fd)
{
    using namespace std::chrono;
    const milliseconds interval{20};
    CmdPacket last_sent{};  // 保存最后发送的命令

    while(go_running.load(std::memory_order_acquire))
    {
        auto start = steady_clock::now();

        // 如果还没收到任何命令，跳过发送
        if (!cmd_received.load())
        {
            auto end = steady_clock::now();
            auto cost = duration_cast<microseconds>(end - start);
            auto sleep_us = interval.count() * 1000 - cost.count();
            if(sleep_us > 0) usleep(sleep_us);
            continue;
        }

        CmdPacket temp;
        
        {
            std::lock_guard<std::mutex> lock(cmd_mtx);
            if (cmd_updated.load())
            {
                temp = latest_cmd;
                cmd_updated.store(false);
                last_sent = temp;  
            }
            else
            {
                
                temp = last_sent;
            }
        }

        // 发送到串口
        serial_send(fd, &temp, sizeof(temp));

        auto end = steady_clock::now();
        auto cost = duration_cast<microseconds>(end - start);
        auto sleep_us = interval.count() * 1000 - cost.count();

        if(sleep_us > 0)
        {
            usleep(sleep_us);
        }
    }
}