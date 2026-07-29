#include "server.h"



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
    if(setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0)
    {
        error_die("setsockopt");  
    }
    
    if (bind(listen_fd, (struct sockaddr*)&name, sizeof(name)) < 0) {
        error_die("bind");
    }
    
    if (*port == 0) {
        socklen_t namelen = sizeof(name);
        if (getsockname(listen_fd, (struct sockaddr*)&name, &namelen) < 0)
        {
            error_die("getsockname");  
        }
        *port = ntohs(name.sin_port);  
    }
    
    if (listen(listen_fd, 5) < 0) {
        error_die("listen");
    }
    
    return listen_fd;
}



ssize_t recv_data(int fd, uint8_t* buf, int len)
{
    int received = 0;
    while(received < len)
    {
        int n = read(fd, buf + received, len - received);
        if(n <= 0) 
        {
            if(n < 0) perror("recv error");
            return -1;
        }
        received += n;
    }
    return 0;
}


void recv_cmd(int fd)
{
    uint8_t buf[DATA_LEN];
    while(go_running)
    {
        if(recv_data(fd, buf, DATA_LEN) < 0)
        {
            go_running.store(false);
            break;
        }
        // 接存储最新命令，丢弃旧命令
        std::lock_guard<std::mutex> lock(cmd_mtx);
        std::memcpy(latest_cmd.data(), buf, DATA_LEN);
        cmd_updated.store(true);
        cmd_received.store(true);  // 标记已收到有效命令
    }
}


