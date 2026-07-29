#include "common.h"

void error_die(const char* msg);

int startup(u_short* port);

ssize_t recv_data(int fd, uint8_t* buf, int len);

void recv_cmd(int fd);