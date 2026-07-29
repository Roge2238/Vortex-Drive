#include "common.h"

bool serial_config(int fd, speed_t baud_rate);

ssize_t serial_send(int fd, const CmdPacket* cmd, size_t len);

void serial_send_thread(int fd);