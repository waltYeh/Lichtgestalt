#ifndef DATA_LINK_H
#define DATA_LINK_H
#include "../Modules/stabilizer_types.h"
unsigned short crc16(void* data, unsigned short cnt);
void send_buffer(void *data, unsigned short len);
void data_link_init(void);
void DataLinkReceive_IDLE(void);
void commandBlockingAcquire(command_t *cmd);
void motionAccAcquire(vec3f_t *motion_acc);
void data_send_start(void);
#endif
