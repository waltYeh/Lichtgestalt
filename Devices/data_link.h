#ifndef DATA_LINK_H
#define DATA_LINK_H
unsigned short crc16(void* data, unsigned short cnt);
void send_buffer(void *data, unsigned short len);
void data_link_init(void);
void DataLinkReceive_IDLE(void);
#endif
