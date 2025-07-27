#ifndef __BSP_K210_H_
#define __BSP_K210_H_

#include "AllHeader.h"
#define BUFFER_SIZE 40


void K210_Send_Msg(const char *data_str);
char* K210_Deal_Recv(uint8_t recv_msg);

#endif
