#ifndef __APP_BLUETOOTH_H_
#define __APP_BLUETOOTH_H_


#include "AllHeader.h"

void Init_PID(void);
void ResetPID(void);
void ProtocolGetPID(void);

void deal_bluetooth(uint8_t rxbuf);
void ProtocolCpyData(void);
void Protocol(void);

int StringFind(const char *pSrc, const char *pDst) ;

void CalcUpData(void);
void SendAutoUp(void);//�Զ��ϱ� ����time6���� Automatically report and put it in time6
#endif

