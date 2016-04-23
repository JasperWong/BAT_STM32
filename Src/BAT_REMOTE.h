#ifndef _BAT_REMOTE_H_
#define _BAT_REMOTE_H_

#include "BAT_STD.H"

void BAT_InitCommunicator(void);

void BAT_CommunicationEventHandler(void);

void BAT_OnRecieveDataStream(uint8_x *data,int32_x length);

void BAT_Send_Heartbeat(void);

#endif
