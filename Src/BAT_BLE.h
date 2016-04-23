#ifndef _BAT_BLE_H_
#define _BAT_BLE_H_

#include "stdint.h"

void BLE_Init(void);
void BLE_SendData(uint8_t *Data,uint16_t Len);

#endif
