#ifndef _BAT_CONTROL_H_
#define _BAT_CONTROL_H_

#include "BAT_STD.H"

void BAT_InitActuator(void);

void BAT_SetActuatorPWM(uint16_x leftFront,uint16_x rightFront,uint16_x leftRear,uint16_x rightRear);

#endif
