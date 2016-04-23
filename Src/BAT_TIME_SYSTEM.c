#include "BAT_TIME_SYSTEM.H"
#include "stm32f10x.h"

void TIME_SYSTEM_Initialize(void)
{
		SysTick_Config(SystemCoreClock/1000);
}
		

uint16_t cnt_1ms,cnt_2ms,cnt_5ms,cnt_10ms,cnt_20ms;
void TIME_SYSTEM_OnMilliSecondTick(void)
{
		cnt_1ms++;
		cnt_2ms++;
		cnt_5ms++;
		cnt_10ms++;
		cnt_20ms++;
}






