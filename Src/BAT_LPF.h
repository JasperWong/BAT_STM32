#ifndef _BAT_LPF_H_
#define _BAT_LPF_H_

#include "BAT_STD.H"

typedef struct _LPF
{
	  float lowPassFactor;
		float lastOutput;
		float output;
}S_LPF;  



#define M_PI (3.141592653f)

void LPF_Init( S_LPF *filter , float periodTime ,float cutFreq );
float LPF_Update( S_LPF *filter ,float input );

#endif


