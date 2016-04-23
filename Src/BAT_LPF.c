#include "BAT_LPF.h"


void LPF_Init( S_LPF *filter , float periodTime ,float cutFreq )
{
		filter->lowPassFactor = periodTime / (periodTime + 1 / (2 * M_PI * cutFreq));
		filter->output = filter->lastOutput = 0.0f;
}


float LPF_Update( S_LPF *filter ,float input )
{
		filter->output = filter->lastOutput * (1 - filter->lowPassFactor) + input * filter->lowPassFactor;
		filter->lastOutput = filter->output;
		return filter->output;
}

