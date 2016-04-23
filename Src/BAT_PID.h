#ifndef _BAT_PID_H_
#define _BAT_PID_H_

#include "BAT_STD.H"

typedef struct _PID
{
    //PID Parameter
    float kP;
    float kI;
    float kD;
    
    //Upper Integral Limit & Lower Integral Limit
    float maxIntegral;
    float minIntegral;
    
    //Processing valuable
    float proportional;
    float integral;
    float derivative;
    float lastError;
    
    //Output
    float output;
		
		bool_x isEnableIntegral;
}S_PID;


void PID_Init(S_PID *pid , float kP ,float kI ,float kD ,float minIntegral ,float maxIntegral );
float PID_PositionUpdate(S_PID *pid ,float error,float dt);
void PID_ResetIntegral(S_PID *pid);
void PID_DisableIntegral(S_PID *pid);
void PID_EnableIntegral(S_PID *pid);


#endif 
