#include "BAT_PID.h"



void PID_Init(S_PID *pid , float kP ,float kI ,float kD ,float minIntegral ,float maxIntegral )
{
    pid->kP = kP;
    pid->kI = kI;
    pid->kD = kD;
    pid->minIntegral = minIntegral;
    pid->maxIntegral = maxIntegral;
		pid->isEnableIntegral = true_x;
}


//
float PID_PositionUpdate(S_PID *pid ,float error,float dt)
{
    //P
    pid->proportional = pid->kP * error ; 


    //I
		if( pid->isEnableIntegral )
		{
				//
				pid->integral += pid->kI * error * dt ;


				//Limit Integral
				if( pid->integral > pid->maxIntegral )
				{
						pid->integral = pid->maxIntegral;
				}
				else if(  pid->integral < pid->minIntegral )
				{
						pid->integral = pid->minIntegral;
				}
		}

    //D
    pid->derivative = pid->kD * ( error - pid->lastError ) / dt;
    pid->lastError = error;


    //Calculate
    pid->output = pid->proportional + pid->integral + pid->derivative;


    return pid->output;
}

void PID_ResetIntegral(S_PID *pid)
{
    pid->output = pid->output - pid->integral;
    pid->integral = 0;
}


void PID_DisableIntegral(S_PID *pid)
{
		pid->isEnableIntegral = false_x;
}


void PID_EnableIntegral(S_PID *pid)
{
		pid->isEnableIntegral = true_x;
}


