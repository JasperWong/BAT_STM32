#include "stm32f10x.h"
#include "BAT_TIME_SYSTEM.H"
#include "BAT_I2C.h"
#include "BAT_MPU6050.h"
#include "BAT_LPF.h"
#include "BAT_PID.H"
#include "BAT_NRF.H"
#include "BAT_REMOTE.H"
#include "BAT_CONTROL.H"
#include "BAT_BLE.H"
#define MPU6050G_s2000dps           ((float)0.0609756f)  // 0.0700000 dps/LSB


typedef struct _BAT
{
	int32_x rawData[8];
	
	
	S_RAW_ACCEL rawAccel;
	S_RAW_GYRO rawGyro;
	
	
	S_GRAVITY_UNIT_ACCEL unitAccel;
	S_DPS_UNIT_GYRO unitGyro;
	
	
	//
	S_LPF accX_LPF;
	S_LPF accY_LPF;
	S_LPF accZ_LPF;	
	
	
	//quaternion
	float q0;
	float q1;
	float q2;
	float q3;

	
	//Sensor Feedback mesure value
	float mesurePitch;
	float mesureRoll;
	float mesureYaw;
	float mesureHeight;


  //Get Target Command Value Sended from BlueTooth
	float targetPitch;
	float targetRoll;
	float targetYaw; 
	float targetThrottle;

	
	float rollError;
	float pitchError;
	float yawError;
	

	//Use PID Controller attain to target Roll Pitch Yaw ,later we might add height control
	S_PID rollPID;
	S_PID pitchPID;
	S_PID yawPID;
	S_PID heightPID;
	
	
	uint16_x motorPWM[6];	
	//left front motor PWM
	uint16_x lfMotorPWM;
	//right front motor PWM
	uint16_x rfMotorPWM;
	//left rear motor PWM
	uint16_x lrMotorPWM;
	//right rear motor PWM
	uint16_x rrMotorPWM;
	
	

	//
	int32_x leftFrontMotorPWM;
	int32_x rightFrontMotorPWM;
	int32_x leftBackMotorPWM;  
	int32_x rightBackMororPWM;
	
	
	//
	int32_x mode;
	//remote communicate register
	uint32_x remote[32];
	//if lost recieve data ,throttle should slowly reduce value
	int32_x checkLost;
	
}S_BAT;


enum
{
		MODE_IDLE = 0,
		MODE_BAT = 1,
		MODE_CAR = 2,
		MODE_FAN = 3,
		MODE_DEBUG 
};


//ProjectX-BAT 
static S_BAT bat;


void BAT_InitTimeSystem(void)
{
    TIME_SYSTEM_Initialize();
}

enum {
	
	//
	MODE_REG  = 0,
	
	
	//
	ROLL_REG = 1,
	PITCH_REG = 2,
	YAW_REG = 3, 
	THROTTLE_REG = 4,
	
	
	//
	LEFT_MOTOR_REG = 5,
	RIGHT_MOTOR_REG = 6,
	
	
	//
	FAN_THROTTLE_REG = 7,
	
	
	//
	OBSERVE_ROLL = 8,
	OBSERVE_PITCH = 9,
	PID_ROLL_KP_REG = 10,
	PID_ROLL_KI_REG = 11,
	PID_ROLL_KD_REG = 12,
	PID_PITCH_KP_REG = 13,
	PID_PITCH_KI_REG = 14,
	PID_PITCH_KD_REG = 15,
	PID_YAW_KP_REG = 16,
	PID_YAW_KI_REG = 17,
	PID_YAW_KD_REG = 18,
};


enum {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4
};



void BAT_InitAHRS()
{
	
	//Init I2C
	I2C_Initialize();
	
	
	//Init MPU6050
	MPU6050_Initialize();
}


//--->MPU6050_Initialize();
//--->MS5611_Initialize();


#include "BAT_PARSER.H"
S_COMMAND_PARSER parser;


//
void BAT_OnRecieveBluetoothUARTValue(uint8_x value)
{
		uint16_x regAddr = 0;
		
		PARSER_RunPaser(&parser,value);
		
		
		if( PARSER_IsParseFinish(&parser) )
		{
				//Update Register and Data to BAT's
				regAddr = (uint16_x)parser.regAddrL + (((uint16_x)parser.regAddrH) << 8);
			
			
				//
				if( regAddr < sizeof(bat.remote)/sizeof(uint32_x) )
				{
						
						bat.remote[regAddr] = (((uint32_x)parser.regData[0]) << 24) +
																	(((uint32_x)parser.regData[1]) << 16) +
																	(((uint32_x)parser.regData[2]) << 8 ) +
																	(((uint32_x)parser.regData[3]) << 0 ) ;
				}
				
				PARSER_Reset(&parser);
		}

}


//
void BAT_OnRecieveCommandFinish()
{
	
}


//
void BAT_CommitCommandValue()
{
	
}


//2000us = 2ms = 0.002s
#define PID_LOOP_TIME		2000	//???us

void BAT_RollPIDCalculate(void)
{
		float error = 0.0f;
		
		
		//-400 , +400  -->   - 20 , + 20  
		error = bat.targetRoll - bat.mesureRoll;
			
	
		//
		if( bat.targetThrottle <= 1150)
		{
				PID_DisableIntegral( &bat.rollPID );
		}
		else
		{
				PID_EnableIntegral( &bat.rollPID );
		}
	
		
		//
		PID_PositionUpdate( &bat.rollPID , error , 0.005f );
}


void BAT_PitchPIDCalculate(void)
{
		float error = bat.targetPitch - bat.mesurePitch ;
			
	
		//
		if( bat.targetThrottle <= 1150)
		{
				PID_DisableIntegral( &bat.pitchPID );
		}
		else
		{
				PID_EnableIntegral( &bat.pitchPID );
		}

	
		PID_PositionUpdate( &bat.pitchPID , error , 0.005f );
}



void BAT_YawPIDCalculate(void)
{
		// delta angular speed * Time = delta angle (degree) ?? we control its degree into zero ??
		//range - 90 - 90 
		float error = (0 - bat.rawGyro.z * MPU6050G_s2000dps * 0.1 ) ;
	
		
		if( bat.targetThrottle <= 1150)
		{
				PID_DisableIntegral( &bat.yawPID );
		}
		else
		{
				PID_EnableIntegral( &bat.yawPID );
		}
	
		PID_PositionUpdate( &bat.yawPID , error , 0.005f );
		
		bat.yawPID.output = -1.0f * bat.yawPID.output;
		if( bat.yawPID.output > 200.0f ) bat.yawPID.output = 200.0f;
		if( bat.yawPID.output < -200.0f ) bat.yawPID.output = -200.0f;
}


// Custom mixer data per motor
typedef struct motorMixer_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;


static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};

#define FRONT_L (3)
#define FRONT_R (1)
#define REAR_L  (2)
#define REAR_R  (0)


/*
static const motorMixer_t mixerHex6X[] = {
    { 1.0f, -0.5f,  0.866025f,  1.0f },     // REAR_R
    { 1.0f, -0.5f, -0.866025f,  1.0f },     // FRONT_R
    { 1.0f,  0.5f,  0.866025f, -1.0f },     // REAR_L
    { 1.0f,  0.5f, -0.866025f, -1.0f },     // FRONT_L
    { 1.0f, -1.0f,  0.0f,      -1.0f },     // RIGHT
    { 1.0f,  1.0f,  0.0f,       1.0f },     // LEFT
};
*/

//
void BAT_PIDCalculateMixer()
{
	uint32_x i = 0;
	
	
	//	
	//	bat.targetThrottle = bat.rawData[THROTTLE];		 
	

	bat.motorPWM[0] = mixerQuadX[FRONT_L].throttle*bat.targetThrottle + mixerQuadX[FRONT_L].roll*bat.rollPID.output + mixerQuadX[FRONT_L].pitch*bat.pitchPID.output + mixerQuadX[FRONT_L].yaw*bat.yawPID.output;
	bat.motorPWM[1] = mixerQuadX[FRONT_R].throttle*bat.targetThrottle + mixerQuadX[FRONT_R].roll*bat.rollPID.output + mixerQuadX[FRONT_R].pitch*bat.pitchPID.output + mixerQuadX[FRONT_R].yaw*bat.yawPID.output;
	bat.motorPWM[2] = mixerQuadX[REAR_R].throttle*bat.targetThrottle  + mixerQuadX[REAR_R].roll*bat.rollPID.output  + mixerQuadX[REAR_R].pitch*bat.pitchPID.output  + mixerQuadX[REAR_R].yaw*bat.yawPID.output;
	bat.motorPWM[3] = mixerQuadX[REAR_L].throttle*bat.targetThrottle  + mixerQuadX[REAR_L].roll*bat.rollPID.output  + mixerQuadX[REAR_L].pitch*bat.pitchPID.output  + mixerQuadX[REAR_L].yaw*bat.yawPID.output;

	
	/*
	//
	bat.motorPWM[0] = mixerHex6X[0].throttle*bat.targetThrottle + mixerHex6X[0].roll*bat.rollPID.output ;//+ mixerHex6X[0].pitch*bat.pitchPID.output + mixerHex6X[0].yaw*bat.yawPID.output;
	bat.motorPWM[1] = mixerHex6X[1].throttle*bat.targetThrottle + mixerHex6X[1].roll*bat.rollPID.output ;//+ mixerHex6X[1].pitch*bat.pitchPID.output + mixerHex6X[1].yaw*bat.yawPID.output;
	bat.motorPWM[2] = mixerHex6X[2].throttle*bat.targetThrottle + mixerHex6X[2].roll*bat.rollPID.output ;//+ mixerHex6X[2].pitch*bat.pitchPID.output + mixerHex6X[2].yaw*bat.yawPID.output;
	bat.motorPWM[3] = mixerHex6X[3].throttle*bat.targetThrottle + mixerHex6X[3].roll*bat.rollPID.output ;//+ mixerHex6X[3].pitch*bat.pitchPID.output + mixerHex6X[3].yaw*bat.yawPID.output;
	bat.motorPWM[4] = mixerHex6X[4].throttle*bat.targetThrottle + mixerHex6X[4].roll*bat.rollPID.output ;//+ mixerHex6X[4].pitch*bat.pitchPID.output + mixerHex6X[4].yaw*bat.yawPID.output;
	bat.motorPWM[5] = mixerHex6X[5].throttle*bat.targetThrottle + mixerHex6X[5].roll*bat.rollPID.output ;//+ mixerHex6X[5].pitch*bat.pitchPID.output + mixerHex6X[5].yaw*bat.yawPID.output;
	*/
	
	//constrain
	for (i = 0; i < 6; i++) 
	{
		if( bat.motorPWM[i] < 1100 ) bat.motorPWM[i] = 1000;
		if( bat.motorPWM[i] > 1900 ) bat.motorPWM[i] = 1900;
	}

}	
//
void BAT_CommitMotorPWMOutput()
{
		if( bat.targetThrottle >= 1100 )
		{
			TIM2->CCR1 = bat.motorPWM[0] - 1000;
			TIM2->CCR2 = bat.motorPWM[1] - 1000;
			TIM2->CCR3 = bat.motorPWM[2] - 1000;
			TIM2->CCR4 = bat.motorPWM[3] - 1000;
		}
		else
		{
			TIM2->CCR1 = 0;
			TIM2->CCR2 = 0;
			TIM2->CCR3 = 0;
			TIM2->CCR4 = 0;
		}
}
//
bool_x BAT_IsExecption()
{
	
	
	return false_x;
}


//8.6  , 1.49 , 0.27 
void BAT_InitController(void)
{	
	//Roll PID
	//Attention : battery is on the upside!
	PID_Init(&bat.rollPID,7.7f,4.4f,1.8f,-2500.0f,2500.0f);//7.3f,14.1f,0.8f,-2500.0f,2500.0f);
	PID_Init(&bat.pitchPID,7.3f,4.3f,1.1f,-2500.0f,2500.0f);
	PID_Init(&bat.yawPID,4.0f,4.0f,0.0f,-2500.0f,2500.0f);//5.3f,1.0f//8.0 8.0
}




void BAT_InitFilter(void)
{
		LPF_Init(&bat.accX_LPF,0.005f,10.0f);
		LPF_Init(&bat.accY_LPF,0.005f,10.0f);
		LPF_Init(&bat.accZ_LPF,0.005f,10.0f);
}


void BAT_ReadRawSensorData(void)
{	
	MPU6050_GetRawAccel(&bat.rawAccel);
	MPU6050_GetRawGyro(&bat.rawGyro);
}







void BAT_LowPassFilteringAccData(void)
{
	LPF_Update(&bat.accX_LPF,bat.unitAccel.x);
	LPF_Update(&bat.accY_LPF,bat.unitAccel.y);
	LPF_Update(&bat.accZ_LPF,bat.unitAccel.z);
}




void BAT_HighPassFilteringGyroData(void)
{
	
}



#include "MahonyAHRS.h"



void BAT_MahonyFilter(void)
{
	MahonyAHRSupdate(
	bat.unitGyro.x, 
	bat.unitGyro.y,
	bat.unitGyro.z,
	bat.accX_LPF.output,
	bat.accY_LPF.output,
	bat.accZ_LPF.output,
	0.0f,
	0.0f,
	0.0f
	);
	
	
	MahonyAHRS_GetQuaternion(&bat.q0,&bat.q1,&bat.q2,&bat.q3);

}


//?????
#define RAD_TO_DEG 57.29577951f
float degrees(float rad) {
	return rad * RAD_TO_DEG;
}

#include <stdint.h>
#include <stddef.h>
#include <math.h>
//
float safe_asin(float v)
{
    if (isnan(v)) {
        return 0.0;
    }
    if (v >= 1.0f) {
        return M_PI/2;
    }
    if (v <= -1.0f) {
        return -M_PI/2;
    }
    return asinf(v);
}


void BAT_Quaternion2EulerAngle(void)
{	
		float q1 = bat.q0;
		float q2 = bat.q1;
		float q3 = bat.q2;
		float q4 = bat.q3;
	
		
		//
		bat.mesureRoll = degrees(atan2f(2.0f*(q1*q2 + q3*q4),1 - 2.0f*(q2*q2 + q3*q3)));
		bat.mesurePitch = degrees(safe_asin(2.0f*(q1*q3 - q2*q4))) ;
		bat.mesureYaw = degrees(atan2f(2.0f*(q2*q3 - q1*q4), 2.0f*(q1*q1 + q2*q2) - 1));	
}




void BAT_SensorDataUnitization(void)
{
		
		MPU6050_AccelGravityUnitization(&bat.unitAccel, &bat.rawAccel);
		MPU6050_GyroDpsUnitization(&bat.unitGyro ,&bat.rawGyro)	;
}



void BAT_OnRecieveDataStream(uint8_t *data,int32_t length)
{
	uint8_x sum = 0,i = 0;
	
	for( i=0;i<(length-1);i++)
	{
			sum += data[i];
	}
	
	
	if( sum != data[length-1])
	{
			return;
	}
	
	
	if( (data[0] != 0XAA ) || ( data[1] != 0XAF ) )
	{
			return;
	}
	
	//updata control information
	if( data[2] == 0X03)
	{
		bat.rawData[THROTTLE] = (vs16)(data[4]<<8) | data[5];
		bat.rawData[YAW] = (vs16)(data[6]<<8) | data[7];
		bat.rawData[ROLL] = (vs16)(data[8]<<8) | data[9];
		bat.rawData[PITCH] = (vs16)(data[10]<<8) | data[11];
		
		
		//change to normal unit  (-400 , +400 )   ->  (-20 deg, +20 deg)
		bat.targetRoll =  (bat.rawData[ROLL] - 1500 ) * 0.05f;  
		bat.targetPitch = (bat.rawData[PITCH] - 1500 ) * 0.05f;
		bat.targetThrottle = bat.rawData[THROTTLE];
		
	}

}




extern uint16_t cnt_1ms,cnt_2ms,cnt_5ms,cnt_10ms,cnt_20ms;
int checkLost = 0;
int main(void)
{
	
	//
	BAT_InitTimeSystem();
	
	//
	BAT_InitCommunicator();
		
	//
	BAT_InitFilter(); 
		
	//
	BAT_InitAHRS();
	
	//
	BAT_InitController();
	
	//
	BAT_InitActuator();
		
	
	//
	while(1)
	{
		if( cnt_5ms >= 5 )
		{
			cnt_5ms = 0;
			
			//
			BAT_ReadRawSensorData();
			
			BAT_SensorDataUnitization();
			
			BAT_LowPassFilteringAccData();
			
			BAT_MahonyFilter();
			
			BAT_Quaternion2EulerAngle();
			
			BAT_RollPIDCalculate();
			
			BAT_PitchPIDCalculate();
			
			BAT_YawPIDCalculate();
			
			BAT_PIDCalculateMixer();
			
			BAT_CommitMotorPWMOutput();
			
			//
			while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET);
			// 1/10 's angular speed 
			USART_SendData(USART1, (int8_x)bat.mesureRoll);
			
			if( checkLost >= 100)
			{
					bat.remote[THROTTLE_REG] = 0;
					bat.targetThrottle = 0;
			}
			else
			{
					checkLost++;
			}
		}

		
		//
		bat.targetRoll = (int8_x)bat.remote[ROLL_REG];  
		bat.targetPitch = (int8_x)bat.remote[PITCH_REG];
		bat.targetYaw = 0;
		bat.targetThrottle = 10 * (int32_x)bat.remote[THROTTLE_REG] +1000; //(0 - 100)		
		
		
		
		//Roll
		if( 0 != bat.remote[PID_ROLL_KP_REG] )
		{
				bat.rollPID.kP = bat.remote[PID_ROLL_KP_REG] / 10.0f;
				bat.remote[PID_ROLL_KP_REG] = 0;
		}
		if( 0 != bat.remote[PID_ROLL_KI_REG] )
		{
				bat.rollPID.kI = bat.remote[PID_ROLL_KI_REG] / 10.0f;
				bat.remote[PID_ROLL_KI_REG] = 0;
		}
		if( 0 != bat.remote[PID_ROLL_KD_REG] )
		{
				bat.rollPID.kD = bat.remote[PID_ROLL_KD_REG] / 10.0f;
				bat.remote[PID_ROLL_KD_REG] = 0;
		}
		
		
		
		//Pitch
		if( 0 != bat.remote[PID_PITCH_KP_REG] )
		{
				bat.pitchPID.kP = bat.remote[PID_PITCH_KP_REG] / 10.0f ;
				bat.remote[PID_PITCH_KP_REG] = 0;
		}
		if( 0 != bat.remote[PID_PITCH_KI_REG] )
		{
				bat.pitchPID.kI = bat.remote[PID_PITCH_KI_REG] / 10.0f;
				bat.remote[PID_PITCH_KI_REG] = 0;
		}
		if( 0 != bat.remote[PID_PITCH_KD_REG] )
		{
				bat.pitchPID.kD = bat.remote[PID_PITCH_KD_REG] / 10.0f;
				bat.remote[PID_PITCH_KD_REG] = 0;
		}

		
//		//Yaw
//		if( 0 != bat.remote[PID_YAW_KP_REG] )
//		{
//				bat.yawPID.kP = bat.remote[PID_YAW_KP_REG] / 10.0f;
//				bat.remote[PID_YAW_KP_REG] = 0;
//		}
//		if( 0 != bat.remote[PID_YAW_KI_REG] )
//		{
//				bat.yawPID.kI = bat.remote[PID_YAW_KI_REG] / 10.0f;
//				bat.remote[PID_YAW_KI_REG] = 0;
//		}
//		if( 0 != bat.remote[PID_YAW_KD_REG] )
//		{
//				bat.yawPID.kD = bat.remote[PID_YAW_KD_REG] / 10.0f;
//				bat.remote[PID_YAW_KD_REG] = 0;
//		}
	}
}

