#include "BAT_MPU6050.h"
#include "BAT_I2C.h"

//大概72*1000个指令 = 1ms
void MPU6050_DelayMs(uint32_x ms)
{
		uint32_x i=0,count=0;
		for( i=0;i<ms;i++ )
		{
				count = 72000;
				while( count-- );
		}
}


void MPU6050_ReadRegister(uint8_x registerAddress,uint8_x *registerData)
{
		I2C_SingleRead(MPU6050_DEFAULT_ADDRESS,registerAddress,registerData);
}


void MPU6050_WriteRegister(uint8_x registerAddress,uint8_x registerData)
{
		I2C_SingleWrite(MPU6050_DEFAULT_ADDRESS,registerAddress,registerData);
}




#define MPU6050_RESET_BIT  (7)
void MPU6050_Reset(void)
{
		MPU6050_WriteRegister(MPU6050_RA_PWR_MGMT_1,1<<MPU6050_RESET_BIT);
}


//陀螺仪采样率，0x00(1000Hz)   采样率 = 陀螺仪的输出率 / (1 + SMPLRT_DIV)
void MPU6050_SetGyroSampleRate(int32_x sampleRate)
{
		I2C_SingleWrite(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_SMPLRT_DIV, (1000/sampleRate - 1));	
}


/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see MPU6050_GetClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
#define MPU6050_PWR1_CLKSEL_BIT   (0)
#define CLOCK_SOURCE_INTRENAL     (0<<MPU6050_PWR1_CLKSEL_BIT)
#define CLOCK_SOURCE_PLL_X_GYRO   (1<<MPU6050_PWR1_CLKSEL_BIT)
#define CLOCK_SOURCE_PLL_Y_GYRO   (2<<MPU6050_PWR1_CLKSEL_BIT)
#define CLOCK_SOURCE_PLL_Z_GYRO   (3<<MPU6050_PWR1_CLKSEL_BIT)
#define CLOCK_SOURCE_EXT_32P768K  (4<<MPU6050_PWR1_CLKSEL_BIT)
#define CLOCK_SOURCE_EXT_19P2K    (5<<MPU6050_PWR1_CLKSEL_BIT)
#define CLOCK_SOURCE_RESET        (7<<MPU6050_PWR1_CLKSEL_BIT)
void MPU6050_SetClockSource(uint8_x source)
{
		MPU6050_WriteRegister(MPU6050_RA_PWR_MGMT_1,source);
}


/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
#define MPU6050_GCONFIG_FS_SEL_BIT (3)
#define GYRO_RANGE_250DEG   (0<<MPU6050_GCONFIG_FS_SEL_BIT)
#define GYRO_RANGE_500DEG   (1<<MPU6050_GCONFIG_FS_SEL_BIT)
#define GYRO_RANGE_1000DEG  (2<<MPU6050_GCONFIG_FS_SEL_BIT)
#define GYRO_RANGE_2000DEG  (3<<MPU6050_GCONFIG_FS_SEL_BIT)
void MPU6050_SetFullScaleGyroRange(uint8_x range)
{
		MPU6050_WriteRegister(MPU6050_RA_GYRO_CONFIG,range);
}





/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
#define MPU6050_ACONFIG_AFS_SEL_BIT (3)
#define ACCEL_RANGE_2G   		(0<<MPU6050_ACONFIG_AFS_SEL_BIT)
#define ACCEL_RANGE_4G   		(1<<MPU6050_ACONFIG_AFS_SEL_BIT)
#define ACCEL_RANGE_8G      (2<<MPU6050_ACONFIG_AFS_SEL_BIT)
#define ACCEL_RANGE_16G     (3<<MPU6050_ACONFIG_AFS_SEL_BIT)
void MPU6050_SetFullScaleAccelRange(uint8_x range)
{
		MPU6050_WriteRegister(MPU6050_RA_ACCEL_CONFIG, range);	
}




/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 : accel bandwidth=260  delay=0ms / gyro bandwidth=256 delay=0.98ms
 * 1 : accel bandwidth=184  delay=2.0ms / gyro bandwidth=188 delay=1.9ms
 * 2 : accel bandwidth=184  delay=2.0ms / gyro bandwidth=188 delay=1.9ms
 * 3 : accel bandwidth=184  delay=2.0ms / gyro bandwidth=188 delay=1.9ms
 * 4 : accel bandwidth=184  delay=2.0ms / gyro bandwidth=188 delay=1.9ms
 * 5 : accel bandwidth=184  delay=2.0ms / gyro bandwidth=188 delay=1.9ms
 * 6 : accel bandwidth=184  delay=2.0ms / gyro bandwidth=188 delay=1.9ms
 * 未完成，需要参考芯片资料完成！
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
#define MPU6050_DLPF_SEL_BIT (0)
#define DLPF_BANDWIDTH_256HZ     (0<<MPU6050_DLPF_SEL_BIT)
#define DLPF_BANDWIDTH_188HZ     (1<<MPU6050_DLPF_SEL_BIT)
#define DLPF_BANDWIDTH_98HZ      (2<<MPU6050_DLPF_SEL_BIT)
#define DLPF_BANDWIDTH_42HZ      (3<<MPU6050_DLPF_SEL_BIT)
#define DLPF_BANDWIDTH_20HZ      (4<<MPU6050_DLPF_SEL_BIT)
#define DLPF_BANDWIDTH_10HZ      (5<<MPU6050_DLPF_SEL_BIT)
#define DLPF_BANDWIDTH_5HZ       (6<<MPU6050_DLPF_SEL_BIT)
void MPU6050_SetLowPassFilterFreq(uint8_x selectFilter)
{
		MPU6050_WriteRegister(MPU6050_RA_CONFIG,selectFilter);
}



#define BYPASS_MODE_ENABLE_BIT (1)
void MPU6050_SetByPassModeEnable(bool_x isEnable)
{
		if( isEnable )
		{
				MPU6050_WriteRegister(MPU6050_RA_INT_PIN_CFG,1<<BYPASS_MODE_ENABLE_BIT);
		}
		else
		{
				MPU6050_WriteRegister(MPU6050_RA_INT_PIN_CFG,0<<BYPASS_MODE_ENABLE_BIT);
		}
}


bool_x MPU6050_TestConnection(void)
{
		return true_x;
}


//normalization
void MPU6050_GetRawAccel(S_RAW_ACCEL *rawAccel)
{
		uint8_x mpu6050_buffer[6];
		bool_x isSuccess = true_x;
	
	
		//
		isSuccess &= I2C_SingleRead(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_ACCEL_XOUT_L,&mpu6050_buffer[0]);
		isSuccess &= I2C_SingleRead(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_ACCEL_XOUT_H,&mpu6050_buffer[1]);
		isSuccess &= I2C_SingleRead(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_ACCEL_YOUT_L,&mpu6050_buffer[2]);
		isSuccess &= I2C_SingleRead(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_ACCEL_YOUT_H,&mpu6050_buffer[3]);
		isSuccess &= I2C_SingleRead(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_ACCEL_ZOUT_L,&mpu6050_buffer[4]);
		isSuccess &= I2C_SingleRead(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_ACCEL_ZOUT_H,&mpu6050_buffer[5]);
		
		
		//
		if( isSuccess )
		{
			rawAccel->x = (int16_t)((((int16_t)mpu6050_buffer[1]) << 8) | (mpu6050_buffer[0]));
			rawAccel->y = (int16_t)((((int16_t)mpu6050_buffer[3]) << 8) | (mpu6050_buffer[2]));
			rawAccel->z = (int16_t)((((int16_t)mpu6050_buffer[5]) << 8) | (mpu6050_buffer[4]));
		}
}


void MPU6050_GetRawGyro(S_RAW_GYRO *rawGyro)
{
		uint8_x mpu6050_buffer[6];
		bool_x isSuccess = true_x;
	
	
		//
		isSuccess &= I2C_SingleRead(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_GYRO_XOUT_L,&mpu6050_buffer[0]) ; 
		isSuccess &= I2C_SingleRead(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_GYRO_XOUT_H,&mpu6050_buffer[1]) ;
		isSuccess &= I2C_SingleRead(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_GYRO_YOUT_L,&mpu6050_buffer[2]) ;
		isSuccess &= I2C_SingleRead(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_GYRO_YOUT_H,&mpu6050_buffer[3]);
		isSuccess &= I2C_SingleRead(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_GYRO_ZOUT_L,&mpu6050_buffer[4]);
		isSuccess &= I2C_SingleRead(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_GYRO_ZOUT_H,&mpu6050_buffer[5]);

		
		//
		if( isSuccess )
		{
			rawGyro->x = (int16_t)((((int16_t)mpu6050_buffer[1]) << 8) | mpu6050_buffer[0]) ;
			rawGyro->y = (int16_t)((((int16_t)mpu6050_buffer[3]) << 8) | mpu6050_buffer[2]) ;
			rawGyro->z = (int16_t)((((int16_t)mpu6050_buffer[5]) << 8) | mpu6050_buffer[4]) ;
		}

}


#define MPU6050A_2mg                ((float)0.00006103f)  // 0.00006250 g/LSB
#define MPU6050A_4mg                ((float)0.00012207f)  // 0.00012500 g/LSB
#define MPU6050A_8mg                ((float)0.00024414f)  // 0.00025000 g/LSB

#define MPU6050G_s250dps            ((float)0.0076335f)  // 0.0087500 dps/LSB
#define MPU6050G_s500dps            ((float)0.0152671f)  // 0.0175000 dps/LSB
#define MPU6050G_s2000dps           ((float)0.0609756f)  // 0.0700000 dps/LSB


void MPU6050_AccelGravityUnitization(S_GRAVITY_UNIT_ACCEL *unitAccel,S_RAW_ACCEL *rawAccel)
{
		unitAccel->x = rawAccel->x * MPU6050A_8mg;
		unitAccel->y = rawAccel->y * MPU6050A_8mg;
		unitAccel->z = rawAccel->z * MPU6050A_8mg;
}


void MPU6050_GetGravityUnitAccel(S_GRAVITY_UNIT_ACCEL *unitAccel)
{
		S_RAW_ACCEL rawAccel;
		MPU6050_GetRawAccel(&rawAccel);
		unitAccel->x = rawAccel.x * MPU6050A_8mg;
		unitAccel->y = rawAccel.y * MPU6050A_8mg;
		unitAccel->z = rawAccel.z * MPU6050A_8mg;
}


#define DEG_TO_RAD 0.01745329f

//角度转弧度
float radians(float deg) 
{
	return deg * DEG_TO_RAD;
}

void MPU6050_GetDpsGyro(S_DPS_UNIT_GYRO *unitGyro)
{
		S_RAW_GYRO rawGyro;
		MPU6050_GetRawGyro(&rawGyro);
		unitGyro->x = radians(rawGyro.x * MPU6050G_s2000dps);
		unitGyro->y = radians(rawGyro.y * MPU6050G_s2000dps);
		unitGyro->z = radians(rawGyro.z * MPU6050G_s2000dps);
}
	

void MPU6050_GyroDpsUnitization(S_DPS_UNIT_GYRO *unitGyro,S_RAW_GYRO *rawGyro)
{
		unitGyro->x = radians(rawGyro->x * MPU6050G_s2000dps);
		unitGyro->y = radians(rawGyro->y * MPU6050G_s2000dps);
		unitGyro->z = radians(rawGyro->z * MPU6050G_s2000dps);
}



void MPU6050_Initialize(void)
{
	//
	MPU6050_Reset();
	//
	MPU6050_DelayMs(5);
	//
	MPU6050_SetGyroSampleRate(1000);
	//
	MPU6050_SetClockSource(CLOCK_SOURCE_PLL_Z_GYRO);
	//
	MPU6050_SetByPassModeEnable(false_x);
	//
	MPU6050_SetLowPassFilterFreq(DLPF_BANDWIDTH_42HZ);
	//
	MPU6050_SetFullScaleGyroRange(GYRO_RANGE_2000DEG);
	//
	MPU6050_SetFullScaleAccelRange(ACCEL_RANGE_8G);
}






