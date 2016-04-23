#ifndef _BAT_I2C_H_
#define _BAT_I2C_H_

#include "BAT_Std.h"
#include "stm32f10x.h"

/***************I2C GPIO**********************/
#define BAT_GPIO_I2C	GPIOB
#define I2C_Pin_SCL		GPIO_Pin_7
#define I2C_Pin_SDA		GPIO_Pin_6
#define BAT_RCC_I2C		RCC_APB2Periph_GPIOB
/*********************************************/
#define SCL_H         GPIOB->BSRR = I2C_Pin_SCL
#define SCL_L         GPIOB->BRR  = I2C_Pin_SCL
#define SDA_H         GPIOB->BSRR = I2C_Pin_SDA
#define SDA_L         GPIOB->BRR  = I2C_Pin_SDA
#define SCL_READ      GPIOB->IDR  & I2C_Pin_SCL
#define SDA_READ      GPIOB->IDR  & I2C_Pin_SDA 
#define I2C_MINIMAL_DELAY_COUNT (10)


void I2C_Initialize(void);

//Interface
bool_x I2C_SingleRead(uint8_x slaveAddress,uint8_x registerAddress,uint8_x *registerData);
bool_x I2C_SingleWrite(uint8_x slaveAddress,uint8_x registerAddress,uint8_x registerData);
bool_x I2C_MultiRead(uint8_x slaveAddress,uint8_x registerAddress,uint8_x *readBytes,uint8_x readSize);

#endif

