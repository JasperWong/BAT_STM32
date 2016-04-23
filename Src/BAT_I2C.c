#include "BAT_I2C.h"



void I2C_Initialize(void)
{
		//InitI2C
		GPIO_InitTypeDef  GPIO_InitStructure; 
		RCC_APB2PeriphClockCmd(BAT_RCC_I2C , ENABLE );
		GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
		GPIO_Init(BAT_GPIO_I2C, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SDA;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
		GPIO_Init(BAT_GPIO_I2C, &GPIO_InitStructure);	
}





//I2C内部使用函数
static void I2C_Delay(void);
static bool_x  I2C_Start(void);
static void I2C_Stop(void);
static void I2C_Ack(void);
static void I2C_NoAck(void);
static bool_x  I2C_WaitAck(void);
static uint8_x I2C_RecieveByte(void);


static void I2C_Delay(void)
{	
    int32_x i = I2C_MINIMAL_DELAY_COUNT; 
    while( i >= 0 ) 
    {
        i--;
    }
}


static bool_x I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_Delay();
    //SDA线为低电平则总线忙
    if(!SDA_READ)
    {
        return false_x;	
    }
    SDA_L;
    I2C_Delay();
    //SDA线为高电平则总线出错
    if(SDA_READ)
    {
        return false_x;	
    }
    SDA_L;
    I2C_Delay();
    return true_x;
}


static void I2C_Stop(void)
{
    SCL_L;
    I2C_Delay();
    SDA_L;
    I2C_Delay();
    SCL_H;
    I2C_Delay();
    SDA_H;
    I2C_Delay();
}


static void I2C_Ack(void)
{	
    SCL_L;
    I2C_Delay();
    SDA_L;
    I2C_Delay();
    SCL_H;
    I2C_Delay();
    SCL_L;
    I2C_Delay();
}


static void I2C_NoAck(void)
{	
    SCL_L;
    I2C_Delay();
    SDA_H;
    I2C_Delay();
    SCL_H;
    I2C_Delay();
    SCL_L;
    I2C_Delay();
}


//返回为:=1有ACK,=0无ACK
static bool_x I2C_WaitAck(void) 	 
{
    SCL_L;
    I2C_Delay();
    SDA_H;			
    I2C_Delay();
    SCL_H;
    I2C_Delay();
    if(SDA_READ)
    {
        SCL_L;
        I2C_Delay();
        return false_x;
    }
    SCL_L;
    I2C_Delay();
    return true_x;
}


//数据从高位到低位//
static void I2C_SendByte(uint8_x sendByte) 
{
    uint8_x i=0;
    for(i=0;i<8;i++)
    {
        SCL_L;
        I2C_Delay();
        if(sendByte&0x80)
        {
            SDA_H;  
        }
        else 
        {
            SDA_L;   
        }
        sendByte<<=1;
        I2C_Delay();
        SCL_H;
        I2C_Delay();
    }
    SCL_L;
}


//数据从高位到低位//
static uint8_x I2C_RecieveByte(void)  
{ 
    uint8_x i=0;
    uint8_x receiveByte=0;
    
    SDA_H;				
    for(i=0;i<8;i++)
    {
        receiveByte<<=1;      
        SCL_L;
        I2C_Delay();
        SCL_H;
        I2C_Delay();	
        if(SDA_READ)
        {
            receiveByte|=0x01;
        }
    }
    SCL_L;
    
    return receiveByte;
} 


bool_x I2C_SingleWrite(uint8_x slaveAddress,uint8_x registerAddress,uint8_x registerData)
{
    if( !I2C_Start())
    {
        return false_x;
    }
		//发送设备地址+写信号//I2C_SendByte(((registerAddress & 0x0700) >>7) | slaveAddress & 0xFFFE);//设置高起始地址+器件地址 
    I2C_SendByte(slaveAddress);   
    if( !I2C_WaitAck() )
    {
        I2C_Stop(); 
        return false_x;
    }
		//设置低起始地址      
    I2C_SendByte(registerAddress);   
    I2C_WaitAck();	
    I2C_SendByte(registerData);
    I2C_WaitAck();   
    I2C_Stop(); 
    return true_x;
}


bool_x I2C_SingleRead(uint8_x slaveAddress,uint8_x registerAddress ,uint8_x *registerData)
{
    if( NULL == registerData)
    {
        return false_x;
    }
    if( !I2C_Start())
    {
        return false_x;
    }
    I2C_SendByte(slaveAddress); //I2C_SendByte(((registerAddress & 0x0700) >>7) | registerAddress & 0xFFFE);//设置高起始地址+器件地址 
    if( !I2C_WaitAck() )
    {
        I2C_Stop();
        return false_x;
    }
    I2C_SendByte(registerAddress);   //设置低起始地址      
    I2C_WaitAck();
    I2C_Start();
		//TODO:有空可以分析一下这里加1的原因
    I2C_SendByte(slaveAddress+1);
    I2C_WaitAck();
    (*registerData) = I2C_RecieveByte();
    I2C_NoAck();
    I2C_Stop();
	  return true_x;
}


bool_x I2C_MultiRead(uint8_x slaveAddress,uint8_x registerAddress,uint8_x *readBytes,uint8_x readSize)
{
    uint8_x i;
    
    if(readSize < 1)
    {
          return false_x;
    }
    if(false_x == I2C_Start())
    {
          return false_x;
    }
    I2C_SendByte(slaveAddress);
    if(false_x == I2C_WaitAck())
    {
          I2C_Stop();
          return false_x;
    }
    I2C_SendByte(registerAddress);    
    I2C_WaitAck();
    
    I2C_Start();
    I2C_SendByte(slaveAddress+1);
    I2C_WaitAck();
    
		//
    for( i=0;i < (readSize - 1);i++ )
    {
        readBytes[i] = I2C_RecieveByte();
        I2C_Ack();
    }
		
		//
		readBytes[readSize - 1] = I2C_RecieveByte();
    I2C_NoAck();
    I2C_Stop();
    return true_x;
}	

