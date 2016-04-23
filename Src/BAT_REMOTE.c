#include "BAT_REMOTE.H"
#include "nrf24.h"
#include "stdint.h"
#include "stm32f10x.h"




/***************SPI2 GPIO定义******************/
#define BAT_GPIO_SPI2		GPIOB
#define BAT_GPIO_CE2		GPIOA
#define SPI2_Pin_SCK		GPIO_Pin_13
#define SPI2_Pin_MISO		GPIO_Pin_14
#define SPI2_Pin_MOSI		GPIO_Pin_15
#define SPI2_Pin_CE2		GPIO_Pin_8
#define SPI2_Pin_CSN		GPIO_Pin_12
#define RCC_GPIO_SPI2		RCC_APB2Periph_GPIOB
#define RCC_GPIO_CE2		RCC_APB2Periph_GPIOA
/*********************************************/
#define TX_ADR_WIDTH    (5) 	 	
#define RX_ADR_WIDTH    (5)
static uint8_t	TX_ADDRESS[TX_ADR_WIDTH] = {0xAA,0xBB,0xCC,0x00,0x01};	//本地地址
static uint8_t	RX_ADDRESS[RX_ADR_WIDTH] = {0xAA,0xBB,0xCC,0x00,0x01};	//接收地址	



/* -------------------------------------------------------------------------- */
/* In this function you should do the following things:
 *    - Set MISO pin input
 *    - Set MOSI pin output
 *    - Set SCK pin output
 *    - Set CSN pin output
 *    - Set CE pin output     */
/* -------------------------------------------------------------------------- */
void nrf24_setupPins(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_GPIO_SPI2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_GPIO_CE2, ENABLE);
		
		//MISO
		GPIO_InitStructure.GPIO_Pin = SPI2_Pin_MISO; 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
		GPIO_Init(BAT_GPIO_SPI2, &GPIO_InitStructure);
	
	
		//MOSI SCK
		GPIO_InitStructure.GPIO_Pin = SPI2_Pin_SCK | SPI2_Pin_MOSI; 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
		GPIO_Init(BAT_GPIO_SPI2, &GPIO_InitStructure);
	
		
	
		//CE CSN
		GPIO_InitStructure.GPIO_Pin = SPI2_Pin_CE2; 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
		GPIO_Init(BAT_GPIO_CE2, &GPIO_InitStructure);
		
		
		GPIO_InitStructure.GPIO_Pin = SPI2_Pin_CSN; 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
		GPIO_Init(BAT_GPIO_SPI2, &GPIO_InitStructure);
		
}



/* -------------------------------------------------------------------------- */
/* nrf24 CE pin control function
 *    - state:1 => Pin HIGH
 *    - state:0 => Pin LOW     */
/* -------------------------------------------------------------------------- */
void nrf24_ce_digitalWrite(uint8_t state)
{
		if( state )
		{
				GPIO_SetBits(BAT_GPIO_CE2, SPI2_Pin_CE2);
		}
		else
		{
				GPIO_ResetBits(BAT_GPIO_CE2, SPI2_Pin_CE2);
		}
}

/* -------------------------------------------------------------------------- */
/* nrf24 CSN pin control function
 *    - state:1 => Pin HIGH
 *    - state:0 => Pin LOW     */
/* -------------------------------------------------------------------------- */
void nrf24_csn_digitalWrite(uint8_t state)
{
		if(state)
		{
				GPIO_SetBits(BAT_GPIO_SPI2, SPI2_Pin_CSN);
		}
		else
		{
				GPIO_ResetBits(BAT_GPIO_SPI2, SPI2_Pin_CSN);
		}
}

/* -------------------------------------------------------------------------- */
/* nrf24 SCK pin control function
 *    - state:1 => Pin HIGH
 *    - state:0 => Pin LOW     */
/* -------------------------------------------------------------------------- */
void nrf24_sck_digitalWrite(uint8_t state)
{
		if(state)
		{
				GPIO_SetBits(BAT_GPIO_SPI2, SPI2_Pin_SCK);
		}
		else
		{
				GPIO_ResetBits(BAT_GPIO_SPI2, SPI2_Pin_SCK);
		}
}

/* -------------------------------------------------------------------------- */
/* nrf24 MOSI pin control function
 *    - state:1 => Pin HIGH
 *    - state:0 => Pin LOW     */
/* -------------------------------------------------------------------------- */
void nrf24_mosi_digitalWrite(uint8_t state)
{
		if(state)
		{
				GPIO_SetBits(BAT_GPIO_SPI2, SPI2_Pin_MOSI);
		}
		else
		{
				GPIO_ResetBits(BAT_GPIO_SPI2, SPI2_Pin_MOSI);
		}	
}


/* -------------------------------------------------------------------------- */
/* nrf24 MISO pin read function						
   - returns: Non-zero if the pin is high */
/* -------------------------------------------------------------------------- */
uint8_t nrf24_miso_digitalRead(void)
{
		return GPIO_ReadInputDataBit(BAT_GPIO_SPI2, SPI2_Pin_MISO);
}

#include "BAT_BLE.H"

void BAT_InitCommunicator(void)
{
		/*
		//
		nrf24_init();
	
		nrf24_tx_address(TX_ADDRESS);
	
		nrf24_rx_address(RX_ADDRESS);
	
		nrf24_config(80);
		*/
		
		BLE_Init();
		
}









void nrf24_sendData(uint8_t * tx_buf, uint8_t len)
{
		// StandBy I模式	
		nrf24_ce_digitalWrite(LOW);		 
		// 装载接收端地址
		nrf24_writeRegister(W_REGISTER | RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); 
		// 装载数据	
		nrf24_writeRegister(W_TX_PAYLOAD, tx_buf, len); 	
		// 置高CE，激发数据发送	
		nrf24_ce_digitalWrite(HIGH);		 
}


void BAT_Send_Heartbeat(void)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i = 0;
	static u8 data_to_send[30] = {0};
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;	
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[3] = _cnt-4;
	
	
	// count and add checksum
	for(i=0;i<_cnt;i++)
	{
			sum += data_to_send[i];
	}
	data_to_send[_cnt++]=sum;
	
	
	nrf24_sendData(data_to_send, _cnt);
	
}





#define RX_PLOAD_WIDTH  255  	
#define TX_PLOAD_WIDTH  255  	
uint8_t NRF24L01_2_RXDATA[RX_PLOAD_WIDTH];

//1-远程写寄存器，加上校验的做法！！！参照MOBUS协议！！！！CRC16校验，，读和写，区分开???
#define RX_FIFO_MAX_LENGTH  (32)

void BAT_CommunicationEventHandler(void)
{
	
	uint8_t status = 0,length=0;
	
	//
	status = nrf24_getStatus();
	
	//
	if( nrf24_isRecieveData(status) )
	{
			length = nrf24_payloadLength();
			
			if( length <= RX_FIFO_MAX_LENGTH )
			{
					nrf24_getData(NRF24L01_2_RXDATA,length);
				
					//
					BAT_OnRecieveDataStream(NRF24L01_2_RXDATA,length);
			}
			else
			{
					nrf24_flushRX();
			}
	}
	
	if( nrf24_isDataSent(status) )
	{
			
	}
	
	//
	if( nrf24_isRetransmitTooMuch(status) )
	{
			if( nrf24_isTXFIFOFull(status ) )
			{
					nrf24_flushTX();
			}
	}
	
	//
	nrf24_clearStatus(status);

}






