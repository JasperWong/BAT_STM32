#ifndef _BAT_PARSER_H_
#define _BAT_PARSER_H_

#include "BAT_STD.H"
#include "stm32f10x.h"

typedef struct
{
	
#define STATE_WAIT_P 								(0)
#define STATE_WAIT_X 								(1)
#define STATE_WAIT_REG_ADDR_H 			(2)
#define STATE_WAIT_REG_ADDR_L 			(3)
#define STATE_WAIT_REG_DATA_1 			(4)
#define STATE_WAIT_REG_DATA_2 			(5)
#define STATE_WAIT_REG_DATA_3 			(6)
#define STATE_WAIT_REG_DATA_4 			(7)
#define STATE_WAIT_CHECKSUM 				(8)
#define STATE_WAIT_E								(9)	
#define STATE_PARSE_FINISH_PENDING 	(10)	
	
	int32_t status;
	uint8_t regAddrL;
	uint8_t regAddrH;
	uint8_t regData[4];
	uint8_t checksum;
	
}S_COMMAND_PARSER;

bool_x PARSER_IsParseFinish(S_COMMAND_PARSER *parser);
void PARSER_Reset(S_COMMAND_PARSER *parser);
void PARSER_RunPaser(S_COMMAND_PARSER *parser,uint8_t data);

#endif

