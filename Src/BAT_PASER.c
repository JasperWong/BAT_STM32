
#include "BAT_PARSER.H"




/*


*/

bool_x PARSER_IsParseFinish(S_COMMAND_PARSER *parser)
{
		if( STATE_PARSE_FINISH_PENDING == parser->status )
		{
				return true_x;
		}
		
		return false_x;
}


void PARSER_Reset(S_COMMAND_PARSER *parser)
{
		parser->status = STATE_WAIT_P;
}



void PARSER_RunPaser(S_COMMAND_PARSER *parser,uint8_t data)
{
	switch( parser->status )
	{
		case STATE_WAIT_P:
		{
				if( data == 'P')
				{
						parser->status = STATE_WAIT_X;
				}
				break; 
		}
		case STATE_WAIT_X:
		{
				if( data == 'X')
				{
					parser->status = STATE_WAIT_REG_ADDR_H;
				}
				else
				{
					parser->status = STATE_WAIT_P;
				}
				break;
		}
		case STATE_WAIT_REG_ADDR_H:
		{
				parser->regAddrH = data;
				parser->status = STATE_WAIT_REG_ADDR_L;
				break;
		}
		case STATE_WAIT_REG_ADDR_L:
		{
				parser->regAddrL = data;
				parser->status = STATE_WAIT_REG_DATA_1;
				break;
		}
		case STATE_WAIT_REG_DATA_1:
		{
				parser->regData[0] = data;
				parser->status = STATE_WAIT_REG_DATA_2;
				break;
		}
		case STATE_WAIT_REG_DATA_2:
		{
				parser->regData[1] = data;
				parser->status = STATE_WAIT_REG_DATA_3;
				break;
		}
		case STATE_WAIT_REG_DATA_3:
		{
				parser->regData[2] = data;
				parser->status = STATE_WAIT_REG_DATA_4;
				break;
		}
		case STATE_WAIT_REG_DATA_4:
		{
				parser->regData[3] = data;
				parser->status = STATE_WAIT_CHECKSUM;
				break;
		}
		case STATE_WAIT_CHECKSUM:
		{
				parser->checksum = data;
				
				//Disable checksum first
				parser->status = STATE_WAIT_E;
			
				break;
		}
		case STATE_WAIT_E:
		{
				if( data == 'E')
				{
					parser->status = STATE_PARSE_FINISH_PENDING;
				}
				else
				{
					parser->status = STATE_WAIT_P;
				}
				break;
		}
		case STATE_PARSE_FINISH_PENDING:
		{
				break;
		}
		default:
		{
				//Impossible to get here
				parser->status = STATE_WAIT_P;
		}
	}
}
 

