#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h> 
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "nrf_gpio.h"
#include "R301.h"
#include "nrf_drv_clock.h"
#include "app_timer.h"

struct Struct
{
		uint8_t uart_buffer[16];
		uint8_t index;
}buffer_uart_receive;

/*----------------------------------------------------------------------------
 *  uart_send(const char *str)
 *	Descriptions:	Send string via Uart
 *	input:	String
 * 	output:	none
 *	return:	none
 *---------------------------------------------------------------------------*/
void uart_send(const uint8_t *str,int length)
{
		uint8_t i = 0;
		for(i=0 ; i < length; i++)
		{
				while(app_uart_put(str[i]) != NRF_SUCCESS);		
		}
}
/*----------------------------------------------------------------------------
 *  handler_uart()
 *	Descriptions:	UART handler receive data
 *	input:	none
 * 	output:	none
 *	return:	cr (1 byte receive data)
 *---------------------------------------------------------------------------*/
uint8_t handler_uart()
{
		uint8_t cr;
		UNUSED_VARIABLE(app_uart_get(&cr));
		buffer_uart_receive.uart_buffer[buffer_uart_receive.index] = cr;
		buffer_uart_receive.index++;
		return cr;
}

/*----------------------------------------------------------------------------
 *  check_sum(int byte_num, int byte)
 *	Descriptions:	caculate check sum
 *	input:	int byte_num, int byte
 * 	output:	none
 *	return:	0x00 if success, another if error
 *---------------------------------------------------------------------------*/
int check_sum(int byte_num, int byte)
{
		uint8_t temp = 0;
		uint16_t check = (buffer_uart_receive.uart_buffer[byte-2] << 2) + buffer_uart_receive.uart_buffer[byte-1];

		for(int i = 0; i < byte_num; i ++)
		{
			temp += buffer_uart_receive.uart_buffer[i+6];
		}
		
		if(temp == check && check != 0x00)
		{
			return buffer_uart_receive.uart_buffer[9];
		}
		else
		{
			return -1;
		}
}

/*----------------------------------------------------------------------------
 *  reset_buffer()
 *	Descriptions:	reset UART buffer
 *	input:	none
 * 	output:	none
 *	return:	none
 *---------------------------------------------------------------------------*/
void reset_buffer()
{
		for(int i = 0; i < 16; i++)
		{
			buffer_uart_receive.uart_buffer[i] = 0x00;
		}
		buffer_uart_receive.index = 0;
}

/*----------------------------------------------------------------------------
 *  delay(int time)
 *	Descriptions:	delay time(ms)
 *	input:	int time
 * 	output:	none
 *	return:	none
 *---------------------------------------------------------------------------*/
void delay(int time)
{
		nrf_delay_ms(time);
}

/*----------------------------------------------------------------------------
 *  Collect_Finger()
 *	Descriptions:	Collect Finger
 *	input:	none
 * 	output:	none
 *	return:	none
 *---------------------------------------------------------------------------*/
void Collect_Finger()
{
		uart_send(CollectFinger,12);
}

/*----------------------------------------------------------------------------
 *  Generate_Character(uint8_t CharID)
 *	Descriptions:	Generate Character
 *	input:	uint8_t CharID (1,2)
 * 	output:	none
 *	return:	none
 *---------------------------------------------------------------------------*/
void Generate_Character(uint8_t CharID)
{
		if(CharID == 1)
		{
			uart_send(GenerateChar1,13);
		}
		else if(CharID == 2)
		{
			uart_send(GenerateChar2,13);
		}
}

/*----------------------------------------------------------------------------
 *  Match_Character()
 *	Descriptions:	Check 2 Character Match 
 *	input:	none
 * 	output:	none
 *	return:	none
 *---------------------------------------------------------------------------*/
void Match_Character()
{
		uart_send(MatchChar,12);
}

/*----------------------------------------------------------------------------
 *  Generate_Template()
 *	Descriptions:	Generate Template
 *	input:	none
 * 	output:	none
 *	return:	none
 *---------------------------------------------------------------------------*/
void Generate_Template()
{
		uart_send(GenerateTemp,12);
}

/*----------------------------------------------------------------------------
 *  Store_Template(uint8_t CharID, uint8_t Page_ID)
 *	Descriptions:	Store Template
 *	input:	uint8_t CharID, uint8_t Page_ID
 * 	output:	none
 *	return:	none
 *---------------------------------------------------------------------------*/
void Store_Template(uint8_t CharID, uint8_t Page_ID)
{
		StoreTemp1[12] = Page_ID;
		if(CharID == 1)
		{
			uart_send(StoreTemp1,15);
		}
		else if(CharID == 2)
		{
			uart_send(StoreTemp2,15);
		}
}

/*----------------------------------------------------------------------------
 *  Search_Finger(uint8_t CharID, uint8_t Page_Num)
 *	Descriptions:	Search Finger
 *	input:	uint8_t CharID, uint8_t Page_Num
 * 	output:	none
 *	return:	none
 *---------------------------------------------------------------------------*/
void Search_Finger(uint8_t CharID, uint8_t Page_Num)
{
		SearchLibrary1[14] = Page_Num;
		SearchLibrary2[14] = Page_Num;
		SearchLibrary1[16] = 0x0E + Page_Num ;
		SearchLibrary2[16] = 0x0F + Page_Num ;
		if(CharID == 1)
		{
			uart_send(SearchLibrary1,17);
		}
		else if(CharID == 2)	
		{
			uart_send(SearchLibrary2,17);
		}
}

/*----------------------------------------------------------------------------
 *  Library_Empty()
 *	Descriptions:	Delete all finger in library
 *	input:	none
 * 	output:	none
 *	return:	none
 *---------------------------------------------------------------------------*/
void Library_Empty()
{
		uart_send(LibEmpty,12);
}


