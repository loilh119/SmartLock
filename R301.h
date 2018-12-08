
#ifdef __cplusplus
extern "C"
{
#endif
/********** Include section ***************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h> 
#include "app_uart.h"
#include "pca10040.h"
#include "nrf_drv_timer.h"

/********** Constant and compile switch definition section *******************/
/*
	 01H Command packet
	 02H Data packet
	 07H Acknowledge packet
	 08H End of Data packet
*/
typedef enum
{
	COMMAND = 0x01,
	DATA		= 0x02,
	ACK			= 0x07,
	END			= 0x08
}PID;
typedef enum
{
	VfyPwd  	    = 0x13,
	SetPwd        = 0x12,
	SetAdder      = 0x15,
	SetSysPara    = 0x0E,
	Control		    = 0x17,
	ReadSysPara   = 0x0F,
	TempleteNum   = 0x1D,
	GenImg			  = 0x01,
	UpImage			  = 0x0A,
	DownImage		  = 0x0B,
	Img2Tz			  = 0x02,
	RegModel		  = 0x05,
	UpChar        = 0x08,
	DownChar   	  = 0x09,
	Store					= 0x06,
	LoadChar  	  = 0x07,
	DeletChar   	= 0x0C,
	Empty         = 0x0D,
	Match					= 0x03,
	Search 				= 0x04,
	GetRandomCode = 0x14,
	WriteNotepad  = 0x18,
	ReadNotepad   = 0x19
}InstrucCode;
typedef enum
{
	CharBuffer1 = 0x01,
	CharBuffer2 = 0x02
}BufferID;
static uint8_t CollectFinger[12] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,COMMAND,0x00,0x03,GenImg,0x00,0x05};
static uint8_t GenerateChar1[13] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,COMMAND,0x00,0x04,Img2Tz,CharBuffer1,0x00,0x08};
static uint8_t GenerateChar2[13] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,COMMAND,0x00,0x04,Img2Tz,CharBuffer2,0x00,0x09};
static uint8_t MatchChar[12]     = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,COMMAND,0x00,0x03,Match,0x00,0x07};
static uint8_t GenerateTemp[12]  = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,COMMAND,0x00,0x03,RegModel,0x00,0x09};
static uint8_t StoreTemp1[15]    = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,COMMAND,0x00,0x06,Store,CharBuffer1,0x00,0x01,0x00,0x0F};
static uint8_t StoreTemp2[15]    = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,COMMAND,0x00,0x06,Store,CharBuffer2,0x00,0x01,0x00,0x10};
static uint8_t SearchLibrary1[17]= {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,COMMAND,0x00,0x08,Search,CharBuffer1,0x00,0x00,0x00,0x00,0x00,0x00};
static uint8_t SearchLibrary2[17]= {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,COMMAND,0x00,0x08,Search,CharBuffer2,0x00,0x00,0x00,0x00,0x00,0x00};
static uint8_t LibEmpty[12]      = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,COMMAND,0x00,0x03,Empty,0x00,0x11};
/********** Type definition section *******************************************/
/********** Macro definition section*******************************************/
/********** Function declaration section **************************************/

void timer_finger_init(void);
void start_finger_timer(void);

int test_timer(void);



uint8_t handler_uart(void);
void reset_buffer(void);
void delay(int time);
void uart_send(const uint8_t *str,int length);
void init_timer_uart(void);
void Collect_Finger(void);
void Generate_Character(uint8_t CharID);
void Match_Character(void);
void Generate_Template(void);
void Store_Template(uint8_t CharID, uint8_t Page_ID);
void Search_Finger(uint8_t CharID, uint8_t Page_Num);
void Library_Empty(void);
int check_sum(int byte_num, int byte);
#ifdef __cplusplus
}
#endif

