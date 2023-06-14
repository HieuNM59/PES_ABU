/*
 * ps2.c
 *
 *  Created on: Feb 18, 2023
 *      Author:
 */

#include "dwt_stm32_delay.h"
#include "ps2.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"

#define PES_CLK_Pin GPIO_PIN_12
#define PES_CLK_GPIO_Port GPIOB
#define PES_ATT_Pin GPIO_PIN_13
#define PES_ATT_GPIO_Port GPIOB
#define PES_CMD_Pin GPIO_PIN_14
#define PES_CMD_GPIO_Port GPIOB
#define PES_DATA_Pin GPIO_PIN_15
#define PES_DATA_GPIO_Port GPIOB

/* Structure ---------------------------------------------------------*/

/* Public variables -------------------------------------------------*/
UART_HandleTypeDef* UartTransmit;
uint8_t PS2_POLL_ARR[]	= {0x1, 0x42, 0x00, 0x00, 0x0};
uint8_t PS2_CONFIG_MODE[5] = {0x01, 0x43, 0x00, 0x01, 0x00};
uint8_t PS2_ANALOG_MODE[9] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
uint8_t PS2_EXIT_CONFIG[9] = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};


pes_button_p pesButton;
pes_joyStick_p pesJoyStick;

unsigned char access(unsigned int tbyte){
	unsigned char rbyte = 0, tempp = 0;
	unsigned int j;
	HAL_GPIO_WritePin(PES_CLK_GPIO_Port, PES_CLK_Pin, GPIO_PIN_SET);
	for(j=0; j<8; j++)
	{
		HAL_GPIO_WritePin(PES_CMD_GPIO_Port, PES_CMD_Pin, tbyte&0x01);
		DWT_Delay_us(10);
		HAL_GPIO_WritePin(PES_CLK_GPIO_Port, PES_CLK_Pin, GPIO_PIN_RESET);
		DWT_Delay_us(10);
		tempp=HAL_GPIO_ReadPin(PES_DATA_GPIO_Port, PES_DATA_Pin);
		rbyte = (rbyte >> 1) | (tempp << 7);
		HAL_GPIO_WritePin(PES_CLK_GPIO_Port, PES_CLK_Pin, GPIO_PIN_SET);
		tbyte = tbyte >> 1;
	}
	DWT_Delay_us(20);
	return rbyte;
}

void ps2_SendConfig(uint8_t* cmd, uint8_t length)
{
	int i;
	HAL_GPIO_WritePin(PES_ATT_GPIO_Port,PES_ATT_Pin,GPIO_PIN_RESET);
	DWT_Delay_us(16);
	for(i = 0 ; i < length ; i++){
		access(cmd[i]);
	}
	DWT_Delay_us(16);
	HAL_GPIO_WritePin(PES_ATT_GPIO_Port,PES_ATT_Pin,GPIO_PIN_SET);
}
/*
 * Bật mode analog trên tay pes
 */
void ps2_EnableAnalogMode(void){
	ps2_SendConfig(PS2_POLL_ARR, sizeof(PS2_POLL_ARR));
	ps2_SendConfig(PS2_POLL_ARR, sizeof(PS2_POLL_ARR));
	ps2_SendConfig(PS2_POLL_ARR, sizeof(PS2_POLL_ARR));

	ps2_SendConfig(PS2_CONFIG_MODE, sizeof(PS2_CONFIG_MODE));
	ps2_SendConfig(PS2_ANALOG_MODE, sizeof(PS2_ANALOG_MODE));
	ps2_SendConfig(PS2_EXIT_CONFIG, sizeof(PS2_EXIT_CONFIG));
}
/**
 * Get digital datain transmit module
 */
void getPesRawData(uint8_t *pData){
	uint8_t analog = 128;
	HAL_GPIO_WritePin(PES_ATT_GPIO_Port,PES_ATT_Pin,GPIO_PIN_RESET);
	access(0x01);
	analog = access(0x42);
	access(0);
	pData[0] = access(0);
	pData[1] = access(0);
	if(analog == 0x73 || analog == 0x23 || analog == 0x53)
	{
		pData[2] = access(0);
		pData[3] = access(0);
		pData[4] = access(0);
		pData[5] = access(0);
	}
	if(analog == 0x12)
	{
		pData[2] = access(0);
		pData[3] = access(0);
	}
	HAL_GPIO_WritePin(PES_ATT_GPIO_Port,PES_ATT_Pin,GPIO_PIN_SET);
}

/*
 *  Get analog in Transmit module
 */
uint8_t getPesAnalog(uint8_t *pesRawData){
	uint8_t analog;

	/* Analog Left Y Axis */
	analog = 0xff;
	if(pesRawData[5] > 200)
		analog = analog & 0xfe;
	else if(pesRawData[5] < 50)
		analog = analog & 0xfd;
	else
		analog = analog | 0x03;

	/* Analog Left X Axis */
	if(pesRawData[4] > 200)
		analog = analog & 0xfb;
	else if(pesRawData[4] < 50)
		analog = analog & 0xf7;
	else
        analog = analog | 0x0c;

	/* Analog Right Y Axis */
	if(pesRawData[3] > 200)
		analog = analog & 0xef;
	else if(pesRawData[3] < 50)
		analog = analog & 0xdf;
	else
		analog = analog | 0x30;

	/* Analog Right X Axis */
	if(pesRawData[2] > 200)
		analog = analog & 0xbf;
	else if(pesRawData[2] < 50)
		analog=analog & 0x7f;
	else
		analog=analog | 0xc0;
	return analog;
}

void pes_monitor_init(UART_HandleTypeDef* pUart){
	UartTransmit = pUart;
}

void pesPrintf(uint8_t line, const char* pString, ...){
	
	NVIC_DisableIRQ(USART3_IRQn);
	uint8_t frame[20] = {0, };
	uint8_t StrBuff[15] = {0, };
	sprintf(&frame[0], "FEE%d%.2d",line, strlen(pString));
	va_list ap;
	va_start(ap, pString);
	vsnprintf(&StrBuff[0], sizeof(StrBuff), pString, ap);
	va_end(ap);

	memcpy(&frame[6], StrBuff, strlen(pString));
	HAL_UART_Transmit(UartTransmit, frame, 11 + strlen(pString), 1000);
	NVIC_EnableIRQ(USART3_IRQn);
	
}

void pesMonitorClear(void){
	uint8_t frame[3] = {'D', 'E', 'L'};
	HAL_UART_Transmit(UartTransmit, frame, sizeof(frame), 200);
}