/*
 * main-app.c
 *
 *  Created on: Feb 18, 2023
 *      Author:
 */
#include "stm32f1xx_hal.h"
#include "string.h"
#include "ps2.h"
#include "dwt_stm32_delay.h"
#include "ssd1306.h"
#include "fonts.h"
#include "fifo.h"
#include "main-app.h"

#define LED_PC13_Pin GPIO_PIN_13
#define LED_PC13_GPIO_Port GPIOC

#define INPUT_12_Pin GPIO_PIN_14
#define INPUT_12_GPIO_Port GPIOC
#define INPUT_11_Pin GPIO_PIN_15
#define INPUT_11_GPIO_Port GPIOC
#define INPUT_10_Pin GPIO_PIN_0
#define INPUT_10_GPIO_Port GPIOA
#define INPUT_9_Pin GPIO_PIN_1
#define INPUT_9_GPIO_Port GPIOA
#define INPUT_8_Pin GPIO_PIN_2
#define INPUT_8_GPIO_Port GPIOA
#define INPUT_7_Pin GPIO_PIN_3
#define INPUT_7_GPIO_Port GPIOA
#define INPUT_6_Pin GPIO_PIN_4
#define INPUT_6_GPIO_Port GPIOA
#define INPUT_5_Pin GPIO_PIN_5
#define INPUT_5_GPIO_Port GPIOA
#define INPUT_4_Pin GPIO_PIN_6
#define INPUT_4_GPIO_Port GPIOA
#define INPUT_3_Pin GPIO_PIN_7
#define INPUT_3_GPIO_Port GPIOA
#define INPUT_2_Pin GPIO_PIN_0
#define INPUT_2_GPIO_Port GPIOB
#define INPUT_1_Pin GPIO_PIN_1
#define INPUT_1_GPIO_Port GPIOB
#define INPUT_13_Pin GPIO_PIN_9
#define INPUT_13_GPIO_Port GPIOB

#define MAX_BUFF						256
#define SIZE_OF_PAYLOAD 		6
#define TIME_POLL_PES				30	//ms
#define TIME_POLL_MONITOR		100	//ms

#define TIME_BOUND					5
#define TIME_SCAN_BUTTON		10 //ms

 #define USE_DEBUG_IN_ONE_BOARD

extern UART_HandleTypeDef huart3;

static fifo_t pesFifo;

uint8_t exButtonVal = 0xFF;

uint8_t exButtonVal_H = 0xFF;

uint8_t pPesBuff[MAX_BUFF];

uint8_t u8_Recv;

static uint16_t digitalDebug = 0xFFFFu;
static uint8_t analogDebug = 0xFFu;

uint8_t g_debugFlag = 0;
pes_button_p debug_pesbutton = (pes_button_p)&digitalDebug;
pes_joyStick_p debug_pesJoyStick = (pes_joyStick_p)&analogDebug;

//static void Debug_SendStr(char *pString);

static uint32_t getTimeElapse(uint32_t timeStart);

static void Debug_ButtonPrintfState(void);

static void pollFifo(void);

static void monitorInit();

static uint32_t getTimeElapse(uint32_t timeStart){
	uint32_t curTime = HAL_GetTick();
	if(curTime > timeStart){
		return (HAL_GetTick() - timeStart);
	}
	return ((0xFFFFFFFF - timeStart) + curTime);

}

/*
 * use for debug in uart2( tx is PA2)
 */
//static void Debug_SendStr(char *pString) {
//    while (*pString != NULL) {
//    	HAL_UART_Transmit(&huart2,(uint8_t*) pString, 1, 100);
//        pString++;
//    }
//}

static void monitorInit(){
	FifoInit(&pesFifo, pPesBuff, 1, MAX_BUFF);
	HAL_UART_Receive_IT(&huart3, &u8_Recv, 1);
}


// Frame = FEE+2col_2Line_1size_2total_string;
/*
 *
 *
 */
static void pollFifo(void){
	uint8_t size, colum, line, byTotal;
	static char data[15] = {0,};

	memset(data, 0, sizeof(data));
	for(uint8_t i = 0; i < 3; i++){
		if(FifoPopData(&pesFifo, &data[i])){
		}
		else{
			return;
		}
	}
	if(data[0] == 'F' && data[1] == 'E' && data[2] == 'E')
	{
		for(uint8_t i = 0; i < 3; i++){
			if(FifoPopData(&pesFifo, &data[i])){
			}
			else{
				return;
			}
		}
			line = (data[0]-0x30);
			byTotal = (data[1]-0x30)*10 + (data[2]-0x30);

			memset(data, 0, sizeof(data));
			for(uint8_t i = 0; i < byTotal; i++){
				if(FifoPopData(&pesFifo, &data[i])){
				}
				else{
					return;
				}
			}

			switch(line){
			case 1:
				ssd1306_SetCursor(0, 2);
				ssd1306_WriteString(data, Font_16x26, White);
				break;
			case 2:
				ssd1306_SetCursor(0, 28);
				ssd1306_WriteString(data, Font_11x18, White);
				break;
			default:
				ssd1306_SetCursor(0, 2);
				ssd1306_WriteString(data, Font_16x26, White);
				break;
			}
			ssd1306_UpdateScreen();

	}
	else if(data[0] == 'D' && data[1] == 'E' && data[2] == 'L'){
		ssd1306_Fill(Black);
		ssd1306_UpdateScreen();
	}
}

uint8_t checkBtn1 = 0;
uint8_t checkBtn2 = 0;
uint8_t checkBtn3 = 0;
uint8_t checkBtn4 = 0;
uint8_t checkBtn5 = 0;
uint8_t checkBtn6 = 0;
uint8_t checkBtn7 = 0;
uint8_t checkBtn8 = 0;

uint8_t checkBtn9 = 0;
uint8_t checkBtn10 = 0;
uint8_t checkBtn11 = 0;
uint8_t checkBtn12 = 0;
uint8_t checkBtn13 = 0;


void buttonScan(void){
	static uint32_t tickScan = 0;

	if(HAL_GetTick() - tickScan > TIME_SCAN_BUTTON){
		if(!HAL_GPIO_ReadPin(INPUT_1_GPIO_Port, INPUT_1_Pin)){
				checkBtn1++;
		}
		else{
			checkBtn1 = 0;
		}

		if(!HAL_GPIO_ReadPin(INPUT_2_GPIO_Port, INPUT_2_Pin)){
				checkBtn2++;
		}
		else{
			checkBtn2 = 0;
		}

		if(!HAL_GPIO_ReadPin(INPUT_3_GPIO_Port, INPUT_3_Pin)){
				checkBtn3++;
		}
		else{
			checkBtn3 = 0;
		}

		if(!HAL_GPIO_ReadPin(INPUT_4_GPIO_Port, INPUT_4_Pin)){
				checkBtn4++;
		}
		else{
			checkBtn4 = 0;
		}

		if(!HAL_GPIO_ReadPin(INPUT_5_GPIO_Port, INPUT_5_Pin)){
				checkBtn5++;
		}
		else{
			checkBtn5 = 0;
		}

		if(!HAL_GPIO_ReadPin(INPUT_6_GPIO_Port, INPUT_6_Pin)){
				checkBtn6++;
		}
		else{
			checkBtn6 = 0;
		}

		if(!HAL_GPIO_ReadPin(INPUT_7_GPIO_Port, INPUT_7_Pin)){
				checkBtn7++;
		}
		else{
			checkBtn7 = 0;
		}

		if(!HAL_GPIO_ReadPin(INPUT_8_GPIO_Port, INPUT_8_Pin)){
				checkBtn8++;
		}
		else{
			checkBtn8 = 0;
		}

		if(!HAL_GPIO_ReadPin(INPUT_9_GPIO_Port, INPUT_9_Pin)){
				checkBtn9++;
		}
		else{
			checkBtn9 = 0;
		}

		if(!HAL_GPIO_ReadPin(INPUT_10_GPIO_Port, INPUT_10_Pin)){
				checkBtn10++;
		}
		else{
			checkBtn10 = 0;
		}

		if(!HAL_GPIO_ReadPin(INPUT_11_GPIO_Port, INPUT_11_Pin)){
				checkBtn11++;
		}
		else{
			checkBtn11 = 0;
		}

		if(!HAL_GPIO_ReadPin(INPUT_12_GPIO_Port, INPUT_12_Pin)){
				checkBtn12++;
		}
		else{
			checkBtn12 = 0;
		}

		if(!HAL_GPIO_ReadPin(INPUT_13_GPIO_Port, INPUT_13_Pin)){
				checkBtn13++;
		}
		else{
			checkBtn13 = 0;
		}


		// Set logic Button1
		if(checkBtn1 > TIME_BOUND){
			exButtonVal &= ~(1 << 0);
		}
		else{
			exButtonVal |= (1 << 0);
		}

		// Set logic Button2
		if(checkBtn2 > TIME_BOUND){
			exButtonVal &= ~(1 << 1);
		}
		else{
			exButtonVal |= (1 << 1);
		}

		// Set logic Button3
		if(checkBtn3 > TIME_BOUND){
			exButtonVal &= ~(1 << 2);
		}
		else{
			exButtonVal |= (1 << 2);
		}

		// Set logic Button4
		if(checkBtn4 > TIME_BOUND){
			exButtonVal &= ~(1 << 3);
		}
		else{
			exButtonVal |= (1 << 3);
		}

		// Set logic Button5
		if(checkBtn5 > TIME_BOUND){
			exButtonVal &= ~(1 << 4);
		}
		else{
			exButtonVal |= (1 << 4);
		}

		// Set logic Button6
		if(checkBtn6 > TIME_BOUND){
			exButtonVal &= ~(1 << 5);
		}
		else{
			exButtonVal |= (1 << 5);
		}

		// Set logic Button7
		if(checkBtn7 > TIME_BOUND){
			exButtonVal &= ~(1 << 6);
		}
		else{
			exButtonVal |= (1 << 6);
		}

		// Set logic Button8
		if(checkBtn8 > TIME_BOUND){
			exButtonVal &= ~(1 << 7);
		}
		else{
			exButtonVal |= (1 << 7);
		}


		// Set logic Button9
		if(checkBtn9 > TIME_BOUND){
			exButtonVal_H &= ~(1);
		}
		else{
			exButtonVal_H |= (1);
		}

		// Set logic Button10
		if(checkBtn10 > TIME_BOUND){
			exButtonVal_H &= ~(1 << 2);
		}
		else{
			exButtonVal_H |= (1 << 2);
		}

		// Set logic Button11
		if(checkBtn11 > TIME_BOUND){
			exButtonVal_H &= ~(1 << 3);
		}
		else{
			exButtonVal_H |= (1 << 3);
		}

		// Set logic Button12
		if(checkBtn12 > TIME_BOUND){
			exButtonVal_H &= ~(1 << 4);
		}
		else{
			exButtonVal_H |= (1 << 4);
		}

		// Set logic Button13
		if(checkBtn13 > TIME_BOUND){
			exButtonVal_H &= ~(1 << 5);
		}
		else{
			exButtonVal_H |= (1 << 5);
		}
	}
}

/*
 * Main Init
 */
void main_Init(void){
	DWT_Delay_Init();

	ssd1306_Init();

	ssd1306_SetCursor(15, 15);
	ssd1306_WriteString("Power On!", Font_11x18, White);
	ssd1306_UpdateScreen();
	monitorInit();
	HAL_Delay(500);
	ssd1306_Fill(Black);
	
//	ssd1306_SetCursor(0, 2);
//	ssd1306_WriteString("100_100", Font_16x26, White);
//	ssd1306_SetCursor(0, 28);
//	ssd1306_WriteString("Line2_Test", Font_11x18, White);
//	ssd1306_UpdateScreen();
	ssd1306_UpdateScreen();
	/* Enable mode analog*/
	ps2_EnableAnalogMode();

//	Debug_SendStr("Main Init! \n");
}

/*
 * Tần số SysCoreClock nên để 8Mhz như đã cấu hình, Cấu hình cao dễ gây nhiễu.
 * Đã Cấu hình I watchdog (500ms) để tránh việc treo MCU -> Không nên bỏ.
 */
void main_process(void){
	static uint32_t ticksPollPes = 0;
	static uint32_t ticksPollMonitor = 0;

	uint8_t PES[6] = {0, };
	uint8_t analogValue = 0xFF;
	uint8_t payload[SIZE_OF_PAYLOAD] = {0, };

	buttonScan();

	if(getTimeElapse(ticksPollPes) >= TIME_POLL_PES){
		getPesRawData(PES);

		analogValue = getPesAnalog(PES);

		payload[0] = 'F';			/* Header bytee */
		payload[1] = PES[0];		/* Low byte digital data */
		payload[2] = PES[1];		/* High byte digital data */
		payload[3] = exButtonVal;	/* Button 1 to 8*/
		payload[4] = exButtonVal_H; /* Buton 9 to 13 */
		payload[5] = analogValue;	/* Analog data */
		
		HAL_UART_Transmit(&huart3, payload, SIZE_OF_PAYLOAD, 200);
		
	#ifdef USE_DEBUG_IN_ONE_BOARD
		digitalDebug = (uint16_t)(payload[1] << 8 | payload[2]);
		analogDebug = analogValue;
		g_debugFlag = 1;
	#endif

		ticksPollPes = HAL_GetTick();
	}

	if(getTimeElapse(ticksPollMonitor) >= TIME_POLL_MONITOR){
		pollFifo();
		ticksPollMonitor = HAL_GetTick();
	}
	if(g_debugFlag){
		Debug_ButtonPrintfState();
		g_debugFlag = 0;
	}
}

/*
 * used for debug button in uart2
 */
static void Debug_ButtonPrintfState(void){

	static uint8_t lastState = 0;
	if(!debug_pesbutton->Up){
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState!=1 ){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("     UP", Font_7x10, White);
			ssd1306_UpdateScreen();
		}
		lastState = 1;
	}
	else if(!debug_pesbutton->Down){
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 2){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("   DOWN", Font_7x10, White);
			ssd1306_UpdateScreen();
		}

		lastState = 2;
	}
	else if(!debug_pesbutton->Right){
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 3){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("  RIGHT", Font_7x10, White);
			ssd1306_UpdateScreen();
		}

		lastState = 3;
	}
	else if(!debug_pesbutton->Left){
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 4){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("   LEFT", Font_7x10, White);
			ssd1306_UpdateScreen();
		}
		lastState = 4;
	}
	else if(!debug_pesbutton->L1){
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 5){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("     L1", Font_7x10, White);
			ssd1306_UpdateScreen();
		}
		lastState = 5;
	}
	else if(!debug_pesbutton->L2){
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 6){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("     L2", Font_7x10, White);
			ssd1306_UpdateScreen();
		}
		lastState = 6;
	}
	else if(!debug_pesbutton->R1){
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 7){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("     R1", Font_7x10, White);
			ssd1306_UpdateScreen();
		}
		lastState = 7;
	}
	else if(!debug_pesbutton->R2){
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 8){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("     R2", Font_7x10, White);
			ssd1306_UpdateScreen();
		}

		lastState = 8;
	}
	else if(!debug_pesbutton->Select){
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 9){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString(" SELECT", Font_7x10, White);
			ssd1306_UpdateScreen();
		}

		lastState = 9;
	}
	else if(!debug_pesbutton->Start){
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 10){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("  START", Font_7x10, White);
			ssd1306_UpdateScreen();
		}
		lastState = 10;
	}
	else if(!debug_pesbutton->Tron){
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 11){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("   TRON", Font_7x10, White);
			ssd1306_UpdateScreen();
		}

		lastState = 11;
	}
	else if(!debug_pesbutton->Vuong){
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 12){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("  VUONG", Font_7x10, White);
			ssd1306_UpdateScreen();
		}

		lastState = 12;
	}
	else if(!debug_pesbutton->Tamgiac){
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 13){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("TAMGIAC", Font_7x10, White);
			ssd1306_UpdateScreen();
		}
		lastState = 13;
	}
	else if(!debug_pesbutton->X){
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
		if( lastState != 14){
			ssd1306_SetCursor(75, 53);
			ssd1306_WriteString("      X", Font_7x10, White);
			ssd1306_UpdateScreen();
		}
		lastState = 14;
	}
	else if(exButtonVal != 0xFF || exButtonVal_H != 0xFF){
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 1);
	}
	else{
		HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, 0);
	}
}


/*
 * Callback trên khối truyền
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3)
	{
		FifoPushData(&pesFifo, &u8_Recv);
		HAL_UART_Receive_IT(&huart3, &u8_Recv, 1);
	}
}
