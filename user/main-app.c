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

#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC

#define IN1_1_Pin GPIO_PIN_2
#define IN1_1_GPIO_Port GPIOB
#define IN1_2_Pin GPIO_PIN_3
#define IN1_2_GPIO_Port GPIOB
#define IN1_3_Pin GPIO_PIN_4
#define IN1_3_GPIO_Port GPIOB
#define IN1_4_Pin GPIO_PIN_5
#define IN1_4_GPIO_Port GPIOB
#define IN1_5_Pin GPIO_PIN_8
#define IN1_5_GPIO_Port GPIOB
#define IN1_6_Pin GPIO_PIN_9
#define IN1_6_GPIO_Port GPIOB
#define IN1_7_Pin GPIO_PIN_14
#define IN1_7_GPIO_Port GPIOC
#define IN1_8_Pin GPIO_PIN_15
#define IN1_8_GPIO_Port GPIOC

#define IN3_1_Pin GPIO_PIN_0
#define IN3_1_GPIO_Port GPIOA
#define IN3_2_Pin GPIO_PIN_1
#define IN3_2_GPIO_Port GPIOA
#define IN3_3_Pin GPIO_PIN_2
#define IN3_3_GPIO_Port GPIOA
#define IN3_4_Pin GPIO_PIN_3
#define IN3_4_GPIO_Port GPIOA
#define IN3_5_Pin GPIO_PIN_4
#define IN3_5_GPIO_Port GPIOA
#define IN3_6_Pin GPIO_PIN_5
#define IN3_6_GPIO_Port GPIOA
#define IN3_7_Pin GPIO_PIN_6
#define IN3_7_GPIO_Port GPIOA
#define IN3_8_Pin GPIO_PIN_7
#define IN3_8_GPIO_Port GPIOA

#define IN2_1_Pin GPIO_PIN_0
#define IN2_1_GPIO_Port GPIOB
#define IN2_2_Pin GPIO_PIN_1
#define IN2_2_GPIO_Port GPIOB
#define IN2_3_Pin GPIO_PIN_8
#define IN2_3_GPIO_Port GPIOA
#define IN2_4_Pin GPIO_PIN_9
#define IN2_4_GPIO_Port GPIOA
#define IN2_5_Pin GPIO_PIN_10
#define IN2_5_GPIO_Port GPIOA
#define IN2_6_Pin GPIO_PIN_11
#define IN2_6_GPIO_Port GPIOA
#define IN2_7_Pin GPIO_PIN_12
#define IN2_7_GPIO_Port GPIOA
#define IN2_8_Pin GPIO_PIN_15
#define IN2_8_GPIO_Port GPIOA

#define MAX_BUFF						256
#define SIZE_OF_PAYLOAD 		8
#define TIME_POLL_PES				30	//ms
#define TIME_POLL_MONITOR		100	//ms

#define TIME_BOUND					5
#define TIME_SCAN_BUTTON		10 //ms

 #define USE_DEBUG_IN_ONE_BOARD

extern UART_HandleTypeDef huart3;

static fifo_t pesFifo;

uint8_t exButton_P1 = 0xFF;
uint8_t exButton_P2 = 0xFF;
uint8_t exButton_P3 = 0xFF;

uint8_t pPesBuff[MAX_BUFF];

uint8_t u8_Recv;

static uint16_t digitalDebug = 0xFFFFu;
static uint8_t analogDebug = 0xFFu;

uint8_t g_debugFlag = 0;
pes_button_p debug_pesbutton = (pes_button_p)&digitalDebug;
pes_joyStick_p debug_pesJoyStick = (pes_joyStick_p)&analogDebug;

static void Debug_ButtonPrintfState(void);

static void pollFifo(void);

static void monitorInit();

static void monitorInit(){
	FifoInit(&pesFifo, pPesBuff, 1, MAX_BUFF);
	HAL_UART_Receive_IT(&huart3, &u8_Recv, 1);
}

// Frame = FEE+2col_2Line_1size_2total_string;
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

void scanInput1(void){
	uint8_t checkBtn1 = 0;
	uint8_t checkBtn2 = 0;
	uint8_t checkBtn3 = 0;
	uint8_t checkBtn4 = 0;
	uint8_t checkBtn5 = 0;
	uint8_t checkBtn6 = 0;
	uint8_t checkBtn7 = 0;
	uint8_t checkBtn8 = 0;

	if(!HAL_GPIO_ReadPin(IN1_1_GPIO_Port, IN1_1_Pin)){
			checkBtn1++;
	}
	else{
		checkBtn1 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_2_GPIO_Port, IN1_2_Pin)){
			checkBtn2++;
	}
	else{
		checkBtn2 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_3_GPIO_Port, IN1_3_Pin)){
			checkBtn3++;
	}
	else{
		checkBtn3 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_4_GPIO_Port, IN1_4_Pin)){
			checkBtn4++;
	}
	else{
		checkBtn4 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_5_GPIO_Port, IN1_5_Pin)){
			checkBtn5++;
	}
	else{
		checkBtn5 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_6_GPIO_Port, IN1_6_Pin)){
			checkBtn6++;
	}
	else{
		checkBtn6 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_7_GPIO_Port, IN1_7_Pin)){
			checkBtn7++;
	}
	else{
		checkBtn7 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_8_GPIO_Port, IN1_8_Pin)){
			checkBtn8++;
	}
	else{
		checkBtn8 = 0;
	}
	
// Set logic Button1
	if(checkBtn1 > TIME_BOUND){
		exButton_P1 &= ~(1 << 0);
	}
	else{
		exButton_P1 |= (1 << 0);
	}

	// Set logic Button2
	if(checkBtn2 > TIME_BOUND){
		exButton_P1 &= ~(1 << 1);
	}
	else{
		exButton_P1 |= (1 << 1);
	}

	// Set logic Button3
	if(checkBtn3 > TIME_BOUND){
		exButton_P1 &= ~(1 << 2);
	}
	else{
		exButton_P1 |= (1 << 2);
	}

	// Set logic Button4
	if(checkBtn4 > TIME_BOUND){
		exButton_P1 &= ~(1 << 3);
	}
	else{
		exButton_P1 |= (1 << 3);
	}

	// Set logic Button5
	if(checkBtn5 > TIME_BOUND){
		exButton_P1 &= ~(1 << 4);
	}
	else{
		exButton_P1 |= (1 << 4);
	}

	// Set logic Button6
	if(checkBtn6 > TIME_BOUND){
		exButton_P1 &= ~(1 << 5);
	}
	else{
		exButton_P1 |= (1 << 5);
	}

	// Set logic Button7
	if(checkBtn7 > TIME_BOUND){
		exButton_P1 &= ~(1 << 6);
	}
	else{
		exButton_P1 |= (1 << 6);
	}

	// Set logic Button8
	if(checkBtn8 > TIME_BOUND){
		exButton_P1 &= ~(1 << 7);
	}
	else{
		exButton_P1 |= (1 << 7);
	}
}


void scanInput2(void){
	uint8_t checkBtn1 = 0;
	uint8_t checkBtn2 = 0;
	uint8_t checkBtn3 = 0;
	uint8_t checkBtn4 = 0;
	uint8_t checkBtn5 = 0;
	uint8_t checkBtn6 = 0;
	uint8_t checkBtn7 = 0;
	uint8_t checkBtn8 = 0;

	if(!HAL_GPIO_ReadPin(IN1_1_GPIO_Port, IN1_1_Pin)){
			checkBtn1++;
	}
	else{
		checkBtn1 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_2_GPIO_Port, IN1_2_Pin)){
			checkBtn2++;
	}
	else{
		checkBtn2 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_3_GPIO_Port, IN1_3_Pin)){
			checkBtn3++;
	}
	else{
		checkBtn3 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_4_GPIO_Port, IN1_4_Pin)){
			checkBtn4++;
	}
	else{
		checkBtn4 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_5_GPIO_Port, IN1_5_Pin)){
			checkBtn5++;
	}
	else{
		checkBtn5 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_6_GPIO_Port, IN1_6_Pin)){
			checkBtn6++;
	}
	else{
		checkBtn6 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_7_GPIO_Port, IN1_7_Pin)){
			checkBtn7++;
	}
	else{
		checkBtn7 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_8_GPIO_Port, IN1_8_Pin)){
			checkBtn8++;
	}
	else{
		checkBtn8 = 0;
	}
	
// Set logic Button1
	if(checkBtn1 > TIME_BOUND){
		exButton_P2 &= ~(1 << 0);
	}
	else{
		exButton_P2 |= (1 << 0);
	}

	// Set logic Button2
	if(checkBtn2 > TIME_BOUND){
		exButton_P2 &= ~(1 << 1);
	}
	else{
		exButton_P2 |= (1 << 1);
	}

	// Set logic Button3
	if(checkBtn3 > TIME_BOUND){
		exButton_P2 &= ~(1 << 2);
	}
	else{
		exButton_P2 |= (1 << 2);
	}

	// Set logic Button4
	if(checkBtn4 > TIME_BOUND){
		exButton_P2 &= ~(1 << 3);
	}
	else{
		exButton_P2 |= (1 << 3);
	}

	// Set logic Button5
	if(checkBtn5 > TIME_BOUND){
		exButton_P2 &= ~(1 << 4);
	}
	else{
		exButton_P2 |= (1 << 4);
	}

	// Set logic Button6
	if(checkBtn6 > TIME_BOUND){
		exButton_P2 &= ~(1 << 5);
	}
	else{
		exButton_P2 |= (1 << 5);
	}

	// Set logic Button7
	if(checkBtn7 > TIME_BOUND){
		exButton_P2 &= ~(1 << 6);
	}
	else{
		exButton_P2 |= (1 << 6);
	}

	// Set logic Button8
	if(checkBtn8 > TIME_BOUND){
		exButton_P2 &= ~(1 << 7);
	}
	else{
		exButton_P2 |= (1 << 7);
	}
}

void scanInput3(void){
	uint8_t checkBtn1 = 0;
	uint8_t checkBtn2 = 0;
	uint8_t checkBtn3 = 0;
	uint8_t checkBtn4 = 0;
	uint8_t checkBtn5 = 0;
	uint8_t checkBtn6 = 0;
	uint8_t checkBtn7 = 0;
	uint8_t checkBtn8 = 0;

	if(!HAL_GPIO_ReadPin(IN1_1_GPIO_Port, IN1_1_Pin)){
			checkBtn1++;
	}
	else{
		checkBtn1 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_2_GPIO_Port, IN1_2_Pin)){
			checkBtn2++;
	}
	else{
		checkBtn2 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_3_GPIO_Port, IN1_3_Pin)){
			checkBtn3++;
	}
	else{
		checkBtn3 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_4_GPIO_Port, IN1_4_Pin)){
			checkBtn4++;
	}
	else{
		checkBtn4 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_5_GPIO_Port, IN1_5_Pin)){
			checkBtn5++;
	}
	else{
		checkBtn5 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_6_GPIO_Port, IN1_6_Pin)){
			checkBtn6++;
	}
	else{
		checkBtn6 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_7_GPIO_Port, IN1_7_Pin)){
			checkBtn7++;
	}
	else{
		checkBtn7 = 0;
	}

	if(!HAL_GPIO_ReadPin(IN1_8_GPIO_Port, IN1_8_Pin)){
			checkBtn8++;
	}
	else{
		checkBtn8 = 0;
	}
	
// Set logic Button1
	if(checkBtn1 > TIME_BOUND){
		exButton_P3 &= ~(1 << 0);
	}
	else{
		exButton_P3 |= (1 << 0);
	}

	// Set logic Button2
	if(checkBtn2 > TIME_BOUND){
		exButton_P3 &= ~(1 << 1);
	}
	else{
		exButton_P3 |= (1 << 1);
	}

	// Set logic Button3
	if(checkBtn3 > TIME_BOUND){
		exButton_P3 &= ~(1 << 2);
	}
	else{
		exButton_P3 |= (1 << 2);
	}

	// Set logic Button4
	if(checkBtn4 > TIME_BOUND){
		exButton_P3 &= ~(1 << 3);
	}
	else{
		exButton_P3 |= (1 << 3);
	}

	// Set logic Button5
	if(checkBtn5 > TIME_BOUND){
		exButton_P3 &= ~(1 << 4);
	}
	else{
		exButton_P3 |= (1 << 4);
	}

	// Set logic Button6
	if(checkBtn6 > TIME_BOUND){
		exButton_P3 &= ~(1 << 5);
	}
	else{
		exButton_P3 |= (1 << 5);
	}

	// Set logic Button7
	if(checkBtn7 > TIME_BOUND){
		exButton_P3 &= ~(1 << 6);
	}
	else{
		exButton_P3 |= (1 << 6);
	}

	// Set logic Button8
	if(checkBtn8 > TIME_BOUND){
		exButton_P3 &= ~(1 << 7);
	}
	else{
		exButton_P3 |= (1 << 7);
	}
}
void buttonScan(void){
	static uint32_t tickScan = 0;

	if(HAL_GetTick() - tickScan > TIME_SCAN_BUTTON){
		scanInput1();
		scanInput2();
		scanInput3();
		tickScan = HAL_GetTick();
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
	ssd1306_UpdateScreen();
	/* Enable mode analog*/
	ps2_EnableAnalogMode();
}


void main_process(void){
	static uint32_t ticksPollPes = 0;
	static uint32_t ticksPollMonitor = 0;

	uint8_t PES[8] = {0, };
	uint8_t analogValue = 0xFF;
	uint8_t payload[SIZE_OF_PAYLOAD] = {0, };

	buttonScan();

	if(HAL_GetTick() - ticksPollPes >= TIME_POLL_PES){
		getPesRawData(PES);

		analogValue = getPesAnalog(PES);

		payload[0] = 'F';			/* Header bytee */
		payload[1] = PES[0];		/* Low byte digital data */
		payload[2] = PES[1];		/* High byte digital data */
		payload[3] = analogValue;	/* Analog data */
		payload[4] = exButton_P1;	/* Button in P1 */
		payload[5] = exButton_P2; /* Button in P2 */
		payload[6] = exButton_P3; /* Button in P3 */
		payload[7] = 0xFF; 				/* Reverse */
				
		HAL_UART_Transmit(&huart3, payload, SIZE_OF_PAYLOAD, 500);
		
	#ifdef USE_DEBUG_IN_ONE_BOARD
		digitalDebug = (uint16_t)(payload[1] << 8 | payload[2]);
		analogDebug = analogValue;
		g_debugFlag = 1;
	#endif

		ticksPollPes = HAL_GetTick();
	}

	if(HAL_GetTick - ticksPollMonitor >= TIME_POLL_MONITOR){
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
	else if(exButton_P1 != 0xFF || exButton_P2 != 0xFF || exButton_P3 != 0xFF){
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
