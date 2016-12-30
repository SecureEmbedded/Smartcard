/*
 * smartcard.c
 *
 *  Created on: 21 dic 2016
 *      Author: Salvatore
 */

#include "smartcard.h"

extern SMARTCARD_HandleTypeDef hsc6;

uint8_t SLJ52_ATR[MAX_ATR_SIZE] = {0x00};
PPSType PPS_Request;
PPSType PPS_Response;
SLJ52ATR ATR_Response;
static __IO uint8_t FailedByte = 0;
SLJ52States status_protocol = CARD_UNKNOWN;
APDU_State status_apdu = IDLE;
uint32_t counter = 0,fl = 0;
uint8_t isTxReady = 0,isRxReady = 0;

uint32_t F_Table[16] = {372,372,558,744,1116,1488,1860,0,0,512,768,1024,1536,2048,0,0};
uint32_t D_Table[16] = {0,1,2,4,8,16,32,64,12,20,0,0,0,0,0,0};

void ErrorsDetect(){
    uint32_t txe = __HAL_SMARTCARD_GET_FLAG(&hsc6,SMARTCARD_FLAG_TXE);
    uint32_t tc = __HAL_SMARTCARD_GET_FLAG(&hsc6,SMARTCARD_FLAG_TC);
    uint32_t rxne = __HAL_SMARTCARD_GET_FLAG(&hsc6,SMARTCARD_FLAG_RXNE);
    uint32_t idle = __HAL_SMARTCARD_GET_FLAG(&hsc6,SMARTCARD_FLAG_IDLE);
    uint32_t overrun = __HAL_SMARTCARD_GET_FLAG(&hsc6,SMARTCARD_FLAG_ORE);
    uint32_t noise = __HAL_SMARTCARD_GET_FLAG(&hsc6,SMARTCARD_FLAG_NE);
    uint32_t frame_err = __HAL_SMARTCARD_GET_FLAG(&hsc6,SMARTCARD_FLAG_FE);
    uint32_t parity_err = __HAL_SMARTCARD_GET_FLAG(&hsc6,SMARTCARD_FLAG_PE);
    return ;
}

void SLJ52_PWR(PowerState status){
	if(status != PW_UP)
		HAL_GPIO_WritePin(SC_ON_OFF_GPIO_Port,SC_ON_OFF_Pin,GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(SC_ON_OFF_GPIO_Port,SC_ON_OFF_Pin,GPIO_PIN_SET);
}


void SLJ52_RST(ResetState rst){
	if(rst != LOW_RESET){
		HAL_GPIO_WritePin(SC_RST_GPIO_Port,SC_RST_Pin,GPIO_PIN_SET);
	}else
		HAL_GPIO_WritePin(SC_RST_GPIO_Port,SC_RST_Pin,GPIO_PIN_RESET);
}

void SLJ52_Init(){
	SLJ52_RST(LOW_RESET);
	SLJ52_PWR(PW_UP);
	SLJ52_Interface_Configuration();
	SLJ52_RST(HIGH_RESET);
	status_protocol = COLD_RESET;
	SLJ52_ATR_Parse();
	SLJ52_PPS();
}

void SLJ52_Interface_Configuration() {
	hsc6.Instance = USART6;
	hsc6.Init.BaudRate = 11290;
	hsc6.Init.WordLength = SMARTCARD_WORDLENGTH_9B;
	hsc6.Init.StopBits = SMARTCARD_STOPBITS_1_5;
	hsc6.Init.Parity = SMARTCARD_PARITY_EVEN;
	hsc6.Init.Mode = SMARTCARD_MODE_TX_RX;
	hsc6.Init.CLKPolarity = SMARTCARD_POLARITY_LOW;
	hsc6.Init.CLKPhase = SMARTCARD_PHASE_1EDGE;
	hsc6.Init.CLKLastBit = SMARTCARD_LASTBIT_ENABLE;
	hsc6.Init.Prescaler = SMARTCARD_PRESCALER_SYSCLK_DIV20;
	hsc6.Init.GuardTime = 16;
	hsc6.Init.NACKState = SMARTCARD_NACK_DISABLED;
	while(HAL_SMARTCARD_Init(&hsc6) != HAL_OK){

	}
	status_protocol = CARD_ACTIVATION;
}

void SLJ52_TransmitByte(uint8_t byte){
#if INTERRUPT_MODE
	HAL_SMARTCARD_Transmit_IT(&hsc6, &byte, 1);
	while(isTxReady == 0){

	}
	isTxReady = 0;
#else
	while(__HAL_SMARTCARD_GET_FLAG(&hsc6,SMARTCARD_FLAG_TXE)== RESET);
	HAL_SMARTCARD_Transmit(&hsc6,&byte,1,0);
	while(__HAL_SMARTCARD_GET_FLAG(&hsc6,SMARTCARD_FLAG_TC)== RESET);

#endif
}

void SLJ52_ReceiveByte(uint8_t* byte){
#if INTERRUPT_MODE
	HAL_SMARTCARD_Receive_IT(&hsc6, byte, 1);
	while(isRxReady == 0){

	}
	isRxReady = 0;
#else
	while(__HAL_SMARTCARD_GET_FLAG(&hsc6,SMARTCARD_FLAG_RXNE)== RESET);
	HAL_SMARTCARD_Receive(&hsc6,byte,1,0);
#endif
}
void SLJ52_ATR_Parse(){
	uint32_t i = 0, flag = 0, buf = 0, protocol = 0;
	for(int i = 0; i < ATR_SIZE ; i++){
		SLJ52_ReceiveByte(&SLJ52_ATR[i]);
	}
	while(__HAL_SMARTCARD_GET_FLAG(&hsc6,SMARTCARD_FLAG_IDLE)== RESET);
	if (SLJ52_ATR[0] != 0x00) {
		ATR_Response.TS = SLJ52_ATR[0];
		ATR_Response.T0 = SLJ52_ATR[1];

		ATR_Response.Hlength = ATR_Response.T0 & (uint8_t) 0x0F;

		if ((ATR_Response.T0 & (uint8_t) 0x80) == 0x80) {
			flag = 1;
		}

		for (i = 0; i < 4; i++) {
			ATR_Response.Tlength = ATR_Response.Tlength
					+ (((ATR_Response.T0 & (uint8_t) 0xF0) >> (4 + i))
							& (uint8_t) 0x1);
		}

		for (i = 0; i < ATR_Response.Tlength; i++) {
			ATR_Response.T[i] = SLJ52_ATR[i + 2];
		}

		if ((ATR_Response.T0 & (uint8_t) 0x80) == 0x00) {
			protocol = 0;
		} else {
			protocol = ATR_Response.T[ATR_Response.Tlength - 1]
					& (uint8_t) 0x0F;
		}

		while (flag) {
			if ((ATR_Response.T[ATR_Response.Tlength - 1] & (uint8_t) 0x80)
					== 0x80) {
				flag = 1;
			} else {
				flag = 0;
			}

			buf = ATR_Response.Tlength;
			ATR_Response.Tlength = 0;

			for (i = 0; i < 4; i++) {
				ATR_Response.Tlength = ATR_Response.Tlength
						+ (((ATR_Response.T[buf - 1] & (uint8_t) 0xF0)
								>> (4 + i)) & (uint8_t) 0x1);
			}

			for (i = 0; i < ATR_Response.Tlength; i++) {
				ATR_Response.T[buf + i] = SLJ52_ATR[i + 2 + buf];
			}
			ATR_Response.Tlength += (uint8_t) buf;
		}

		for (i = 0; i < ATR_Response.Hlength; i++) {
			ATR_Response.H[i] = SLJ52_ATR[i + 2 + ATR_Response.Tlength];
		}
	}
	status_protocol = ATR_DECODED;
}

void SLJ52_PPS(){
	while(__HAL_SMARTCARD_GET_FLAG(&hsc6,SMARTCARD_FLAG_IDLE) == RESET);
	uint8_t Request[4] = {0x00};

	// Define the PTS Struct
	PPS_Request.PTSS = 0xFF;
	PPS_Request.PTS0 = 0x10;
	PPS_Request.PTS1 = ATR_Response.T[0];
	PPS_Request.PCK = (uint8_t)0xFF^(uint8_t)0x10^(uint8_t)0x18;

	Request[0] = PPS_Request.PTSS;
	Request[1] = PPS_Request.PTS0;
	Request[2] = PPS_Request.PTS1;
	Request[3] = PPS_Request.PCK;

	uint8_t Response[4] = {0x00};
	uint8_t flush;
	for(int i = 0; i < 4; i++){
		SLJ52_TransmitByte(Request[i]);
		SLJ52_ReceiveByte(&flush);
    }

	for(int i = 0; i < 4; i++){
		SLJ52_ReceiveByte(&Response[i]);
	}
	Request[0] = 0xFF;

	uint8_t flag = 1;
	for(int i = 0; i < 4; i++){
		if(Request[i] != Response[i]){
			flag = 0;
		}
	}

	if(flag == 1){
		// Set tehe PPS response
		PPS_Response.PTSS = Response[0];
		PPS_Response.PTS0 = Response[1];
		PPS_Response.PTS1 = Response[2];
		PPS_Response.PCK = Response[3];

		// Calculate the F and D Values
		uint8_t F = (ATR_Response.T[0]&0xF0) >> 4;
		uint8_t D = (ATR_Response.T[0]&0x0F);

		// Obtain the prescaled frequency
		uint32_t apb2_freq = HAL_RCC_GetPCLK2Freq();
		uint32_t prescaled_freq = (apb2_freq/((USART6->GTPR)&(uint16_t)0x00FF)/2);

		// Calculate the new baudrate
		uint32_t baudrate = (prescaled_freq*D_Table[D])/F_Table[F];

		//Set the new baudarate
		hsc6.Instance = USART6;
		hsc6.Init.BaudRate = baudrate;
		hsc6.Init.WordLength = SMARTCARD_WORDLENGTH_9B;
		hsc6.Init.StopBits = SMARTCARD_STOPBITS_1_5;
		hsc6.Init.Parity = SMARTCARD_PARITY_EVEN;
		hsc6.Init.Mode = SMARTCARD_MODE_TX_RX;
		hsc6.Init.CLKPolarity = SMARTCARD_POLARITY_LOW;
		hsc6.Init.CLKPhase = SMARTCARD_PHASE_1EDGE;
		hsc6.Init.CLKLastBit = SMARTCARD_LASTBIT_ENABLE;
		hsc6.Init.Prescaler = SMARTCARD_PRESCALER_SYSCLK_DIV20;
		hsc6.Init.GuardTime = 16;
		hsc6.Init.NACKState = SMARTCARD_NACK_DISABLED;
		while(HAL_SMARTCARD_Init(&hsc6) != HAL_OK){

		}
		status_protocol = PTS_EXCHANGED;
		SendAPDU();
	}
}

void SendAPDU(){
	SLJ52_APDU APDU;
	APDU.CLA = 0x00;
	APDU.INS = 0xA4;
	APDU.P1 = 0x04;
	APDU.P2 = 0x00;
	APDU.LC = 0x0B;
	APDU.Data[0] = 0x01;
	APDU.Data[1] = 0x02;
	APDU.Data[2] = 0x03;
	APDU.Data[3] = 0x04;
	APDU.Data[4] = 0x05;
	APDU.Data[5] = 0x06;
	APDU.Data[6] = 0x07;
	APDU.Data[7] = 0x08;
	APDU.Data[8] = 0x09;
	APDU.Data[9] = 0x00;
	APDU.Data[10] = 0x00;
	APDU.LE = 0x00;

	uint8_t command[17];
	command[0] = APDU.CLA;
	command[1] = APDU.INS;
	command[2] = APDU.P1;
	command[3] = APDU.P2;
	command[4] = APDU.LC;
	command[5] = APDU.Data[0];
	command[6] = APDU.Data[1];
	command[7] = APDU.Data[2];
	command[8] = APDU.Data[3];
	command[9] = APDU.Data[4];
	command[10] = APDU.Data[5];
	command[11] = APDU.Data[6];
	command[12] = APDU.Data[7];
	command[13] = APDU.Data[8];
	command[14] = APDU.Data[9];
	command[15] = APDU.Data[10];
	command[16] = APDU.LE;
	uint8_t Response[8] = {0x00};

	uint8_t flush;
	for(int i = 0; i < 5; i++){
		SLJ52_TransmitByte(command[i]);
		SLJ52_ReceiveByte(&flush);
	}

	SLJ52_ReceiveByte(&Response[0]);

	if(Response[0] == 0x00){

	}
	else{

		for(int i = 0; i < command[4]; i++){
			SLJ52_TransmitByte(command[i+5]);
			SLJ52_ReceiveByte(&flush);
		}
		SLJ52_ReceiveByte(&Response[1]);
	}

	while(__HAL_SMARTCARD_GET_FLAG(&hsc6,SMARTCARD_FLAG_RXNE)== RESET);
	int x = 0;
}

void HAL_SMARTCARD_TxCpltCallback(SMARTCARD_HandleTypeDef* hsc){
	if(hsc->Instance == USART6){
		isTxReady = 1;
	}
}

void HAL_SMARTCARD_RxCpltCallback(SMARTCARD_HandleTypeDef* hsc){
	if(hsc->Instance == USART6){
		isRxReady = 1;
	}
}

void SC_ParityErrorHandler(void)
{
  HAL_SMARTCARD_Transmit(&hsc6, (uint8_t *)&FailedByte, 1, WORK_ETU);
}

/**
  * @brief SMARTCARD error callbacks
  * @param hsc: SMARTCARD handle
  * @retval None
  */
void HAL_SMARTCARD_ErrorCallback(SMARTCARD_HandleTypeDef *hsc)
{
  if(HAL_SMARTCARD_GetError(hsc) & HAL_SMARTCARD_ERROR_FE)
  {
    __HAL_SMARTCARD_FLUSH_DRREGISTER(hsc);
    /* Resend the byte that failed to be received (by the Smartcard) correctly */
    SC_ParityErrorHandler();
  }

  if(HAL_SMARTCARD_GetError(hsc) & HAL_SMARTCARD_ERROR_PE)
  {
    /* Enable SC_USART RXNE Interrupt (until receiving the corrupted byte) */
    __HAL_SMARTCARD_ENABLE_IT(hsc, SMARTCARD_IT_RXNE);
    /* Flush the SC_USART DR register */
    __HAL_SMARTCARD_FLUSH_DRREGISTER(hsc);
  }

  if(HAL_SMARTCARD_GetError(hsc) & HAL_SMARTCARD_ERROR_NE)
  {
    __HAL_SMARTCARD_FLUSH_DRREGISTER(hsc);
  }

  if(HAL_SMARTCARD_GetError(hsc) & HAL_SMARTCARD_ERROR_ORE)
  {
    __HAL_SMARTCARD_FLUSH_DRREGISTER(hsc);
  }
}
