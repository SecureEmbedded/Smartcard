/*
 * smartcard.h
 *
 *  Created on: 21 dic 2016
 *      Author: Salvatore
 */

#ifndef APPLICATION_USER_SMARTCARD_H_
#define APPLICATION_USER_SMARTCARD_H_

#include "stm32f4xx_hal.h"

#define MAX_ATR_SIZE	   40
#define SETUP_LENGTH       20
#define HIST_LENGTH        20
#define ATR_SIZE		   24


#define INIT_ETU_CLK_CICLES		250000
#define WORK_ETU_CLK_CICLES		25*250000
#define INIT_ETU 		   22
#define WORK_ETU 		   7

#define INTERRUPT_MODE 		0

typedef enum
{
  PW_DOWN = GPIO_PIN_RESET,
  PW_UP = GPIO_PIN_SET
}PowerState;

typedef enum{
	LOW_RESET = GPIO_PIN_RESET,
	HIGH_RESET = GPIO_PIN_SET
}ResetState;

typedef enum{
	CARD_UNKNOWN = 0x00,
	CARD_ACTIVATION = 0x01,
	COLD_RESET = 0x02,
	ATR_RESPONSE = 0x03,
	ATR_DECODED = 0x04,
	PTS_REQUEST_SENT = 0x05,
	PTS_RESPONSE = 0x06,
	PTS_EXCHANGED = 0x07
}SLJ52States;

typedef enum{
	IDLE = 0x00,
	SENT = 0x01,
	RECEIVED = 0x02
} APDU_State;

typedef struct{
	uint8_t CLA;
	uint8_t INS;
	uint8_t P1;
	uint8_t P2;
	uint8_t LC;
	uint8_t Data[100];
	uint8_t LE;
}SLJ52_APDU;

typedef struct {
	uint8_t TS;
	uint8_t T0;
	uint8_t T[SETUP_LENGTH];
	uint8_t H[HIST_LENGTH];
	uint8_t Tlength;
	uint8_t Hlength;
}SLJ52ATR;

typedef struct{
	uint8_t PTSS;
	uint8_t PTS0;
	uint8_t PTS1;
	uint8_t PTS2;
	uint8_t PTS3;
	uint8_t PCK;
}PPSType;

void SLJ52_PWR(PowerState status);
void SLJ52_RST(ResetState rst);
void SLJ52_Init();
void SLJ52_Interface_Configuration();
void SLJ52_ATR_Parse();
void SLJ52_PPS();
void SendAPDU();
#endif /* APPLICATION_USER_SMARTCARD_H_ */
