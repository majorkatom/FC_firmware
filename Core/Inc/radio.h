/*
 * radio.h
 *
 *  Created on: 2022. okt. 30.
 *      Author: Tamas
 */

#ifndef INC_RADIO_H_
#define INC_RADIO_H_

#define RADIO_CH_NUM 14U
#define RADIO_MIN_CH_VAL 1000U
#define RADIO_MAX_CH_VAL 2000U

#define RADIO_CH_IDX_ARM_SWC 5U

typedef struct RADIO_MessageType_ {
	uint8_t length;
	uint8_t msgType;
	uint16_t checkSum;
	uint8_t rxBuffer[31];
	volatile uint8_t dataReceived;
} RADIO_MessageType;

typedef enum RADIO_StatusType_ {
	RADIO_OK,
	RADIO_CHECKSUM_ERROR,
	RADIO_LENGTH_ERROR,
	RADIO_MSG_TYPE_ERROR,
	RADIO_TIMEOUT,
	RADIO_UART_ERROR
} RADIO_StatusType;

void radioInit();
void radioUartRxCpltCallback(UART_HandleTypeDef *huart);
void radioReadData(uint16_t *radioChannels);

#endif /* INC_RADIO_H_ */
