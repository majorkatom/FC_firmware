/*
 * radio.h
 *
 *  Created on: 2022. okt. 30.
 *      Author: Tamas
 */

#ifndef INC_RADIO_H_
#define INC_RADIO_H_

#define RADIO_CH_NUM 14U

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
	RADIO_SEMAPHORE_TIMEOUT_ERROR,
	RADIO_UART_ERROR
} RADIO_StatusType;

extern UART_HandleTypeDef huart3;

void radioInit();
void radioUartRxCpltCallback(UART_HandleTypeDef *huart);
void radioReceiveTask(void *param);

#endif /* INC_RADIO_H_ */
