/*
 * wifi.h
 *
 *  Created on: Nov 22, 2022
 *      Author: Tamas
 */

#ifndef INC_WIFI_H_
#define INC_WIFI_H_

#define WIFI_ENABLE 1U
#define WIFI_MESSAGE_SIZE	8U
#define WIFI_CIRC_BUFF_SIZE	100U

typedef struct WIFI_CircularBuffType_ {
	uint8_t buffer[WIFI_CIRC_BUFF_SIZE][WIFI_MESSAGE_SIZE];
	volatile uint8_t head;
	volatile uint8_t tail;
	volatile uint8_t full;
	volatile uint8_t empty;
} WIFI_CircularBuffType;

typedef enum WIFI_MessageType_ {
	WIFI_ACC_DATA,
	WIFI_GYRO_DATA,
	WIFI_MAG_DATA
} WIFI_MessageType;

typedef enum WIFI_StatusType_ {
	WIFI_OK,
	WIFI_DISABLED,
	WIFI_TIMEOUT,
	WIFI_FULL,
	WIFI_EMPTY
} WIFI_StatusType;

WIFI_StatusType wifiInit();
WIFI_StatusType wifiPutMessage(WIFI_MessageType MsgTyp, uint8_t *data, uint8_t dataLen);
void wifiUartTxCpltCallback(UART_HandleTypeDef * huart);

#endif /* INC_WIFI_H_ */
