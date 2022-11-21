/*
 * magneto.h
 *
 *  Created on: Nov 20, 2022
 *      Author: Tamas
 */

#ifndef INC_MAGNETO_H_
#define INC_MAGNETO_H_

#define MAG_CFG_REG_A (uint8_t)0x60
#define MAG_OUTX_L_REG (uint8_t)0x68

#define MAG_RESET_REG_VAL (uint8_t)0b00100011
#define MAG_CFG_REG_A_VAL (uint8_t)0b10001100
#define MAG_CFG_REG_B_VAL (uint8_t)0b00000011
#define MAG_CFG_REG_C_VAL (uint8_t)0b00110001

typedef struct MAG_DataType_ {
	float x;
	float y;
	float z;
} MAG_DataType;

typedef enum MAG_TxOrRxType_ {
	Tx,
	Rx
} MAG_TxOrRxType;

typedef struct MAG_HandleType_ {
	SemaphoreHandle_t txRxFinishedSemaphore;
	MAG_TxOrRxType txOrRx;
	uint8_t rawDataBuff[6];
} MAG_HandleType;

typedef enum MAG_StatusType_ {
	MAG_OK,
	MAG_INIT_FAILED,
	MAG_ERROR,
	MAG_TIMEOUT
} MAG_StatusType;

MAG_StatusType magInit();
void magGpioExtiCallback(uint16_t GPIO_Pin);
void magSpiTxCpltCallback(SPI_HandleTypeDef * hspi);
void magSpiRxCpltCallback(SPI_HandleTypeDef * hspi);
MAG_StatusType magReadData(MAG_DataType *dataOut);

#endif /* INC_MAGNETO_H_ */
