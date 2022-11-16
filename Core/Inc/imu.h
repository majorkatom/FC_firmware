/*
 * imu.h
 *
 *  Created on: Nov 16, 2022
 *      Author: Tamas
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

typedef struct IMU_HandleType_ {
	uint8_t transmitOrReceiveFlag;  // 0 if receive, 1 if transmit
	volatile uint8_t regAddrSentFlag;
	volatile uint8_t txRxFinishedFlag;
	volatile HAL_StatusTypeDef txRxRetVal;
	uint8_t *regData;
	uint16_t len;
	GPIO_TypeDef * CSPort;
	uint16_t CSPin;
} IMU_HandleType;

typedef enum IMU_StatusType_{
	IMU_OK,
	IMU_INIT_FAILED
} IMU_StatusType;

extern SPI_HandleTypeDef hspi1;

IMU_StatusType imuInit();
void imuDelayUs(uint32_t period, void* intf_ptr);
void imuGpioExtiCallback(uint16_t GPIO_Pin);
void imuSpiTxCpltCallback(SPI_HandleTypeDef * hspi);
void imuSpiRxCpltCallback(SPI_HandleTypeDef * hspi);

#endif /* INC_IMU_H_ */
