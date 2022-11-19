/*
 * imu.h
 *
 *  Created on: Nov 16, 2022
 *      Author: Tamas
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

typedef struct IMU_HandleType_ {
	SemaphoreHandle_t txRxFinishedSemaphore;
	volatile HAL_StatusTypeDef txRxRetVal;
	GPIO_TypeDef * CSPort;
	uint16_t CSPin;
} IMU_HandleType;

typedef enum IMU_StatusType_{
	IMU_OK,
	IMU_INIT_FAILED
} IMU_StatusType;

// TODO: remove huart4 extern
extern UART_HandleTypeDef huart4;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;

IMU_StatusType imuInit();
void imuDelayUs(uint32_t period, void* intfPtr);
void imuGpioExtiCallback(uint16_t GPIO_Pin);
void imuSpiTxRxCpltCallback(SPI_HandleTypeDef * hspi);

#endif /* INC_IMU_H_ */
