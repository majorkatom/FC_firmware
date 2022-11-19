/*
 * imu.h
 *
 *  Created on: Nov 16, 2022
 *      Author: Tamas
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

typedef struct bmi08x_sensor_data IMU_RawSensorDataType;
typedef struct bmi08x_sensor_data_f IMU_FloatSensorDataType;

typedef struct IMU_DataType_ {
	IMU_FloatSensorDataType accel;
	IMU_FloatSensorDataType gyro;
} IMU_DataType;

typedef struct IMU_HandleType_ {
	SemaphoreHandle_t txRxFinishedSemaphore;
	volatile HAL_StatusTypeDef txRxRetVal;
	GPIO_TypeDef * CSPort;
	uint16_t CSPin;
	IMU_RawSensorDataType accelData;
	IMU_RawSensorDataType gyroData;
} IMU_HandleType;

typedef enum IMU_StatusType_{
	IMU_OK,
	IMU_INIT_FAILED,
	IMU_TIMEOUT
} IMU_StatusType;

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;

IMU_StatusType imuInit();
void imuDelayUs(uint32_t period, void* intfPtr);
void imuGpioExtiCallback(uint16_t GPIO_Pin);
void imuSpiTxRxCpltCallback(SPI_HandleTypeDef * hspi);
IMU_StatusType imuReadData(IMU_DataType *dataOut);

#endif /* INC_IMU_H_ */
