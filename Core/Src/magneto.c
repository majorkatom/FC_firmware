/*
 * magneto.c
 *
 *  Created on: Nov 20, 2022
 *      Author: Tamas
 */

#include "bsp.h"

extern SPI_HandleTypeDef hspi2;
static const float magConversionCoeff = 0.15;  // to uT
static MAG_HandleType hmag0;
const uint8_t magCfgValsBuff[3] = {MAG_CFG_REG_A_INIT_VAL, MAG_CFG_REG_B_VAL, MAG_CFG_REG_C_VAL};
static SemaphoreHandle_t magHandleLockSemaphore;
static TaskHandle_t magReceiveTaskHandle = NULL;
static const MAG_DataType magOffsets = {.x = -17.662177579818167, .y = -10.273775677635934, .z = 55.149884780691890};
static const float magScaleMatrix[3][3] = {{1.003989851105835, -1.561557387852464e-4, -0.008737941055190},
											  {-1.561557387852464e-4, 1.006942966572786, 0.036916526658156},
											  {-0.008737941055190, 0.036916526658156, 0.990587716987121}};

static MAG_StatusType magRead(const uint8_t regAddrs, uint8_t *readBuff, uint8_t len);
static MAG_StatusType magWrite(const uint8_t regAddrs,const uint8_t *writeBuff, uint8_t len);
static void magReceiveTask(void *param);
static void magCalibrate(MAG_DataType *magData);

MAG_StatusType magInit()
{
	MAG_StatusType retVal = MAG_INIT_FAILED;
	uint8_t errorCntr = 0u;

	const uint8_t resetVal = MAG_RESET_REG_VAL;
	uint8_t readBuff[6];

	vTaskDelay(21);

	hmag0.txRxFinishedSemaphore = xSemaphoreCreateBinary();

	if(MAG_OK != magWrite(MAG_CFG_REG_A, &resetVal, 1))
	{
		errorCntr++;
	}
	vTaskDelay(1);

	if(MAG_OK != magWrite(MAG_CFG_REG_A, magCfgValsBuff, 3))
	{
		errorCntr++;
	}
	vTaskDelay(1);

	if(MAG_OK != magRead(MAG_OUTX_L_REG, readBuff, 6))
	{
		errorCntr++;
	}

	if(0u == errorCntr)
	{
		retVal = MAG_OK;
		magHandleLockSemaphore = xSemaphoreCreateMutex();
		xSemaphoreGive(magHandleLockSemaphore);

		xTaskCreate(&magReceiveTask, "MAG_RECEIVE", 256, NULL, MAG_RECEIVE_PRIO, &magReceiveTaskHandle);

		HAL_NVIC_EnableIRQ(DRDY_MAG_EXTI_IRQn);
	}

	return retVal;
}

void magGpioExtiCallback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == DRDY_MAG_Pin)
	{
		BaseType_t higherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(magReceiveTaskHandle, &higherPriorityTaskWoken);
		portYIELD_FROM_ISR(higherPriorityTaskWoken);
	}
}

void magSpiTxCpltCallback(SPI_HandleTypeDef * hspi)
{
	if(hspi == &hspi2)
	{
		if(Tx == hmag0.txOrRx)
		{
			HAL_GPIO_WritePin(CS_MAG_GPIO_Port, CS_MAG_Pin, SET);
		}
		BaseType_t higherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(hmag0.txRxFinishedSemaphore, &higherPriorityTaskWoken);
		portYIELD_FROM_ISR(higherPriorityTaskWoken);
	}
}
void magSpiRxCpltCallback(SPI_HandleTypeDef * hspi)
{
	if(hspi == &hspi2)
	{
		HAL_GPIO_WritePin(CS_MAG_GPIO_Port, CS_MAG_Pin, SET);
		BaseType_t higherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(hmag0.txRxFinishedSemaphore, &higherPriorityTaskWoken);
		portYIELD_FROM_ISR(higherPriorityTaskWoken);
	}
}

MAG_StatusType magRead(const uint8_t regAddrs, uint8_t *readBuff, uint8_t len)
{
	MAG_StatusType retVal = MAG_ERROR;
	HAL_StatusTypeDef spiTxRetVal;
	HAL_StatusTypeDef spiRxRetVal;

	uint8_t rxAddr = 0b10000000 | regAddrs;
	uint8_t rxBuff[len];

	hmag0.txOrRx = Rx;

	HAL_GPIO_WritePin(CS_MAG_GPIO_Port, CS_MAG_Pin, RESET);
	spiTxRetVal = HAL_SPI_Transmit_IT(&hspi2, &rxAddr, 1);
	if(pdTRUE == xSemaphoreTake(hmag0.txRxFinishedSemaphore, 2))
	{
		spiRxRetVal = HAL_SPI_Receive_IT(&hspi2, rxBuff, len);
		if((HAL_OK == spiTxRetVal) && (HAL_OK == spiRxRetVal) && (pdTRUE == xSemaphoreTake(hmag0.txRxFinishedSemaphore, 2)))
		{
			retVal = MAG_OK;
		}
		else
		{
			HAL_GPIO_WritePin(CS_MAG_GPIO_Port, CS_MAG_Pin, SET);
		}
	}

	for(uint8_t i = 0u; i < len; i++)
	{
		readBuff[i] = rxBuff[i];
	}

	return retVal;
}

static MAG_StatusType magWrite(const uint8_t regAddrs,const uint8_t *writeBuff, uint8_t len)
{
	MAG_StatusType retVal = MAG_ERROR;
	HAL_StatusTypeDef spiRetVal;

	uint8_t txBuff[len + 1];
	txBuff[0] = regAddrs;
	for(uint8_t i = 0u; i < len; i++)
	{
		txBuff[i + 1] = writeBuff[i];
	}

	hmag0.txOrRx = Tx;

	HAL_GPIO_WritePin(CS_MAG_GPIO_Port, CS_MAG_Pin, RESET);
	spiRetVal = HAL_SPI_Transmit_IT(&hspi2, txBuff, len + 1);
	if((HAL_OK == spiRetVal) && (pdTRUE == xSemaphoreTake(hmag0.txRxFinishedSemaphore, 2)))
	{
		retVal = MAG_OK;
	}
	else
	{
		HAL_GPIO_WritePin(CS_MAG_GPIO_Port, CS_MAG_Pin, SET);
	}

	return retVal;
}

static void magReceiveTask(void *param)
{
	const uint8_t startMeasurementRegVal = MAG_CFG_REG_A_MEASURE_VAL;
	uint16_t noDataDebounceCntr = 0u;
	while(1)
	{
		if(30u > noDataDebounceCntr)
		{
			if(pdTRUE == xSemaphoreTake(magHandleLockSemaphore, 2))
			{
				if(MAG_OK != magWrite(MAG_CFG_REG_A, &startMeasurementRegVal, 1))
				{
					magWrite(MAG_CFG_REG_A, magCfgValsBuff, 3);  // recover
					magWrite(MAG_CFG_REG_A, &startMeasurementRegVal, 1);
				}
				magWrite(MAG_CFG_REG_A, &startMeasurementRegVal, 1);
				xSemaphoreGive(magHandleLockSemaphore);
				uint32_t notified = ulTaskNotifyTake(pdTRUE, 10);
				if(0u != notified)
				{
					if(pdTRUE == xSemaphoreTake(magHandleLockSemaphore, 3))
					{
						magRead(MAG_OUTX_L_REG, hmag0.rawDataBuff, 6);
//						wifiPutMessage(WIFI_MAG_DATA, hmag0.rawDataBuff, 6);
						xSemaphoreGive(magHandleLockSemaphore);
						noDataDebounceCntr = 0u;
					}
					else
					{
						noDataDebounceCntr++;
					}
				}
				else
				{
					noDataDebounceCntr++;
				}
			}
			else
			{
				noDataDebounceCntr++;
			}
		}
		else
		{
			escStopPWM();
			stateSetState(STATE_DISARMED, STATE_MAG_ERROR);
		}
	}
}

MAG_StatusType magReadData(MAG_DataType *dataOut)
{
	MAG_StatusType retVal = MAG_TIMEOUT;

	if(pdTRUE == xSemaphoreTake(magHandleLockSemaphore, 5))
	{
		int16_t xRaw = hmag0.rawDataBuff[1] << 8;
		xRaw |= hmag0.rawDataBuff[0];

		int16_t yRaw = hmag0.rawDataBuff[3] << 8;
		yRaw |= hmag0.rawDataBuff[2];

		int16_t zRaw = hmag0.rawDataBuff[5] << 8;
		zRaw |= hmag0.rawDataBuff[4];

		xSemaphoreGive(magHandleLockSemaphore);

		dataOut->x = (float)xRaw * magConversionCoeff;
		dataOut->y = (float)(-yRaw) * magConversionCoeff;  // transform to body coordinate system
		dataOut->z = (float)zRaw * magConversionCoeff;

		magCalibrate(dataOut);

		retVal = MAG_OK;
	}

	return retVal;
}

static void magCalibrate(MAG_DataType *magData)
{
	// acc
	float magXTemp = magData->x - magOffsets.x;
	float magYTemp = magData->y - magOffsets.y;
	float magZTemp = magData->z - magOffsets.z;

	magData->x = magXTemp * magScaleMatrix[0][0] +  magYTemp * magScaleMatrix[0][1] +  magZTemp * magScaleMatrix[0][2];
	magData->y = magXTemp * magScaleMatrix[1][0] +  magYTemp * magScaleMatrix[1][1] +  magZTemp * magScaleMatrix[1][2];
	magData->z = magXTemp * magScaleMatrix[2][0] +  magYTemp * magScaleMatrix[2][1] +  magZTemp * magScaleMatrix[2][2];
}
