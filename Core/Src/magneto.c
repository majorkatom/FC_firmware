/*
 * magneto.c
 *
 *  Created on: Nov 20, 2022
 *      Author: Tamas
 */

#include "bsp.h"
// TODO: delete stdio.h and string.h
#include <stdio.h>
#include <string.h>

extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart4;  // TODO: delete huart4
const float magConversionCoeff = 1.5;
MAG_HandleType hmag0;
SemaphoreHandle_t magHandleLockSemaphore;
TaskHandle_t magReceiveTaskHandle = NULL;
const uint8_t magCfgValsBuff[3] = {MAG_CFG_REG_A_VAL, MAG_CFG_REG_B_VAL, MAG_CFG_REG_C_VAL};

static MAG_StatusType magRead(const uint8_t regAddrs, uint8_t *readBuff, uint8_t len);
static MAG_StatusType magWrite(const uint8_t regAddrs,const uint8_t *writeBuff, uint8_t len);
static void magReceiveTask(void *param);
static void magRecover(uint8_t *dataBuff);
// TODO: delete read task
static void magReadTask(void *param);


MAG_StatusType magInit()
{
	MAG_StatusType retVal = MAG_INIT_FAILED;
	uint8_t errorCntr = 0u;

	const uint8_t magResetVal = MAG_RESET_REG_VAL;
	uint8_t readBuff[6];

	HAL_NVIC_DisableIRQ(DRDY_MAG_EXTI_IRQn);
	vTaskDelay(21);

	hmag0.txRxFinishedSemaphore = xSemaphoreCreateBinary();

	if(MAG_OK != magWrite(MAG_CFG_REG_A, &magResetVal, 1))
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
		xTaskCreate(&magReadTask, "MAG_RECEIVE", 128, NULL, 1, NULL);

		HAL_NVIC_SetPriority(DRDY_MAG_EXTI_IRQn, 5, 0);
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

	return retVal;
}

static void magReceiveTask(void *param)
{
	while(1)
	{
		uint32_t notified = ulTaskNotifyTake(pdTRUE, 500);
		if(0u != notified)
		{
			if(pdTRUE == xSemaphoreTake(magHandleLockSemaphore, 3))
			{
				magRead(MAG_OUTX_L_REG, hmag0.rawDataBuff, 6);
				magRecover(hmag0.rawDataBuff);
				xSemaphoreGive(magHandleLockSemaphore);
			}
		}
		else
		{
			// error TODO: stop motors or something
		}
	}
}

static void magRecover(uint8_t *dataBuff)
{
	uint8_t zeroCnt = 0u;
	for(uint8_t i = 0u; i < 6; i++)
	{
		if(0u == dataBuff[i])
		{
			zeroCnt++;
		}
	}
	if(6u == zeroCnt)
	{
		magWrite(MAG_CFG_REG_A, magCfgValsBuff, 3);
		magRead(MAG_OUTX_L_REG, dataBuff, 6);
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
		dataOut->y = (float)yRaw * magConversionCoeff;
		dataOut->z = (float)zRaw * magConversionCoeff;

		retVal = MAG_OK;
	}

	return retVal;
}

static void magReadTask(void *param)
{
	while(1)
	{
		MAG_DataType magData;
		magReadData(&magData);

		char stlink_buff[32];
		sprintf(stlink_buff, "x: %.1f, y: %.1f, z: %.1f\r\n", magData.x, magData.y, magData.z);
		HAL_UART_Transmit(&huart4, (uint8_t*)stlink_buff, strlen(stlink_buff), 10);
		vTaskDelay(10);
	}
}
