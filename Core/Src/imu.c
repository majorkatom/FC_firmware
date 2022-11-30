/*
 * imu.c
 *
 *  Created on: Nov 16, 2022
 *      Author: Tamas
 */

#include "bsp.h"

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
static struct bmi08x_dev imuDev;
static uint8_t imuAccDevAddr = 0u;
static uint8_t imuGyroDevAddr = 0u;
static const float imuAccConversionCoeff = 6000.0 / 32768.0;
static const float imuGyroConversionCoeff = 2000.0 / 32768.0;
static IMU_HandleType himu0;
static SemaphoreHandle_t imuHandleLockSemaphore;
static TaskHandle_t imuAccReceiveTaskHandle = NULL;
static TaskHandle_t imuGyroReceiveTaskHandle = NULL;

static BMI08X_INTF_RET_TYPE imuRead(uint8_t regAddr, uint8_t *regData, uint32_t len, void *intfPtr);
static BMI08X_INTF_RET_TYPE imuWrite(uint8_t regAddr, const uint8_t *regData, uint32_t len, void *intfPtr);
static void imuAccReceiveTask(void *param);
static void imuGyroReceiveTask(void *param);

IMU_StatusType imuInit()
{
	IMU_StatusType retVal = IMU_INIT_FAILED;
	uint8_t errorCntr = 0u;
	struct bmi08x_int_cfg intConfig;
	struct bmi08x_data_sync_cfg syncConfig;

	imuDev.intf_ptr_accel = &imuAccDevAddr;
	imuDev.intf_ptr_gyro = &imuGyroDevAddr;
	imuDev.intf = BMI08X_SPI_INTF;
	imuDev.read = imuRead;
	imuDev.write = imuWrite;
	imuDev.delay_us = imuDelayUs;
	imuDev.variant = BMI088_VARIANT;
	imuDev.read_write_len = 32u;

	imuDev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
	imuDev.accel_cfg.odr = BMI08X_ACCEL_ODR_100_HZ;
	imuDev.accel_cfg.range = BMI088_ACCEL_RANGE_6G;

	imuDev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
	imuDev.gyro_cfg.odr = BMI08X_GYRO_BW_32_ODR_100_HZ;
	imuDev.gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS;

	syncConfig.mode = BMI08X_ACCEL_DATA_SYNC_MODE_OFF;

	intConfig.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;
	intConfig.accel_int_config_1.int_type = BMI08X_ACCEL_INT_DATA_RDY;
	intConfig.accel_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
	intConfig.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	intConfig.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

	intConfig.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;
	intConfig.gyro_int_config_1.int_type = BMI08X_GYRO_INT_DATA_RDY;
	intConfig.gyro_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
	intConfig.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	intConfig.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

	HAL_TIM_Base_Start(&htim2);

	himu0.txRxFinishedSemaphore = xSemaphoreCreateBinary();

	if(BMI08X_OK != bmi08a_init(&imuDev))
	{
		errorCntr++;
	}

	if(BMI08X_OK != bmi08g_init(&imuDev))
	{
		errorCntr++;
	}

	if(BMI08X_OK != bmi08a_soft_reset(&imuDev))
	{
		errorCntr++;
	}

	if(BMI08X_OK != bmi08a_load_config_file(&imuDev))
	{
		errorCntr++;
	}

	if(BMI08X_OK != bmi08a_set_power_mode(&imuDev))
	{
		errorCntr++;
	}

	if(BMI08X_OK != bmi08g_set_power_mode(&imuDev))
	{
		errorCntr++;
	}

	if(BMI08X_OK != bmi08a_configure_data_synchronization(syncConfig, &imuDev))
	{
		errorCntr++;
	}

	if(BMI08X_OK != bmi08a_set_int_config(&intConfig.accel_int_config_1, &imuDev))
	{
		errorCntr++;
	}

	if(BMI08X_OK != bmi08g_set_int_config(&intConfig.gyro_int_config_1, &imuDev))
	{
		errorCntr++;
	}

	if(0u == errorCntr)
	{
		retVal = IMU_OK;

		imuHandleLockSemaphore = xSemaphoreCreateMutex();
		xSemaphoreGive(imuHandleLockSemaphore);

		xTaskCreate(&imuAccReceiveTask, "IMU_ACC_RECEIVE", 256, NULL, IMU_ACC_GYRO_RECEIVE_PRIO, &imuAccReceiveTaskHandle);
		xTaskCreate(&imuGyroReceiveTask, "IMU_GYRO_RECEIVE", 256, NULL, IMU_ACC_GYRO_RECEIVE_PRIO, &imuGyroReceiveTaskHandle);

		HAL_NVIC_EnableIRQ(DRDY_ACC_EXTI_IRQn);
		HAL_NVIC_EnableIRQ(DRDY_GYRO_EXTI_IRQn);
	}

	return retVal;
}

void imuDelayUs(uint32_t period, void* intfPtr)
{
	htim2.Instance->CNT = 0;
	while (htim2.Instance->CNT < period);
}

void imuGpioExtiCallback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == DRDY_ACC_Pin)
	{
		BaseType_t higherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(imuAccReceiveTaskHandle, &higherPriorityTaskWoken);
		portYIELD_FROM_ISR(higherPriorityTaskWoken);
	}
	else if (GPIO_Pin == DRDY_GYRO_Pin)
	{
		BaseType_t higherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(imuGyroReceiveTaskHandle, &higherPriorityTaskWoken);
		portYIELD_FROM_ISR(higherPriorityTaskWoken);
	}
}

void imuSpiTxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
	if(hspi == &hspi1)
	{
		HAL_GPIO_WritePin(himu0.CSPort, himu0.CSPin, SET);
		BaseType_t higherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(himu0.txRxFinishedSemaphore, &higherPriorityTaskWoken);
		portYIELD_FROM_ISR(higherPriorityTaskWoken);
	}
}

static BMI08X_INTF_RET_TYPE imuRead(uint8_t regAddr, uint8_t *regData, uint32_t len, void *intfPtr)
{
	BMI08X_INTF_RET_TYPE retVal = BMI08X_E_COM_FAIL;
	uint16_t buffLen = len + 1;
	uint8_t txBuff[buffLen];
	uint8_t rxBuff[buffLen];

	himu0.txRxRetVal = HAL_ERROR;

	if(intfPtr == imuDev.intf_ptr_accel)
	{
		himu0.CSPort = CS_ACC_GPIO_Port;
		himu0.CSPin = CS_ACC_Pin;
	}
	else
	{
		himu0.CSPort = CS_GYRO_GPIO_Port;
		himu0.CSPin = CS_GYRO_Pin;
	}

	txBuff[0] = regAddr;

	HAL_GPIO_WritePin(himu0.CSPort, himu0.CSPin, RESET);
	himu0.txRxRetVal = HAL_SPI_TransmitReceive_DMA(&hspi1, txBuff, rxBuff, buffLen);

	BaseType_t txRxFinishedRslt = xSemaphoreTake(himu0.txRxFinishedSemaphore, 2);

	for(uint8_t i = 0u;i < len;i++)
	{
		regData[i] = rxBuff[i + 1];
	}

	if((HAL_OK == himu0.txRxRetVal)&&(pdTRUE == txRxFinishedRslt))
	{
		TaskHandle_t currentTaskHandle = xTaskGetCurrentTaskHandle();
		if(imuAccReceiveTaskHandle == currentTaskHandle)
		{
			wifiPutMessage(WIFI_ACC_DATA, rxBuff + 2, 6);
		}
		else if (imuGyroReceiveTaskHandle == currentTaskHandle)
		{
			wifiPutMessage(WIFI_GYRO_DATA, rxBuff + 1, 6);
		}

		retVal = BMI08X_OK;
	}
	else
	{
		HAL_GPIO_WritePin(himu0.CSPort, himu0.CSPin, SET);
	}

	return retVal;
}

static BMI08X_INTF_RET_TYPE imuWrite(uint8_t regAddr, const uint8_t *regData, uint32_t len, void *intfPtr)
{
	BMI08X_INTF_RET_TYPE retVal = BMI08X_E_COM_FAIL;
	uint16_t buffLen = len + 1;
	uint8_t txBuff[buffLen];
	uint8_t rxBuff[buffLen];

	himu0.txRxRetVal = HAL_ERROR;

	if(intfPtr == imuDev.intf_ptr_accel)
	{
		himu0.CSPort = CS_ACC_GPIO_Port;
		himu0.CSPin = CS_ACC_Pin;
	}
	else
	{
		himu0.CSPort = CS_GYRO_GPIO_Port;
		himu0.CSPin = CS_GYRO_Pin;
	}

	txBuff[0] = regAddr;
	for(uint8_t i = 0u;i < len;i++)
	{
		txBuff[i + 1] = regData[i];
	}

	HAL_GPIO_WritePin(himu0.CSPort, himu0.CSPin, RESET);
	himu0.txRxRetVal = HAL_SPI_TransmitReceive_DMA(&hspi1, txBuff, rxBuff, buffLen);

	BaseType_t txRxFinishedRslt = xSemaphoreTake(himu0.txRxFinishedSemaphore, 2);

	if((HAL_OK == himu0.txRxRetVal)&&(pdTRUE == txRxFinishedRslt))
	{
		retVal = BMI08X_OK;
	}
	else
	{
		HAL_GPIO_WritePin(himu0.CSPort, himu0.CSPin, SET);
	}

	return retVal;
}

static void imuAccReceiveTask(void *param)
{
	while(1)
	{
		uint32_t notified = ulTaskNotifyTake(pdTRUE, 200);
		if(0u != notified)
		{
			if(pdTRUE == xSemaphoreTake(imuHandleLockSemaphore, 5))
			{

				vTaskPrioritySet(imuGyroReceiveTaskHandle, IMU_ACC_GYRO_RECEIVE_PRIO - 1);

				bmi08a_get_data(&(himu0.accelData), &imuDev);

				xSemaphoreGive(imuHandleLockSemaphore);
				vTaskPrioritySet(imuGyroReceiveTaskHandle, IMU_ACC_GYRO_RECEIVE_PRIO);
			}
		}
		else
		{
			stateSetState(STATE_DISARMED);
		}
	}
}

static void imuGyroReceiveTask(void *param)
{
	while(1)
	{
		uint32_t notified = ulTaskNotifyTake(pdTRUE, 200);
		if(0u != notified)
		{
			if(pdTRUE == xSemaphoreTake(imuHandleLockSemaphore, 5))
			{
				vTaskPrioritySet(imuAccReceiveTaskHandle, IMU_ACC_GYRO_RECEIVE_PRIO - 1);

				bmi08g_get_data(&(himu0.gyroData), &imuDev);

				xSemaphoreGive(imuHandleLockSemaphore);
				vTaskPrioritySet(imuAccReceiveTaskHandle, IMU_ACC_GYRO_RECEIVE_PRIO);
			}
		}
		else
		{
			stateSetState(STATE_DISARMED);
		}
	}
}

IMU_StatusType imuReadData(IMU_DataType *dataOut)
{
	IMU_StatusType retVal = IMU_TIMEOUT;

	if(pdTRUE == xSemaphoreTake(imuHandleLockSemaphore, 5))
	{
		dataOut->accel.x = (float)himu0.accelData.x * imuAccConversionCoeff;
		dataOut->accel.y = (float)himu0.accelData.y * imuAccConversionCoeff;
		dataOut->accel.z = (float)himu0.accelData.z * imuAccConversionCoeff;

		dataOut->gyro.x = (float)himu0.gyroData.x * imuGyroConversionCoeff;
		dataOut->gyro.y = (float)himu0.gyroData.y * imuGyroConversionCoeff;
		dataOut->gyro.z = (float)himu0.gyroData.z * imuGyroConversionCoeff;

		xSemaphoreGive(imuHandleLockSemaphore);

		retVal = IMU_OK;
	}

	return retVal;
}
