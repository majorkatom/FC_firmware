/*
 * imu.c
 *
 *  Created on: Nov 16, 2022
 *      Author: Tamas
 */

#include "bsp.h"
#include "bmi08x.h"
// TODO: remove stdio.h and string.h include
#include <stdio.h>
#include <string.h>

struct bmi08x_dev imuDev;
uint8_t imuAccDevAddr = 0u;
uint8_t imuGyroDevAddr = 0u;
IMU_HandleType himu0;

static BMI08X_INTF_RET_TYPE imuRead(uint8_t regAddr, uint8_t *regData, uint32_t len, void *intfPtr);
static BMI08X_INTF_RET_TYPE imuWrite(uint8_t regAddr, const uint8_t *regData, uint32_t len, void *intfPtr);


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

	HAL_NVIC_DisableIRQ(DRDY_ACC_EXTI_IRQn);
	HAL_NVIC_DisableIRQ(DRDY_GYRO_EXTI_IRQn);

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
	}

	HAL_NVIC_EnableIRQ(DRDY_ACC_EXTI_IRQn);
	HAL_NVIC_EnableIRQ(DRDY_GYRO_EXTI_IRQn);

	return retVal;
}

void imuDelayUs(uint32_t period, void* intfPtr)
{
	htim2.Instance->CNT = 0;
	while (htim2.Instance->CNT < period);
}

static BMI08X_INTF_RET_TYPE imuRead(uint8_t regAddr, uint8_t *regData, uint32_t len, void *intfPtr)
{
	BMI08X_INTF_RET_TYPE retVal = BMI08X_E_COM_FAIL;
	himu0.txRxRetVal = HAL_ERROR;
	himu0.regAddrSentFlag = 0;
	himu0.txRxFinishedFlag = 0;
	himu0.regData = regData;
	himu0.len = (uint16_t)len;
	himu0.transmitOrReceiveFlag = 0;

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
	HAL_GPIO_WritePin(himu0.CSPort, himu0.CSPin, RESET);
	HAL_SPI_Transmit_DMA(&hspi1, &regAddr, 1);
	while(1 != himu0.txRxFinishedFlag){}
	if(HAL_OK == himu0.txRxRetVal)
	{
		retVal = BMI08X_OK;
	}

	return retVal;
}

BMI08X_INTF_RET_TYPE imuWrite(uint8_t regAddr, const uint8_t *regData, uint32_t len, void *intfPtr)
{
	BMI08X_INTF_RET_TYPE retVal = BMI08X_E_COM_FAIL;
	himu0.txRxRetVal = HAL_ERROR;
	himu0.regAddrSentFlag = 0;
	himu0.txRxFinishedFlag = 0;
	himu0.regData = regData;
	himu0.len = (uint16_t)len;
	himu0.transmitOrReceiveFlag = 1;

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
	HAL_GPIO_WritePin(himu0.CSPort, himu0.CSPin, RESET);
	HAL_SPI_Transmit_DMA(&hspi1, &regAddr, 1);
	while(1 != himu0.txRxFinishedFlag){}
	if(HAL_OK == himu0.txRxRetVal)
	{
		retVal = BMI08X_OK;
	}

	return retVal;
}

void imuSpiTxCpltCallback(SPI_HandleTypeDef * hspi)
{
    if(hspi == &hspi1)
    {
    	if(0 == himu0.regAddrSentFlag)
    	{
    		himu0.regAddrSentFlag = 1;
    		if(0 == himu0.transmitOrReceiveFlag)
    		{
    			himu0.txRxRetVal = HAL_SPI_Receive_DMA(&hspi1, himu0.regData, himu0.len);
    		}
    		else
    		{
    			himu0.txRxRetVal = HAL_SPI_Transmit_DMA(&hspi1, himu0.regData, himu0.len);
    		}
    	}
    	else
    	{
    		HAL_GPIO_WritePin(himu0.CSPort, himu0.CSPin, SET);
    		himu0.txRxFinishedFlag = 1;
    	}
    }
}

void imuSpiRxCpltCallback(SPI_HandleTypeDef * hspi)
{
    if(hspi == &hspi1)
    {
		HAL_GPIO_WritePin(himu0.CSPort, himu0.CSPin, SET);
		himu0.txRxFinishedFlag = 1;
    }
}

void imuGpioExtiCallback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == DRDY_ACC_Pin)
	{
		struct bmi08x_sensor_data accelBmi088;
		double conversionCoeff = 6000.0 / 32768.0;
		char stlink_buff_it[64];

		int8_t rslt = bmi08a_get_data(&accelBmi088, &imuDev);

		double accOutX = accelBmi088.x * conversionCoeff;
		double accOutY = accelBmi088.y * conversionCoeff;
		double accOutZ = accelBmi088.z * conversionCoeff;

		sprintf(stlink_buff_it, "rslt: %i\tacc_x: %.1f\tacc_y: %.1f\tacc_z: %.1f\r\n", rslt, accOutX, accOutY, accOutZ);
		HAL_UART_Transmit(&huart4, (uint8_t*)stlink_buff_it, strlen(stlink_buff_it), 100);
	}
	else if (GPIO_Pin == DRDY_GYRO_Pin)
	{
		struct bmi08x_sensor_data gyroBmi088;
		double conversionCoeff = 2000.0 / 32768.0;
		char stlink_buff_it[64];

		int8_t rslt = bmi08g_get_data(&gyroBmi088, &imuDev);

		double rateOutX = gyroBmi088.x * conversionCoeff;
		double rateOutY = gyroBmi088.y * conversionCoeff;
		double rateOutZ = gyroBmi088.z * conversionCoeff;

		sprintf(stlink_buff_it, "rslt: %i\trate_x: %.1f\trate_y: %.1f\trate_z: %.1f\r\n", rslt, rateOutX, rateOutY, rateOutZ);
		HAL_UART_Transmit(&huart4, (uint8_t*)stlink_buff_it, strlen(stlink_buff_it), 100);
	}
}
