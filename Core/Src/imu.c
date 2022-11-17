/*
 * imu.c
 *
 *  Created on: Nov 16, 2022
 *      Author: Tamas
 */

#include "bsp.h"
#include "bmi08x.h"

struct bmi08x_dev imuDev;
uint8_t imuAccDevAddr = 0u;
uint8_t imuGyroDevAddr = 0u;
IMU_HandleType himu0;

static BMI08X_INTF_RET_TYPE imuRead(uint8_t, uint8_t*, uint32_t, void*);
static BMI08X_INTF_RET_TYPE imuWrite(uint8_t, const uint8_t*, uint32_t, void*);


IMU_StatusType imuInit()
{
	IMU_StatusType retVal = IMU_INIT_FAILED;
	uint8_t errorCntr = 0u;
	struct bmi08x_int_cfg intConfig;
	struct bmi08x_data_sync_cfg synConfig;

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

	synConfig.mode = BMI08X_ACCEL_DATA_SYNC_MODE_OFF;

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

	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

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

	if(BMI08X_OK != bmi08a_configure_data_synchronization(synConfig, &imuDev))
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

	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	return retVal;
}
