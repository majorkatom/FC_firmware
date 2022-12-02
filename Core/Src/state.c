/*
 * state.c
 *
 *  Created on: 2022. nov. 30.
 *      Author: Tamas
 */

#include <math.h>
#include "bsp.h"
#include "oriGetData.h"

static STATE_StateType stateVariable = STATE_INIT;
static const float stateRad2Deg = 180 / M_PI;

static void stateTask(void *param);

void stateInit(IMU_StatusType imuInitRetVal, MAG_StatusType magInitRetVal)
{
	if((IMU_OK == imuInitRetVal) && (MAG_OK == magInitRetVal))
	{
		xTaskCreate(&stateTask, "STATE", 2048, NULL, STATE_PRIO, NULL);
	}
}

STATE_StateType stateGetState()
{
	return stateVariable;
}

void stateSetState(STATE_StateType stateToSet)
{
	if(STATE_INIT != stateVariable)
	{
		stateVariable = stateToSet;
	}
}

static void stateTask(void *param)
{
	uint16_t *radioChannels = NULL;
	IMU_DataType imuData;
	MAG_DataType magData;
	STATE_OrientationDatatType orientation;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, 10);

		radioReadData(radioChannels);
		IMU_StatusType imuReadRetVal = imuReadData(&imuData);
		MAG_StatusType magReadRetVal = magReadData(&magData);

		if((IMU_OK == imuReadRetVal) && (MAG_OK == magReadRetVal))
		{
			oriGetData(&imuData, &magData, &orientation);

			int16_t pitchInt = (int16_t)(orientation.pitch * stateRad2Deg);
			int16_t rollInt = (int16_t)(orientation.roll  * stateRad2Deg);
			int16_t yawRateInt = (int16_t)(orientation.yawRate * stateRad2Deg);
			uint8_t oriBuffToSend[6];
			oriBuffToSend[0] = pitchInt >> 8;
			oriBuffToSend[1] = pitchInt & 0x00ff;
			oriBuffToSend[2] = rollInt >> 8;
			oriBuffToSend[3] = rollInt & 0x00ff;
			oriBuffToSend[4] = yawRateInt >> 8;
			oriBuffToSend[5] = yawRateInt & 0x00ff;
			wifiPutMessage(WIFI_ORIENTATION, oriBuffToSend, 6);
		}

		if((NULL != radioChannels) && (IMU_OK == imuReadRetVal) && (MAG_OK == magReadRetVal))
		{
			switch(stateVariable)
			{
				case STATE_INIT:
					break;
				case STATE_DISARMED:
					break;
				case STATE_ARMED:
					break;
			}
		}
	}
}
