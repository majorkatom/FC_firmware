/*
 * state.c
 *
 *  Created on: 2022. nov. 30.
 *      Author: Tamas
 */

#include "bsp.h"

static STATE_StateType stateVariable = STATE_INIT;

static void stateTask(void *param);

void stateInit(IMU_StatusType imuInitRetVal, MAG_StatusType magInitRetVal)
{
	if((IMU_OK == imuInitRetVal) && (MAG_OK == magInitRetVal))
	{
		xTaskCreate(&stateTask, "STATE", 512, NULL, STATE_PRIO, NULL);
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
	// TODO: calibration of sensor data, write state machine
}
