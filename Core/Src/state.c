/*
 * state.c
 *
 *  Created on: 2022. nov. 30.
 *      Author: Tamas
 */

#include <math.h>
#include "bsp.h"
#include "oriGetData.h"

static STATE_StateType stateVariable = {.main = STATE_DISARMED, .sub = STATE_NORMAL_OPERATION};
static const float stateRad2Deg = 180 / M_PI;

static void stateTask(void *param);

void stateInit(IMU_StatusType imuInitRetVal, MAG_StatusType magInitRetVal)
{
	if((IMU_OK == imuInitRetVal) && (MAG_OK == magInitRetVal))
	{
		xTaskCreate(&stateTask, "STATE", 2048, NULL, STATE_PRIO, NULL);
	}
}

STATE_MainStateType stateGetMainState()
{
	return stateVariable.main;
}

STATE_SubStateType stateGetSubState()
{
	return stateVariable.sub;
}

void stateSetState(STATE_MainStateType mainState, STATE_SubStateType subState)
{
	stateVariable.main = mainState;
	stateVariable.sub = subState;
}

static void stateTask(void *param)
{
	RADIO_ChannelsType radioChannels = {.roll = 1500u, .pitch = 1500u, .throttle = 1000u, .yaw = 1500u, .arm = 0u};
	IMU_DataType imuData;
	MAG_DataType magData;
	CTRL_OrientationDatatType orientation;
	CTRL_OrientationDatatType orientationPrev = {.pitch = 0.0f, .roll = 0.0f, .yawRate = 0.0f};
	CTRL_MotorValsType motorVals;
	uint8_t radioTxInSafeState = 0u;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, CTRL_TIME_INTERVAL_MS);

		switch(stateVariable.main)
		{

			case STATE_DISARMED:
			{
				escSetMotorVals(0, 0, 0, 0);

				switch(stateVariable.sub)
				{
					case STATE_NORMAL_OPERATION:
					{
						RADIO_StatusType radioRetVal = radioReadData(&radioChannels);
						if(RADIO_OK == radioRetVal)
						{
							if(0u == radioTxInSafeState)
							{
								if((RADIO_MIN_CH_VAL == radioChannels.arm) && ((RADIO_MIN_CH_VAL + 15u) >= radioChannels.throttle))
								{
									radioTxInSafeState = 1u;
								}
							}
							else
							{
								if((RADIO_MIN_CH_VAL == radioChannels.arm) && ((RADIO_MIN_CH_VAL + 15u) < radioChannels.throttle))
								{
									radioTxInSafeState = 0u;
								}
								else if((RADIO_MAX_CH_VAL == radioChannels.arm) && ((RADIO_MIN_CH_VAL + 15u) >= radioChannels.throttle))
								{
									radioTxInSafeState = 0u;
									stateVariable.main = STATE_ARMED;
								}
							}
						}

						break;
					}
					case STATE_ACC_ERROR:
					{
						vTaskDelete(NULL);
						break;
					}
					case STATE_GYRO_ERROR:
					{
						vTaskDelete(NULL);
						break;
					}
					case STATE_MAG_ERROR:
					{
						vTaskDelete(NULL);
						break;
					}
					case STATE_RADIO_ERROR:
					{
						vTaskDelete(NULL);
						break;
					}
				}

				break;
			}
			case STATE_ARMED:
			{
				switch(stateVariable.sub)
				{
					case STATE_NORMAL_OPERATION:
					{
						IMU_StatusType imuReadRetVal = imuReadData(&imuData);
						MAG_StatusType magReadRetVal = magReadData(&magData);
						radioReadData(&radioChannels);

						if(RADIO_MAX_CH_VAL != radioChannels.arm)
						{
							escSetMotorVals(0, 0, 0, 0);
							stateVariable.main = STATE_DISARMED;
						}

						if((IMU_OK == imuReadRetVal) && (MAG_OK == magReadRetVal))
						{
							oriGetData(&imuData, &magData, &orientation);
							ctrlGetMotorVals2Set(&motorVals, orientation, orientationPrev, radioChannels);
							escSetMotorVals(motorVals.motor1, motorVals.motor2, motorVals.motor3, motorVals.motor4);

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

						break;
					}
					case STATE_ACC_ERROR:
						escSetMotorVals(0, 0, 0, 0);
						stateVariable.main = STATE_DISARMED;
						break;
					case STATE_GYRO_ERROR:
						escSetMotorVals(0, 0, 0, 0);
						stateVariable.main = STATE_DISARMED;
						break;
					case STATE_MAG_ERROR:
						escSetMotorVals(0, 0, 0, 0);
						stateVariable.main = STATE_DISARMED;
						break;
					case STATE_RADIO_ERROR:
						escSetMotorVals(0, 0, 0, 0);
						stateVariable.main = STATE_DISARMED;
						break;
				}

				break;
			}
		}

		orientationPrev.pitch = orientation.pitch;
		orientationPrev.roll = orientation.roll;
		orientationPrev.yawRate = orientation.yawRate;
	}
}
