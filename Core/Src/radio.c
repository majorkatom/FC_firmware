/*
 * radio.c
 *
 *  Created on: 2022. okt. 30.
 *      Author: Tamas
 */

#include "bsp.h"

extern UART_HandleTypeDef huart3;
static uint16_t *radioChannelsPtr = NULL;
static SemaphoreHandle_t radioChannelsSemaphore;
static RADIO_MessageType radioMessage;
static TaskHandle_t radioReceiveTaskHandle = NULL;

static RADIO_StatusType radioStartReceive(uint16_t *channelArray);
static void radioReceiveTask(void *param);

void radioInit()
{
	radioMessage.dataReceived = 0u;
	radioChannelsSemaphore = xSemaphoreCreateMutex();
	xSemaphoreGive(radioChannelsSemaphore);
	xTaskCreate(&radioReceiveTask, "RADIO_RECEIVE", 512, NULL, RADIO_RECEIVE_PRIO, &radioReceiveTaskHandle);
}

static RADIO_StatusType radioStartReceive(uint16_t *channelArray)
{
	RADIO_StatusType retVal = RADIO_OK;
	HAL_StatusTypeDef uartRet = HAL_UART_Receive_IT(&huart3, &(radioMessage.length), 1);
	if(HAL_OK != uartRet)
	{
		retVal = RADIO_UART_ERROR;
	}
	else
	{
		if(32 !=radioMessage.length)
		{
			retVal = RADIO_LENGTH_ERROR;
		}
		else
		{
			uint32_t notified = ulTaskNotifyTake(pdTRUE, 3);
			if(0u != notified)
			{
				radioMessage.msgType = radioMessage.rxBuffer[0];
				if(0x40 == radioMessage.msgType)
				{
					for(uint8_t i = 0u;i < RADIO_CH_NUM;i++)
					{
						channelArray[i] = radioMessage.rxBuffer[i * 2 + 2] << 8;
						channelArray[i] |= radioMessage.rxBuffer[i * 2 + 1];
					}

					radioMessage.checkSum = radioMessage.rxBuffer[radioMessage.length - 2] << 8;
					radioMessage.checkSum |= radioMessage.rxBuffer[radioMessage.length - 3];
					radioMessage.checkSum += radioMessage.length;
					for(uint8_t i = 0u;i < radioMessage.length - 3;i++)
					{
						radioMessage.checkSum += radioMessage.rxBuffer[i];
					}

					if(0xFFFF != radioMessage.checkSum)
					{
						retVal = RADIO_CHECKSUM_ERROR;
					}
				}
				else
				{
					retVal = RADIO_MSG_TYPE_ERROR;
				}
			}
			else
			{
				retVal = RADIO_TIMEOUT;
			}
		}
	}

	return retVal;
}

void radioUartRxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart3)
	{
		if(!radioMessage.dataReceived)
		{
			if(radioMessage.length == 32u)
			{
				HAL_UART_Receive_IT(&huart3, radioMessage.rxBuffer, radioMessage.length - 1);
				radioMessage.dataReceived = 1u;
			}
		}
		else
		{
			radioMessage.dataReceived = 0u;
			BaseType_t higherPriorityTaskWoken = pdFALSE;
			vTaskNotifyGiveFromISR(radioReceiveTaskHandle, &higherPriorityTaskWoken);
			portYIELD_FROM_ISR(higherPriorityTaskWoken);
		}
	}
}

static void radioReceiveTask(void *param)
{
	static uint16_t radioChannelsReceiveArray[RADIO_CH_NUM];
	uint16_t noDataDebounceCntr = 0u;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, 6);

		if (50u > noDataDebounceCntr || STATE_DISARMED == stateGetMainState())
		{
			if(pdTRUE == xSemaphoreTake(radioChannelsSemaphore, 3))
			{
				RADIO_StatusType retValReceive = radioStartReceive(radioChannelsReceiveArray);
				if(RADIO_OK == retValReceive)
				{
					noDataDebounceCntr = 0u;
					if((RADIO_MIN_CH_VAL == radioChannelsReceiveArray[RADIO_CH_IDX_ARM_SWC]) && (STATE_ARMED == stateGetMainState()))
					{
						stateSetState(STATE_DISARMED, STATE_NORMAL_OPERATION);
					}
					radioChannelsPtr = radioChannelsReceiveArray;
				}
				else
				{
					noDataDebounceCntr++;
					radioChannelsPtr = NULL;
				}
				xSemaphoreGive(radioChannelsSemaphore);
			}
		}
		else if(STATE_ARMED == stateGetMainState())
		{
			escSetMotorVals(0, 0, 0, 0);
			stateSetState(STATE_DISARMED, STATE_RADIO_ERROR);
		}
	}
}

RADIO_StatusType radioReadData(RADIO_ChannelsType *radioChannels)
{
	RADIO_StatusType retVal = RADIO_TIMEOUT;
	if(pdTRUE == xSemaphoreTake(radioChannelsSemaphore, 5))
	{
		if(NULL != radioChannelsPtr)
		{
			radioChannels->roll = radioChannelsPtr[RADIO_CH_IDX_ROLL];
			radioChannels->pitch = radioChannelsPtr[RADIO_CH_IDX_PITCH];
			radioChannels->throttle = radioChannelsPtr[RADIO_CH_IDX_THROTTLE];
			radioChannels->yaw = radioChannelsPtr[RADIO_CH_IDX_YAW];
			radioChannels->arm = radioChannelsPtr[RADIO_CH_IDX_ARM_SWC];

			retVal = RADIO_OK;
		}
		else
		{
			retVal = RADIO_NULL_PTR;
		}

		xSemaphoreGive(radioChannelsSemaphore);
	}

	return retVal;
}
