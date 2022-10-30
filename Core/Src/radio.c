/*
 * radio.c
 *
 *  Created on: 2022. okt. 30.
 *      Author: Tamas
 */

#include "bsp.h"

SemaphoreHandle_t radioSemaphore;
RADIO_MessageType radioMessage;

uint16_t testVal = 0u;

static RADIO_StatusType radioStartReceive(uint16_t *channelArray);

void radioInit()
{
	radioMessage.dataReceived = 0u;
	radioSemaphore = xSemaphoreCreateBinary();
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
			BaseType_t semaphoreTakeRet = xSemaphoreTake(radioSemaphore, 5);
			if(pdTRUE == semaphoreTakeRet)
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
				retVal = RADIO_SEMAPHORE_TIMEOUT_ERROR;
			}
		}
	}

	return retVal;
}

void radioUartRxCpltCallback()
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
		xSemaphoreGiveFromISR(radioSemaphore, &higherPriorityTaskWoken);
		portYIELD_FROM_ISR(higherPriorityTaskWoken);
	}
}

void radioReceiveTask(void *param)
{
	while(1)
	{
		uint16_t radioChannels[RADIO_CH_NUM];
		RADIO_StatusType radioRetVal = radioStartReceive(radioChannels);
		if(RADIO_OK == radioRetVal)
		{

		}
		vTaskDelay(7);
	}
}
