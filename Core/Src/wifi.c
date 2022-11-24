/*
 * wifi.c
 *
 *  Created on: Nov 22, 2022
 *      Author: Tamas
 */

#include "bsp.h"

extern UART_HandleTypeDef huart2;
static uint8_t wifiEnabled = WIFI_ENABLE;
static WIFI_CircularBuffType *wifiCircularBufferPtr;
static SemaphoreHandle_t wifiCircBuffLock;
static uint8_t wifiMessageCntr = 0u;
static TaskHandle_t wifiSendTaskHandle = NULL;

static void wifiSendTask(void *param);

WIFI_StatusType wifiInit()
{
	vTaskDelay(20);
	WIFI_StatusType retVal = WIFI_DISABLED;
	if(0u != WIFI_ENABLE)
	{
		static WIFI_CircularBuffType wifiCircularBuffer = {
				.head = 0u,
				.tail = 0u,
				.full = 0u,
				.empty = 1u,
		};
		wifiCircularBufferPtr = &wifiCircularBuffer;

		wifiCircBuffLock = xSemaphoreCreateMutex();
		xSemaphoreGive(wifiCircBuffLock);
		xTaskCreate(&wifiSendTask, "WIFI_SEND", 256, NULL, WIFI_SEND_PRIO, &wifiSendTaskHandle);

		retVal = WIFI_OK;
	}

	return retVal;
}

WIFI_StatusType wifiPutMessage(WIFI_MessageType MsgTyp, uint8_t *data, uint8_t dataLen)
{
	WIFI_StatusType retVal = WIFI_DISABLED;
	if(0u != wifiEnabled)
	{
		if(pdTRUE == xSemaphoreTake(wifiCircBuffLock, 5))
		{
			if(0u == wifiCircularBufferPtr->full)
			{
				wifiCircularBufferPtr->buffer[wifiCircularBufferPtr->head][0] = wifiMessageCntr;
				wifiCircularBufferPtr->buffer[wifiCircularBufferPtr->head][1] = (uint8_t)MsgTyp;
				for(uint8_t i = 0u; i < dataLen; i++)
				{
					wifiCircularBufferPtr->buffer[wifiCircularBufferPtr->head][i + 2u] = data[i];
				}

				wifiCircularBufferPtr->head = (wifiCircularBufferPtr->head + 1) % WIFI_CIRC_BUFF_SIZE;
				wifiCircularBufferPtr->empty = 0u;

				if (wifiCircularBufferPtr->head == wifiCircularBufferPtr->tail)
				{
					wifiCircularBufferPtr->full = 1u;
				}

				wifiMessageCntr = (wifiMessageCntr + 1) % 256;

				retVal = WIFI_OK;
			}
			else
			{
				retVal = WIFI_FULL;
			}

			xSemaphoreGive(wifiCircBuffLock);
		}
		else
		{
			retVal = WIFI_TIMEOUT;
		}
	}


	return retVal;
}

void wifiUartTxCpltCallback(UART_HandleTypeDef * huart)
{
	if(huart == &huart2)
	{
		wifiCircularBufferPtr->tail = (wifiCircularBufferPtr->tail + 1u) % WIFI_CIRC_BUFF_SIZE;
		wifiCircularBufferPtr->full = 0u;
		if(wifiCircularBufferPtr->tail == wifiCircularBufferPtr->head)
		{
			wifiCircularBufferPtr->empty = 1u;
		}
		BaseType_t higherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(wifiSendTaskHandle, &higherPriorityTaskWoken);
		portYIELD_FROM_ISR(higherPriorityTaskWoken);
	}
}

static void wifiSendTask(void *param)
{
	while(1)
	{
		if(pdTRUE == xSemaphoreTake(wifiCircBuffLock, 200))
		{
			if(0u == wifiCircularBufferPtr->empty)
			{
				HAL_UART_Transmit_DMA(&huart2, wifiCircularBufferPtr->buffer[wifiCircularBufferPtr->tail], WIFI_MESSAGE_SIZE);
				uint32_t notified = ulTaskNotifyTake(pdTRUE, 8);
				if(0u != notified)
				{
					xSemaphoreGive(wifiCircBuffLock);
				}
				else
				{
					wifiEnabled = 0u;
					xSemaphoreGive(wifiCircBuffLock);
				}
			}
			else
			{
				xSemaphoreGive(wifiCircBuffLock);
			}
		}
		vTaskDelay(1);
	}
}
