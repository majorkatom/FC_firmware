/*
 * esc.c
 *
 *  Created on: 2022. okt. 30.
 *      Author: Tamas
 */

#include "bsp.h"

extern TIM_HandleTypeDef htim1;
SemaphoreHandle_t escMotorValsChangedSemaphores[4];
ESC_HandleType hesc0[4];

static uint16_t escMotorVal2Packet(uint16_t motorVal);
static void escMotorBuffFill(uint16_t data, uint16_t *buffer);
static void escDmaHalfBuffFill(uint16_t *motorBuff, uint16_t *dmaBuffer, uint16_t dmaBuffSize, ESC_HalfSelectorType halfSelector);

void escInit()
{
	for(uint8_t i = 0u;i < 4u;i++)
	{
		hesc0[i].motorVal = 0u;
		escMotorBuffFill(escMotorVal2Packet(hesc0[i].motorVal), hesc0[i].motorBuff);
		escDmaHalfBuffFill(hesc0[i].motorBuff, hesc0[i].dmaBuff, ESC_DMA_BUFF_SIZE, ESC_LOWER_HALF);
		escDmaHalfBuffFill(hesc0[i].motorBuff, hesc0[i].dmaBuff, ESC_DMA_BUFF_SIZE, ESC_UPPER_HALF);
		escMotorValsChangedSemaphores[i] = xSemaphoreCreateCounting(2, 2);
	}

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)hesc0[0].dmaBuff, ESC_DMA_BUFF_SIZE);
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t *)hesc0[1].dmaBuff, ESC_DMA_BUFF_SIZE);
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t *)hesc0[2].dmaBuff, ESC_DMA_BUFF_SIZE);
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_4, (uint32_t *)hesc0[3].dmaBuff, ESC_DMA_BUFF_SIZE);
}

static uint16_t escMotorVal2Packet(uint16_t motorVal)
{
	uint16_t packet = 0u;
	uint16_t crc = 0u;
	if(2000u < motorVal)
	{
		motorVal = 2000u;
	}
	if(motorVal)
	{
		motorVal += 47u;
	}
	packet = motorVal << 1;
	crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
	packet = (packet << 4) | crc;

	return packet;
}


static void escMotorBuffFill(uint16_t data, uint16_t *buffer)
{
	uint8_t i = 0u;
	while(i < 16u)
	{
		if(data & (0x8000 >> i))
		{
			buffer[i] = ESC_1_BIT;
		}
		else
		{
			buffer[i] = ESC_0_BIT;
		}
		i++;
	}
}

static void escDmaHalfBuffFill(uint16_t *motorBuff, uint16_t *dmaBuffer, uint16_t dmaBuffSize, ESC_HalfSelectorType halfSelector)
{
	uint16_t offset = dmaBuffSize / 2;
	for(uint8_t i = 0u;i < 16u;i++)
	{
		dmaBuffer[halfSelector * offset + i] = motorBuff[i];
	}
}

void escSetMotorVals(uint16_t motorVal1, uint16_t motorVal2, uint16_t motorVal3, uint16_t motorVal4)
{
	uint16_t motorVals[4] = {motorVal1, motorVal2, motorVal3, motorVal4};
	for(uint8_t i = 0u;i < 4u;i++)
	{
		hesc0[i].motorVal = motorVals[i];
		escMotorBuffFill(escMotorVal2Packet(hesc0[i].motorVal), hesc0[i].motorBuff);
		if(pdTRUE == xSemaphoreTake(escMotorValsChangedSemaphores[i], 5))
		{
			xSemaphoreTake(escMotorValsChangedSemaphores[i], 5);
		}
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(&htim1 == htim)
	{
		switch (htim->Channel) {
			case HAL_TIM_ACTIVE_CHANNEL_1:
			{
				BaseType_t higherPriorityTaskWoken = pdFALSE;
				if(pdTRUE == xSemaphoreGiveFromISR(escMotorValsChangedSemaphores[0], &higherPriorityTaskWoken))
				{
					escDmaHalfBuffFill(hesc0[0].motorBuff, hesc0[0].dmaBuff, ESC_DMA_BUFF_SIZE, ESC_UPPER_HALF);
				}
				portYIELD_FROM_ISR(higherPriorityTaskWoken);
			}
				break;
			case HAL_TIM_ACTIVE_CHANNEL_2:
			{
				BaseType_t higherPriorityTaskWoken = pdFALSE;
				if(pdTRUE == xSemaphoreGiveFromISR(escMotorValsChangedSemaphores[1], &higherPriorityTaskWoken))
				{
					escDmaHalfBuffFill(hesc0[1].motorBuff, hesc0[1].dmaBuff, ESC_DMA_BUFF_SIZE, ESC_UPPER_HALF);
				}
				portYIELD_FROM_ISR(higherPriorityTaskWoken);
			}
				break;
			case HAL_TIM_ACTIVE_CHANNEL_3:
			{
				BaseType_t higherPriorityTaskWoken = pdFALSE;
				if(pdTRUE == xSemaphoreGiveFromISR(escMotorValsChangedSemaphores[2], &higherPriorityTaskWoken))
				{
					escDmaHalfBuffFill(hesc0[2].motorBuff, hesc0[2].dmaBuff, ESC_DMA_BUFF_SIZE, ESC_UPPER_HALF);
				}
				portYIELD_FROM_ISR(higherPriorityTaskWoken);
			}
				break;
			case HAL_TIM_ACTIVE_CHANNEL_4:
			{
				BaseType_t higherPriorityTaskWoken = pdFALSE;
				if(pdTRUE == xSemaphoreGiveFromISR(escMotorValsChangedSemaphores[3], &higherPriorityTaskWoken))
				{
					escDmaHalfBuffFill(hesc0[3].motorBuff, hesc0[3].dmaBuff, ESC_DMA_BUFF_SIZE, ESC_UPPER_HALF);
				}
				portYIELD_FROM_ISR(higherPriorityTaskWoken);
			}
				break;
			default:
				break;
		}
	}
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
	if(&htim1 == htim)
	{
		switch (htim->Channel) {
			case HAL_TIM_ACTIVE_CHANNEL_1:
			{
				BaseType_t higherPriorityTaskWoken = pdFALSE;
				if(pdTRUE == xSemaphoreGiveFromISR(escMotorValsChangedSemaphores[0], &higherPriorityTaskWoken))
				{
					escDmaHalfBuffFill(hesc0[0].motorBuff, hesc0[0].dmaBuff, ESC_DMA_BUFF_SIZE, ESC_LOWER_HALF);
				}
				portYIELD_FROM_ISR(higherPriorityTaskWoken);
			}
				break;
			case HAL_TIM_ACTIVE_CHANNEL_2:
			{
				BaseType_t higherPriorityTaskWoken = pdFALSE;
				if(pdTRUE == xSemaphoreGiveFromISR(escMotorValsChangedSemaphores[1], &higherPriorityTaskWoken))
				{
					escDmaHalfBuffFill(hesc0[1].motorBuff, hesc0[1].dmaBuff, ESC_DMA_BUFF_SIZE, ESC_LOWER_HALF);
				}
				portYIELD_FROM_ISR(higherPriorityTaskWoken);
			}
				break;
			case HAL_TIM_ACTIVE_CHANNEL_3:
			{
				BaseType_t higherPriorityTaskWoken = pdFALSE;
				if(pdTRUE == xSemaphoreGiveFromISR(escMotorValsChangedSemaphores[2], &higherPriorityTaskWoken))
				{
					escDmaHalfBuffFill(hesc0[2].motorBuff, hesc0[2].dmaBuff, ESC_DMA_BUFF_SIZE, ESC_LOWER_HALF);
				}
				portYIELD_FROM_ISR(higherPriorityTaskWoken);
			}
				break;
			case HAL_TIM_ACTIVE_CHANNEL_4:
			{
				BaseType_t higherPriorityTaskWoken = pdFALSE;
				if(pdTRUE == xSemaphoreGiveFromISR(escMotorValsChangedSemaphores[3], &higherPriorityTaskWoken))
				{
					escDmaHalfBuffFill(hesc0[3].motorBuff, hesc0[3].dmaBuff, ESC_DMA_BUFF_SIZE, ESC_LOWER_HALF);
				}
				portYIELD_FROM_ISR(higherPriorityTaskWoken);
			}
				break;
			default:
				break;
		}
	}
}
