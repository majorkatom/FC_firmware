/*
 * esc.h
 *
 *  Created on: 2022. okt. 30.
 *      Author: Tamas
 */

#ifndef INC_ESC_H_
#define INC_ESC_H_

#define ESC_0_BIT (uint16_t)106U
#define ESC_1_BIT (uint16_t)212U
#define ESC_DMA_BUFF_SIZE (uint16_t)200U

typedef struct ESC_HandleType_ {
	uint16_t motorVal;
	uint16_t motorBuff[16];
	uint16_t dmaBuff[ESC_DMA_BUFF_SIZE];
} ESC_HandleType;

typedef enum ESC_HalfSelectorType_ {
	ESC_LOWER_HALF = 0,
	ESC_UPPER_HALF = 1
} ESC_HalfSelectorType;

void escInit();
void escSetMotorVals(uint16_t motorVal1, uint16_t motorVal2, uint16_t motorVal3, uint16_t motorVal4);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim);

#endif /* INC_ESC_H_ */
