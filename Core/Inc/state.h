/*
 * state.h
 *
 *  Created on: 2022. nov. 30.
 *      Author: Tamas
 */

#ifndef INC_STATE_H_
#define INC_STATE_H_

typedef enum STATE_MainStateType_ {
	STATE_DISARMED,
	STATE_ARMED
} STATE_MainStateType;

typedef enum STATE_SubStateType_ {
	STATE_NORMAL_OPERATION,
	STATE_ACC_ERROR,
	STATE_GYRO_ERROR,
	STATE_MAG_ERROR,
	STATE_RADIO_ERROR
} STATE_SubStateType;

typedef struct STATE_StateType_ {
	STATE_MainStateType main;
	STATE_SubStateType sub;
} STATE_StateType;

void stateInit(IMU_StatusType imuInitRetVal, MAG_StatusType magInitRetVal);
STATE_MainStateType stateGetMainState();
STATE_SubStateType stateGetSubState();
void stateSetState(STATE_MainStateType mainState, STATE_SubStateType subState);

#endif /* INC_STATE_H_ */
