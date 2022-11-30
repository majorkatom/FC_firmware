/*
 * state.h
 *
 *  Created on: 2022. nov. 30.
 *      Author: Tamas
 */

#ifndef INC_STATE_H_
#define INC_STATE_H_

typedef struct STATE_OrientationDatatType_{
	float pitch;	// rad
	float roll;		// rad
	float yawRate;	// rad/s
} STATE_OrientationDatatType;

typedef enum STATE_StateType_ {
	STATE_INIT,
	STATE_DISARMED,
	STATE_ARMED
} STATE_StateType;

void stateInit(IMU_StatusType imuInitRetVal, MAG_StatusType magInitRetVal);
STATE_StateType stateGetState();
void stateSetState(STATE_StateType stateToSet);

#endif /* INC_STATE_H_ */
