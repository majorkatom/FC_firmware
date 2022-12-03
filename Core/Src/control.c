/*
 * control.c
 *
 *  Created on: 2022. dec. 2.
 *      Author: Tamas
 */

#include <math.h>
#include "bsp.h"

void ctrlGetMotorVals2Set(CTRL_MotorValsType *motorVals, CTRL_OrientationDatatType orientation, CTRL_OrientationDatatType prevOrientation, RADIO_ChannelsType radioChannels)
{
	float radioChRangeRecipr = 1.0 / (float)(RADIO_MAX_CH_VAL - RADIO_MIN_CH_VAL);
	float ctrlTimeRecipr = (float)(CTRL_TIME_INTERVAL_MS) / 1000.0;

	uint16_t desiredThrottle = (uint16_t)(CTRL_IDLE_THROTTLE + (radioChannels.throttle - RADIO_MIN_CH_VAL) * (CTRL_MAX_THROTTLE - CTRL_IDLE_THROTTLE) * radioChRangeRecipr);
	CTRL_OrientationDatatType desiredOri;
	desiredOri.pitch = -(-CTRL_MAX_PITCH + (radioChannels.pitch - RADIO_MIN_CH_VAL)* 2 * CTRL_MAX_PITCH * radioChRangeRecipr);
	desiredOri.roll = -CTRL_MAX_ROLL + (radioChannels.roll - RADIO_MIN_CH_VAL)* 2 * CTRL_MAX_ROLL * radioChRangeRecipr;
	desiredOri.yawRate = -(-CTRL_MAX_YAW_RATE + (radioChannels.yaw - RADIO_MIN_CH_VAL)* 2 * CTRL_MAX_YAW_RATE * radioChRangeRecipr);

	float pitchCompensationSquare = CTRL_PITCH_P * (desiredOri.pitch - orientation.pitch) + CTRL_PITCH_D * ctrlTimeRecipr * (orientation.pitch - prevOrientation.pitch);
	float rollCompensationSquare = CTRL_ROLL_P * (desiredOri.roll - orientation.roll) + CTRL_ROLL_D * ctrlTimeRecipr * (orientation.roll - prevOrientation.roll);
	float yawRateCompensationSquare = CTRL_YAW_RATE_P * (desiredOri.yawRate - orientation.yawRate) + CTRL_YAW_RATE_D * ctrlTimeRecipr * (orientation.yawRate - prevOrientation.yawRate);

	int16_t pitchCompensation = 0.0f <= pitchCompensationSquare ? (int16_t)sqrt(pitchCompensationSquare) : (int16_t)(-sqrt(-pitchCompensationSquare));
	int16_t rollCompensation = 0.0f <= rollCompensationSquare ? (int16_t)sqrt(rollCompensationSquare) : (int16_t)(-sqrt(-rollCompensationSquare));
	int16_t yawRateCompensation = 0.0f <= yawRateCompensationSquare ? (int16_t)sqrt(yawRateCompensationSquare) : (int16_t)(-sqrt(-yawRateCompensationSquare));

	uint16_t motor1Val = (uint16_t)((int16_t)desiredThrottle - pitchCompensation - rollCompensation - yawRateCompensation);
	uint16_t motor2Val = (uint16_t)((int16_t)desiredThrottle + pitchCompensation - rollCompensation + yawRateCompensation);
	uint16_t motor3Val = (uint16_t)((int16_t)desiredThrottle - pitchCompensation + rollCompensation + yawRateCompensation);
	uint16_t motor4Val = (uint16_t)((int16_t)desiredThrottle + pitchCompensation + rollCompensation - yawRateCompensation);

	motorVals->motor1 = CTRL_IDLE_THROTTLE < motor1Val ? motor1Val : CTRL_IDLE_THROTTLE;
	motorVals->motor2 = CTRL_IDLE_THROTTLE < motor2Val ? motor2Val : CTRL_IDLE_THROTTLE;
	motorVals->motor3 = CTRL_IDLE_THROTTLE < motor3Val ? motor3Val : CTRL_IDLE_THROTTLE;
	motorVals->motor4 = CTRL_IDLE_THROTTLE < motor4Val ? motor4Val : CTRL_IDLE_THROTTLE;
}
