/*
 * control.h
 *
 *  Created on: 2022. dec. 2.
 *      Author: Tamas
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#define CTRL_PITCH_P 5483.2201F
#define CTRL_PITCH_D 13776.1302F
#define CTRL_ROLL_P 3950.6223F
#define CTRL_ROLL_D 9925.607F
#define CTRL_YAW_RATE_P 21364.4112F
#define CTRL_YAW_RATE_D 53676.2897F

#define CTRL_TIME_INTERVAL_MS 10U

#define CTRL_MIN_THROTTLE 100U
#define CTRL_IDLE_THROTTLE 200U
#define CTRL_MAX_THROTTLE 1600U
#define CTRL_MAX_PITCH 0.523598775F // 30°
#define CTRL_MAX_ROLL 0.523598775F // 30°
#define CTRL_MAX_YAW_RATE 0.785398163 //45°/s

typedef struct CTRL_OrientationDatatType_{
	float pitch;	// rad
	float roll;		// rad
	float yawRate;	// rad/s
} CTRL_OrientationDatatType;

typedef struct CTRL_MotorValsType_ {
	uint16_t motor1;
	uint16_t motor2;
	uint16_t motor3;
	uint16_t motor4;
} CTRL_MotorValsType;

void ctrlGetMotorVals2Set(CTRL_MotorValsType *motorVals, CTRL_OrientationDatatType orientation, CTRL_OrientationDatatType prevOrientation, RADIO_ChannelsType radioChannels);

#endif /* INC_CONTROL_H_ */
