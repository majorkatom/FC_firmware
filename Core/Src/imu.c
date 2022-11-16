/*
 * imu.c
 *
 *  Created on: Nov 16, 2022
 *      Author: Tamas
 */

#include "bsp.h"
#include "bmi08x.h"

struct bmi08x_dev imuDev;
IMU_HandleType himu0;

static BMI08X_INTF_RET_TYPE bmi08x_read(uint8_t, uint8_t*, uint32_t, void*);
static BMI08X_INTF_RET_TYPE bmi08x_write(uint8_t, const uint8_t*, uint32_t, void*);
