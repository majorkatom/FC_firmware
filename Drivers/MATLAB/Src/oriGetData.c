/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: oriGetData.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Nov-2022 19:47:18
 */

/* Include Files */
#include "oriGetData.h"
#include "AHRSFilterBase.h"
#include "oriGetData_data.h"
#include "oriGetData_internal_types.h"
#include "bsp.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const IMU_DataType *imuData
 *                const MAG_DataType *magData
 *                struct0_T *dataOut
 * Return Type  : void
 */
void oriGetData(const IMU_DataType *imuData, const MAG_DataType *magData,
		STATE_OrientationDatatType *dataOut)
{
  static const float fv[144] = {
      6.09234849E-6F, 0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           6.09234849E-6F, 0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           6.09234849E-6F, 0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           7.61543561E-5F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      7.61543561E-5F, 0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           7.61543561E-5F, 0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.00962361F,    0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.00962361F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.00962361F,    0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.6F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.6F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.0F,
      0.0F,           0.0F,           0.0F,           0.6F};
  ahrsfilter fuse;
  float b_imuData[3];
  float b_magData[3];
  float varargout_2[3];
  float b;
  float b_x;
  float c_x;
  float d_x;
  float n;
  float tmp;
  float varargout_1_a;
  float varargout_1_d;
  float x;
  int32_t i;
  int32_t i2;
  int8_t i1;
  fuse.MagneticDisturbanceNoise = 0.5;
  fuse.MagneticDisturbanceDecayFactor = 0.5;
  fuse.GyroscopeDriftNoise = 3.0462E-13;
  fuse.LinearAccelerationDecayFactor = 0.5;
  fuse.ExpectedMagneticFieldStrength = 58.9373515433;
  fuse.AccelerometerNoise = 0.0012689939;
  fuse.GyroscopeNoise = 3.069947716E-6;
  fuse.MagnetometerNoise = 0.23918104866865;
  fuse.LinearAccelerationNoise = 0.0005;
  fuse.pInputPrototype[0] = imuData->accel.x;
  fuse.pInputPrototype[1] = imuData->accel.y;
  fuse.pInputPrototype[2] = imuData->accel.z;
  fuse.isInitialized = 1;
  fuse.pSensorPeriod = 0.01;
  fuse.pKalmanPeriod = 0.01;
  fuse.TunablePropsChanged = false;
  fuse.pOrientPost.a = 1.0F;
  fuse.pOrientPost.b = 0.0F;
  fuse.pOrientPost.c = 0.0F;
  fuse.pOrientPost.d = 0.0F;
  fuse.pGyroOffset[0] = 0.0F;
  fuse.pMagVec[0] = 0.0F;
  fuse.pGyroOffset[1] = 0.0F;
  fuse.pGyroOffset[2] = 0.0F;
  fuse.pMagVec[2] = 0.0F;
  fuse.pMagVec[1] = 58.9373512F;
  memset(&fuse.pQv[0], 0, 36U * sizeof(double));
  for (i = 0; i < 3; i++) {
    i1 = iv[3 * i];
    fuse.pQv[6 * i] = 0.001768994206994802 * (double)i1;
    i2 = 6 * (i + 3);
    fuse.pQv[i2 + 3] = 0.73918104897564474 * (double)i1;
    i1 = iv[3 * i + 1];
    fuse.pQv[6 * i + 1] = 0.001768994206994802 * (double)i1;
    fuse.pQv[i2 + 4] = 0.73918104897564474 * (double)i1;
    i1 = iv[3 * i + 2];
    fuse.pQv[6 * i + 2] = 0.001768994206994802 * (double)i1;
    fuse.pQv[i2 + 5] = 0.73918104897564474 * (double)i1;
  }
  memcpy(&fuse.pQw[0], &fv[0], 144U * sizeof(float));
  fuse.pLinAccelPost[0] = 0.0F;
  fuse.pLinAccelPost[1] = 0.0F;
  fuse.pLinAccelPost[2] = 0.0F;
  fuse.pFirstTime = true;
  b_imuData[0] = imuData->gyro.x;
  b_imuData[1] = imuData->gyro.y;
  b_imuData[2] = imuData->gyro.z;
  b_magData[0] = magData->x;
  b_magData[1] = magData->y;
  b_magData[2] = magData->z;
  AHRSFilterBase_stepImpl(&fuse, fuse.pInputPrototype, b_imuData, b_magData,
                          &varargout_1_a, &tmp, &b, &varargout_1_d,
                          varargout_2);
  n = (float)sqrt(((varargout_1_a * varargout_1_a + tmp * tmp) + b * b) +
                  varargout_1_d * varargout_1_d);
  x = varargout_1_a;
  varargout_1_a /= n;
  b_x = tmp;
  tmp /= n;
  c_x = b;
  b /= n;
  d_x = varargout_1_d;
  varargout_1_d /= n;
  tmp = varargout_1_a * tmp * 2.0F + b * varargout_1_d * 2.0F;
  if (tmp > 1.0F) {
    tmp = 1.0F;
  }
  if (tmp < -1.0F) {
    tmp = -1.0F;
  }
  b = (float)asin(tmp);
  if (b >= 1.57079518F) {
    tmp = 0.0F;
  } else if (b <= -1.57079518F) {
    tmp = 0.0F;
  } else {
    tmp = (float)atan2(x / n * (c_x / n) * 2.0F - b_x / n * (d_x / n) * 2.0F,
                       (varargout_1_a * varargout_1_a * 2.0F - 1.0F) +
                           varargout_1_d * varargout_1_d * 2.0F);
  }
  dataOut->pitch = 0.0174532924F * (57.2957802F * b);
  dataOut->roll = 0.0174532924F * (57.2957802F * tmp);
  dataOut->yawRate = varargout_2[2];
}

/*
 * File trailer for oriGetData.c
 *
 * [EOF]
 */
