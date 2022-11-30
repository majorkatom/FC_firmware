/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: oriGetData_internal_types.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Nov-2022 19:47:18
 */

#ifndef ORIGETDATA_INTERNAL_TYPES_H
#define ORIGETDATA_INTERNAL_TYPES_H

/* Include Files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_quaternion
#define typedef_quaternion
typedef struct {
  float a;
  float b;
  float c;
  float d;
} quaternion;
#endif /* typedef_quaternion */

#ifndef typedef_cell_wrap_3
#define typedef_cell_wrap_3
typedef struct {
  uint32_t f1[8];
} cell_wrap_3;
#endif /* typedef_cell_wrap_3 */

#ifndef typedef_ahrsfilter
#define typedef_ahrsfilter
typedef struct {
  int32_t isInitialized;
  boolean_T TunablePropsChanged;
  cell_wrap_3 inputVarSize[3];
  double AccelerometerNoise;
  double GyroscopeNoise;
  double GyroscopeDriftNoise;
  double LinearAccelerationNoise;
  double LinearAccelerationDecayFactor;
  float pQw[144];
  double pQv[36];
  quaternion pOrientPost;
  quaternion pOrientPrior;
  boolean_T pFirstTime;
  double pSensorPeriod;
  double pKalmanPeriod;
  float pGyroOffset[3];
  float pLinAccelPrior[3];
  float pLinAccelPost[3];
  float pInputPrototype[3];
  double MagnetometerNoise;
  double MagneticDisturbanceNoise;
  double MagneticDisturbanceDecayFactor;
  double ExpectedMagneticFieldStrength;
  float pMagVec[3];
} ahrsfilter;
#endif /* typedef_ahrsfilter */

#endif
/*
 * File trailer for oriGetData_internal_types.h
 *
 * [EOF]
 */
