/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: AHRSFilterBase.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Nov-2022 19:47:18
 */

#ifndef AHRSFILTERBASE_H
#define AHRSFILTERBASE_H

/* Include Files */
#include "oriGetData_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void AHRSFilterBase_stepImpl(ahrsfilter *obj, const float accelIn[3],
                             const float gyroIn[3], const float magIn[3],
                             float *orientOut_a, float *orientOut_b,
                             float *orientOut_c, float *orientOut_d,
                             float av[3]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for AHRSFilterBase.h
 *
 * [EOF]
 */
