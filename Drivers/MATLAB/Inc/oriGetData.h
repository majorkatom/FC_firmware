/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: oriGetData.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Nov-2022 19:47:18
 */

#ifndef ORIGETDATA_H
#define ORIGETDATA_H

/* Include Files */
#include "oriGetData_types.h"
#include "rtwtypes.h"
#include "bsp.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void oriGetData(const IMU_DataType *imuData, const MAG_DataType *magData,
                       struct0_T *dataOut);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for oriGetData.h
 *
 * [EOF]
 */
