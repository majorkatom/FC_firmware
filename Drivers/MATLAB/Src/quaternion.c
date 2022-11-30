/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: quaternion.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Nov-2022 19:47:18
 */

/* Include Files */
#include "quaternion.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const float varargin_1[3]
 *                float *obj_a
 *                float *obj_b
 *                float *obj_c
 *                float *obj_d
 * Return Type  : void
 */
void quaternion_quaternion(const float varargin_1[3], float *obj_a,
                           float *obj_b, float *obj_c, float *obj_d)
{
  float st;
  float theta;
  *obj_a = 1.0F;
  *obj_b = 0.0F;
  *obj_c = 0.0F;
  *obj_d = 0.0F;
  theta = (float)sqrt(
      (varargin_1[0] * varargin_1[0] + varargin_1[1] * varargin_1[1]) +
      varargin_1[2] * varargin_1[2]);
  st = (float)sin(theta / 2.0F);
  if (theta != 0.0F) {
    *obj_a = (float)cos(theta / 2.0F);
    *obj_b = varargin_1[0] / theta * st;
    *obj_c = varargin_1[1] / theta * st;
    *obj_d = varargin_1[2] / theta * st;
  }
}

/*
 * File trailer for quaternion.c
 *
 * [EOF]
 */
