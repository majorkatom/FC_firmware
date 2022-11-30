/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: rotmat.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Nov-2022 19:47:18
 */

/* Include Files */
#include "rotmat.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : float q_a
 *                float q_b
 *                float q_c
 *                float q_d
 *                float r[9]
 * Return Type  : void
 */
void quaternionBase_rotmat(float q_a, float q_b, float q_c, float q_d,
                           float r[9])
{
  float aasq;
  float ab2;
  float ad2;
  float bc2;
  float bd2;
  float cd2;
  float n;
  float tb;
  float tc;
  float td;
  n = (float)sqrt(((q_a * q_a + q_b * q_b) + q_c * q_c) + q_d * q_d);
  q_a /= n;
  td = q_b;
  q_b /= n;
  ab2 = q_c;
  q_c /= n;
  ad2 = q_d;
  q_d /= n;
  tb = td / n;
  tc = ab2 / n;
  td = ad2 / n;
  ab2 = q_a * q_b * 2.0F;
  n = q_a * q_c * 2.0F;
  ad2 = q_a * q_d * 2.0F;
  bc2 = q_b * q_c * 2.0F;
  bd2 = q_b * q_d * 2.0F;
  cd2 = q_c * q_d * 2.0F;
  aasq = q_a * q_a * 2.0F - 1.0F;
  r[0] = aasq + tb * tb * 2.0F;
  r[3] = bc2 + ad2;
  r[6] = bd2 - n;
  r[1] = bc2 - ad2;
  r[4] = aasq + tc * tc * 2.0F;
  r[7] = cd2 + ab2;
  r[2] = bd2 + n;
  r[5] = cd2 - ab2;
  r[8] = aasq + td * td * 2.0F;
}

/*
 * File trailer for rotmat.c
 *
 * [EOF]
 */
