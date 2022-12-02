/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: AHRSFilterBase.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Nov-2022 19:47:18
 */

/* Include Files */
#include "AHRSFilterBase.h"
#include "mrdivide_helper.h"
#include "oriGetData_data.h"
#include "oriGetData_internal_types.h"
#include "quaternion.h"
#include "rotmat.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : ahrsfilter *obj
 *                const float accelIn[3]
 *                const float gyroIn[3]
 *                const float magIn[3]
 *                float *orientOut_a
 *                float *orientOut_b
 *                float *orientOut_c
 *                float *orientOut_d
 *                float av[3]
 * Return Type  : void
 */
void AHRSFilterBase_stepImpl(ahrsfilter *obj, const float accelIn[3],
                             const float gyroIn[3], const float magIn[3],
                             float *orientOut_a, float *orientOut_b,
                             float *orientOut_c, float *orientOut_d,
                             float av[3])
{
  static const int8_t b_iv[9] = {-1, 0, 0, 0, -1, 0, 0, 0, -1};
  quaternion qerr;
  float a;
  float a_tmp_tmp;
  float Ppost[144];
  float H[72];
  float b_tmp_tmp[72];
  float y_tmp[72];
  float tmp_tmp[36];
  float xe_post[12];
  float Rpost[9];
  float Rprior[9];
  float b_h2[9];
  float h2[9];
  float ze[6];
  float Rup[3];
  float gravityAccelGyroDiff[3];
  float magDistErr[3];
  float Reast_idx_0;
  float Reast_idx_1;
  float Reast_idx_2;
  float deltaq_a;
  float deltaq_b;
  float deltaq_c;
  float deltaq_d;
  float pd;
  int32_t H_tmp;
  int32_t i;
  int32_t i1;
  int32_t xi;
  int32_t xpageoffset;
  boolean_T isJamming;
  av[0] = gyroIn[0] - obj->pGyroOffset[0];
  av[1] = gyroIn[1] - obj->pGyroOffset[1];
  av[2] = gyroIn[2] - obj->pGyroOffset[2];
  if (obj->pFirstTime) {
    Rpost[6] = -accelIn[0];
    Rpost[7] = -accelIn[1];
    Rpost[8] = -accelIn[2];
    Reast_idx_0 = magIn[1] * -accelIn[2] - -accelIn[1] * magIn[2];
    Reast_idx_1 = -accelIn[0] * magIn[2] - magIn[0] * -accelIn[2];
    Reast_idx_2 = magIn[0] * -accelIn[1] - -accelIn[0] * magIn[1];
    Rpost[3] = -accelIn[1] * Reast_idx_2 - Reast_idx_1 * -accelIn[2];
    Rpost[4] = Reast_idx_0 * -accelIn[2] - -accelIn[0] * Reast_idx_2;
    Rpost[5] = -accelIn[0] * Reast_idx_1 - Reast_idx_0 * -accelIn[1];
    Rpost[0] = Reast_idx_0;
    Rpost[1] = Reast_idx_1;
    Rpost[2] = Reast_idx_2;
    for (i = 0; i < 9; i++) {
      deltaq_c = Rpost[i];
      Rprior[i] = deltaq_c * deltaq_c;
    }
    for (xi = 0; xi < 3; xi++) {
      xpageoffset = xi * 3;
      Rup[xi] = (float)sqrt((Rprior[xpageoffset] + Rprior[xpageoffset + 1]) +
                            Rprior[xpageoffset + 2]);
    }
    for (i = 0; i < 9; i++) {
      Rprior[i] = Rpost[i];
    }
    for (xi = 0; xi < 3; xi++) {
      deltaq_c = Rup[xi];
      Rpost[3 * xi] = Rprior[3 * xi] / deltaq_c;
      xpageoffset = 3 * xi + 1;
      Rpost[xpageoffset] = Rprior[xpageoffset] / deltaq_c;
      xpageoffset = 3 * xi + 2;
      Rpost[xpageoffset] = Rprior[xpageoffset] / deltaq_c;
    }
    obj->pFirstTime = false;
    pd = (Rpost[0] + Rpost[4]) + Rpost[8];
    Reast_idx_0 = (2.0F * pd + 1.0F) - pd;
    xpageoffset = -1;
    deltaq_c = (2.0F * Rpost[0] + 1.0F) - pd;
    if (Reast_idx_0 < deltaq_c) {
      Reast_idx_0 = deltaq_c;
      xpageoffset = 0;
    }
    deltaq_c = (2.0F * Rpost[4] + 1.0F) - pd;
    if (Reast_idx_0 < deltaq_c) {
      Reast_idx_0 = deltaq_c;
      xpageoffset = 1;
    }
    deltaq_c = (2.0F * Rpost[8] + 1.0F) - pd;
    if (Reast_idx_0 < deltaq_c) {
      Reast_idx_0 = deltaq_c;
      xpageoffset = 2;
    }
    switch (xpageoffset + 2) {
    case 1:
      pd = (float)sqrt(Reast_idx_0);
      deltaq_a = 0.5F * pd;
      pd = 0.5F / pd;
      deltaq_b = pd * (Rpost[7] - Rpost[5]);
      Reast_idx_0 = pd * (Rpost[2] - Rpost[6]);
      deltaq_c = pd * (Rpost[3] - Rpost[1]);
      break;
    case 2:
      pd = (float)sqrt(Reast_idx_0);
      deltaq_b = 0.5F * pd;
      pd = 0.5F / pd;
      deltaq_a = pd * (Rpost[7] - Rpost[5]);
      Reast_idx_0 = pd * (Rpost[1] + Rpost[3]);
      deltaq_c = pd * (Rpost[2] + Rpost[6]);
      break;
    case 3:
      pd = (float)sqrt(Reast_idx_0);
      Reast_idx_0 = 0.5F * pd;
      pd = 0.5F / pd;
      deltaq_a = pd * (Rpost[2] - Rpost[6]);
      deltaq_b = pd * (Rpost[1] + Rpost[3]);
      deltaq_c = pd * (Rpost[5] + Rpost[7]);
      break;
    default:
      pd = (float)sqrt(Reast_idx_0);
      deltaq_c = 0.5F * pd;
      pd = 0.5F / pd;
      deltaq_a = pd * (Rpost[3] - Rpost[1]);
      deltaq_b = pd * (Rpost[2] + Rpost[6]);
      Reast_idx_0 = pd * (Rpost[5] + Rpost[7]);
      break;
    }
    if (deltaq_a < 0.0F) {
      deltaq_a = -deltaq_a;
      deltaq_b = -deltaq_b;
      Reast_idx_0 = -Reast_idx_0;
      deltaq_c = -deltaq_c;
    }
    obj->pOrientPost.a = deltaq_a;
    obj->pOrientPost.b = deltaq_b;
    obj->pOrientPost.c = Reast_idx_0;
    obj->pOrientPost.d = deltaq_c;
  }
  qerr = obj->pOrientPost;
  gravityAccelGyroDiff[0] =
      (gyroIn[0] - obj->pGyroOffset[0]) * (float)obj->pSensorPeriod;
  gravityAccelGyroDiff[1] =
      (gyroIn[1] - obj->pGyroOffset[1]) * (float)obj->pSensorPeriod;
  gravityAccelGyroDiff[2] =
      (gyroIn[2] - obj->pGyroOffset[2]) * (float)obj->pSensorPeriod;
  quaternion_quaternion(gravityAccelGyroDiff, &deltaq_a, &deltaq_b, &deltaq_c,
                        &deltaq_d);
  pd = qerr.a;
  Reast_idx_0 = qerr.b;
  Reast_idx_1 = qerr.c;
  qerr.a = ((qerr.a * deltaq_a - qerr.b * deltaq_b) - qerr.c * deltaq_c) -
           qerr.d * deltaq_d;
  qerr.b = ((pd * deltaq_b + qerr.b * deltaq_a) + qerr.c * deltaq_d) -
           qerr.d * deltaq_c;
  qerr.c = ((pd * deltaq_c - Reast_idx_0 * deltaq_d) + qerr.c * deltaq_a) +
           qerr.d * deltaq_b;
  qerr.d = ((pd * deltaq_d + Reast_idx_0 * deltaq_c) - Reast_idx_1 * deltaq_b) +
           qerr.d * deltaq_a;
  if (qerr.a < 0.0F) {
    qerr.a = -qerr.a;
    qerr.b = -qerr.b;
    qerr.c = -qerr.c;
    qerr.d = -qerr.d;
  }
  obj->pOrientPrior = qerr;
  quaternionBase_rotmat(obj->pOrientPrior.a, obj->pOrientPrior.b,
                        obj->pOrientPrior.c, obj->pOrientPrior.d, Rprior);
  a_tmp_tmp = obj->LinearAccelerationDecayFactor;
  deltaq_c = -Rprior[6] * 9.81F;
  Rup[0] = deltaq_c;
  obj->pLinAccelPrior[0] = (float)a_tmp_tmp * obj->pLinAccelPost[0];
  gravityAccelGyroDiff[0] = (accelIn[0] + obj->pLinAccelPrior[0]) - deltaq_c;
  Reast_idx_0 = obj->pMagVec[0];
  deltaq_c = -Rprior[7] * 9.81F;
  Rup[1] = deltaq_c;
  obj->pLinAccelPrior[1] = (float)a_tmp_tmp * obj->pLinAccelPost[1];
  gravityAccelGyroDiff[1] = (accelIn[1] + obj->pLinAccelPrior[1]) - deltaq_c;
  Reast_idx_1 = obj->pMagVec[1];
  deltaq_c = -Rprior[8] * 9.81F;
  obj->pLinAccelPrior[2] = (float)a_tmp_tmp * obj->pLinAccelPost[2];
  gravityAccelGyroDiff[2] = (accelIn[2] + obj->pLinAccelPrior[2]) - deltaq_c;
  Reast_idx_2 = obj->pMagVec[2];
  for (i = 0; i < 3; i++) {
    magDistErr[i] = (Rprior[i] * Reast_idx_0 + Rprior[i + 3] * Reast_idx_1) +
                    Rprior[i + 6] * Reast_idx_2;
  }
  for (i = 0; i < 9; i++) {
    Rpost[i] = 0.0F;
  }
  Rpost[3] = deltaq_c;
  Rpost[6] = -Rup[1];
  Rpost[7] = Rup[0];
  for (i = 0; i < 3; i++) {
    h2[3 * i] = Rpost[3 * i];
    xpageoffset = 3 * i + 1;
    h2[xpageoffset] = Rpost[xpageoffset] - Rpost[i + 3];
    xpageoffset = 3 * i + 2;
    h2[xpageoffset] = Rpost[xpageoffset] - Rpost[i + 6];
  }
  for (i = 0; i < 9; i++) {
    Rpost[i] = h2[i];
    h2[i] = 0.0F;
  }
  h2[3] = magDistErr[2];
  h2[6] = -magDistErr[1];
  h2[7] = magDistErr[0];
  for (i = 0; i < 3; i++) {
    b_h2[3 * i] = h2[3 * i] - h2[i];
    xpageoffset = 3 * i + 1;
    b_h2[xpageoffset] = h2[xpageoffset] - h2[i + 3];
    xpageoffset = 3 * i + 2;
    b_h2[xpageoffset] = h2[xpageoffset] - h2[i + 6];
  }
  for (i = 0; i < 9; i++) {
    h2[i] = b_h2[i];
  }
  for (i = 0; i < 3; i++) {
    deltaq_c = Rpost[3 * i];
    H[6 * i] = deltaq_c;
    xpageoffset = 6 * (i + 3);
    H[xpageoffset] = -deltaq_c * (float)obj->pKalmanPeriod;
    xi = 6 * (i + 6);
    H[xi] = iv[3 * i];
    H_tmp = 6 * (i + 9);
    H[H_tmp] = 0.0F;
    deltaq_c = h2[3 * i];
    H[6 * i + 3] = deltaq_c;
    H[xpageoffset + 3] = -deltaq_c * (float)obj->pKalmanPeriod;
    H[xi + 3] = 0.0F;
    H[H_tmp + 3] = b_iv[3 * i];
    i1 = 3 * i + 1;
    deltaq_c = Rpost[i1];
    H[6 * i + 1] = deltaq_c;
    H[xpageoffset + 1] = -deltaq_c * (float)obj->pKalmanPeriod;
    H[xi + 1] = iv[i1];
    H[H_tmp + 1] = 0.0F;
    deltaq_c = h2[i1];
    H[6 * i + 4] = deltaq_c;
    H[xpageoffset + 4] = -deltaq_c * (float)obj->pKalmanPeriod;
    H[xi + 4] = 0.0F;
    H[H_tmp + 4] = b_iv[i1];
    i1 = 3 * i + 2;
    deltaq_c = Rpost[i1];
    H[6 * i + 2] = deltaq_c;
    H[xpageoffset + 2] = -deltaq_c * (float)obj->pKalmanPeriod;
    H[xi + 2] = iv[i1];
    H[H_tmp + 2] = 0.0F;
    deltaq_c = h2[i1];
    H[6 * i + 5] = deltaq_c;
    H[xpageoffset + 5] = -deltaq_c * (float)obj->pKalmanPeriod;
    H[xi + 5] = 0.0F;
    H[H_tmp + 5] = b_iv[i1];
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 12; i1++) {
      deltaq_c = 0.0F;
      for (xpageoffset = 0; xpageoffset < 12; xpageoffset++) {
        deltaq_c += H[i + 6 * xpageoffset] * obj->pQw[xpageoffset + 12 * i1];
      }
      xpageoffset = i + 6 * i1;
      b_tmp_tmp[xpageoffset] = deltaq_c;
      y_tmp[i1 + 12 * i] = H[xpageoffset];
    }
  }
  for (i = 0; i < 12; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      deltaq_c = 0.0F;
      for (xpageoffset = 0; xpageoffset < 12; xpageoffset++) {
        deltaq_c +=
            obj->pQw[i + 12 * xpageoffset] * y_tmp[xpageoffset + 12 * i1];
      }
      H[i + 12 * i1] = deltaq_c;
    }
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      deltaq_c = 0.0F;
      for (xpageoffset = 0; xpageoffset < 12; xpageoffset++) {
        deltaq_c +=
            b_tmp_tmp[i1 + 6 * xpageoffset] * y_tmp[xpageoffset + 12 * i];
      }
      tmp_tmp[i + 6 * i1] = deltaq_c + (float)obj->pQv[i1 + 6 * i];
    }
  }
  mrdiv(H, tmp_tmp);
  for (i = 0; i < 3; i++) {
    ze[i] = gravityAccelGyroDiff[i];
    ze[i + 3] =
        magIn[i] - ((Rprior[i] * Reast_idx_0 + Rprior[i + 3] * Reast_idx_1) +
                    Rprior[i + 6] * Reast_idx_2);
  }
  Reast_idx_1 = 0.0F;
  Reast_idx_2 = 1.29246971E-26F;
  for (xi = 0; xi < 3; xi++) {
    deltaq_c = 0.0F;
    deltaq_a = 0.0F;
    for (i = 0; i < 6; i++) {
      deltaq_b = H[(xi + 12 * i) + 9] * ze[i];
      deltaq_c += deltaq_b;
      deltaq_a += deltaq_b;
    }
    magDistErr[xi] = deltaq_a;
    pd = (float)fabs(deltaq_c);
    if (pd > Reast_idx_2) {
      Reast_idx_0 = Reast_idx_2 / pd;
      Reast_idx_1 = Reast_idx_1 * Reast_idx_0 * Reast_idx_0 + 1.0F;
      Reast_idx_2 = pd;
    } else {
      Reast_idx_0 = pd / Reast_idx_2;
      Reast_idx_1 += Reast_idx_0 * Reast_idx_0;
    }
  }
  Reast_idx_1 = Reast_idx_2 * (float)sqrt(Reast_idx_1);
  a_tmp_tmp = obj->ExpectedMagneticFieldStrength;
  isJamming = (Reast_idx_1 * Reast_idx_1 > 4.0 * (a_tmp_tmp * a_tmp_tmp));
  if (isJamming) {
    deltaq_c = gravityAccelGyroDiff[0];
    deltaq_a = gravityAccelGyroDiff[1];
    deltaq_b = gravityAccelGyroDiff[2];
    for (i = 0; i < 9; i++) {
      Rprior[i] =
          (H[i] * deltaq_c + H[i + 12] * deltaq_a) + H[i + 24] * deltaq_b;
    }
    Rup[0] = Rprior[0];
    Reast_idx_0 = Rprior[3];
    gravityAccelGyroDiff[0] = Rprior[6];
    Rup[1] = Rprior[1];
    Reast_idx_1 = Rprior[4];
    gravityAccelGyroDiff[1] = Rprior[7];
    Rup[2] = Rprior[2];
    Reast_idx_2 = Rprior[5];
    gravityAccelGyroDiff[2] = Rprior[8];
  } else {
    for (i = 0; i < 12; i++) {
      deltaq_c = 0.0F;
      for (i1 = 0; i1 < 6; i1++) {
        deltaq_c += H[i + 12 * i1] * ze[i1];
      }
      xe_post[i] = deltaq_c;
    }
    Rup[0] = xe_post[0];
    Reast_idx_0 = xe_post[3];
    gravityAccelGyroDiff[0] = xe_post[6];
    Rup[1] = xe_post[1];
    Reast_idx_1 = xe_post[4];
    gravityAccelGyroDiff[1] = xe_post[7];
    Rup[2] = xe_post[2];
    Reast_idx_2 = xe_post[5];
    gravityAccelGyroDiff[2] = xe_post[8];
  }
  quaternion_quaternion(Rup, &qerr.a, &qerr.b, &qerr.c, &qerr.d);
  qerr.b = -qerr.b;
  qerr.c = -qerr.c;
  qerr.d = -qerr.d;
  deltaq_a = obj->pOrientPrior.a;
  deltaq_b = obj->pOrientPrior.b;
  deltaq_c = obj->pOrientPrior.c;
  deltaq_d = obj->pOrientPrior.d;
  obj->pOrientPost.a =
      ((deltaq_a * qerr.a - deltaq_b * qerr.b) - deltaq_c * qerr.c) -
      deltaq_d * qerr.d;
  obj->pOrientPost.b =
      ((deltaq_a * qerr.b + deltaq_b * qerr.a) + deltaq_c * qerr.d) -
      deltaq_d * qerr.c;
  obj->pOrientPost.c =
      ((deltaq_a * qerr.c - deltaq_b * qerr.d) + deltaq_c * qerr.a) +
      deltaq_d * qerr.b;
  obj->pOrientPost.d =
      ((deltaq_a * qerr.d + deltaq_b * qerr.c) - deltaq_c * qerr.b) +
      deltaq_d * qerr.a;
  if (obj->pOrientPost.a < 0.0F) {
    qerr = obj->pOrientPost;
    qerr.a = -qerr.a;
    qerr.b = -qerr.b;
    qerr.c = -qerr.c;
    qerr.d = -qerr.d;
    obj->pOrientPost = qerr;
  }
  qerr = obj->pOrientPost;
  pd = (float)sqrt(((qerr.a * qerr.a + qerr.b * qerr.b) + qerr.c * qerr.c) +
                   qerr.d * qerr.d);
  qerr.a /= pd;
  qerr.b /= pd;
  qerr.c /= pd;
  qerr.d /= pd;
  obj->pOrientPost = qerr;
  quaternionBase_rotmat(obj->pOrientPost.a, obj->pOrientPost.b,
                        obj->pOrientPost.c, obj->pOrientPost.d, Rpost);
  obj->pGyroOffset[0] -= Reast_idx_0;
  obj->pLinAccelPost[0] = obj->pLinAccelPrior[0] - gravityAccelGyroDiff[0];
  obj->pGyroOffset[1] -= Reast_idx_1;
  obj->pLinAccelPost[1] = obj->pLinAccelPrior[1] - gravityAccelGyroDiff[1];
  obj->pGyroOffset[2] -= Reast_idx_2;
  obj->pLinAccelPost[2] = obj->pLinAccelPrior[2] - gravityAccelGyroDiff[2];
  if (!isJamming) {
    deltaq_c = magDistErr[0];
    deltaq_a = magDistErr[1];
    deltaq_b = magDistErr[2];
    for (xi = 0; xi < 3; xi++) {
      xpageoffset = xi * 3;
      Rup[xi] =
          obj->pMagVec[xi] -
          ((Rpost[xpageoffset] * deltaq_c + Rpost[xpageoffset + 1] * deltaq_a) +
           Rpost[xpageoffset + 2] * deltaq_b);
    }
    pd = (float)atan2(-Rup[2], Rup[1]);
    if (pd < -1.5707963267948966) {
      pd = -1.57079637F;
    }
    if (pd > 1.5707963267948966) {
      pd = 1.57079637F;
    }
    obj->pMagVec[0] = 0.0F;
    obj->pMagVec[1] = 0.0F;
    obj->pMagVec[2] = 0.0F;
    obj->pMagVec[1] = (float)cos(pd);
    obj->pMagVec[2] = -(float)sin(pd);
    obj->pMagVec[0] *= (float)obj->ExpectedMagneticFieldStrength;
    obj->pMagVec[1] *= (float)obj->ExpectedMagneticFieldStrength;
    obj->pMagVec[2] *= (float)obj->ExpectedMagneticFieldStrength;
  }
  for (i = 0; i < 12; i++) {
    for (i1 = 0; i1 < 12; i1++) {
      deltaq_c = 0.0F;
      for (xpageoffset = 0; xpageoffset < 6; xpageoffset++) {
        deltaq_c += H[i + 12 * xpageoffset] * b_tmp_tmp[xpageoffset + 6 * i1];
      }
      xpageoffset = i + 12 * i1;
      Ppost[xpageoffset] = obj->pQw[xpageoffset] - deltaq_c;
    }
  }
  memset(&obj->pQw[0], 0, 144U * sizeof(float));
  a_tmp_tmp = obj->pKalmanPeriod;
  a_tmp_tmp *= a_tmp_tmp;
  pd = (float)(obj->GyroscopeDriftNoise + obj->GyroscopeNoise);
  obj->pQw[0] = Ppost[0] + (float)a_tmp_tmp * (Ppost[39] + pd);
  obj->pQw[39] = Ppost[39] + (float)obj->GyroscopeDriftNoise;
  obj->pQw[13] = Ppost[13] + (float)a_tmp_tmp * (Ppost[52] + pd);
  obj->pQw[52] = Ppost[52] + (float)obj->GyroscopeDriftNoise;
  obj->pQw[26] = Ppost[26] + (float)a_tmp_tmp * (Ppost[65] + pd);
  obj->pQw[65] = Ppost[65] + (float)obj->GyroscopeDriftNoise;
  a_tmp_tmp = -obj->pKalmanPeriod;
  Rup[0] = (float)a_tmp_tmp * obj->pQw[39];
  Rup[1] = (float)a_tmp_tmp * obj->pQw[52];
  Rup[2] = (float)a_tmp_tmp * obj->pQw[65];
  a_tmp_tmp = obj->LinearAccelerationDecayFactor;
  a_tmp_tmp *= a_tmp_tmp;
  a = obj->MagneticDisturbanceDecayFactor;
  a *= a;
  obj->pQw[3] = Rup[0];
  obj->pQw[36] = Rup[0];
  obj->pQw[78] =
      (float)a_tmp_tmp * Ppost[78] + (float)obj->LinearAccelerationNoise;
  obj->pQw[117] = (float)a * Ppost[117] + (float)obj->MagneticDisturbanceNoise;
  obj->pQw[16] = Rup[1];
  obj->pQw[49] = Rup[1];
  obj->pQw[91] =
      (float)a_tmp_tmp * Ppost[91] + (float)obj->LinearAccelerationNoise;
  obj->pQw[130] = (float)a * Ppost[130] + (float)obj->MagneticDisturbanceNoise;
  obj->pQw[29] = Rup[2];
  obj->pQw[62] = Rup[2];
  obj->pQw[104] =
      (float)a_tmp_tmp * Ppost[104] + (float)obj->LinearAccelerationNoise;
  obj->pQw[143] = (float)a * Ppost[143] + (float)obj->MagneticDisturbanceNoise;
  qerr = obj->pOrientPost;
  *orientOut_a = qerr.a;
  *orientOut_b = qerr.b;
  *orientOut_c = qerr.c;
  *orientOut_d = qerr.d;
}

/*
 * File trailer for AHRSFilterBase.c
 *
 * [EOF]
 */
