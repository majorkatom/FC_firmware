/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mrdivide_helper.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 30-Nov-2022 19:47:18
 */

/* Include Files */
#include "mrdivide_helper.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : float A[72]
 *                const float B[36]
 * Return Type  : void
 */
void mrdiv(float A[72], const float B[36])
{
  float b_A[36];
  float s;
  float smax;
  int32_t b_i;
  int32_t b_tmp;
  int32_t i;
  int32_t j;
  int32_t jA;
  int32_t jBcol;
  int32_t jp1j;
  int32_t k;
  int32_t kBcol;
  int32_t mmj_tmp;
  int8_t ipiv[6];
  int8_t i1;
  memcpy(&b_A[0], &B[0], 36U * sizeof(float));
  for (i = 0; i < 6; i++) {
    ipiv[i] = (int8_t)(i + 1);
  }
  for (j = 0; j < 5; j++) {
    mmj_tmp = 4 - j;
    b_tmp = j * 7;
    jp1j = b_tmp + 2;
    jA = 6 - j;
    jBcol = 0;
    smax = (float)fabs(b_A[b_tmp]);
    for (k = 2; k <= jA; k++) {
      s = (float)fabs(b_A[(b_tmp + k) - 1]);
      if (s > smax) {
        jBcol = k - 1;
        smax = s;
      }
    }
    if (b_A[b_tmp + jBcol] != 0.0F) {
      if (jBcol != 0) {
        jA = j + jBcol;
        ipiv[j] = (int8_t)(jA + 1);
        for (k = 0; k < 6; k++) {
          kBcol = j + k * 6;
          smax = b_A[kBcol];
          jBcol = jA + k * 6;
          b_A[kBcol] = b_A[jBcol];
          b_A[jBcol] = smax;
        }
      }
      i = (b_tmp - j) + 6;
      for (b_i = jp1j; b_i <= i; b_i++) {
        b_A[b_i - 1] /= b_A[b_tmp];
      }
    }
    jA = b_tmp;
    for (jBcol = 0; jBcol <= mmj_tmp; jBcol++) {
      smax = b_A[(b_tmp + jBcol * 6) + 6];
      if (smax != 0.0F) {
        i = jA + 8;
        jp1j = (jA - j) + 12;
        for (kBcol = i; kBcol <= jp1j; kBcol++) {
          b_A[kBcol - 1] += b_A[((b_tmp + kBcol) - jA) - 7] * -smax;
        }
      }
      jA += 6;
    }
  }
  for (j = 0; j < 6; j++) {
    jBcol = 12 * j - 1;
    jA = 6 * j;
    for (k = 0; k < j; k++) {
      kBcol = 12 * k;
      smax = b_A[k + jA];
      if (smax != 0.0F) {
        for (b_i = 0; b_i < 12; b_i++) {
          i = (b_i + jBcol) + 1;
          A[i] -= smax * A[b_i + kBcol];
        }
      }
    }
    smax = 1.0F / b_A[j + jA];
    for (b_i = 0; b_i < 12; b_i++) {
      i = (b_i + jBcol) + 1;
      A[i] *= smax;
    }
  }
  for (j = 5; j >= 0; j--) {
    jBcol = 12 * j - 1;
    jA = 6 * j - 1;
    i = j + 2;
    for (k = i; k < 7; k++) {
      kBcol = 12 * (k - 1);
      smax = b_A[k + jA];
      if (smax != 0.0F) {
        for (b_i = 0; b_i < 12; b_i++) {
          jp1j = (b_i + jBcol) + 1;
          A[jp1j] -= smax * A[b_i + kBcol];
        }
      }
    }
  }
  for (j = 4; j >= 0; j--) {
    i1 = ipiv[j];
    if (i1 != j + 1) {
      for (b_i = 0; b_i < 12; b_i++) {
        kBcol = b_i + 12 * j;
        smax = A[kBcol];
        i = b_i + 12 * (i1 - 1);
        A[kBcol] = A[i];
        A[i] = smax;
      }
    }
  }
}

/*
 * File trailer for mrdivide_helper.c
 *
 * [EOF]
 */
