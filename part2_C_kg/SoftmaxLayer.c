/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: SoftmaxLayer.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 13-Mar-2025 22:28:36
 */

/* Include Files */
#include "SoftmaxLayer.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const float X1_Data[10]
 *                float Z1_Data[10]
 * Return Type  : void
 */
void SoftmaxLayer_predict(const float X1_Data[10], float Z1_Data[10])
{
  float maxVal;
  float sumX;
  int idx;
  int k;
  if (!rtIsNaNF(X1_Data[0])) {
    idx = 1;
  } else {
    bool exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 11)) {
      if (!rtIsNaNF(X1_Data[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (idx == 0) {
    maxVal = X1_Data[0];
  } else {
    maxVal = X1_Data[idx - 1];
    idx++;
    for (k = idx; k < 11; k++) {
      sumX = X1_Data[k - 1];
      if (maxVal < sumX) {
        maxVal = sumX;
      }
    }
  }
  for (idx = 0; idx < 10; idx++) {
    Z1_Data[idx] = expf(X1_Data[idx] - maxVal);
  }
  sumX = Z1_Data[0];
  for (k = 0; k < 9; k++) {
    sumX += Z1_Data[k + 1];
  }
  for (idx = 0; idx < 10; idx++) {
    Z1_Data[idx] /= sumX;
  }
}

/*
 * File trailer for SoftmaxLayer.c
 *
 * [EOF]
 */
