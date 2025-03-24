/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: predict.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 13-Mar-2025 22:28:36
 */

/* Include Files */
#include "predict.h"
#include "callPredict.h"
#include "digitsNetPredict_internal_types.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : c_coder_internal_ctarget_DeepLe *obj
 *                const unsigned char varargin_1[784]
 *                float varargout_1[10]
 * Return Type  : void
 */
void DeepLearningNetwork_predict(c_coder_internal_ctarget_DeepLe *obj,
                                 const unsigned char varargin_1[784],
                                 float varargout_1[10])
{
  float b_varargin_1[784];
  int i;
  for (i = 0; i < 784; i++) {
    b_varargin_1[i] = varargin_1[i];
  }
  DeepLearningNetwork_callPredict(obj, b_varargin_1, varargout_1);
}

/*
 * File trailer for predict.c
 *
 * [EOF]
 */
