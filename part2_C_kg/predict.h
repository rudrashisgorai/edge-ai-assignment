/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: predict.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 13-Mar-2025 22:28:36
 */

#ifndef PREDICT_H
#define PREDICT_H

/* Include Files */
#include "digitsNetPredict_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void DeepLearningNetwork_predict(c_coder_internal_ctarget_DeepLe *obj,
                                 const unsigned char varargin_1[784],
                                 float varargout_1[10]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for predict.h
 *
 * [EOF]
 */
