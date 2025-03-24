/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: callPredict.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 13-Mar-2025 22:28:36
 */

#ifndef CALLPREDICT_H
#define CALLPREDICT_H

/* Include Files */
#include "digitsNetPredict_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void DeepLearningNetwork_callPredict(c_coder_internal_ctarget_DeepLe *obj,
                                     const float inputsT_0_f1[784],
                                     float outputs_0_f1[10]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for callPredict.h
 *
 * [EOF]
 */
