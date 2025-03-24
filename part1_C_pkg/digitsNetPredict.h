/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: digitsNetPredict.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 11-Mar-2025 17:19:10
 */

#ifndef DIGITSNETPREDICT_H
#define DIGITSNETPREDICT_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void digitsNetPredict(const unsigned char in[784], float out[10]);

void digitsNetPredict_delete(void);

void digitsNetPredict_init(void);

void digitsNetPredict_new(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for digitsNetPredict.h
 *
 * [EOF]
 */
