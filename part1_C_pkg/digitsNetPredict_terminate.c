/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: digitsNetPredict_terminate.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 11-Mar-2025 17:19:10
 */

/* Include Files */
#include "digitsNetPredict_terminate.h"
#include "digitsNetPredict.h"
#include "digitsNetPredict_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void digitsNetPredict_terminate(void)
{
  digitsNetPredict_delete();
  isInitialized_digitsNetPredict = false;
}

/*
 * File trailer for digitsNetPredict_terminate.c
 *
 * [EOF]
 */
