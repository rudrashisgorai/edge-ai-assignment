/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: digitsNetPredict.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 13-Mar-2025 22:28:36
 */

/* Include Files */
#include "digitsNetPredict.h"
#include "digitsNetPredict_data.h"
#include "digitsNetPredict_initialize.h"
#include "digitsNetPredict_internal_types.h"
#include "predict.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static c_coder_internal_ctarget_DeepLe mynet;

static bool mynet_not_empty;

/* Function Definitions */
/*
 * Arguments    : const unsigned char in[784]
 *                float out[10]
 * Return Type  : void
 */
void digitsNetPredict(const unsigned char in[784], float out[10])
{
  if (!isInitialized_digitsNetPredict) {
    digitsNetPredict_initialize();
  }
  if (!mynet_not_empty) {
    mynet.IsNetworkInitialized = false;
    mynet.matlabCodegenIsDeleted = false;
    mynet_not_empty = true;
  }
  DeepLearningNetwork_predict(&mynet, in, out);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void digitsNetPredict_delete(void)
{
  if (!mynet.matlabCodegenIsDeleted) {
    mynet.matlabCodegenIsDeleted = true;
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void digitsNetPredict_init(void)
{
  mynet_not_empty = false;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void digitsNetPredict_new(void)
{
  mynet.matlabCodegenIsDeleted = true;
}

/*
 * File trailer for digitsNetPredict.c
 *
 * [EOF]
 */
