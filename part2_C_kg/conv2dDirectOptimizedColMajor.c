/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: conv2dDirectOptimizedColMajor.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 13-Mar-2025 22:28:36
 */

/* Include Files */
#include "conv2dDirectOptimizedColMajor.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static void b_convolution(const float *inputTensor, float *outputTensor,
                          const float *weightsTensor, const float *biasTensor);

static void b_convolutionKernel(const float *inputBufferPtr,
                                float *outputBufferPtr,
                                const float *weightsBufferPtr,
                                const float *biasBufferPtr);

static void convolution(const float *inputTensor, float *outputTensor,
                        const float *weightsTensor, const float *biasTensor);

static void convolutionKernel(const float *inputBufferPtr,
                              float *outputBufferPtr,
                              const float *weightsBufferPtr,
                              const float *biasBufferPtr);

static void groupedConvolution(const float *inputTensor, float *outputTensor,
                               const float *weights, const float *bias);

static void groupedConvolutionKernel(const float *inputBufferPtr,
                                     float *outputBufferPtr,
                                     const float *weightsBufferPtr,
                                     int convOutputWidthIdx,
                                     int c_startIdxWithPaddingOffsetInpu,
                                     const float *biasBufferPtr);

/* Function Definitions */
/*
 * Arguments    : const float *inputTensor
 *                float *outputTensor
 *                const float *weightsTensor
 *                const float *biasTensor
 * Return Type  : void
 */
static void b_convolution(const float *inputTensor, float *outputTensor,
                          const float *weightsTensor, const float *biasTensor)
{
  static float inputScratchpadBuffer[1296];
  static bool bufferInitialized;
  int fusedInputChannelMiniBlockIdx;
  int fusedInputWidthAndHeightIdx;
  int inputWidthIdx;
  if (!bufferInitialized) {
    memset(&inputScratchpadBuffer[0], 0, 5184U);
    bufferInitialized = true;
  }
  for (fusedInputChannelMiniBlockIdx = 0; fusedInputChannelMiniBlockIdx < 49;
       fusedInputChannelMiniBlockIdx++) {
    int c_inputChannelMiniblockBaseIdxI;
    fusedInputWidthAndHeightIdx = fusedInputChannelMiniBlockIdx % 49;
    inputWidthIdx = fusedInputWidthAndHeightIdx / 7;
    fusedInputWidthAndHeightIdx %= 7;
    c_inputChannelMiniblockBaseIdxI =
        fusedInputWidthAndHeightIdx + inputWidthIdx * 7;
    fusedInputWidthAndHeightIdx =
        ((fusedInputWidthAndHeightIdx << 4) + inputWidthIdx * 144) + 160;
    for (inputWidthIdx = 0; inputWidthIdx < 6; inputWidthIdx++) {
      inputScratchpadBuffer[fusedInputWidthAndHeightIdx + inputWidthIdx] =
          inputTensor[c_inputChannelMiniblockBaseIdxI + inputWidthIdx * 49];
    }
  }
  for (inputWidthIdx = 0; inputWidthIdx < 7; inputWidthIdx++) {
    fusedInputWidthAndHeightIdx = inputWidthIdx % 7;
    b_convolutionKernel(
        &inputScratchpadBuffer[fusedInputWidthAndHeightIdx * 144],
        &outputTensor[fusedInputWidthAndHeightIdx * 7], &weightsTensor[0],
        &biasTensor[0]);
  }
}

/*
 * Arguments    : const float *inputBufferPtr
 *                float *outputBufferPtr
 *                const float *weightsBufferPtr
 *                const float *biasBufferPtr
 * Return Type  : void
 */
static void b_convolutionKernel(const float *inputBufferPtr,
                                float *outputBufferPtr,
                                const float *weightsBufferPtr,
                                const float *biasBufferPtr)
{
  float ab_outputRegister;
  float ac_outputRegister;
  float b_outputRegister;
  float b_outputRegister_tmp;
  float bb_outputRegister;
  float bc_outputRegister;
  float c_outputRegister;
  float c_outputRegister_tmp;
  float cb_outputRegister;
  float cc_outputRegister;
  float d_outputRegister;
  float d_outputRegister_tmp;
  float db_outputRegister;
  float dc_outputRegister;
  float e_outputRegister;
  float e_outputRegister_tmp;
  float eb_outputRegister;
  float ec_outputRegister;
  float f_outputRegister;
  float f_outputRegister_tmp;
  float fb_outputRegister;
  float fc_outputRegister;
  float g_outputRegister;
  float g_outputRegister_tmp;
  float gb_outputRegister;
  float gc_outputRegister;
  float h_outputRegister;
  float h_outputRegister_tmp;
  float hb_outputRegister;
  float hc_outputRegister;
  float i_outputRegister;
  float i_outputRegister_tmp;
  float ib_outputRegister;
  float ic_outputRegister;
  float j_outputRegister;
  float j_outputRegister_tmp;
  float jb_outputRegister;
  float jc_outputRegister;
  float k_outputRegister;
  float k_outputRegister_tmp;
  float kb_outputRegister;
  float kc_outputRegister;
  float l_outputRegister;
  float l_outputRegister_tmp;
  float lb_outputRegister;
  float lc_outputRegister;
  float m_outputRegister;
  float mb_outputRegister;
  float mc_outputRegister;
  float n_outputRegister;
  float nb_outputRegister;
  float nc_outputRegister;
  float o_outputRegister;
  float ob_outputRegister;
  float oc_outputRegister;
  float outputRegister;
  float outputRegister_tmp;
  float p_outputRegister;
  float pb_outputRegister;
  float pc_outputRegister;
  float q_outputRegister;
  float qb_outputRegister;
  float qc_outputRegister;
  float r_outputRegister;
  float rb_outputRegister;
  float rc_outputRegister;
  float s_outputRegister;
  float sb_outputRegister;
  float sc_outputRegister;
  float t_outputRegister;
  float tb_outputRegister;
  float tc_outputRegister;
  float u_outputRegister;
  float ub_outputRegister;
  float uc_outputRegister;
  float v_outputRegister;
  float vb_outputRegister;
  float vc_outputRegister;
  float w_outputRegister;
  float wb_outputRegister;
  float x_outputRegister;
  float xb_outputRegister;
  float y_outputRegister;
  float yb_outputRegister;
  int c_idxToStrideInputBufferAlongWi;
  int inputChannelIdx;
  int kernelHeightIdx;
  int kernelWidthIdx;
  int weightsIdx;
  outputRegister_tmp = biasBufferPtr[0];
  outputRegister = outputRegister_tmp;
  b_outputRegister = outputRegister_tmp;
  c_outputRegister = outputRegister_tmp;
  d_outputRegister = outputRegister_tmp;
  e_outputRegister = outputRegister_tmp;
  f_outputRegister = outputRegister_tmp;
  b_outputRegister_tmp = biasBufferPtr[1];
  g_outputRegister = b_outputRegister_tmp;
  h_outputRegister = b_outputRegister_tmp;
  i_outputRegister = b_outputRegister_tmp;
  j_outputRegister = b_outputRegister_tmp;
  k_outputRegister = b_outputRegister_tmp;
  l_outputRegister = b_outputRegister_tmp;
  c_outputRegister_tmp = biasBufferPtr[2];
  m_outputRegister = c_outputRegister_tmp;
  n_outputRegister = c_outputRegister_tmp;
  o_outputRegister = c_outputRegister_tmp;
  p_outputRegister = c_outputRegister_tmp;
  q_outputRegister = c_outputRegister_tmp;
  r_outputRegister = c_outputRegister_tmp;
  d_outputRegister_tmp = biasBufferPtr[3];
  s_outputRegister = d_outputRegister_tmp;
  t_outputRegister = d_outputRegister_tmp;
  u_outputRegister = d_outputRegister_tmp;
  v_outputRegister = d_outputRegister_tmp;
  w_outputRegister = d_outputRegister_tmp;
  x_outputRegister = d_outputRegister_tmp;
  e_outputRegister_tmp = biasBufferPtr[4];
  y_outputRegister = e_outputRegister_tmp;
  ab_outputRegister = e_outputRegister_tmp;
  bb_outputRegister = e_outputRegister_tmp;
  cb_outputRegister = e_outputRegister_tmp;
  db_outputRegister = e_outputRegister_tmp;
  eb_outputRegister = e_outputRegister_tmp;
  f_outputRegister_tmp = biasBufferPtr[5];
  fb_outputRegister = f_outputRegister_tmp;
  gb_outputRegister = f_outputRegister_tmp;
  hb_outputRegister = f_outputRegister_tmp;
  ib_outputRegister = f_outputRegister_tmp;
  jb_outputRegister = f_outputRegister_tmp;
  kb_outputRegister = f_outputRegister_tmp;
  g_outputRegister_tmp = biasBufferPtr[6];
  lb_outputRegister = g_outputRegister_tmp;
  mb_outputRegister = g_outputRegister_tmp;
  nb_outputRegister = g_outputRegister_tmp;
  ob_outputRegister = g_outputRegister_tmp;
  pb_outputRegister = g_outputRegister_tmp;
  qb_outputRegister = g_outputRegister_tmp;
  h_outputRegister_tmp = biasBufferPtr[7];
  rb_outputRegister = h_outputRegister_tmp;
  sb_outputRegister = h_outputRegister_tmp;
  tb_outputRegister = h_outputRegister_tmp;
  ub_outputRegister = h_outputRegister_tmp;
  vb_outputRegister = h_outputRegister_tmp;
  wb_outputRegister = h_outputRegister_tmp;
  i_outputRegister_tmp = biasBufferPtr[8];
  xb_outputRegister = i_outputRegister_tmp;
  yb_outputRegister = i_outputRegister_tmp;
  ac_outputRegister = i_outputRegister_tmp;
  bc_outputRegister = i_outputRegister_tmp;
  cc_outputRegister = i_outputRegister_tmp;
  dc_outputRegister = i_outputRegister_tmp;
  j_outputRegister_tmp = biasBufferPtr[9];
  ec_outputRegister = j_outputRegister_tmp;
  fc_outputRegister = j_outputRegister_tmp;
  gc_outputRegister = j_outputRegister_tmp;
  hc_outputRegister = j_outputRegister_tmp;
  ic_outputRegister = j_outputRegister_tmp;
  jc_outputRegister = j_outputRegister_tmp;
  k_outputRegister_tmp = biasBufferPtr[10];
  kc_outputRegister = k_outputRegister_tmp;
  lc_outputRegister = k_outputRegister_tmp;
  mc_outputRegister = k_outputRegister_tmp;
  nc_outputRegister = k_outputRegister_tmp;
  oc_outputRegister = k_outputRegister_tmp;
  pc_outputRegister = k_outputRegister_tmp;
  l_outputRegister_tmp = biasBufferPtr[11];
  qc_outputRegister = l_outputRegister_tmp;
  rc_outputRegister = l_outputRegister_tmp;
  sc_outputRegister = l_outputRegister_tmp;
  tc_outputRegister = l_outputRegister_tmp;
  uc_outputRegister = l_outputRegister_tmp;
  vc_outputRegister = l_outputRegister_tmp;
  c_idxToStrideInputBufferAlongWi = 0;
  weightsIdx = 0;
  for (kernelWidthIdx = 0; kernelWidthIdx < 3; kernelWidthIdx++) {
    int c_idxToStrideInputBufferAlongFi;
    c_idxToStrideInputBufferAlongFi = c_idxToStrideInputBufferAlongWi;
    for (kernelHeightIdx = 0; kernelHeightIdx < 3; kernelHeightIdx++) {
      for (inputChannelIdx = 0; inputChannelIdx < 16; inputChannelIdx++) {
        float b_inputRegister;
        float c_inputRegister;
        float d_inputRegister;
        float e_inputRegister;
        float f_inputRegister;
        float g_inputRegister;
        float inputRegister;
        float weightsRegister;
        int inputRegister_tmp;
        inputRegister_tmp = c_idxToStrideInputBufferAlongFi + inputChannelIdx;
        inputRegister = inputBufferPtr[inputRegister_tmp];
        b_inputRegister = inputBufferPtr[inputRegister_tmp + 16];
        c_inputRegister = inputBufferPtr[inputRegister_tmp + 32];
        d_inputRegister = inputBufferPtr[inputRegister_tmp + 48];
        e_inputRegister = inputBufferPtr[inputRegister_tmp + 64];
        f_inputRegister = inputBufferPtr[inputRegister_tmp + 80];
        g_inputRegister = inputBufferPtr[inputRegister_tmp + 96];
        inputRegister_tmp = weightsIdx + inputChannelIdx;
        weightsRegister = weightsBufferPtr[inputRegister_tmp];
        outputRegister_tmp += inputRegister * weightsRegister;
        outputRegister += b_inputRegister * weightsRegister;
        b_outputRegister += c_inputRegister * weightsRegister;
        c_outputRegister += d_inputRegister * weightsRegister;
        d_outputRegister += e_inputRegister * weightsRegister;
        e_outputRegister += f_inputRegister * weightsRegister;
        f_outputRegister += g_inputRegister * weightsRegister;
        weightsRegister = weightsBufferPtr[inputRegister_tmp + 144];
        b_outputRegister_tmp += inputRegister * weightsRegister;
        g_outputRegister += b_inputRegister * weightsRegister;
        h_outputRegister += c_inputRegister * weightsRegister;
        i_outputRegister += d_inputRegister * weightsRegister;
        j_outputRegister += e_inputRegister * weightsRegister;
        k_outputRegister += f_inputRegister * weightsRegister;
        l_outputRegister += g_inputRegister * weightsRegister;
        weightsRegister = weightsBufferPtr[inputRegister_tmp + 288];
        c_outputRegister_tmp += inputRegister * weightsRegister;
        m_outputRegister += b_inputRegister * weightsRegister;
        n_outputRegister += c_inputRegister * weightsRegister;
        o_outputRegister += d_inputRegister * weightsRegister;
        p_outputRegister += e_inputRegister * weightsRegister;
        q_outputRegister += f_inputRegister * weightsRegister;
        r_outputRegister += g_inputRegister * weightsRegister;
        weightsRegister = weightsBufferPtr[inputRegister_tmp + 432];
        d_outputRegister_tmp += inputRegister * weightsRegister;
        s_outputRegister += b_inputRegister * weightsRegister;
        t_outputRegister += c_inputRegister * weightsRegister;
        u_outputRegister += d_inputRegister * weightsRegister;
        v_outputRegister += e_inputRegister * weightsRegister;
        w_outputRegister += f_inputRegister * weightsRegister;
        x_outputRegister += g_inputRegister * weightsRegister;
        weightsRegister = weightsBufferPtr[inputRegister_tmp + 576];
        e_outputRegister_tmp += inputRegister * weightsRegister;
        y_outputRegister += b_inputRegister * weightsRegister;
        ab_outputRegister += c_inputRegister * weightsRegister;
        bb_outputRegister += d_inputRegister * weightsRegister;
        cb_outputRegister += e_inputRegister * weightsRegister;
        db_outputRegister += f_inputRegister * weightsRegister;
        eb_outputRegister += g_inputRegister * weightsRegister;
        weightsRegister = weightsBufferPtr[inputRegister_tmp + 720];
        f_outputRegister_tmp += inputRegister * weightsRegister;
        fb_outputRegister += b_inputRegister * weightsRegister;
        gb_outputRegister += c_inputRegister * weightsRegister;
        hb_outputRegister += d_inputRegister * weightsRegister;
        ib_outputRegister += e_inputRegister * weightsRegister;
        jb_outputRegister += f_inputRegister * weightsRegister;
        kb_outputRegister += g_inputRegister * weightsRegister;
        weightsRegister = weightsBufferPtr[inputRegister_tmp + 864];
        g_outputRegister_tmp += inputRegister * weightsRegister;
        lb_outputRegister += b_inputRegister * weightsRegister;
        mb_outputRegister += c_inputRegister * weightsRegister;
        nb_outputRegister += d_inputRegister * weightsRegister;
        ob_outputRegister += e_inputRegister * weightsRegister;
        pb_outputRegister += f_inputRegister * weightsRegister;
        qb_outputRegister += g_inputRegister * weightsRegister;
        weightsRegister = weightsBufferPtr[inputRegister_tmp + 1008];
        h_outputRegister_tmp += inputRegister * weightsRegister;
        rb_outputRegister += b_inputRegister * weightsRegister;
        sb_outputRegister += c_inputRegister * weightsRegister;
        tb_outputRegister += d_inputRegister * weightsRegister;
        ub_outputRegister += e_inputRegister * weightsRegister;
        vb_outputRegister += f_inputRegister * weightsRegister;
        wb_outputRegister += g_inputRegister * weightsRegister;
        weightsRegister = weightsBufferPtr[inputRegister_tmp + 1152];
        i_outputRegister_tmp += inputRegister * weightsRegister;
        xb_outputRegister += b_inputRegister * weightsRegister;
        yb_outputRegister += c_inputRegister * weightsRegister;
        ac_outputRegister += d_inputRegister * weightsRegister;
        bc_outputRegister += e_inputRegister * weightsRegister;
        cc_outputRegister += f_inputRegister * weightsRegister;
        dc_outputRegister += g_inputRegister * weightsRegister;
        weightsRegister = weightsBufferPtr[inputRegister_tmp + 1296];
        j_outputRegister_tmp += inputRegister * weightsRegister;
        ec_outputRegister += b_inputRegister * weightsRegister;
        fc_outputRegister += c_inputRegister * weightsRegister;
        gc_outputRegister += d_inputRegister * weightsRegister;
        hc_outputRegister += e_inputRegister * weightsRegister;
        ic_outputRegister += f_inputRegister * weightsRegister;
        jc_outputRegister += g_inputRegister * weightsRegister;
        weightsRegister = weightsBufferPtr[inputRegister_tmp + 1440];
        k_outputRegister_tmp += inputRegister * weightsRegister;
        kc_outputRegister += b_inputRegister * weightsRegister;
        lc_outputRegister += c_inputRegister * weightsRegister;
        mc_outputRegister += d_inputRegister * weightsRegister;
        nc_outputRegister += e_inputRegister * weightsRegister;
        oc_outputRegister += f_inputRegister * weightsRegister;
        pc_outputRegister += g_inputRegister * weightsRegister;
        weightsRegister = weightsBufferPtr[inputRegister_tmp + 1584];
        l_outputRegister_tmp += inputRegister * weightsRegister;
        qc_outputRegister += b_inputRegister * weightsRegister;
        rc_outputRegister += c_inputRegister * weightsRegister;
        sc_outputRegister += d_inputRegister * weightsRegister;
        tc_outputRegister += e_inputRegister * weightsRegister;
        uc_outputRegister += f_inputRegister * weightsRegister;
        vc_outputRegister += g_inputRegister * weightsRegister;
      }
      weightsIdx += 16;
      c_idxToStrideInputBufferAlongFi += 16;
    }
    c_idxToStrideInputBufferAlongWi += 144;
  }
  outputBufferPtr[0] = fmaxf(outputRegister_tmp, 0.0F);
  outputBufferPtr[1] = fmaxf(outputRegister, 0.0F);
  outputBufferPtr[2] = fmaxf(b_outputRegister, 0.0F);
  outputBufferPtr[3] = fmaxf(c_outputRegister, 0.0F);
  outputBufferPtr[4] = fmaxf(d_outputRegister, 0.0F);
  outputBufferPtr[5] = fmaxf(e_outputRegister, 0.0F);
  outputBufferPtr[6] = fmaxf(f_outputRegister, 0.0F);
  outputBufferPtr[49] = fmaxf(b_outputRegister_tmp, 0.0F);
  outputBufferPtr[50] = fmaxf(g_outputRegister, 0.0F);
  outputBufferPtr[51] = fmaxf(h_outputRegister, 0.0F);
  outputBufferPtr[52] = fmaxf(i_outputRegister, 0.0F);
  outputBufferPtr[53] = fmaxf(j_outputRegister, 0.0F);
  outputBufferPtr[54] = fmaxf(k_outputRegister, 0.0F);
  outputBufferPtr[55] = fmaxf(l_outputRegister, 0.0F);
  outputBufferPtr[98] = fmaxf(c_outputRegister_tmp, 0.0F);
  outputBufferPtr[99] = fmaxf(m_outputRegister, 0.0F);
  outputBufferPtr[100] = fmaxf(n_outputRegister, 0.0F);
  outputBufferPtr[101] = fmaxf(o_outputRegister, 0.0F);
  outputBufferPtr[102] = fmaxf(p_outputRegister, 0.0F);
  outputBufferPtr[103] = fmaxf(q_outputRegister, 0.0F);
  outputBufferPtr[104] = fmaxf(r_outputRegister, 0.0F);
  outputBufferPtr[147] = fmaxf(d_outputRegister_tmp, 0.0F);
  outputBufferPtr[148] = fmaxf(s_outputRegister, 0.0F);
  outputBufferPtr[149] = fmaxf(t_outputRegister, 0.0F);
  outputBufferPtr[150] = fmaxf(u_outputRegister, 0.0F);
  outputBufferPtr[151] = fmaxf(v_outputRegister, 0.0F);
  outputBufferPtr[152] = fmaxf(w_outputRegister, 0.0F);
  outputBufferPtr[153] = fmaxf(x_outputRegister, 0.0F);
  outputBufferPtr[196] = fmaxf(e_outputRegister_tmp, 0.0F);
  outputBufferPtr[197] = fmaxf(y_outputRegister, 0.0F);
  outputBufferPtr[198] = fmaxf(ab_outputRegister, 0.0F);
  outputBufferPtr[199] = fmaxf(bb_outputRegister, 0.0F);
  outputBufferPtr[200] = fmaxf(cb_outputRegister, 0.0F);
  outputBufferPtr[201] = fmaxf(db_outputRegister, 0.0F);
  outputBufferPtr[202] = fmaxf(eb_outputRegister, 0.0F);
  outputBufferPtr[245] = fmaxf(f_outputRegister_tmp, 0.0F);
  outputBufferPtr[246] = fmaxf(fb_outputRegister, 0.0F);
  outputBufferPtr[247] = fmaxf(gb_outputRegister, 0.0F);
  outputBufferPtr[248] = fmaxf(hb_outputRegister, 0.0F);
  outputBufferPtr[249] = fmaxf(ib_outputRegister, 0.0F);
  outputBufferPtr[250] = fmaxf(jb_outputRegister, 0.0F);
  outputBufferPtr[251] = fmaxf(kb_outputRegister, 0.0F);
  outputBufferPtr[294] = fmaxf(g_outputRegister_tmp, 0.0F);
  outputBufferPtr[295] = fmaxf(lb_outputRegister, 0.0F);
  outputBufferPtr[296] = fmaxf(mb_outputRegister, 0.0F);
  outputBufferPtr[297] = fmaxf(nb_outputRegister, 0.0F);
  outputBufferPtr[298] = fmaxf(ob_outputRegister, 0.0F);
  outputBufferPtr[299] = fmaxf(pb_outputRegister, 0.0F);
  outputBufferPtr[300] = fmaxf(qb_outputRegister, 0.0F);
  outputBufferPtr[343] = fmaxf(h_outputRegister_tmp, 0.0F);
  outputBufferPtr[344] = fmaxf(rb_outputRegister, 0.0F);
  outputBufferPtr[345] = fmaxf(sb_outputRegister, 0.0F);
  outputBufferPtr[346] = fmaxf(tb_outputRegister, 0.0F);
  outputBufferPtr[347] = fmaxf(ub_outputRegister, 0.0F);
  outputBufferPtr[348] = fmaxf(vb_outputRegister, 0.0F);
  outputBufferPtr[349] = fmaxf(wb_outputRegister, 0.0F);
  outputBufferPtr[392] = fmaxf(i_outputRegister_tmp, 0.0F);
  outputBufferPtr[393] = fmaxf(xb_outputRegister, 0.0F);
  outputBufferPtr[394] = fmaxf(yb_outputRegister, 0.0F);
  outputBufferPtr[395] = fmaxf(ac_outputRegister, 0.0F);
  outputBufferPtr[396] = fmaxf(bc_outputRegister, 0.0F);
  outputBufferPtr[397] = fmaxf(cc_outputRegister, 0.0F);
  outputBufferPtr[398] = fmaxf(dc_outputRegister, 0.0F);
  outputBufferPtr[441] = fmaxf(j_outputRegister_tmp, 0.0F);
  outputBufferPtr[442] = fmaxf(ec_outputRegister, 0.0F);
  outputBufferPtr[443] = fmaxf(fc_outputRegister, 0.0F);
  outputBufferPtr[444] = fmaxf(gc_outputRegister, 0.0F);
  outputBufferPtr[445] = fmaxf(hc_outputRegister, 0.0F);
  outputBufferPtr[446] = fmaxf(ic_outputRegister, 0.0F);
  outputBufferPtr[447] = fmaxf(jc_outputRegister, 0.0F);
  outputBufferPtr[490] = fmaxf(k_outputRegister_tmp, 0.0F);
  outputBufferPtr[491] = fmaxf(kc_outputRegister, 0.0F);
  outputBufferPtr[492] = fmaxf(lc_outputRegister, 0.0F);
  outputBufferPtr[493] = fmaxf(mc_outputRegister, 0.0F);
  outputBufferPtr[494] = fmaxf(nc_outputRegister, 0.0F);
  outputBufferPtr[495] = fmaxf(oc_outputRegister, 0.0F);
  outputBufferPtr[496] = fmaxf(pc_outputRegister, 0.0F);
  outputBufferPtr[539] = fmaxf(l_outputRegister_tmp, 0.0F);
  outputBufferPtr[540] = fmaxf(qc_outputRegister, 0.0F);
  outputBufferPtr[541] = fmaxf(rc_outputRegister, 0.0F);
  outputBufferPtr[542] = fmaxf(sc_outputRegister, 0.0F);
  outputBufferPtr[543] = fmaxf(tc_outputRegister, 0.0F);
  outputBufferPtr[544] = fmaxf(uc_outputRegister, 0.0F);
  outputBufferPtr[545] = fmaxf(vc_outputRegister, 0.0F);
}

/*
 * Arguments    : const float *inputTensor
 *                float *outputTensor
 *                const float *weightsTensor
 *                const float *biasTensor
 * Return Type  : void
 */
static void convolution(const float *inputTensor, float *outputTensor,
                        const float *weightsTensor, const float *biasTensor)
{
  static float inputScratchpadBuffer[4096];
  static bool bufferInitialized;
  int fusedInputChannelMiniBlockIdx;
  int fusedInputWidthAndHeightIdx;
  int inputWidthIdx;
  if (!bufferInitialized) {
    memset(&inputScratchpadBuffer[0], 0, 16384U);
    bufferInitialized = true;
  }
  for (fusedInputChannelMiniBlockIdx = 0; fusedInputChannelMiniBlockIdx < 196;
       fusedInputChannelMiniBlockIdx++) {
    fusedInputWidthAndHeightIdx = fusedInputChannelMiniBlockIdx % 196;
    inputWidthIdx = fusedInputWidthAndHeightIdx / 14;
    fusedInputWidthAndHeightIdx %= 14;
    inputScratchpadBuffer[((fusedInputWidthAndHeightIdx << 4) +
                           (inputWidthIdx << 8)) +
                          272] =
        inputTensor[fusedInputWidthAndHeightIdx + inputWidthIdx * 14];
  }
  for (inputWidthIdx = 0; inputWidthIdx < 14; inputWidthIdx++) {
    fusedInputWidthAndHeightIdx = inputWidthIdx % 14;
    convolutionKernel(&inputScratchpadBuffer[fusedInputWidthAndHeightIdx << 8],
                      &outputTensor[fusedInputWidthAndHeightIdx * 14],
                      &weightsTensor[0], &biasTensor[0]);
  }
}

/*
 * Arguments    : const float *inputBufferPtr
 *                float *outputBufferPtr
 *                const float *weightsBufferPtr
 *                const float *biasBufferPtr
 * Return Type  : void
 */
static void convolutionKernel(const float *inputBufferPtr,
                              float *outputBufferPtr,
                              const float *weightsBufferPtr,
                              const float *biasBufferPtr)
{
  float b_outputRegister_tmp;
  float c_outputRegister_tmp;
  float d_outputRegister_tmp;
  float e_outputRegister_tmp;
  float f_outputRegister_tmp;
  float outputRegister_tmp;
  int c_idxToStrideInputBufferAlongHe;
  int inputChannelIdx;
  int kernelHeightIdx;
  int kernelWidthIdx;
  int outputHeightBlockIdx;
  c_idxToStrideInputBufferAlongHe = 0;
  outputRegister_tmp = biasBufferPtr[0];
  b_outputRegister_tmp = biasBufferPtr[1];
  c_outputRegister_tmp = biasBufferPtr[2];
  d_outputRegister_tmp = biasBufferPtr[3];
  e_outputRegister_tmp = biasBufferPtr[4];
  f_outputRegister_tmp = biasBufferPtr[5];
  for (outputHeightBlockIdx = 0; outputHeightBlockIdx < 2;
       outputHeightBlockIdx++) {
    float ab_outputRegister;
    float b_outputRegister;
    float bb_outputRegister;
    float c_outputRegister;
    float cb_outputRegister;
    float d_outputRegister;
    float db_outputRegister;
    float e_outputRegister;
    float eb_outputRegister;
    float f_outputRegister;
    float fb_outputRegister;
    float g_outputRegister;
    float gb_outputRegister;
    float h_outputRegister;
    float hb_outputRegister;
    float i_outputRegister;
    float ib_outputRegister;
    float j_outputRegister;
    float jb_outputRegister;
    float k_outputRegister;
    float kb_outputRegister;
    float l_outputRegister;
    float lb_outputRegister;
    float m_outputRegister;
    float mb_outputRegister;
    float n_outputRegister;
    float nb_outputRegister;
    float o_outputRegister;
    float ob_outputRegister;
    float outputRegister;
    float p_outputRegister;
    float pb_outputRegister;
    float q_outputRegister;
    float qb_outputRegister;
    float r_outputRegister;
    float s_outputRegister;
    float t_outputRegister;
    float u_outputRegister;
    float v_outputRegister;
    float w_outputRegister;
    float x_outputRegister;
    float y_outputRegister;
    int c_idxToStrideInputBufferAlongWi;
    int weightsIdx;
    outputRegister = outputRegister_tmp;
    b_outputRegister = outputRegister_tmp;
    c_outputRegister = outputRegister_tmp;
    d_outputRegister = outputRegister_tmp;
    e_outputRegister = outputRegister_tmp;
    f_outputRegister = outputRegister_tmp;
    g_outputRegister = outputRegister_tmp;
    h_outputRegister = b_outputRegister_tmp;
    i_outputRegister = b_outputRegister_tmp;
    j_outputRegister = b_outputRegister_tmp;
    k_outputRegister = b_outputRegister_tmp;
    l_outputRegister = b_outputRegister_tmp;
    m_outputRegister = b_outputRegister_tmp;
    n_outputRegister = b_outputRegister_tmp;
    o_outputRegister = c_outputRegister_tmp;
    p_outputRegister = c_outputRegister_tmp;
    q_outputRegister = c_outputRegister_tmp;
    r_outputRegister = c_outputRegister_tmp;
    s_outputRegister = c_outputRegister_tmp;
    t_outputRegister = c_outputRegister_tmp;
    u_outputRegister = c_outputRegister_tmp;
    v_outputRegister = d_outputRegister_tmp;
    w_outputRegister = d_outputRegister_tmp;
    x_outputRegister = d_outputRegister_tmp;
    y_outputRegister = d_outputRegister_tmp;
    ab_outputRegister = d_outputRegister_tmp;
    bb_outputRegister = d_outputRegister_tmp;
    cb_outputRegister = d_outputRegister_tmp;
    db_outputRegister = e_outputRegister_tmp;
    eb_outputRegister = e_outputRegister_tmp;
    fb_outputRegister = e_outputRegister_tmp;
    gb_outputRegister = e_outputRegister_tmp;
    hb_outputRegister = e_outputRegister_tmp;
    ib_outputRegister = e_outputRegister_tmp;
    jb_outputRegister = e_outputRegister_tmp;
    kb_outputRegister = f_outputRegister_tmp;
    lb_outputRegister = f_outputRegister_tmp;
    mb_outputRegister = f_outputRegister_tmp;
    nb_outputRegister = f_outputRegister_tmp;
    ob_outputRegister = f_outputRegister_tmp;
    pb_outputRegister = f_outputRegister_tmp;
    qb_outputRegister = f_outputRegister_tmp;
    c_idxToStrideInputBufferAlongWi = c_idxToStrideInputBufferAlongHe;
    weightsIdx = 0;
    for (kernelWidthIdx = 0; kernelWidthIdx < 3; kernelWidthIdx++) {
      int c_idxToStrideInputBufferAlongFi;
      c_idxToStrideInputBufferAlongFi = c_idxToStrideInputBufferAlongWi;
      for (kernelHeightIdx = 0; kernelHeightIdx < 3; kernelHeightIdx++) {
        for (inputChannelIdx = 0; inputChannelIdx < 16; inputChannelIdx++) {
          float b_inputRegister;
          float c_inputRegister;
          float d_inputRegister;
          float e_inputRegister;
          float f_inputRegister;
          float g_inputRegister;
          float inputRegister;
          float weightsRegister;
          int inputRegister_tmp;
          inputRegister_tmp = c_idxToStrideInputBufferAlongFi + inputChannelIdx;
          inputRegister = inputBufferPtr[inputRegister_tmp];
          b_inputRegister = inputBufferPtr[inputRegister_tmp + 16];
          c_inputRegister = inputBufferPtr[inputRegister_tmp + 32];
          d_inputRegister = inputBufferPtr[inputRegister_tmp + 48];
          e_inputRegister = inputBufferPtr[inputRegister_tmp + 64];
          f_inputRegister = inputBufferPtr[inputRegister_tmp + 80];
          g_inputRegister = inputBufferPtr[inputRegister_tmp + 96];
          inputRegister_tmp = weightsIdx + inputChannelIdx;
          weightsRegister = weightsBufferPtr[inputRegister_tmp];
          outputRegister += inputRegister * weightsRegister;
          b_outputRegister += b_inputRegister * weightsRegister;
          c_outputRegister += c_inputRegister * weightsRegister;
          d_outputRegister += d_inputRegister * weightsRegister;
          e_outputRegister += e_inputRegister * weightsRegister;
          f_outputRegister += f_inputRegister * weightsRegister;
          g_outputRegister += g_inputRegister * weightsRegister;
          weightsRegister = weightsBufferPtr[inputRegister_tmp + 144];
          h_outputRegister += inputRegister * weightsRegister;
          i_outputRegister += b_inputRegister * weightsRegister;
          j_outputRegister += c_inputRegister * weightsRegister;
          k_outputRegister += d_inputRegister * weightsRegister;
          l_outputRegister += e_inputRegister * weightsRegister;
          m_outputRegister += f_inputRegister * weightsRegister;
          n_outputRegister += g_inputRegister * weightsRegister;
          weightsRegister = weightsBufferPtr[inputRegister_tmp + 288];
          o_outputRegister += inputRegister * weightsRegister;
          p_outputRegister += b_inputRegister * weightsRegister;
          q_outputRegister += c_inputRegister * weightsRegister;
          r_outputRegister += d_inputRegister * weightsRegister;
          s_outputRegister += e_inputRegister * weightsRegister;
          t_outputRegister += f_inputRegister * weightsRegister;
          u_outputRegister += g_inputRegister * weightsRegister;
          weightsRegister = weightsBufferPtr[inputRegister_tmp + 432];
          v_outputRegister += inputRegister * weightsRegister;
          w_outputRegister += b_inputRegister * weightsRegister;
          x_outputRegister += c_inputRegister * weightsRegister;
          y_outputRegister += d_inputRegister * weightsRegister;
          ab_outputRegister += e_inputRegister * weightsRegister;
          bb_outputRegister += f_inputRegister * weightsRegister;
          cb_outputRegister += g_inputRegister * weightsRegister;
          weightsRegister = weightsBufferPtr[inputRegister_tmp + 576];
          db_outputRegister += inputRegister * weightsRegister;
          eb_outputRegister += b_inputRegister * weightsRegister;
          fb_outputRegister += c_inputRegister * weightsRegister;
          gb_outputRegister += d_inputRegister * weightsRegister;
          hb_outputRegister += e_inputRegister * weightsRegister;
          ib_outputRegister += f_inputRegister * weightsRegister;
          jb_outputRegister += g_inputRegister * weightsRegister;
          weightsRegister = weightsBufferPtr[inputRegister_tmp + 720];
          kb_outputRegister += inputRegister * weightsRegister;
          lb_outputRegister += b_inputRegister * weightsRegister;
          mb_outputRegister += c_inputRegister * weightsRegister;
          nb_outputRegister += d_inputRegister * weightsRegister;
          ob_outputRegister += e_inputRegister * weightsRegister;
          pb_outputRegister += f_inputRegister * weightsRegister;
          qb_outputRegister += g_inputRegister * weightsRegister;
        }
        weightsIdx += 16;
        c_idxToStrideInputBufferAlongFi += 16;
      }
      c_idxToStrideInputBufferAlongWi += 256;
    }
    outputBufferPtr[outputHeightBlockIdx * 7] = fmaxf(outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1] =
        fmaxf(b_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2] =
        fmaxf(c_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 3] =
        fmaxf(d_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 4] =
        fmaxf(e_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 5] =
        fmaxf(f_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 6] =
        fmaxf(g_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 196] =
        fmaxf(h_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 197] =
        fmaxf(i_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 198] =
        fmaxf(j_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 199] =
        fmaxf(k_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 200] =
        fmaxf(l_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 201] =
        fmaxf(m_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 202] =
        fmaxf(n_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 392] =
        fmaxf(o_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 393] =
        fmaxf(p_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 394] =
        fmaxf(q_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 395] =
        fmaxf(r_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 396] =
        fmaxf(s_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 397] =
        fmaxf(t_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 398] =
        fmaxf(u_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 588] =
        fmaxf(v_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 589] =
        fmaxf(w_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 590] =
        fmaxf(x_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 591] =
        fmaxf(y_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 592] =
        fmaxf(ab_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 593] =
        fmaxf(bb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 594] =
        fmaxf(cb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 784] =
        fmaxf(db_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 785] =
        fmaxf(eb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 786] =
        fmaxf(fb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 787] =
        fmaxf(gb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 788] =
        fmaxf(hb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 789] =
        fmaxf(ib_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 790] =
        fmaxf(jb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 980] =
        fmaxf(kb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 981] =
        fmaxf(lb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 982] =
        fmaxf(mb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 983] =
        fmaxf(nb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 984] =
        fmaxf(ob_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 985] =
        fmaxf(pb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 986] =
        fmaxf(qb_outputRegister, 0.0F);
    c_idxToStrideInputBufferAlongHe += 112;
  }
}

/*
 * Arguments    : const float *inputTensor
 *                float *outputTensor
 *                const float *weights
 *                const float *bias
 * Return Type  : void
 */
static void groupedConvolution(const float *inputTensor, float *outputTensor,
                               const float *weights, const float *bias)
{
  int c_fusedOutputWidthAndChannelBlo;
  for (c_fusedOutputWidthAndChannelBlo = 0;
       c_fusedOutputWidthAndChannelBlo < 28;
       c_fusedOutputWidthAndChannelBlo++) {
    int outputWidthIdx_tmp;
    outputWidthIdx_tmp = c_fusedOutputWidthAndChannelBlo % 28;
    groupedConvolutionKernel(
        &inputTensor[0], &outputTensor[outputWidthIdx_tmp * 28], &weights[0],
        outputWidthIdx_tmp, (outputWidthIdx_tmp - 1) * 28 - 1, &bias[0]);
  }
}

/*
 * Arguments    : const float *inputBufferPtr
 *                float *outputBufferPtr
 *                const float *weightsBufferPtr
 *                int convOutputWidthIdx
 *                int c_startIdxWithPaddingOffsetInpu
 *                const float *biasBufferPtr
 * Return Type  : void
 */
static void groupedConvolutionKernel(const float *inputBufferPtr,
                                     float *outputBufferPtr,
                                     const float *weightsBufferPtr,
                                     int convOutputWidthIdx,
                                     int c_startIdxWithPaddingOffsetInpu,
                                     const float *biasBufferPtr)
{
  float biasRegister;
  int c_mIdxToStrideInputBufferAlongH;
  int kernelWidthIdx;
  int mOutputHeightIdx;
  int outputHeightBlockIdx;
  biasRegister = biasBufferPtr[0];
  c_mIdxToStrideInputBufferAlongH = c_startIdxWithPaddingOffsetInpu;
  mOutputHeightIdx = 0;
  for (outputHeightBlockIdx = 0; outputHeightBlockIdx < 4;
       outputHeightBlockIdx++) {
    float b_outputRegister;
    float c_outputRegister;
    float d_outputRegister;
    float e_outputRegister;
    float f_outputRegister;
    float g_outputRegister;
    float outputRegister;
    int c_mIdxToStrideInputBufferAlongW;
    int c_mIdxToStrideWeightsBufferAlon;
    outputRegister = biasRegister;
    b_outputRegister = biasRegister;
    c_outputRegister = biasRegister;
    d_outputRegister = biasRegister;
    e_outputRegister = biasRegister;
    f_outputRegister = biasRegister;
    g_outputRegister = biasRegister;
    c_mIdxToStrideInputBufferAlongW = c_mIdxToStrideInputBufferAlongH;
    c_mIdxToStrideWeightsBufferAlon = 0;
    for (kernelWidthIdx = 0; kernelWidthIdx < 3; kernelWidthIdx++) {
      float b_inputRegister;
      float c_inputRegister;
      float d_inputRegister;
      float e_inputRegister;
      float f_inputRegister;
      float g_inputRegister;
      float inputRegister;
      float weightsRegister;
      int leftPaddingBound_tmp;
      bool b;
      bool b1;
      bool b2;
      bool b3;
      bool b4;
      bool b5;
      bool b6;
      leftPaddingBound_tmp = (convOutputWidthIdx + kernelWidthIdx) - 1;
      b = ((leftPaddingBound_tmp >= 0) && (leftPaddingBound_tmp < 28));
      if (b && (mOutputHeightIdx - 1 >= 0) && (mOutputHeightIdx - 1 < 28)) {
        inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW];
      } else {
        inputRegister = 0.0F;
      }
      b1 = (b && (mOutputHeightIdx >= 0) && (mOutputHeightIdx < 28));
      if (b1) {
        b_inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 1];
      } else {
        b_inputRegister = 0.0F;
      }
      b2 = (b && (mOutputHeightIdx + 1 >= 0) && (mOutputHeightIdx + 1 < 28));
      if (b2) {
        c_inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 2];
      } else {
        c_inputRegister = 0.0F;
      }
      b3 = (b && (mOutputHeightIdx + 2 >= 0) && (mOutputHeightIdx + 2 < 28));
      if (b3) {
        d_inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 3];
      } else {
        d_inputRegister = 0.0F;
      }
      b4 = (b && (mOutputHeightIdx + 3 >= 0) && (mOutputHeightIdx + 3 < 28));
      if (b4) {
        e_inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 4];
      } else {
        e_inputRegister = 0.0F;
      }
      b5 = (b && (mOutputHeightIdx + 4 >= 0) && (mOutputHeightIdx + 4 < 28));
      if (b5) {
        f_inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 5];
      } else {
        f_inputRegister = 0.0F;
      }
      b6 = (b && (mOutputHeightIdx + 5 >= 0) && (mOutputHeightIdx + 5 < 28));
      if (b6) {
        g_inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 6];
      } else {
        g_inputRegister = 0.0F;
      }
      weightsRegister = weightsBufferPtr[c_mIdxToStrideWeightsBufferAlon];
      outputRegister += inputRegister * weightsRegister;
      b_outputRegister += b_inputRegister * weightsRegister;
      c_outputRegister += c_inputRegister * weightsRegister;
      d_outputRegister += d_inputRegister * weightsRegister;
      e_outputRegister += e_inputRegister * weightsRegister;
      f_outputRegister += f_inputRegister * weightsRegister;
      g_outputRegister += g_inputRegister * weightsRegister;
      if (b1) {
        inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 1];
      } else {
        inputRegister = 0.0F;
      }
      if (b2) {
        b_inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 2];
      } else {
        b_inputRegister = 0.0F;
      }
      if (b3) {
        c_inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 3];
      } else {
        c_inputRegister = 0.0F;
      }
      if (b4) {
        d_inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 4];
      } else {
        d_inputRegister = 0.0F;
      }
      if (b5) {
        e_inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 5];
      } else {
        e_inputRegister = 0.0F;
      }
      if (b6) {
        f_inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 6];
      } else {
        f_inputRegister = 0.0F;
      }
      b1 = (b && (mOutputHeightIdx + 6 >= 0) && (mOutputHeightIdx + 6 < 28));
      if (b1) {
        g_inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 7];
      } else {
        g_inputRegister = 0.0F;
      }
      weightsRegister = weightsBufferPtr[c_mIdxToStrideWeightsBufferAlon + 1];
      outputRegister += inputRegister * weightsRegister;
      b_outputRegister += b_inputRegister * weightsRegister;
      c_outputRegister += c_inputRegister * weightsRegister;
      d_outputRegister += d_inputRegister * weightsRegister;
      e_outputRegister += e_inputRegister * weightsRegister;
      f_outputRegister += f_inputRegister * weightsRegister;
      g_outputRegister += g_inputRegister * weightsRegister;
      if (b2) {
        inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 2];
      } else {
        inputRegister = 0.0F;
      }
      if (b3) {
        b_inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 3];
      } else {
        b_inputRegister = 0.0F;
      }
      if (b4) {
        c_inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 4];
      } else {
        c_inputRegister = 0.0F;
      }
      if (b5) {
        d_inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 5];
      } else {
        d_inputRegister = 0.0F;
      }
      if (b6) {
        e_inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 6];
      } else {
        e_inputRegister = 0.0F;
      }
      if (b1) {
        f_inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 7];
      } else {
        f_inputRegister = 0.0F;
      }
      if (b && (mOutputHeightIdx + 7 >= 0) && (mOutputHeightIdx + 7 < 28)) {
        g_inputRegister = inputBufferPtr[c_mIdxToStrideInputBufferAlongW + 8];
      } else {
        g_inputRegister = 0.0F;
      }
      weightsRegister = weightsBufferPtr[c_mIdxToStrideWeightsBufferAlon + 2];
      outputRegister += inputRegister * weightsRegister;
      b_outputRegister += b_inputRegister * weightsRegister;
      c_outputRegister += c_inputRegister * weightsRegister;
      d_outputRegister += d_inputRegister * weightsRegister;
      e_outputRegister += e_inputRegister * weightsRegister;
      f_outputRegister += f_inputRegister * weightsRegister;
      g_outputRegister += g_inputRegister * weightsRegister;
      c_mIdxToStrideWeightsBufferAlon += 3;
      c_mIdxToStrideInputBufferAlongW += 28;
    }
    outputBufferPtr[outputHeightBlockIdx * 7] = fmaxf(outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1] =
        fmaxf(b_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2] =
        fmaxf(c_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 3] =
        fmaxf(d_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 4] =
        fmaxf(e_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 5] =
        fmaxf(f_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 6] =
        fmaxf(g_outputRegister, 0.0F);
    c_mIdxToStrideInputBufferAlongH += 7;
    mOutputHeightIdx += 7;
  }
}

/*
 * Arguments    : const float X[196]
 *                float Z[1176]
 * Return Type  : void
 */
void b_conv2dDirectOptimizedColMajor(const float X[196], float Z[1176])
{
  static const float reformattedAndTruncatedWeights[864] = {-1.7813921F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            1.00164545F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.89017868F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -1.10711122F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -0.470249653F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            1.57914257F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -0.579804897F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -0.371890217F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            1.12052536F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -0.973040223F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -0.35414955F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.512812912F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.171753347F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -0.931700885F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -0.346230358F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            1.56613743F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -0.214780256F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -1.43076563F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.991596699F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            1.66767168F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            1.1420877F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            1.13995433F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -0.00780565338F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -1.25282633F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            1.50218654F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -0.776298285F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -1.06946075F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            1.30406666F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -0.200204089F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -2.38369036F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.327664942F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            3.14207506F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -0.614765F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -1.89969766F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.777588785F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            1.40061438F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            1.2956003F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.763860703F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.992126942F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -0.79799664F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.39619574F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.632731616F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -1.11477125F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -1.44840765F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -1.21592689F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -0.117439635F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -0.950345397F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -0.108235613F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            1.63097095F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            1.73050594F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            2.11607146F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.125330701F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -0.775898457F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            -0.80902952F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F,
                                                            0.0F};
  static const float biasReformatted[6] = {-0.0274518728F, 0.945787668F,
                                           -1.19014251F,   -1.37664628F,
                                           0.18596366F,    -1.39164102F};
  convolution(&X[0], &Z[0], &reformattedAndTruncatedWeights[0],
              &biasReformatted[0]);
}

/*
 * Arguments    : const float X[294]
 *                float Z[588]
 * Return Type  : void
 */
void c_conv2dDirectOptimizedColMajor(const float X[294], float Z[588])
{
  static const float reformattedAndTruncatedWeights[1728] = {-0.12749064F,
                                                             -0.159403786F,
                                                             -0.109646879F,
                                                             -0.252818376F,
                                                             -0.310005724F,
                                                             -0.0991627127F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0457620025F,
                                                             -0.192182377F,
                                                             -0.294992357F,
                                                             -0.0895827189F,
                                                             -0.109903924F,
                                                             0.0129951658F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.272907972F,
                                                             -0.403087884F,
                                                             -0.408487678F,
                                                             -0.277190804F,
                                                             0.0429630242F,
                                                             -0.00404750928F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0541873947F,
                                                             -0.0965252146F,
                                                             -0.14499931F,
                                                             -0.165368885F,
                                                             -0.228028402F,
                                                             -0.213780016F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0163426343F,
                                                             0.0739887208F,
                                                             0.444521874F,
                                                             -0.011156315F,
                                                             0.132325947F,
                                                             0.067039445F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.203816935F,
                                                             -0.0156123778F,
                                                             0.112684749F,
                                                             -0.0876311585F,
                                                             -0.00652877474F,
                                                             0.369987875F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.141368404F,
                                                             -0.179358959F,
                                                             -0.0219693296F,
                                                             -0.315342665F,
                                                             -0.110656291F,
                                                             -0.312216282F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.330007195F,
                                                             -0.2528947F,
                                                             0.232573509F,
                                                             -0.448806167F,
                                                             -0.0681973696F,
                                                             -0.231819019F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.230160132F,
                                                             -0.184319898F,
                                                             0.169773281F,
                                                             -0.430362076F,
                                                             -0.254107058F,
                                                             -0.193946049F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.269122362F,
                                                             0.262943F,
                                                             0.208863825F,
                                                             -0.286848068F,
                                                             -0.129568845F,
                                                             0.0660004616F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.322790325F,
                                                             0.0457868911F,
                                                             0.237189755F,
                                                             -0.513783F,
                                                             -0.3259525F,
                                                             -0.0541133806F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0358185954F,
                                                             0.144212335F,
                                                             -0.265566528F,
                                                             -0.113599248F,
                                                             -0.114432186F,
                                                             0.0891760811F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.118095785F,
                                                             0.0143346274F,
                                                             0.1476174F,
                                                             -0.246938899F,
                                                             -0.0340292379F,
                                                             -0.073467955F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.230687901F,
                                                             0.0169108026F,
                                                             -0.057782162F,
                                                             -0.220068142F,
                                                             -0.0878453255F,
                                                             -0.095048666F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.490953237F,
                                                             0.25461489F,
                                                             -0.143251508F,
                                                             0.037395902F,
                                                             0.390097767F,
                                                             0.197001472F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0887594074F,
                                                             0.0235353801F,
                                                             -0.179986432F,
                                                             -0.315473348F,
                                                             0.0298317857F,
                                                             -0.0970146134F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.305829287F,
                                                             0.0544874482F,
                                                             -0.0976641774F,
                                                             -0.322622418F,
                                                             -0.0888431072F,
                                                             -0.228183433F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.214688629F,
                                                             0.0644892231F,
                                                             -0.00191529095F,
                                                             -0.586904407F,
                                                             -0.121055365F,
                                                             -0.138929084F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0107901357F,
                                                             -0.00318603171F,
                                                             -0.141258463F,
                                                             -0.0305371098F,
                                                             0.277842402F,
                                                             -0.198681697F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.369327664F,
                                                             -0.0254186708F,
                                                             0.0697841421F,
                                                             0.0239906292F,
                                                             0.241186365F,
                                                             -0.339159936F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.248047054F,
                                                             0.163903117F,
                                                             -0.0803258196F,
                                                             -0.05279034F,
                                                             -0.0400381796F,
                                                             -0.272539973F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.325917542F,
                                                             -0.0806024745F,
                                                             -0.411748171F,
                                                             -0.0750217885F,
                                                             0.41405353F,
                                                             -0.291098893F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.251214653F,
                                                             -0.132123902F,
                                                             -0.299026608F,
                                                             0.690445483F,
                                                             0.152297601F,
                                                             -0.303391457F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.214201838F,
                                                             0.407869786F,
                                                             0.00754319131F,
                                                             0.340513587F,
                                                             -0.430462092F,
                                                             -0.195338205F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0631329641F,
                                                             0.0122060189F,
                                                             -0.611753583F,
                                                             -0.0183641538F,
                                                             0.0712530464F,
                                                             -0.219073042F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.674053431F,
                                                             -0.108727507F,
                                                             -0.489566088F,
                                                             0.155239627F,
                                                             -0.226713628F,
                                                             -0.378575683F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.112377763F,
                                                             -0.0327673554F,
                                                             0.109855406F,
                                                             0.33380276F,
                                                             -0.635501862F,
                                                             -0.143734455F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.246309042F,
                                                             -0.0526921861F,
                                                             0.439887166F,
                                                             0.00947642885F,
                                                             0.201280281F,
                                                             -0.0695599914F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.28797847F,
                                                             -0.0675298572F,
                                                             0.243309483F,
                                                             -0.283383638F,
                                                             0.373956114F,
                                                             -0.161829919F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0377848856F,
                                                             -0.0507903F,
                                                             -0.156634301F,
                                                             -0.252589405F,
                                                             -0.00965105277F,
                                                             -0.0476764143F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.141600877F,
                                                             -0.18708615F,
                                                             0.00423698034F,
                                                             0.470191061F,
                                                             -0.0402261801F,
                                                             0.0895395577F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.367386788F,
                                                             -0.109052911F,
                                                             -0.0122181904F,
                                                             -0.312967807F,
                                                             -0.22032465F,
                                                             0.126238421F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0274284706F,
                                                             0.0812175497F,
                                                             -0.00892537087F,
                                                             -0.00513463141F,
                                                             -0.250997F,
                                                             -0.127460748F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.178824082F,
                                                             -0.0596990436F,
                                                             0.250277519F,
                                                             -0.0391126573F,
                                                             -0.497715F,
                                                             0.0163003486F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.287712961F,
                                                             -0.00421807263F,
                                                             0.42607969F,
                                                             0.101997554F,
                                                             -0.583692968F,
                                                             0.202626839F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.165675417F,
                                                             0.083214581F,
                                                             0.044089172F,
                                                             0.338031232F,
                                                             -0.196597934F,
                                                             0.199392349F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.245902017F,
                                                             0.00359678012F,
                                                             -0.147537917F,
                                                             0.0354208872F,
                                                             0.261670023F,
                                                             -0.184724092F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.102375947F,
                                                             -0.302270591F,
                                                             -0.12532118F,
                                                             -0.218054831F,
                                                             -0.166980162F,
                                                             -0.249772385F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.608752728F,
                                                             -0.219250336F,
                                                             -0.221100897F,
                                                             0.20692955F,
                                                             -0.00187741627F,
                                                             0.0243464466F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.196391091F,
                                                             -0.236947566F,
                                                             -0.345099181F,
                                                             -0.0857031047F,
                                                             0.116940513F,
                                                             -0.0788322538F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.152023673F,
                                                             -0.360195875F,
                                                             -0.756244F,
                                                             0.175513774F,
                                                             -0.294197321F,
                                                             -0.214243039F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.374201775F,
                                                             -0.287068814F,
                                                             -0.192473441F,
                                                             -0.0122769512F,
                                                             -0.158653021F,
                                                             -0.179672107F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.353537977F,
                                                             -0.155729458F,
                                                             -0.18227303F,
                                                             0.603737772F,
                                                             0.274032F,
                                                             0.190245733F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0850281F,
                                                             -0.0482892245F,
                                                             -0.069598332F,
                                                             0.142850399F,
                                                             -0.0278326124F,
                                                             0.411591053F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.21679081F,
                                                             -0.1928588F,
                                                             0.0767586678F,
                                                             -0.188698977F,
                                                             0.135097504F,
                                                             0.21371761F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0722589344F,
                                                             0.109402493F,
                                                             0.132293314F,
                                                             -0.179190695F,
                                                             -0.124877855F,
                                                             0.170972079F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.292445779F,
                                                             -0.128816515F,
                                                             0.266379923F,
                                                             -0.35089156F,
                                                             0.344147712F,
                                                             0.485493392F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.268766701F,
                                                             -0.0189565662F,
                                                             0.435495198F,
                                                             -0.285559416F,
                                                             0.464971185F,
                                                             0.449087769F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.199517444F,
                                                             0.119122058F,
                                                             0.301590711F,
                                                             -0.255174786F,
                                                             -0.384655297F,
                                                             0.0939366296F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.332708985F,
                                                             0.0894402F,
                                                             0.0488731638F,
                                                             -0.510409415F,
                                                             -0.363851637F,
                                                             -0.0968115404F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.247565478F,
                                                             -0.000350512622F,
                                                             -0.069255814F,
                                                             -0.36909312F,
                                                             -0.0943982527F,
                                                             -0.142900318F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.160417348F,
                                                             0.00622157147F,
                                                             0.099404417F,
                                                             -0.285569549F,
                                                             -0.171660066F,
                                                             -0.112026654F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.209636837F,
                                                             -0.0707850233F,
                                                             -0.221279874F,
                                                             -0.333523095F,
                                                             -0.21926412F,
                                                             -0.160352543F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.18478623F,
                                                             0.153106764F,
                                                             -0.24393712F,
                                                             -0.304071754F,
                                                             0.0297714267F,
                                                             0.0412336849F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.360845655F,
                                                             -0.158290446F,
                                                             0.43524763F,
                                                             -0.515361726F,
                                                             0.272229105F,
                                                             0.0474770926F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.290330589F,
                                                             -0.0513681322F,
                                                             0.129925624F,
                                                             -0.265048057F,
                                                             0.501870871F,
                                                             -0.0328643657F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.603902042F,
                                                             0.304093838F,
                                                             -0.414677441F,
                                                             0.242298186F,
                                                             0.517069697F,
                                                             0.116626918F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.193276F,
                                                             -0.119258426F,
                                                             -0.0837730616F,
                                                             -0.393668115F,
                                                             -0.18626824F,
                                                             -0.170696482F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0965530798F,
                                                             -0.0802160427F,
                                                             -0.3471995F,
                                                             -0.452125967F,
                                                             -0.248155847F,
                                                             -0.36794439F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.518343866F,
                                                             -0.140538335F,
                                                             -0.301174F,
                                                             -0.368279457F,
                                                             -0.30444935F,
                                                             -0.207964927F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0230781566F,
                                                             0.298488289F,
                                                             0.182789356F,
                                                             0.840909064F,
                                                             0.212249905F,
                                                             0.209797055F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.107616395F,
                                                             0.174177304F,
                                                             0.0242529213F,
                                                             -0.0297785737F,
                                                             -0.377883524F,
                                                             -0.202556089F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.151493743F,
                                                             -0.00777093973F,
                                                             -0.212086827F,
                                                             -0.514012635F,
                                                             -0.129388779F,
                                                             -0.165306449F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.203312F,
                                                             -0.143515527F,
                                                             -0.122523211F,
                                                             0.0871332958F,
                                                             -0.148723F,
                                                             0.0116730183F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0755691603F,
                                                             -0.17584452F,
                                                             -0.307360619F,
                                                             0.0633061752F,
                                                             -0.0525251813F,
                                                             -0.130933046F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0185554028F,
                                                             -0.00953979325F,
                                                             -0.424253941F,
                                                             0.707230687F,
                                                             0.358310521F,
                                                             -0.0740240738F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.239312127F,
                                                             -0.11109627F,
                                                             -0.115952209F,
                                                             0.0113066463F,
                                                             0.0314974636F,
                                                             -0.0280972645F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.318757087F,
                                                             0.0849686638F,
                                                             0.0790606141F,
                                                             -0.124637641F,
                                                             -0.228491902F,
                                                             -0.0646473393F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0446926095F,
                                                             -0.120074011F,
                                                             0.121835358F,
                                                             0.337495267F,
                                                             0.111839667F,
                                                             0.585194468F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.252345145F,
                                                             -0.117881075F,
                                                             0.0207357761F,
                                                             -0.328426778F,
                                                             -0.0343586281F,
                                                             0.0432539545F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.296913117F,
                                                             -0.00492296275F,
                                                             0.425119281F,
                                                             -0.574021816F,
                                                             -0.127704263F,
                                                             -0.0777744353F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0932060629F,
                                                             -0.0382636338F,
                                                             0.601750851F,
                                                             -0.445949733F,
                                                             0.11612355F,
                                                             0.01991155F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.122224182F,
                                                             0.353743911F,
                                                             0.019334089F,
                                                             0.515352845F,
                                                             0.706352174F,
                                                             0.46231392F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.304037958F,
                                                             -0.16489394F,
                                                             -0.346825927F,
                                                             -0.28263092F,
                                                             -0.0421337932F,
                                                             0.362133533F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.344835967F,
                                                             -0.0137105025F,
                                                             0.0782606304F,
                                                             -0.40092808F,
                                                             -0.214744329F,
                                                             -0.293748766F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.138407F,
                                                             0.504751921F,
                                                             0.0735438317F,
                                                             0.187988058F,
                                                             0.525185525F,
                                                             0.322114438F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.275031716F,
                                                             -0.296545774F,
                                                             -0.266860694F,
                                                             -0.497990787F,
                                                             -0.0986201912F,
                                                             0.00382381119F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.288372517F,
                                                             -0.150359541F,
                                                             0.00843476225F,
                                                             -0.125273839F,
                                                             -0.173413903F,
                                                             -0.122355752F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.399929434F,
                                                             0.196442366F,
                                                             0.116822287F,
                                                             -0.165110618F,
                                                             -0.0656231865F,
                                                             0.0888280347F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0445465222F,
                                                             -0.0896858F,
                                                             -0.168388486F,
                                                             -0.304670215F,
                                                             -0.204982206F,
                                                             -0.307113558F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.2215202F,
                                                             0.157743543F,
                                                             0.0545505732F,
                                                             -0.121484242F,
                                                             -0.0207426045F,
                                                             0.0137812728F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.270411193F,
                                                             -0.066002652F,
                                                             -0.124279469F,
                                                             -0.413031071F,
                                                             -0.00367079F,
                                                             0.0639674067F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.00397867057F,
                                                             -0.0895577371F,
                                                             -0.526619F,
                                                             -0.237800404F,
                                                             -0.0400852561F,
                                                             -0.285874963F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.202836946F,
                                                             -0.187131166F,
                                                             -0.0242155641F,
                                                             -0.563800216F,
                                                             0.0531398617F,
                                                             -0.108595133F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.476745456F,
                                                             -0.37456432F,
                                                             0.0920488164F,
                                                             -0.604676902F,
                                                             0.00328856544F,
                                                             0.0409668684F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.214555904F,
                                                             -0.0941928923F,
                                                             0.00348966F,
                                                             -0.219058856F,
                                                             -0.20537135F,
                                                             0.366868198F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0656051412F,
                                                             -0.0894547924F,
                                                             0.0263374299F,
                                                             0.502137423F,
                                                             0.284359932F,
                                                             0.350231081F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0784115195F,
                                                             -0.234847143F,
                                                             0.336585075F,
                                                             -0.434525967F,
                                                             -0.335745186F,
                                                             -0.203601703F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.41686213F,
                                                             -0.519512117F,
                                                             0.395516396F,
                                                             -0.203991935F,
                                                             -0.0785811F,
                                                             0.41331473F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0583553687F,
                                                             -0.0469080806F,
                                                             0.386235982F,
                                                             0.498021692F,
                                                             0.225015461F,
                                                             0.51387614F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0642992631F,
                                                             0.355239958F,
                                                             0.418777823F,
                                                             -0.177305415F,
                                                             -0.244118035F,
                                                             0.247099772F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.186277F,
                                                             0.0851376876F,
                                                             -0.413697749F,
                                                             -0.473394394F,
                                                             0.059751574F,
                                                             -0.138322756F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0620078593F,
                                                             0.0928065777F,
                                                             0.282803744F,
                                                             -0.02382873F,
                                                             0.593582451F,
                                                             -0.269431293F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.182234436F,
                                                             0.0299029257F,
                                                             0.576271713F,
                                                             -0.202093557F,
                                                             -0.411605984F,
                                                             0.260437667F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0219806544F,
                                                             -0.292772353F,
                                                             0.140968397F,
                                                             -0.139992625F,
                                                             -0.340738565F,
                                                             0.00464799302F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0787954852F,
                                                             0.012284359F,
                                                             -0.131114185F,
                                                             0.16279161F,
                                                             -0.0234948F,
                                                             -0.00377348764F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.109949134F,
                                                             -0.0240162704F,
                                                             0.0855864286F,
                                                             0.041625496F,
                                                             -0.17095226F,
                                                             0.144103348F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.069709681F,
                                                             -0.13729085F,
                                                             -0.0860168487F,
                                                             -0.251389682F,
                                                             -0.264387101F,
                                                             -0.186186135F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0862445459F,
                                                             0.0359567069F,
                                                             -0.0921617597F,
                                                             0.0763940811F,
                                                             -0.222734824F,
                                                             -0.110660829F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.348198473F,
                                                             0.10385365F,
                                                             -0.43692714F,
                                                             -0.384046018F,
                                                             -0.269426972F,
                                                             -0.361473262F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.248693809F,
                                                             0.0651333556F,
                                                             -0.185440063F,
                                                             -0.231049955F,
                                                             -0.22782281F,
                                                             -0.0180024169F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.180291116F,
                                                             0.226506844F,
                                                             0.115828663F,
                                                             -0.490770459F,
                                                             -0.0458490178F,
                                                             -0.372116476F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.410231173F,
                                                             0.197194189F,
                                                             -0.323261827F,
                                                             -0.599558532F,
                                                             -0.0329081677F,
                                                             0.0380728841F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0455877073F,
                                                             -0.093842F,
                                                             0.359937578F,
                                                             -0.464026362F,
                                                             0.340618461F,
                                                             0.213838369F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.249789923F,
                                                             -0.115999207F,
                                                             0.0706459433F,
                                                             -0.326141655F,
                                                             0.0985796526F,
                                                             -0.308089793F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.095975548F,
                                                             0.204787761F,
                                                             0.61114949F,
                                                             -0.258547932F,
                                                             0.287067235F,
                                                             0.385851532F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.313222885F,
                                                             0.080015339F,
                                                             0.685490549F,
                                                             -0.494352F,
                                                             0.455793023F,
                                                             0.596935093F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.321091086F,
                                                             -0.0988807231F,
                                                             -0.130762458F,
                                                             -0.283533961F,
                                                             0.154816613F,
                                                             -0.0806669369F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F};
  static const float biasReformatted[12] = {
      0.0670342073F,  0.178781435F,    0.0153835928F, -0.00647611357F,
      -0.0373346917F, -0.00842731632F, 0.110284418F,  0.0892422721F,
      0.0691269636F,  0.138015419F,    0.0265274849F, 0.514638F};
  b_convolution(&X[0], &Z[0], &reformattedAndTruncatedWeights[0],
                &biasReformatted[0]);
}

/*
 * Arguments    : const float X[784]
 *                float Z[784]
 * Return Type  : void
 */
void conv2dDirectOptimizedColMajor(const float X[784], float Z[784])
{
  static const float reformattedAndTruncatedWeights[9] = {
      0.883234143F,  -0.678772211F, -0.399831116F, 0.907962918F, 0.298665881F,
      -0.269733697F, -0.381392241F, 1.23407531F,   0.926634669F};
  float biasReformatted;
  biasReformatted = 0.0598201F;
  groupedConvolution(&X[0], &Z[0], &reformattedAndTruncatedWeights[0],
                     &biasReformatted);
}

/*
 * File trailer for conv2dDirectOptimizedColMajor.c
 *
 * [EOF]
 */
