/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: conv2dDirectOptimizedColMajor.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 11-Mar-2025 17:19:10
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

static void c_convolution(const float *inputTensor, float *outputTensor,
                          const float *weightsTensor, const float *biasTensor);

static void c_convolutionKernel(const float *inputBufferPtr,
                                float *outputBufferPtr,
                                const float *weightsBufferPtr,
                                const float *biasBufferPtr);

static void convolution(const float *inputTensor, float *outputTensor,
                        const float *weightsTensor, const float *biasTensor);

static void convolutionKernel(const float *inputBufferPtr,
                              float *outputBufferPtr,
                              const float *weightsBufferPtr,
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
    int c_inputChannelMiniblockBaseIdxI;
    fusedInputWidthAndHeightIdx = fusedInputChannelMiniBlockIdx % 196;
    inputWidthIdx = fusedInputWidthAndHeightIdx / 14;
    fusedInputWidthAndHeightIdx %= 14;
    c_inputChannelMiniblockBaseIdxI =
        fusedInputWidthAndHeightIdx + inputWidthIdx * 14;
    fusedInputWidthAndHeightIdx =
        ((fusedInputWidthAndHeightIdx << 4) + (inputWidthIdx << 8)) + 272;
    for (inputWidthIdx = 0; inputWidthIdx < 8; inputWidthIdx++) {
      inputScratchpadBuffer[fusedInputWidthAndHeightIdx + inputWidthIdx] =
          inputTensor[c_inputChannelMiniblockBaseIdxI + inputWidthIdx * 196];
    }
  }
  for (inputWidthIdx = 0; inputWidthIdx < 14; inputWidthIdx++) {
    fusedInputWidthAndHeightIdx = inputWidthIdx % 14;
    b_convolutionKernel(
        &inputScratchpadBuffer[fusedInputWidthAndHeightIdx << 8],
        &outputTensor[fusedInputWidthAndHeightIdx * 14], &weightsTensor[0],
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
  float b_outputRegister_tmp;
  float c_outputRegister_tmp;
  float d_outputRegister_tmp;
  float e_outputRegister_tmp;
  float f_outputRegister_tmp;
  float g_outputRegister_tmp;
  float h_outputRegister_tmp;
  float i_outputRegister_tmp;
  float j_outputRegister_tmp;
  float k_outputRegister_tmp;
  float l_outputRegister_tmp;
  float m_outputRegister_tmp;
  float n_outputRegister_tmp;
  float o_outputRegister_tmp;
  float outputRegister_tmp;
  float p_outputRegister_tmp;
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
  g_outputRegister_tmp = biasBufferPtr[6];
  h_outputRegister_tmp = biasBufferPtr[7];
  i_outputRegister_tmp = biasBufferPtr[8];
  j_outputRegister_tmp = biasBufferPtr[9];
  k_outputRegister_tmp = biasBufferPtr[10];
  l_outputRegister_tmp = biasBufferPtr[11];
  m_outputRegister_tmp = biasBufferPtr[12];
  n_outputRegister_tmp = biasBufferPtr[13];
  o_outputRegister_tmp = biasBufferPtr[14];
  p_outputRegister_tmp = biasBufferPtr[15];
  for (outputHeightBlockIdx = 0; outputHeightBlockIdx < 2;
       outputHeightBlockIdx++) {
    float ab_outputRegister;
    float ac_outputRegister;
    float ad_outputRegister;
    float ae_outputRegister;
    float b_outputRegister;
    float bb_outputRegister;
    float bc_outputRegister;
    float bd_outputRegister;
    float be_outputRegister;
    float c_outputRegister;
    float cb_outputRegister;
    float cc_outputRegister;
    float cd_outputRegister;
    float ce_outputRegister;
    float d_outputRegister;
    float db_outputRegister;
    float dc_outputRegister;
    float dd_outputRegister;
    float de_outputRegister;
    float e_outputRegister;
    float eb_outputRegister;
    float ec_outputRegister;
    float ed_outputRegister;
    float ee_outputRegister;
    float f_outputRegister;
    float fb_outputRegister;
    float fc_outputRegister;
    float fd_outputRegister;
    float fe_outputRegister;
    float g_outputRegister;
    float gb_outputRegister;
    float gc_outputRegister;
    float gd_outputRegister;
    float ge_outputRegister;
    float h_outputRegister;
    float hb_outputRegister;
    float hc_outputRegister;
    float hd_outputRegister;
    float he_outputRegister;
    float i_outputRegister;
    float ib_outputRegister;
    float ic_outputRegister;
    float id_outputRegister;
    float ie_outputRegister;
    float j_outputRegister;
    float jb_outputRegister;
    float jc_outputRegister;
    float jd_outputRegister;
    float je_outputRegister;
    float k_outputRegister;
    float kb_outputRegister;
    float kc_outputRegister;
    float kd_outputRegister;
    float ke_outputRegister;
    float l_outputRegister;
    float lb_outputRegister;
    float lc_outputRegister;
    float ld_outputRegister;
    float le_outputRegister;
    float m_outputRegister;
    float mb_outputRegister;
    float mc_outputRegister;
    float md_outputRegister;
    float n_outputRegister;
    float nb_outputRegister;
    float nc_outputRegister;
    float nd_outputRegister;
    float o_outputRegister;
    float ob_outputRegister;
    float oc_outputRegister;
    float od_outputRegister;
    float outputRegister;
    float p_outputRegister;
    float pb_outputRegister;
    float pc_outputRegister;
    float pd_outputRegister;
    float q_outputRegister;
    float qb_outputRegister;
    float qc_outputRegister;
    float qd_outputRegister;
    float r_outputRegister;
    float rb_outputRegister;
    float rc_outputRegister;
    float rd_outputRegister;
    float s_outputRegister;
    float sb_outputRegister;
    float sc_outputRegister;
    float sd_outputRegister;
    float t_outputRegister;
    float tb_outputRegister;
    float tc_outputRegister;
    float td_outputRegister;
    float u_outputRegister;
    float ub_outputRegister;
    float uc_outputRegister;
    float ud_outputRegister;
    float v_outputRegister;
    float vb_outputRegister;
    float vc_outputRegister;
    float vd_outputRegister;
    float w_outputRegister;
    float wb_outputRegister;
    float wc_outputRegister;
    float wd_outputRegister;
    float x_outputRegister;
    float xb_outputRegister;
    float xc_outputRegister;
    float xd_outputRegister;
    float y_outputRegister;
    float yb_outputRegister;
    float yc_outputRegister;
    float yd_outputRegister;
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
    rb_outputRegister = g_outputRegister_tmp;
    sb_outputRegister = g_outputRegister_tmp;
    tb_outputRegister = g_outputRegister_tmp;
    ub_outputRegister = g_outputRegister_tmp;
    vb_outputRegister = g_outputRegister_tmp;
    wb_outputRegister = g_outputRegister_tmp;
    xb_outputRegister = g_outputRegister_tmp;
    yb_outputRegister = h_outputRegister_tmp;
    ac_outputRegister = h_outputRegister_tmp;
    bc_outputRegister = h_outputRegister_tmp;
    cc_outputRegister = h_outputRegister_tmp;
    dc_outputRegister = h_outputRegister_tmp;
    ec_outputRegister = h_outputRegister_tmp;
    fc_outputRegister = h_outputRegister_tmp;
    gc_outputRegister = i_outputRegister_tmp;
    hc_outputRegister = i_outputRegister_tmp;
    ic_outputRegister = i_outputRegister_tmp;
    jc_outputRegister = i_outputRegister_tmp;
    kc_outputRegister = i_outputRegister_tmp;
    lc_outputRegister = i_outputRegister_tmp;
    mc_outputRegister = i_outputRegister_tmp;
    nc_outputRegister = j_outputRegister_tmp;
    oc_outputRegister = j_outputRegister_tmp;
    pc_outputRegister = j_outputRegister_tmp;
    qc_outputRegister = j_outputRegister_tmp;
    rc_outputRegister = j_outputRegister_tmp;
    sc_outputRegister = j_outputRegister_tmp;
    tc_outputRegister = j_outputRegister_tmp;
    uc_outputRegister = k_outputRegister_tmp;
    vc_outputRegister = k_outputRegister_tmp;
    wc_outputRegister = k_outputRegister_tmp;
    xc_outputRegister = k_outputRegister_tmp;
    yc_outputRegister = k_outputRegister_tmp;
    ad_outputRegister = k_outputRegister_tmp;
    bd_outputRegister = k_outputRegister_tmp;
    cd_outputRegister = l_outputRegister_tmp;
    dd_outputRegister = l_outputRegister_tmp;
    ed_outputRegister = l_outputRegister_tmp;
    fd_outputRegister = l_outputRegister_tmp;
    gd_outputRegister = l_outputRegister_tmp;
    hd_outputRegister = l_outputRegister_tmp;
    id_outputRegister = l_outputRegister_tmp;
    jd_outputRegister = m_outputRegister_tmp;
    kd_outputRegister = m_outputRegister_tmp;
    ld_outputRegister = m_outputRegister_tmp;
    md_outputRegister = m_outputRegister_tmp;
    nd_outputRegister = m_outputRegister_tmp;
    od_outputRegister = m_outputRegister_tmp;
    pd_outputRegister = m_outputRegister_tmp;
    qd_outputRegister = n_outputRegister_tmp;
    rd_outputRegister = n_outputRegister_tmp;
    sd_outputRegister = n_outputRegister_tmp;
    td_outputRegister = n_outputRegister_tmp;
    ud_outputRegister = n_outputRegister_tmp;
    vd_outputRegister = n_outputRegister_tmp;
    wd_outputRegister = n_outputRegister_tmp;
    xd_outputRegister = o_outputRegister_tmp;
    yd_outputRegister = o_outputRegister_tmp;
    ae_outputRegister = o_outputRegister_tmp;
    be_outputRegister = o_outputRegister_tmp;
    ce_outputRegister = o_outputRegister_tmp;
    de_outputRegister = o_outputRegister_tmp;
    ee_outputRegister = o_outputRegister_tmp;
    fe_outputRegister = p_outputRegister_tmp;
    ge_outputRegister = p_outputRegister_tmp;
    he_outputRegister = p_outputRegister_tmp;
    ie_outputRegister = p_outputRegister_tmp;
    je_outputRegister = p_outputRegister_tmp;
    ke_outputRegister = p_outputRegister_tmp;
    le_outputRegister = p_outputRegister_tmp;
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
          weightsRegister = weightsBufferPtr[inputRegister_tmp + 864];
          rb_outputRegister += inputRegister * weightsRegister;
          sb_outputRegister += b_inputRegister * weightsRegister;
          tb_outputRegister += c_inputRegister * weightsRegister;
          ub_outputRegister += d_inputRegister * weightsRegister;
          vb_outputRegister += e_inputRegister * weightsRegister;
          wb_outputRegister += f_inputRegister * weightsRegister;
          xb_outputRegister += g_inputRegister * weightsRegister;
          weightsRegister = weightsBufferPtr[inputRegister_tmp + 1008];
          yb_outputRegister += inputRegister * weightsRegister;
          ac_outputRegister += b_inputRegister * weightsRegister;
          bc_outputRegister += c_inputRegister * weightsRegister;
          cc_outputRegister += d_inputRegister * weightsRegister;
          dc_outputRegister += e_inputRegister * weightsRegister;
          ec_outputRegister += f_inputRegister * weightsRegister;
          fc_outputRegister += g_inputRegister * weightsRegister;
          weightsRegister = weightsBufferPtr[inputRegister_tmp + 1152];
          gc_outputRegister += inputRegister * weightsRegister;
          hc_outputRegister += b_inputRegister * weightsRegister;
          ic_outputRegister += c_inputRegister * weightsRegister;
          jc_outputRegister += d_inputRegister * weightsRegister;
          kc_outputRegister += e_inputRegister * weightsRegister;
          lc_outputRegister += f_inputRegister * weightsRegister;
          mc_outputRegister += g_inputRegister * weightsRegister;
          weightsRegister = weightsBufferPtr[inputRegister_tmp + 1296];
          nc_outputRegister += inputRegister * weightsRegister;
          oc_outputRegister += b_inputRegister * weightsRegister;
          pc_outputRegister += c_inputRegister * weightsRegister;
          qc_outputRegister += d_inputRegister * weightsRegister;
          rc_outputRegister += e_inputRegister * weightsRegister;
          sc_outputRegister += f_inputRegister * weightsRegister;
          tc_outputRegister += g_inputRegister * weightsRegister;
          weightsRegister = weightsBufferPtr[inputRegister_tmp + 1440];
          uc_outputRegister += inputRegister * weightsRegister;
          vc_outputRegister += b_inputRegister * weightsRegister;
          wc_outputRegister += c_inputRegister * weightsRegister;
          xc_outputRegister += d_inputRegister * weightsRegister;
          yc_outputRegister += e_inputRegister * weightsRegister;
          ad_outputRegister += f_inputRegister * weightsRegister;
          bd_outputRegister += g_inputRegister * weightsRegister;
          weightsRegister = weightsBufferPtr[inputRegister_tmp + 1584];
          cd_outputRegister += inputRegister * weightsRegister;
          dd_outputRegister += b_inputRegister * weightsRegister;
          ed_outputRegister += c_inputRegister * weightsRegister;
          fd_outputRegister += d_inputRegister * weightsRegister;
          gd_outputRegister += e_inputRegister * weightsRegister;
          hd_outputRegister += f_inputRegister * weightsRegister;
          id_outputRegister += g_inputRegister * weightsRegister;
          weightsRegister = weightsBufferPtr[inputRegister_tmp + 1728];
          jd_outputRegister += inputRegister * weightsRegister;
          kd_outputRegister += b_inputRegister * weightsRegister;
          ld_outputRegister += c_inputRegister * weightsRegister;
          md_outputRegister += d_inputRegister * weightsRegister;
          nd_outputRegister += e_inputRegister * weightsRegister;
          od_outputRegister += f_inputRegister * weightsRegister;
          pd_outputRegister += g_inputRegister * weightsRegister;
          weightsRegister = weightsBufferPtr[inputRegister_tmp + 1872];
          qd_outputRegister += inputRegister * weightsRegister;
          rd_outputRegister += b_inputRegister * weightsRegister;
          sd_outputRegister += c_inputRegister * weightsRegister;
          td_outputRegister += d_inputRegister * weightsRegister;
          ud_outputRegister += e_inputRegister * weightsRegister;
          vd_outputRegister += f_inputRegister * weightsRegister;
          wd_outputRegister += g_inputRegister * weightsRegister;
          weightsRegister = weightsBufferPtr[inputRegister_tmp + 2016];
          xd_outputRegister += inputRegister * weightsRegister;
          yd_outputRegister += b_inputRegister * weightsRegister;
          ae_outputRegister += c_inputRegister * weightsRegister;
          be_outputRegister += d_inputRegister * weightsRegister;
          ce_outputRegister += e_inputRegister * weightsRegister;
          de_outputRegister += f_inputRegister * weightsRegister;
          ee_outputRegister += g_inputRegister * weightsRegister;
          weightsRegister = weightsBufferPtr[inputRegister_tmp + 2160];
          fe_outputRegister += inputRegister * weightsRegister;
          ge_outputRegister += b_inputRegister * weightsRegister;
          he_outputRegister += c_inputRegister * weightsRegister;
          ie_outputRegister += d_inputRegister * weightsRegister;
          je_outputRegister += e_inputRegister * weightsRegister;
          ke_outputRegister += f_inputRegister * weightsRegister;
          le_outputRegister += g_inputRegister * weightsRegister;
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
    outputBufferPtr[outputHeightBlockIdx * 7 + 1176] =
        fmaxf(rb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1177] =
        fmaxf(sb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1178] =
        fmaxf(tb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1179] =
        fmaxf(ub_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1180] =
        fmaxf(vb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1181] =
        fmaxf(wb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1182] =
        fmaxf(xb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1372] =
        fmaxf(yb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1373] =
        fmaxf(ac_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1374] =
        fmaxf(bc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1375] =
        fmaxf(cc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1376] =
        fmaxf(dc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1377] =
        fmaxf(ec_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1378] =
        fmaxf(fc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1568] =
        fmaxf(gc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1569] =
        fmaxf(hc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1570] =
        fmaxf(ic_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1571] =
        fmaxf(jc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1572] =
        fmaxf(kc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1573] =
        fmaxf(lc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1574] =
        fmaxf(mc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1764] =
        fmaxf(nc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1765] =
        fmaxf(oc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1766] =
        fmaxf(pc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1767] =
        fmaxf(qc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1768] =
        fmaxf(rc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1769] =
        fmaxf(sc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1770] =
        fmaxf(tc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1960] =
        fmaxf(uc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1961] =
        fmaxf(vc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1962] =
        fmaxf(wc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1963] =
        fmaxf(xc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1964] =
        fmaxf(yc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1965] =
        fmaxf(ad_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1966] =
        fmaxf(bd_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2156] =
        fmaxf(cd_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2157] =
        fmaxf(dd_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2158] =
        fmaxf(ed_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2159] =
        fmaxf(fd_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2160] =
        fmaxf(gd_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2161] =
        fmaxf(hd_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2162] =
        fmaxf(id_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2352] =
        fmaxf(jd_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2353] =
        fmaxf(kd_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2354] =
        fmaxf(ld_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2355] =
        fmaxf(md_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2356] =
        fmaxf(nd_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2357] =
        fmaxf(od_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2358] =
        fmaxf(pd_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2548] =
        fmaxf(qd_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2549] =
        fmaxf(rd_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2550] =
        fmaxf(sd_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2551] =
        fmaxf(td_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2552] =
        fmaxf(ud_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2553] =
        fmaxf(vd_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2554] =
        fmaxf(wd_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2744] =
        fmaxf(xd_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2745] =
        fmaxf(yd_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2746] =
        fmaxf(ae_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2747] =
        fmaxf(be_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2748] =
        fmaxf(ce_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2749] =
        fmaxf(de_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2750] =
        fmaxf(ee_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2940] =
        fmaxf(fe_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2941] =
        fmaxf(ge_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2942] =
        fmaxf(he_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2943] =
        fmaxf(ie_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2944] =
        fmaxf(je_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2945] =
        fmaxf(ke_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2946] =
        fmaxf(le_outputRegister, 0.0F);
    c_idxToStrideInputBufferAlongHe += 112;
  }
}

/*
 * Arguments    : const float *inputTensor
 *                float *outputTensor
 *                const float *weightsTensor
 *                const float *biasTensor
 * Return Type  : void
 */
static void c_convolution(const float *inputTensor, float *outputTensor,
                          const float *weightsTensor, const float *biasTensor)
{
  static float inputScratchpadBuffer[1296];
  static bool bufferInitialized;
  int c_inputChannelMiniblockBaseIdxI;
  int fusedInputChannelMiniBlockIdx;
  int fusedInputWidthAndHeightIdx;
  int inputWidthIdx;
  if (!bufferInitialized) {
    memset(&inputScratchpadBuffer[0], 0, 5184U);
    bufferInitialized = true;
  }
  for (fusedInputChannelMiniBlockIdx = 0; fusedInputChannelMiniBlockIdx < 49;
       fusedInputChannelMiniBlockIdx++) {
    fusedInputWidthAndHeightIdx = fusedInputChannelMiniBlockIdx % 49;
    inputWidthIdx = fusedInputWidthAndHeightIdx / 7;
    fusedInputWidthAndHeightIdx %= 7;
    c_inputChannelMiniblockBaseIdxI =
        fusedInputWidthAndHeightIdx + inputWidthIdx * 7;
    fusedInputWidthAndHeightIdx =
        ((fusedInputWidthAndHeightIdx << 4) + inputWidthIdx * 144) + 160;
    for (inputWidthIdx = 0; inputWidthIdx < 16; inputWidthIdx++) {
      inputScratchpadBuffer[fusedInputWidthAndHeightIdx + inputWidthIdx] =
          inputTensor[c_inputChannelMiniblockBaseIdxI + inputWidthIdx * 49];
    }
  }
  for (c_inputChannelMiniblockBaseIdxI = 0;
       c_inputChannelMiniblockBaseIdxI < 14;
       c_inputChannelMiniblockBaseIdxI++) {
    fusedInputWidthAndHeightIdx = c_inputChannelMiniblockBaseIdxI / 7;
    inputWidthIdx = c_inputChannelMiniblockBaseIdxI % 7;
    c_convolutionKernel(
        &inputScratchpadBuffer[inputWidthIdx * 144],
        &outputTensor[inputWidthIdx * 7 + fusedInputWidthAndHeightIdx * 784],
        &weightsTensor[fusedInputWidthAndHeightIdx * 2304],
        &biasTensor[fusedInputWidthAndHeightIdx << 4]);
  }
}

/*
 * Arguments    : const float *inputBufferPtr
 *                float *outputBufferPtr
 *                const float *weightsBufferPtr
 *                const float *biasBufferPtr
 * Return Type  : void
 */
static void c_convolutionKernel(const float *inputBufferPtr,
                                float *outputBufferPtr,
                                const float *weightsBufferPtr,
                                const float *biasBufferPtr)
{
  float ab_outputRegister;
  float ac_outputRegister;
  float ad_outputRegister;
  float b_outputRegister;
  float b_outputRegister_tmp;
  float bb_outputRegister;
  float bc_outputRegister;
  float bd_outputRegister;
  float c_outputRegister;
  float c_outputRegister_tmp;
  float cb_outputRegister;
  float cc_outputRegister;
  float cd_outputRegister;
  float d_outputRegister;
  float d_outputRegister_tmp;
  float db_outputRegister;
  float dc_outputRegister;
  float dd_outputRegister;
  float e_outputRegister;
  float e_outputRegister_tmp;
  float eb_outputRegister;
  float ec_outputRegister;
  float ed_outputRegister;
  float f_outputRegister;
  float f_outputRegister_tmp;
  float fb_outputRegister;
  float fc_outputRegister;
  float fd_outputRegister;
  float g_outputRegister;
  float g_outputRegister_tmp;
  float gb_outputRegister;
  float gc_outputRegister;
  float gd_outputRegister;
  float h_outputRegister;
  float h_outputRegister_tmp;
  float hb_outputRegister;
  float hc_outputRegister;
  float hd_outputRegister;
  float i_outputRegister;
  float i_outputRegister_tmp;
  float ib_outputRegister;
  float ic_outputRegister;
  float id_outputRegister;
  float j_outputRegister;
  float j_outputRegister_tmp;
  float jb_outputRegister;
  float jc_outputRegister;
  float jd_outputRegister;
  float k_outputRegister;
  float k_outputRegister_tmp;
  float kb_outputRegister;
  float kc_outputRegister;
  float kd_outputRegister;
  float l_outputRegister;
  float l_outputRegister_tmp;
  float lb_outputRegister;
  float lc_outputRegister;
  float ld_outputRegister;
  float m_outputRegister;
  float m_outputRegister_tmp;
  float mb_outputRegister;
  float mc_outputRegister;
  float md_outputRegister;
  float n_outputRegister;
  float n_outputRegister_tmp;
  float nb_outputRegister;
  float nc_outputRegister;
  float nd_outputRegister;
  float o_outputRegister;
  float o_outputRegister_tmp;
  float ob_outputRegister;
  float oc_outputRegister;
  float od_outputRegister;
  float outputRegister;
  float outputRegister_tmp;
  float p_outputRegister;
  float p_outputRegister_tmp;
  float pb_outputRegister;
  float pc_outputRegister;
  float pd_outputRegister;
  float q_outputRegister;
  float qb_outputRegister;
  float qc_outputRegister;
  float qd_outputRegister;
  float r_outputRegister;
  float rb_outputRegister;
  float rc_outputRegister;
  float rd_outputRegister;
  float s_outputRegister;
  float sb_outputRegister;
  float sc_outputRegister;
  float sd_outputRegister;
  float t_outputRegister;
  float tb_outputRegister;
  float tc_outputRegister;
  float td_outputRegister;
  float u_outputRegister;
  float ub_outputRegister;
  float uc_outputRegister;
  float ud_outputRegister;
  float v_outputRegister;
  float vb_outputRegister;
  float vc_outputRegister;
  float w_outputRegister;
  float wb_outputRegister;
  float wc_outputRegister;
  float x_outputRegister;
  float xb_outputRegister;
  float xc_outputRegister;
  float y_outputRegister;
  float yb_outputRegister;
  float yc_outputRegister;
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
  m_outputRegister_tmp = biasBufferPtr[12];
  wc_outputRegister = m_outputRegister_tmp;
  xc_outputRegister = m_outputRegister_tmp;
  yc_outputRegister = m_outputRegister_tmp;
  ad_outputRegister = m_outputRegister_tmp;
  bd_outputRegister = m_outputRegister_tmp;
  cd_outputRegister = m_outputRegister_tmp;
  n_outputRegister_tmp = biasBufferPtr[13];
  dd_outputRegister = n_outputRegister_tmp;
  ed_outputRegister = n_outputRegister_tmp;
  fd_outputRegister = n_outputRegister_tmp;
  gd_outputRegister = n_outputRegister_tmp;
  hd_outputRegister = n_outputRegister_tmp;
  id_outputRegister = n_outputRegister_tmp;
  o_outputRegister_tmp = biasBufferPtr[14];
  jd_outputRegister = o_outputRegister_tmp;
  kd_outputRegister = o_outputRegister_tmp;
  ld_outputRegister = o_outputRegister_tmp;
  md_outputRegister = o_outputRegister_tmp;
  nd_outputRegister = o_outputRegister_tmp;
  od_outputRegister = o_outputRegister_tmp;
  p_outputRegister_tmp = biasBufferPtr[15];
  pd_outputRegister = p_outputRegister_tmp;
  qd_outputRegister = p_outputRegister_tmp;
  rd_outputRegister = p_outputRegister_tmp;
  sd_outputRegister = p_outputRegister_tmp;
  td_outputRegister = p_outputRegister_tmp;
  ud_outputRegister = p_outputRegister_tmp;
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
        weightsRegister = weightsBufferPtr[inputRegister_tmp + 1728];
        m_outputRegister_tmp += inputRegister * weightsRegister;
        wc_outputRegister += b_inputRegister * weightsRegister;
        xc_outputRegister += c_inputRegister * weightsRegister;
        yc_outputRegister += d_inputRegister * weightsRegister;
        ad_outputRegister += e_inputRegister * weightsRegister;
        bd_outputRegister += f_inputRegister * weightsRegister;
        cd_outputRegister += g_inputRegister * weightsRegister;
        weightsRegister = weightsBufferPtr[inputRegister_tmp + 1872];
        n_outputRegister_tmp += inputRegister * weightsRegister;
        dd_outputRegister += b_inputRegister * weightsRegister;
        ed_outputRegister += c_inputRegister * weightsRegister;
        fd_outputRegister += d_inputRegister * weightsRegister;
        gd_outputRegister += e_inputRegister * weightsRegister;
        hd_outputRegister += f_inputRegister * weightsRegister;
        id_outputRegister += g_inputRegister * weightsRegister;
        weightsRegister = weightsBufferPtr[inputRegister_tmp + 2016];
        o_outputRegister_tmp += inputRegister * weightsRegister;
        jd_outputRegister += b_inputRegister * weightsRegister;
        kd_outputRegister += c_inputRegister * weightsRegister;
        ld_outputRegister += d_inputRegister * weightsRegister;
        md_outputRegister += e_inputRegister * weightsRegister;
        nd_outputRegister += f_inputRegister * weightsRegister;
        od_outputRegister += g_inputRegister * weightsRegister;
        weightsRegister = weightsBufferPtr[inputRegister_tmp + 2160];
        p_outputRegister_tmp += inputRegister * weightsRegister;
        pd_outputRegister += b_inputRegister * weightsRegister;
        qd_outputRegister += c_inputRegister * weightsRegister;
        rd_outputRegister += d_inputRegister * weightsRegister;
        sd_outputRegister += e_inputRegister * weightsRegister;
        td_outputRegister += f_inputRegister * weightsRegister;
        ud_outputRegister += g_inputRegister * weightsRegister;
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
  outputBufferPtr[588] = fmaxf(m_outputRegister_tmp, 0.0F);
  outputBufferPtr[589] = fmaxf(wc_outputRegister, 0.0F);
  outputBufferPtr[590] = fmaxf(xc_outputRegister, 0.0F);
  outputBufferPtr[591] = fmaxf(yc_outputRegister, 0.0F);
  outputBufferPtr[592] = fmaxf(ad_outputRegister, 0.0F);
  outputBufferPtr[593] = fmaxf(bd_outputRegister, 0.0F);
  outputBufferPtr[594] = fmaxf(cd_outputRegister, 0.0F);
  outputBufferPtr[637] = fmaxf(n_outputRegister_tmp, 0.0F);
  outputBufferPtr[638] = fmaxf(dd_outputRegister, 0.0F);
  outputBufferPtr[639] = fmaxf(ed_outputRegister, 0.0F);
  outputBufferPtr[640] = fmaxf(fd_outputRegister, 0.0F);
  outputBufferPtr[641] = fmaxf(gd_outputRegister, 0.0F);
  outputBufferPtr[642] = fmaxf(hd_outputRegister, 0.0F);
  outputBufferPtr[643] = fmaxf(id_outputRegister, 0.0F);
  outputBufferPtr[686] = fmaxf(o_outputRegister_tmp, 0.0F);
  outputBufferPtr[687] = fmaxf(jd_outputRegister, 0.0F);
  outputBufferPtr[688] = fmaxf(kd_outputRegister, 0.0F);
  outputBufferPtr[689] = fmaxf(ld_outputRegister, 0.0F);
  outputBufferPtr[690] = fmaxf(md_outputRegister, 0.0F);
  outputBufferPtr[691] = fmaxf(nd_outputRegister, 0.0F);
  outputBufferPtr[692] = fmaxf(od_outputRegister, 0.0F);
  outputBufferPtr[735] = fmaxf(p_outputRegister_tmp, 0.0F);
  outputBufferPtr[736] = fmaxf(pd_outputRegister, 0.0F);
  outputBufferPtr[737] = fmaxf(qd_outputRegister, 0.0F);
  outputBufferPtr[738] = fmaxf(rd_outputRegister, 0.0F);
  outputBufferPtr[739] = fmaxf(sd_outputRegister, 0.0F);
  outputBufferPtr[740] = fmaxf(td_outputRegister, 0.0F);
  outputBufferPtr[741] = fmaxf(ud_outputRegister, 0.0F);
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
  static float inputScratchpadBuffer[14400];
  static bool bufferInitialized;
  int fusedInputChannelMiniBlockIdx;
  int fusedInputWidthAndHeightIdx;
  int inputWidthIdx;
  if (!bufferInitialized) {
    memset(&inputScratchpadBuffer[0], 0, 57600U);
    bufferInitialized = true;
  }
  for (fusedInputChannelMiniBlockIdx = 0; fusedInputChannelMiniBlockIdx < 784;
       fusedInputChannelMiniBlockIdx++) {
    fusedInputWidthAndHeightIdx = fusedInputChannelMiniBlockIdx % 784;
    inputWidthIdx = fusedInputWidthAndHeightIdx / 28;
    fusedInputWidthAndHeightIdx %= 28;
    inputScratchpadBuffer[((fusedInputWidthAndHeightIdx << 4) +
                           inputWidthIdx * 480) +
                          496] =
        inputTensor[fusedInputWidthAndHeightIdx + inputWidthIdx * 28];
  }
  for (inputWidthIdx = 0; inputWidthIdx < 28; inputWidthIdx++) {
    fusedInputWidthAndHeightIdx = inputWidthIdx % 28;
    convolutionKernel(&inputScratchpadBuffer[fusedInputWidthAndHeightIdx * 480],
                      &outputTensor[fusedInputWidthAndHeightIdx * 28],
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
  float g_outputRegister_tmp;
  float h_outputRegister_tmp;
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
  g_outputRegister_tmp = biasBufferPtr[6];
  h_outputRegister_tmp = biasBufferPtr[7];
  for (outputHeightBlockIdx = 0; outputHeightBlockIdx < 4;
       outputHeightBlockIdx++) {
    float ab_outputRegister;
    float ac_outputRegister;
    float b_outputRegister;
    float bb_outputRegister;
    float bc_outputRegister;
    float c_outputRegister;
    float cb_outputRegister;
    float cc_outputRegister;
    float d_outputRegister;
    float db_outputRegister;
    float dc_outputRegister;
    float e_outputRegister;
    float eb_outputRegister;
    float ec_outputRegister;
    float f_outputRegister;
    float fb_outputRegister;
    float fc_outputRegister;
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
    float rb_outputRegister;
    float s_outputRegister;
    float sb_outputRegister;
    float t_outputRegister;
    float tb_outputRegister;
    float u_outputRegister;
    float ub_outputRegister;
    float v_outputRegister;
    float vb_outputRegister;
    float w_outputRegister;
    float wb_outputRegister;
    float x_outputRegister;
    float xb_outputRegister;
    float y_outputRegister;
    float yb_outputRegister;
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
    rb_outputRegister = g_outputRegister_tmp;
    sb_outputRegister = g_outputRegister_tmp;
    tb_outputRegister = g_outputRegister_tmp;
    ub_outputRegister = g_outputRegister_tmp;
    vb_outputRegister = g_outputRegister_tmp;
    wb_outputRegister = g_outputRegister_tmp;
    xb_outputRegister = g_outputRegister_tmp;
    yb_outputRegister = h_outputRegister_tmp;
    ac_outputRegister = h_outputRegister_tmp;
    bc_outputRegister = h_outputRegister_tmp;
    cc_outputRegister = h_outputRegister_tmp;
    dc_outputRegister = h_outputRegister_tmp;
    ec_outputRegister = h_outputRegister_tmp;
    fc_outputRegister = h_outputRegister_tmp;
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
          weightsRegister = weightsBufferPtr[inputRegister_tmp + 864];
          rb_outputRegister += inputRegister * weightsRegister;
          sb_outputRegister += b_inputRegister * weightsRegister;
          tb_outputRegister += c_inputRegister * weightsRegister;
          ub_outputRegister += d_inputRegister * weightsRegister;
          vb_outputRegister += e_inputRegister * weightsRegister;
          wb_outputRegister += f_inputRegister * weightsRegister;
          xb_outputRegister += g_inputRegister * weightsRegister;
          weightsRegister = weightsBufferPtr[inputRegister_tmp + 1008];
          yb_outputRegister += inputRegister * weightsRegister;
          ac_outputRegister += b_inputRegister * weightsRegister;
          bc_outputRegister += c_inputRegister * weightsRegister;
          cc_outputRegister += d_inputRegister * weightsRegister;
          dc_outputRegister += e_inputRegister * weightsRegister;
          ec_outputRegister += f_inputRegister * weightsRegister;
          fc_outputRegister += g_inputRegister * weightsRegister;
        }
        weightsIdx += 16;
        c_idxToStrideInputBufferAlongFi += 16;
      }
      c_idxToStrideInputBufferAlongWi += 480;
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
    outputBufferPtr[outputHeightBlockIdx * 7 + 784] =
        fmaxf(h_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 785] =
        fmaxf(i_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 786] =
        fmaxf(j_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 787] =
        fmaxf(k_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 788] =
        fmaxf(l_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 789] =
        fmaxf(m_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 790] =
        fmaxf(n_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1568] =
        fmaxf(o_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1569] =
        fmaxf(p_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1570] =
        fmaxf(q_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1571] =
        fmaxf(r_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1572] =
        fmaxf(s_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1573] =
        fmaxf(t_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 1574] =
        fmaxf(u_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2352] =
        fmaxf(v_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2353] =
        fmaxf(w_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2354] =
        fmaxf(x_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2355] =
        fmaxf(y_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2356] =
        fmaxf(ab_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2357] =
        fmaxf(bb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 2358] =
        fmaxf(cb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 3136] =
        fmaxf(db_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 3137] =
        fmaxf(eb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 3138] =
        fmaxf(fb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 3139] =
        fmaxf(gb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 3140] =
        fmaxf(hb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 3141] =
        fmaxf(ib_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 3142] =
        fmaxf(jb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 3920] =
        fmaxf(kb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 3921] =
        fmaxf(lb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 3922] =
        fmaxf(mb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 3923] =
        fmaxf(nb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 3924] =
        fmaxf(ob_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 3925] =
        fmaxf(pb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 3926] =
        fmaxf(qb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 4704] =
        fmaxf(rb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 4705] =
        fmaxf(sb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 4706] =
        fmaxf(tb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 4707] =
        fmaxf(ub_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 4708] =
        fmaxf(vb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 4709] =
        fmaxf(wb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 4710] =
        fmaxf(xb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 5488] =
        fmaxf(yb_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 5489] =
        fmaxf(ac_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 5490] =
        fmaxf(bc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 5491] =
        fmaxf(cc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 5492] =
        fmaxf(dc_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 5493] =
        fmaxf(ec_outputRegister, 0.0F);
    outputBufferPtr[outputHeightBlockIdx * 7 + 5494] =
        fmaxf(fc_outputRegister, 0.0F);
    c_idxToStrideInputBufferAlongHe += 112;
  }
}

/*
 * Arguments    : const float X[1568]
 *                float Z[3136]
 * Return Type  : void
 */
void b_conv2dDirectOptimizedColMajor(const float X[1568], float Z[3136])
{
  static const float reformattedAndTruncatedWeights[2304] = {0.217966408F,
                                                             -0.0269200671F,
                                                             -0.166818663F,
                                                             0.14366217F,
                                                             -0.141593382F,
                                                             -0.0590293109F,
                                                             -0.0229621679F,
                                                             0.0212474074F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.134818569F,
                                                             0.0670221224F,
                                                             -0.0694886893F,
                                                             -0.0147541258F,
                                                             0.154366478F,
                                                             -0.16274564F,
                                                             -0.0520954F,
                                                             0.207867533F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0263083652F,
                                                             0.206913233F,
                                                             -0.157138243F,
                                                             -0.0707559958F,
                                                             -0.00361382565F,
                                                             0.00536517054F,
                                                             0.0357765071F,
                                                             -0.149296731F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.12575677F,
                                                             -0.0531048402F,
                                                             -0.145760521F,
                                                             0.211457565F,
                                                             -0.0544714071F,
                                                             0.200031534F,
                                                             -0.0728818104F,
                                                             -0.00836442225F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.130412251F,
                                                             0.171282783F,
                                                             0.0899129957F,
                                                             0.21129559F,
                                                             0.00187738903F,
                                                             -0.0805287436F,
                                                             -0.194143906F,
                                                             0.0343294628F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.107867807F,
                                                             0.11659354F,
                                                             -0.151604742F,
                                                             -0.185762912F,
                                                             -0.140507519F,
                                                             0.200356528F,
                                                             -0.222362146F,
                                                             -0.108984523F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.114741698F,
                                                             -0.176748589F,
                                                             0.0660752431F,
                                                             0.229537562F,
                                                             0.221689194F,
                                                             0.0498032793F,
                                                             0.147725701F,
                                                             0.0552110858F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0315481313F,
                                                             0.072679095F,
                                                             0.0633468181F,
                                                             0.0365217738F,
                                                             -0.177889466F,
                                                             -0.130572259F,
                                                             0.139212534F,
                                                             0.0531808436F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.171435133F,
                                                             0.0537913628F,
                                                             -0.165036261F,
                                                             -0.156944528F,
                                                             0.116265886F,
                                                             0.042665571F,
                                                             -0.158355936F,
                                                             0.193950117F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.434301913F,
                                                             0.191460565F,
                                                             -0.241821915F,
                                                             -0.0393971279F,
                                                             -0.321780413F,
                                                             0.447179973F,
                                                             -0.017574057F,
                                                             0.392211914F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.155126929F,
                                                             -0.0873643234F,
                                                             -0.0229360275F,
                                                             0.325464129F,
                                                             0.0711964667F,
                                                             -0.223659888F,
                                                             -0.191646457F,
                                                             -0.0156351104F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.440556F,
                                                             -0.0320058726F,
                                                             0.202056304F,
                                                             -0.0401516519F,
                                                             0.331327021F,
                                                             0.158421531F,
                                                             0.307708681F,
                                                             0.292398423F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.164533287F,
                                                             -0.398365736F,
                                                             0.445519418F,
                                                             0.20330143F,
                                                             0.211178973F,
                                                             0.064115189F,
                                                             0.121517569F,
                                                             0.142047673F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.239785716F,
                                                             -0.211986452F,
                                                             -0.188055634F,
                                                             -0.122733183F,
                                                             -0.32109946F,
                                                             0.249702051F,
                                                             0.193540707F,
                                                             -0.137243554F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.121788546F,
                                                             -0.248992234F,
                                                             0.0277555026F,
                                                             -0.327501208F,
                                                             0.317546695F,
                                                             -0.0169173554F,
                                                             -0.09457203F,
                                                             -0.28272602F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.189419419F,
                                                             0.244347021F,
                                                             0.000844446477F,
                                                             -0.0939178765F,
                                                             -0.172400594F,
                                                             0.0325674415F,
                                                             -0.142861828F,
                                                             0.0206517521F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.26057598F,
                                                             -0.257655114F,
                                                             0.308073103F,
                                                             0.294987947F,
                                                             0.186494917F,
                                                             0.185608089F,
                                                             0.292925924F,
                                                             0.28348437F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.21278441F,
                                                             -0.1584685F,
                                                             -0.106420591F,
                                                             0.070844762F,
                                                             -0.417863816F,
                                                             0.0846067667F,
                                                             0.401198119F,
                                                             -0.0892295092F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.197766677F,
                                                             0.206535548F,
                                                             0.231241062F,
                                                             -0.0908152834F,
                                                             0.0136599792F,
                                                             -0.0914448574F,
                                                             -0.110252239F,
                                                             -0.0202372931F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.107200049F,
                                                             -0.261296272F,
                                                             0.269633114F,
                                                             0.0608963557F,
                                                             -0.0891907513F,
                                                             0.190207899F,
                                                             0.0197064392F,
                                                             -0.0197587907F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.107204601F,
                                                             0.185890064F,
                                                             0.130851239F,
                                                             0.216642559F,
                                                             -0.103528664F,
                                                             -0.0769432038F,
                                                             0.0147902016F,
                                                             0.125779524F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.154533476F,
                                                             -0.149148017F,
                                                             0.360642344F,
                                                             -0.0575229228F,
                                                             -0.17983228F,
                                                             0.0377549678F,
                                                             0.111049712F,
                                                             -0.120376319F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0951074809F,
                                                             -0.14993909F,
                                                             -0.0985160843F,
                                                             0.0120848073F,
                                                             0.0388728864F,
                                                             0.0134209376F,
                                                             0.0304171648F,
                                                             0.131636277F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0422019623F,
                                                             -0.0128483567F,
                                                             -0.0414551571F,
                                                             -0.216816053F,
                                                             -0.185178757F,
                                                             -0.206233174F,
                                                             -0.224560738F,
                                                             0.0866981298F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0479416512F,
                                                             -0.227588743F,
                                                             0.213322952F,
                                                             -0.0379366167F,
                                                             -0.221780881F,
                                                             0.138831884F,
                                                             -0.253038615F,
                                                             0.0430735685F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0868890956F,
                                                             -0.0698074475F,
                                                             -0.0657187626F,
                                                             0.097261481F,
                                                             -0.203948438F,
                                                             0.241843879F,
                                                             -0.246209309F,
                                                             -0.080960229F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0860115737F,
                                                             -0.0204778686F,
                                                             -0.225206092F,
                                                             0.0924664959F,
                                                             -0.0386111401F,
                                                             -0.153903887F,
                                                             0.00909843948F,
                                                             -0.0288103018F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.112085946F,
                                                             -0.0782459F,
                                                             0.0183738563F,
                                                             0.016199939F,
                                                             0.244822949F,
                                                             -0.149534136F,
                                                             -0.184241012F,
                                                             0.410033256F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.181137219F,
                                                             0.206250966F,
                                                             -0.0372415185F,
                                                             0.0447430238F,
                                                             0.218158647F,
                                                             0.171681404F,
                                                             -0.116058864F,
                                                             0.351385593F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.179674193F,
                                                             -0.243053421F,
                                                             0.108222865F,
                                                             -0.115798868F,
                                                             -0.0590011515F,
                                                             0.0444353931F,
                                                             0.00313696172F,
                                                             0.00591687812F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0178732F,
                                                             -0.215079978F,
                                                             0.0168350618F,
                                                             0.196576342F,
                                                             0.31311819F,
                                                             0.0449524596F,
                                                             -0.0175721198F,
                                                             0.225774959F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.10749682F,
                                                             0.0407745801F,
                                                             -0.157051235F,
                                                             -0.200279713F,
                                                             0.218919232F,
                                                             0.0149646094F,
                                                             -0.270303875F,
                                                             -0.113985598F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.168452337F,
                                                             -0.0419749469F,
                                                             -0.00175963342F,
                                                             -0.0511396527F,
                                                             -0.218721494F,
                                                             -0.149675786F,
                                                             -0.11060141F,
                                                             -0.0699156448F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.182354107F,
                                                             -0.0754349381F,
                                                             -0.302089095F,
                                                             -0.103405319F,
                                                             0.0633861646F,
                                                             -0.0371888392F,
                                                             0.0181321725F,
                                                             -0.200333193F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.214610279F,
                                                             0.142484859F,
                                                             0.0399752297F,
                                                             0.0972984657F,
                                                             0.161709145F,
                                                             -0.17101939F,
                                                             0.0942221656F,
                                                             -0.16346103F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.253278404F,
                                                             0.102924399F,
                                                             0.320931166F,
                                                             0.259294063F,
                                                             -0.0340603217F,
                                                             -0.218130082F,
                                                             -0.0835471451F,
                                                             -0.234125882F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0310675651F,
                                                             -0.23632668F,
                                                             -0.0979716629F,
                                                             0.0988024473F,
                                                             0.187559649F,
                                                             -0.174547628F,
                                                             -0.0149065806F,
                                                             0.282068163F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.1775143F,
                                                             -0.0420640297F,
                                                             -0.0105179474F,
                                                             0.107546389F,
                                                             -0.107577078F,
                                                             -0.21882005F,
                                                             0.113073438F,
                                                             0.181247041F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.00930654909F,
                                                             -0.237797618F,
                                                             -0.0727112368F,
                                                             0.159516826F,
                                                             0.162008777F,
                                                             0.0382490046F,
                                                             0.0563928671F,
                                                             0.0501933396F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.172090411F,
                                                             -0.268073827F,
                                                             0.0338375568F,
                                                             0.31938681F,
                                                             0.0790984556F,
                                                             0.427583963F,
                                                             0.186058328F,
                                                             -0.0721393824F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.289657533F,
                                                             0.0908657163F,
                                                             0.202060446F,
                                                             -0.192826957F,
                                                             -0.314040214F,
                                                             -0.103321269F,
                                                             0.0937166065F,
                                                             0.0107687563F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.170357183F,
                                                             0.120632976F,
                                                             -0.255700231F,
                                                             0.101840086F,
                                                             -0.00678136945F,
                                                             -0.0100213867F,
                                                             -0.117188454F,
                                                             -0.271098047F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.310253948F,
                                                             0.128186241F,
                                                             0.107823707F,
                                                             0.200144053F,
                                                             -0.233690217F,
                                                             -0.147366852F,
                                                             -0.0271159504F,
                                                             0.22001949F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0710642934F,
                                                             0.122818038F,
                                                             -0.187930688F,
                                                             -0.154501587F,
                                                             0.128939897F,
                                                             -0.130965844F,
                                                             -0.195648491F,
                                                             -0.104676753F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.228201151F,
                                                             0.167361334F,
                                                             0.205754757F,
                                                             0.00943345763F,
                                                             -0.0810607597F,
                                                             -0.269300878F,
                                                             -0.128366113F,
                                                             -0.235854089F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0610337853F,
                                                             -0.278593928F,
                                                             0.214456961F,
                                                             0.000810205471F,
                                                             -0.282584161F,
                                                             -0.0513158962F,
                                                             0.132512227F,
                                                             0.17862615F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0973000601F,
                                                             -0.2780146F,
                                                             -0.021392649F,
                                                             0.061249882F,
                                                             0.101123892F,
                                                             0.163436934F,
                                                             -0.0235607065F,
                                                             0.222862676F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.688529551F,
                                                             0.207727507F,
                                                             -0.479368061F,
                                                             -0.323512F,
                                                             0.0302865859F,
                                                             -0.157860145F,
                                                             0.579294086F,
                                                             -0.317698538F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0404711813F,
                                                             -0.0799081624F,
                                                             0.269003123F,
                                                             0.308047712F,
                                                             -0.183975622F,
                                                             -0.437989861F,
                                                             -0.26101926F,
                                                             0.310488641F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0923072323F,
                                                             -0.175410777F,
                                                             0.407915354F,
                                                             -0.517454147F,
                                                             -0.094035849F,
                                                             0.181338951F,
                                                             0.456317365F,
                                                             0.203992695F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.266477466F,
                                                             0.307598948F,
                                                             -0.0983317196F,
                                                             -0.415056437F,
                                                             0.211887449F,
                                                             -0.00059273903F,
                                                             0.40937677F,
                                                             0.123364046F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.189337671F,
                                                             -0.339617223F,
                                                             0.297604591F,
                                                             -0.154575393F,
                                                             0.303697735F,
                                                             -0.315284431F,
                                                             0.0900891423F,
                                                             0.130674869F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.250657737F,
                                                             0.0452456772F,
                                                             0.175716788F,
                                                             -0.13376151F,
                                                             0.264518052F,
                                                             -0.242381886F,
                                                             -0.194765508F,
                                                             0.0269827899F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.178016379F,
                                                             -0.242445543F,
                                                             -0.128996521F,
                                                             -0.126899749F,
                                                             0.272807032F,
                                                             0.121125206F,
                                                             -0.0250311755F,
                                                             -0.111145325F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.269360065F,
                                                             0.3410514F,
                                                             -0.250706136F,
                                                             -0.41755563F,
                                                             0.254992217F,
                                                             -0.150425345F,
                                                             -0.135359675F,
                                                             -0.376883924F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.257124901F,
                                                             -0.0731755793F,
                                                             0.348295629F,
                                                             0.310730875F,
                                                             0.277721703F,
                                                             -0.132874653F,
                                                             0.10465268F,
                                                             0.437372595F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.163889885F,
                                                             -0.148305953F,
                                                             0.253274918F,
                                                             -0.141469195F,
                                                             -0.0838917196F,
                                                             0.175014779F,
                                                             -0.012514622F,
                                                             0.202982679F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0640459657F,
                                                             0.224020883F,
                                                             0.0971074104F,
                                                             -0.00262330589F,
                                                             0.375561893F,
                                                             -0.310837656F,
                                                             0.126848429F,
                                                             -0.0485393852F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.29986009F,
                                                             0.341525584F,
                                                             0.0464026965F,
                                                             0.229659677F,
                                                             0.660388052F,
                                                             -0.352357298F,
                                                             0.454912305F,
                                                             0.559610248F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.312491238F,
                                                             0.308792889F,
                                                             0.254262418F,
                                                             0.210419208F,
                                                             -0.174039438F,
                                                             -0.15600957F,
                                                             0.348624974F,
                                                             0.276007116F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.331017524F,
                                                             0.317549467F,
                                                             -0.0409304388F,
                                                             -0.15718843F,
                                                             0.0525530726F,
                                                             0.0313120782F,
                                                             0.0464225784F,
                                                             0.168682545F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.119490921F,
                                                             0.260407627F,
                                                             0.100116529F,
                                                             -0.139201775F,
                                                             0.366123259F,
                                                             0.107883297F,
                                                             -0.091264382F,
                                                             0.168624982F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0336170681F,
                                                             0.243854091F,
                                                             -0.227180153F,
                                                             -0.0608019419F,
                                                             -0.306323022F,
                                                             0.00508523453F,
                                                             -0.123007558F,
                                                             -0.572689414F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.23733297F,
                                                             0.156816974F,
                                                             0.0734848231F,
                                                             0.0166147836F,
                                                             0.0460969135F,
                                                             0.340054095F,
                                                             0.247236639F,
                                                             -0.173734426F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.321888059F,
                                                             -0.0209290627F,
                                                             0.0742055923F,
                                                             -0.171048F,
                                                             0.0423476696F,
                                                             -0.00889599882F,
                                                             0.467234343F,
                                                             -0.171461836F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.26386562F,
                                                             0.120466597F,
                                                             0.19848235F,
                                                             -0.0334280953F,
                                                             0.150029913F,
                                                             0.432069629F,
                                                             0.0479984842F,
                                                             0.152574658F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.104748696F,
                                                             -0.210859895F,
                                                             0.0514153466F,
                                                             0.0845680237F,
                                                             -0.212054357F,
                                                             0.199267596F,
                                                             -0.161603704F,
                                                             0.131709F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.026933996F,
                                                             -0.00995736569F,
                                                             -0.339619637F,
                                                             -0.340612769F,
                                                             0.115672402F,
                                                             -0.216950119F,
                                                             -0.0871706456F,
                                                             0.193379834F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.376858145F,
                                                             0.0217665173F,
                                                             -0.359918F,
                                                             -0.182828665F,
                                                             0.166075498F,
                                                             -0.365658969F,
                                                             0.169786543F,
                                                             0.340468436F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0200405512F,
                                                             0.261857778F,
                                                             0.0506375171F,
                                                             -0.00670097815F,
                                                             -0.00112723815F,
                                                             0.244824767F,
                                                             0.161654741F,
                                                             -0.0900254324F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.124032073F,
                                                             -0.269982249F,
                                                             0.043225497F,
                                                             -0.335701704F,
                                                             -0.25700745F,
                                                             -0.0623012111F,
                                                             0.421287686F,
                                                             -0.0849434F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0391564853F,
                                                             0.016277032F,
                                                             0.198806837F,
                                                             0.149892539F,
                                                             -0.250804931F,
                                                             0.0917735174F,
                                                             0.160596699F,
                                                             0.103648454F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0423842371F,
                                                             0.0585987158F,
                                                             0.180835873F,
                                                             0.151899084F,
                                                             -0.204645962F,
                                                             0.0711172596F,
                                                             0.271571606F,
                                                             0.206660271F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.064613007F,
                                                             0.145626932F,
                                                             -0.0453568362F,
                                                             -0.132517412F,
                                                             -0.190254495F,
                                                             0.172433525F,
                                                             0.244390041F,
                                                             -0.0167282838F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0985716507F,
                                                             0.115284756F,
                                                             0.0489267595F,
                                                             0.0547122359F,
                                                             0.144145831F,
                                                             -0.0327333175F,
                                                             0.128747016F,
                                                             -0.0479280464F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0493111722F,
                                                             0.0789617673F,
                                                             -0.281787068F,
                                                             0.102818757F,
                                                             0.0786072835F,
                                                             -0.17690146F,
                                                             0.113960512F,
                                                             -0.255506396F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0109768156F,
                                                             0.179985017F,
                                                             0.0633435696F,
                                                             -0.160966679F,
                                                             -0.0215196665F,
                                                             0.0864923298F,
                                                             -0.079299055F,
                                                             0.195204228F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.11374516F,
                                                             0.000377067598F,
                                                             -0.0790119767F,
                                                             0.132982075F,
                                                             -0.0930787474F,
                                                             -0.111773022F,
                                                             0.125209168F,
                                                             -0.134876758F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.107784458F,
                                                             0.122609697F,
                                                             -0.192586467F,
                                                             -0.18910037F,
                                                             -0.158878446F,
                                                             -0.121322975F,
                                                             0.169634327F,
                                                             -0.120242156F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0144731961F,
                                                             -0.041412469F,
                                                             -0.0362364314F,
                                                             0.073984839F,
                                                             0.223813087F,
                                                             -0.208328649F,
                                                             -0.127467245F,
                                                             -0.0810189247F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0205720868F,
                                                             0.185918793F,
                                                             -0.159997642F,
                                                             -0.206751093F,
                                                             -0.166982F,
                                                             -0.18930009F,
                                                             -0.217885152F,
                                                             0.215551466F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.402273327F,
                                                             -0.35832727F,
                                                             0.0208334755F,
                                                             -0.0928338245F,
                                                             -0.329179585F,
                                                             0.147986352F,
                                                             0.107569404F,
                                                             -0.0849034712F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.26422444F,
                                                             0.186459303F,
                                                             0.248641953F,
                                                             0.263967842F,
                                                             -0.192920193F,
                                                             -0.277618438F,
                                                             -0.195156366F,
                                                             0.382762402F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.289534479F,
                                                             -0.112870529F,
                                                             -0.226809204F,
                                                             0.0496182106F,
                                                             -0.340058714F,
                                                             -0.294135541F,
                                                             0.372617066F,
                                                             -0.129371107F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.197311625F,
                                                             -0.29549107F,
                                                             0.0203151591F,
                                                             -0.378059715F,
                                                             0.249212757F,
                                                             0.325267524F,
                                                             -0.373654455F,
                                                             -0.26440835F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.255302846F,
                                                             -0.413780808F,
                                                             0.127576694F,
                                                             -0.144450799F,
                                                             0.182413429F,
                                                             0.412659824F,
                                                             -0.0876008347F,
                                                             -0.196867898F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.135953218F,
                                                             0.0623213761F,
                                                             -0.0471189842F,
                                                             -0.202062711F,
                                                             -0.0107110506F,
                                                             -0.19227156F,
                                                             0.0381205902F,
                                                             0.0308540706F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.124871083F,
                                                             -0.40283367F,
                                                             0.173518881F,
                                                             0.0615778342F,
                                                             0.101668946F,
                                                             0.273283035F,
                                                             0.0282760728F,
                                                             -0.0714759827F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.11822322F,
                                                             -0.0341402069F,
                                                             0.642006397F,
                                                             0.0683729351F,
                                                             0.161157668F,
                                                             0.0866709501F,
                                                             -0.0882695094F,
                                                             0.235227242F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.28672722F,
                                                             -0.175961792F,
                                                             0.160091862F,
                                                             0.418024898F,
                                                             -0.207402959F,
                                                             0.34754926F,
                                                             -0.139954299F,
                                                             -0.0553910546F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.142609134F,
                                                             -0.0423809551F,
                                                             0.441671729F,
                                                             0.124829531F,
                                                             -0.218267307F,
                                                             -0.179982692F,
                                                             0.239629477F,
                                                             0.494396925F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.125064209F,
                                                             0.0750649348F,
                                                             0.580425382F,
                                                             0.440500915F,
                                                             0.575358093F,
                                                             -0.187520117F,
                                                             0.0911446735F,
                                                             0.429078579F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.147214353F,
                                                             0.154206544F,
                                                             0.288616478F,
                                                             -0.334487438F,
                                                             0.308986932F,
                                                             -0.247240037F,
                                                             -0.230830163F,
                                                             -0.107583649F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.710364819F,
                                                             -0.332072407F,
                                                             0.532482445F,
                                                             0.160481766F,
                                                             -0.206025541F,
                                                             0.214770347F,
                                                             0.0894558132F,
                                                             0.267884493F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.446029067F,
                                                             0.386154026F,
                                                             0.0464680046F,
                                                             -0.332828611F,
                                                             -0.306341439F,
                                                             -0.12214683F,
                                                             -0.255252361F,
                                                             -0.00681978883F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.414967209F,
                                                             0.473440737F,
                                                             -0.0675250515F,
                                                             -0.208015606F,
                                                             -0.215304807F,
                                                             -0.0406431742F,
                                                             -0.0453616828F,
                                                             -0.375981838F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.478105903F,
                                                             -0.336608976F,
                                                             -0.666018844F,
                                                             0.264878452F,
                                                             0.140382051F,
                                                             0.425966531F,
                                                             0.128026053F,
                                                             -0.556076169F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0810360759F,
                                                             0.15601173F,
                                                             0.215666845F,
                                                             -0.144909054F,
                                                             0.10293337F,
                                                             -0.312882125F,
                                                             -0.20780009F,
                                                             -0.667738199F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0844071507F,
                                                             -0.13130714F,
                                                             0.340985268F,
                                                             -0.0492821708F,
                                                             -0.286465704F,
                                                             0.136880696F,
                                                             0.083146818F,
                                                             0.0347238034F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.280583858F,
                                                             0.0212338679F,
                                                             0.0241817813F,
                                                             -0.258989096F,
                                                             -0.139264807F,
                                                             -0.0742554963F,
                                                             0.0490282215F,
                                                             0.246242881F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.194772214F,
                                                             -0.242587283F,
                                                             0.176782325F,
                                                             0.0802786499F,
                                                             -0.139386579F,
                                                             -0.20114693F,
                                                             -0.134303629F,
                                                             -0.243810505F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0776440725F,
                                                             0.0275648609F,
                                                             0.00920392666F,
                                                             -0.167664543F,
                                                             -0.12482623F,
                                                             -0.0802843571F,
                                                             -0.123381846F,
                                                             -0.228742197F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.00511887483F,
                                                             0.258406907F,
                                                             0.0312603451F,
                                                             -0.0470572375F,
                                                             -0.242394373F,
                                                             0.0907729939F,
                                                             -0.296170712F,
                                                             -0.253810316F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0527092703F,
                                                             -0.131687492F,
                                                             -0.0488603041F,
                                                             -0.064687714F,
                                                             0.105611652F,
                                                             -0.00966569781F,
                                                             0.16228202F,
                                                             0.141424865F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.0374372564F,
                                                             0.303319961F,
                                                             -0.025729781F,
                                                             0.130324855F,
                                                             0.118468136F,
                                                             -0.0290936902F,
                                                             -0.0395697281F,
                                                             -0.286324054F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.221949473F,
                                                             0.0952698812F,
                                                             -0.183318496F,
                                                             0.190150619F,
                                                             -0.253194213F,
                                                             0.0855535194F,
                                                             -0.272413254F,
                                                             0.0321892537F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.446553975F,
                                                             0.180092797F,
                                                             -0.13489683F,
                                                             0.294045925F,
                                                             -0.229259849F,
                                                             -0.220501721F,
                                                             0.249261379F,
                                                             -0.288322F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.227914646F,
                                                             -0.177440926F,
                                                             -0.444167107F,
                                                             0.28570056F,
                                                             -0.368570715F,
                                                             0.268246889F,
                                                             -0.0803520307F,
                                                             -0.313855439F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.197216332F,
                                                             0.47796F,
                                                             -0.130907431F,
                                                             0.0875919834F,
                                                             0.0855617821F,
                                                             0.227848262F,
                                                             -0.156918377F,
                                                             -0.274777293F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0919944346F,
                                                             -0.383759F,
                                                             0.0903542861F,
                                                             -0.338073373F,
                                                             0.54112637F,
                                                             0.0506161749F,
                                                             -0.115784757F,
                                                             0.162818F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0187462606F,
                                                             0.451753676F,
                                                             0.29127869F,
                                                             -0.289561361F,
                                                             -0.238135427F,
                                                             -0.0363657475F,
                                                             0.309563518F,
                                                             0.196232483F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.459338337F,
                                                             0.0482245758F,
                                                             0.508907795F,
                                                             -0.271523476F,
                                                             -0.163750201F,
                                                             0.113420285F,
                                                             0.0288455505F,
                                                             0.352441758F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.252269357F,
                                                             0.143659174F,
                                                             -0.0290139578F,
                                                             0.134251267F,
                                                             0.687851787F,
                                                             -0.223926991F,
                                                             0.108974405F,
                                                             0.598169744F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.246297956F,
                                                             -0.144633815F,
                                                             0.3735618F,
                                                             0.0585184284F,
                                                             0.35952422F,
                                                             0.220965073F,
                                                             -0.367735922F,
                                                             0.00815594941F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.177353308F,
                                                             0.16629152F,
                                                             0.195397064F,
                                                             0.24559848F,
                                                             0.332187116F,
                                                             -0.29467082F,
                                                             -0.105120756F,
                                                             -0.546993434F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.170029908F,
                                                             0.247067407F,
                                                             -0.116904609F,
                                                             -0.0110542495F,
                                                             -0.0953629687F,
                                                             0.092482321F,
                                                             -0.213767275F,
                                                             0.0430602394F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.243369952F,
                                                             -0.285334945F,
                                                             0.679420352F,
                                                             -0.151778623F,
                                                             0.390053153F,
                                                             -0.48260954F,
                                                             0.24141863F,
                                                             0.193086222F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.126626328F,
                                                             0.084118F,
                                                             0.404974282F,
                                                             0.0740158632F,
                                                             0.195499465F,
                                                             0.0794935226F,
                                                             -0.167409509F,
                                                             0.179831237F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.228341892F,
                                                             -0.184265658F,
                                                             0.191795737F,
                                                             0.16675435F,
                                                             0.425666809F,
                                                             -0.0953817517F,
                                                             0.184337184F,
                                                             0.185856402F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.186999395F,
                                                             -0.033525832F,
                                                             0.00597676355F,
                                                             -0.0280583985F,
                                                             0.160887137F,
                                                             -0.199076F,
                                                             -0.202798545F,
                                                             -0.160333812F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.387299329F,
                                                             -0.103143916F,
                                                             0.389283895F,
                                                             -0.396993577F,
                                                             -0.275721818F,
                                                             0.0690589696F,
                                                             -0.290549517F,
                                                             0.158702299F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.285457939F,
                                                             -0.377002686F,
                                                             0.0893540829F,
                                                             0.00309275859F,
                                                             0.296277374F,
                                                             -0.0375449173F,
                                                             -0.0255618319F,
                                                             -0.156493366F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.362757742F,
                                                             0.273519635F,
                                                             -0.231897324F,
                                                             0.202560037F,
                                                             0.306489199F,
                                                             0.223861367F,
                                                             0.0615063943F,
                                                             0.38641879F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.180995896F,
                                                             0.191612542F,
                                                             -0.0740824118F,
                                                             -0.218465939F,
                                                             0.132189706F,
                                                             -0.073014155F,
                                                             0.125250787F,
                                                             0.356298119F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.184860826F,
                                                             -0.3345671F,
                                                             0.345732361F,
                                                             -0.185824588F,
                                                             -0.152918667F,
                                                             -0.12511678F,
                                                             0.129421696F,
                                                             0.158122405F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0899651572F,
                                                             -0.0913093F,
                                                             0.171822757F,
                                                             0.050655432F,
                                                             0.354140729F,
                                                             -0.103331F,
                                                             0.350996435F,
                                                             0.347632676F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.268606633F,
                                                             -0.221907109F,
                                                             -0.286987036F,
                                                             0.139690086F,
                                                             -0.0863215178F,
                                                             -0.20584102F,
                                                             -0.156025648F,
                                                             -0.0160205569F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0798615143F,
                                                             0.300802261F,
                                                             -0.124398492F,
                                                             -0.0178078394F,
                                                             -0.199912488F,
                                                             0.00344731612F,
                                                             -0.0833234F,
                                                             -0.0419618897F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.307546437F,
                                                             0.190263644F,
                                                             -0.147617757F,
                                                             0.18248F,
                                                             -0.358442068F,
                                                             0.199653074F,
                                                             0.0638226569F,
                                                             -0.12856634F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.281873256F,
                                                             0.0318901464F,
                                                             0.0194914918F,
                                                             -0.0282675792F,
                                                             -0.371348977F,
                                                             0.0215978101F,
                                                             0.122697361F,
                                                             -0.0544349253F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.32254076F,
                                                             -0.299295366F,
                                                             0.21852085F,
                                                             0.332377F,
                                                             -0.243238211F,
                                                             0.0955464467F,
                                                             -0.289580107F,
                                                             -0.59815222F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.151996389F,
                                                             0.0279799812F,
                                                             -0.212507978F,
                                                             0.198020041F,
                                                             -0.234537229F,
                                                             -0.250243515F,
                                                             0.241874307F,
                                                             -0.225766331F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.171696067F,
                                                             0.115980573F,
                                                             0.171989515F,
                                                             0.278415471F,
                                                             -0.120454803F,
                                                             -0.208529145F,
                                                             0.0457814112F,
                                                             0.0859491602F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.161802605F,
                                                             0.126657113F,
                                                             -0.360558748F,
                                                             0.393384695F,
                                                             0.0108119585F,
                                                             -0.00105210172F,
                                                             0.000986992382F,
                                                             -0.327637404F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.117905788F,
                                                             -0.058248464F,
                                                             0.0878974497F,
                                                             0.0896341F,
                                                             0.0787308514F,
                                                             0.163626298F,
                                                             -0.260540068F,
                                                             -0.00305625587F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.345755875F,
                                                             -0.0949145555F,
                                                             -0.375578284F,
                                                             0.144904494F,
                                                             0.228171974F,
                                                             -0.288526207F,
                                                             0.266099F,
                                                             -0.357050627F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.376094908F,
                                                             -0.629299F,
                                                             -0.0358141847F,
                                                             0.0987949818F,
                                                             -0.36172235F,
                                                             -0.0419615805F,
                                                             -0.138534471F,
                                                             0.321532071F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.165589571F,
                                                             0.0498067774F,
                                                             -0.222881243F,
                                                             -0.0232422762F,
                                                             0.241924748F,
                                                             -0.0699514598F,
                                                             0.186684936F,
                                                             0.241176859F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.13994889F,
                                                             -0.287709773F,
                                                             -0.351082295F,
                                                             0.21279943F,
                                                             0.00951359421F,
                                                             0.0485416129F,
                                                             0.30696556F,
                                                             -0.0283595026F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.179041058F,
                                                             -0.275007844F,
                                                             -0.0641296431F,
                                                             -0.0483999252F,
                                                             -0.23837328F,
                                                             0.316559166F,
                                                             -0.0409914479F,
                                                             0.430299222F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.183098912F,
                                                             -0.493578613F,
                                                             -0.288070291F,
                                                             -0.456961781F,
                                                             -0.0152775394F,
                                                             0.399220943F,
                                                             0.00724925194F,
                                                             0.465476394F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.47751078F,
                                                             0.177344754F,
                                                             0.0917308778F,
                                                             -0.0368946195F,
                                                             -0.519952297F,
                                                             0.37341857F,
                                                             -0.346623033F,
                                                             -0.269257456F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0287261307F,
                                                             -0.165115476F,
                                                             -0.0139572136F,
                                                             -0.0715441555F,
                                                             0.278430045F,
                                                             0.241350368F,
                                                             -0.290611953F,
                                                             0.0774937123F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             -0.272194654F,
                                                             -0.137006104F,
                                                             0.3410092F,
                                                             0.359271258F,
                                                             0.360172F,
                                                             0.100420132F,
                                                             0.244457558F,
                                                             0.179376498F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F,
                                                             0.0F};
  static const float biasReformatted[16] = {
      -0.342148274F, 0.0331672728F, 0.173396364F,  -0.102866396F,
      0.023172887F,  -0.169731334F, -1.17096734F,  -0.439235955F,
      0.129483461F,  0.357094347F,  -0.319511384F, 0.682772338F,
      -1.36194468F,  -0.596095443F, 0.37525627F,   0.367722303F};
  b_convolution(&X[0], &Z[0], &reformattedAndTruncatedWeights[0],
                &biasReformatted[0]);
}

/*
 * Arguments    : const float X[784]
 *                float Z[1568]
 * Return Type  : void
 */
void c_conv2dDirectOptimizedColMajor(const float X[784], float Z[1568])
{
  static const float reformattedAndTruncatedWeights[4608] = {
      -0.0378298424F,   0.00222212449F,  -0.0953017175F,   0.0740959942F,
      -0.10818845F,     -0.0990690887F,  -0.0911500305F,   -0.0694585815F,
      -0.0852030069F,   -0.0239171442F,  -0.0291433427F,   -0.060722813F,
      -0.130843744F,    0.0330467261F,   0.00150762917F,   0.0302749947F,
      0.0795808434F,    0.0436458513F,   -0.0385965034F,   -0.0756471902F,
      -0.106741697F,    -0.0262984149F,  0.129477248F,     -0.126689479F,
      -0.0382122F,      0.0982449874F,   -0.0248148963F,   0.039150767F,
      0.0763801187F,    -0.00927144103F, 0.0694244206F,    0.120388418F,
      0.0718254372F,    0.072168F,       0.0771071315F,    0.136454895F,
      0.034946762F,     0.00699468423F,  0.031714011F,     0.0134212654F,
      0.00750202639F,   0.00246610469F,  0.10828767F,      -0.0161127057F,
      -0.00670678541F,  0.00579599245F,  0.13441135F,      0.100673564F,
      0.0433362685F,    0.00879435148F,  -0.0573210157F,   -0.0148046538F,
      -0.0987671316F,   -0.115492396F,   -0.0799376369F,   -0.0450900458F,
      0.0247744862F,    -0.0627929643F,  -0.0674913302F,   0.128438801F,
      0.0058434559F,    -0.0717907697F,  0.0924445912F,    -0.0054259114F,
      -0.0951745212F,   -0.0513008423F,  -0.0694009438F,   -0.0261674784F,
      -0.0329125F,      -0.0986166596F,  0.0255027879F,    -0.0955294892F,
      -0.0963297635F,   0.00815780088F,  0.0261192508F,    0.0464683957F,
      0.0806360468F,    -0.0606688485F,  0.00179543567F,   -0.121702537F,
      0.0758507401F,    -0.11579214F,    0.084685348F,     -0.069819361F,
      0.0803109482F,    0.013717602F,    0.0295173712F,    0.0695892423F,
      0.0913523063F,    0.0666013658F,   0.0563097894F,    0.00443462469F,
      0.0953761F,       -0.0730855465F,  0.150688842F,     0.0368057676F,
      0.180305183F,     0.109971546F,    -0.0665682629F,   0.0923892558F,
      0.175826535F,     0.0583884418F,   0.0659984723F,    0.0298229773F,
      -0.00163168088F,  -0.106535912F,   0.0176961385F,    0.0390197225F,
      0.0262526628F,    -0.104275964F,   -0.00374968071F,  0.0543096215F,
      -0.0345661454F,   -0.0333369188F,  0.0556084923F,    -0.0110030668F,
      -0.000640235259F, -0.112610675F,   -0.0609815642F,   0.0182079729F,
      -0.0228656791F,   -0.0792912319F,  0.0808768198F,    -0.00227974425F,
      -0.0678674355F,   -0.00579148903F, -0.117898427F,    -0.097280845F,
      -0.10021735F,     0.00356915197F,  -0.0156827848F,   -0.00865349919F,
      -0.0474774428F,   -0.0139355147F,  -0.0877379F,      -0.113348015F,
      -0.116754398F,    0.14630942F,     0.0561856329F,    0.0738430619F,
      0.120498665F,     0.0765176713F,   0.0187196955F,    0.0266161915F,
      -0.00525888614F,  -0.0365088172F,  -0.0939724818F,   0.0722538307F,
      0.0672638491F,    0.147906318F,    0.148283184F,     -0.115922369F,
      0.0698092729F,    0.0305442177F,   -0.01975099F,     0.182214439F,
      -0.0424906574F,   -0.174645036F,   0.114702716F,     -0.189239666F,
      -0.0427639596F,   0.0209982507F,   -0.0363539606F,   -0.128282279F,
      -0.0424863845F,   0.121763773F,    -0.0640183389F,   0.0828991F,
      0.0786971524F,    0.0336280726F,   0.123092324F,     0.0344961099F,
      -0.0817746669F,   -0.102164842F,   0.145197943F,     0.123345964F,
      0.0919755F,       -0.020359898F,   -0.0574494451F,   -0.0926190317F,
      0.0926275104F,    -0.0219930187F,  -0.105271049F,    -0.0964589193F,
      0.0788842961F,    -0.0532232821F,  -0.0190740731F,   -0.0726485401F,
      -0.0736231059F,   -0.0189203918F,  0.146678075F,     0.0712263286F,
      -0.0447168797F,   -0.166281715F,   0.111203074F,     0.184995681F,
      0.157334641F,     0.0176196508F,   0.120475791F,     -0.182948008F,
      -0.153176278F,    0.0826791897F,   0.00171730272F,   0.00618970487F,
      0.128750443F,     -0.0569167398F,  0.171787187F,     -0.182072595F,
      -0.0307396F,      0.00498031033F,  -0.00769564789F,  0.102708168F,
      0.0902038366F,    -0.129249543F,   0.0310330465F,    0.0604172945F,
      -0.0863525495F,   0.0432952307F,   0.098298274F,     0.0867376F,
      0.0315841027F,    -0.0236440226F,  0.109186284F,     -0.0153934583F,
      -0.0890408158F,   0.141032889F,    0.073386088F,     -0.0669178516F,
      -0.00763550447F,  0.125878856F,    -0.137433663F,    0.0269465148F,
      -0.0995332599F,   0.11314483F,     0.0886510462F,    -0.149660096F,
      0.0343418904F,    0.192134991F,    -0.0926080346F,   0.118707784F,
      -0.023872463F,    -0.0479217246F,  0.0394976139F,    -0.0216545034F,
      0.115026563F,     -0.0828846395F,  -0.0406868272F,   -0.0617068708F,
      0.0779390335F,    -0.0927528292F,  0.189946383F,     0.167531207F,
      0.116355538F,     -0.143129572F,   0.134645611F,     -0.204100028F,
      -0.0269943234F,   -0.0146666402F,  0.0675400794F,    -0.00232034852F,
      0.0355502032F,    -0.0979864225F,  0.13702403F,      0.0478082113F,
      0.0207605418F,    -0.0938114151F,  -0.0186674669F,   0.043427255F,
      -0.0496498086F,   -0.0784336403F,  -0.0500556789F,   -0.136358157F,
      0.12187393F,      -0.078876026F,   0.0536426269F,    -0.00441759266F,
      -0.105143756F,    0.151766181F,    0.133518964F,     -0.0394385643F,
      0.0191406589F,    -0.104254395F,   -0.0520585254F,   0.0325589776F,
      -0.0469291024F,   0.183064595F,    -0.00865099113F,  0.00334985461F,
      0.131427199F,     -0.0848916769F,  -0.0811229646F,   -0.064648807F,
      -0.0451078042F,   -0.0873895437F,  0.0488895439F,    -0.0975796506F,
      0.000610865129F,  -0.207387835F,   0.0150760226F,    -0.0550805517F,
      0.0934200063F,    -0.124235101F,   -0.100974113F,    -0.070873037F,
      -0.0657200515F,   -0.0997766F,     -0.0244712792F,   0.0483982265F,
      -0.105781212F,    -0.189775333F,   -0.0527240746F,   -0.194484308F,
      -0.00583749777F,  -0.0723413602F,  0.1096365F,       0.065933831F,
      0.0441227742F,    -0.0123617258F,  0.00742799044F,   0.0250844546F,
      -0.147540733F,    -0.169761151F,   -0.0142138051F,   0.0447154902F,
      0.113415308F,     -0.0804726705F,  0.0591045469F,    -0.0110977236F,
      -0.126068115F,    0.0800021738F,   0.0432775F,       0.0102822231F,
      0.0445732065F,    0.0306673367F,   -0.0195369665F,   0.0205249954F,
      0.0564209558F,    0.0419679768F,   0.0737392083F,    0.102033705F,
      0.115497425F,     -0.091412805F,   -0.0937682167F,   0.0631710067F,
      -0.108097121F,    -0.0999407247F,  -0.046182964F,    -0.0870615244F,
      0.0430161618F,    -0.0671212673F,  -0.108457349F,    0.0724598318F,
      0.158935875F,     -0.12417195F,    -0.183405921F,    0.0759298727F,
      -0.0270146076F,   -0.0341821685F,  0.0260115452F,    0.0359566472F,
      -0.152600959F,    0.000376114825F, 0.0856179893F,    -0.0475159287F,
      0.120878465F,     0.185035527F,    0.0315771252F,    0.140204504F,
      0.156301886F,     0.0866437778F,   -0.0577323586F,   0.15486294F,
      -0.116906926F,    0.0895509794F,   0.144776464F,     0.0527163707F,
      -0.159256309F,    0.1742322F,      -0.125396609F,    0.0160417445F,
      0.0830949172F,    0.206735179F,    -0.0564890318F,   0.0137105752F,
      0.057531368F,     0.113668025F,    0.0470332764F,    0.0695595667F,
      0.0712919757F,    0.00467831036F,  -0.138531908F,    -0.0278294384F,
      0.0819927529F,    -0.0497736335F,  0.0404765233F,    0.100416087F,
      -0.0158680454F,   -0.0661575571F,  0.0327529609F,    -0.0685190856F,
      -0.03541F,        0.129725754F,    -0.0545854382F,   -0.0644948408F,
      -0.0111616747F,   0.132838845F,    0.065438427F,     -0.0797020346F,
      0.108024232F,     -0.0972095057F,  0.0816864073F,    0.0252237339F,
      0.106115244F,     0.112933837F,    -0.106185444F,    -0.00196809275F,
      0.0700572729F,    0.153595597F,    -0.0989247039F,   -0.0160834752F,
      0.0665613711F,    0.137073368F,    -0.127563253F,    0.0513988025F,
      0.17198655F,      -0.0127613302F,  -0.100792393F,    -0.0681899637F,
      -0.162609875F,    0.161348283F,    0.118019558F,     0.0423011892F,
      0.085069418F,     0.126145333F,    -0.0501561686F,   0.0430918261F,
      0.0498249568F,    0.0750372335F,   0.113456123F,     -0.084396787F,
      0.00526167871F,   0.0261379965F,   0.107538044F,     -0.0879285112F,
      0.0142831644F,    -0.103732951F,   0.0140190069F,    -0.00551207829F,
      -0.0112528196F,   -0.0122500621F,  -0.0202613343F,   0.0500638261F,
      0.119103149F,     -0.107294008F,   0.0212774761F,    -0.0280350596F,
      -0.0916681662F,   -0.096362792F,   0.0265630577F,    0.0950642F,
      -0.000154026071F, 0.048733F,       -0.0256569907F,   -0.0141177401F,
      -0.0222150739F,   0.0524822511F,   0.00835696328F,   -0.125568673F,
      -0.0744217262F,   -0.0249114297F,  -0.0728813559F,   0.129616886F,
      -0.0912445709F,   0.124180615F,    -0.192163438F,    -0.0707645491F,
      0.0483598411F,    0.0990278199F,   -0.0127989491F,   0.139161736F,
      -0.0489200056F,   0.0288151316F,   0.0357811712F,    -0.0892498791F,
      0.0594238639F,    0.078041F,       0.0940780267F,    -0.0401381962F,
      0.106414095F,     -0.0784349218F,  -0.068622537F,    -0.102959648F,
      -0.0189128611F,   -0.0538890623F,  0.046818F,        0.0995187312F,
      0.127231225F,     0.0936420709F,   -0.0146241495F,   -0.00196716399F,
      0.140877962F,     -0.0448113382F,  0.0447757505F,    -0.0528827719F,
      -0.0205887891F,   0.0331833F,      0.161223918F,     -0.0601445697F,
      0.0635185391F,    0.0179644711F,   -0.0616943687F,   0.097184822F,
      -0.0305034369F,   -0.0763605088F,  0.0749824271F,    0.0779600367F,
      -0.0644394159F,   -0.0218977295F,  -0.0724356398F,   0.0830789357F,
      -0.125367045F,    0.0789167F,      -0.0559207611F,   0.0948159099F,
      -0.0677185655F,   0.00969366543F,  -0.0569669716F,   0.01864071F,
      0.0858240277F,    -0.0186841395F,  0.0933882073F,    0.108052433F,
      -0.081658788F,    0.165694579F,    0.160442978F,     0.0895848274F,
      -0.0585849844F,   -0.0700825825F,  -0.0757219F,      0.0468117818F,
      0.030470252F,     0.0619186088F,   -0.145292968F,    -0.155715093F,
      -0.0257226396F,   -0.0634844452F,  0.118812673F,     -0.0886548534F,
      0.00723101432F,   -0.0386381932F,  0.1411286F,       0.0425985381F,
      -0.0347023606F,   0.0278501F,      -0.00808795F,     -0.0529900715F,
      0.0695748702F,    -0.108047269F,   -0.038484104F,    -0.0201640632F,
      -0.100504078F,    -0.0128231747F,  0.0858195201F,    0.114573859F,
      0.12934804F,      -0.05915327F,    0.0752606243F,    -0.04717388F,
      -0.028607605F,    0.0149989491F,   0.196448907F,     -0.0667154863F,
      0.0815086439F,    0.154749751F,    0.0549586192F,    -0.117087036F,
      -0.124368496F,    0.0382277779F,   -0.00399513636F,  0.0190642122F,
      0.0286712125F,    0.0467794873F,   -0.0737630874F,   0.0410972349F,
      -0.044314947F,    0.00506407302F,  0.112086222F,     -0.219535679F,
      0.131227627F,     0.130773678F,    -0.105798833F,    -0.0294096209F,
      -0.113250047F,    0.00434207963F,  -0.0572172143F,   -0.0844396725F,
      -0.131449625F,    -0.024378391F,   0.0963022858F,    -0.157355607F,
      -0.0138688311F,   0.16759038F,     0.080050163F,     -0.0983607695F,
      0.0932151154F,    0.143123478F,    -0.137408271F,    0.0458722971F,
      0.123369277F,     -0.044347439F,   0.0481536388F,    0.0822179F,
      0.0306363925F,    0.0811171383F,   0.0651797801F,    0.0512714684F,
      -0.0112239346F,   -0.0843163505F,  0.0390858464F,    -0.133740544F,
      0.218079388F,     -0.0575855747F,  -0.121840209F,    0.0296240486F,
      0.130546957F,     -0.0783570856F,  -0.0282024778F,   0.204147816F,
      0.0726194605F,    -0.034464F,      0.166094959F,     -0.182448417F,
      -0.0724313185F,   -0.0902164876F,  0.228917599F,     0.0818501F,
      0.12273436F,      -0.0482377782F,  -0.19262591F,     -0.0771645084F,
      -0.07802248F,     -0.173254654F,   -0.0663598F,      0.0124397259F,
      -0.0907319337F,   -0.0475444607F,  0.215776622F,     -0.0912062228F,
      0.0734490678F,    0.0104723368F,   0.111198954F,     -0.0481523126F,
      0.059180744F,     0.185773119F,    -0.119862974F,    -0.179502428F,
      0.094979994F,     -0.046849627F,   -0.00468116114F,  0.0498169139F,
      -0.03603407F,     -0.0196707F,     -0.0496279299F,   -0.036486771F,
      0.0430832058F,    -0.0102268532F,  0.227908045F,     -0.137516692F,
      0.136281192F,     0.0740878507F,   -0.139275163F,    -0.0126104355F,
      -0.0244296156F,   0.0615137741F,   0.0689291358F,    0.123614252F,
      0.0515192561F,    -0.109431818F,   0.132650077F,     -0.0207044706F,
      0.04575409F,      0.140430614F,    0.0169011578F,    0.0146920271F,
      0.148595348F,     0.131651178F,    0.00607679971F,   0.0588141233F,
      0.115294583F,     -0.0148221394F,  0.0979817957F,    0.0278959479F,
      0.103098415F,     0.0552407578F,   0.184480011F,     -0.161390275F,
      -0.0527680442F,   -0.173713163F,   0.209948033F,     0.0458791032F,
      0.00315331621F,   -0.100561619F,   0.0984510556F,    -0.0588465519F,
      0.0127489492F,    -0.087845549F,   -0.107845269F,    0.130139709F,
      -0.0380876213F,   -0.120012507F,   0.116788372F,     -0.180844307F,
      -0.00711843232F,  0.0305916201F,   0.1305224F,       0.00095964F,
      0.160778716F,     -0.0835604444F,  0.124737218F,     -0.171270847F,
      0.0446190797F,    0.0672439486F,   0.0796633288F,    0.120994486F,
      -0.0802176595F,   -0.0438400023F,  -0.0396436155F,   -0.0652610809F,
      0.0770544484F,    0.0606417507F,   -0.113852188F,    -0.0539420955F,
      0.0389117375F,    -0.0611921363F,  0.0315745212F,    0.00725713791F,
      0.0368891917F,    0.0962438062F,   0.0742339641F,    0.0344031304F,
      -0.104811676F,    -0.0751640201F,  0.066283986F,     -0.00512181735F,
      -0.016094083F,    -0.0010911877F,  -0.0837474838F,   0.0592233315F,
      0.0332190879F,    0.00495303376F,  -0.07261803F,     0.0117456829F,
      -0.104496494F,    -0.0585878193F,  0.0896078721F,    -0.0720350519F,
      0.0078368457F,    -0.0229684319F,  0.0285925344F,    0.0877651051F,
      0.0774255842F,    0.0958570093F,   -0.0749022F,      -0.0752749816F,
      -0.0225700401F,   -0.0841302574F,  -0.0512436517F,   -0.0810691193F,
      -0.0229227915F,   -0.0465634726F,  -0.123904251F,    0.0900798F,
      0.0315608867F,    0.00381955504F,  -0.0336494558F,   0.00146644982F,
      0.0508618094F,    -0.0160139184F,  -0.0793960318F,   -0.0311735608F,
      0.0323847346F,    -0.0255483426F,  -0.0302622169F,   -0.109543778F,
      0.0846810341F,    -0.0290079396F,  0.0493014976F,    0.078714408F,
      0.0348304696F,    -0.0733962F,     0.105547652F,     -0.0894732252F,
      -0.110425644F,    0.0249563102F,   -0.124213554F,    -0.0425814725F,
      -0.0545485429F,   -0.0152082462F,  0.0212059226F,    -0.00297781755F,
      -0.0829710215F,   0.0891709626F,   -0.027521668F,    0.126434639F,
      -0.0294459797F,   0.10982167F,     -0.0406158678F,   -0.0790714249F,
      0.07092309F,      -0.05845185F,    0.0850493237F,    -0.0754174963F,
      -0.0223182663F,   -0.0678793192F,  -0.0144494781F,   -0.0450841412F,
      0.00130190933F,   -0.00903770421F, 0.0567855611F,    -0.0873716176F,
      0.104883768F,     0.0690506473F,   -0.0239839852F,   0.0943709537F,
      0.0765804052F,    -0.111776292F,   -0.0170266889F,   0.139867231F,
      0.09406849F,      0.0504346751F,   0.0976802F,       0.0268161129F,
      0.0139405308F,    -0.0830512717F,  -0.072672084F,    -0.0963889956F,
      -0.0339085646F,   -0.10799206F,    0.0391417928F,    -0.0245137233F,
      -0.0703138039F,   0.0681281313F,   -0.104121372F,    0.0572787486F,
      0.0141488463F,    0.0381304175F,   0.0432539806F,    0.0267502926F,
      0.0298432317F,    0.0878518373F,   -0.0878766477F,   0.121618621F,
      -0.0346864052F,   -0.0981215686F,  -0.0111307921F,   -0.106098011F,
      0.103584222F,     -0.0439247936F,  -0.109930798F,    0.0118026668F,
      0.0447333679F,    -0.10973911F,    0.0510925278F,    -0.0398361348F,
      -0.0224519353F,   0.108102299F,    -0.00604558457F,  0.00747895148F,
      0.104561746F,     0.0971550569F,   0.142754212F,     0.0530920178F,
      -0.0173570253F,   0.0266934056F,   -0.203107029F,    0.226518705F,
      -0.179894537F,    -0.0519438386F,  0.137932181F,     0.097638227F,
      -0.161361441F,    0.0662995577F,   0.0822863728F,    -0.151940927F,
      -0.0589791797F,   -0.0928007513F,  -0.158551112F,    0.0379078612F,
      0.126408383F,     -0.107955076F,   0.121236563F,     0.183203623F,
      -0.0740302131F,   0.0042138F,      0.194481641F,     -0.0187746938F,
      -0.138476685F,    0.107244633F,    0.10196045F,      -0.0185257513F,
      0.0323984809F,    0.137262374F,    -0.111414552F,    0.0310124699F,
      -0.0323127583F,   0.0215293374F,   0.0696165487F,    -0.0304311F,
      -0.0243982486F,   -0.12400879F,    0.184444636F,     0.0970616341F,
      -0.140969664F,    -0.0299541317F,  -0.0579158776F,   0.0518030599F,
      0.0325672105F,    0.250561982F,    -0.0400605947F,   0.221022442F,
      0.179456487F,     0.0679709613F,   -0.16312328F,     0.13616F,
      -0.123495258F,    -0.0873441473F,  0.0629063249F,    -0.0766656846F,
      -0.168842718F,    0.0494341776F,   0.132391393F,     -0.157781556F,
      0.02584216F,      -0.0782477185F,  -0.122267485F,    0.131968021F,
      0.160902292F,     -0.029661933F,   -0.047920119F,    0.168058559F,
      -0.143537939F,    -0.202189207F,   0.276860833F,     0.0347207971F,
      -0.0162084792F,   0.0438079052F,   0.147266865F,     -0.10888125F,
      0.0338139199F,    0.0967453122F,   -0.181794837F,    0.121139176F,
      -0.0960178673F,   0.161751509F,    0.0572712533F,    0.00389625202F,
      0.0938789845F,    -0.0800701454F,  0.112704746F,     0.174391344F,
      0.0368549675F,    -0.173565492F,   -0.00043579971F,  0.0175211877F,
      -0.0435417071F,   0.113367505F,    -0.0941847339F,   -0.0952987671F,
      -0.0321406163F,   -0.0637403578F,  -0.104632214F,    0.0561601594F,
      -0.00686646812F,  -0.023573393F,   0.0970390961F,    -0.0733381808F,
      -0.193612441F,    -0.0710483193F,  0.0321307331F,    0.083312F,
      -0.0546439514F,   -0.0814976469F,  -0.0817985088F,   0.0724127218F,
      0.0225226376F,    0.0556168705F,   0.00520866271F,   0.0132259419F,
      0.0620513037F,    -0.0969019756F,  0.162915066F,     0.00893423706F,
      -0.0827712342F,   0.00884783268F,  0.0323842F,       -0.103176072F,
      0.0588878579F,    -0.0681928396F,  -0.00599889504F,  -0.0445243753F,
      -0.116204366F,    0.187625229F,    0.204447076F,     -0.122358888F,
      0.102085084F,     0.18656604F,     0.190155402F,     0.0917660594F,
      -0.119646013F,    -0.16123727F,    -0.0762953758F,   0.0489869565F,
      -0.111945458F,    -0.15404281F,    0.00523549877F,   -0.114915922F,
      -0.121294446F,    0.00215620082F,  0.0679548755F,    0.0411770232F,
      -0.0846383497F,   0.0676733777F,   0.107179917F,     0.0271182954F,
      -0.0507650264F,   -0.0660212785F,  -0.0615289F,      -0.0284471624F,
      -0.137742132F,    0.00428405078F,  0.165881217F,     -0.0490620248F,
      0.0272178035F,    -0.0290231798F,  -0.0186630487F,   0.0360662192F,
      0.215879038F,     -0.0960613564F,  -0.0174842607F,   0.0609260201F,
      0.020236725F,     -0.120136626F,   0.0220399965F,    0.123694107F,
      -0.129962504F,    0.0380516052F,   0.0737900212F,    -0.152674377F,
      -0.0257875565F,   -0.0332883634F,  0.0724943578F,    0.0138981715F,
      0.221115798F,     -0.00470834924F, -0.0818667188F,   -0.14107646F,
      0.0523972549F,    0.110819504F,    -0.0522451028F,   0.000328303577F,
      -0.0296092983F,   -0.0136026638F,  0.133530542F,     -0.0607201271F,
      0.100662194F,     -0.0308033898F,  -0.0342277698F,   -0.0346563645F,
      -0.0763640925F,   0.062248081F,    0.133085504F,     -0.0563762076F,
      -0.00312237628F,  0.14305684F,     0.0048163943F,    0.0832555F,
      0.116102621F,     -0.0772183686F,  0.0125943273F,    0.124894992F,
      0.0908459798F,    -0.103037968F,   -0.0357851945F,   0.0922584906F,
      0.0405718759F,    -0.0708906054F,  -0.0111829452F,   0.0776848495F,
      -0.0599026158F,   -0.0409435593F,  0.0720101669F,    0.0792260841F,
      -0.0511585958F,   -0.0289885253F,  -0.0552396141F,   -0.0363817587F,
      0.0410114974F,    0.0793546066F,   0.109232821F,     -0.0277332384F,
      0.1463577F,       -0.0272043142F,  -0.0953431204F,   0.0729846805F,
      0.0393285826F,    0.106365412F,    -0.149989769F,    -0.0188019555F,
      -0.0128960907F,   -0.104618073F,   0.0605999529F,    0.0430552587F,
      0.0363249928F,    -0.0725640282F,  0.0746154264F,    0.02634307F,
      0.00248699822F,   -0.0638888925F,  -0.0138075743F,   -0.0151805226F,
      -0.00111883564F,  0.146512493F,    -0.176383764F,    0.0573944598F,
      0.0868963301F,    0.0225195382F,   0.0961937383F,    -0.0570153631F,
      0.0968396217F,    -0.0925019383F,  -0.102490604F,    -0.121878475F,
      0.0304629337F,    0.133798108F,    -0.0237626508F,   -0.0120203421F,
      0.071221292F,     0.0644475892F,   0.0770089775F,    -0.0139595494F,
      -0.0674776807F,   0.0354415514F,   -0.00898058247F,  -0.044854898F,
      0.114460461F,     -0.107019253F,   -0.0207279529F,   -0.0705625415F,
      -0.0871403068F,   -0.0755475461F,  0.0618624F,       0.104382135F,
      -0.0383516103F,   -0.103083439F,   0.0484307632F,    -0.0472482257F,
      -0.0145737119F,   0.0555339903F,   0.103042282F,     0.0192250945F,
      0.0622667894F,    -0.0753545F,     -0.0206408761F,   -0.0554631166F,
      0.0128124971F,    0.123202138F,    0.0856654942F,    0.0944188088F,
      -0.119755164F,    -0.0390774943F,  0.00524891494F,   -0.116119541F,
      0.0403495841F,    0.0406458639F,   -0.022061605F,    -0.115206331F,
      0.0552028678F,    -0.120678239F,   0.0458745137F,    -0.0969441384F,
      -0.0842169F,      0.0394390859F,   -0.113882199F,    0.00726704905F,
      -0.0817282796F,   -0.0235764962F,  0.063092947F,     -0.0330591947F,
      -0.0650793537F,   -0.0238775238F,  0.108825073F,     -0.0776996762F,
      -0.0940863714F,   -0.0788066834F,  0.0157221593F,    0.0700667202F,
      0.0357047208F,    0.105864726F,    -0.106370673F,    0.018182192F,
      0.0396883152F,    0.039346166F,    0.0593794882F,    -0.0556980968F,
      -0.0421141274F,   0.102539465F,    0.11009834F,      -0.125917226F,
      0.0109384665F,    0.088227421F,    -0.0851133466F,   0.00165386242F,
      0.0134477969F,    -0.0877454281F,  -0.108369909F,    -0.0688825622F,
      -0.103362814F,    -0.0107190758F,  0.00876168627F,   0.0769815296F,
      -0.0586467683F,   -0.110732168F,   -0.0610877126F,   0.0592901632F,
      -0.0126753272F,   -0.0250598658F,  -0.101177312F,    0.0574479736F,
      0.0753976405F,    -0.0457064211F,  -0.106681705F,    -0.0358553417F,
      -0.111306287F,    0.0333573F,      -0.0576679185F,   0.106857106F,
      -0.0338609815F,   -0.101332389F,   -0.0608081594F,   0.0639591515F,
      0.0457867905F,    0.0480018407F,   -0.0723929629F,   -0.00642649597F,
      0.0955790803F,    -0.068036966F,   0.0110953525F,    0.0523358807F,
      0.0541568473F,    -0.044779893F,   -0.0712603256F,   -0.0159847494F,
      -0.0407339372F,   0.112366311F,    -0.105494045F,    -0.0995425358F,
      0.119718201F,     -0.0661281571F,  -0.0623511F,      0.0925816894F,
      -0.0337190628F,   -0.0908884332F,  -0.0592285618F,   0.00723575428F,
      -0.028178351F,    0.107752673F,    0.0943493769F,    0.0549525954F,
      0.0804121569F,    0.0551709458F,   0.11314813F,      -0.0840561241F,
      -0.0790381208F,   -0.00487697637F, -0.0139350686F,   0.0101626366F,
      0.0249935966F,    -0.0403400958F,  -0.100482292F,    -0.0515779257F,
      -0.120760337F,    -0.0820322409F,  -0.0163641758F,   0.102065109F,
      0.0830638781F,    -0.0269285832F,  0.0344161242F,    0.025463501F,
      -0.05751881F,     -0.0779341608F,  0.0859111696F,    -0.0194167513F,
      -0.0301486067F,   -0.0662247166F,  0.0350306295F,    0.0391350798F,
      -0.030395288F,    -0.0386268795F,  -0.058968816F,    -0.101821601F,
      0.122079991F,     -0.101255633F,   -0.0886551365F,   -0.00606729F,
      -0.0883236825F,   0.131153792F,    0.0651120394F,    0.0637754F,
      0.0199674927F,    -0.0513812825F,  -0.00727686239F,  0.097540535F,
      -0.0301294196F,   -0.0449397042F,  0.112134367F,     -0.0914540291F,
      -0.00383761455F,  -0.0744904876F,  -0.0881832615F,   0.117392324F,
      0.106650278F,     0.104059301F,    -0.0615562387F,   -0.0438064672F,
      0.118067861F,     0.0115411077F,   -0.00630178535F,  -0.0261590835F,
      -0.0643822253F,   0.043482285F,    0.0284109768F,    0.0458244458F,
      0.11516162F,      0.0732845068F,   -0.115161151F,    -0.0417250879F,
      0.0448305644F,    -0.0623976029F,  -0.0809063464F,   0.0998646691F,
      0.0312360488F,    -0.041204147F,   0.104846269F,     0.0897412747F,
      -0.0997900888F,   0.111574672F,    -0.0695398748F,   0.0928735584F,
      0.0967640877F,    -0.0155718205F,  0.0887230858F,    0.1131429F,
      0.106201373F,     -0.102023013F,   0.021028595F,     -0.0201858897F,
      0.0187386256F,    0.0712722F,      -0.0344780646F,   -0.000818778644F,
      0.0477330536F,    0.130647257F,    -0.0433210284F,   -0.0512044542F,
      0.0351159126F,    0.108951F,       -0.026168583F,    0.0114599895F,
      0.105303019F,     -0.0806801468F,  0.014732223F,     -0.114130899F,
      -0.0371869355F,   0.00961696915F,  -0.12576884F,     -0.0631194487F,
      0.0109564578F,    -0.0892328843F,  -0.0413038582F,   -0.0373058356F,
      0.0235815849F,    -0.0157823525F,  0.0141443312F,    -0.0238726977F,
      -0.0297440365F,   0.0448963381F,   -0.0847835094F,   -0.0166929923F,
      0.114524238F,     0.0299766026F,   0.0762704089F,    -0.060910102F,
      -0.0798995F,      0.103075489F,    0.0387515388F,    0.0989370048F,
      0.0364001393F,    0.0731611103F,   -0.0869623646F,   0.165041268F,
      -0.00325577497F,  0.0145207988F,   -0.116252795F,    -0.0188065F,
      -0.073631227F,    0.0239111576F,   0.0425667837F,    0.120711885F,
      0.0166650862F,    -0.0314045623F,  0.0579998605F,    -0.0425200872F,
      0.0185561255F,    -0.00519108074F, 0.0166350082F,    0.130956218F,
      0.116727307F,     -0.0710345879F,  -0.16346173F,     -0.0869860649F,
      -0.0624128394F,   -0.0382978655F,  -0.127720386F,    0.0176182482F,
      -0.0147189F,      0.0662739053F,   0.0309332162F,    -0.0459782854F,
      -0.120556407F,    -0.0645975396F,  0.00398388738F,   -0.0739157274F,
      0.0210462157F,    0.00537637668F,  -0.0703983903F,   0.113062255F,
      0.0478877F,       0.128128F,       -0.0144487219F,   -0.0611594357F,
      0.0856869295F,    -0.0155499727F,  0.00678200461F,   0.10344746F,
      0.0314028151F,    0.0663498715F,   -0.0266291499F,   -0.108046234F,
      -0.0469368026F,   -0.0409734696F,  -0.0110278465F,   0.135102704F,
      -0.153930157F,    -0.00893764105F, 0.0118045555F,    -0.11343044F,
      -0.0645103082F,   -0.12490081F,    0.124827683F,     0.0303313658F,
      -0.0071668732F,   -0.0850701109F,  0.147771969F,     -0.0538062677F,
      0.105234452F,     -0.108401783F,   0.151450232F,     0.2681894F,
      0.113131151F,     -0.0190674141F,  0.258064777F,     -0.0974821672F,
      0.16461958F,      0.0940777F,      0.136555359F,     0.0450969636F,
      0.102711156F,     -0.102825984F,   0.124960333F,     -0.00244651781F,
      -0.0218362622F,   -0.138606563F,   0.235901326F,     -0.081221208F,
      0.119845502F,     -0.186907291F,   0.051319696F,     -0.114586808F,
      -0.0100197857F,   -0.0592670478F,  0.109123521F,     0.163274691F,
      -0.0796384588F,   0.133032814F,    0.00585956452F,   -0.0323927067F,
      0.147588134F,     0.10425666F,     -0.0197399054F,   0.0373161919F,
      -0.0458223112F,   -0.100193076F,   0.112514175F,     0.0431723967F,
      0.115962952F,     -0.0877743885F,  0.12977083F,      -0.0264170673F,
      0.109014355F,     -0.120919652F,   0.0764366314F,    -0.0102250827F,
      0.103271529F,     0.0375388935F,   -0.0197325405F,   -0.027260486F,
      -0.0361728519F,   -0.0728189126F,  0.208610445F,     -0.0900170282F,
      0.141036451F,     -0.0928899497F,  -0.0790400654F,   0.00209972169F,
      0.098315604F,     -0.0438503288F,  -0.103890203F,    0.0118993474F,
      0.0933673531F,    -0.0533060469F,  -0.0702734292F,   -0.00683695823F,
      -0.025653949F,    -0.0529850088F,  -0.203121588F,    -0.0166740064F,
      -0.0242116544F,   0.0616667F,      -0.105116732F,    -0.0731850192F,
      -0.0024544003F,   0.0819288269F,   0.0324130692F,    0.108614706F,
      0.125504598F,     0.101730891F,    -0.0532699116F,   0.00810293574F,
      -0.0275356639F,   -0.0310161654F,  0.094065994F,     -0.0181589331F,
      -0.0923000798F,   0.0430368483F,   -0.0193193052F,   0.00244638929F,
      -0.00629954878F,  0.000636562F,    -0.0994662046F,   -0.018975446F,
      -0.0234786645F,   -0.0497926846F,  -0.163062379F,    0.0426234901F,
      -0.00465394324F,  -0.123159245F,   -0.12403781F,     -0.0943653062F,
      -0.0248729698F,   -0.0978705436F,  -0.136549547F,    0.0328892618F,
      -0.152111545F,    0.0196886975F,   0.0873976424F,    -0.10899201F,
      0.0556235611F,    -0.119394228F,   -0.019197369F,    -0.0668154F,
      0.0997175053F,    -0.0702855065F,  -0.0621884093F,   -0.122864589F,
      0.113275677F,     -0.0471921228F,  -0.00742279273F,  -0.00148549792F,
      -0.144470528F,    -0.14631927F,    -0.0852646083F,   0.126649231F,
      -0.00860683899F,  0.0671400502F,   -0.00261499593F,  0.0595360734F,
      -0.0736107F,      0.0247058645F,   -0.109343462F,    0.094540745F,
      -0.0560207255F,   0.0565049089F,   0.089725256F,     0.0968003348F,
      -0.0695368F,      -0.130860344F,   0.115646899F,     -0.102698892F,
      0.0450040027F,    -0.0313154832F,  0.123587698F,     0.00251689157F,
      0.0724560693F,    0.0791303068F,   0.0836339593F,    -0.138039142F,
      0.0712992772F,    -0.119674198F,   0.0300774537F,    0.109868698F,
      -0.0849201381F,   -0.0712782666F,  -0.0299366415F,   -0.0846417695F,
      -0.0540907718F,   -0.00717794849F, 0.023887964F,     -0.106731117F,
      -0.0111045688F,   0.0500847287F,   -0.0972718075F,   -0.00527710235F,
      0.0550812595F,    0.113929339F,    -0.0414961129F,   -0.193623871F,
      -0.0206883941F,   0.0165434983F,   -0.0841346607F,   -0.00164260878F,
      -0.0835278F,      -0.00868891459F, -0.0686769F,      -0.044535391F,
      -0.0866762549F,   0.0976778567F,   0.0278753936F,    0.111803703F,
      -0.0711988434F,   0.0427723378F,   -0.0722054541F,   -0.00477459747F,
      -0.0823107585F,   0.0947512F,      -0.135269523F,    0.0382442847F,
      0.0360502116F,    -0.0799398422F,  0.107541136F,     -0.135662183F,
      0.0460811108F,    0.10206043F,     -0.0956784F,      -0.0224029906F,
      0.112328328F,     -0.031275887F,   -0.0665279925F,   0.0558646694F,
      -0.0599922612F,   0.0619224831F,   -0.0279828757F,   0.0210915525F,
      0.0309686866F,    0.0262257513F,   -0.0156990383F,   -0.0738939196F,
      0.158877343F,     0.106675461F,    0.070829764F,     -0.109136373F,
      0.0025575445F,    0.106730871F,    0.113571897F,     -0.0695177913F,
      0.0312143657F,    0.101393469F,    0.0438564755F,    -0.0382342599F,
      0.103590444F,     -0.13258934F,    -0.069208F,       0.00261325086F,
      -0.0252314638F,   -0.0800846592F,  0.0714223757F,    0.0819501802F,
      0.12012F,         0.0314964801F,   -0.0616344325F,   0.0273994692F,
      -0.0482851304F,   0.0298864935F,   0.0184369199F,    0.179607F,
      0.0802128091F,    0.0288523231F,   0.0865069404F,    -0.054395929F,
      0.150000378F,     -0.0100537268F,  0.0368696228F,    -0.0282761715F,
      -0.0862619206F,   -0.0290895477F,  0.141090691F,     0.0432184264F,
      0.052078817F,     -0.0450328775F,  -0.0294187609F,   0.12635535F,
      0.00918247F,      0.122768164F,    0.163076147F,     -0.0524961948F,
      0.0333527774F,    0.0807556957F,   0.113754913F,     0.0601112917F,
      0.0100325942F,    -0.0690639913F,  0.0807120502F,    -0.0619118139F,
      0.100539766F,     0.0295867845F,   0.141700059F,     -0.124796264F,
      -0.00429495331F,  -0.0506934449F,  0.20303002F,      -0.0402704552F,
      0.144613504F,     -0.071971342F,   0.0206889585F,    -0.124432094F,
      -0.00787310209F,  -0.169424191F,   -0.0642818F,      -0.0147486646F,
      -0.0183361508F,   0.00414591283F,  0.121397935F,     -0.132819846F,
      0.0647794083F,    -0.109732822F,   0.00185126264F,   0.0311605297F,
      0.058544755F,     0.0662727207F,   -0.00426039891F,  -0.125657529F,
      -0.11420542F,     0.0810292736F,   -0.081082F,       -0.0336894728F,
      0.0835883F,       0.16649428F,     0.136969507F,     0.16250214F,
      0.114347175F,     -0.0183593594F,  -0.0906428471F,   0.124291137F,
      0.0734577477F,    0.0748021454F,   0.0835593864F,    -0.00594365457F,
      -0.0190753732F,   -0.0837568343F,  0.0841327235F,    -0.0122270221F,
      0.0955938175F,    -0.0390304811F,  -0.0758769587F,   -0.114081778F,
      0.137921825F,     -0.161239058F,   0.133049086F,     -0.0284208041F,
      -0.126251936F,    -0.0577955954F,  -0.0986484587F,   -0.00302642304F,
      -0.154160246F,    -0.0354033783F,  -0.0591033585F,   0.179912969F,
      -0.0193831902F,   -0.0756905079F,  0.0914234444F,    -0.17628628F,
      0.0238025431F,    0.0621826947F,   0.0125661446F,    0.018837573F,
      -0.0211075842F,   -0.133894667F,   -0.0976096392F,   0.0019558738F,
      -0.134584606F,    0.00850220304F,  0.123571068F,     0.0929538831F,
      0.00219472451F,   0.0888895392F,   0.121458299F,     -0.0337725542F,
      -0.00486544939F,  -0.0906984061F,  -0.0955431312F,   0.158649728F,
      0.0551359616F,    0.00681940839F,  0.1520302F,       -0.0890778154F,
      -0.000699473778F, -0.104808949F,   0.0414218493F,    0.000842738431F,
      -0.0216759723F,   0.0851063207F,   -0.0334768966F,   0.136535347F,
      -0.107966311F,    0.0153261553F,   -0.0983975902F,   0.119866841F,
      0.0700432509F,    0.0875318274F,   0.0916510895F,    -0.0877003744F,
      0.114464454F,     -0.124235235F,   -0.0591351166F,   0.0622542351F,
      -0.0490206219F,   -0.151805684F,   0.127731204F,     -0.0437856317F,
      -0.00263517303F,  0.0569886081F,   0.130091608F,     0.133253217F,
      -0.01797566F,     0.049707789F,    0.0559217408F,    -0.00372290146F,
      0.0509539098F,    -0.0768981352F,  0.0742924958F,    -0.0838879F,
      0.0648989379F,    -0.0176422838F,  0.0503812954F,    0.0441680588F,
      -0.0715456903F,   -0.0438801534F,  0.0275473036F,    0.00440185145F,
      0.0507997572F,    -0.0312923305F,  -0.0262451489F,   -0.142015383F,
      0.02399051F,      -0.0639598891F,  -0.0380451381F,   -0.0984444618F,
      -0.0814079866F,   0.0878599F,      0.0435750671F,    -0.0320186652F,
      0.0376195684F,    -0.0725692883F,  -0.116991401F,    -0.0815623328F,
      0.00334290648F,   -0.0746029168F,  -0.0260207858F,   -0.0401229113F,
      -0.0930748209F,   0.0308074802F,   0.0440655164F,    -0.0527118258F,
      0.0827302411F,    0.0403873213F,   -0.000903715088F, -0.0921773911F,
      -0.0675640106F,   -0.0262335707F,  -0.11307466F,     -0.0192720219F,
      0.0596616901F,    -0.0553196855F,  0.0401913598F,    0.0501818F,
      -0.00828384236F,  0.0802019462F,   -0.0169405583F,   -0.0240686F,
      -0.0750312F,      0.08391352F,     -0.101647504F,    0.0482268035F,
      -0.00315357279F,  -0.00375159F,    -0.0135233533F,   0.0843806341F,
      -0.00805241428F,  0.155969307F,    -0.0188326351F,   -0.0775790289F,
      -0.0555552952F,   0.0611915253F,   0.143729985F,     -0.015061222F,
      0.126135051F,     0.0469893888F,   -0.0595657565F,   0.0564361364F,
      0.052264113F,     0.0306319296F,   0.0846394449F,    -0.0630691871F,
      0.100247853F,     0.0125314314F,   0.0193006527F,    0.0786077F,
      0.0253422931F,    -0.0445925668F,  -0.0827589855F,   0.130833864F,
      0.0308458321F,    0.00132431556F,  0.0854106322F,    -0.0427403264F,
      -0.0839337856F,   -0.0773126483F,  -0.10533426F,     0.0313895456F,
      -0.0720493495F,   -0.0555625744F,  -0.00504273875F,  0.0314474888F,
      -0.106613174F,    -0.0709214732F,  -0.0388646F,      -0.00213466445F,
      0.00434572F,      0.075288713F,    0.0268781446F,    -0.022392517F,
      -0.0553377382F,   0.0244905353F,   0.00568044651F,   -0.0605249219F,
      -0.0734948218F,   0.119381927F,    -0.107730322F,    0.115511276F,
      0.0953321457F,    -0.0158713646F,  -0.0215730537F,   0.0537248477F,
      -0.0102402288F,   0.0487216823F,   0.111096278F,     -0.0936533213F,
      0.0869167745F,    0.0352064595F,   0.114980526F,     0.0879889056F,
      0.107928284F,     -0.111213043F,   0.0235561F,       0.0915179178F,
      0.0852934867F,    -0.042834565F,   -0.101913616F,    0.0476881824F,
      -0.0558006912F,   -0.003353548F,   -0.0956965685F,   0.07988628F,
      -0.0365650691F,   -0.0188444238F,  0.0694816485F,    -0.0217161346F,
      -0.0236613359F,   -0.0835711658F,  -0.0862396434F,   -0.0485274121F,
      -0.0708824322F,   0.0781134218F,   -0.124775231F,    0.0859199911F,
      0.0594131686F,    -0.086966455F,   -0.0204266738F,   -0.0107208677F,
      0.0290290229F,    0.00701537495F,  0.0933095515F,    -0.106732555F,
      -0.014167116F,    -0.101173006F,   -0.0915323123F,   0.0103673125F,
      0.0326875411F,    0.0343328565F,   0.12238849F,      0.108259298F,
      0.00628737547F,   0.00566375814F,  -0.106902175F,    -0.0109628718F,
      -0.0966349542F,   0.114469551F,    -0.0627401695F,   0.0848171189F,
      0.0573852547F,    0.0982954502F,   -0.0548049621F,   -0.0688637793F,
      0.01394636F,      -0.0880086273F,  0.00751511753F,   -0.0342646129F,
      -0.0669059679F,   -0.0669577271F,  -0.0849948227F,   -0.0572054908F,
      0.0950353742F,    -0.136588156F,   0.103049286F,     0.0360649824F,
      -0.00756776333F,  -0.0616691411F,  0.0108383931F,    0.0238671619F,
      -0.0111020347F,   0.00904453639F,  -0.0695429F,      -0.0632599369F,
      -0.02248214F,     0.123432554F,    -0.0545986295F,   0.0964976773F,
      0.0914442241F,    -0.0731085613F,  0.077335842F,     0.0817274749F,
      -0.0773615167F,   0.034975756F,    0.0425286815F,    -0.118476488F,
      -0.0453733951F,   0.0959321856F,   -0.0318931937F,   0.0344071873F,
      0.0163838528F,    0.0461555235F,   0.00100868975F,   0.000847434509F,
      0.115774862F,     -0.0606666021F,  0.0516729504F,    -0.018004559F,
      -0.176512644F,    -0.0481988862F,  -0.00547755649F,  0.100575007F,
      0.0801790133F,    0.0191542394F,   -0.0614641868F,   -0.00513029192F,
      0.0101493923F,    -0.0427006185F,  -0.0141546447F,   0.0982723683F,
      0.00134457904F,   -0.0919115469F,  -0.0297919791F,   0.131604359F,
      -0.0485128909F,   -0.124607034F,   0.0608307607F,    0.137871847F,
      0.148896366F,     -0.0612648F,     -0.0748414546F,   0.0648687333F,
      0.0699508637F,    0.103911936F,    -0.0913354382F,   0.113987461F,
      0.0403015576F,    0.00331030646F,  -0.0581278875F,   -0.0449036844F,
      -0.0936289504F,   -0.0875062F,     -0.0755321383F,   0.121593475F,
      -0.0763959959F,   -0.0572153516F,  -0.15593569F,     -0.0970635414F,
      0.0800267309F,    0.166060612F,    -0.00340077025F,  -0.0814570636F,
      0.141859189F,     -0.0998609737F,  0.0732995346F,    0.0863599777F,
      -0.0738498F,      -0.0780406594F,  -0.103468359F,    0.0689995363F,
      0.0275188796F,    0.00989835151F,  -0.073752068F,    0.0721311644F,
      0.110453442F,     0.151495159F,    -0.100111276F,    0.0779000446F,
      0.16599372F,      -0.0900583F,     -0.105818383F,    -0.00913492F,
      -0.109448053F,    -0.1330937F,     0.034189418F,     0.0595706329F,
      -0.0534196123F,   0.0465445817F,   -0.162122533F,    -0.0761385486F,
      -0.0390820242F,   0.104276076F,    0.00384151726F,   0.176550493F,
      -0.0121691162F,   -0.112278409F,   0.0265315566F,    0.13280037F,
      0.0615822412F,    0.114732884F,    -0.104565479F,    0.13778916F,
      0.10932973F,      0.13423939F,     -0.026114786F,    0.0785948038F,
      -0.0294038933F,   0.114117727F,    -0.125510335F,    0.193598807F,
      0.0180232804F,    0.160602018F,    -0.154887393F,    -0.0738506839F,
      -0.0130926222F,   0.16140376F,     0.0732422F,       -0.0254057478F,
      0.0378117263F,    -0.0455447435F,  0.0357968733F,    -0.0815395117F,
      0.0976244062F,    0.0160523299F,   0.102247514F,     -0.0412316099F,
      0.0293804333F,    -0.00869222824F, -0.0926412344F,   0.0533182F,
      -0.159457564F,    -0.164732173F,   0.165000573F,     0.050859265F,
      -0.00441703247F,  0.0723470673F,   0.00590139534F,   0.0251664F,
      -0.0868539959F,   -0.021820087F,   0.10234037F,      0.113613188F,
      0.114068165F,     -0.0877878442F,  0.104379363F,     -0.0912488103F,
      -0.0966052189F,   -0.172306731F,   0.002149F,        0.0256181397F,
      0.0708443522F,    0.169378355F,    0.111494154F,     0.104080357F,
      0.135358304F,     0.101908587F,    0.0691802502F,    0.0300577804F,
      -0.0640371665F,   -0.00995336566F, 0.0475497395F,    -0.0439762287F,
      -0.054985635F,    -0.00389261846F, 0.0433073156F,    0.0345471203F,
      0.0458712652F,    -0.0127803935F,  -0.109485425F,    -0.0579422452F,
      -0.0117840674F,   0.0827315F,      -0.0791767761F,   0.0606362075F,
      -0.0650191307F,   -0.172062784F,   -0.0513154827F,   0.0836684927F,
      -0.125074461F,    -0.08857207F,    -0.0827714056F,   0.0447014533F,
      -0.143286422F,    0.0258766524F,   -0.162174806F,    -0.158160061F,
      -0.0935328677F,   -0.046097558F,   -0.071703881F,    0.0968864188F,
      0.0532537363F,    -0.0737080425F,  -0.155836686F,    0.118692331F,
      -0.052608F,       -0.138282642F,   0.13860032F,      -0.0556646436F,
      0.0244804267F,    0.116919473F,    0.132333681F,     -0.081585817F,
      0.104899585F,     -0.0765026286F,  -0.149017617F,    0.0493630469F,
      0.0324992947F,    0.170089617F,    0.119994424F,     0.00498929806F,
      0.0180193726F,    0.0615074597F,   0.0531931408F,    0.06703154F,
      0.119397759F,     0.0601101071F,   -0.0591332503F,   -0.120334111F,
      -0.0567704402F,   0.0346121714F,   -0.0513730235F,   -0.116638348F,
      0.0454719F,       0.0831849054F,   -0.114132859F,    -0.0771078095F,
      0.10261061F,      0.12811245F,     -0.0165809356F,   0.200000361F,
      -0.110005543F,    0.0856442228F,   0.106283598F,     -0.0762258545F,
      -0.00023815826F,  0.0624846146F,   -0.0835056454F,   0.0449359417F,
      0.0076907808F,    0.0234768726F,   0.0198621489F,    -0.0896643F,
      0.0196901243F,    0.0554467961F,   -0.0711219832F,   -0.0138401492F,
      0.10672307F,      0.0585503876F,   0.0409761108F,    -0.083352007F,
      0.0294481888F,    -0.0530479588F,  0.128043368F,     -0.0753614679F,
      -0.0542001091F,   -0.00440202281F, 0.159548253F,     0.0175582878F,
      0.0390371196F,    0.132833883F,    -0.00834911224F,  -0.0022779163F,
      0.0869696587F,    0.0326505154F,   0.0423924327F,    0.0436839722F,
      -0.0209709406F,   -0.0183671862F,  0.0251038875F,    -0.0289066378F,
      0.110944F,        0.00372492941F,  -0.0132763535F,   0.0162746869F,
      0.0842201188F,    -0.0686780065F,  0.0895641223F,    0.0350463241F,
      0.0246331524F,    -0.0955447778F,  -0.113719523F,    0.0472872742F,
      0.0671844482F,    -0.0614211671F,  -0.00716620823F,  0.0361375958F,
      -0.0259409323F,   0.0569657907F,   0.190344885F,     -0.0967250317F,
      0.0496354029F,    0.0852049738F,   -0.00781868678F,  -0.0631352141F,
      0.0188524686F,    -0.0692496449F,  0.0538438559F,    -0.0193413477F,
      0.0434303172F,    0.0495211445F,   -0.111550868F,    -0.00432114908F,
      -0.103358656F,    -0.0799788386F,  -0.142563641F,    -0.0282920077F,
      0.0242738463F,    0.13223353F,     0.080309622F,     -0.0132456794F,
      -0.0177652538F,   -0.0381357409F,  -0.0314550959F,   0.115738273F,
      0.0264816582F,    -0.0845652372F,  -0.0398454145F,   0.0329742432F,
      0.0706814155F,    -0.0163197722F,  -0.0594103672F,   0.101548068F,
      0.0319118686F,    0.0946196392F,   -0.0906107128F,   -0.0792775154F,
      -0.128216028F,    -0.024960082F,   -0.037765976F,    0.123803407F,
      0.101071931F,     -0.139040068F,   0.0380878672F,    -0.0365570039F,
      -0.0128859896F,   -0.0637539402F,  -0.0269444752F,   0.106781788F,
      0.106631078F,     0.0951402932F,   -0.0019113298F,   -0.101335235F,
      0.0882545784F,    0.0507815406F,   -0.00437912205F,  0.0124572804F,
      -0.106681958F,    0.107720785F,    -0.0737811923F,   0.107149251F,
      0.0124142719F,    -0.0185310412F,  -0.109521084F,    -0.128488287F,
      -0.0941758603F,   0.0101188589F,   -0.0453622378F,   0.053437341F,
      -0.0335134827F,   0.044092685F,    -0.0578330532F,   0.0235873852F,
      0.0358886346F,    0.110468447F,    -0.0456454866F,   -0.000522544084F,
      0.128959894F,     -0.0595111288F,  -0.0872144476F,   0.0630345792F,
      0.0230786651F,    -0.0558691882F,  -0.0601625517F,   -0.0582692921F,
      -0.0729978383F,   0.0326530077F,   0.0420315377F,    -0.0929755047F,
      -0.069236517F,    0.0401308276F,   -0.0714333877F,   -0.0272233337F,
      0.0145618459F,    -0.0687855259F,  0.0672472566F,    -0.0453355648F,
      0.137456104F,     -0.0716708601F,  0.105464175F,     -0.126508072F,
      0.0248319618F,    0.174581438F,    -0.0118144043F,   0.0759713128F,
      0.0452234447F,    0.0438035727F,   0.134721354F,     0.148490205F,
      -0.0763413757F,   0.0482674837F,   0.128250629F,     0.0429755971F,
      0.00671777874F,   0.129737705F,    0.0141822947F,    0.0637562275F,
      0.0470634662F,    0.0996716544F,   0.0445964038F,    -0.0798853114F,
      0.143368661F,     -0.0581138358F,  -0.118316211F,    -0.0996742174F,
      0.08228039F,      0.00626675459F,  -0.0310790632F,   0.00887196232F,
      -0.00886390358F,  0.108571969F,    -0.139190316F,    0.00841185637F,
      0.11724288F,      -0.110560782F,   0.0609868728F,    0.00462500378F,
      0.0329060405F,    0.0182554685F,   -0.125177845F,    0.060449F,
      -0.0852144733F,   -0.0299384873F,  0.0322540626F,    -0.0685975701F,
      -0.0389794223F,   -0.0538827F,     -0.0320139416F,   0.0441462621F,
      -0.0210862979F,   0.0781030804F,   -0.0488828346F,   -0.0242235493F,
      -0.00689750491F,  -0.0784721375F,  -0.0111264158F,   0.0705476329F,
      0.103845626F,     -0.0434579477F,  -0.044768218F,    -0.118176527F,
      -0.026280323F,    -0.00931648724F, -0.103296123F,    -0.0205175169F,
      -0.00475431327F,  -0.049784217F,   0.0697259754F,    -0.0607960522F,
      0.0104668466F,    -0.0761206672F,  -0.104723074F,    -0.00526869763F,
      0.105835013F,     -0.0192658715F,  -0.00443140278F,  0.0294918567F,
      0.0265736971F,    -0.0530766845F,  0.0262859929F,    0.0334760584F,
      -0.0287544653F,   -0.0173434541F,  -0.0776386857F,   -0.069894135F,
      -0.0458855629F,   -0.0330552869F,  -0.0900309086F,   -0.0351275466F,
      -0.103831686F,    -0.00464266352F, -0.0600309372F,   -0.037760593F,
      -0.1171652F,      -0.0509863794F,  -0.0914578736F,   0.0597760677F,
      0.0563194938F,    -0.122502245F,   -0.0680221915F,   0.128605157F,
      -0.0657229349F,   -0.106669925F,   0.0542816184F,    -0.132814795F,
      -0.00246456941F,  0.109976098F,    -0.0435615517F,   -0.0350053422F,
      -0.0531230755F,   0.111811586F,    -0.0229589194F,   -0.0578128099F,
      -0.0438392125F,   -0.116815418F,   0.0492961369F,    0.028676929F,
      -0.0801682919F,   0.0149185443F,   0.0339042F,       0.0244737621F,
      -0.0858272165F,   0.0654196218F,   -0.0821003243F,   0.0169184431F,
      -0.0587361641F,   -0.115438618F,   0.0546593331F,    0.0040326044F,
      -0.0964446887F,   -0.018877849F,   -0.0648608655F,   -0.0096555762F,
      0.0543777123F,    0.0656093433F,   -0.0106023513F,   -0.0161797609F,
      -0.0395791195F,   -0.0949529633F,  -0.0733826086F,   -0.1169025F,
      -0.0887498185F,   0.0340980105F,   -0.0459129F,      -0.0344293118F,
      0.0250443537F,    0.0524165332F,   0.0120622963F,    0.094560042F,
      -0.0347252525F,   -0.10701403F,    -0.138792098F,    0.0503813513F,
      -0.0849236324F,   0.0525557138F,   0.0199352764F,    0.00695417123F,
      -0.08108107F,     0.0359454788F,   0.137052491F,     0.0850886703F,
      -0.0349369124F,   -0.0323692672F,  -0.0865510926F,   -0.0868369192F,
      0.0181648526F,    -0.020216221F,   0.00156148034F,   -0.0225790832F,
      0.071202375F,     -0.151242971F,   -0.0184075702F,   0.155499592F,
      0.025847286F,     -0.155191809F,   0.136164472F,     -0.119319566F,
      0.0523664951F,    0.0536212362F,   0.0553029552F,    -0.039122276F,
      -0.097759977F,    0.109930329F,    -0.0607367121F,   0.0866675526F,
      -0.0112168295F,   0.0789532065F,   -0.0400106311F,   0.0626526847F,
      -0.0375062339F,   0.180237323F,    -0.0620308F,      0.040903084F,
      0.0224662609F,    0.118906967F,    0.101010755F,     -0.123015031F,
      -0.00382842915F,  -0.0670679957F,  -0.06305217F,     -0.0774294883F,
      -0.138444245F,    0.0284486767F,   -0.0471832789F,   -0.00244962564F,
      -0.126044407F,    0.0834207386F,   -0.0074815047F,   0.154651418F,
      -0.107720271F,    0.0755272657F,   -0.0217135344F,   0.0920345F,
      0.00300064357F,   -0.0703554377F,  -0.0447529815F,   0.0413446613F,
      0.0189152509F,    0.082176663F,    0.00187248F,      -0.0592491105F,
      -0.0993365F,      0.10089995F,     0.160842389F,     0.0711424425F,
      -0.144472793F,    -0.0840277225F,  -0.0654070899F,   -0.0741417706F,
      0.149314493F,     0.145014137F,    -0.0995173231F,   0.0799448714F,
      0.0191677697F,    0.0999416336F,   0.115130328F,     0.0617128499F,
      0.126327574F,     -0.00637988886F, 0.107326545F,     0.157488331F,
      -0.150313F,       0.0725682676F,   0.157384947F,     0.0129952729F,
      -0.0206007101F,   -0.0174028371F,  -0.130477592F,    0.103684194F,
      -0.00621571392F,  -0.0436527543F,  0.0506577641F,    -0.105701812F,
      0.0464914553F,    0.0782856494F,   0.0685774088F,    0.111282647F,
      -0.0249951705F,   0.0315861739F,   -0.0154222213F,   -0.00590180745F,
      -0.103660665F,    0.118133709F,    -0.183141798F,    0.13948822F,
      -0.10049089F,     -0.053980764F,   0.0869267136F,    -0.0182872843F,
      0.00837585516F,   -0.0759938806F,  0.203757107F,     0.105059534F,
      0.113523155F,     -0.00083369296F, -0.0892061666F,   -0.131429136F,
      0.118339717F,     0.140913665F,    -0.116080977F,    0.0919679254F,
      0.0569369607F,    0.0102646099F,   -0.0558749847F,   -0.0728749931F,
      0.0905062407F,    -0.0845647454F,  -0.0182064176F,   0.0785243288F,
      0.00726223877F,   -0.1088899F,     -0.0146746347F,   -0.021551298F,
      0.000109097578F,  -0.0861631557F,  0.128948346F,     -0.00362208486F,
      0.0951988399F,    0.059727855F,    0.0154951233F,    0.0856399164F,
      -0.0939406231F,   -0.175003558F,   0.131092101F,     -0.107991084F,
      -0.014190793F,    0.0327386558F,   -0.119307466F,    0.11923562F,
      -0.10873758F,     0.0180768743F,   0.0679347068F,    -0.026849566F,
      -0.0575049482F,   0.0348722972F,   -0.130768389F,    -0.0731997341F,
      0.00749131199F,   -0.041161824F,   0.118056118F,     -0.073797293F,
      0.114342012F,     0.140360773F,    -0.0187479742F,   -0.0243167412F,
      -0.0573564731F,   0.109564029F,    0.0593455695F,    0.0717166364F,
      -0.000443932222F, 0.0419195816F,   -0.0501092337F,   0.0759600475F,
      -0.112920366F,    -0.0162726827F,  -0.0552961417F,   0.155486509F,
      0.166714728F,     -0.0665454343F,  -0.0357108042F,   0.0489307456F,
      -0.0360894054F,   -0.0312046632F,  -0.0092698F,      -0.0844919309F,
      -0.0236976184F,   0.130601048F,    0.0428996198F,    -0.0875291303F,
      0.036587473F,     -0.0734876469F,  -0.00744811306F,  0.14281261F,
      0.128001362F,     -0.0590803884F,  -0.0249587242F,   -0.0348400921F,
      -0.0838277414F,   -0.043767564F,   0.04714505F,      -0.111796021F,
      0.0928905234F,    0.0282882266F,   -0.0176239945F,   -0.0479720049F,
      0.0866786391F,    -0.165900186F,   0.00592145F,      -0.119190834F,
      -0.0273868032F,   -0.104662843F,   -0.0186753068F,   0.103998728F,
      -0.12203601F,     -0.0883009285F,  0.0627429858F,    -0.0249130763F,
      -0.0617971197F,   0.00170067081F,  -0.0496348888F,   -0.0392684937F,
      -0.101059921F,    0.057792794F,    -0.0567458048F,   -0.00749288313F,
      0.133844957F,     0.150869131F,    0.0101783145F,    -0.0787561461F,
      0.0415802747F,    0.116338409F,    0.115809157F,     0.141074032F,
      0.0960498229F,    -0.0126874382F,  -0.00993508752F,  0.151197881F,
      -0.0326154679F,   0.0817567259F,   0.118432216F,     0.00981208868F,
      0.0589010157F,    0.120352045F,    0.0236965697F,    0.0351272412F,
      -0.120594226F,    -0.0381169021F,  -0.082398504F,    -0.0186265744F,
      0.141848207F,     -0.0813354701F,  -0.0986143351F,   0.137670323F,
      -0.0872572809F,   -0.0070252F,     0.0596870631F,    -0.0101672625F,
      -0.0807816163F,   -0.0681467205F,  0.0621439815F,    0.0420495234F,
      0.0674719661F,    -0.0956378281F,  -0.08851403F,     -0.0742013082F,
      -0.0769965276F,   -0.122906446F,   -0.0567914508F,   0.0739358291F,
      -0.0149958264F,   0.0564342849F,   -0.0038920329F,   0.0087009063F,
      0.0161854364F,    0.0858812258F,   0.166567728F,     -0.146816701F,
      -0.0877391472F,   0.0622761F,      0.0275613908F,    -0.0922044F,
      0.0511245839F,    0.0847700238F,   -0.0582173F,      0.0282322746F,
      0.132814288F,     0.042829372F,    0.183985248F,     0.115276299F,
      0.142495215F,     0.127311751F,    -0.079968296F,    -0.106775433F,
      0.0579806976F,    0.00222070026F,  -0.0429664627F,   0.0802956F,
      0.0370359942F,    -0.0195328817F,  0.011174242F,     0.0330888741F,
      0.0902574435F,    0.0998691842F,   0.00462475326F,   0.0156780891F,
      -0.0241670683F,   -0.114782408F,   -0.0679823086F,   -0.0137545504F,
      -0.105769858F,    0.113501117F,    0.0622827262F,    -0.0204307642F,
      0.0708007663F,    -0.11262887F,    -0.183083832F,    0.10554719F,
      -0.0684563294F,   0.0676344112F,   -0.0930714458F,   0.111093789F,
      0.0231218487F,    0.143427F,       0.203232303F,     0.104233623F,
      -0.127084881F,    -0.140882537F,   0.063679561F,     0.122604117F,
      -0.0131240292F,   0.0195437446F,   0.172207639F,     0.000632205396F,
      0.0706368461F,    -0.0788342878F,  0.104403414F,     -0.0270232949F,
      0.041063413F,     -0.0120659722F,  0.0195437018F,    -0.113372505F,
      0.0879280791F,    0.07202176F,     -0.0282367878F,   0.118129544F,
      -0.021659907F,    -0.104320437F,   0.0746807754F,    -0.0117736F,
      -0.115599953F,    -0.0214840136F,  -0.133243665F,    -0.0672882795F,
      0.000492447754F,  -0.0966766849F,  0.0189000107F,    -0.130062819F,
      -0.0968476385F,   -0.126059398F,   -0.0649439767F,   -0.0307122655F,
      0.114316054F,     -0.0907926F,     -0.133529022F,    -0.0979988873F,
      -0.00171150197F,  0.013246946F,    -0.0660339892F,   -0.0196352713F,
      0.106150232F,     0.131953672F,    0.0682211146F,    0.0114280228F,
      0.00811086874F,   -0.0508587919F,  -0.0948280171F,   0.0419201925F,
      0.0981102884F,    0.0563616641F,   0.113315083F,     0.0988195F,
      0.0178663377F,    0.158212751F,    0.0362154581F,    0.167342991F,
      0.0622220933F,    0.0280166958F,   -0.0752200857F,   -0.112748846F,
      0.00534542976F,   -0.126405403F,   0.0281058121F,    -0.0581773669F,
      0.0966817588F,    -0.00108381035F, 0.0847350657F,    -0.0177989434F,
      -0.110510722F,    -0.126365557F,   0.0397015139F,    0.0248738136F,
      -0.147357717F,    -0.0796263069F,  0.0163662657F,    -0.0556412637F,
      -0.0179671757F,   -0.00353125436F, 0.0208466854F,    0.0373011678F,
      -0.084833093F,    0.0337567553F,   -0.140329897F,    -0.106694393F,
      -0.126504719F,    0.0759764239F,   0.0151013313F,    -0.0724162832F,
      0.112534739F,     0.0689143389F,   0.0880299062F,    0.0594979562F,
      0.0852076113F,    -0.0104321148F,  -0.0316760391F,   -0.0578152202F,
      -0.0914410874F,   0.0158288684F,   0.105966F,        0.0349809863F,
      -0.0752778277F,   -0.0904471576F,  -0.0652466044F,   -0.0782317F,
      -0.0538066253F,   -0.0482211336F,  0.0688735917F,    -0.0353830531F,
      -0.0452110432F,   0.029005371F,    0.0731091797F,    -0.120952621F,
      -0.0329261944F,   -0.0466833077F,  -0.0975114182F,   0.0144063681F,
      0.0249742493F,    0.117750615F,    -0.0829467252F,   0.0778534636F,
      -0.0100295357F,   -0.0611724F,     -0.0964915827F,   -0.0709843189F,
      0.0195544884F,    -0.0938012153F,  0.124803558F,     -0.0737758875F,
      -0.0129468851F,   -0.0199716017F,  -0.0713288411F,   0.00222364068F,
      0.137061551F,     -0.11609979F,    -0.0193354227F,   -0.0450084396F,
      -0.104092009F,    -0.0471058749F,  -0.0672728866F,   0.046990186F,
      -0.107037008F,    -0.0481147803F,  -0.0942718238F,   0.0361928828F,
      0.124871828F,     -0.0143296011F,  0.0205207039F,    -0.0817578062F,
      0.0988467F,       0.0114278216F,   -0.132196292F,    -0.00463847909F,
      0.108340144F,     -0.0956014246F,  -0.0377622172F,   0.00653850567F,
      0.00967657194F,   -0.0911514834F,  0.00598323951F,   0.0471551679F,
      0.0717258528F,    0.0209768F,      0.0267786384F,    0.111660726F,
      0.0377657749F,    0.106269158F,    -0.149135798F,    0.0272833575F,
      0.0319465362F,    -0.0667056367F,  0.00761471037F,   0.124324955F,
      0.0553230532F,    0.0308600329F,   0.0480099F,       -0.00489748688F,
      -0.0774324462F,   0.0328291841F,   -0.0236966815F,   0.169415727F,
      0.0258440562F,    0.0834100321F,   0.0770990402F,    0.0647766739F,
      -0.0850091055F,   -0.0869647F,     0.00482839951F,   -0.0316032171F,
      -0.0787034556F,   -0.0978983641F,  0.0865182802F,    -0.0631117225F,
      -0.0809138566F,   -0.00227381079F, 0.0481189899F,    0.0212652273F,
      0.0709843263F,    -0.118960783F,   -0.138654634F,    0.0368162431F,
      -0.0480461F,      0.122793622F,    -0.00879946724F,  0.0259197373F,
      -0.133474573F,    0.0963581279F,   -0.0106465248F,   0.0013354097F,
      0.0616342202F,    0.0652737692F,   -0.0575072505F,   -0.00400816975F,
      -0.00837730058F,  0.0435323715F,   -0.0660168231F,   0.0402242653F,
      -0.0846885517F,   0.123528197F,    0.082722269F,     0.0941726416F,
      0.0308986716F,    0.020017907F,    -0.0509570539F,   -0.104785547F,
      0.0393586084F,    -0.006015555F,   -0.107002489F,    -0.0104677537F,
      0.0498147309F,    0.0692570657F,   0.0664607361F,    0.0882041231F,
      0.0387456603F,    -0.0792811438F,  0.0558785722F,    0.0804138333F,
      -0.0661780238F,   0.0639173314F,   0.107992619F,     -0.104910135F,
      0.132954583F,     -0.0720346421F,  -0.0807651579F,   0.114815868F,
      0.0219192F,       -0.00885720737F, 0.128915891F,     -0.0118922107F,
      -0.103053823F,    0.0273007303F,   0.127432033F,     0.0370567F,
      -0.144753456F,    0.0614874251F,   0.0925552398F,    -0.0745080262F,
      -0.0868880451F,   -0.00478479825F, 0.136620671F,     0.125409782F,
      -0.0102370111F,   0.00282766647F,  0.063987039F,     -0.0364828818F,
      0.160963282F,     -0.0333306082F,  0.026446877F,     -0.0920093283F,
      -0.0902559757F,   0.114146173F,    -0.0449262932F,   0.0722790211F,
      0.158315778F,     -0.106286719F,   -0.188557968F,    0.160629734F,
      -0.18511337F,     -0.0390378274F,  0.0265108198F,    -0.0691666603F,
      0.0290420316F,    0.112661295F,    -0.0701237619F,   0.0138680348F,
      0.0543335229F,    -0.0435464568F,  0.0890717879F,    -0.156897441F,
      0.126433611F,     0.0610050596F,   -0.0471833795F,   0.179047301F,
      -0.0638317168F,   -0.00464456948F, 0.0745158494F,    0.026603397F,
      -0.0440404415F,   -0.066110149F,   -0.028338965F,    -0.0356130265F,
      -0.0193261076F,   -0.0496600196F,  0.0548178665F,    0.0411072187F,
      0.128709987F,     -0.183659256F,   0.0103095677F,    0.186077088F,
      -0.0803180262F,   -0.0769929066F,  0.0569142178F,    0.061119087F,
      -0.116031937F,    0.121635541F,    -0.204128712F,    -0.0170931946F,
      -0.043316137F,    0.129800379F,    0.116323166F,     0.193539083F,
      0.254155517F,     -0.0323084183F,  -0.0964536443F,   0.0591772832F,
      -0.2071459F,      -0.0378803946F,  -0.184064925F,    0.0875194296F,
      0.0385542549F,    -0.0633010566F,  0.119663902F,     0.00940695591F,
      -0.11293637F,     0.0501354262F,   -0.0888303369F,   -0.0537369177F,
      0.0120431781F,    -0.0443161242F,  -0.113361701F,    0.206982821F,
      0.0229289625F,    -0.0476866215F,  0.153596848F,     0.0848742872F,
      0.0470501184F,    7.75573135E-5F,  0.000639077334F,  -0.112106457F,
      0.0524362102F,    -0.00416567642F, 0.0499627627F,    0.148218706F,
      0.0692253783F,    -0.0292289462F,  -0.206791177F,    0.136504546F,
      -0.114407897F,    -0.0852755085F,  0.0858490467F,    0.0940989256F,
      0.00667955074F,   0.017745303F,    -0.202016637F,    -0.0547709242F,
      -0.0817761794F,   -0.0373731889F,  0.0920580477F,    0.112398505F,
      0.166918188F,     -0.0139172086F,  -0.0931431651F,   -0.124667332F,
      -0.0706970766F,   0.138305008F,    -0.219215825F,    0.100995444F,
      -0.0447948165F,   0.0758460164F,   0.116709083F,     0.115129232F,
      0.177605435F,     -0.0854813829F,  0.0362114869F,    -0.0254708957F,
      0.021638684F,     -0.091111213F,   0.274633229F,     -0.161839291F,
      0.178031549F,     0.0756439045F,   0.0524539724F,    -0.0892186165F,
      0.0378458463F,    -0.0876994F,     -0.0878884569F,   0.11131835F,
      0.0196513887F,    0.0146532841F,   0.00357508F,      0.0968838856F,
      -0.0844877809F,   0.0295026265F,   -0.105706856F,    0.0338697769F,
      -0.0516887419F,   0.145263135F,    -0.01164209F,     -0.0296936594F,
      0.0836978257F,    -0.0228714198F,  -0.034476839F,    0.0988607779F,
      0.0310751144F,    -0.00476427656F, -0.0269233286F,   0.0828133076F,
      -5.53284872E-6F,  -0.113134429F,   -0.059760198F,    -0.0627897531F,
      -0.0618654266F,   -0.0196831338F,  -0.0732239559F,   -0.141003877F,
      0.0247876495F,    -0.0263536554F,  -0.0481459685F,   0.117358141F,
      0.0391057953F,    0.0896818489F,   0.0654849187F,    -0.00768541312F,
      0.0538121276F,    -0.047982987F,   -0.0404589549F,   -0.135821819F,
      0.0268000811F,    0.0711977258F,   -0.17732428F,     -0.0564031117F,
      0.0406806059F,    0.0476550907F,   -0.00615551416F,  0.0347627401F,
      -0.0756491944F,   0.113155693F,    -0.0397058316F,   -0.13160032F,
      -0.0346546955F,   0.060410995F,    0.0291102938F,    0.0599014796F,
      0.0854606181F,    0.00702863559F,  0.0160447583F,    -0.00386568F,
      -0.0831882954F,   -0.133795351F,   -0.0174864959F,   0.0369861908F,
      -0.0319713019F,   -0.103933722F,   0.0674948916F,    -0.0395425186F,
      -0.0368990786F,   0.0848057941F,   0.000755182584F,  0.0333253257F,
      -0.0613935664F,   0.033692766F,    0.0119208144F,    -0.0833143294F,
      -0.0375730954F,   -0.0658254176F,  0.0554527417F,    0.0683134496F,
      -0.000156933806F, -0.0615414158F,  0.0618847273F,    -0.0927947685F,
      -0.0748310387F,   0.0107859913F,   -0.0461571254F,   0.0321356133F,
      0.147818565F,     0.0709265396F,   0.0458077081F,    -0.126907185F,
      -0.0407897532F,   0.0203497689F,   0.0383627713F,    0.0267330818F,
      -0.00844893F,     0.119115047F,    0.0410956778F,    -0.0161716677F,
      0.0212592389F,    -0.0937888F,     0.194120735F,     -0.111686707F,
      0.0527529344F,    0.173062578F,    -0.0845781267F,   -0.0451371409F,
      -0.0230786148F,   0.0797374919F,   0.0838034153F,    0.0859962255F,
      0.0857650563F,    0.117152028F,    -0.0873487666F,   -0.0366373621F,
      -0.16395162F,     0.198779836F,    -0.122809708F,    0.112364255F,
      -0.0608512945F,   0.136146441F,    0.101224154F,     -0.0293973349F,
      0.0118879657F,    -0.105348967F,   0.0315760635F,    -0.0825052038F,
      -0.0161802974F,   0.124513149F,    -0.0738495886F,   0.0854940191F,
      -0.0228314176F,   0.12336836F,     -0.0864895731F,   -0.0122804325F,
      -0.08522138F,     0.0396742076F,   -0.134331524F,    0.116553783F,
      0.0379217565F,    0.0901923627F,   -0.0321430787F,   -0.149291709F,
      0.0575617924F,    0.0372415669F,   -0.12450175F,     -0.0644409F,
      -0.0950976387F,   0.0709939599F,   -0.063473776F,    0.0970487595F,
      0.0531020351F,    -0.133462399F,   0.0753263459F,    0.135036364F,
      0.113375962F,     0.0448867977F,   0.148495749F,     -0.084458679F,
      0.106599547F,     0.134337202F,    -0.0225863792F,   0.139135018F,
      -0.0932145417F,   0.172870591F,    -0.0492766052F,   -0.170713648F,
      0.072303541F,     0.0543272607F,   0.00664667133F,   -0.0314666517F,
      -0.0689923465F,   -0.12722832F,    0.0856631249F,    -0.0985553935F,
      -0.108839639F,    -0.0379955098F,  -0.00690981839F,  0.0811573192F,
      -0.116188638F,    -0.0412975512F,  -0.0196486F,      0.0512614325F,
      -0.00666035851F,  -0.0136203552F,  0.0949152857F,    0.00967262592F,
      0.104683153F,     0.13578096F,     -0.0579584651F,   -0.0995246843F,
      -0.0331460498F,   -0.00990602467F, -0.0982328132F,   -0.098602213F,
      -0.185585305F,    -0.00702982349F, -0.0214692038F,   -0.129947647F,
      -0.00763604976F,  0.0911096185F,   0.0778742284F,    0.0522789583F,
      -0.0399058238F,   0.158155113F,    0.0602001F,       0.035784F,
      0.159825027F,     -0.0734739676F,  -0.10909877F,     0.159629703F,
      -0.0240609217F,   -0.0632497519F,  0.190185755F,     -0.133200943F,
      0.152819812F,     0.0906430259F,   0.0164673459F,    0.081808731F,
      0.0505970269F,    0.0744183436F,   0.0222218931F,    -0.101006575F,
      -0.100737482F,    -0.0229815971F,  0.0480807051F,    -0.0171223599F,
      -0.0980991349F,   -0.0687330887F,  -0.0312971622F,   -0.106182F,
      0.0226897579F,    -0.0518402494F,  -0.0173405353F,   0.0810321718F,
      -0.03371666F,     0.086226128F,    0.124178387F,     -0.148859724F,
      0.0453832038F,    -0.0604256876F,  -0.049505081F,    -0.0438276641F,
      -0.128427505F,    0.0700635F,      -0.0603688024F,   -0.206760243F,
      -0.0724458843F,   0.0156603251F,   -0.100084931F,    -0.000625798071F,
      0.0178161766F,    0.114546634F,    0.00561303506F,   0.10927213F,
      -0.0574391596F,   0.186856732F,    -0.0304800607F,   0.156549633F,
      0.103008211F,     -0.0390539579F,  0.030436594F,     -0.027494492F,
      0.0583722852F,    0.166237026F,    -0.020939799F,    0.136222243F,
      -0.0286387671F,   0.0596624129F,   -0.0662406459F,   -0.0548776F,
      -0.105013885F,    0.0374726057F,   0.0330056623F,    0.0227742139F,
      0.0137794F,       0.0322305448F,   0.00449689198F,   -0.117792785F,
      0.0262855757F,    -0.0173454955F,  -0.00260197488F,  0.00320907123F,
      -0.0114111248F,   -0.0394091792F,  0.0803578347F,    0.0249521863F,
      0.0763862133F,    0.0236630253F,   -0.0122313565F,   0.0984600782F,
      0.0106714237F,    0.0503323674F,   0.0933761597F,    0.0671246052F,
      -0.050377354F,    0.082435526F,    -0.0923466757F,   0.0750206262F,
      -0.0781476051F,   0.0281693786F,   -0.0122709535F,   -0.104039542F,
      -0.0103955688F,   0.0596652776F,   -0.106113054F,    -0.101902135F,
      -0.0351420678F,   0.0230276026F,   -0.0370125137F,   -0.00636257138F,
      0.00630288338F,   -0.00111086294F, 0.000712709036F,  -0.0174674299F,
      -0.0941598192F,   -0.0292181857F,  -0.0824666098F,   0.00491729705F,
      0.109702043F,     0.0759979784F,   -0.0647334307F,   -0.0139490962F,
      -0.0569295F,      -0.0123433424F,  0.0401845165F,    0.0833024308F,
      -0.0256386138F,   -0.092207782F,   -0.0419321842F,   -0.000921578321F,
      -0.107464023F,    -0.0654481724F,  0.0233785417F,    -0.0572612546F,
      -0.0712999254F,   0.0903860703F,   -0.055113595F,    -0.120963439F,
      -0.0930436254F,   -0.0191022661F,  -0.061055813F,    0.0602079257F,
      0.108348422F,     -0.0695156753F,  -0.0762485787F,   0.0598745532F,
      -0.0518926606F,   0.0101316841F,   0.0673437342F,    0.0633847117F,
      -0.00913793407F,  -0.0652967542F,  0.059617646F,     -0.121958978F,
      0.0760002062F,    0.0564365387F,   -0.0875393823F,   0.0877022594F,
      0.0271002762F,    -0.0326790176F,  0.018124273F,     -0.0290423613F,
      -0.0400544181F,   -0.0922092572F,  0.124076314F,     -0.086169444F,
      0.0185506102F,    -0.0998144224F,  -0.0987201855F,   -0.00707692F,
      0.0650679693F,    0.113489419F,    -0.0681109205F,   0.0652892217F,
      -0.0511508361F,   -0.105363257F,   0.0950206742F,    0.0865170285F,
      -0.0348257609F,   0.0238106195F,   0.0418223441F,    0.0854484811F,
      0.0837034509F,    -0.0149038872F,  -0.0208017081F,   -0.0843695477F,
      -0.0792575553F,   -0.112986669F,   -0.0829631612F,   -0.000987681F,
      0.0267963298F,    0.106654324F,    -0.0414419807F,   0.026084166F,
      -0.0413365923F,   -0.0473554172F,  -0.103913069F,    -0.0172792301F,
      -0.0431996696F,   0.106199302F,    0.0334751904F,    -0.111225F,
      0.0356809236F,    -0.0870870054F,  0.0067235F,       0.0681192279F,
      -0.019269241F,    -0.0419334173F,  -0.0510771424F,   0.0517879315F,
      -0.0632262677F,   0.0101514654F,   -0.0621988922F,   0.042753987F,
      -0.12717852F,     0.0990773439F,   -0.0262881387F,   0.0953026563F,
      0.00772025203F,   -0.0839307681F,  -0.0807953551F,   0.0220895931F,
      -0.0460209176F,   0.0946666151F,   -0.0209907629F,   -0.0921818912F,
      -0.0465116054F,   -0.143315971F,   0.0631280094F,    0.0408173501F,
      0.00611649221F,   0.0121075474F,   0.0820310265F,    0.0915657952F,
      -0.0645360127F,   0.0182728413F,   0.0619116202F,    0.0329780914F,
      -0.0564574189F,   -0.0971924961F,  0.0224602222F,    0.00911763404F,
      -0.117636681F,    -0.12542899F,    -0.104668617F,    -0.0133570554F,
      0.093621172F,     0.085325934F,    0.0148837576F,    -0.00971004274F,
      -0.0910268575F,   0.0385629386F,   -0.0417402163F,   0.0986020267F,
      0.0644508228F,    -0.0586029701F,  0.0978698134F,    -0.0910311863F,
      0.0218677316F,    -0.0037425824F,  0.0650147796F,    0.114847332F,
      -0.00402344204F,  0.0937549397F,   -0.0152258389F,   -0.144428819F,
      -0.129116654F,    0.0332253501F,   0.0725724846F,    0.0965769887F,
      0.0568207279F,    -0.00640153605F, 0.0827859789F,    -0.0839517787F,
      0.0763064176F,    -0.0761756077F,  0.125345901F,     -0.0394925363F,
      -0.050645262F,    0.0178174023F,   0.118309908F,     -0.146140963F,
      0.0402463302F,    -0.0740658641F,  0.0628780052F,    -0.03012155F,
      -0.0505024604F,   0.0376312733F,   -0.0557635278F,   -0.105379432F,
      0.0167877246F,    0.103883632F,    -0.00211233483F,  0.0163460709F,
      -0.0589657426F,   -0.0050314921F,  0.0424911492F,    0.0950980037F,
      0.0287932549F,    -0.0717626959F,  -0.0252620764F,   0.0844539478F,
      -0.116111189F,    0.0667367131F,   0.124386117F,     -0.112033345F,
      0.0790243521F,    -0.119469926F,   0.0910510942F,    0.0613993257F,
      -0.0653587282F,   0.0784661397F,   -0.103654936F,    -0.0274015386F,
      -0.135077909F,    0.123248711F,    0.0859509F,       0.0619340166F,
      -0.00372597878F,  0.0639410317F,   0.157469213F,     -0.111828752F,
      0.0429846495F,    0.0186849944F,   0.15712668F,      0.0429285914F,
      0.0215316117F,    0.00650637643F,  0.136407614F,     0.129792675F,
      0.0187378284F,    -0.022072468F,   0.0601069257F,    0.152158052F,
      -0.0878715217F,   -0.0847347826F,  0.018058341F,     -0.0280724466F,
      -0.00939107314F,  -0.086060442F,   -0.065787904F,    0.0114327148F,
      0.0201708023F,    0.0524258241F,   -0.0547132F,      -0.0751906857F,
      -0.0919147357F,   0.0538353361F,   0.0129408482F,    0.0182740465F,
      0.0817092061F,    0.0890251175F,   -0.0942152F,      -0.0103004724F,
      -0.0234939661F,   0.0349049419F,   -0.00719124777F,  -0.0197384749F,
      0.0920883194F,    -0.0539329872F,  -0.0674274266F,   -0.0496856757F,
      0.0922817737F,    -0.137026F,      0.129159763F,     0.0193040203F,
      -0.136629254F,    -0.0780118108F,  0.0385443121F,    -0.032529F,
      0.117177077F,     -0.0324228965F,  0.059403915F,     -0.0357053764F,
      -0.0835044682F,   -0.136406377F,   -0.0745083764F,   -0.00981676672F,
      -0.0689353123F,   -0.0939826444F,  0.0570775531F,    0.0193875618F,
      0.0594892763F,    -0.0329343714F,  0.0155667467F,    0.120782882F,
      0.0203972254F,    -0.0189508665F,  0.0514345318F,    0.150782153F,
      -0.0294890106F,   -0.048721537F,   0.17220439F,      -0.00633769762F,
      -0.116064481F,    -0.0327969193F,  0.0729694664F,    0.00168218976F,
      0.0440196507F,    0.0573156551F,   0.0689464062F,    -0.0104655959F,
      -0.0327497572F,   -0.0612644441F,  -0.119876437F,    0.0115393698F,
      0.0636619329F,    -0.00185219443F, 0.0926178172F,    0.140532359F,
      0.0642057583F,    -0.167636886F,   0.0809207857F,    0.108951136F,
      -0.0412605405F,   -0.21232909F,    0.041136533F,     0.0758529752F,
      -0.0273527578F,   -0.0622390956F,  0.0577226616F,    -0.0548450723F,
      -0.146157607F,    -0.0498999394F,  -0.0614416152F,   -0.0899194F,
      0.0730410144F,    0.107318759F,    0.0209784359F,    0.121407166F,
      -0.10185045F,     0.0800137445F,   0.097667031F,     0.0839982703F,
      0.173727691F,     0.140017763F,    0.181164265F,     0.0292240027F,
      -0.0766950771F,   -0.0382101908F,  0.0056706653F,    -0.109975919F,
      0.0332124531F,    0.159884095F,    -0.0242874082F,   0.0603067204F,
      0.138288021F,     0.047375191F,    0.105112761F,     0.134472743F,
      0.0480530113F,    -0.117753677F,   -0.0691135228F,   -0.0788356513F,
      0.0814505F,       0.0340339653F,   -0.0376321375F,   0.0374054648F,
      0.00642241491F,   -0.0057197758F,  -0.133563757F,    0.142615452F,
      -0.165361688F,    -0.0716057F,     0.0202622283F,    -0.112669326F,
      -0.0112865679F,   -0.0275298394F,  0.0468882F,       0.00738190953F,
      0.0414359644F,    -0.121455356F,   -0.123071074F,    -0.10155011F,
      -0.0384909511F,   -0.0172224119F,  -0.109828494F,    -0.00842307787F,
      -0.124874823F,    -0.0323432572F,  0.0595502853F,    0.0254798159F,
      -0.0298359059F,   0.253414333F,    0.0771639422F,    -0.117799088F,
      -0.150014207F,    0.0918069854F,   0.012742091F,     0.131465688F,
      0.0830694959F,    0.12665315F,     -0.0347700939F,   0.0255609229F,
      -0.0416398942F,   0.204449058F,    -0.119880006F,    0.110563792F,
      -0.0981165096F,   0.0958400816F,   0.115008675F,     -0.0126012322F,
      0.0479550734F,    -0.0606010072F,  0.00297392695F,   0.0449414439F,
      -0.0532571413F,   0.158002242F,    0.0598014295F,    0.0495886F,
      0.050338883F,     -0.0729027F,     0.177168295F,     0.00599678326F,
      0.0401936434F,    0.0349661112F,   -0.0852381736F,   -0.176923886F,
      0.106409967F,     -0.0647714883F,  -0.102077514F,    0.00075661391F,
      0.0163254216F,    1.99940787E-5F,  0.0194786396F,    -0.0977714136F,
      -0.1725889F,      0.0862423107F,   -0.0305911712F,   0.0380277596F,
      -0.0390973873F,   0.0757724792F,   -0.105386198F,    0.00089222379F,
      0.0723330677F,    0.080902949F,    -0.106388479F,    0.0767930374F,
      -0.0926558599F,   -0.115308821F,   0.0245152377F,    0.0355490185F,
      0.00544892438F,   0.0801868439F,   0.0179699603F,    -0.0934417173F,
      -0.0663856938F,   -0.00567197334F, 0.0915769935F,    -0.110461496F,
      -0.13134104F,     0.00645587F,     -0.072645247F,    0.021853853F,
      -0.0797636732F,   -0.00678417506F, 0.0691073686F,    0.111756906F,
      0.0240086354F,    0.0814612508F,   0.0590652265F,    -0.0643337443F,
      -0.0826344639F,   -0.0652111F,     0.1135794F,       -0.0993528739F,
      -0.143949389F,    0.179153621F,    -0.0149881281F,   0.111642823F,
      -0.0308573116F,   -0.0493075438F,  -0.085044153F,    -0.106312536F,
      -0.08493945F,     0.18463926F,     -0.132567808F,    0.164054126F,
      0.0901309699F,    -0.0924946293F,  0.0566681065F,    -0.0679724738F,
      -0.0778311491F,   0.129184261F,    0.141541466F,     0.128786787F,
      0.0364949666F,    -0.0508815274F,  -0.0492066108F,   -0.140492871F,
      0.163634673F,     0.135455951F,    -0.156085685F,    0.09982308F,
      0.109712414F,     0.135235593F,    0.128545716F,     -0.122692414F,
      -0.102641724F,    0.173991278F,    -0.0525043048F,   -0.0802662224F,
      0.0396923535F,    0.0237194952F,   -0.0134486789F,   0.000755336601F,
      -0.0064434628F,   0.102970295F,    -0.00130873243F,  0.116328083F,
      -0.0164225437F,   0.12168026F,     -0.130479589F,    0.0144948075F,
      -0.0291617103F,   0.0543891154F,   0.0581531636F,    0.0682066083F,
      0.152341411F,     0.0984841734F,   0.0215226077F,    0.0881834328F,
      -0.0287534799F,   0.0197603349F,   0.0910580903F,    0.190130055F,
      0.0983341485F,    -0.0555179752F,  -0.144032523F,    0.105838507F,
      -0.109091111F,    -0.0252765659F,  0.077643618F,     0.0182174891F,
      0.0205379147F,    0.149344802F,    -0.0808189139F,   -0.0612568669F,
      0.0974766761F,    0.14105019F,     -0.0935703665F,   0.199608818F,
      -0.0265384782F,   -0.082493268F,   -0.00454291329F,  -0.089124769F,
      -0.000650276896F, -0.0530781783F,  -0.12495894F,     -0.0186285935F,
      0.0166934952F,    0.167077184F,    0.0229749158F,    -0.0773676708F,
      0.028285088F,     0.216900796F,    0.113195725F,     0.0319654122F,
      -0.0346673951F,   -0.0178626012F,  0.131276846F,     -0.0341532715F,
      -0.0210375506F,   -0.0472938232F,  0.0501490645F,    0.00735924905F,
      -0.0424951576F,   -0.0733274147F,  0.0718803331F,    0.0941091701F,
      -0.0592870079F,   -0.0738162771F,  0.0596700497F,    0.0657569319F,
      0.0229726695F,    0.0843008682F,   0.202255413F,     0.059145052F,
      0.0397545397F,    0.0168585815F,   -0.0629669875F,   -0.0977250263F,
      -0.0858192891F,   0.118601926F,    -0.00212982064F,  0.0207471F,
      -0.00342234643F,  -0.125328347F,   0.138305858F,     0.059122093F,
      -0.0804603472F,   -0.090779826F,   0.00255943649F,   -0.0695958F,
      0.0986626223F,    0.152122229F,    0.104133345F,     0.191209793F,
      0.0148916589F,    0.103003338F,    -0.147017583F,    -0.156829089F,
      -0.147536099F,    0.0857738405F,   -0.0542778596F,   0.0297151F,
      -0.0481166802F,   0.0116807166F,   0.0363710076F,    -0.00343599496F,
      0.126720041F,     0.087882936F,    -0.079114683F,    -0.0137390383F,
      -0.050218951F,    0.0138292F,      -0.0620346293F,   -0.0540550463F,
      -0.154633328F,    0.0291706827F,   0.177623317F,     0.0886102542F,
      -0.0513099283F,   0.0961052701F,   0.0563338287F,    0.0101916697F,
      -0.0666881725F,   -0.0162595473F,  0.00848564878F,   -0.131739452F,
      -0.0737029836F,   -0.00783841126F, 0.0452649444F,    0.00134132407F,
      -0.0759446248F,   -0.0776563F,     0.177480221F,     -0.00543112773F,
      0.00470650662F,   0.00776873808F,  -0.065174222F,    0.0307017826F,
      0.0810232535F,    0.0236821324F,   0.0858344287F,    -0.117731988F,
      -0.0683387741F,   0.0848049596F,   -0.0966516F,      0.132371292F,
      -0.0647952F,      0.13899447F,     0.0438596047F,    -0.103762284F,
      -0.0400625505F,   -0.00942956749F, 0.132936791F,     -0.00314361649F,
      -0.0314762779F,   -0.0269308817F,  -0.00901281F,     -0.0475669317F,
      -0.0120685548F,   0.01297809F,     -0.03099012F,     0.140594393F,
      -0.00202900497F,  0.0272421688F,   0.102720335F,     0.0547563806F,
      -0.031261459F,    0.0834965F,      -0.126942828F,    -0.0749066547F,
      0.0430760831F,    -0.147610664F,   -0.0512115322F,   -0.045694124F,
      -0.019018501F,    -0.015247318F,   -0.0916811675F,   0.0271477811F,
      -0.0938241333F,   -0.150766626F,   0.0299603567F,    0.0484463908F,
      0.101930454F,     -0.10477601F,    0.0213143285F,    0.0673241466F,
      -0.0752780661F,   -0.114447638F,   -0.0716421753F,   0.112199962F,
      0.00602410501F,   -0.131891236F,   -0.0151935499F,   0.0163289104F,
      0.0125290202F,    -0.113147803F,   -0.00253576506F,  0.071982421F,
      -0.00961636286F,  -0.100217164F,   -0.12670286F,     0.0608386025F,
      0.00872459542F,   -0.0656627417F,  0.142614439F,     -0.0377269387F,
      -0.0576112568F,   -0.0502623655F,  -0.00946835149F,  -0.0945003107F,
      0.0453272462F,    0.10180705F,     -0.0596834533F,   -0.0442722961F,
      -0.0427642092F,   0.100553676F,    -0.0443282053F,   -0.0628707111F,
      -0.125081658F,    0.0848966688F,   -0.0175691787F,   -0.0932586268F,
      0.0848971158F,    -0.0027200873F,  -0.0169706531F,   -0.10911727F,
      0.0441650264F,    0.19564195F,     -0.035425745F,    -0.0744880363F,
      0.027337499F,     -0.046947F,      0.0622675382F,    -0.0242769141F,
      -0.0834351853F,   -0.0213134252F,  -0.0464185514F,   -0.00444738194F,
      -0.0397727415F,   0.0487582609F,   0.000358051184F,  -0.110072419F,
      -0.0775030181F,   -0.110241242F,   0.0010701007F,    0.0965658054F,
      -0.0628931597F,   -0.00214723404F, -0.00743995095F,  0.111304171F,
      0.0411540717F,    0.0363700613F,   -0.0231012404F,   -0.0033403961F,
      -0.100506857F,    -0.0775597095F,  -0.0143303331F,   -0.0550396256F,
      0.096535109F,     0.0219373144F,   0.0386979692F,    0.0288808905F,
      0.00897307F,      -0.0816249102F,  -0.0329664387F,   0.0826085657F,
      -0.0295410361F,   0.0715284571F,   0.130532131F,     0.0544961393F,
      -0.0519875027F,   0.0166679509F,   0.109321803F,     -0.119946659F,
      0.132664F,        0.160941735F,    -0.119937815F,    0.0703003928F,
      0.106960982F,     0.00784922484F,  0.156688496F,     0.0762825534F,
      -0.0696859658F,   -0.0515788645F,  -0.0569707677F,   -0.0759049058F,
      -0.120025955F,    0.115818836F,    0.148258388F,     -0.0868986174F,
      0.16286172F,      -0.0671370775F,  0.0517541803F,    -0.0512105338F,
      0.122819319F,     0.0606419332F,   -0.0769887045F,   0.0766797438F,
      0.0680364147F,    -0.018069528F,   -0.0594109595F,   -0.00995222572F,
      -0.123909272F,    0.0500809699F,   0.11403209F,      -0.00115769415F,
      0.107444629F,     -0.0941019133F,  0.0377660766F,    0.146029875F,
      -0.0537516363F,   -0.108115725F,   0.064667955F,     -0.0710544884F,
      0.00413537771F,   0.0961169F,      0.0103418566F,    0.0733284F,
      0.0588771217F,    -0.0959097892F,  0.00811543595F,   -0.0246123243F,
      0.0142494719F,    -0.0307864323F,  0.12577045F,      0.00311803608F,
      0.0383239649F,    -0.112344645F,   -0.117083326F,    0.0145641351F,
      0.0373218097F,    0.0873665735F,   -0.0078363372F,   0.11309164F,
      0.0207652561F,    0.0406454392F,   0.0112124262F,    0.00356944627F,
      -0.0875472F,      0.0370840728F,   0.0688489899F,    -0.0436923541F,
      -0.0459358841F,   0.0503792502F,   -0.0256128293F,   0.0900644064F,
      -0.0396757647F,   -0.0604242459F,  0.0720745623F,    0.0107310265F,
      0.000457796268F,  -0.136573866F,   -0.125950024F,    0.041056972F,
      -0.129256219F,    -0.113692105F,   -0.0223981366F,   -0.0660093948F,
      -0.113542877F,    -0.100006871F,   0.0246075597F,    -0.0176056232F,
      0.0450695418F,    -0.00532934302F, -0.0194428787F,   0.0307229646F,
      -0.0698571876F,   -0.055615373F,   -0.0810462534F,   0.0699828938F,
      -0.0583231747F,   -0.0671498552F,  -0.0807767883F,   0.0583155155F,
      0.0212360471F,    -0.047202643F,   -0.0439172164F,   -0.0487083346F,
      -0.135371715F,    -0.0705275F,     0.0976638794F,    0.0626369715F,
      -0.0242412612F,   -0.0503163114F,  0.058569137F,     0.00765272928F,
      -0.00922718F,     0.169982478F,    -0.0120892571F,   -0.0644522533F,
      0.0768864155F,    -0.0520249233F,  -0.00850425195F,  -0.0439061485F,
      0.00930914376F,   -0.0762360394F,  0.0381214842F,    0.0176291224F,
      -0.138830453F,    -0.0737030953F,  0.083261244F,     0.0478087887F,
      0.0402943119F,    -0.013984791F,   0.0273161065F,    0.0571782179F,
      0.0882850885F,    -0.0435743965F,  0.124258131F,     -0.127011791F,
      -0.0519235134F,   0.0205655042F,   0.0346890166F,    -0.0199490637F,
      0.0898336F,       -0.0353171974F,  0.0205505267F,    0.0251662061F,
      0.026807148F,     0.0624708273F,   -0.0104733771F,   -0.00127756735F,
      0.0350503735F,    0.0914469734F,   0.011201825F,     -0.150467247F,
      -0.0686576143F,   -0.042861376F,   0.0627437606F,    0.0746969432F,
      0.100855082F,     0.056695316F,    0.112503F,        -0.0902731046F,
      0.0574462861F,    0.107643425F,    -0.0821380094F,   -0.017203575F,
      0.0599273406F,    0.136755794F,    0.0746793374F,    -0.000889278657F,
      0.0945320651F,    -0.0124241281F,  -0.142580464F,    -0.0934234634F,
      -0.0900548548F,   0.154668242F,    -0.106412396F,    -0.0353086628F,
      -0.101034746F,    0.0464365967F,   -0.0177986231F,   0.185294747F,
      0.0318957902F,    0.102035932F,    0.147325709F,     -0.0371455587F,
      -0.0601232573F,   -0.0267079F,     -0.0398281701F,   -0.0552593432F,
      -0.107220851F,    -0.0306924954F,  0.0228389613F,    -0.0314679854F,
      -0.0690614358F,   -0.1549142F,     0.159945428F,     0.114636265F};
  static const float biasReformatted[32] = {
      0.00711876946F,   0.00203418359F, 0.000780606F,   0.00729124947F,
      -0.00296763983F,  0.0385469273F,  0.0410197861F,  0.0423283167F,
      -0.000714529946F, 0.00969356298F, 0.0487383567F,  0.00497996621F,
      0.0813640952F,    0.0185002722F,  0.00452362048F, -0.0154616833F,
      -0.0369951613F,   0.013363339F,   -0.0113032656F, 0.0420880765F,
      -0.0223521721F,   0.0545247942F,  0.0902045816F,  0.0258819498F,
      -0.00313426787F,  0.00511612836F, 0.00891159195F, 0.0773750469F,
      -0.0215606652F,   0.0216861088F,  -0.0121716522F, 0.0128642088F};
  c_convolution(&X[0], &Z[0], &reformattedAndTruncatedWeights[0],
                &biasReformatted[0]);
}

/*
 * Arguments    : const float X[784]
 *                float Z[6272]
 * Return Type  : void
 */
void conv2dDirectOptimizedColMajor(const float X[784], float Z[6272])
{
  static const float reformattedAndTruncatedWeights[1152] = {
      -2.38893747F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -2.15054F,      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -0.471868813F,  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      1.10695517F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.371866614F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      1.97878599F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      1.19690025F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.432158023F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      1.21241784F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -1.77888668F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.650644F,      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -1.3448683F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -0.42457068F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -1.67887414F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -0.934826672F,  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -0.300812334F,  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -0.523534834F,  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      1.30798304F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -2.37964034F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      1.05848181F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      1.88873291F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -2.29561591F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.155495092F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      1.71867025F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -1.2278837F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -0.229714394F,  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.920673966F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      1.5980531F,     0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -0.446940571F,  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      1.31606984F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      2.17930341F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -1.48023438F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -0.62566179F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      1.55840361F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -1.58660877F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      2.06921864F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -1.98235416F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -0.0484360531F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -1.47720277F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      1.03461409F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -2.18049312F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.263626397F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      1.56553173F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -0.0265448503F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -2.07452464F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      1.68857026F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      3.53303F,       0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -0.620135844F,  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -1.39296949F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -0.239413962F,  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -0.635813534F,  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      1.58316267F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -2.07559F,      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      2.10373425F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      2.2616117F,     0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      1.82612014F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      2.02969718F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -3.44172788F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0432825424F,  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.344503343F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -0.44454062F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -0.496031761F,  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -1.83852398F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -0.75915724F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -2.61581564F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -1.73088121F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      2.58849406F,    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0929945409F,  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -1.87408519F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.318932265F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      1.7736094F,     0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -1.29142356F,   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F,           0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  static const float biasReformatted[8] = {
      -0.128483444F, 0.50627923F,   0.00699076243F, -0.353693664F,
      0.361517698F,  -0.371032566F, -0.0906580389F, 0.208790556F};
  convolution(&X[0], &Z[0], &reformattedAndTruncatedWeights[0],
              &biasReformatted[0]);
}

/*
 * File trailer for conv2dDirectOptimizedColMajor.c
 *
 * [EOF]
 */
