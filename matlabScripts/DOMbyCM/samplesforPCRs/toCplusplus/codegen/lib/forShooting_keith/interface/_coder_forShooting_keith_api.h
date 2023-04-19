//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_forShooting_keith_api.h
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 13-Apr-2023 16:35:29
//

#ifndef _CODER_FORSHOOTING_KEITH_API_H
#define _CODER_FORSHOOTING_KEITH_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void forShooting_keith(real_T Guess[20], real_T qa[12], real_T ksi[12],
                       real_T Rsd[20], real_T t0[3], real_T b_y0[66],
                       real_T t1_data[], int32_T t1_size[1], real_T b_y1[2706],
                       real_T t2_data[], int32_T t2_size[1], real_T y2[860],
                       real_T t3_data[], int32_T t3_size[1], real_T y3[779]);

void forShooting_keith_api(const mxArray *const prhs[3], int32_T nlhs,
                           const mxArray *plhs[9]);

void forShooting_keith_atexit();

void forShooting_keith_initialize();

void forShooting_keith_terminate();

void forShooting_keith_xil_shutdown();

void forShooting_keith_xil_terminate();

void odeCosserat_keith(real_T y0_data[], int32_T y0_size[2], real_T v[12],
                       int8_T SegIdx, real_T ksi[4], real_T t_data[],
                       int32_T t_size[2], real_T y_data[], int32_T y_size[2],
                       real_T U_data[], int32_T U_size[2]);

void odeCosserat_keith_api(const mxArray *const prhs[4], int32_T nlhs,
                           const mxArray *plhs[3]);

void shootingOpt_keith(real_T Guess[20], real_T qa[12], real_T ksi[12],
                       real_T t0[3], real_T t1_data[], int32_T t1_size[1],
                       real_T t2_data[], int32_T t2_size[1], real_T t3_data[],
                       int32_T t3_size[1], real_T b_y0[66], real_T b_y1[2706],
                       real_T y2[860], real_T y3[779]);

void shootingOpt_keith_api(const mxArray *const prhs[3], int32_T nlhs,
                           const mxArray *plhs[9]);

void skewMatrix_keith(real_T p[3], real_T T[9]);

void skewMatrix_keith_api(const mxArray *prhs, const mxArray **plhs);

#endif
//
// File trailer for _coder_forShooting_keith_api.h
//
// [EOF]
//
