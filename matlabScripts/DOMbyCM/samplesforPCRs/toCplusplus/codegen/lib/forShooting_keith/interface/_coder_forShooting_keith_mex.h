//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_forShooting_keith_mex.h
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 13-Apr-2023 16:35:29
//

#ifndef _CODER_FORSHOOTING_KEITH_MEX_H
#define _CODER_FORSHOOTING_KEITH_MEX_H

// Include Files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

// Function Declarations
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS();

void unsafe_forShooting_keith_mexFunction(int32_T nlhs, mxArray *plhs[9],
                                          int32_T nrhs, const mxArray *prhs[3]);

void unsafe_odeCosserat_keith_mexFunction(int32_T nlhs, mxArray *plhs[3],
                                          int32_T nrhs, const mxArray *prhs[4]);

void unsafe_shootingOpt_keith_mexFunction(int32_T nlhs, mxArray *plhs[9],
                                          int32_T nrhs, const mxArray *prhs[3]);

void unsafe_skewMatrix_keith_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                         int32_T nrhs, const mxArray *prhs[1]);

#endif
//
// File trailer for _coder_forShooting_keith_mex.h
//
// [EOF]
//
