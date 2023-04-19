//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_forShooting_keith_api.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 13-Apr-2023 16:35:29
//

// Include Files
#include "_coder_forShooting_keith_api.h"
#include "_coder_forShooting_keith_mex.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131611U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "forShooting_keith",                                  // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

// Function Declarations
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T **ret_data, int32_T ret_size[2]);

static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *qa,
                                   const char_T *identifier))[12];

static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[12];

static const mxArray *b_emlrt_marshallOut(const real_T u_data[],
                                          const int32_T u_size[2]);

static const mxArray *b_emlrt_marshallOut(const real_T u[3]);

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *v,
                                   const char_T *identifier))[12];

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[12];

static const mxArray *c_emlrt_marshallOut(const real_T u[66]);

static int8_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *SegIdx,
                                 const char_T *identifier);

static int8_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static const mxArray *d_emlrt_marshallOut(const real_T u[2706]);

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *ksi,
                                   const char_T *identifier))[4];

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[4];

static const mxArray *e_emlrt_marshallOut(const real_T u[860]);

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *b_y0,
                             const char_T *identifier, real_T **y_data,
                             int32_T y_size[2]);

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             real_T **y_data, int32_T y_size[2]);

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *Guess,
                                 const char_T *identifier))[20];

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[20];

static const mxArray *emlrt_marshallOut(const real_T u[20]);

static void emlrt_marshallOut(const real_T u[20], const mxArray *y);

static const mxArray *emlrt_marshallOut(const real_T u_data[],
                                        const int32_T *u_size);

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *p,
                                   const char_T *identifier))[3];

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[3];

static const mxArray *f_emlrt_marshallOut(const real_T u[779]);

static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[20];

static const mxArray *g_emlrt_marshallOut(const real_T u[9]);

static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[12];

static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[12];

static int8_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[4];

static real_T (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3];

// Function Definitions
//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
//                real_T **ret_data
//                int32_T ret_size[2]
// Return Type  : void
//
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T **ret_data, int32_T ret_size[2])
{
  static const int32_T dims[2]{20, 22};
  int32_T iv[2];
  const boolean_T bv[2]{true, true};
  emlrtCheckVsBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                            false, 2U, (void *)&dims[0], &bv[0], &iv[0]);
  ret_size[0] = iv[0];
  ret_size[1] = iv[1];
  *ret_data = (real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *qa
//                const char_T *identifier
// Return Type  : real_T (*)[12]
//
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *qa,
                                   const char_T *identifier))[12]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[12];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(qa), &thisId);
  emlrtDestroyArray(&qa);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[12]
//
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[12]
{
  real_T(*y)[12];
  y = h_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const real_T u[3]
// Return Type  : const mxArray *
//
static const mxArray *b_emlrt_marshallOut(const real_T u[3])
{
  static const int32_T i{0};
  static const int32_T i1{3};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const real_T u_data[]
//                const int32_T u_size[2]
// Return Type  : const mxArray *
//
static const mxArray *b_emlrt_marshallOut(const real_T u_data[],
                                          const int32_T u_size[2])
{
  static const int32_T iv[2]{0, 0};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u_data[0]);
  emlrtSetDimensions((mxArray *)m, &u_size[0], 2);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *v
//                const char_T *identifier
// Return Type  : real_T (*)[12]
//
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *v,
                                   const char_T *identifier))[12]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[12];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = c_emlrt_marshallIn(sp, emlrtAlias(v), &thisId);
  emlrtDestroyArray(&v);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[12]
//
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[12]
{
  real_T(*y)[12];
  y = i_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const real_T u[66]
// Return Type  : const mxArray *
//
static const mxArray *c_emlrt_marshallOut(const real_T u[66])
{
  static const int32_T iv[2]{0, 0};
  static const int32_T iv1[2]{3, 22};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : int8_T
//
static int8_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  int8_T y;
  y = j_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *SegIdx
//                const char_T *identifier
// Return Type  : int8_T
//
static int8_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *SegIdx,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  int8_T y;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(SegIdx), &thisId);
  emlrtDestroyArray(&SegIdx);
  return y;
}

//
// Arguments    : const real_T u[2706]
// Return Type  : const mxArray *
//
static const mxArray *d_emlrt_marshallOut(const real_T u[2706])
{
  static const int32_T iv[2]{0, 0};
  static const int32_T iv1[2]{123, 22};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *ksi
//                const char_T *identifier
// Return Type  : real_T (*)[4]
//
static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *ksi,
                                   const char_T *identifier))[4]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[4];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = e_emlrt_marshallIn(sp, emlrtAlias(ksi), &thisId);
  emlrtDestroyArray(&ksi);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[4]
//
static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[4]
{
  real_T(*y)[4];
  y = k_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const real_T u[860]
// Return Type  : const mxArray *
//
static const mxArray *e_emlrt_marshallOut(const real_T u[860])
{
  static const int32_T iv[2]{0, 0};
  static const int32_T iv1[2]{43, 20};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *b_y0
//                const char_T *identifier
//                real_T **y_data
//                int32_T y_size[2]
// Return Type  : void
//
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *b_y0,
                             const char_T *identifier, real_T **y_data,
                             int32_T y_size[2])
{
  emlrtMsgIdentifier thisId;
  real_T *r;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  emlrt_marshallIn(sp, emlrtAlias(b_y0), &thisId, &r, y_size);
  *y_data = r;
  emlrtDestroyArray(&b_y0);
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *Guess
//                const char_T *identifier
// Return Type  : real_T (*)[20]
//
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *Guess,
                                 const char_T *identifier))[20]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[20];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = emlrt_marshallIn(sp, emlrtAlias(Guess), &thisId);
  emlrtDestroyArray(&Guess);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
//                real_T **y_data
//                int32_T y_size[2]
// Return Type  : void
//
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             real_T **y_data, int32_T y_size[2])
{
  real_T *r;
  b_emlrt_marshallIn(sp, emlrtAlias(u), parentId, &r, y_size);
  *y_data = r;
  emlrtDestroyArray(&u);
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[20]
//
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[20]
{
  real_T(*y)[20];
  y = g_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const real_T u[20]
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const real_T u[20])
{
  static const int32_T i{0};
  static const int32_T i1{20};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const real_T u_data[]
//                const int32_T *u_size
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const real_T u_data[],
                                        const int32_T *u_size)
{
  static const int32_T i{0};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u_data[0]);
  emlrtSetDimensions((mxArray *)m, u_size, 1);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const real_T u[20]
//                const mxArray *y
// Return Type  : void
//
static void emlrt_marshallOut(const real_T u[20], const mxArray *y)
{
  static const int32_T i{20};
  emlrtMxSetData((mxArray *)y, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)y, &i, 1);
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *p
//                const char_T *identifier
// Return Type  : real_T (*)[3]
//
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *p,
                                   const char_T *identifier))[3]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[3];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(p), &thisId);
  emlrtDestroyArray(&p);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[3]
//
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[3]
{
  real_T(*y)[3];
  y = l_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const real_T u[779]
// Return Type  : const mxArray *
//
static const mxArray *f_emlrt_marshallOut(const real_T u[779])
{
  static const int32_T iv[2]{0, 0};
  static const int32_T iv1[2]{41, 19};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[20]
//
static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[20]
{
  static const int32_T dims{20};
  real_T(*ret)[20];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)&dims);
  ret = (real_T(*)[20])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const real_T u[9]
// Return Type  : const mxArray *
//
static const mxArray *g_emlrt_marshallOut(const real_T u[9])
{
  static const int32_T iv[2]{0, 0};
  static const int32_T iv1[2]{3, 3};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[12]
//
static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[12]
{
  static const int32_T dims{12};
  real_T(*ret)[12];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)&dims);
  ret = (real_T(*)[12])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[12]
//
static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[12]
{
  static const int32_T dims[2]{3, 4};
  real_T(*ret)[12];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  ret = (real_T(*)[12])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : int8_T
//
static int8_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  int8_T ret;
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"int8",
                          false, 0U, (void *)&dims);
  ret = *(int8_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[4]
//
static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[4]
{
  static const int32_T dims{4};
  real_T(*ret)[4];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)&dims);
  ret = (real_T(*)[4])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[3]
//
static real_T (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3]
{
  static const int32_T dims{3};
  real_T(*ret)[3];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)&dims);
  ret = (real_T(*)[3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const mxArray * const prhs[3]
//                int32_T nlhs
//                const mxArray *plhs[9]
// Return Type  : void
//
void forShooting_keith_api(const mxArray *const prhs[3], int32_T nlhs,
                           const mxArray *plhs[9])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  real_T(*b_y1)[2706];
  real_T(*y2)[860];
  real_T(*y3)[779];
  real_T(*t1_data)[123];
  real_T(*b_y0)[66];
  real_T(*t2_data)[43];
  real_T(*t3_data)[41];
  real_T(*Guess)[20];
  real_T(*Rsd)[20];
  real_T(*ksi)[12];
  real_T(*qa)[12];
  real_T(*t0)[3];
  int32_T t1_size;
  int32_T t2_size;
  int32_T t3_size;
  st.tls = emlrtRootTLSGlobal;
  Rsd = (real_T(*)[20])mxMalloc(sizeof(real_T[20]));
  t0 = (real_T(*)[3])mxMalloc(sizeof(real_T[3]));
  b_y0 = (real_T(*)[66])mxMalloc(sizeof(real_T[66]));
  t1_data = (real_T(*)[123])mxMalloc(sizeof(real_T[123]));
  b_y1 = (real_T(*)[2706])mxMalloc(sizeof(real_T[2706]));
  t2_data = (real_T(*)[43])mxMalloc(sizeof(real_T[43]));
  y2 = (real_T(*)[860])mxMalloc(sizeof(real_T[860]));
  t3_data = (real_T(*)[41])mxMalloc(sizeof(real_T[41]));
  y3 = (real_T(*)[779])mxMalloc(sizeof(real_T[779]));
  // Marshall function inputs
  Guess = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "Guess");
  qa = b_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "qa");
  ksi = b_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "ksi");
  // Invoke the target function
  forShooting_keith(*Guess, *qa, *ksi, *Rsd, *t0, *b_y0, *t1_data,
                    *(int32_T(*)[1]) & t1_size, *b_y1, *t2_data,
                    *(int32_T(*)[1]) & t2_size, *y2, *t3_data,
                    *(int32_T(*)[1]) & t3_size, *y3);
  // Marshall function outputs
  plhs[0] = emlrt_marshallOut(*Rsd);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(*t0);
  }
  if (nlhs > 2) {
    plhs[2] = c_emlrt_marshallOut(*b_y0);
  }
  if (nlhs > 3) {
    plhs[3] = emlrt_marshallOut(*t1_data, &t1_size);
  }
  if (nlhs > 4) {
    plhs[4] = d_emlrt_marshallOut(*b_y1);
  }
  if (nlhs > 5) {
    plhs[5] = emlrt_marshallOut(*t2_data, &t2_size);
  }
  if (nlhs > 6) {
    plhs[6] = e_emlrt_marshallOut(*y2);
  }
  if (nlhs > 7) {
    plhs[7] = emlrt_marshallOut(*t3_data, &t3_size);
  }
  if (nlhs > 8) {
    plhs[8] = f_emlrt_marshallOut(*y3);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void forShooting_keith_atexit()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  forShooting_keith_xil_terminate();
  forShooting_keith_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void forShooting_keith_initialize()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void forShooting_keith_terminate()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

//
// Arguments    : const mxArray * const prhs[4]
//                int32_T nlhs
//                const mxArray *plhs[3]
// Return Type  : void
//
void odeCosserat_keith_api(const mxArray *const prhs[4], int32_T nlhs,
                           const mxArray *plhs[3])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  real_T(*y_data)[1320];
  real_T(*y0_data)[440];
  real_T(*U_data)[180];
  real_T(*t_data)[60];
  real_T(*v)[12];
  real_T(*ksi)[4];
  int32_T U_size[2];
  int32_T t_size[2];
  int32_T y0_size[2];
  int32_T y_size[2];
  int8_T SegIdx;
  st.tls = emlrtRootTLSGlobal;
  t_data = (real_T(*)[60])mxMalloc(sizeof(real_T[60]));
  y_data = (real_T(*)[1320])mxMalloc(sizeof(real_T[1320]));
  U_data = (real_T(*)[180])mxMalloc(sizeof(real_T[180]));
  // Marshall function inputs
  emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "y0", (real_T **)&y0_data,
                   y0_size);
  v = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "v");
  SegIdx = d_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "SegIdx");
  ksi = e_emlrt_marshallIn(&st, emlrtAlias(prhs[3]), "ksi");
  // Invoke the target function
  odeCosserat_keith(*y0_data, y0_size, *v, SegIdx, *ksi, *t_data, t_size,
                    *y_data, y_size, *U_data, U_size);
  // Marshall function outputs
  plhs[0] = b_emlrt_marshallOut(*t_data, t_size);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(*y_data, y_size);
  }
  if (nlhs > 2) {
    plhs[2] = b_emlrt_marshallOut(*U_data, U_size);
  }
}

//
// Arguments    : const mxArray * const prhs[3]
//                int32_T nlhs
//                const mxArray *plhs[9]
// Return Type  : void
//
void shootingOpt_keith_api(const mxArray *const prhs[3], int32_T nlhs,
                           const mxArray *plhs[9])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *prhs_copy_idx_0;
  real_T(*b_y1)[2706];
  real_T(*y2)[860];
  real_T(*y3)[779];
  real_T(*t1_data)[123];
  real_T(*b_y0)[66];
  real_T(*t2_data)[43];
  real_T(*t3_data)[41];
  real_T(*Guess)[20];
  real_T(*ksi)[12];
  real_T(*qa)[12];
  real_T(*t0)[3];
  int32_T t1_size;
  int32_T t2_size;
  int32_T t3_size;
  st.tls = emlrtRootTLSGlobal;
  t0 = (real_T(*)[3])mxMalloc(sizeof(real_T[3]));
  t1_data = (real_T(*)[123])mxMalloc(sizeof(real_T[123]));
  t2_data = (real_T(*)[43])mxMalloc(sizeof(real_T[43]));
  t3_data = (real_T(*)[41])mxMalloc(sizeof(real_T[41]));
  b_y0 = (real_T(*)[66])mxMalloc(sizeof(real_T[66]));
  b_y1 = (real_T(*)[2706])mxMalloc(sizeof(real_T[2706]));
  y2 = (real_T(*)[860])mxMalloc(sizeof(real_T[860]));
  y3 = (real_T(*)[779])mxMalloc(sizeof(real_T[779]));
  prhs_copy_idx_0 = emlrtProtectR2012b(prhs[0], 0, true, -1);
  // Marshall function inputs
  Guess = emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_0), "Guess");
  qa = b_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "qa");
  ksi = b_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "ksi");
  // Invoke the target function
  shootingOpt_keith(*Guess, *qa, *ksi, *t0, *t1_data,
                    *(int32_T(*)[1]) & t1_size, *t2_data,
                    *(int32_T(*)[1]) & t2_size, *t3_data,
                    *(int32_T(*)[1]) & t3_size, *b_y0, *b_y1, *y2, *y3);
  // Marshall function outputs
  emlrt_marshallOut(*Guess, prhs_copy_idx_0);
  plhs[0] = prhs_copy_idx_0;
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(*t0);
  }
  if (nlhs > 2) {
    plhs[2] = emlrt_marshallOut(*t1_data, &t1_size);
  }
  if (nlhs > 3) {
    plhs[3] = emlrt_marshallOut(*t2_data, &t2_size);
  }
  if (nlhs > 4) {
    plhs[4] = emlrt_marshallOut(*t3_data, &t3_size);
  }
  if (nlhs > 5) {
    plhs[5] = c_emlrt_marshallOut(*b_y0);
  }
  if (nlhs > 6) {
    plhs[6] = d_emlrt_marshallOut(*b_y1);
  }
  if (nlhs > 7) {
    plhs[7] = e_emlrt_marshallOut(*y2);
  }
  if (nlhs > 8) {
    plhs[8] = f_emlrt_marshallOut(*y3);
  }
}

//
// Arguments    : const mxArray *prhs
//                const mxArray **plhs
// Return Type  : void
//
void skewMatrix_keith_api(const mxArray *prhs, const mxArray **plhs)
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  real_T(*T)[9];
  real_T(*p)[3];
  st.tls = emlrtRootTLSGlobal;
  T = (real_T(*)[9])mxMalloc(sizeof(real_T[9]));
  // Marshall function inputs
  p = f_emlrt_marshallIn(&st, emlrtAlias(prhs), "p");
  // Invoke the target function
  skewMatrix_keith(*p, *T);
  // Marshall function outputs
  *plhs = g_emlrt_marshallOut(*T);
}

//
// File trailer for _coder_forShooting_keith_api.cpp
//
// [EOF]
//
