//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: rotm2axang.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 13-Apr-2023 16:35:29
//

// Include Files
#include "rotm2axang.h"
#include "forShooting_keith_rtwutil.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "svd.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double R[9]
//                double axang[4]
// Return Type  : void
//
namespace coder {
void rotm2axang(const double R[9], double axang[4])
{
  creal_T u;
  creal_T v;
  double U[9];
  double V[9];
  double b_I[9];
  double b_v[3];
  double vspecial_data[3];
  double br_tmp_tmp;
  int k;
  int nz_tmp;
  boolean_T b;
  boolean_T exitg1;
  boolean_T rEQ0;
  u.re = 0.5 * (((R[0] + R[4]) + R[8]) - 1.0);
  if (!(std::abs(u.re) > 1.0)) {
    br_tmp_tmp = u.re;
    u.re = std::acos(br_tmp_tmp);
  } else {
    v.re = u.re + 1.0;
    v.im = 0.0;
    internal::scalar::b_sqrt(&v);
    br_tmp_tmp = u.re;
    u.re = 1.0 - br_tmp_tmp;
    u.im = 0.0;
    internal::scalar::b_sqrt(&u);
    br_tmp_tmp = u.re;
    u.re = 2.0 * rt_atan2d_snf(br_tmp_tmp, v.re);
  }
  br_tmp_tmp = 2.0 * std::sin(u.re);
  b_v[0] = (R[5] - R[7]) / br_tmp_tmp;
  b_v[1] = (R[6] - R[2]) / br_tmp_tmp;
  b_v[2] = (R[1] - R[3]) / br_tmp_tmp;
  if (std::isnan(u.re) || std::isinf(u.re)) {
    br_tmp_tmp = rtNaN;
  } else if (u.re == 0.0) {
    br_tmp_tmp = 0.0;
  } else {
    br_tmp_tmp = std::fmod(u.re, 3.1415926535897931);
    rEQ0 = (br_tmp_tmp == 0.0);
    if (!rEQ0) {
      double q;
      q = std::abs(u.re / 3.1415926535897931);
      rEQ0 = !(std::abs(q - std::floor(q + 0.5)) > 2.2204460492503131E-16 * q);
    }
    if (rEQ0) {
      br_tmp_tmp = 0.0;
    } else if (u.re < 0.0) {
      br_tmp_tmp += 3.1415926535897931;
    }
  }
  rEQ0 = (br_tmp_tmp == 0.0);
  b = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if (!(b_v[k] == 0.0)) {
      b = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  nz_tmp = (rEQ0 || b);
  if (rEQ0 || b) {
    k = 3 * nz_tmp;
    if (0 <= k - 1) {
      std::memset(&vspecial_data[0], 0, k * sizeof(double));
    }
    for (int i{0}; i < nz_tmp; i++) {
      boolean_T p;
      std::memset(&b_I[0], 0, 9U * sizeof(double));
      b_I[0] = 1.0;
      b_I[4] = 1.0;
      b_I[8] = 1.0;
      p = true;
      for (k = 0; k < 9; k++) {
        br_tmp_tmp = b_I[k] - R[k];
        b_I[k] = br_tmp_tmp;
        if ((!p) || (std::isinf(br_tmp_tmp) || std::isnan(br_tmp_tmp))) {
          p = false;
        }
      }
      if (p) {
        internal::svd(b_I, U, vspecial_data, V);
      } else {
        for (k = 0; k < 9; k++) {
          V[k] = rtNaN;
        }
      }
      vspecial_data[0] = V[6];
      vspecial_data[1] = V[7];
      vspecial_data[2] = V[8];
    }
    k = 0;
    if (rEQ0 || b) {
      k = 1;
    }
    if (0 <= k - 1) {
      b_v[0] = vspecial_data[0];
      b_v[1] = vspecial_data[1];
      b_v[2] = vspecial_data[2];
    }
  }
  br_tmp_tmp =
      1.0 / std::sqrt((b_v[0] * b_v[0] + b_v[1] * b_v[1]) + b_v[2] * b_v[2]);
  axang[0] = b_v[0] * br_tmp_tmp;
  axang[1] = b_v[1] * br_tmp_tmp;
  axang[2] = b_v[2] * br_tmp_tmp;
  axang[3] = u.re;
}

} // namespace coder

//
// File trailer for rotm2axang.cpp
//
// [EOF]
//
