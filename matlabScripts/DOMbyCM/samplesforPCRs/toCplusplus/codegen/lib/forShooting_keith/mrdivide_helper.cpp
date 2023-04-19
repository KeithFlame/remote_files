//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mrdivide_helper.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 13-Apr-2023 16:35:29
//

// Include Files
#include "mrdivide_helper.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>

// Function Definitions
//
// Arguments    : double A[400]
//                const double B[400]
// Return Type  : void
//
namespace coder {
namespace internal {
void mrdiv(double A[400], const double B[400])
{
  double b_A[400];
  double smax;
  int b_i;
  int i;
  int j;
  int jA;
  int jAcol;
  int jp1j;
  int k;
  int kBcol;
  signed char ipiv[20];
  std::copy(&B[0], &B[400], &b_A[0]);
  for (i = 0; i < 20; i++) {
    ipiv[i] = static_cast<signed char>(i + 1);
  }
  for (j = 0; j < 19; j++) {
    int b_tmp;
    int mmj_tmp;
    mmj_tmp = 18 - j;
    b_tmp = j * 21;
    jp1j = b_tmp + 2;
    jA = 20 - j;
    jAcol = 0;
    smax = std::abs(b_A[b_tmp]);
    for (k = 2; k <= jA; k++) {
      double s;
      s = std::abs(b_A[(b_tmp + k) - 1]);
      if (s > smax) {
        jAcol = k - 1;
        smax = s;
      }
    }
    if (b_A[b_tmp + jAcol] != 0.0) {
      if (jAcol != 0) {
        jA = j + jAcol;
        ipiv[j] = static_cast<signed char>(jA + 1);
        for (k = 0; k < 20; k++) {
          kBcol = j + k * 20;
          smax = b_A[kBcol];
          jAcol = jA + k * 20;
          b_A[kBcol] = b_A[jAcol];
          b_A[jAcol] = smax;
        }
      }
      i = (b_tmp - j) + 20;
      for (b_i = jp1j; b_i <= i; b_i++) {
        b_A[b_i - 1] /= b_A[b_tmp];
      }
    }
    jA = b_tmp;
    for (jAcol = 0; jAcol <= mmj_tmp; jAcol++) {
      smax = b_A[(b_tmp + jAcol * 20) + 20];
      if (smax != 0.0) {
        i = jA + 22;
        jp1j = (jA - j) + 40;
        for (kBcol = i; kBcol <= jp1j; kBcol++) {
          b_A[kBcol - 1] += b_A[((b_tmp + kBcol) - jA) - 21] * -smax;
        }
      }
      jA += 20;
    }
  }
  for (j = 0; j < 20; j++) {
    jA = 20 * j - 1;
    jAcol = 20 * j;
    for (k = 0; k < j; k++) {
      kBcol = 20 * k;
      smax = b_A[k + jAcol];
      if (smax != 0.0) {
        for (b_i = 0; b_i < 20; b_i++) {
          i = (b_i + jA) + 1;
          A[i] -= smax * A[b_i + kBcol];
        }
      }
    }
    smax = 1.0 / b_A[j + jAcol];
    for (b_i = 0; b_i < 20; b_i++) {
      i = (b_i + jA) + 1;
      A[i] *= smax;
    }
  }
  for (j = 19; j >= 0; j--) {
    jA = 20 * j - 1;
    i = j + 2;
    for (k = i; k < 21; k++) {
      kBcol = 20 * (k - 1);
      smax = b_A[k + jA];
      if (smax != 0.0) {
        for (b_i = 0; b_i < 20; b_i++) {
          jp1j = (b_i + jA) + 1;
          A[jp1j] -= smax * A[b_i + kBcol];
        }
      }
    }
  }
  for (j = 18; j >= 0; j--) {
    signed char i1;
    i1 = ipiv[j];
    if (i1 != j + 1) {
      for (b_i = 0; b_i < 20; b_i++) {
        kBcol = b_i + 20 * j;
        smax = A[kBcol];
        i = b_i + 20 * (i1 - 1);
        A[kBcol] = A[i];
        A[i] = smax;
      }
    }
  }
}

} // namespace internal
} // namespace coder

//
// File trailer for mrdivide_helper.cpp
//
// [EOF]
//
