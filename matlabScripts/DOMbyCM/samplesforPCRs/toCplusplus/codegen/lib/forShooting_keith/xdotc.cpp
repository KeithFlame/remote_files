//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xdotc.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 13-Apr-2023 16:35:29
//

// Include Files
#include "xdotc.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : int n
//                const double x[9]
//                int ix0
//                const double y[9]
//                int iy0
// Return Type  : double
//
namespace coder {
namespace internal {
namespace blas {
double xdotc(int n, const double x[9], int ix0, const double y[9], int iy0)
{
  double d;
  d = 0.0;
  for (int k{0}; k < n; k++) {
    d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
  }
  return d;
}

} // namespace blas
} // namespace internal
} // namespace coder

//
// File trailer for xdotc.cpp
//
// [EOF]
//
