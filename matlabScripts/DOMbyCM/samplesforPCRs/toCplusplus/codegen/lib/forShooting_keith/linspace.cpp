//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: linspace.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 13-Apr-2023 16:35:29
//

// Include Files
#include "linspace.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : double d1
//                double d2
//                double y[20]
// Return Type  : void
//
namespace coder {
void linspace(double d1, double d2, double y[20])
{
  double delta1;
  y[19] = d2;
  y[0] = d1;
  delta1 = (d2 - d1) / 19.0;
  for (int k{0}; k < 18; k++) {
    y[k + 1] = d1 + (static_cast<double>(k) + 1.0) * delta1;
  }
}

} // namespace coder

//
// File trailer for linspace.cpp
//
// [EOF]
//
