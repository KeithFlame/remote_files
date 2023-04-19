//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mldivide.h
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 13-Apr-2023 16:35:29
//

#ifndef MLDIVIDE_H
#define MLDIVIDE_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void mldivide(const double A[9], const double B_data[], const int B_size[2],
              double Y_data[], int Y_size[2]);

void mldivide(const double A[9], const double B[9], double Y[9]);

} // namespace coder

#endif
//
// File trailer for mldivide.h
//
// [EOF]
//
