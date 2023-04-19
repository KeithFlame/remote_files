//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mldivide.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 13-Apr-2023 16:35:29
//

// Include Files
#include "mldivide.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>

// Function Definitions
//
// Arguments    : const double A[9]
//                const double B_data[]
//                const int B_size[2]
//                double Y_data[]
//                int Y_size[2]
// Return Type  : void
//
namespace coder {
void mldivide(const double A[9], const double B_data[], const int B_size[2],
              double Y_data[], int Y_size[2])
{
  double b_A[9];
  double a21;
  double maxval;
  int r1;
  int r2;
  int r3;
  int rtemp;
  std::copy(&A[0], &A[9], &b_A[0]);
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = std::abs(A[0]);
  a21 = std::abs(A[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }
  if (std::abs(A[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }
  b_A[r2] = A[r2] / A[r1];
  b_A[r3] /= b_A[r1];
  b_A[r2 + 3] -= b_A[r2] * b_A[r1 + 3];
  b_A[r3 + 3] -= b_A[r3] * b_A[r1 + 3];
  b_A[r2 + 6] -= b_A[r2] * b_A[r1 + 6];
  b_A[r3 + 6] -= b_A[r3] * b_A[r1 + 6];
  if (std::abs(b_A[r3 + 3]) > std::abs(b_A[r2 + 3])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }
  b_A[r3 + 3] /= b_A[r2 + 3];
  b_A[r3 + 6] -= b_A[r3 + 3] * b_A[r2 + 6];
  rtemp = B_size[1];
  Y_size[0] = 3;
  Y_size[1] = B_size[1];
  for (int k{0}; k < rtemp; k++) {
    double d;
    int i;
    i = B_size[0] * k;
    maxval = B_data[r1 + i];
    a21 = B_data[r2 + i] - maxval * b_A[r2];
    d = ((B_data[r3 + i] - maxval * b_A[r3]) - a21 * b_A[r3 + 3]) / b_A[r3 + 6];
    Y_data[3 * k + 2] = d;
    maxval -= d * b_A[r1 + 6];
    a21 -= d * b_A[r2 + 6];
    a21 /= b_A[r2 + 3];
    Y_data[3 * k + 1] = a21;
    maxval -= a21 * b_A[r1 + 3];
    maxval /= b_A[r1];
    Y_data[3 * k] = maxval;
  }
}

//
// Arguments    : const double A[9]
//                const double B[9]
//                double Y[9]
// Return Type  : void
//
void mldivide(const double A[9], const double B[9], double Y[9])
{
  double b_A[9];
  double a21;
  double d;
  double d1;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  double maxval;
  int r1;
  int r2;
  int r3;
  std::copy(&A[0], &A[9], &b_A[0]);
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = std::abs(A[0]);
  a21 = std::abs(A[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }
  if (std::abs(A[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }
  b_A[r2] = A[r2] / A[r1];
  b_A[r3] /= b_A[r1];
  b_A[r2 + 3] -= b_A[r2] * b_A[r1 + 3];
  b_A[r3 + 3] -= b_A[r3] * b_A[r1 + 3];
  b_A[r2 + 6] -= b_A[r2] * b_A[r1 + 6];
  b_A[r3 + 6] -= b_A[r3] * b_A[r1 + 6];
  if (std::abs(b_A[r3 + 3]) > std::abs(b_A[r2 + 3])) {
    int rtemp;
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }
  b_A[r3 + 3] /= b_A[r2 + 3];
  b_A[r3 + 6] -= b_A[r3 + 3] * b_A[r2 + 6];
  maxval = B[r1];
  a21 = B[r2] - maxval * b_A[r2];
  d = b_A[r3 + 3];
  d1 = b_A[r3 + 6];
  d2 = ((B[r3] - maxval * b_A[r3]) - a21 * d) / d1;
  Y[2] = d2;
  d3 = b_A[r1 + 6];
  maxval -= d2 * d3;
  d4 = b_A[r2 + 6];
  a21 -= d2 * d4;
  d5 = b_A[r2 + 3];
  a21 /= d5;
  Y[1] = a21;
  d6 = b_A[r1 + 3];
  maxval -= a21 * d6;
  maxval /= b_A[r1];
  Y[0] = maxval;
  maxval = B[r1 + 3];
  a21 = B[r2 + 3] - maxval * b_A[r2];
  d2 = ((B[r3 + 3] - maxval * b_A[r3]) - a21 * d) / d1;
  Y[5] = d2;
  maxval -= d2 * d3;
  a21 -= d2 * d4;
  a21 /= d5;
  Y[4] = a21;
  maxval -= a21 * d6;
  maxval /= b_A[r1];
  Y[3] = maxval;
  maxval = B[r1 + 6];
  a21 = B[r2 + 6] - maxval * b_A[r2];
  d2 = ((B[r3 + 6] - maxval * b_A[r3]) - a21 * d) / d1;
  Y[8] = d2;
  maxval -= d2 * d3;
  a21 -= d2 * d4;
  a21 /= d5;
  Y[7] = a21;
  maxval -= a21 * d6;
  maxval /= b_A[r1];
  Y[6] = maxval;
}

} // namespace coder

//
// File trailer for mldivide.cpp
//
// [EOF]
//
