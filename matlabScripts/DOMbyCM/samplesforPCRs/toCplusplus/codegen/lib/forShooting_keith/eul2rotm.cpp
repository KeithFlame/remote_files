//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eul2rotm.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 13-Apr-2023 16:35:29
//

// Include Files
#include "eul2rotm.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const double eul[3]
//                double R[9]
// Return Type  : void
//
namespace coder {
void eul2rotm(const double eul[3], double R[9])
{
  double R_tmp;
  double b_R_tmp;
  double ct_idx_0;
  double ct_idx_1;
  double ct_idx_2;
  double st_idx_0;
  double st_idx_1;
  double st_idx_2;
  ct_idx_0 = std::cos(eul[0]);
  st_idx_0 = std::sin(eul[0]);
  ct_idx_1 = std::cos(eul[1]);
  st_idx_1 = std::sin(eul[1]);
  ct_idx_2 = std::cos(eul[2]);
  st_idx_2 = std::sin(eul[2]);
  R[0] = ct_idx_0 * ct_idx_1;
  R_tmp = st_idx_1 * st_idx_2;
  R[3] = R_tmp * ct_idx_0 - st_idx_0 * ct_idx_2;
  b_R_tmp = st_idx_1 * ct_idx_2;
  R[6] = b_R_tmp * ct_idx_0 + st_idx_0 * st_idx_2;
  R[1] = st_idx_0 * ct_idx_1;
  R[4] = R_tmp * st_idx_0 + ct_idx_0 * ct_idx_2;
  R[7] = b_R_tmp * st_idx_0 - ct_idx_0 * st_idx_2;
  R[2] = -st_idx_1;
  R[5] = ct_idx_1 * st_idx_2;
  R[8] = ct_idx_1 * ct_idx_2;
}

} // namespace coder

//
// File trailer for eul2rotm.cpp
//
// [EOF]
//
