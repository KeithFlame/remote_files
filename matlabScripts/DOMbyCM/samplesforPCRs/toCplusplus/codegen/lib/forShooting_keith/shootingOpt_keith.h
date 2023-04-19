//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: shootingOpt_keith.h
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 13-Apr-2023 16:35:29
//

#ifndef SHOOTINGOPT_KEITH_H
#define SHOOTINGOPT_KEITH_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void shootingOpt_keith(double Guess[20], const double qa[12],
                              const double ksi[12], double t0[3],
                              double t1_data[], int t1_size[1],
                              double t2_data[], int t2_size[1],
                              double t3_data[], int t3_size[1], double b_y0[66],
                              double b_y1[2706], double y2[860],
                              double y3[779]);

#endif
//
// File trailer for shootingOpt_keith.h
//
// [EOF]
//
