//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: odeCosserat_keith.h
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 13-Apr-2023 16:35:29
//

#ifndef ODECOSSERAT_KEITH_H
#define ODECOSSERAT_KEITH_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void b_odeCosserat_keith(const double b_y0[22], const double v[12],
                         double t_data[], int t_size[2], double y[1320]);

void c_odeCosserat_keith(const double b_y0[20], const double v[12],
                         double t_data[], int t_size[2], double y[400]);

void d_odeCosserat_keith(const double b_y0[19], const double ksi[4],
                         double t_data[], int t_size[2], double y[380]);

extern void odeCosserat_keith(const double y0_data[], const int y0_size[2],
                              const double v[12], signed char SegIdx,
                              const double ksi[4], double t_data[],
                              int t_size[2], double y_data[], int y_size[2],
                              double U_data[], int U_size[2]);

#endif
//
// File trailer for odeCosserat_keith.h
//
// [EOF]
//
