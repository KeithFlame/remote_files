//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: skewMatrix_keith.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 13-Apr-2023 16:35:29
//

// Include Files
#include "skewMatrix_keith.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// this function gives the skew-symmetric matrix of the vector p
//
// Arguments    : const double p[3]
//                double T[9]
// Return Type  : void
//
void skewMatrix_keith(const double p[3], double T[9])
{
  //      if(length(p)==3)
  T[0] = 0.0;
  T[3] = -p[2];
  T[6] = p[1];
  T[1] = p[2];
  T[4] = 0.0;
  T[7] = -p[0];
  T[2] = -p[1];
  T[5] = p[0];
  T[8] = 0.0;
  //      elseif(length(p)==6)
  //          R=[0 -p(6) p(5); p(6) 0 -p(4);-p(5) p(4) 0];
  //          T=[R p(1:3);zeros(1,4)];
  //      end
}

//
// File trailer for skewMatrix_keith.cpp
//
// [EOF]
//
