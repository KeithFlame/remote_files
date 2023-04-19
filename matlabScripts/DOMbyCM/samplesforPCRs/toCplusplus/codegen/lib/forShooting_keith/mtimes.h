//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mtimes.h
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 13-Apr-2023 16:35:29
//

#ifndef MTIMES_H
#define MTIMES_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
void mtimes(const double A_data[], const int A_size[2], const double B_data[],
            const int B_size[2], double C_data[], int C_size[2]);

}
} // namespace internal
} // namespace coder

#endif
//
// File trailer for mtimes.h
//
// [EOF]
//
