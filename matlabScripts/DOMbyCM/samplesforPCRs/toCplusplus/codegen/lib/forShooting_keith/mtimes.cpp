//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mtimes.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 13-Apr-2023 16:35:29
//

// Include Files
#include "mtimes.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : const double A_data[]
//                const int A_size[2]
//                const double B_data[]
//                const int B_size[2]
//                double C_data[]
//                int C_size[2]
// Return Type  : void
//
namespace coder {
namespace internal {
namespace blas {
void mtimes(const double A_data[], const int A_size[2], const double B_data[],
            const int B_size[2], double C_data[], int C_size[2])
{
  int inner;
  int nc;
  inner = A_size[1];
  nc = B_size[1];
  C_size[0] = 3;
  C_size[1] = B_size[1];
  for (int j{0}; j < nc; j++) {
    int boffset;
    int coffset;
    coffset = j * 3;
    boffset = j * B_size[0];
    C_data[coffset] = 0.0;
    C_data[coffset + 1] = 0.0;
    C_data[coffset + 2] = 0.0;
    for (int k{0}; k < inner; k++) {
      double bkj;
      int aoffset;
      aoffset = k * 3;
      bkj = B_data[boffset + k];
      C_data[coffset] += A_data[aoffset] * bkj;
      C_data[coffset + 1] += A_data[aoffset + 1] * bkj;
      C_data[coffset + 2] += A_data[aoffset + 2] * bkj;
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

//
// File trailer for mtimes.cpp
//
// [EOF]
//
