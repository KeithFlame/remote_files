//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: shootingOpt_keith.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 13-Apr-2023 16:35:29
//

// Include Files
#include "shootingOpt_keith.h"
#include "eul2rotm.h"
#include "forShooting_keith.h"
#include "forShooting_keith_data.h"
#include "mrdivide_helper.h"
#include "odeCosserat_keith.h"
#include "rotm2axang.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <stdio.h>

// Function Definitions
//
// Arguments    : double Guess[20]
//                const double qa[12]
//                const double ksi[12]
//                double t0[3]
//                double t1_data[]
//                int t1_size[1]
//                double t2_data[]
//                int t2_size[1]
//                double t3_data[]
//                int t3_size[1]
//                double b_y0[66]
//                double b_y1[2706]
//                double y2[860]
//                double y3[779]
// Return Type  : void
//
void shootingOpt_keith(double Guess[20], const double qa[12],
                       const double ksi[12], double t0[3], double t1_data[],
                       int t1_size[1], double t2_data[], int t2_size[1],
                       double t3_data[], int t3_size[1], double b_y0[66],
                       double b_y1[2706], double y2[860], double y3[779])
{
  static const double b_y[400]{
      1.0E-6, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-6, 0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      1.0E-6, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-6, 0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      1.0E-6, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-6, 0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      1.0E-6, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-6, 0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      1.0E-6, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-6, 0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 1.0E-6};
  static const double dGuess[400]{
      1.0E-9, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-9, 0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-9, 0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 1.0E-9, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      1.0E-9, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-9, 0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-9, 0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 1.0E-9, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      1.0E-9, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-9, 0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-9, 0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 1.0E-9, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      1.0E-9, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-9, 0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-9, 0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 1.0E-9, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      1.0E-9, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-9, 0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 1.0E-9, 0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0, 0.0,    0.0, 0.0,
      0.0,    0.0, 0.0, 1.0E-9};
  static const signed char b[400]{
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  coder::array<char, 2U> b_str;
  coder::array<char, 2U> str;
  double b_y11[1342];
  double y21[1342];
  double y11[1320];
  double c_y12[420];
  double y22[420];
  double J[400];
  double b_dv[400];
  double b_y12[400];
  double y12[400];
  double y13[380];
  double y23[380];
  double t11_data[60];
  double yL11[22];
  double ynew[22];
  double Rsd[20];
  double Rsd_[20];
  double b_ynew[20];
  double yL12[20];
  double b_yL12[19];
  double T10[16];
  double v1[12];
  double v2[12];
  double R11[9];
  double R12_[9];
  double y12__tmp[9];
  double b_dv1[4];
  double c_y[3];
  double d_y[3];
  double e_y[3];
  double f_y[3];
  double g_y[3];
  double absxk;
  double scale;
  double t;
  double y;
  int t11_size[2];
  int nbytes;
  short iter;
  //      dGuess(10,10) = 1;dGuess(11,11) = 1;dGuess(12,12) = 1;
  //  Jacobian damping
  //  residue tolerance
  //      nc=[0 0 0]';mc=[0 0 0]';v=[0 0 0 0]';Guess=[nc;mc;v;nc;mc;v]; %
  //      guessed base force moment strain
  forShooting_keith(Guess, qa, ksi, Rsd, t0, b_y0, t1_data, t1_size, b_y1,
                    t2_data, t2_size, y2, t3_data, t3_size, y3);
  iter = 0;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (nbytes = 0; nbytes < 20; nbytes++) {
    absxk = std::abs(Rsd[nbytes]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }
  y = scale * std::sqrt(y);
  nbytes =
      (int)snprintf(nullptr, 0,
                    "-> Now the iteration is %04d. and error is %06f", 0, y) +
      1;
  str.set_size(1, nbytes);
  snprintf(&str[0], (size_t)nbytes,
           "-> Now the iteration is %04d. and error is %06f", 0, y);
  int exitg1;
  do {
    exitg1 = 0;
    y = 0.0;
    scale = 3.3121686421112381E-170;
    for (nbytes = 0; nbytes < 20; nbytes++) {
      absxk = std::abs(Rsd[nbytes]);
      if (absxk > scale) {
        t = scale / absxk;
        y = y * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }
    y = scale * std::sqrt(y);
    if (y > 2.0E-5) {
      double d;
      int R12__tmp;
      int b_i;
      for (int i{0}; i < 20; i++) {
        double d1;
        double d2;
        double d3;
        int b_R12__tmp;
        //  finite differencing for Jacobian of initial guess
        for (b_i = 0; b_i < 20; b_i++) {
          Rsd_[b_i] = Guess[b_i] + dGuess[b_i + 20 * i];
        }
        //  do an IVP shooting
        //  arm1
        //  base internal force
        //  base internal load
        std::memset(&v1[0], 0, 12U * sizeof(double));
        v1[2] = Rsd_[6];
        v1[5] = Rsd_[7];
        v1[8] = Rsd_[8];
        v1[11] = Rsd_[9];
        //  rod elongation strain
        //  Idx = int8(0);
        for (b_i = 0; b_i < 16; b_i++) {
          T10[b_i] = iv[b_i];
        }
        T10[12] = -0.028730000000000002;
        T10[13] = 0.0668106;
        T10[14] = 0.10500040000000001;
        T10[8] = 0.36668905632841847;
        T10[9] = -0.92977225136123109;
        T10[10] = -0.032599027096554239;
        T10[0] = 0.88783816286542694;
        T10[1] = 0.33940218069306965;
        T10[2] = 0.3065676906238029;
        scale = 3.3121686421112381E-170;
        absxk = std::abs(T10[0]);
        if (absxk > 3.3121686421112381E-170) {
          y = 1.0;
          scale = absxk;
        } else {
          t = absxk / 3.3121686421112381E-170;
          y = t * t;
        }
        absxk = std::abs(T10[1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
        absxk = std::abs(T10[2]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
        y = scale * std::sqrt(y);
        T10[0] /= y;
        T10[1] /= y;
        T10[2] /= y;
        T10[4] = T10[2] * -0.92977225136123109 - T10[1] * -0.032599027096554239;
        T10[5] = T10[0] * -0.032599027096554239 - T10[2] * 0.36668905632841847;
        T10[6] = T10[1] * 0.36668905632841847 - T10[0] * -0.92977225136123109;
        c_y[0] = qa[0];
        c_y[1] = 0.0;
        c_y[2] = 0.0;
        coder::eul2rotm(c_y, R11);
        for (b_i = 0; b_i < 3; b_i++) {
          d = T10[b_i];
          absxk = T10[b_i + 4];
          y = T10[b_i + 8];
          for (R12__tmp = 0; R12__tmp < 3; R12__tmp++) {
            R12_[b_i + 3 * R12__tmp] =
                (d * R11[3 * R12__tmp] + absxk * R11[3 * R12__tmp + 1]) +
                y * R11[3 * R12__tmp + 2];
          }
        }
        //  p10=[0 0 0]';R10=eul2rotm([qa(1) 0 0]);
        for (b_i = 0; b_i < 3; b_i++) {
          nbytes = b_i << 2;
          T10[nbytes] = R12_[3 * b_i];
          T10[nbytes + 1] = R12_[3 * b_i + 1];
          T10[nbytes + 2] = R12_[3 * b_i + 2];
          ynew[b_i] = T10[b_i + 12];
        }
        ynew[3] = T10[0];
        ynew[6] = T10[4];
        ynew[9] = T10[8];
        ynew[12] = Rsd_[0];
        ynew[15] = Rsd_[3];
        ynew[4] = T10[1];
        ynew[7] = T10[5];
        ynew[10] = T10[9];
        ynew[13] = Rsd_[1];
        ynew[16] = Rsd_[4];
        ynew[5] = T10[2];
        ynew[8] = T10[6];
        ynew[11] = T10[10];
        ynew[14] = Rsd_[2];
        ynew[17] = Rsd_[5];
        ynew[18] = 0.0;
        ynew[19] = 0.0;
        ynew[20] = 0.0;
        ynew[21] = 0.0;
        b_odeCosserat_keith(ynew, v1, t11_data, t11_size, y11);
        //  first seg y=p,R,n,m,q for each row
        R11[0] = y11[239];
        R11[3] = y11[419];
        R11[6] = y11[599];
        R11[1] = y11[299];
        R11[4] = y11[479];
        R11[7] = y11[659];
        R11[2] = y11[359];
        R11[5] = y11[539];
        R11[8] = y11[719];
        //  R1e
        for (b_i = 0; b_i < 22; b_i++) {
          ynew[b_i] = y11[60 * b_i + 59];
        }
        for (b_i = 0; b_i < 3; b_i++) {
          ynew[b_i] =
              y11[60 * b_i + 59] +
              ((0.0 * R11[b_i] + 0.0 * R11[b_i + 3]) + 0.01 * R11[b_i + 6]);
        }
        //  move from 1e to 2b
        for (b_i = 0; b_i < 22; b_i++) {
          d = ynew[b_i];
          b_y11[61 * b_i + 60] = d;
          std::copy(&y11[b_i * 60], &y11[static_cast<int>(b_i * 60 + 60U)],
                    &b_y11[b_i * 61]);
          yL11[b_i] = d;
        }
        for (b_i = 0; b_i < 3; b_i++) {
          d = 0.0;
          absxk = 0.0;
          y = 0.0;
          scale = 0.0;
          t = 0.0;
          for (R12__tmp = 0; R12__tmp < 3; R12__tmp++) {
            d1 = (R11[b_i] * static_cast<double>(iv1[3 * R12__tmp]) +
                  R11[b_i + 3] * static_cast<double>(iv1[3 * R12__tmp + 1])) +
                 R11[b_i + 6] * static_cast<double>(iv1[3 * R12__tmp + 2]);
            d2 = R11[b_i + 3 * R12__tmp];
            d += d2 * dv[R12__tmp];
            absxk += d2 * dv1[R12__tmp];
            y += d1 * v1[R12__tmp];
            scale += d2 * dv2[R12__tmp];
            t += d1 * v1[R12__tmp + 3];
          }
          g_y[b_i] = t;
          c_y[b_i] = scale;
          e_y[b_i] = y;
          d_y[b_i] = absxk;
          f_y[b_i] = d;
        }
        //  ? start of 2nd seg
        absxk = d_y[1] * e_y[2] - e_y[1] * d_y[2];
        y = e_y[0] * d_y[2] - d_y[0] * e_y[2];
        scale = d_y[0] * e_y[1] - e_y[0] * d_y[1];
        std::copy(&yL11[0], &yL11[15], &yL12[0]);
        yL12[15] = ((f_y[1] * b_y11[914] - f_y[2] * b_y11[853]) + yL11[15]) +
                   2.0 * (absxk + (c_y[1] * g_y[2] - g_y[1] * c_y[2]));
        yL12[16] = ((f_y[2] * b_y11[792] - f_y[0] * b_y11[914]) + yL11[16]) +
                   2.0 * (y + (g_y[0] * c_y[2] - c_y[0] * g_y[2]));
        yL12[17] = ((f_y[0] * b_y11[853] - f_y[1] * b_y11[792]) + yL11[17]) +
                   2.0 * (scale + (c_y[0] * g_y[1] - g_y[0] * c_y[1]));
        yL12[18] = yL11[20];
        yL12[19] = yL11[21];
        c_odeCosserat_keith(yL12, v1, b_ynew, t11_size, y12);
        //  second seg
        R11[0] = y12[79];
        R11[3] = y12[139];
        R11[6] = y12[199];
        R11[1] = y12[99];
        R11[4] = y12[159];
        R11[7] = y12[219];
        R11[2] = y12[119];
        R11[5] = y12[179];
        R11[8] = y12[239];
        //  R2e
        for (b_i = 0; b_i < 20; b_i++) {
          b_ynew[b_i] = y12[20 * b_i + 19];
        }
        for (b_i = 0; b_i < 3; b_i++) {
          b_ynew[b_i] =
              y12[20 * b_i + 19] +
              ((0.0 * R11[b_i] + 0.0 * R11[b_i + 3]) + 0.015 * R11[b_i + 6]);
        }
        //  move from 2e to g
        for (b_i = 0; b_i < 20; b_i++) {
          d = b_ynew[b_i];
          c_y12[21 * b_i + 20] = d;
          std::copy(&y12[b_i * 20], &y12[static_cast<int>(b_i * 20 + 20U)],
                    &c_y12[b_i * 21]);
          yL12[b_i] = d;
        }
        //  elongation seg1
        //  elongation seg2
        //  R12_ = R12 * eul2rotm([-1.5850    0.8312    1.8071]);
        for (b_i = 0; b_i < 3; b_i++) {
          d = 0.0;
          absxk = 0.0;
          y = 0.0;
          scale = 0.0;
          t = 0.0;
          for (R12__tmp = 0; R12__tmp < 3; R12__tmp++) {
            d1 = R11[b_i];
            d2 = d1 * dv3[3 * R12__tmp];
            d3 = d1 * static_cast<double>(iv2[3 * R12__tmp]);
            d1 = R11[b_i + 3];
            nbytes = 3 * R12__tmp + 1;
            d2 += d1 * dv3[nbytes];
            d3 += d1 * static_cast<double>(iv2[nbytes]);
            d1 = R11[b_i + 6];
            nbytes = 3 * R12__tmp + 2;
            d2 += d1 * dv3[nbytes];
            d3 += d1 * static_cast<double>(iv2[nbytes]);
            b_R12__tmp = b_i + 3 * R12__tmp;
            R12_[b_R12__tmp] = d2;
            d1 = R11[b_R12__tmp];
            d += d1 * dv4[R12__tmp];
            absxk += d1 * dv5[R12__tmp];
            y += d3 * v1[R12__tmp + 6];
            scale += d1 * dv6[R12__tmp];
            t += d3 * v1[R12__tmp + 9];
          }
          g_y[b_i] = t;
          c_y[b_i] = scale;
          e_y[b_i] = y;
          d_y[b_i] = absxk;
          f_y[b_i] = d;
        }
        //  ? start of 3nd seg
        absxk = d_y[1] * e_y[2] - e_y[1] * d_y[2];
        y = e_y[0] * d_y[2] - d_y[0] * e_y[2];
        scale = d_y[0] * e_y[1] - e_y[0] * d_y[1];
        b_yL12[12] = (f_y[1] * c_y12[314] - f_y[2] * c_y12[293]) + yL12[12];
        b_yL12[13] = (f_y[2] * c_y12[272] - f_y[0] * c_y12[314]) + yL12[13];
        b_yL12[14] = (f_y[0] * c_y12[293] - f_y[1] * c_y12[272]) + yL12[14];
        b_yL12[0] = yL12[0];
        b_yL12[3] = R12_[0];
        b_yL12[6] = R12_[3];
        b_yL12[9] = R12_[6];
        b_yL12[15] =
            yL12[15] + 2.0 * (absxk + (c_y[1] * g_y[2] - g_y[1] * c_y[2]));
        b_yL12[1] = yL12[1];
        b_yL12[4] = R12_[1];
        b_yL12[7] = R12_[4];
        b_yL12[10] = R12_[7];
        b_yL12[16] = yL12[16] + 2.0 * (y + (g_y[0] * c_y[2] - c_y[0] * g_y[2]));
        b_yL12[2] = yL12[2];
        b_yL12[5] = R12_[2];
        b_yL12[8] = R12_[5];
        b_yL12[11] = R12_[8];
        b_yL12[17] =
            yL12[17] + 2.0 * (scale + (c_y[0] * g_y[1] - g_y[0] * c_y[1]));
        b_yL12[18] = 0.0;
        d_odeCosserat_keith(b_yL12, *(double(*)[4]) & ksi[0], b_ynew, t11_size,
                            y13);
        //  second seg
        //  R2e
        //  arm2
        //  base internal force
        //  base internal load
        std::memset(&v2[0], 0, 12U * sizeof(double));
        v2[2] = Rsd_[16];
        v2[5] = Rsd_[17];
        v2[8] = Rsd_[18];
        v2[11] = Rsd_[19];
        //  rod elongation strain
        for (b_i = 0; b_i < 16; b_i++) {
          T10[b_i] = iv[b_i];
        }
        T10[12] = -0.134715;
        T10[13] = 0.0487399;
        T10[14] = 0.107036;
        T10[8] = 0.80809285245032991;
        T10[9] = -0.58179485404727371;
        T10[10] = -0.092199184501819589;
        T10[0] = 0.57298033201137633;
        T10[1] = 0.74058665953996561;
        T10[2] = 0.348721125575285;
        scale = 3.3121686421112381E-170;
        absxk = std::abs(T10[0]);
        if (absxk > 3.3121686421112381E-170) {
          y = 1.0;
          scale = absxk;
        } else {
          t = absxk / 3.3121686421112381E-170;
          y = t * t;
        }
        absxk = std::abs(T10[1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
        absxk = std::abs(T10[2]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
        y = scale * std::sqrt(y);
        T10[0] /= y;
        T10[1] /= y;
        T10[2] /= y;
        T10[4] = T10[2] * -0.58179485404727371 - T10[1] * -0.092199184501819589;
        T10[5] = T10[0] * -0.092199184501819589 - T10[2] * 0.80809285245032991;
        T10[6] = T10[1] * 0.80809285245032991 - T10[0] * -0.58179485404727371;
        c_y[0] = qa[6];
        c_y[1] = 0.0;
        c_y[2] = 0.0;
        coder::eul2rotm(c_y, R11);
        for (b_i = 0; b_i < 3; b_i++) {
          d = T10[b_i];
          absxk = T10[b_i + 4];
          y = T10[b_i + 8];
          for (R12__tmp = 0; R12__tmp < 3; R12__tmp++) {
            R12_[b_i + 3 * R12__tmp] =
                (d * R11[3 * R12__tmp] + absxk * R11[3 * R12__tmp + 1]) +
                y * R11[3 * R12__tmp + 2];
          }
        }
        //  p20=[-0.02 0 0]';R20=eul2rotm([qa(7) 0 0]);
        for (b_i = 0; b_i < 3; b_i++) {
          nbytes = b_i << 2;
          T10[nbytes] = R12_[3 * b_i];
          T10[nbytes + 1] = R12_[3 * b_i + 1];
          T10[nbytes + 2] = R12_[3 * b_i + 2];
          ynew[b_i] = T10[b_i + 12];
        }
        ynew[3] = T10[0];
        ynew[6] = T10[4];
        ynew[9] = T10[8];
        ynew[12] = Rsd_[10];
        ynew[15] = Rsd_[13];
        ynew[4] = T10[1];
        ynew[7] = T10[5];
        ynew[10] = T10[9];
        ynew[13] = Rsd_[11];
        ynew[16] = Rsd_[14];
        ynew[5] = T10[2];
        ynew[8] = T10[6];
        ynew[11] = T10[10];
        ynew[14] = Rsd_[12];
        ynew[17] = Rsd_[15];
        ynew[18] = 0.0;
        ynew[19] = 0.0;
        ynew[20] = 0.0;
        ynew[21] = 0.0;
        b_odeCosserat_keith(ynew, v2, t11_data, t11_size, y11);
        //  first seg y=p,R,n,m,q for each row
        R11[0] = y11[239];
        R11[3] = y11[419];
        R11[6] = y11[599];
        R11[1] = y11[299];
        R11[4] = y11[479];
        R11[7] = y11[659];
        R11[2] = y11[359];
        R11[5] = y11[539];
        R11[8] = y11[719];
        //  R1e
        for (b_i = 0; b_i < 22; b_i++) {
          ynew[b_i] = y11[60 * b_i + 59];
        }
        for (b_i = 0; b_i < 3; b_i++) {
          ynew[b_i] =
              y11[60 * b_i + 59] +
              ((0.0 * R11[b_i] + 0.0 * R11[b_i + 3]) + 0.01 * R11[b_i + 6]);
        }
        //  move from 1e to 2b
        for (b_i = 0; b_i < 22; b_i++) {
          d = ynew[b_i];
          y21[61 * b_i + 60] = d;
          std::copy(&y11[b_i * 60], &y11[static_cast<int>(b_i * 60 + 60U)],
                    &y21[b_i * 61]);
          yL11[b_i] = d;
        }
        for (b_i = 0; b_i < 3; b_i++) {
          d = 0.0;
          absxk = 0.0;
          y = 0.0;
          scale = 0.0;
          t = 0.0;
          for (R12__tmp = 0; R12__tmp < 3; R12__tmp++) {
            d1 = (R11[b_i] * static_cast<double>(iv1[3 * R12__tmp]) +
                  R11[b_i + 3] * static_cast<double>(iv1[3 * R12__tmp + 1])) +
                 R11[b_i + 6] * static_cast<double>(iv1[3 * R12__tmp + 2]);
            d2 = R11[b_i + 3 * R12__tmp];
            d += d2 * dv[R12__tmp];
            absxk += d2 * dv1[R12__tmp];
            y += d1 * v2[R12__tmp];
            scale += d2 * dv2[R12__tmp];
            t += d1 * v2[R12__tmp + 3];
          }
          g_y[b_i] = t;
          c_y[b_i] = scale;
          e_y[b_i] = y;
          d_y[b_i] = absxk;
          f_y[b_i] = d;
        }
        //  ? start of 2nd seg
        absxk = d_y[1] * e_y[2] - e_y[1] * d_y[2];
        y = e_y[0] * d_y[2] - d_y[0] * e_y[2];
        scale = d_y[0] * e_y[1] - e_y[0] * d_y[1];
        std::copy(&yL11[0], &yL11[15], &yL12[0]);
        yL12[15] = ((f_y[1] * y21[914] - f_y[2] * y21[853]) + yL11[15]) +
                   2.0 * (absxk + (c_y[1] * g_y[2] - g_y[1] * c_y[2]));
        yL12[16] = ((f_y[2] * y21[792] - f_y[0] * y21[914]) + yL11[16]) +
                   2.0 * (y + (g_y[0] * c_y[2] - c_y[0] * g_y[2]));
        yL12[17] = ((f_y[0] * y21[853] - f_y[1] * y21[792]) + yL11[17]) +
                   2.0 * (scale + (c_y[0] * g_y[1] - g_y[0] * c_y[1]));
        yL12[18] = yL11[20];
        yL12[19] = yL11[21];
        c_odeCosserat_keith(yL12, v2, b_ynew, t11_size, y12);
        //  second seg
        R11[0] = y12[79];
        R11[3] = y12[139];
        R11[6] = y12[199];
        R11[1] = y12[99];
        R11[4] = y12[159];
        R11[7] = y12[219];
        R11[2] = y12[119];
        R11[5] = y12[179];
        R11[8] = y12[239];
        //  R2e
        for (b_i = 0; b_i < 20; b_i++) {
          b_ynew[b_i] = y12[20 * b_i + 19];
        }
        for (b_i = 0; b_i < 3; b_i++) {
          b_ynew[b_i] =
              y12[20 * b_i + 19] +
              ((0.0 * R11[b_i] + 0.0 * R11[b_i + 3]) + 0.015 * R11[b_i + 6]);
        }
        //  move from 2e to g
        for (b_i = 0; b_i < 20; b_i++) {
          d = b_ynew[b_i];
          y22[21 * b_i + 20] = d;
          std::copy(&y12[b_i * 20], &y12[static_cast<int>(b_i * 20 + 20U)],
                    &y22[b_i * 21]);
          Rsd_[b_i] = d;
        }
        //  elongation seg1
        //  elongation seg2
        //  R22_ = R22 * eul2rotm([1.9264   -0.4921    0.7374]);
        for (b_i = 0; b_i < 3; b_i++) {
          d = 0.0;
          absxk = 0.0;
          y = 0.0;
          scale = 0.0;
          t = 0.0;
          for (R12__tmp = 0; R12__tmp < 3; R12__tmp++) {
            d1 = R11[b_i];
            d2 = d1 * dv7[3 * R12__tmp];
            d3 = d1 * static_cast<double>(iv2[3 * R12__tmp]);
            d1 = R11[b_i + 3];
            nbytes = 3 * R12__tmp + 1;
            d2 += d1 * dv7[nbytes];
            d3 += d1 * static_cast<double>(iv2[nbytes]);
            d1 = R11[b_i + 6];
            nbytes = 3 * R12__tmp + 2;
            d2 += d1 * dv7[nbytes];
            d3 += d1 * static_cast<double>(iv2[nbytes]);
            b_R12__tmp = b_i + 3 * R12__tmp;
            R12_[b_R12__tmp] = d2;
            d1 = R11[b_R12__tmp];
            d += d1 * dv4[R12__tmp];
            absxk += d1 * dv5[R12__tmp];
            y += d3 * v2[R12__tmp + 6];
            scale += d1 * dv6[R12__tmp];
            t += d3 * v2[R12__tmp + 9];
          }
          g_y[b_i] = t;
          c_y[b_i] = scale;
          e_y[b_i] = y;
          d_y[b_i] = absxk;
          f_y[b_i] = d;
        }
        //  ? start of 2nd seg
        absxk = d_y[1] * e_y[2] - e_y[1] * d_y[2];
        y = e_y[0] * d_y[2] - d_y[0] * e_y[2];
        scale = d_y[0] * e_y[1] - e_y[0] * d_y[1];
        b_yL12[0] = Rsd_[0];
        b_yL12[3] = R12_[0];
        b_yL12[6] = R12_[3];
        b_yL12[9] = R12_[6];
        b_yL12[12] = y22[272];
        b_yL12[15] = ((f_y[1] * y22[314] - f_y[2] * y22[293]) + Rsd_[15]) +
                     2.0 * (absxk + (c_y[1] * g_y[2] - g_y[1] * c_y[2]));
        b_yL12[1] = Rsd_[1];
        b_yL12[4] = R12_[1];
        b_yL12[7] = R12_[4];
        b_yL12[10] = R12_[7];
        b_yL12[13] = y22[293];
        b_yL12[16] = ((f_y[2] * y22[272] - f_y[0] * y22[314]) + Rsd_[16]) +
                     2.0 * (y + (g_y[0] * c_y[2] - c_y[0] * g_y[2]));
        b_yL12[2] = Rsd_[2];
        b_yL12[5] = R12_[2];
        b_yL12[8] = R12_[5];
        b_yL12[11] = R12_[8];
        b_yL12[14] = y22[314];
        b_yL12[17] = ((f_y[0] * y22[293] - f_y[1] * y22[272]) + Rsd_[17]) +
                     2.0 * (scale + (c_y[0] * g_y[1] - g_y[0] * c_y[1]));
        b_yL12[18] = 0.0;
        d_odeCosserat_keith(b_yL12, *(double(*)[4]) & ksi[4], b_ynew, t11_size,
                            y23);
        //  second seg
        //  R2e
        std::memset(&Rsd_[0], 0, 20U * sizeof(double));
        //  boundary conditions
        for (b_i = 0; b_i < 3; b_i++) {
          nbytes = 20 * (b_i + 12) + 19;
          Rsd_[b_i] = (0.0 - y13[nbytes]) - y23[nbytes];
          nbytes = 20 * (b_i + 15) + 19;
          Rsd_[b_i + 3] = (0.0 - y13[nbytes]) - y23[nbytes];
          nbytes = 20 * b_i + 19;
          Rsd_[b_i + 6] = y13[nbytes] - y23[nbytes];
          b_R12__tmp = 20 * (b_i + 3) + 19;
          d = y13[b_R12__tmp];
          nbytes = 20 * (b_i + 6) + 19;
          absxk = y13[nbytes];
          R12__tmp = 20 * (b_i + 9) + 19;
          y = y13[R12__tmp];
          y12__tmp[3 * b_i] = y23[b_R12__tmp];
          y12__tmp[3 * b_i + 1] = y23[nbytes];
          y12__tmp[3 * b_i + 2] = y23[R12__tmp];
          for (R12__tmp = 0; R12__tmp < 3; R12__tmp++) {
            R11[b_i + 3 * R12__tmp] =
                (d * dv8[3 * R12__tmp] + absxk * dv8[3 * R12__tmp + 1]) +
                y * dv8[3 * R12__tmp + 2];
          }
        }
        for (b_i = 0; b_i < 3; b_i++) {
          d = y12__tmp[b_i];
          absxk = y12__tmp[b_i + 3];
          y = y12__tmp[b_i + 6];
          for (R12__tmp = 0; R12__tmp < 3; R12__tmp++) {
            R12_[b_i + 3 * R12__tmp] =
                (d * R11[3 * R12__tmp] + absxk * R11[3 * R12__tmp + 1]) +
                y * R11[3 * R12__tmp + 2];
          }
        }
        coder::rotm2axang(R12_, b_dv1);
        Rsd_[9] = b_dv1[0] * b_dv1[3];
        Rsd_[10] = b_dv1[1] * b_dv1[3];
        Rsd_[11] = b_dv1[2] * b_dv1[3];
        //  % % % % Rsd(1:3)=Fe-yL12(13:15)-yL22(13:15); % boundary conditions
        //  % % % %
        //  Rsd(4:6)=Me-yL12(16:18)-2*(cross(R12*MBP1.r21,R12*MBP1.Ke1*v1(:,3))+...
        //  % % % %     cross(R12*MBP1.r22,R12*MBP1.Ke1*v1(:,4)))...
        //  % % % % -yL22(16:18)-2*(cross(R22*MBP2.r21,R22*MBP2.Ke1*v2(:,3))+...
        //  % % % %     cross(R22*MBP2.r22,R22*MBP2.Ke1*v2(:,4)));
        //  % % % % Rsd(7:9)=yL12(1:3)-yL22(1:3);
        //  % % % % axang = rotm2axang(R23'*(R13*eul2rotm([0 pi 0])))';
        //  % % % % Rsd(10:12)=axang(1:3)*axang(4)*0;
        Rsd_[12] = b_y11[1158] - (qa[2] + 0.7 * v1[2]);
        Rsd_[13] = b_y11[1219] - (qa[3] + 0.7 * v1[5]);
        //  path length
        Rsd_[14] = c_y12[398] - (qa[4] + 0.73 * v1[8]);
        Rsd_[15] = c_y12[419] - (qa[5] + 0.73 * v1[11]);
        Rsd_[16] = y21[1158] - (qa[8] + 0.7 * v2[2]);
        Rsd_[17] = y21[1219] - (qa[9] + 0.7 * v2[5]);
        //  path length
        Rsd_[18] = y22[398] - (qa[10] + 0.73 * v2[8]);
        Rsd_[19] = y22[419] - (qa[11] + 0.73 * v2[11]);
        y = 0.0;
        scale = 3.3121686421112381E-170;
        for (nbytes = 0; nbytes < 20; nbytes++) {
          d = dGuess[nbytes + 20 * i];
          if (d > scale) {
            t = scale / d;
            y = y * t * t + 1.0;
            scale = d;
          } else {
            t = d / scale;
            y += t * t;
          }
        }
        y = scale * std::sqrt(y);
        for (b_i = 0; b_i < 20; b_i++) {
          J[b_i + 20 * i] = (Rsd_[b_i] - Rsd[b_i]) / y;
        }
      }
      for (b_i = 0; b_i < 20; b_i++) {
        for (R12__tmp = 0; R12__tmp < 20; R12__tmp++) {
          y12[R12__tmp + 20 * b_i] = J[b_i + 20 * R12__tmp];
        }
      }
      for (b_i = 0; b_i < 400; b_i++) {
        b_dv[b_i] = b[b_i];
      }
      for (b_i = 0; b_i < 20; b_i++) {
        for (R12__tmp = 0; R12__tmp < 20; R12__tmp++) {
          d = 0.0;
          for (nbytes = 0; nbytes < 20; nbytes++) {
            d += y12[b_i + 20 * nbytes] * J[nbytes + 20 * R12__tmp];
          }
          nbytes = b_i + 20 * R12__tmp;
          b_y12[nbytes] = d + b_y[nbytes];
        }
      }
      coder::internal::mrdiv(b_dv, b_y12);
      for (b_i = 0; b_i < 20; b_i++) {
        d = 0.0;
        for (R12__tmp = 0; R12__tmp < 20; R12__tmp++) {
          absxk = 0.0;
          for (nbytes = 0; nbytes < 20; nbytes++) {
            absxk += b_dv[b_i + 20 * nbytes] * y12[nbytes + 20 * R12__tmp];
          }
          d += absxk * Rsd[R12__tmp];
        }
        Guess[b_i] -= d;
      }
      //  update guess
      forShooting_keith(Guess, qa, ksi, Rsd, t0, b_y0, t1_data, t1_size, b_y1,
                        t2_data, t2_size, y2, t3_data, t3_size, y3);
      //  number IVP of equal to number of guessed values
      b_i = iter + 1;
      if (iter + 1 > 32767) {
        b_i = 32767;
      }
      iter = static_cast<short>(b_i);
      y = 0.0;
      scale = 3.3121686421112381E-170;
      for (nbytes = 0; nbytes < 20; nbytes++) {
        absxk = std::abs(Rsd[nbytes]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }
      y = scale * std::sqrt(y);
      nbytes = (int)snprintf(nullptr, 0,
                             "-> Now the iteration is %04d. and error is %06f",
                             static_cast<short>(b_i), y) +
               1;
      b_str.set_size(1, nbytes);
      snprintf(&b_str[0], (size_t)nbytes,
               "-> Now the iteration is %04d. and error is %06f",
               static_cast<short>(b_i), y);
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

//
// File trailer for shootingOpt_keith.cpp
//
// [EOF]
//
