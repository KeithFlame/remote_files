//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: forShooting_keith.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 13-Apr-2023 16:35:29
//

// Include Files
#include "forShooting_keith.h"
#include "eul2rotm.h"
#include "forShooting_keith_data.h"
#include "odeCosserat_keith.h"
#include "rotm2axang.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
//
// do an IVP shooting
//
// Arguments    : const double Guess[20]
//                const double qa[12]
//                const double ksi[12]
//                double Rsd[20]
//                double t0[3]
//                double b_y0[66]
//                double t1_data[]
//                int t1_size[1]
//                double b_y1[2706]
//                double t2_data[]
//                int t2_size[1]
//                double y2[860]
//                double t3_data[]
//                int t3_size[1]
//                double y3[779]
// Return Type  : void
//
void forShooting_keith(const double Guess[20], const double qa[12],
                       const double ksi[12], double Rsd[20], double t0[3],
                       double b_y0[66], double t1_data[], int t1_size[1],
                       double b_y1[2706], double t2_data[], int t2_size[1],
                       double y2[860], double t3_data[], int t3_size[1],
                       double y3[779])
{
  double b_y11[1342];
  double y21[1342];
  double y11[1320];
  double b_y12[420];
  double y22[420];
  double y12[400];
  double y13[380];
  double y23[380];
  double t11_data[60];
  double t21_data[60];
  double y10[22];
  double y20[22];
  double ynew[22];
  double b_ynew[20];
  double t12_data[20];
  double t13_data[20];
  double t22_data[20];
  double c_y12[19];
  double T10[16];
  double v1[12];
  double R11[9];
  double R12_[9];
  double y12__tmp[9];
  double b_dv[4];
  double b_T10[3];
  double b_y[3];
  double c_y[3];
  double d_y[3];
  double e_y[3];
  double absxk;
  double d;
  double d1;
  double d2;
  double d3;
  double scale;
  double t;
  double y;
  int t11_size[2];
  int R12__tmp;
  int T10_tmp;
  int i;
  int i1;
  int y1_tmp;
  signed char b_input_sizes_idx_0;
  signed char c_input_sizes_idx_0;
  signed char d_input_sizes_idx_0;
  signed char input_sizes_idx_0;
  //  arm1
  //  base internal force
  //  base internal load
  std::memset(&v1[0], 0, 12U * sizeof(double));
  v1[2] = Guess[6];
  v1[5] = Guess[7];
  v1[8] = Guess[8];
  v1[11] = Guess[9];
  //  rod elongation strain
  //  Idx = int8(0);
  for (i = 0; i < 16; i++) {
    T10[i] = iv[i];
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
  b_T10[0] = qa[0];
  b_T10[1] = 0.0;
  b_T10[2] = 0.0;
  coder::eul2rotm(b_T10, R11);
  for (i = 0; i < 3; i++) {
    scale = T10[i];
    absxk = T10[i + 4];
    t = T10[i + 8];
    for (i1 = 0; i1 < 3; i1++) {
      R12_[i + 3 * i1] =
          (scale * R11[3 * i1] + absxk * R11[3 * i1 + 1]) + t * R11[3 * i1 + 2];
    }
  }
  //  p10=[0 0 0]';R10=eul2rotm([qa(1) 0 0]);
  for (i = 0; i < 3; i++) {
    T10_tmp = i << 2;
    T10[T10_tmp] = R12_[3 * i];
    T10[T10_tmp + 1] = R12_[3 * i + 1];
    T10[T10_tmp + 2] = R12_[3 * i + 2];
    y10[i] = T10[i + 12];
  }
  y10[3] = T10[0];
  y10[6] = T10[4];
  y10[9] = T10[8];
  y10[12] = Guess[0];
  y10[15] = Guess[3];
  y10[4] = T10[1];
  y10[7] = T10[5];
  y10[10] = T10[9];
  y10[13] = Guess[1];
  y10[16] = Guess[4];
  y10[5] = T10[2];
  y10[8] = T10[6];
  y10[11] = T10[10];
  y10[14] = Guess[2];
  y10[17] = Guess[5];
  y10[18] = 0.0;
  y10[19] = 0.0;
  y10[20] = 0.0;
  y10[21] = 0.0;
  b_odeCosserat_keith(y10, v1, t11_data, t11_size, y11);
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
  for (i = 0; i < 22; i++) {
    ynew[i] = y11[60 * i + 59];
  }
  for (i = 0; i < 3; i++) {
    ynew[i] = y11[60 * i + 59] +
              ((0.0 * R11[i] + 0.0 * R11[i + 3]) + 0.01 * R11[i + 6]);
  }
  //  move from 1e to 2b
  for (i = 0; i < 22; i++) {
    std::copy(&y11[i * 60], &y11[static_cast<int>(i * 60 + 60U)],
              &b_y11[i * 61]);
    b_y11[61 * i + 60] = ynew[i];
  }
  if (t11_size[0] != 0) {
    input_sizes_idx_0 = static_cast<signed char>(t11_size[0]);
  } else {
    input_sizes_idx_0 = 0;
  }
  for (i = 0; i < 3; i++) {
    scale = 0.0;
    absxk = 0.0;
    t = 0.0;
    y = 0.0;
    d = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      d1 = (R11[i] * static_cast<double>(iv1[3 * i1]) +
            R11[i + 3] * static_cast<double>(iv1[3 * i1 + 1])) +
           R11[i + 6] * static_cast<double>(iv1[3 * i1 + 2]);
      d2 = R11[i + 3 * i1];
      scale += d2 * dv[i1];
      absxk += d2 * dv1[i1];
      t += d1 * v1[i1];
      y += d2 * dv2[i1];
      d += d1 * v1[i1 + 3];
    }
    d_y[i] = d;
    e_y[i] = y;
    c_y[i] = t;
    b_y[i] = absxk;
    b_T10[i] = scale;
  }
  //  ? start of 2nd seg
  scale = b_T10[1] * b_y11[914] - b_T10[2] * b_y11[853];
  absxk = b_T10[2] * b_y11[792] - b_T10[0] * b_y11[914];
  t = b_T10[0] * b_y11[853] - b_T10[1] * b_y11[792];
  b_T10[0] = b_y[1] * c_y[2] - c_y[1] * b_y[2];
  b_T10[1] = c_y[0] * b_y[2] - b_y[0] * c_y[2];
  b_T10[2] = b_y[0] * c_y[1] - c_y[0] * b_y[1];
  b_ynew[0] = b_y11[60];
  b_ynew[1] = b_y11[121];
  b_ynew[2] = b_y11[182];
  for (i = 0; i < 12; i++) {
    b_ynew[i + 3] = b_y11[61 * (i + 3) + 60];
  }
  b_ynew[15] = (scale + b_y11[975]) +
               2.0 * (b_T10[0] + (e_y[1] * d_y[2] - d_y[1] * e_y[2]));
  b_ynew[16] = (absxk + b_y11[1036]) +
               2.0 * (b_T10[1] + (d_y[0] * e_y[2] - e_y[0] * d_y[2]));
  b_ynew[17] = (t + b_y11[1097]) +
               2.0 * (b_T10[2] + (e_y[0] * d_y[1] - d_y[0] * e_y[1]));
  b_ynew[18] = b_y11[1280];
  b_ynew[19] = b_y11[1341];
  c_odeCosserat_keith(b_ynew, v1, t12_data, t11_size, y12);
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
  for (i = 0; i < 20; i++) {
    b_ynew[i] = y12[20 * i + 19];
  }
  for (i = 0; i < 3; i++) {
    b_ynew[i] = y12[20 * i + 19] +
                ((0.0 * R11[i] + 0.0 * R11[i + 3]) + 0.015 * R11[i + 6]);
  }
  //  move from 2e to g
  for (i = 0; i < 20; i++) {
    std::copy(&y12[i * 20], &y12[static_cast<int>(i * 20 + 20U)],
              &b_y12[i * 21]);
    b_y12[21 * i + 20] = b_ynew[i];
  }
  if (t11_size[0] != 0) {
    b_input_sizes_idx_0 = static_cast<signed char>(t11_size[0]);
  } else {
    b_input_sizes_idx_0 = 0;
  }
  //  elongation seg1
  //  elongation seg2
  //  R12_ = R12 * eul2rotm([-1.5850    0.8312    1.8071]);
  for (i = 0; i < 3; i++) {
    scale = 0.0;
    absxk = 0.0;
    t = 0.0;
    y = 0.0;
    d = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      d1 = R11[i];
      d2 = d1 * dv3[3 * i1];
      d3 = d1 * static_cast<double>(iv2[3 * i1]);
      d1 = R11[i + 3];
      T10_tmp = 3 * i1 + 1;
      d2 += d1 * dv3[T10_tmp];
      d3 += d1 * static_cast<double>(iv2[T10_tmp]);
      d1 = R11[i + 6];
      T10_tmp = 3 * i1 + 2;
      d2 += d1 * dv3[T10_tmp];
      d3 += d1 * static_cast<double>(iv2[T10_tmp]);
      R12__tmp = i + 3 * i1;
      R12_[R12__tmp] = d2;
      d1 = R11[R12__tmp];
      scale += d1 * dv4[i1];
      absxk += d1 * dv5[i1];
      t += d3 * v1[i1 + 6];
      y += d1 * dv6[i1];
      d += d3 * v1[i1 + 9];
    }
    d_y[i] = d;
    e_y[i] = y;
    c_y[i] = t;
    b_y[i] = absxk;
    b_T10[i] = scale;
  }
  //  ? start of 3nd seg
  scale = b_T10[1] * b_y12[314] - b_T10[2] * b_y12[293];
  absxk = b_T10[2] * b_y12[272] - b_T10[0] * b_y12[314];
  t = b_T10[0] * b_y12[293] - b_T10[1] * b_y12[272];
  b_T10[0] = b_y[1] * c_y[2] - c_y[1] * b_y[2];
  b_T10[1] = c_y[0] * b_y[2] - b_y[0] * c_y[2];
  b_T10[2] = b_y[0] * c_y[1] - c_y[0] * b_y[1];
  c_y12[0] = b_y12[20];
  c_y12[3] = R12_[0];
  c_y12[6] = R12_[3];
  c_y12[9] = R12_[6];
  c_y12[12] = scale + b_y12[272];
  c_y12[15] =
      b_y12[335] + 2.0 * (b_T10[0] + (e_y[1] * d_y[2] - d_y[1] * e_y[2]));
  c_y12[1] = b_y12[41];
  c_y12[4] = R12_[1];
  c_y12[7] = R12_[4];
  c_y12[10] = R12_[7];
  c_y12[13] = absxk + b_y12[293];
  c_y12[16] =
      b_y12[356] + 2.0 * (b_T10[1] + (d_y[0] * e_y[2] - e_y[0] * d_y[2]));
  c_y12[2] = b_y12[62];
  c_y12[5] = R12_[2];
  c_y12[8] = R12_[5];
  c_y12[11] = R12_[8];
  c_y12[14] = t + b_y12[314];
  c_y12[17] =
      b_y12[377] + 2.0 * (b_T10[2] + (e_y[0] * d_y[1] - d_y[0] * e_y[1]));
  c_y12[18] = 0.0;
  d_odeCosserat_keith(c_y12, *(double(*)[4]) & ksi[0], t13_data, t11_size, y13);
  //  second seg
  //  R2e
  //  arm2
  //  base internal force
  //  base internal load
  std::memset(&v1[0], 0, 12U * sizeof(double));
  v1[2] = Guess[16];
  v1[5] = Guess[17];
  v1[8] = Guess[18];
  v1[11] = Guess[19];
  //  rod elongation strain
  for (i = 0; i < 16; i++) {
    T10[i] = iv[i];
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
  b_T10[0] = qa[6];
  b_T10[1] = 0.0;
  b_T10[2] = 0.0;
  coder::eul2rotm(b_T10, R11);
  for (i = 0; i < 3; i++) {
    scale = T10[i];
    absxk = T10[i + 4];
    t = T10[i + 8];
    for (i1 = 0; i1 < 3; i1++) {
      R12_[i + 3 * i1] =
          (scale * R11[3 * i1] + absxk * R11[3 * i1 + 1]) + t * R11[3 * i1 + 2];
    }
  }
  //  p20=[-0.02 0 0]';R20=eul2rotm([qa(7) 0 0]);
  for (i = 0; i < 3; i++) {
    T10_tmp = i << 2;
    T10[T10_tmp] = R12_[3 * i];
    T10[T10_tmp + 1] = R12_[3 * i + 1];
    T10[T10_tmp + 2] = R12_[3 * i + 2];
    y20[i] = T10[i + 12];
  }
  y20[3] = T10[0];
  y20[6] = T10[4];
  y20[9] = T10[8];
  y20[12] = Guess[10];
  y20[15] = Guess[13];
  y20[4] = T10[1];
  y20[7] = T10[5];
  y20[10] = T10[9];
  y20[13] = Guess[11];
  y20[16] = Guess[14];
  y20[5] = T10[2];
  y20[8] = T10[6];
  y20[11] = T10[10];
  y20[14] = Guess[12];
  y20[17] = Guess[15];
  y20[18] = 0.0;
  y20[19] = 0.0;
  y20[20] = 0.0;
  y20[21] = 0.0;
  b_odeCosserat_keith(y20, v1, t21_data, t11_size, y11);
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
  for (i = 0; i < 22; i++) {
    ynew[i] = y11[60 * i + 59];
  }
  for (i = 0; i < 3; i++) {
    ynew[i] = y11[60 * i + 59] +
              ((0.0 * R11[i] + 0.0 * R11[i + 3]) + 0.01 * R11[i + 6]);
  }
  //  move from 1e to 2b
  for (i = 0; i < 22; i++) {
    std::copy(&y11[i * 60], &y11[static_cast<int>(i * 60 + 60U)], &y21[i * 61]);
    y21[61 * i + 60] = ynew[i];
  }
  if (t11_size[0] != 0) {
    c_input_sizes_idx_0 = static_cast<signed char>(t11_size[0]);
  } else {
    c_input_sizes_idx_0 = 0;
  }
  for (i = 0; i < 3; i++) {
    scale = 0.0;
    absxk = 0.0;
    t = 0.0;
    y = 0.0;
    d = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      d1 = (R11[i] * static_cast<double>(iv1[3 * i1]) +
            R11[i + 3] * static_cast<double>(iv1[3 * i1 + 1])) +
           R11[i + 6] * static_cast<double>(iv1[3 * i1 + 2]);
      d2 = R11[i + 3 * i1];
      scale += d2 * dv[i1];
      absxk += d2 * dv1[i1];
      t += d1 * v1[i1];
      y += d2 * dv2[i1];
      d += d1 * v1[i1 + 3];
    }
    d_y[i] = d;
    e_y[i] = y;
    c_y[i] = t;
    b_y[i] = absxk;
    b_T10[i] = scale;
  }
  //  ? start of 2nd seg
  scale = b_T10[1] * y21[914] - b_T10[2] * y21[853];
  absxk = b_T10[2] * y21[792] - b_T10[0] * y21[914];
  t = b_T10[0] * y21[853] - b_T10[1] * y21[792];
  b_T10[0] = b_y[1] * c_y[2] - c_y[1] * b_y[2];
  b_T10[1] = c_y[0] * b_y[2] - b_y[0] * c_y[2];
  b_T10[2] = b_y[0] * c_y[1] - c_y[0] * b_y[1];
  b_ynew[0] = y21[60];
  b_ynew[1] = y21[121];
  b_ynew[2] = y21[182];
  for (i = 0; i < 12; i++) {
    b_ynew[i + 3] = y21[61 * (i + 3) + 60];
  }
  b_ynew[15] = (scale + y21[975]) +
               2.0 * (b_T10[0] + (e_y[1] * d_y[2] - d_y[1] * e_y[2]));
  b_ynew[16] = (absxk + y21[1036]) +
               2.0 * (b_T10[1] + (d_y[0] * e_y[2] - e_y[0] * d_y[2]));
  b_ynew[17] =
      (t + y21[1097]) + 2.0 * (b_T10[2] + (e_y[0] * d_y[1] - d_y[0] * e_y[1]));
  b_ynew[18] = y21[1280];
  b_ynew[19] = y21[1341];
  c_odeCosserat_keith(b_ynew, v1, t22_data, t11_size, y12);
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
  for (i = 0; i < 20; i++) {
    b_ynew[i] = y12[20 * i + 19];
  }
  for (i = 0; i < 3; i++) {
    b_ynew[i] = y12[20 * i + 19] +
                ((0.0 * R11[i] + 0.0 * R11[i + 3]) + 0.015 * R11[i + 6]);
  }
  //  move from 2e to g
  for (i = 0; i < 20; i++) {
    std::copy(&y12[i * 20], &y12[static_cast<int>(i * 20 + 20U)], &y22[i * 21]);
    y22[21 * i + 20] = b_ynew[i];
  }
  if (t11_size[0] != 0) {
    d_input_sizes_idx_0 = static_cast<signed char>(t11_size[0]);
  } else {
    d_input_sizes_idx_0 = 0;
  }
  //  elongation seg1
  //  elongation seg2
  //  R22_ = R22 * eul2rotm([1.9264   -0.4921    0.7374]);
  for (i = 0; i < 3; i++) {
    scale = 0.0;
    absxk = 0.0;
    t = 0.0;
    y = 0.0;
    d = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      d1 = R11[i];
      d2 = d1 * dv7[3 * i1];
      d3 = d1 * static_cast<double>(iv2[3 * i1]);
      d1 = R11[i + 3];
      T10_tmp = 3 * i1 + 1;
      d2 += d1 * dv7[T10_tmp];
      d3 += d1 * static_cast<double>(iv2[T10_tmp]);
      d1 = R11[i + 6];
      T10_tmp = 3 * i1 + 2;
      d2 += d1 * dv7[T10_tmp];
      d3 += d1 * static_cast<double>(iv2[T10_tmp]);
      R12__tmp = i + 3 * i1;
      R12_[R12__tmp] = d2;
      d1 = R11[R12__tmp];
      scale += d1 * dv4[i1];
      absxk += d1 * dv5[i1];
      t += d3 * v1[i1 + 6];
      y += d1 * dv6[i1];
      d += d3 * v1[i1 + 9];
    }
    d_y[i] = d;
    e_y[i] = y;
    c_y[i] = t;
    b_y[i] = absxk;
    b_T10[i] = scale;
  }
  //  ? start of 2nd seg
  scale = b_y[1] * c_y[2] - c_y[1] * b_y[2];
  absxk = c_y[0] * b_y[2] - b_y[0] * c_y[2];
  t = b_y[0] * c_y[1] - c_y[0] * b_y[1];
  c_y12[0] = y22[20];
  c_y12[3] = R12_[0];
  c_y12[6] = R12_[3];
  c_y12[9] = R12_[6];
  c_y12[12] = y22[272];
  c_y12[15] = ((b_T10[1] * y22[314] - b_T10[2] * y22[293]) + y22[335]) +
              2.0 * (scale + (e_y[1] * d_y[2] - d_y[1] * e_y[2]));
  c_y12[1] = y22[41];
  c_y12[4] = R12_[1];
  c_y12[7] = R12_[4];
  c_y12[10] = R12_[7];
  c_y12[13] = y22[293];
  c_y12[16] = ((b_T10[2] * y22[272] - b_T10[0] * y22[314]) + y22[356]) +
              2.0 * (absxk + (d_y[0] * e_y[2] - e_y[0] * d_y[2]));
  c_y12[2] = y22[62];
  c_y12[5] = R12_[2];
  c_y12[8] = R12_[5];
  c_y12[11] = R12_[8];
  c_y12[14] = y22[314];
  c_y12[17] = ((b_T10[0] * y22[293] - b_T10[1] * y22[272]) + y22[377]) +
              2.0 * (t + (e_y[0] * d_y[1] - d_y[0] * e_y[1]));
  c_y12[18] = 0.0;
  d_odeCosserat_keith(c_y12, *(double(*)[4]) & ksi[4], b_ynew, t11_size, y23);
  //  second seg
  //  R2e
  t0[0] = 0.0;
  t0[1] = 0.0;
  t0[2] = 0.0;
  for (i = 0; i < 22; i++) {
    b_y0[3 * i] = y10[i];
    b_y0[3 * i + 1] = 0.0;
    b_y0[3 * i + 2] = y20[i];
  }
  t1_size[0] = (input_sizes_idx_0 + c_input_sizes_idx_0) + 3;
  T10_tmp = input_sizes_idx_0;
  if (0 <= T10_tmp - 1) {
    std::copy(&t11_data[0], &t11_data[T10_tmp], &t1_data[0]);
  }
  t1_data[input_sizes_idx_0] = t11_data[59] + 0.01;
  t1_data[input_sizes_idx_0 + 1] = 0.0;
  T10_tmp = c_input_sizes_idx_0;
  for (i = 0; i < T10_tmp; i++) {
    t1_data[(i + input_sizes_idx_0) + 2] = t21_data[i];
  }
  t1_data[(input_sizes_idx_0 + c_input_sizes_idx_0) + 2] = t21_data[59] + 0.01;
  for (i = 0; i < 22; i++) {
    b_y1[123 * i + 61] = 0.0;
    for (i1 = 0; i1 < 61; i1++) {
      T10_tmp = i1 + 61 * i;
      y1_tmp = i1 + 123 * i;
      b_y1[y1_tmp] = b_y11[T10_tmp];
      b_y1[y1_tmp + 62] = y21[T10_tmp];
    }
  }
  i = b_input_sizes_idx_0 + d_input_sizes_idx_0;
  t2_size[0] = i + 3;
  T10_tmp = b_input_sizes_idx_0;
  if (0 <= T10_tmp - 1) {
    std::copy(&t12_data[0], &t12_data[T10_tmp], &t2_data[0]);
  }
  t2_data[b_input_sizes_idx_0] = t12_data[19] + 0.015;
  t2_data[b_input_sizes_idx_0 + 1] = 0.0;
  T10_tmp = d_input_sizes_idx_0;
  for (i1 = 0; i1 < T10_tmp; i1++) {
    t2_data[(i1 + b_input_sizes_idx_0) + 2] = t22_data[i1];
  }
  t2_data[i + 2] = t22_data[19] + 0.015;
  t3_size[0] = 41;
  t3_data[20] = 0.0;
  for (i = 0; i < 20; i++) {
    y2[43 * i + 21] = 0.0;
    for (i1 = 0; i1 < 21; i1++) {
      T10_tmp = i1 + 21 * i;
      y1_tmp = i1 + 43 * i;
      y2[y1_tmp] = b_y12[T10_tmp];
      y2[y1_tmp + 22] = y22[T10_tmp];
    }
    t3_data[i] = t13_data[i];
    t3_data[i + 21] = b_ynew[i];
  }
  for (i = 0; i < 19; i++) {
    y3[41 * i + 20] = 0.0;
    for (i1 = 0; i1 < 20; i1++) {
      T10_tmp = i1 + 20 * i;
      y1_tmp = i1 + 41 * i;
      y3[y1_tmp] = y13[T10_tmp];
      y3[y1_tmp + 21] = y23[T10_tmp];
    }
  }
  std::memset(&Rsd[0], 0, 20U * sizeof(double));
  //  boundary conditions
  for (i = 0; i < 3; i++) {
    T10_tmp = 20 * (i + 12) + 19;
    Rsd[i] = (0.0 - y13[T10_tmp]) - y23[T10_tmp];
    T10_tmp = 20 * (i + 15) + 19;
    Rsd[i + 3] = (0.0 - y13[T10_tmp]) - y23[T10_tmp];
    T10_tmp = 20 * i + 19;
    Rsd[i + 6] = y13[T10_tmp] - y23[T10_tmp];
    R12__tmp = 20 * (i + 3) + 19;
    scale = y13[R12__tmp];
    T10_tmp = 20 * (i + 6) + 19;
    absxk = y13[T10_tmp];
    y1_tmp = 20 * (i + 9) + 19;
    t = y13[y1_tmp];
    y12__tmp[3 * i] = y23[R12__tmp];
    y12__tmp[3 * i + 1] = y23[T10_tmp];
    y12__tmp[3 * i + 2] = y23[y1_tmp];
    for (i1 = 0; i1 < 3; i1++) {
      R11[i + 3 * i1] =
          (scale * dv8[3 * i1] + absxk * dv8[3 * i1 + 1]) + t * dv8[3 * i1 + 2];
    }
  }
  for (i = 0; i < 3; i++) {
    scale = y12__tmp[i];
    absxk = y12__tmp[i + 3];
    t = y12__tmp[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      R12_[i + 3 * i1] =
          (scale * R11[3 * i1] + absxk * R11[3 * i1 + 1]) + t * R11[3 * i1 + 2];
    }
  }
  coder::rotm2axang(R12_, b_dv);
  Rsd[9] = b_dv[0] * b_dv[3];
  Rsd[10] = b_dv[1] * b_dv[3];
  Rsd[11] = b_dv[2] * b_dv[3];
  //  % % % % Rsd(1:3)=Fe-yL12(13:15)-yL22(13:15); % boundary conditions
  //  % % % %
  //  Rsd(4:6)=Me-yL12(16:18)-2*(cross(R12*MBP1.r21,R12*MBP1.Ke1*v1(:,3))+... %
  //  % % %     cross(R12*MBP1.r22,R12*MBP1.Ke1*v1(:,4)))... % % % %
  //  -yL22(16:18)-2*(cross(R22*MBP2.r21,R22*MBP2.Ke1*v2(:,3))+... % % % %
  //  cross(R22*MBP2.r22,R22*MBP2.Ke1*v2(:,4))); % % % %
  //  Rsd(7:9)=yL12(1:3)-yL22(1:3); % % % % axang =
  //  rotm2axang(R23'*(R13*eul2rotm([0 pi 0])))'; % % % %
  //  Rsd(10:12)=axang(1:3)*axang(4)*0;
  Rsd[12] = b_y11[1158] - (qa[2] + 0.7 * Guess[6]);
  Rsd[13] = b_y11[1219] - (qa[3] + 0.7 * Guess[7]);
  //  path length
  Rsd[14] = b_y12[398] - (qa[4] + 0.73 * Guess[8]);
  Rsd[15] = b_y12[419] - (qa[5] + 0.73 * Guess[9]);
  Rsd[16] = y21[1158] - (qa[8] + 0.7 * Guess[16]);
  Rsd[17] = y21[1219] - (qa[9] + 0.7 * Guess[17]);
  //  path length
  Rsd[18] = y22[398] - (qa[10] + 0.73 * Guess[18]);
  Rsd[19] = y22[419] - (qa[11] + 0.73 * Guess[19]);
}

//
// File trailer for forShooting_keith.cpp
//
// [EOF]
//
