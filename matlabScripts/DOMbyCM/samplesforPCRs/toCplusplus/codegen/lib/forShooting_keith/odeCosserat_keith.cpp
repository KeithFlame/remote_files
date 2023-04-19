//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: odeCosserat_keith.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 13-Apr-2023 16:35:29
//

// Include Files
#include "odeCosserat_keith.h"
#include "eul2rotm.h"
#include "forShooting_keith_data.h"
#include "forShooting_keith_rtwutil.h"
#include "linspace.h"
#include "mldivide.h"
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "skewMatrix_keith.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Variable Definitions
static const double dv9[12]{0.0,     -0.0025, 0.0, 0.0025, -0.0,    0.0,
                            -0.0019, -0.0019, 0.0, 0.0019, -0.0019, 0.0};

static const double dv10[6]{-0.0019, -0.0019, 0.0, 0.0019, -0.0019, 0.0};

static const double dv11[9]{0.0017643200000000002,
                            0.0,
                            0.0,
                            0.0,
                            0.0017643200000000002,
                            0.0,
                            0.0,
                            0.0,
                            0.00132464};

static const signed char iv3[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};

// Function Declarations
static void binary_expand_op(double q_data[], int q_size[2],
                             const double b_q_data[], const int *b_q_size,
                             const double C_data[], const int C_size[2],
                             double step);

// Function Definitions
//
// Arguments    : double q_data[]
//                int q_size[2]
//                const double b_q_data[]
//                const int *b_q_size
//                const double C_data[]
//                const int C_size[2]
//                double step
// Return Type  : void
//
static void binary_expand_op(double q_data[], int q_size[2],
                             const double b_q_data[], const int *b_q_size,
                             const double C_data[], const int C_size[2],
                             double step)
{
  int q_idx_0;
  int stride_0_0;
  int stride_1_0;
  q_idx_0 = *b_q_size;
  if (C_size[0] == 1) {
    q_size[0] = q_idx_0;
  } else {
    q_size[0] = C_size[0];
  }
  q_size[1] = 1;
  stride_0_0 = (q_idx_0 != 1);
  stride_1_0 = (C_size[0] != 1);
  if (C_size[0] != 1) {
    q_idx_0 = C_size[0];
  }
  for (int i{0}; i < q_idx_0; i++) {
    q_data[i] = b_q_data[i * stride_0_0] + C_data[i * stride_1_0] * step;
  }
}

//
// -----Integral of IVP using difference equation-------
//  for a coninuum robot kinematics script, forward differential equations.
//
//  input1: y0 initial condition
//  input2: v rod elongation strain
//  SegIdx- the segment index (1 or 2) you are integrating
//  MBP- Mechanical properties
//  fe, le- external distributed loads
//  step- integrating step size
// ----------info-----------%
//  ver f1.0
//  by Yuyang Chen
//  date 20200524
// -------------------------------------------------------------------------%
//
// Arguments    : const double b_y0[22]
//                const double v[12]
//                double t_data[]
//                int t_size[2]
//                double y[1320]
// Return Type  : void
//
void b_odeCosserat_keith(const double b_y0[22], const double v[12],
                         double t_data[], int t_size[2], double y[1320])
{
  static const double b_dv[3]{0.0, 0.0, 0.0010169491525423729};
  double b_y[60];
  double B_data[27];
  double R_data[27];
  double b_R_data[27];
  double d_y_data[22];
  double e_y_data[22];
  double Q_data[12];
  double C_data[9];
  double R_dot[9];
  double b_dv1[9];
  double b_y_data[9];
  double tmp_data[9];
  double y_data[9];
  double b[3];
  double c_y_data[3];
  double d_y[3];
  double e_y[3];
  double f_y[3];
  double g_y[3];
  double h_y[3];
  double i_y[3];
  double j_y[3];
  double k_y[3];
  double l_y[3];
  double m_y[3];
  double n_y[3];
  double o_y[3];
  double u_data[3];
  double d;
  double d1;
  double d10;
  double d11;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  double d7;
  double d8;
  double d9;
  int a_size[2];
  int b_y_size[2];
  int y_size[2];
  int i;
  int k;
  std::copy(&dv9[0], &dv9[12], &Q_data[0]);
  b_y[59] = 0.06;
  b_y[0] = 0.0;
  for (k = 0; k < 58; k++) {
    b_y[k + 1] = (static_cast<double>(k) + 1.0) * 0.0010169491525423729;
  }
  t_size[0] = 60;
  t_size[1] = 1;
  std::copy(&b_y[0], &b_y[60], &t_data[0]);
  std::memset(&y[0], 0, 1320U * sizeof(double));
  for (i = 0; i < 22; i++) {
    y[60 * i] = b_y0[i];
  }
  a_size[0] = 3;
  a_size[1] = 3;
  d = v[0];
  d1 = v[3];
  d2 = v[6];
  d3 = v[9];
  d4 = v[1];
  d5 = v[4];
  d6 = v[7];
  d7 = v[10];
  d8 = v[2];
  d9 = v[5];
  d10 = v[8];
  d11 = v[11];
  for (int b_i{0}; b_i < 59; b_i++) {
    double absxk;
    double b_y_tmp;
    double c_y;
    double c_y_tmp;
    double cosdelta;
    double costheta;
    double d12;
    double d13;
    double d14;
    double d15;
    double d16;
    double d17;
    double p_dot_data_idx_0;
    double p_dot_data_idx_1;
    double p_dot_data_idx_2;
    double scale;
    double sintheta;
    double t;
    double y_tmp;
    int aoffset;
    R_dot[0] = y[b_i + 180];
    R_dot[3] = y[b_i + 360];
    R_dot[6] = y[b_i + 540];
    R_dot[1] = y[b_i + 240];
    R_dot[4] = y[b_i + 420];
    R_dot[7] = y[b_i + 600];
    R_dot[2] = y[b_i + 300];
    R_dot[5] = y[b_i + 480];
    R_dot[8] = y[b_i + 660];
    std::copy(&R_dot[0], &R_dot[9], &R_data[0]);
    // constant-curvature-based evolution
    for (i = 0; i < 3; i++) {
      for (aoffset = 0; aoffset < 3; aoffset++) {
        b_R_data[aoffset + 3 * i] = R_data[i + 3 * aoffset];
      }
    }
    std::copy(&b_R_data[0], &b_R_data[9], &B_data[0]);
    y_size[0] = 3;
    y_size[1] = 1;
    d12 = B_data[1] - B_data[0] * 0.0;
    d13 = ((B_data[2] - B_data[0] * 0.0) - d12 * 0.0) / 0.011404639999999999;
    R_dot[2] = d13;
    d12 -= d13 * 0.0;
    d12 /= 0.01520432;
    R_dot[1] = d12;
    R_dot[0] = ((B_data[0] - d13 * 0.0) - d12 * 0.0) / 0.01520432;
    y_tmp = y[b_i + 900];
    y_data[0] = y_tmp;
    d12 = B_data[4] - B_data[3] * 0.0;
    d13 = ((B_data[5] - B_data[3] * 0.0) - d12 * 0.0) / 0.011404639999999999;
    R_dot[5] = d13;
    d12 -= d13 * 0.0;
    d12 /= 0.01520432;
    R_dot[4] = d12;
    R_dot[3] = ((B_data[3] - d13 * 0.0) - d12 * 0.0) / 0.01520432;
    b_y_tmp = y[b_i + 960];
    y_data[1] = b_y_tmp;
    d12 = B_data[7] - B_data[6] * 0.0;
    d13 = ((B_data[8] - B_data[6] * 0.0) - d12 * 0.0) / 0.011404639999999999;
    R_dot[8] = d13;
    d12 -= d13 * 0.0;
    d12 /= 0.01520432;
    R_dot[7] = d12;
    R_dot[6] = ((B_data[6] - d13 * 0.0) - d12 * 0.0) / 0.01520432;
    c_y_tmp = y[b_i + 1020];
    y_data[2] = c_y_tmp;
    coder::internal::blas::mtimes(R_dot, a_size, y_data, y_size, b_y_data,
                                  b_y_size);
    y_size[0] = 3;
    y_size[1] = 1;
    y_data[0] = y_tmp;
    y_data[1] = b_y_tmp;
    y_data[2] = c_y_tmp;
    coder::internal::blas::mtimes(R_dot, a_size, y_data, y_size, tmp_data,
                                  b_y_size);
    aoffset = 3 * b_y_size[1];
    if (0 <= aoffset - 1) {
      std::copy(&tmp_data[0], &tmp_data[aoffset], &u_data[0]);
    }
    u_data[2] = 0.0;
    scale = 3.3121686421112381E-170;
    absxk = std::abs(u_data[0]);
    if (absxk > 3.3121686421112381E-170) {
      c_y = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      c_y = t * t;
    }
    absxk = std::abs(u_data[1]);
    if (absxk > scale) {
      t = scale / absxk;
      c_y = c_y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      c_y += t * t;
    }
    c_y = scale * std::sqrt(c_y);
    absxk = 0.0010169491525423729 * c_y;
    scale = rt_atan2d_snf(u_data[1], u_data[0]);
    costheta = std::cos(absxk);
    sintheta = std::sin(absxk);
    cosdelta = std::cos(-scale + 1.5707963267948966);
    t = std::sin(-scale + 1.5707963267948966);
    //  cache
    if (absxk != 0.0) {
      c_y = 0.0010169491525423729 / absxk;
      d_y[0] = c_y * (cosdelta * (1.0 - costheta));
      d_y[1] = c_y * (t * (costheta - 1.0));
      d_y[2] = c_y * sintheta;
      p_dot_data_idx_0 = 0.0;
      p_dot_data_idx_1 = 0.0;
      p_dot_data_idx_2 = 0.0;
      for (k = 0; k < 3; k++) {
        aoffset = k * 3;
        d12 = d_y[k];
        p_dot_data_idx_0 += R_data[aoffset] * d12;
        p_dot_data_idx_1 += R_data[aoffset + 1] * d12;
        p_dot_data_idx_2 += R_data[aoffset + 2] * d12;
      }
      scale = t * t;
      absxk = cosdelta * cosdelta;
      R_dot[0] = absxk * costheta + scale;
      R_dot[3] = -t * cosdelta * (costheta - 1.0);
      R_dot[6] = cosdelta * sintheta;
      R_dot[1] = t * cosdelta * (1.0 - costheta);
      R_dot[4] = absxk + costheta * scale;
      R_dot[7] = -t * sintheta;
      R_dot[2] = -cosdelta * sintheta;
      R_dot[5] = t * sintheta;
      R_dot[8] = costheta;
    } else {
      p_dot_data_idx_0 = 0.0;
      p_dot_data_idx_1 = 0.0;
      p_dot_data_idx_2 = 0.0;
      for (k = 0; k < 3; k++) {
        aoffset = k * 3;
        d12 = b_dv[k];
        p_dot_data_idx_0 += R_data[aoffset] * d12;
        p_dot_data_idx_1 += R_data[aoffset + 1] * d12;
        p_dot_data_idx_2 += R_data[aoffset + 2] * d12;
      }
      std::memset(&R_dot[0], 0, 9U * sizeof(double));
      R_dot[0] = 1.0;
      R_dot[4] = 1.0;
      R_dot[8] = 1.0;
    }
    // this function gives the skew-symmetric matrix of the vector p
    //      if(length(p)==3)
    y_data[0] = 0.0;
    y_data[3] = -0.0;
    y_data[6] = b_y_data[1];
    y_data[1] = 0.0;
    y_data[4] = 0.0;
    y_data[7] = -b_y_data[0];
    y_data[2] = -b_y_data[1];
    y_data[5] = b_y_data[0];
    y_data[8] = 0.0;
    //      elseif(length(p)==6)
    //          R=[0 -p(6) p(5); p(6) 0 -p(4);-p(5) p(4) 0];
    //          T=[R p(1:3);zeros(1,4)];
    //      end
    for (int j{0}; j < 3; j++) {
      int coffset_tmp;
      coffset_tmp = j * 3;
      d12 = 0.0;
      d13 = 0.0;
      d14 = 0.0;
      d15 = y_data[j];
      d16 = y_data[j + 3];
      d17 = y_data[j + 6];
      for (k = 0; k < 3; k++) {
        int bkj_tmp;
        aoffset = k * 3;
        bkj_tmp = coffset_tmp + k;
        scale = R_dot[bkj_tmp];
        d12 += R_data[aoffset] * scale;
        d13 += R_data[aoffset + 1] * scale;
        d14 += R_data[aoffset + 2] * scale;
        i = j + 3 * k;
        b_dv1[i] = (d15 * static_cast<double>(iv2[3 * k]) +
                    d16 * static_cast<double>(iv2[3 * k + 1])) +
                   d17 * static_cast<double>(iv2[3 * k + 2]);
        tmp_data[i] = (d15 * static_cast<double>(iv1[3 * k]) +
                       d16 * static_cast<double>(iv1[3 * k + 1])) +
                      d17 * static_cast<double>(iv1[3 * k + 2]);
        C_data[bkj_tmp] = 0.0;
      }
      b_y_data[coffset_tmp + 2] = d14;
      b_y_data[coffset_tmp + 1] = d13;
      b_y_data[coffset_tmp] = d12;
      d12 = C_data[coffset_tmp];
      d13 = C_data[coffset_tmp + 1];
      d14 = C_data[coffset_tmp + 2];
      for (k = 0; k < 3; k++) {
        aoffset = k * 3;
        scale = R_dot[coffset_tmp + k];
        d12 += R_data[aoffset] * scale;
        d13 += R_data[aoffset + 1] * scale;
        d14 += R_data[aoffset + 2] * scale;
      }
      C_data[coffset_tmp + 2] = d14;
      C_data[coffset_tmp + 1] = d13;
      C_data[coffset_tmp] = d12;
    }
    for (i = 0; i < 9; i++) {
      C_data[i] *= 2.0;
    }
    for (i = 0; i < 3; i++) {
      double d18;
      short i1;
      d12 = y_data[i];
      d13 = d12 * 0.0025;
      i1 = iv1[i];
      d14 = static_cast<double>(i1) * d;
      d15 = tmp_data[i];
      d16 = d15 * d;
      d17 = d12 * 0.0;
      scale = static_cast<double>(i1) * d1;
      absxk = d15 * d1;
      t = d12 * 0.0019;
      i1 = iv2[i];
      c_y = static_cast<double>(i1) * d2;
      d15 = b_dv1[i];
      costheta = d15 * d2;
      sintheta = static_cast<double>(i1) * d3;
      cosdelta = d15 * d3;
      d12 = y_data[i + 3];
      d13 += d12 * 0.0;
      i1 = iv1[i + 3];
      d14 += static_cast<double>(i1) * d4;
      d15 = tmp_data[i + 3];
      d16 += d15 * d4;
      d17 += d12 * 0.0025;
      scale += static_cast<double>(i1) * d5;
      absxk += d15 * d5;
      d18 = t + d12 * -0.0019;
      i1 = iv2[i + 3];
      c_y += static_cast<double>(i1) * d6;
      d15 = b_dv1[i + 3];
      costheta += d15 * d6;
      t += d12 * 0.0019;
      sintheta += static_cast<double>(i1) * d7;
      cosdelta += d15 * d7;
      d12 = y_data[i + 6];
      d13 += d12 * 0.0;
      i1 = iv1[i + 6];
      d14 += static_cast<double>(i1) * d8;
      d15 = tmp_data[i + 6];
      d16 += d15 * d8;
      d17 += d12 * 0.0;
      scale += static_cast<double>(i1) * d9;
      absxk += d15 * d9;
      d18 += d12 * 0.0;
      i1 = iv2[i + 6];
      c_y += static_cast<double>(i1) * d10;
      d15 = b_dv1[i + 6];
      costheta += d15 * d10;
      t += d12 * 0.0;
      sintheta += static_cast<double>(i1) * d11;
      cosdelta += d15 * d11;
      o_y[i] = cosdelta;
      n_y[i] = sintheta;
      m_y[i] = t;
      l_y[i] = costheta;
      k_y[i] = c_y;
      j_y[i] = d18;
      i_y[i] = absxk;
      h_y[i] = scale;
      g_y[i] = d17;
      f_y[i] = d16;
      e_y[i] = d14;
      d_y[i] = d13;
    }
    b[0] = ((((d_y[1] * e_y[2] - e_y[1] * d_y[2]) +
              (0.0 * f_y[2] - 0.0 * f_y[1])) +
             (g_y[1] * h_y[2] - h_y[1] * g_y[2])) +
            (0.0025 * i_y[2] - 0.0 * i_y[1])) +
           ((((j_y[1] * k_y[2] - k_y[1] * j_y[2]) +
              (-0.0019 * l_y[2] - 0.0 * l_y[1])) +
             (m_y[1] * n_y[2] - n_y[1] * m_y[2])) +
            (0.0019 * o_y[2] - 0.0 * o_y[1]));
    b[1] = ((((e_y[0] * d_y[2] - d_y[0] * e_y[2]) +
              (0.0 * f_y[0] - 0.0025 * f_y[2])) +
             (h_y[0] * g_y[2] - g_y[0] * h_y[2])) +
            (0.0 * i_y[0] - 0.0 * i_y[2])) +
           ((((k_y[0] * j_y[2] - j_y[0] * k_y[2]) +
              (0.0 * l_y[0] - 0.0019 * l_y[2])) +
             (n_y[0] * m_y[2] - m_y[0] * n_y[2])) +
            (0.0 * o_y[0] - 0.0019 * o_y[2]));
    b[2] = ((((d_y[0] * e_y[1] - e_y[0] * d_y[1]) +
              (0.0025 * f_y[1] - 0.0 * f_y[0])) +
             (g_y[0] * h_y[1] - h_y[0] * g_y[1])) +
            (0.0 * i_y[1] - 0.0025 * i_y[0])) +
           ((((j_y[0] * k_y[1] - k_y[0] * j_y[1]) +
              (0.0019 * l_y[1] - -0.0019 * l_y[0])) +
             (m_y[0] * n_y[1] - n_y[0] * m_y[1])) +
            (0.0019 * o_y[1] - 0.0019 * o_y[0]));
    // this function gives the skew-symmetric matrix of the vector p
    //      if(length(p)==3)
    //      elseif(length(p)==6)
    //          R=[0 -p(6) p(5); p(6) 0 -p(4);-p(5) p(4) 0];
    //          T=[R p(1:3);zeros(1,4)];
    //      end
    scale = 0.0;
    absxk = 0.0;
    t = 0.0;
    c_y = 0.0;
    d12 = 0.0;
    d13 = 0.0;
    d14 = 0.0;
    for (k = 0; k < 3; k++) {
      aoffset = k * 3;
      d15 = b[k];
      d12 += C_data[aoffset] * d15;
      d13 += C_data[aoffset + 1] * d15;
      d14 += C_data[aoffset + 2] * d15;
      d15 = u_data[k];
      scale += Q_data[k] * d15;
      absxk += Q_data[k + 3] * d15;
      t += Q_data[k + 6] * d15;
      c_y += Q_data[k + 9] * d15;
    }
    c_y_data[2] = d14;
    c_y_data[1] = d13;
    c_y_data[0] = d12;
    y_data[0] = -0.0;
    y_data[3] = p_dot_data_idx_2;
    y_data[6] = -p_dot_data_idx_1;
    y_data[1] = -p_dot_data_idx_2;
    y_data[4] = -0.0;
    y_data[7] = p_dot_data_idx_0;
    y_data[2] = p_dot_data_idx_1;
    y_data[5] = -p_dot_data_idx_0;
    y_data[8] = -0.0;
    d12 = y[b_i + 720];
    d13 = y[b_i + 780];
    d14 = y[b_i + 840];
    for (i = 0; i < 3; i++) {
      e_y[i] = ((y_data[i] * d12 + y_data[i + 3] * d13) + y_data[i + 6] * d14) -
               c_y_data[i] * 0.0010169491525423729;
    }
    d_y_data[0] = y[b_i + 1080] + scale * 0.0010169491525423729;
    d_y_data[1] = y[b_i + 1140] + absxk * 0.0010169491525423729;
    d_y_data[2] = y[b_i + 1200] + t * 0.0010169491525423729;
    d_y_data[3] = y[b_i + 1260] + c_y * 0.0010169491525423729;
    e_y_data[0] = y[b_i] + p_dot_data_idx_0;
    e_y_data[3] = b_y_data[0];
    e_y_data[6] = b_y_data[3];
    e_y_data[9] = b_y_data[6];
    e_y_data[12] = d12;
    e_y_data[15] = y_tmp + e_y[0];
    e_y_data[1] = y[b_i + 60] + p_dot_data_idx_1;
    e_y_data[4] = b_y_data[1];
    e_y_data[7] = b_y_data[4];
    e_y_data[10] = b_y_data[7];
    e_y_data[13] = d13;
    e_y_data[16] = b_y_tmp + e_y[1];
    e_y_data[2] = y[b_i + 120] + p_dot_data_idx_2;
    e_y_data[5] = b_y_data[2];
    e_y_data[8] = b_y_data[5];
    e_y_data[11] = b_y_data[8];
    e_y_data[14] = d14;
    e_y_data[17] = c_y_tmp + e_y[2];
    for (i = 0; i < 4; i++) {
      e_y_data[i + 18] = d_y_data[i];
    }
    for (i = 0; i < 22; i++) {
      y[(b_i + 60 * i) + 1] = e_y_data[i];
    }
  }
}

//
// -----Integral of IVP using difference equation-------
//  for a coninuum robot kinematics script, forward differential equations.
//
//  input1: y0 initial condition
//  input2: v rod elongation strain
//  SegIdx- the segment index (1 or 2) you are integrating
//  MBP- Mechanical properties
//  fe, le- external distributed loads
//  step- integrating step size
// ----------info-----------%
//  ver f1.0
//  by Yuyang Chen
//  date 20200524
// -------------------------------------------------------------------------%
//
// Arguments    : const double b_y0[20]
//                const double v[12]
//                double t_data[]
//                int t_size[2]
//                double y[400]
// Return Type  : void
//
void c_odeCosserat_keith(const double b_y0[20], const double v[12],
                         double t_data[], int t_size[2], double y[400])
{
  double b_y[20];
  double R[9];
  double R_dot[9];
  double a[9];
  double Q_data[6];
  double d_y[3];
  double e_y[3];
  double f_y[3];
  double g_y[3];
  double h_y[3];
  double i_y[3];
  double j_y[3];
  double k_y[3];
  double l_y[3];
  double m_y[3];
  double n_y[3];
  double o_y[3];
  double p_dot[3];
  double u[3];
  double d;
  double d1;
  double d10;
  double d11;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  double d7;
  double d8;
  double d9;
  int i;
  int k;
  for (i = 0; i < 6; i++) {
    Q_data[i] = dv10[i];
  }
  b_y[19] = 0.09;
  b_y[0] = 0.069999999999999993;
  for (k = 0; k < 18; k++) {
    b_y[k + 1] = (static_cast<double>(k) + 1.0) * 0.0010526315789473686 +
                 0.069999999999999993;
  }
  t_size[0] = 20;
  t_size[1] = 1;
  std::copy(&b_y[0], &b_y[20], &t_data[0]);
  std::memset(&y[0], 0, 400U * sizeof(double));
  for (i = 0; i < 20; i++) {
    y[20 * i] = b_y0[i];
  }
  d = v[0];
  d1 = v[1];
  d2 = v[2];
  d3 = v[3];
  d4 = v[4];
  d5 = v[5];
  d6 = v[6];
  d7 = v[7];
  d8 = v[8];
  d9 = v[9];
  d10 = v[10];
  d11 = v[11];
  for (int b_i{0}; b_i < 19; b_i++) {
    double absxk;
    double c_y;
    double cosdelta;
    double costheta;
    double d12;
    double d13;
    double scale;
    double sintheta;
    double t;
    double y_data_idx_1;
    // constant-curvature-based evolution
    for (i = 0; i < 3; i++) {
      d12 = y[b_i + 20 * (i + 3)];
      R[i] = d12;
      d13 = y[b_i + 20 * (i + 6)];
      R[i + 3] = d13;
      absxk = y[b_i + 20 * (i + 9)];
      R[i + 6] = absxk;
      a[3 * i] = d12;
      a[3 * i + 1] = d13;
      a[3 * i + 2] = absxk;
    }
    coder::mldivide(dv11, a, R_dot);
    d12 = y[b_i + 300];
    d13 = y[b_i + 320];
    absxk = y[b_i + 340];
    for (i = 0; i < 3; i++) {
      u[i] = (R_dot[i] * d12 + R_dot[i + 3] * d13) + R_dot[i + 6] * absxk;
    }
    u[2] = 0.0;
    scale = 3.3121686421112381E-170;
    absxk = std::abs(u[0]);
    if (absxk > 3.3121686421112381E-170) {
      c_y = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      c_y = t * t;
    }
    absxk = std::abs(u[1]);
    if (absxk > scale) {
      t = scale / absxk;
      c_y = c_y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      c_y += t * t;
    }
    c_y = scale * std::sqrt(c_y);
    absxk = 0.0010526315789473684 * c_y;
    scale = rt_atan2d_snf(u[1], u[0]);
    costheta = std::cos(absxk);
    sintheta = std::sin(absxk);
    cosdelta = std::cos(-scale + 1.5707963267948966);
    t = std::sin(-scale + 1.5707963267948966);
    //  cache
    if (absxk != 0.0) {
      c_y = 0.0010526315789473684 / absxk;
      d12 = c_y * (cosdelta * (1.0 - costheta));
      d13 = c_y * (t * (costheta - 1.0));
      absxk = c_y * sintheta;
      for (i = 0; i < 3; i++) {
        p_dot[i] = (R[i] * d12 + R[i + 3] * d13) + R[i + 6] * absxk;
      }
      scale = t * t;
      absxk = cosdelta * cosdelta;
      R_dot[0] = absxk * costheta + scale;
      R_dot[3] = -t * cosdelta * (costheta - 1.0);
      R_dot[6] = cosdelta * sintheta;
      R_dot[1] = t * cosdelta * (1.0 - costheta);
      R_dot[4] = absxk + costheta * scale;
      R_dot[7] = -t * sintheta;
      R_dot[2] = -cosdelta * sintheta;
      R_dot[5] = t * sintheta;
      R_dot[8] = costheta;
    } else {
      for (i = 0; i < 3; i++) {
        p_dot[i] =
            (R[i] * 0.0 + R[i + 3] * 0.0) + R[i + 6] * 0.0010526315789473684;
      }
      std::memset(&R_dot[0], 0, 9U * sizeof(double));
      R_dot[0] = 1.0;
      R_dot[4] = 1.0;
      R_dot[8] = 1.0;
    }
    for (i = 0; i < 3; i++) {
      d12 = R[i];
      d13 = R[i + 3];
      absxk = R[i + 6];
      for (k = 0; k < 3; k++) {
        a[i + 3 * k] = (d12 * R_dot[3 * k] + d13 * R_dot[3 * k + 1]) +
                       absxk * R_dot[3 * k + 2];
      }
    }
    std::copy(&a[0], &a[9], &R[0]);
    skewMatrix_keith(p_dot, R_dot);
    for (i = 0; i < 9; i++) {
      R_dot[i] = -R_dot[i];
    }
    skewMatrix_keith(u, a);
    for (i = 0; i < 3; i++) {
      d_y[i] = (static_cast<double>(iv1[i]) * d +
                static_cast<double>(iv1[i + 3]) * d1) +
               static_cast<double>(iv1[i + 6]) * d2;
      e_y[i] = (a[i] * 0.0025 + a[i + 3] * 0.0) + a[i + 6] * 0.0;
    }
    skewMatrix_keith(u, a);
    for (i = 0; i < 3; i++) {
      d12 = 0.0;
      d13 = a[i];
      absxk = a[i + 3];
      scale = a[i + 6];
      for (k = 0; k < 3; k++) {
        d12 += ((d13 * static_cast<double>(iv1[3 * k]) +
                 absxk * static_cast<double>(iv1[3 * k + 1])) +
                scale * static_cast<double>(iv1[3 * k + 2])) *
               v[k];
      }
      h_y[i] = d12;
    }
    skewMatrix_keith(u, a);
    for (i = 0; i < 3; i++) {
      f_y[i] = (static_cast<double>(iv1[i]) * d3 +
                static_cast<double>(iv1[i + 3]) * d4) +
               static_cast<double>(iv1[i + 6]) * d5;
      g_y[i] = (a[i] * 0.0 + a[i + 3] * 0.0025) + a[i + 6] * 0.0;
    }
    skewMatrix_keith(u, a);
    for (i = 0; i < 3; i++) {
      d12 = 0.0;
      d13 = a[i];
      absxk = a[i + 3];
      scale = a[i + 6];
      for (k = 0; k < 3; k++) {
        d12 += ((d13 * static_cast<double>(iv1[3 * k]) +
                 absxk * static_cast<double>(iv1[3 * k + 1])) +
                scale * static_cast<double>(iv1[3 * k + 2])) *
               v[k + 3];
      }
      k_y[i] = d12;
    }
    skewMatrix_keith(u, a);
    for (i = 0; i < 3; i++) {
      i_y[i] = (static_cast<double>(iv2[i]) * d6 +
                static_cast<double>(iv2[i + 3]) * d7) +
               static_cast<double>(iv2[i + 6]) * d8;
      j_y[i] = (a[i] * 0.0019 + a[i + 3] * -0.0019) + a[i + 6] * 0.0;
    }
    skewMatrix_keith(u, a);
    for (i = 0; i < 3; i++) {
      d12 = 0.0;
      d13 = a[i];
      absxk = a[i + 3];
      scale = a[i + 6];
      for (k = 0; k < 3; k++) {
        d12 += ((d13 * static_cast<double>(iv2[3 * k]) +
                 absxk * static_cast<double>(iv2[3 * k + 1])) +
                scale * static_cast<double>(iv2[3 * k + 2])) *
               v[k + 6];
      }
      n_y[i] = d12;
    }
    skewMatrix_keith(u, a);
    for (i = 0; i < 3; i++) {
      l_y[i] = (static_cast<double>(iv2[i]) * d9 +
                static_cast<double>(iv2[i + 3]) * d10) +
               static_cast<double>(iv2[i + 6]) * d11;
      m_y[i] = (a[i] * 0.0019 + a[i + 3] * 0.0019) + a[i + 6] * 0.0;
    }
    skewMatrix_keith(u, a);
    cosdelta = 0.0;
    y_data_idx_1 = 0.0;
    for (k = 0; k < 3; k++) {
      d12 = 0.0;
      d13 = a[k];
      absxk = a[k + 3];
      scale = a[k + 6];
      for (i = 0; i < 3; i++) {
        d12 += ((d13 * static_cast<double>(iv2[3 * i]) +
                 absxk * static_cast<double>(iv2[3 * i + 1])) +
                scale * static_cast<double>(iv2[3 * i + 2])) *
               v[i + 9];
      }
      o_y[k] = d12;
      d12 = u[k];
      cosdelta += Q_data[k] * d12;
      y_data_idx_1 += Q_data[k + 3] * d12;
    }
    c_y = e_y[1] * d_y[2] - d_y[1] * e_y[2];
    costheta = d_y[0] * e_y[2] - e_y[0] * d_y[2];
    sintheta = e_y[0] * d_y[1] - d_y[0] * e_y[1];
    scale = 0.0 * h_y[2] - 0.0 * h_y[1];
    absxk = 0.0 * h_y[0] - 0.0025 * h_y[2];
    t = 0.0025 * h_y[1] - 0.0 * h_y[0];
    e_y[0] = g_y[1] * f_y[2] - f_y[1] * g_y[2];
    e_y[1] = f_y[0] * g_y[2] - g_y[0] * f_y[2];
    e_y[2] = g_y[0] * f_y[1] - f_y[0] * g_y[1];
    c_y = (((c_y + scale) + e_y[0]) + (0.0025 * k_y[2] - 0.0 * k_y[1])) * 0.0 +
          ((((j_y[1] * i_y[2] - i_y[1] * j_y[2]) +
             (-0.0019 * n_y[2] - 0.0 * n_y[1])) +
            (m_y[1] * l_y[2] - l_y[1] * m_y[2])) +
           (0.0019 * o_y[2] - 0.0 * o_y[1]));
    costheta =
        (((costheta + absxk) + e_y[1]) + (0.0 * k_y[0] - 0.0 * k_y[2])) * 0.0 +
        ((((i_y[0] * j_y[2] - j_y[0] * i_y[2]) +
           (0.0 * n_y[0] - 0.0019 * n_y[2])) +
          (l_y[0] * m_y[2] - m_y[0] * l_y[2])) +
         (0.0 * o_y[0] - 0.0019 * o_y[2]));
    sintheta =
        (((sintheta + t) + e_y[2]) + (0.0 * k_y[1] - 0.0025 * k_y[0])) * 0.0 +
        ((((j_y[0] * i_y[1] - i_y[0] * j_y[1]) +
           (0.0019 * n_y[1] - -0.0019 * n_y[0])) +
          (m_y[0] * l_y[1] - l_y[0] * m_y[1])) +
         (0.0019 * o_y[1] - 0.0019 * o_y[0]));
    for (i = 0; i < 3; i++) {
      d12 = R[i + 3];
      d13 = R[i + 6];
      b_y[i] = y[b_i + 20 * i] + p_dot[i];
      absxk = R[i];
      b_y[i + 3] = absxk;
      b_y[i + 6] = d12;
      b_y[i + 9] = d13;
      b_y[i + 12] = y[b_i + 20 * (i + 12)];
      b_y[i + 15] =
          y[b_i + 20 * (i + 15)] +
          (((R_dot[i] * y[b_i + 240] + R_dot[i + 3] * y[b_i + 260]) +
            R_dot[i + 6] * y[b_i + 280]) -
           ((2.0 * absxk * c_y + 2.0 * d12 * costheta) + 2.0 * d13 * sintheta) *
               0.0010526315789473684);
    }
    b_y[18] = y[b_i + 360] + cosdelta * 0.0010526315789473684;
    b_y[19] = y[b_i + 380] + y_data_idx_1 * 0.0010526315789473684;
    for (i = 0; i < 20; i++) {
      y[(b_i + 20 * i) + 1] = b_y[i];
    }
  }
}

//
// -----Integral of IVP using difference equation-------
//  for a coninuum robot kinematics script, forward differential equations.
//
//  input1: y0 initial condition
//  input2: v rod elongation strain
//  SegIdx- the segment index (1 or 2) you are integrating
//  MBP- Mechanical properties
//  fe, le- external distributed loads
//  step- integrating step size
// ----------info-----------%
//  ver f1.0
//  by Yuyang Chen
//  date 20200524
// -------------------------------------------------------------------------%
//
// Arguments    : const double b_y0[19]
//                const double ksi[4]
//                double t_data[]
//                int t_size[2]
//                double y[380]
// Return Type  : void
//
void d_odeCosserat_keith(const double b_y0[19], const double ksi[4],
                         double t_data[], int t_size[2], double y[380])
{
  double b_y[20];
  double e_y[19];
  double Kb[9];
  double Ke[9];
  double R[9];
  double R_dot[9];
  double b_R[9];
  double b_cosdelta[9];
  double c_y[3];
  double u[3];
  int k;
  b_y[19] = 0.02;
  b_y[0] = 0.0;
  for (k = 0; k < 18; k++) {
    b_y[k + 1] = (static_cast<double>(k) + 1.0) * 0.0010526315789473684;
  }
  t_size[0] = 20;
  t_size[1] = 1;
  std::copy(&b_y[0], &b_y[20], &t_data[0]);
  std::memset(&Kb[0], 0, 9U * sizeof(double));
  Kb[0] = ksi[1];
  Kb[4] = ksi[2];
  Kb[8] = ksi[3];
  std::memset(&Ke[0], 0, 9U * sizeof(double));
  Ke[0] = 10000.0 * ksi[0];
  Ke[4] = 10000.0 * ksi[0];
  Ke[8] = ksi[0];
  std::memset(&y[0], 0, 380U * sizeof(double));
  for (k = 0; k < 19; k++) {
    y[20 * k] = b_y0[k];
  }
  for (int i{0}; i < 19; i++) {
    double absxk;
    double cosdelta;
    double costheta;
    double d;
    double d1;
    double d2;
    double d_y;
    double scale;
    double sindelta;
    double sintheta;
    double t;
    double v_do;
    int i3;
    // constant-curvature-based evolution
    for (k = 0; k < 3; k++) {
      d = y[i + 20 * (k + 3)];
      b_R[k] = d;
      d1 = y[i + 20 * (k + 6)];
      b_R[k + 3] = d1;
      d2 = y[i + 20 * (k + 9)];
      b_R[k + 6] = d2;
      R_dot[3 * k] = d;
      R_dot[3 * k + 1] = d1;
      R_dot[3 * k + 2] = d2;
    }
    coder::mldivide(Kb, R_dot, R);
    d = y[i + 300];
    d1 = y[i + 320];
    d2 = y[i + 340];
    for (k = 0; k < 3; k++) {
      u[k] = (R[k] * d + R[k + 3] * d1) + R[k + 6] * d2;
    }
    coder::mldivide(Ke, R_dot, R);
    d = y[i + 240];
    d1 = y[i + 260];
    d2 = y[i + 280];
    for (k = 0; k < 3; k++) {
      c_y[k] = (R[k] * d + R[k + 3] * d1) + R[k + 6] * d2;
    }
    v_do = c_y[2] * 0.0010526315789473684;
    scale = 3.3121686421112381E-170;
    absxk = std::abs(u[0]);
    if (absxk > 3.3121686421112381E-170) {
      d_y = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      d_y = t * t;
    }
    absxk = std::abs(u[1]);
    if (absxk > scale) {
      t = scale / absxk;
      d_y = d_y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      d_y += t * t;
    }
    d_y = scale * std::sqrt(d_y);
    absxk = 0.0010526315789473684 * d_y;
    scale = rt_atan2d_snf(u[1], u[0]);
    t = 0.0010526315789473684 * u[2];
    costheta = std::cos(absxk);
    sintheta = std::sin(absxk);
    cosdelta = std::cos(-scale + 1.5707963267948966);
    sindelta = std::sin(-scale + 1.5707963267948966);
    //  cache
    if (absxk != 0.0) {
      d_y = (v_do + 0.0010526315789473684) / absxk;
      c_y[0] = d_y * (cosdelta * (1.0 - costheta));
      c_y[1] = d_y * (sindelta * (costheta - 1.0));
      c_y[2] = d_y * sintheta;
      u[0] = t;
      u[1] = 0.0;
      u[2] = 0.0;
      coder::eul2rotm(u, R);
      scale = sindelta * sindelta;
      absxk = cosdelta * cosdelta;
      b_cosdelta[0] = absxk * costheta + scale;
      b_cosdelta[3] = -sindelta * cosdelta * (costheta - 1.0);
      b_cosdelta[6] = cosdelta * sintheta;
      b_cosdelta[1] = sindelta * cosdelta * (1.0 - costheta);
      b_cosdelta[4] = absxk + costheta * scale;
      b_cosdelta[7] = -sindelta * sintheta;
      b_cosdelta[2] = -cosdelta * sintheta;
      b_cosdelta[5] = sindelta * sintheta;
      b_cosdelta[8] = costheta;
      for (k = 0; k < 3; k++) {
        scale = 0.0;
        absxk = b_cosdelta[k];
        t = b_cosdelta[k + 3];
        d_y = b_cosdelta[k + 6];
        for (i3 = 0; i3 < 3; i3++) {
          int u_tmp;
          u_tmp = k + 3 * i3;
          scale += b_R[u_tmp] * c_y[i3];
          R_dot[u_tmp] =
              (absxk * R[3 * i3] + t * R[3 * i3 + 1]) + d_y * R[3 * i3 + 2];
        }
        u[k] = scale;
      }
    } else {
      c_y[0] = 0.0;
      c_y[1] = 0.0;
      c_y[2] = v_do + 0.0010526315789473684;
      u[0] = t;
      u[1] = 0.0;
      u[2] = 0.0;
      coder::eul2rotm(u, R);
      for (k = 0; k < 3; k++) {
        signed char b_i;
        signed char i1;
        signed char i2;
        scale = 0.0;
        b_i = iv3[k];
        i1 = iv3[k + 3];
        i2 = iv3[k + 6];
        for (i3 = 0; i3 < 3; i3++) {
          int u_tmp;
          u_tmp = k + 3 * i3;
          scale += b_R[u_tmp] * c_y[i3];
          R_dot[u_tmp] = (static_cast<double>(b_i) * R[3 * i3] +
                          static_cast<double>(i1) * R[3 * i3 + 1]) +
                         static_cast<double>(i2) * R[3 * i3 + 2];
        }
        u[k] = scale;
      }
    }
    for (k = 0; k < 3; k++) {
      scale = b_R[k];
      absxk = b_R[k + 3];
      t = b_R[k + 6];
      for (i3 = 0; i3 < 3; i3++) {
        R[k + 3 * i3] = (scale * R_dot[3 * i3] + absxk * R_dot[3 * i3 + 1]) +
                        t * R_dot[3 * i3 + 2];
      }
    }
    std::copy(&R[0], &R[9], &b_R[0]);
    skewMatrix_keith(u, R_dot);
    for (k = 0; k < 9; k++) {
      R_dot[k] = -R_dot[k];
    }
    for (k = 0; k < 3; k++) {
      e_y[k] = y[i + 20 * k] + u[k];
      e_y[k + 3] = b_R[k];
      e_y[k + 6] = b_R[k + 3];
      e_y[k + 9] = b_R[k + 6];
      e_y[k + 12] = y[i + 20 * (k + 12)];
      e_y[k + 15] = y[i + 20 * (k + 15)] +
                    ((R_dot[k] * d + R_dot[k + 3] * d1) + R_dot[k + 6] * d2);
    }
    e_y[18] = y[i + 360] + v_do;
    for (k = 0; k < 19; k++) {
      y[(i + 20 * k) + 1] = e_y[k];
    }
  }
}

//
// -----Integral of IVP using difference equation-------
//  for a coninuum robot kinematics script, forward differential equations.
//
//  input1: y0 initial condition
//  input2: v rod elongation strain
//  SegIdx- the segment index (1 or 2) you are integrating
//  MBP- Mechanical properties
//  fe, le- external distributed loads
//  step- integrating step size
// ----------info-----------%
//  ver f1.0
//  by Yuyang Chen
//  date 20200524
// -------------------------------------------------------------------------%
//
// Arguments    : const double y0_data[]
//                const int y0_size[2]
//                const double v[12]
//                signed char SegIdx
//                const double ksi[4]
//                double t_data[]
//                int t_size[2]
//                double y_data[]
//                int y_size[2]
//                double U_data[]
//                int U_size[2]
// Return Type  : void
//
void odeCosserat_keith(const double y0_data[], const int y0_size[2],
                       const double v[12], signed char SegIdx,
                       const double ksi[4], double t_data[], int t_size[2],
                       double y_data[], int y_size[2], double U_data[],
                       int U_size[2])
{
  static const double b_dv1[9]{
      0.0760216, 0.0, 0.0, 0.0, 0.0760216, 0.0, 0.0, 0.0, 0.057023199999999989};
  static const double b_dv2[9]{0.01520432, 0.0,        0.0,
                               0.0,        0.01520432, 0.0,
                               0.0,        0.0,        0.011404639999999999};
  double y[60];
  double R_data[27];
  double a_tmp_data[27];
  double b_q_data[22];
  double q_data[22];
  double b_dv[20];
  double Q_data[12];
  double C_data[9];
  double Kb[9];
  double Ke[9];
  double R_dot[9];
  double b_y[9];
  double b_y0[9];
  double b_y_data[9];
  double m_data[9];
  double b_C_data[4];
  double B[3];
  double a[3];
  double b[3];
  double b_a[3];
  double b_b[3];
  double b_dv3[3];
  double c_a[3];
  double c_b[3];
  double d_b[3];
  double e_b[3];
  double f_b[3];
  double g_b[3];
  double n[3];
  double p[3];
  double p_dot_data[3];
  double reshapes_f1[3];
  double reshapes_f5[3];
  double u_data[3];
  double step;
  int C_size[2];
  int a_size[2];
  int a_tmp_size[2];
  int m_size[2];
  int q[2];
  int q_size[2];
  int tmp_size[2];
  int N;
  int Q_size_idx_0;
  int Q_size_idx_1;
  int R_size_idx_0;
  int R_size_idx_1;
  int aoffset;
  int i;
  int i1;
  int k;
  int loop_ub;
  int u0;
  int u1;
  signed char Kb_tmp[9];
  signed char mod_SegIdx;
  if ((y0_size[0] == 0) || (y0_size[1] == 0)) {
    u1 = 0;
  } else {
    u0 = y0_size[0];
    u1 = y0_size[1];
    if (u0 >= u1) {
      u1 = u0;
    }
  }
  step = 0.001;
  N = 0;
  mod_SegIdx = 0;
  for (i = 0; i < 9; i++) {
    Kb_tmp[i] = 0;
  }
  Kb_tmp[0] = 1;
  Kb_tmp[4] = 1;
  Kb_tmp[8] = 1;
  for (i = 0; i < 9; i++) {
    u0 = Kb_tmp[i];
    Kb[i] = u0;
    Ke[i] = u0;
  }
  Q_size_idx_0 = 0;
  Q_size_idx_1 = 0;
  t_size[0] = 0;
  t_size[1] = 0;
  if (SegIdx == 1) {
    Q_size_idx_0 = 3;
    Q_size_idx_1 = 4;
    std::copy(&dv9[0], &dv9[12], &Q_data[0]);
    N = 60;
    y[59] = 0.06;
    y[0] = 0.0;
    for (k = 0; k < 58; k++) {
      y[k + 1] = (static_cast<double>(k) + 1.0) * 0.0010169491525423729;
    }
    t_size[0] = 60;
    t_size[1] = 1;
    std::copy(&y[0], &y[60], &t_data[0]);
    mod_SegIdx = 1;
    std::copy(&b_dv2[0], &b_dv2[9], &Kb[0]);
    step = 0.0010169491525423729;
  } else if (SegIdx == 2) {
    Q_size_idx_0 = 3;
    Q_size_idx_1 = 2;
    for (i = 0; i < 6; i++) {
      Q_data[i] = dv10[i];
    }
    N = 20;
    coder::linspace(0.069999999999999993, 0.09, b_dv);
    t_size[0] = 20;
    t_size[1] = 1;
    std::copy(&b_dv[0], &b_dv[20], &t_data[0]);
    std::copy(&dv11[0], &dv11[9], &Kb[0]);
    step = 0.0010526315789473684;
  } else if (SegIdx == 0) {
    Q_size_idx_0 = 3;
    Q_size_idx_1 = 4;
    std::copy(&dv9[0], &dv9[12], &Q_data[0]);
    t_size[0] = 0;
    t_size[1] = 1;
    mod_SegIdx = 1;
    std::copy(&b_dv1[0], &b_dv1[9], &Kb[0]);
    step = -0.0;
  } else if (SegIdx == 3) {
    N = 20;
    coder::linspace(0.0, 0.02, b_dv);
    t_size[0] = 20;
    t_size[1] = 1;
    std::copy(&b_dv[0], &b_dv[20], &t_data[0]);
    mod_SegIdx = 3;
    std::memset(&Kb[0], 0, 9U * sizeof(double));
    Kb[0] = ksi[1];
    Kb[4] = ksi[2];
    Kb[8] = ksi[3];
    std::memset(&Ke[0], 0, 9U * sizeof(double));
    Ke[0] = 10000.0 * ksi[0];
    Ke[4] = 10000.0 * ksi[0];
    Ke[8] = ksi[0];
    step = 0.0010526315789473684;
  }
  y_size[0] = N;
  y_size[1] = u1;
  aoffset = N * u1;
  if (0 <= aoffset - 1) {
    std::memset(&y_data[0], 0, aoffset * sizeof(double));
  }
  U_size[0] = N;
  U_size[1] = 3;
  aoffset = N * 3;
  if (0 <= aoffset - 1) {
    std::memset(&U_data[0], 0, aoffset * sizeof(double));
  }
  aoffset = y0_size[0];
  for (i = 0; i < aoffset; i++) {
    u0 = y0_size[1];
    for (k = 0; k < u0; k++) {
      y_data[N * i] = y0_data[i + y0_size[0] * k];
    }
  }
  b_y0[0] = y0_data[3];
  b_y0[3] = y0_data[6];
  b_y0[6] = y0_data[9];
  b_y0[1] = y0_data[4];
  b_y0[4] = y0_data[7];
  b_y0[7] = y0_data[10];
  b_y0[2] = y0_data[5];
  b_y0[5] = y0_data[8];
  b_y0[8] = y0_data[11];
  R_size_idx_0 = 1;
  R_size_idx_1 = 9;
  std::copy(&b_y0[0], &b_y0[9], &R_data[0]);
  m_size[0] = 1;
  m_size[1] = 3;
  m_data[0] = y0_data[15];
  m_data[1] = y0_data[16];
  m_data[2] = y0_data[17];
  if (0 <= N - 2) {
    R_size_idx_0 = 3;
    if (19 > u1) {
      i1 = 0;
      u1 = 0;
    } else {
      i1 = 18;
    }
    loop_ub = u1 - i1;
  }
  for (int b_i{0}; b_i <= N - 2; b_i++) {
    p[0] = y_data[b_i];
    b_y[0] = y_data[b_i + N * 3];
    b_y[3] = y_data[b_i + N * 6];
    b_y[6] = y_data[b_i + N * 9];
    p[1] = y_data[b_i + N];
    b_y[1] = y_data[b_i + N * 4];
    b_y[4] = y_data[b_i + N * 7];
    b_y[7] = y_data[b_i + N * 10];
    p[2] = y_data[b_i + N * 2];
    b_y[2] = y_data[b_i + N * 5];
    b_y[5] = y_data[b_i + N * 8];
    b_y[8] = y_data[b_i + N * 11];
    std::copy(&b_y[0], &b_y[9], &R_data[0]);
    m_size[0] = 3;
    m_size[1] = 1;
    n[0] = y_data[b_i + N * 12];
    m_data[0] = y_data[b_i + N * 15];
    n[1] = y_data[b_i + N * 13];
    m_data[1] = y_data[b_i + N * 16];
    n[2] = y_data[b_i + N * 14];
    m_data[2] = y_data[b_i + N * 17];
    for (i = 0; i < loop_ub; i++) {
      q_data[i] = y_data[b_i + N * (i1 + i)];
    }
    // constant-curvature-based evolution
    if (SegIdx < 3) {
      double absxk;
      double c_y;
      double cosdelta;
      double costheta;
      double d;
      double d1;
      double d2;
      double d3;
      double scale;
      double sindelta;
      double sintheta;
      double t;
      for (i = 0; i < 3; i++) {
        for (k = 0; k < 3; k++) {
          a_tmp_data[k + 3 * i] = R_data[i + 3 * k];
        }
      }
      q[0] = 3;
      q[1] = 3;
      coder::mldivide(Kb, a_tmp_data, q, b_y0, a_size);
      coder::internal::blas::mtimes(b_y0, a_size, m_data, m_size, b_y_data, q);
      coder::internal::blas::mtimes(b_y0, a_size, m_data, m_size, R_dot,
                                    tmp_size);
      aoffset = 3 * tmp_size[1];
      if (0 <= aoffset - 1) {
        std::copy(&R_dot[0], &R_dot[aoffset], &u_data[0]);
      }
      u_data[2] = 0.0;
      scale = 3.3121686421112381E-170;
      absxk = std::abs(u_data[0]);
      if (absxk > 3.3121686421112381E-170) {
        c_y = 1.0;
        scale = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        c_y = t * t;
      }
      absxk = std::abs(u_data[1]);
      if (absxk > scale) {
        t = scale / absxk;
        c_y = c_y * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        c_y += t * t;
      }
      c_y = scale * std::sqrt(c_y);
      absxk = step * c_y;
      scale = rt_atan2d_snf(u_data[1], u_data[0]);
      costheta = std::cos(absxk);
      sintheta = std::sin(absxk);
      cosdelta = std::cos(-scale + 1.5707963267948966);
      sindelta = std::sin(-scale + 1.5707963267948966);
      //  cache
      if (absxk != 0.0) {
        c_y = step / absxk;
        B[0] = c_y * (cosdelta * (1.0 - costheta));
        B[1] = c_y * (sindelta * (costheta - 1.0));
        B[2] = c_y * sintheta;
        d = 0.0;
        d1 = 0.0;
        d2 = 0.0;
        for (k = 0; k < 3; k++) {
          aoffset = k * 3;
          d3 = B[k];
          d += R_data[aoffset] * d3;
          d1 += R_data[aoffset + 1] * d3;
          d2 += R_data[aoffset + 2] * d3;
        }
        p_dot_data[2] = d2;
        p_dot_data[1] = d1;
        p_dot_data[0] = d;
        scale = sindelta * sindelta;
        absxk = cosdelta * cosdelta;
        R_dot[0] = absxk * costheta + scale;
        R_dot[3] = -sindelta * cosdelta * (costheta - 1.0);
        R_dot[6] = cosdelta * sintheta;
        R_dot[1] = sindelta * cosdelta * (1.0 - costheta);
        R_dot[4] = absxk + costheta * scale;
        R_dot[7] = -sindelta * sintheta;
        R_dot[2] = -cosdelta * sintheta;
        R_dot[5] = sindelta * sintheta;
        R_dot[8] = costheta;
      } else {
        B[0] = 0.0;
        B[1] = 0.0;
        B[2] = step;
        d = 0.0;
        d1 = 0.0;
        d2 = 0.0;
        for (k = 0; k < 3; k++) {
          aoffset = k * 3;
          d3 = B[k];
          d += R_data[aoffset] * d3;
          d1 += R_data[aoffset + 1] * d3;
          d2 += R_data[aoffset + 2] * d3;
        }
        p_dot_data[2] = d2;
        p_dot_data[1] = d1;
        p_dot_data[0] = d;
        for (i = 0; i < 9; i++) {
          R_dot[i] = Kb_tmp[i];
        }
      }
      for (u1 = 0; u1 < 3; u1++) {
        reshapes_f1[u1] = p[u1] + p_dot_data[u1];
        u0 = u1 * 3;
        d = 0.0;
        d1 = 0.0;
        d2 = 0.0;
        for (k = 0; k < 3; k++) {
          aoffset = k * 3;
          scale = R_dot[u0 + k];
          d += R_data[aoffset] * scale;
          d1 += R_data[aoffset + 1] * scale;
          d2 += R_data[aoffset + 2] * scale;
        }
        C_data[u0 + 2] = d2;
        C_data[u0 + 1] = d1;
        C_data[u0] = d;
      }
      R_size_idx_1 = 3;
      std::copy(&C_data[0], &C_data[9], &R_data[0]);
      // this function gives the skew-symmetric matrix of the vector p
      //      if(length(p)==3)
      b_y0[0] = 0.0;
      b_y0[3] = -0.0;
      b_y0[6] = b_y_data[1];
      b_y0[1] = 0.0;
      b_y0[4] = 0.0;
      b_y0[7] = -b_y_data[0];
      b_y0[2] = -b_y_data[1];
      b_y0[5] = b_y_data[0];
      b_y0[8] = 0.0;
      //      elseif(length(p)==6)
      //          R=[0 -p(6) p(5); p(6) 0 -p(4);-p(5) p(4) 0];
      //          T=[R p(1:3);zeros(1,4)];
      //      end
      // this function gives the skew-symmetric matrix of the vector p
      //      if(length(p)==3)
      //      elseif(length(p)==6)
      //          R=[0 -p(6) p(5); p(6) 0 -p(4);-p(5) p(4) 0];
      //          T=[R p(1:3);zeros(1,4)];
      //      end
      for (aoffset = 0; aoffset < 3; aoffset++) {
        double v_do;
        reshapes_f5[aoffset] = n[aoffset];
        d = 0.0;
        d1 = 0.0;
        d2 = 0.0;
        d3 = 0.0;
        absxk = 0.0;
        t = 0.0;
        c_y = 0.0;
        sindelta = 0.0;
        costheta = 0.0;
        sintheta = 0.0;
        cosdelta = 0.0;
        v_do = 0.0;
        for (i = 0; i < 3; i++) {
          double d4;
          double d5;
          double d6;
          short i5;
          d4 = b_y0[aoffset];
          d5 = d4 * static_cast<double>(iv1[3 * i]);
          d6 = d4 * static_cast<double>(iv2[3 * i]);
          d4 = b_y0[aoffset + 3];
          k = 3 * i + 1;
          d5 += d4 * static_cast<double>(iv1[k]);
          d6 += d4 * static_cast<double>(iv2[k]);
          d4 = b_y0[aoffset + 6];
          k = 3 * i + 2;
          d5 += d4 * static_cast<double>(iv1[k]);
          d6 += d4 * static_cast<double>(iv2[k]);
          k = aoffset + 3 * i;
          d4 = b_y0[k];
          d += d4 * dv1[i];
          i5 = iv1[k];
          scale = v[i];
          d1 += static_cast<double>(i5) * scale;
          d2 += d5 * scale;
          d3 += d4 * dv2[i];
          scale = v[i + 3];
          absxk += static_cast<double>(i5) * scale;
          t += d5 * scale;
          c_y += d4 * dv5[i];
          i5 = iv2[k];
          scale = v[i + 6];
          sindelta += static_cast<double>(i5) * scale;
          costheta += d6 * scale;
          sintheta += d4 * dv6[i];
          scale = v[i + 9];
          cosdelta += static_cast<double>(i5) * scale;
          v_do += d6 * scale;
        }
        g_b[aoffset] = v_do;
        f_b[aoffset] = cosdelta;
        c_a[aoffset] = sintheta;
        e_b[aoffset] = costheta;
        c_b[aoffset] = sindelta;
        b_a[aoffset] = c_y;
        d_b[aoffset] = t;
        b_b[aoffset] = absxk;
        a[aoffset] = d3;
        B[aoffset] = d2;
        b[aoffset] = d1;
        p[aoffset] = d;
      }
      for (i = 0; i < 9; i++) {
        C_data[i] *= 2.0;
      }
      scale = p[1] * b[2] - b[1] * p[2];
      absxk = b[0] * p[2] - p[0] * b[2];
      t = p[0] * b[1] - b[0] * p[1];
      b_dv3[0] = 0.0 * B[2] - B[1] * 0.0;
      b_dv3[1] = B[0] * 0.0 - 0.0025 * B[2];
      b_dv3[2] = 0.0025 * B[1] - B[0] * 0.0;
      c_y = a[1] * b_b[2] - b_b[1] * a[2];
      sindelta = b_b[0] * a[2] - a[0] * b_b[2];
      costheta = a[0] * b_b[1] - b_b[0] * a[1];
      a[0] = b_a[1] * c_b[2] - c_b[1] * b_a[2];
      a[1] = c_b[0] * b_a[2] - b_a[0] * c_b[2];
      a[2] = b_a[0] * c_b[1] - c_b[0] * b_a[1];
      B[0] = (((scale + b_dv3[0]) + c_y) + (0.0025 * d_b[2] - d_b[1] * 0.0)) *
                 static_cast<double>(mod_SegIdx) +
             (((a[0] + (-0.0019 * e_b[2] - e_b[1] * 0.0)) +
               (c_a[1] * f_b[2] - f_b[1] * c_a[2])) +
              (0.0019 * g_b[2] - g_b[1] * 0.0));
      B[1] = (((absxk + b_dv3[1]) + sindelta) + (d_b[0] * 0.0 - 0.0 * d_b[2])) *
                 static_cast<double>(mod_SegIdx) +
             (((a[1] + (e_b[0] * 0.0 - 0.0019 * e_b[2])) +
               (f_b[0] * c_a[2] - c_a[0] * f_b[2])) +
              (g_b[0] * 0.0 - 0.0019 * g_b[2]));
      B[2] = (((t + b_dv3[2]) + costheta) + (0.0 * d_b[1] - d_b[0] * 0.0025)) *
                 static_cast<double>(mod_SegIdx) +
             (((a[2] + (0.0019 * e_b[1] - e_b[0] * -0.0019)) +
               (c_a[0] * f_b[1] - f_b[0] * c_a[1])) +
              (0.0019 * g_b[1] - g_b[0] * 0.0019));
      d = 0.0;
      d1 = 0.0;
      d2 = 0.0;
      for (k = 0; k < 3; k++) {
        aoffset = k * 3;
        d3 = B[k];
        d += C_data[aoffset] * d3;
        d1 += C_data[aoffset + 1] * d3;
        d2 += C_data[aoffset + 2] * d3;
      }
      b[2] = d2;
      b[1] = d1;
      b[0] = d;
      b_y0[0] = -0.0;
      b_y0[3] = p_dot_data[2];
      b_y0[6] = -p_dot_data[1];
      b_y0[1] = -p_dot_data[2];
      b_y0[4] = -0.0;
      b_y0[7] = p_dot_data[0];
      b_y0[2] = p_dot_data[1];
      b_y0[5] = -p_dot_data[0];
      b_y0[8] = -0.0;
      d = n[0];
      d1 = n[1];
      d2 = n[2];
      for (i = 0; i < 3; i++) {
        b_dv3[i] =
            ((b_y0[i] * d + b_y0[i + 3] * d1) + b_y0[i + 6] * d2) - b[i] * step;
      }
      m_size[0] = 3;
      m_size[1] = 1;
      for (i = 0; i < 3; i++) {
        m_data[i] += b_dv3[i];
      }
      u0 = Q_size_idx_1 - 1;
      C_size[0] = Q_size_idx_1;
      C_size[1] = 1;
      if (0 <= u0) {
        std::memset(&b_C_data[0], 0, (u0 + 1) * sizeof(double));
      }
      for (k = 0; k < Q_size_idx_0; k++) {
        for (aoffset = 0; aoffset <= u0; aoffset++) {
          b_C_data[aoffset] += Q_data[aoffset * Q_size_idx_0 + k] * u_data[k];
        }
      }
      if (loop_ub == Q_size_idx_1) {
        q_size[0] = loop_ub;
        q_size[1] = 1;
        for (i = 0; i < loop_ub; i++) {
          b_q_data[i] = q_data[i] + b_C_data[i] * step;
        }
      } else {
        binary_expand_op(b_q_data, q_size, q_data, &loop_ub, b_C_data, C_size,
                         step);
      }
    } else {
      double absxk;
      double c_y;
      double cosdelta;
      double costheta;
      double d;
      double d1;
      double d2;
      double scale;
      double sindelta;
      double sintheta;
      double t;
      double v_do;
      for (i = 0; i < 3; i++) {
        for (k = 0; k < 3; k++) {
          a_tmp_data[k + 3 * i] = R_data[i + 3 * k];
        }
      }
      q[0] = 3;
      q[1] = 3;
      coder::mldivide(Kb, a_tmp_data, q, b_y0, a_size);
      coder::internal::blas::mtimes(b_y0, a_size, m_data, m_size, b_y_data, q);
      coder::internal::blas::mtimes(b_y0, a_size, m_data, m_size, R_dot,
                                    tmp_size);
      aoffset = 3 * tmp_size[1];
      if (0 <= aoffset - 1) {
        std::copy(&R_dot[0], &R_dot[aoffset], &u_data[0]);
      }
      q[0] = 3;
      q[1] = 3;
      coder::mldivide(Ke, a_tmp_data, q, R_dot, tmp_size);
      d = n[0];
      d1 = n[1];
      d2 = n[2];
      for (i = 0; i < 3; i++) {
        b_dv3[i] = (R_dot[i] * d + R_dot[i + 3] * d1) + R_dot[i + 6] * d2;
      }
      v_do = b_dv3[2] * step;
      scale = 3.3121686421112381E-170;
      absxk = std::abs(b_y_data[0]);
      if (absxk > 3.3121686421112381E-170) {
        c_y = 1.0;
        scale = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        c_y = t * t;
      }
      absxk = std::abs(b_y_data[1]);
      if (absxk > scale) {
        t = scale / absxk;
        c_y = c_y * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        c_y += t * t;
      }
      c_y = scale * std::sqrt(c_y);
      absxk = step * c_y;
      scale = rt_atan2d_snf(u_data[1], u_data[0]);
      t = step * b_y_data[2];
      costheta = std::cos(absxk);
      sintheta = std::sin(absxk);
      cosdelta = std::cos(-scale + 1.5707963267948966);
      sindelta = std::sin(-scale + 1.5707963267948966);
      //  cache
      if (absxk != 0.0) {
        c_y = (step + v_do) / absxk;
        B[0] = c_y * (cosdelta * (1.0 - costheta));
        B[1] = c_y * (sindelta * (costheta - 1.0));
        B[2] = c_y * sintheta;
        d = 0.0;
        d1 = 0.0;
        d2 = 0.0;
        for (k = 0; k < 3; k++) {
          double d3;
          aoffset = k * 3;
          d3 = B[k];
          d += R_data[aoffset] * d3;
          d1 += R_data[aoffset + 1] * d3;
          d2 += R_data[aoffset + 2] * d3;
        }
        p_dot_data[2] = d2;
        p_dot_data[1] = d1;
        p_dot_data[0] = d;
        B[0] = t;
        B[1] = 0.0;
        B[2] = 0.0;
        coder::eul2rotm(B, b_y0);
        b_y[0] = cosdelta * cosdelta * costheta + sindelta * sindelta;
        b_y[3] = -sindelta * cosdelta * (costheta - 1.0);
        b_y[6] = cosdelta * sintheta;
        b_y[1] = sindelta * cosdelta * (1.0 - costheta);
        b_y[4] = cosdelta * cosdelta + costheta * (sindelta * sindelta);
        b_y[7] = -sindelta * sintheta;
        b_y[2] = -cosdelta * sintheta;
        b_y[5] = sindelta * sintheta;
        b_y[8] = costheta;
        for (i = 0; i < 3; i++) {
          d = b_y[i];
          d1 = b_y[i + 3];
          d2 = b_y[i + 6];
          for (k = 0; k < 3; k++) {
            R_dot[i + 3 * k] =
                (d * b_y0[3 * k] + d1 * b_y0[3 * k + 1]) + d2 * b_y0[3 * k + 2];
          }
        }
      } else {
        B[0] = 0.0;
        B[1] = 0.0;
        B[2] = step + v_do;
        d = 0.0;
        d1 = 0.0;
        d2 = 0.0;
        for (k = 0; k < 3; k++) {
          double d3;
          aoffset = k * 3;
          d3 = B[k];
          d += R_data[aoffset] * d3;
          d1 += R_data[aoffset + 1] * d3;
          d2 += R_data[aoffset + 2] * d3;
        }
        p_dot_data[2] = d2;
        p_dot_data[1] = d1;
        p_dot_data[0] = d;
        B[0] = t;
        B[1] = 0.0;
        B[2] = 0.0;
        coder::eul2rotm(B, b_y0);
        for (i = 0; i < 3; i++) {
          signed char i2;
          signed char i3;
          signed char i4;
          i2 = iv3[i];
          i3 = iv3[i + 3];
          i4 = iv3[i + 6];
          for (k = 0; k < 3; k++) {
            R_dot[i + 3 * k] = (static_cast<double>(i2) * b_y0[3 * k] +
                                static_cast<double>(i3) * b_y0[3 * k + 1]) +
                               static_cast<double>(i4) * b_y0[3 * k + 2];
          }
        }
      }
      for (u1 = 0; u1 < 3; u1++) {
        reshapes_f1[u1] = p[u1] + p_dot_data[u1];
        u0 = u1 * 3;
        d = 0.0;
        d1 = 0.0;
        d2 = 0.0;
        for (k = 0; k < 3; k++) {
          aoffset = k * 3;
          scale = R_dot[u0 + k];
          d += R_data[aoffset] * scale;
          d1 += R_data[aoffset + 1] * scale;
          d2 += R_data[aoffset + 2] * scale;
        }
        C_data[u0 + 2] = d2;
        C_data[u0 + 1] = d1;
        C_data[u0] = d;
      }
      R_size_idx_1 = 3;
      std::copy(&C_data[0], &C_data[9], &R_data[0]);
      // this function gives the skew-symmetric matrix of the vector p
      //      if(length(p)==3)
      //      elseif(length(p)==6)
      //          R=[0 -p(6) p(5); p(6) 0 -p(4);-p(5) p(4) 0];
      //          T=[R p(1:3);zeros(1,4)];
      //      end
      reshapes_f5[0] = n[0];
      reshapes_f5[1] = n[1];
      reshapes_f5[2] = n[2];
      b_y0[0] = -0.0;
      b_y0[3] = p_dot_data[2];
      b_y0[6] = -p_dot_data[1];
      b_y0[1] = -p_dot_data[2];
      b_y0[4] = -0.0;
      b_y0[7] = p_dot_data[0];
      b_y0[2] = p_dot_data[1];
      b_y0[5] = -p_dot_data[0];
      b_y0[8] = -0.0;
      d = n[0];
      d1 = n[1];
      d2 = n[2];
      for (i = 0; i < 3; i++) {
        b_dv3[i] = (b_y0[i] * d + b_y0[i + 3] * d1) + b_y0[i + 6] * d2;
      }
      m_size[0] = 3;
      m_size[1] = 1;
      for (i = 0; i < 3; i++) {
        m_data[i] += b_dv3[i];
      }
      q_size[0] = loop_ub;
      q_size[1] = 1;
      for (i = 0; i < loop_ub; i++) {
        b_q_data[i] = q_data[i] + v_do;
      }
    }
    y_data[b_i + 1] = reshapes_f1[0];
    y_data[(b_i + N * 3) + 1] = R_data[0];
    y_data[(b_i + N * 6) + 1] = R_data[3];
    y_data[(b_i + N * 9) + 1] = R_data[6];
    y_data[(b_i + N * 12) + 1] = reshapes_f5[0];
    y_data[(b_i + N * 15) + 1] = m_data[0];
    y_data[(b_i + N) + 1] = reshapes_f1[1];
    y_data[(b_i + N * 4) + 1] = R_data[1];
    y_data[(b_i + N * 7) + 1] = R_data[4];
    y_data[(b_i + N * 10) + 1] = R_data[7];
    y_data[(b_i + N * 13) + 1] = reshapes_f5[1];
    y_data[(b_i + N * 16) + 1] = m_data[1];
    y_data[(b_i + N * 2) + 1] = reshapes_f1[2];
    y_data[(b_i + N * 5) + 1] = R_data[2];
    y_data[(b_i + N * 8) + 1] = R_data[5];
    y_data[(b_i + N * 11) + 1] = R_data[8];
    y_data[(b_i + N * 14) + 1] = reshapes_f5[2];
    y_data[(b_i + N * 17) + 1] = m_data[2];
    if (q_size[0] != 0) {
      aoffset = static_cast<signed char>(q_size[0]);
    } else {
      aoffset = 0;
    }
    for (i = 0; i < aoffset; i++) {
      y_data[(b_i + N * (i + 18)) + 1] = b_q_data[i];
    }
    U_data[b_i] = u_data[0];
    U_data[b_i + N] = u_data[1];
    U_data[b_i + N * 2] = u_data[2];
  }
  a_tmp_size[0] = R_size_idx_1;
  a_tmp_size[1] = R_size_idx_0;
  for (i = 0; i < R_size_idx_0; i++) {
    for (k = 0; k < R_size_idx_1; k++) {
      a_tmp_data[k + R_size_idx_1 * i] = R_data[i + R_size_idx_0 * k];
    }
  }
  coder::mldivide(Kb, a_tmp_data, a_tmp_size, R_dot, tmp_size);
  coder::internal::blas::mtimes(R_dot, tmp_size, m_data, m_size, b_y_data, q);
  U_data[N - 1] = b_y_data[0];
  U_data[(N + N) - 1] = b_y_data[1];
  U_data[(N + N * 2) - 1] = b_y_data[2];
}

//
// File trailer for odeCosserat_keith.cpp
//
// [EOF]
//
