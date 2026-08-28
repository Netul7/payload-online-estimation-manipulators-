//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Observation_matrix_Franka.cpp
//
// Code generation for function 'Observation_matrix_Franka'
//

// Include files
#include "Observation_matrix_Franka.h"
#include "inv.h"
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
void Observation_matrix_Franka(const double In[21], double K[560])
{
  static const double b[36]{1.0, 0.0,   0.0, 0.0, 0.0, 0.0, 0.0,    1.0, 0.0,
                            0.0, 0.0,   0.0, 0.0, 0.0, 1.0, 0.0,    0.0, 0.0,
                            0.0, 0.107, 0.0, 1.0, 0.0, 0.0, -0.107, 0.0, 0.0,
                            0.0, 1.0,   0.0, 0.0, 0.0, 0.0, 0.0,    0.0, 1.0};
  static const double b_a[9]{0.0, 0.0,    0.316, -0.0, 0.0,
                             0.0, -0.316, -0.0,  0.0};
  static const double c_a[9]{0.0,    0.0, -0.0,    -0.0, 0.0,
                             0.0825, 0.0, -0.0825, 0.0};
  static const double d_a[9]{0.0,     0.0,   -0.384, -0.0, 0.0,
                             -0.0825, 0.384, 0.0825, 0.0};
  static const double e_a[9]{0.0,   0.0, -0.0,   -0.0, 0.0,
                             0.088, 0.0, -0.088, 0.0};
  static const signed char iv[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
  static const signed char i_a[6]{0, 0, 1, 0, 0, 0};
  static const signed char a[3]{0, 0, 1};
  __m128d r;
  __m128d r1;
  __m128d r2;
  double A22[60];
  double A33[60];
  double A44[60];
  double A55[60];
  double A66[60];
  double A77[60];
  double A88[60];
  double dv1[60];
  double X12F2[36];
  double X13F3_tmp[36];
  double X23F3[36];
  double X24F4[36];
  double X25F5[36];
  double X26F6[36];
  double X27F7[36];
  double X34F4[36];
  double X35F5[36];
  double X36F6[36];
  double X37F7[36];
  double X45F5[36];
  double X46F6[36];
  double X47F7[36];
  double X56F6[36];
  double X57F7[36];
  double X67F7[36];
  double b_T12[36];
  double b_Sw22[18];
  double b_dw22[18];
  double b_w22[18];
  double T01[16];
  double T12[16];
  double T23[16];
  double T34[16];
  double T45[16];
  double T56[16];
  double T67[16];
  double ab_a[10];
  double bb_a[10];
  double cb_a[10];
  double db_a[10];
  double eb_a[10];
  double fb_a[10];
  double gb_a[10];
  double hb_a[10];
  double ib_a[10];
  double jb_a[10];
  double kb_a[10];
  double lb_a[10];
  double mb_a[10];
  double n_a[10];
  double nb_a[10];
  double o_a[10];
  double ob_a[10];
  double p_a[10];
  double pb_a[10];
  double q_a[10];
  double qb_a[10];
  double r_a[10];
  double rb_a[10];
  double s_a[10];
  double t_a[10];
  double u_a[10];
  double v_a[10];
  double w_a[10];
  double x_a[10];
  double y_a[10];
  double Sw11[9];
  double Sw22[9];
  double c_Sw22[9];
  double dv[9];
  double ddq[8];
  double dq[8];
  double q[8];
  double f_a[6];
  double g_a[6];
  double h_a[6];
  double j_a[6];
  double k_a[6];
  double l_a[6];
  double m_a[6];
  double b_w11_tmp[3];
  double dv11[3];
  double dv22[3];
  double dv33[3];
  double dv44[3];
  double dv55[3];
  double dv66[3];
  double dv77[3];
  double dv88[3];
  double dw22[3];
  double dw33[3];
  double dw44[3];
  double dw55[3];
  double dw66[3];
  double dw77[3];
  double dw88[3];
  double w11[3];
  double w11_tmp[3];
  double w22[3];
  double w33[3];
  double w44[3];
  double w55[3];
  double w66[3];
  double w77[3];
  double w88[3];
  double c1;
  double c2;
  double c3;
  double c4;
  double c5;
  double c6;
  double c7;
  double s1;
  double s2;
  double s3;
  double s4;
  double s5;
  double s6;
  double s7;
  int K_tmp;
  int T12_tmp;
  int a_tmp;
  int b_K_tmp;
  int b_i;
  // global d1 d2 d3 d4 aa1 aa2 aa3 g
  //  [m]
  //  [m/s^2]
  std::memset(&q[0], 0, 8U * sizeof(double));
  std::memset(&dq[0], 0, 8U * sizeof(double));
  std::memset(&ddq[0], 0, 8U * sizeof(double));
  for (int i{0}; i < 7; i++) {
    q[i] = In[i];
    dq[i] = In[i + 7];
    ddq[i] = In[i + 14];
  }
  s1 = std::sin(q[0]);
  c1 = std::cos(q[0]);
  s2 = std::sin(q[1]);
  c2 = std::cos(q[1]);
  s3 = std::sin(q[2]);
  c3 = std::cos(q[2]);
  s4 = std::sin(q[3]);
  c4 = std::cos(q[3]);
  s5 = std::sin(q[4]);
  c5 = std::cos(q[4]);
  s6 = std::sin(q[5]);
  c6 = std::cos(q[5]);
  s7 = std::sin(q[6]);
  c7 = std::cos(q[6]);
  T01[0] = c1;
  T01[4] = -s1;
  T01[8] = 0.0;
  T01[12] = 0.0;
  T01[1] = s1;
  T01[5] = c1;
  T01[9] = 0.0;
  T01[13] = 0.0;
  // Handbook
  T12[0] = c2;
  T12[4] = -s2;
  T12[8] = 0.0;
  T12[12] = 0.0;
  T12[2] = -s2;
  T12[6] = -c2;
  T12[10] = 0.0;
  T12[14] = 0.0;
  T23[0] = c3;
  T23[4] = -s3;
  T23[8] = 0.0;
  T23[12] = 0.0;
  T23[2] = s3;
  T23[6] = c3;
  T23[10] = 0.0;
  T23[14] = 0.0;
  T34[0] = c4;
  T34[4] = -s4;
  T34[8] = 0.0;
  T34[12] = 0.0825;
  T34[2] = s4;
  T34[6] = c4;
  T34[10] = 0.0;
  T34[14] = 0.0;
  T45[0] = c5;
  T45[4] = -s5;
  T45[8] = 0.0;
  T45[12] = -0.0825;
  T45[2] = -s5;
  T45[6] = -c5;
  T45[10] = 0.0;
  T45[14] = 0.0;
  T56[0] = c6;
  T56[4] = -s6;
  T56[8] = 0.0;
  T56[12] = 0.0;
  T56[2] = s6;
  T56[6] = c6;
  T56[10] = 0.0;
  T56[14] = 0.0;
  T67[0] = c7;
  T67[4] = -s7;
  T67[8] = 0.0;
  T67[12] = 0.088;
  T67[2] = s7;
  T67[6] = c7;
  T67[10] = 0.0;
  T67[14] = 0.0;
  T01[2] = 0.0;
  T01[3] = 0.0;
  T12[1] = 0.0;
  T12[3] = 0.0;
  T23[1] = 0.0;
  T23[3] = 0.0;
  T34[1] = 0.0;
  T34[3] = 0.0;
  T45[1] = 0.0;
  T45[3] = 0.0;
  T56[1] = 0.0;
  T56[3] = 0.0;
  T67[1] = 0.0;
  T67[3] = 0.0;
  T01[6] = 0.0;
  T01[7] = 0.0;
  T12[5] = 0.0;
  T12[7] = 0.0;
  T23[5] = 0.0;
  T23[7] = 0.0;
  T34[5] = 0.0;
  T34[7] = 0.0;
  T45[5] = 0.0;
  T45[7] = 0.0;
  T56[5] = 0.0;
  T56[7] = 0.0;
  T67[5] = 0.0;
  T67[7] = 0.0;
  T01[10] = 1.0;
  T01[11] = 0.0;
  T12[9] = 1.0;
  T12[11] = 0.0;
  T23[9] = -1.0;
  T23[11] = 0.0;
  T34[9] = -1.0;
  T34[11] = 0.0;
  T45[9] = 1.0;
  T45[11] = 0.0;
  T56[9] = -1.0;
  T56[11] = 0.0;
  T67[9] = -1.0;
  T67[11] = 0.0;
  T01[14] = 0.333;
  T01[15] = 1.0;
  T12[13] = 0.0;
  T12[15] = 1.0;
  T23[13] = -0.316;
  T23[15] = 1.0;
  T34[13] = 0.0;
  T34[15] = 1.0;
  T45[13] = 0.384;
  T45[15] = 1.0;
  T56[13] = 0.0;
  T56[15] = 1.0;
  T67[13] = 0.0;
  T67[15] = 1.0;
  // null matrix 3*3
  // null matrix 3*6
  // null matrix 6*10
  // --------------------------------------------------------------------------
  // w, dw, dv and ddd
  // --------------------------------------------------------------------------
  // dddi=dvi-g;
  //  i = 1:
  for (int i{0}; i < 3; i++) {
    Sw22[3 * i] = T01[i];
    Sw22[3 * i + 1] = T01[i + 4];
    Sw22[3 * i + 2] = T01[i + 8];
  }
  for (b_i = 0; b_i < 3; b_i++) {
    s1 = (Sw22[b_i] * 0.0 + Sw22[b_i + 3] * 0.0) + Sw22[b_i + 6] * 0.0;
    w11_tmp[b_i] = s1;
    c1 = static_cast<double>(a[b_i]) * dq[0];
    dv88[b_i] = c1;
    w11[b_i] = s1 + c1;
  }
  s2 = (w11_tmp[0] + (w11_tmp[1] * dv88[2] - dv88[1] * w11_tmp[2])) +
       0.0 * ddq[0];
  c2 = (w11_tmp[1] + (dv88[0] * w11_tmp[2] - w11_tmp[0] * dv88[2])) +
       0.0 * ddq[0];
  s3 = (w11_tmp[2] + (w11_tmp[0] * dv88[1] - dv88[0] * w11_tmp[1])) + ddq[0];
  r = _mm_loadu_pd(&Sw22[0]);
  r = _mm_mul_pd(r, _mm_set1_pd(0.0));
  r1 = _mm_loadu_pd(&Sw22[3]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(0.0));
  r = _mm_add_pd(r, r1);
  r1 = _mm_loadu_pd(&Sw22[6]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(9.81));
  r = _mm_add_pd(r, r1);
  _mm_storeu_pd(&dv11[0], r);
  dv11[2] = (Sw22[2] * 0.0 + Sw22[5] * 0.0) + Sw22[8] * 9.81;
  //  i = 2:
  for (int i{0}; i < 3; i++) {
    Sw22[3 * i] = T12[i];
    Sw22[3 * i + 1] = T12[i + 4];
    Sw22[3 * i + 2] = T12[i + 8];
  }
  s1 = w11[0];
  c1 = w11[1];
  c3 = w11[2];
  for (b_i = 0; b_i < 3; b_i++) {
    s4 = static_cast<double>(a[b_i]) * dq[1];
    dv88[b_i] = s4;
    c4 = Sw22[b_i];
    s5 = c4 * s1;
    c5 = c4 * s2;
    c4 = Sw22[b_i + 3];
    s5 += c4 * c1;
    c5 += c4 * c2;
    c4 = Sw22[b_i + 6];
    s5 += c4 * c3;
    c5 += c4 * s3;
    dw22[b_i] = c5;
    w11_tmp[b_i] = s5;
    w22[b_i] = s5 + s4;
  }
  dw22[0] =
      (dw22[0] + (w11_tmp[1] * dv88[2] - dv88[1] * w11_tmp[2])) + 0.0 * ddq[1];
  dw22[1] =
      (dw22[1] + (dv88[0] * w11_tmp[2] - w11_tmp[0] * dv88[2])) + 0.0 * ddq[1];
  dw22[2] = (dw22[2] + (w11_tmp[0] * dv88[1] - dv88[0] * w11_tmp[1])) + ddq[1];
  w11_tmp[0] = w11[1] * 0.0 - w11[2] * 0.0;
  w11_tmp[1] = w11[2] * 0.0 - w11[0] * 0.0;
  w11_tmp[2] = w11[0] * 0.0 - w11[1] * 0.0;
  dv88[0] = c2 * 0.0 - s3 * 0.0;
  dv88[1] = s3 * 0.0 - s2 * 0.0;
  dv88[2] = s2 * 0.0 - c2 * 0.0;
  b_w11_tmp[0] = w11[1] * w11_tmp[2] - w11_tmp[1] * w11[2];
  b_w11_tmp[1] = w11_tmp[0] * w11[2] - w11[0] * w11_tmp[2];
  b_w11_tmp[2] = w11[0] * w11_tmp[1] - w11_tmp[0] * w11[1];
  r = _mm_loadu_pd(&dv88[0]);
  r1 = _mm_loadu_pd(&b_w11_tmp[0]);
  r2 = _mm_loadu_pd(&dv11[0]);
  _mm_storeu_pd(&dv88[0], _mm_add_pd(_mm_add_pd(r, r1), r2));
  dv88[2] = (dv88[2] + b_w11_tmp[2]) + dv11[2];
  s1 = dv88[0];
  c1 = dv88[1];
  c3 = dv88[2];
  r = _mm_loadu_pd(&Sw22[0]);
  r = _mm_mul_pd(r, _mm_set1_pd(s1));
  r1 = _mm_loadu_pd(&Sw22[3]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(c1));
  r = _mm_add_pd(r, r1);
  r1 = _mm_loadu_pd(&Sw22[6]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(c3));
  r = _mm_add_pd(r, r1);
  _mm_storeu_pd(&dv22[0], r);
  dv22[2] = (Sw22[2] * s1 + Sw22[5] * c1) + Sw22[8] * c3;
  //  i = 3:
  for (int i{0}; i < 3; i++) {
    Sw22[3 * i] = T23[i];
    Sw22[3 * i + 1] = T23[i + 4];
    Sw22[3 * i + 2] = T23[i + 8];
  }
  s1 = w22[0];
  c1 = w22[1];
  c3 = w22[2];
  s4 = dw22[0];
  c4 = dw22[1];
  s5 = dw22[2];
  for (b_i = 0; b_i < 3; b_i++) {
    c5 = static_cast<double>(a[b_i]) * dq[2];
    dv88[b_i] = c5;
    s6 = Sw22[b_i];
    c6 = s6 * s1;
    s7 = s6 * s4;
    s6 = Sw22[b_i + 3];
    c6 += s6 * c1;
    s7 += s6 * c4;
    s6 = Sw22[b_i + 6];
    c6 += s6 * c3;
    s7 += s6 * s5;
    dw33[b_i] = s7;
    w11_tmp[b_i] = c6;
    w33[b_i] = c6 + c5;
  }
  dw33[0] =
      (dw33[0] + (w11_tmp[1] * dv88[2] - dv88[1] * w11_tmp[2])) + 0.0 * ddq[2];
  dw33[1] =
      (dw33[1] + (dv88[0] * w11_tmp[2] - w11_tmp[0] * dv88[2])) + 0.0 * ddq[2];
  dw33[2] = (dw33[2] + (w11_tmp[0] * dv88[1] - dv88[0] * w11_tmp[1])) + ddq[2];
  w11_tmp[0] = w22[1] * 0.0 - w22[2] * -0.316;
  w11_tmp[1] = w22[2] * 0.0 - w22[0] * 0.0;
  w11_tmp[2] = w22[0] * -0.316 - w22[1] * 0.0;
  dv88[0] = dw22[1] * 0.0 - dw22[2] * -0.316;
  dv88[1] = dw22[2] * 0.0 - dw22[0] * 0.0;
  dv88[2] = dw22[0] * -0.316 - dw22[1] * 0.0;
  b_w11_tmp[0] = w22[1] * w11_tmp[2] - w11_tmp[1] * w22[2];
  b_w11_tmp[1] = w11_tmp[0] * w22[2] - w22[0] * w11_tmp[2];
  b_w11_tmp[2] = w22[0] * w11_tmp[1] - w11_tmp[0] * w22[1];
  r = _mm_loadu_pd(&dv88[0]);
  r1 = _mm_loadu_pd(&b_w11_tmp[0]);
  r2 = _mm_loadu_pd(&dv22[0]);
  _mm_storeu_pd(&dv88[0], _mm_add_pd(_mm_add_pd(r, r1), r2));
  dv88[2] = (dv88[2] + b_w11_tmp[2]) + dv22[2];
  s1 = dv88[0];
  c1 = dv88[1];
  c3 = dv88[2];
  r = _mm_loadu_pd(&Sw22[0]);
  r = _mm_mul_pd(r, _mm_set1_pd(s1));
  r1 = _mm_loadu_pd(&Sw22[3]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(c1));
  r = _mm_add_pd(r, r1);
  r1 = _mm_loadu_pd(&Sw22[6]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(c3));
  r = _mm_add_pd(r, r1);
  _mm_storeu_pd(&dv33[0], r);
  dv33[2] = (Sw22[2] * s1 + Sw22[5] * c1) + Sw22[8] * c3;
  //  i = 4:
  for (int i{0}; i < 3; i++) {
    Sw22[3 * i] = T34[i];
    Sw22[3 * i + 1] = T34[i + 4];
    Sw22[3 * i + 2] = T34[i + 8];
  }
  s1 = w33[0];
  c1 = w33[1];
  c3 = w33[2];
  s4 = dw33[0];
  c4 = dw33[1];
  s5 = dw33[2];
  for (b_i = 0; b_i < 3; b_i++) {
    c5 = static_cast<double>(a[b_i]) * dq[3];
    dv88[b_i] = c5;
    s6 = Sw22[b_i];
    c6 = s6 * s1;
    s7 = s6 * s4;
    s6 = Sw22[b_i + 3];
    c6 += s6 * c1;
    s7 += s6 * c4;
    s6 = Sw22[b_i + 6];
    c6 += s6 * c3;
    s7 += s6 * s5;
    dw44[b_i] = s7;
    w11_tmp[b_i] = c6;
    w44[b_i] = c6 + c5;
  }
  dw44[0] =
      (dw44[0] + (w11_tmp[1] * dv88[2] - dv88[1] * w11_tmp[2])) + 0.0 * ddq[3];
  dw44[1] =
      (dw44[1] + (dv88[0] * w11_tmp[2] - w11_tmp[0] * dv88[2])) + 0.0 * ddq[3];
  dw44[2] = (dw44[2] + (w11_tmp[0] * dv88[1] - dv88[0] * w11_tmp[1])) + ddq[3];
  w11_tmp[0] = w33[1] * 0.0 - w33[2] * 0.0;
  w11_tmp[1] = w33[2] * 0.0825 - w33[0] * 0.0;
  w11_tmp[2] = w33[0] * 0.0 - w33[1] * 0.0825;
  dv88[0] = dw33[1] * 0.0 - dw33[2] * 0.0;
  dv88[1] = dw33[2] * 0.0825 - dw33[0] * 0.0;
  dv88[2] = dw33[0] * 0.0 - dw33[1] * 0.0825;
  b_w11_tmp[0] = w33[1] * w11_tmp[2] - w11_tmp[1] * w33[2];
  b_w11_tmp[1] = w11_tmp[0] * w33[2] - w33[0] * w11_tmp[2];
  b_w11_tmp[2] = w33[0] * w11_tmp[1] - w11_tmp[0] * w33[1];
  r = _mm_loadu_pd(&dv88[0]);
  r1 = _mm_loadu_pd(&b_w11_tmp[0]);
  r2 = _mm_loadu_pd(&dv33[0]);
  _mm_storeu_pd(&dv88[0], _mm_add_pd(_mm_add_pd(r, r1), r2));
  dv88[2] = (dv88[2] + b_w11_tmp[2]) + dv33[2];
  s1 = dv88[0];
  c1 = dv88[1];
  c3 = dv88[2];
  r = _mm_loadu_pd(&Sw22[0]);
  r = _mm_mul_pd(r, _mm_set1_pd(s1));
  r1 = _mm_loadu_pd(&Sw22[3]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(c1));
  r = _mm_add_pd(r, r1);
  r1 = _mm_loadu_pd(&Sw22[6]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(c3));
  r = _mm_add_pd(r, r1);
  _mm_storeu_pd(&dv44[0], r);
  dv44[2] = (Sw22[2] * s1 + Sw22[5] * c1) + Sw22[8] * c3;
  //  i = 5:
  for (int i{0}; i < 3; i++) {
    Sw22[3 * i] = T45[i];
    Sw22[3 * i + 1] = T45[i + 4];
    Sw22[3 * i + 2] = T45[i + 8];
  }
  s1 = w44[0];
  c1 = w44[1];
  c3 = w44[2];
  s4 = dw44[0];
  c4 = dw44[1];
  s5 = dw44[2];
  for (b_i = 0; b_i < 3; b_i++) {
    c5 = static_cast<double>(a[b_i]) * dq[4];
    dv88[b_i] = c5;
    s6 = Sw22[b_i];
    c6 = s6 * s1;
    s7 = s6 * s4;
    s6 = Sw22[b_i + 3];
    c6 += s6 * c1;
    s7 += s6 * c4;
    s6 = Sw22[b_i + 6];
    c6 += s6 * c3;
    s7 += s6 * s5;
    dw55[b_i] = s7;
    w11_tmp[b_i] = c6;
    w55[b_i] = c6 + c5;
  }
  dw55[0] =
      (dw55[0] + (w11_tmp[1] * dv88[2] - dv88[1] * w11_tmp[2])) + 0.0 * ddq[4];
  dw55[1] =
      (dw55[1] + (dv88[0] * w11_tmp[2] - w11_tmp[0] * dv88[2])) + 0.0 * ddq[4];
  dw55[2] = (dw55[2] + (w11_tmp[0] * dv88[1] - dv88[0] * w11_tmp[1])) + ddq[4];
  w11_tmp[0] = w44[1] * 0.0 - w44[2] * 0.384;
  w11_tmp[1] = w44[2] * -0.0825 - w44[0] * 0.0;
  w11_tmp[2] = w44[0] * 0.384 - w44[1] * -0.0825;
  dv88[0] = dw44[1] * 0.0 - dw44[2] * 0.384;
  dv88[1] = dw44[2] * -0.0825 - dw44[0] * 0.0;
  dv88[2] = dw44[0] * 0.384 - dw44[1] * -0.0825;
  b_w11_tmp[0] = w44[1] * w11_tmp[2] - w11_tmp[1] * w44[2];
  b_w11_tmp[1] = w11_tmp[0] * w44[2] - w44[0] * w11_tmp[2];
  b_w11_tmp[2] = w44[0] * w11_tmp[1] - w11_tmp[0] * w44[1];
  r = _mm_loadu_pd(&dv88[0]);
  r1 = _mm_loadu_pd(&b_w11_tmp[0]);
  r2 = _mm_loadu_pd(&dv44[0]);
  _mm_storeu_pd(&dv88[0], _mm_add_pd(_mm_add_pd(r, r1), r2));
  dv88[2] = (dv88[2] + b_w11_tmp[2]) + dv44[2];
  s1 = dv88[0];
  c1 = dv88[1];
  c3 = dv88[2];
  r = _mm_loadu_pd(&Sw22[0]);
  r = _mm_mul_pd(r, _mm_set1_pd(s1));
  r1 = _mm_loadu_pd(&Sw22[3]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(c1));
  r = _mm_add_pd(r, r1);
  r1 = _mm_loadu_pd(&Sw22[6]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(c3));
  r = _mm_add_pd(r, r1);
  _mm_storeu_pd(&dv55[0], r);
  dv55[2] = (Sw22[2] * s1 + Sw22[5] * c1) + Sw22[8] * c3;
  //  i = 6:
  for (int i{0}; i < 3; i++) {
    Sw22[3 * i] = T56[i];
    Sw22[3 * i + 1] = T56[i + 4];
    Sw22[3 * i + 2] = T56[i + 8];
  }
  s1 = w55[0];
  c1 = w55[1];
  c3 = w55[2];
  s4 = dw55[0];
  c4 = dw55[1];
  s5 = dw55[2];
  for (b_i = 0; b_i < 3; b_i++) {
    c5 = static_cast<double>(a[b_i]) * dq[5];
    dv88[b_i] = c5;
    s6 = Sw22[b_i];
    c6 = s6 * s1;
    s7 = s6 * s4;
    s6 = Sw22[b_i + 3];
    c6 += s6 * c1;
    s7 += s6 * c4;
    s6 = Sw22[b_i + 6];
    c6 += s6 * c3;
    s7 += s6 * s5;
    dw66[b_i] = s7;
    w11_tmp[b_i] = c6;
    w66[b_i] = c6 + c5;
  }
  dw66[0] =
      (dw66[0] + (w11_tmp[1] * dv88[2] - dv88[1] * w11_tmp[2])) + 0.0 * ddq[5];
  dw66[1] =
      (dw66[1] + (dv88[0] * w11_tmp[2] - w11_tmp[0] * dv88[2])) + 0.0 * ddq[5];
  dw66[2] = (dw66[2] + (w11_tmp[0] * dv88[1] - dv88[0] * w11_tmp[1])) + ddq[5];
  w11_tmp[0] = w55[1] * 0.0 - w55[2] * 0.0;
  w11_tmp[1] = w55[2] * 0.0 - w55[0] * 0.0;
  w11_tmp[2] = w55[0] * 0.0 - w55[1] * 0.0;
  dv88[0] = dw55[1] * 0.0 - dw55[2] * 0.0;
  dv88[1] = dw55[2] * 0.0 - dw55[0] * 0.0;
  dv88[2] = dw55[0] * 0.0 - dw55[1] * 0.0;
  b_w11_tmp[0] = w55[1] * w11_tmp[2] - w11_tmp[1] * w55[2];
  b_w11_tmp[1] = w11_tmp[0] * w55[2] - w55[0] * w11_tmp[2];
  b_w11_tmp[2] = w55[0] * w11_tmp[1] - w11_tmp[0] * w55[1];
  r = _mm_loadu_pd(&dv88[0]);
  r1 = _mm_loadu_pd(&b_w11_tmp[0]);
  r2 = _mm_loadu_pd(&dv55[0]);
  _mm_storeu_pd(&dv88[0], _mm_add_pd(_mm_add_pd(r, r1), r2));
  dv88[2] = (dv88[2] + b_w11_tmp[2]) + dv55[2];
  s1 = dv88[0];
  c1 = dv88[1];
  c3 = dv88[2];
  r = _mm_loadu_pd(&Sw22[0]);
  r = _mm_mul_pd(r, _mm_set1_pd(s1));
  r1 = _mm_loadu_pd(&Sw22[3]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(c1));
  r = _mm_add_pd(r, r1);
  r1 = _mm_loadu_pd(&Sw22[6]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(c3));
  r = _mm_add_pd(r, r1);
  _mm_storeu_pd(&dv66[0], r);
  dv66[2] = (Sw22[2] * s1 + Sw22[5] * c1) + Sw22[8] * c3;
  //  i = 7:
  for (int i{0}; i < 3; i++) {
    Sw22[3 * i] = T67[i];
    Sw22[3 * i + 1] = T67[i + 4];
    Sw22[3 * i + 2] = T67[i + 8];
  }
  s1 = w66[0];
  c1 = w66[1];
  c3 = w66[2];
  s4 = dw66[0];
  c4 = dw66[1];
  s5 = dw66[2];
  for (b_i = 0; b_i < 3; b_i++) {
    c5 = static_cast<double>(a[b_i]) * dq[6];
    dv88[b_i] = c5;
    s6 = Sw22[b_i];
    c6 = s6 * s1;
    s7 = s6 * s4;
    s6 = Sw22[b_i + 3];
    c6 += s6 * c1;
    s7 += s6 * c4;
    s6 = Sw22[b_i + 6];
    c6 += s6 * c3;
    s7 += s6 * s5;
    dw77[b_i] = s7;
    w11_tmp[b_i] = c6;
    w77[b_i] = c6 + c5;
  }
  dw77[0] =
      (dw77[0] + (w11_tmp[1] * dv88[2] - dv88[1] * w11_tmp[2])) + 0.0 * ddq[6];
  dw77[1] =
      (dw77[1] + (dv88[0] * w11_tmp[2] - w11_tmp[0] * dv88[2])) + 0.0 * ddq[6];
  dw77[2] = (dw77[2] + (w11_tmp[0] * dv88[1] - dv88[0] * w11_tmp[1])) + ddq[6];
  w11_tmp[0] = w66[1] * 0.0 - w66[2] * 0.0;
  w11_tmp[1] = w66[2] * 0.088 - w66[0] * 0.0;
  w11_tmp[2] = w66[0] * 0.0 - w66[1] * 0.088;
  dv88[0] = dw66[1] * 0.0 - dw66[2] * 0.0;
  dv88[1] = dw66[2] * 0.088 - dw66[0] * 0.0;
  dv88[2] = dw66[0] * 0.0 - dw66[1] * 0.088;
  b_w11_tmp[0] = w66[1] * w11_tmp[2] - w11_tmp[1] * w66[2];
  b_w11_tmp[1] = w11_tmp[0] * w66[2] - w66[0] * w11_tmp[2];
  b_w11_tmp[2] = w66[0] * w11_tmp[1] - w11_tmp[0] * w66[1];
  r = _mm_loadu_pd(&dv88[0]);
  r1 = _mm_loadu_pd(&b_w11_tmp[0]);
  r2 = _mm_loadu_pd(&dv66[0]);
  _mm_storeu_pd(&dv88[0], _mm_add_pd(_mm_add_pd(r, r1), r2));
  dv88[2] = (dv88[2] + b_w11_tmp[2]) + dv66[2];
  //  i = 8:
  s1 = w77[0];
  c1 = w77[1];
  c3 = w77[2];
  s4 = dw77[0];
  c4 = dw77[1];
  s5 = dw77[2];
  c5 = dv88[0];
  s6 = dv88[1];
  c6 = dv88[2];
  for (b_i = 0; b_i < 3; b_i++) {
    signed char i1;
    i1 = iv[b_i];
    s7 = static_cast<double>(i1) * s1;
    c7 = static_cast<double>(i1) * s4;
    i1 = iv[b_i + 3];
    s7 += static_cast<double>(i1) * c1;
    c7 += static_cast<double>(i1) * c4;
    i1 = iv[b_i + 6];
    s7 += static_cast<double>(i1) * c3;
    c7 += static_cast<double>(i1) * s5;
    dw88[b_i] = c7;
    w11_tmp[b_i] = s7;
    dv77[b_i] = (Sw22[b_i] * c5 + Sw22[b_i + 3] * s6) + Sw22[b_i + 6] * c6;
    w88[b_i] = s7;
  }
  b_w11_tmp[0] = w11_tmp[1] * 0.0 - 0.0 * w11_tmp[2];
  b_w11_tmp[1] = 0.0 * w11_tmp[2] - w11_tmp[0] * 0.0;
  b_w11_tmp[2] = w11_tmp[0] * 0.0 - 0.0 * w11_tmp[1];
  r = _mm_loadu_pd(&dw88[0]);
  r1 = _mm_loadu_pd(&b_w11_tmp[0]);
  _mm_storeu_pd(&dw88[0], _mm_add_pd(r, r1));
  dw88[2] += b_w11_tmp[2];
  w11_tmp[0] = w77[1] * 0.107 - 0.0 * w77[2];
  w11_tmp[1] = 0.0 * w77[2] - w77[0] * 0.107;
  w11_tmp[2] = w77[0] * 0.0 - 0.0 * w77[1];
  b_w11_tmp[0] = dw77[1] * 0.107 - 0.0 * dw77[2];
  b_w11_tmp[1] = 0.0 * dw77[2] - dw77[0] * 0.107;
  b_w11_tmp[2] = dw77[0] * 0.0 - 0.0 * dw77[1];
  dv88[0] = w77[1] * w11_tmp[2] - w11_tmp[1] * w77[2];
  dv88[1] = w11_tmp[0] * w77[2] - w77[0] * w11_tmp[2];
  dv88[2] = w77[0] * w11_tmp[1] - w11_tmp[0] * w77[1];
  r = _mm_loadu_pd(&b_w11_tmp[0]);
  r1 = _mm_loadu_pd(&dv88[0]);
  r2 = _mm_loadu_pd(&dv77[0]);
  _mm_storeu_pd(&b_w11_tmp[0], _mm_add_pd(_mm_add_pd(r, r1), r2));
  b_w11_tmp[2] = (b_w11_tmp[2] + dv88[2]) + dv77[2];
  // --------------------------------------------------------------------------
  // Identification of matrix A
  // --------------------------------------------------------------------------
  //  Aii
  // Lwii=[wii(1) wii(2) wii(3) 0 0 0;0 wii(1) 0 wii(2) wii(3) 0;0 0 wii(1) 0
  // wii(2) wii(3)]; Ldwii=[dwii(1) dwii(2) dwii(3) 0 0 0;0 dwii(1) 0 dwii(2)
  // dwii(3) 0;0 0 dwii(1) 0 dwii(2) dwii(3)]; Swii=[0 -wii(3) wii(2);wii(3) 0
  // -wii(1);-wii(2) wii(1) 0]; Sdwii=[0 -dwii(3) dwii(2);dwii(3) 0
  // -dwii(1);-dwii(2) dwii(1) 0]; Sdddi=[0 -dddi(3) dddi(2);dddi(3) 0
  // -dddi(1);-dddi(2) dddi(1) 0]; somi=Ldwii+(Swii*Lwii);
  // somii=Sdwii+(Swii*Swii);
  // ai=horzcat(null,-Sddd,somi);
  // bi=horzcat(ddd,somii,null2);
  // Aii= [ai;bi];
  // i=1;
  Sw11[0] = 0.0;
  Sw11[3] = -w11[2];
  Sw11[6] = w11[1];
  Sw11[1] = w11[2];
  Sw11[4] = 0.0;
  Sw11[7] = -w11[0];
  Sw11[2] = -w11[1];
  Sw11[5] = w11[0];
  Sw11[8] = 0.0;
  // i=2;
  Sw22[0] = 0.0;
  Sw22[3] = -w22[2];
  Sw22[6] = w22[1];
  Sw22[1] = w22[2];
  Sw22[4] = 0.0;
  Sw22[7] = -w22[0];
  Sw22[2] = -w22[1];
  Sw22[5] = w22[0];
  Sw22[8] = 0.0;
  b_w22[0] = w22[0];
  b_w22[3] = w22[1];
  b_w22[6] = w22[2];
  b_w22[9] = 0.0;
  b_w22[12] = 0.0;
  b_w22[15] = 0.0;
  b_w22[1] = 0.0;
  b_w22[4] = w22[0];
  b_w22[7] = 0.0;
  b_w22[10] = w22[1];
  b_w22[13] = w22[2];
  b_w22[16] = 0.0;
  b_w22[2] = 0.0;
  b_w22[5] = 0.0;
  b_w22[8] = w22[0];
  b_w22[11] = 0.0;
  b_w22[14] = w22[1];
  b_w22[17] = w22[2];
  b_dw22[0] = dw22[0];
  b_dw22[3] = dw22[1];
  b_dw22[6] = dw22[2];
  b_dw22[9] = 0.0;
  b_dw22[12] = 0.0;
  b_dw22[15] = 0.0;
  b_dw22[1] = 0.0;
  b_dw22[4] = dw22[0];
  b_dw22[7] = 0.0;
  b_dw22[10] = dw22[1];
  b_dw22[13] = dw22[2];
  b_dw22[16] = 0.0;
  b_dw22[2] = 0.0;
  b_dw22[5] = 0.0;
  b_dw22[8] = dw22[0];
  b_dw22[11] = 0.0;
  b_dw22[14] = dw22[1];
  b_dw22[17] = dw22[2];
  dv[0] = 0.0;
  dv[3] = -dw22[2];
  dv[6] = dw22[1];
  dv[1] = dw22[2];
  dv[4] = 0.0;
  dv[7] = -dw22[0];
  dv[2] = -dw22[1];
  dv[5] = dw22[0];
  dv[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    dv88[i] = (static_cast<double>(iv[i]) * b_w11_tmp[0] +
               static_cast<double>(iv[i + 3]) * b_w11_tmp[1]) +
              static_cast<double>(iv[i + 6]) * b_w11_tmp[2];
    s1 = Sw22[i];
    c1 = Sw22[i + 3];
    c3 = Sw22[i + 6];
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      b_Sw22[i + 3 * K_tmp] =
          (s1 * b_w22[3 * K_tmp] + c1 * b_w22[3 * K_tmp + 1]) +
          c3 * b_w22[3 * K_tmp + 2];
    }
    for (K_tmp = 0; K_tmp < 3; K_tmp++) {
      c_Sw22[i + 3 * K_tmp] =
          (Sw22[i] * Sw22[3 * K_tmp] + c1 * Sw22[3 * K_tmp + 1]) +
          c3 * Sw22[3 * K_tmp + 2];
    }
    A22[i] = 0.0;
  }
  A22[6] = -0.0;
  A22[12] = dv22[2];
  A22[18] = -dv22[1];
  A22[7] = -dv22[2];
  A22[13] = -0.0;
  A22[19] = dv22[0];
  A22[8] = dv22[1];
  A22[14] = -dv22[0];
  A22[20] = -0.0;
  for (int i{0}; i < 6; i++) {
    r = _mm_loadu_pd(&b_dw22[3 * i]);
    r1 = _mm_loadu_pd(&b_Sw22[3 * i]);
    K_tmp = 6 * (i + 4);
    _mm_storeu_pd(&A22[K_tmp], _mm_add_pd(r, r1));
    b_i = 3 * i + 2;
    A22[K_tmp + 2] = b_dw22[b_i] + b_Sw22[b_i];
  }
  A22[3] = dv22[0];
  r = _mm_loadu_pd(&dv[0]);
  r1 = _mm_loadu_pd(&c_Sw22[0]);
  _mm_storeu_pd(&A22[9], _mm_add_pd(r, r1));
  A22[11] = -dw22[1] + c_Sw22[2];
  A22[4] = dv22[1];
  r = _mm_loadu_pd(&dv[3]);
  r1 = _mm_loadu_pd(&c_Sw22[3]);
  _mm_storeu_pd(&A22[15], _mm_add_pd(r, r1));
  A22[17] = dw22[0] + c_Sw22[5];
  A22[5] = dv22[2];
  r = _mm_loadu_pd(&dv[6]);
  r1 = _mm_loadu_pd(&c_Sw22[6]);
  _mm_storeu_pd(&A22[21], _mm_add_pd(r, r1));
  A22[23] = c_Sw22[8];
  for (int i{0}; i < 6; i++) {
    b_i = 6 * (i + 4);
    A22[b_i + 3] = 0.0;
    A22[b_i + 4] = 0.0;
    A22[b_i + 5] = 0.0;
  }
  // i=3;
  Sw22[0] = 0.0;
  Sw22[3] = -w33[2];
  Sw22[6] = w33[1];
  Sw22[1] = w33[2];
  Sw22[4] = 0.0;
  Sw22[7] = -w33[0];
  Sw22[2] = -w33[1];
  Sw22[5] = w33[0];
  Sw22[8] = 0.0;
  b_w22[0] = w33[0];
  b_w22[3] = w33[1];
  b_w22[6] = w33[2];
  b_w22[9] = 0.0;
  b_w22[12] = 0.0;
  b_w22[15] = 0.0;
  b_w22[1] = 0.0;
  b_w22[4] = w33[0];
  b_w22[7] = 0.0;
  b_w22[10] = w33[1];
  b_w22[13] = w33[2];
  b_w22[16] = 0.0;
  b_w22[2] = 0.0;
  b_w22[5] = 0.0;
  b_w22[8] = w33[0];
  b_w22[11] = 0.0;
  b_w22[14] = w33[1];
  b_w22[17] = w33[2];
  b_dw22[0] = dw33[0];
  b_dw22[3] = dw33[1];
  b_dw22[6] = dw33[2];
  b_dw22[9] = 0.0;
  b_dw22[12] = 0.0;
  b_dw22[15] = 0.0;
  b_dw22[1] = 0.0;
  b_dw22[4] = dw33[0];
  b_dw22[7] = 0.0;
  b_dw22[10] = dw33[1];
  b_dw22[13] = dw33[2];
  b_dw22[16] = 0.0;
  b_dw22[2] = 0.0;
  b_dw22[5] = 0.0;
  b_dw22[8] = dw33[0];
  b_dw22[11] = 0.0;
  b_dw22[14] = dw33[1];
  b_dw22[17] = dw33[2];
  dv[0] = 0.0;
  dv[3] = -dw33[2];
  dv[6] = dw33[1];
  dv[1] = dw33[2];
  dv[4] = 0.0;
  dv[7] = -dw33[0];
  dv[2] = -dw33[1];
  dv[5] = dw33[0];
  dv[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    s1 = Sw22[i];
    c1 = Sw22[i + 3];
    c3 = Sw22[i + 6];
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      b_Sw22[i + 3 * K_tmp] =
          (s1 * b_w22[3 * K_tmp] + c1 * b_w22[3 * K_tmp + 1]) +
          c3 * b_w22[3 * K_tmp + 2];
    }
    for (K_tmp = 0; K_tmp < 3; K_tmp++) {
      c_Sw22[i + 3 * K_tmp] =
          (Sw22[i] * Sw22[3 * K_tmp] + c1 * Sw22[3 * K_tmp + 1]) +
          c3 * Sw22[3 * K_tmp + 2];
    }
    A33[i] = 0.0;
  }
  A33[6] = -0.0;
  A33[12] = dv33[2];
  A33[18] = -dv33[1];
  A33[7] = -dv33[2];
  A33[13] = -0.0;
  A33[19] = dv33[0];
  A33[8] = dv33[1];
  A33[14] = -dv33[0];
  A33[20] = -0.0;
  for (int i{0}; i < 6; i++) {
    r = _mm_loadu_pd(&b_dw22[3 * i]);
    r1 = _mm_loadu_pd(&b_Sw22[3 * i]);
    K_tmp = 6 * (i + 4);
    _mm_storeu_pd(&A33[K_tmp], _mm_add_pd(r, r1));
    b_i = 3 * i + 2;
    A33[K_tmp + 2] = b_dw22[b_i] + b_Sw22[b_i];
  }
  A33[3] = dv33[0];
  r = _mm_loadu_pd(&dv[0]);
  r1 = _mm_loadu_pd(&c_Sw22[0]);
  _mm_storeu_pd(&A33[9], _mm_add_pd(r, r1));
  A33[11] = -dw33[1] + c_Sw22[2];
  A33[4] = dv33[1];
  r = _mm_loadu_pd(&dv[3]);
  r1 = _mm_loadu_pd(&c_Sw22[3]);
  _mm_storeu_pd(&A33[15], _mm_add_pd(r, r1));
  A33[17] = dw33[0] + c_Sw22[5];
  A33[5] = dv33[2];
  r = _mm_loadu_pd(&dv[6]);
  r1 = _mm_loadu_pd(&c_Sw22[6]);
  _mm_storeu_pd(&A33[21], _mm_add_pd(r, r1));
  A33[23] = c_Sw22[8];
  for (int i{0}; i < 6; i++) {
    b_i = 6 * (i + 4);
    A33[b_i + 3] = 0.0;
    A33[b_i + 4] = 0.0;
    A33[b_i + 5] = 0.0;
  }
  // i=4;
  Sw22[0] = 0.0;
  Sw22[3] = -w44[2];
  Sw22[6] = w44[1];
  Sw22[1] = w44[2];
  Sw22[4] = 0.0;
  Sw22[7] = -w44[0];
  Sw22[2] = -w44[1];
  Sw22[5] = w44[0];
  Sw22[8] = 0.0;
  b_w22[0] = w44[0];
  b_w22[3] = w44[1];
  b_w22[6] = w44[2];
  b_w22[9] = 0.0;
  b_w22[12] = 0.0;
  b_w22[15] = 0.0;
  b_w22[1] = 0.0;
  b_w22[4] = w44[0];
  b_w22[7] = 0.0;
  b_w22[10] = w44[1];
  b_w22[13] = w44[2];
  b_w22[16] = 0.0;
  b_w22[2] = 0.0;
  b_w22[5] = 0.0;
  b_w22[8] = w44[0];
  b_w22[11] = 0.0;
  b_w22[14] = w44[1];
  b_w22[17] = w44[2];
  b_dw22[0] = dw44[0];
  b_dw22[3] = dw44[1];
  b_dw22[6] = dw44[2];
  b_dw22[9] = 0.0;
  b_dw22[12] = 0.0;
  b_dw22[15] = 0.0;
  b_dw22[1] = 0.0;
  b_dw22[4] = dw44[0];
  b_dw22[7] = 0.0;
  b_dw22[10] = dw44[1];
  b_dw22[13] = dw44[2];
  b_dw22[16] = 0.0;
  b_dw22[2] = 0.0;
  b_dw22[5] = 0.0;
  b_dw22[8] = dw44[0];
  b_dw22[11] = 0.0;
  b_dw22[14] = dw44[1];
  b_dw22[17] = dw44[2];
  dv[0] = 0.0;
  dv[3] = -dw44[2];
  dv[6] = dw44[1];
  dv[1] = dw44[2];
  dv[4] = 0.0;
  dv[7] = -dw44[0];
  dv[2] = -dw44[1];
  dv[5] = dw44[0];
  dv[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    s1 = Sw22[i];
    c1 = Sw22[i + 3];
    c3 = Sw22[i + 6];
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      b_Sw22[i + 3 * K_tmp] =
          (s1 * b_w22[3 * K_tmp] + c1 * b_w22[3 * K_tmp + 1]) +
          c3 * b_w22[3 * K_tmp + 2];
    }
    for (K_tmp = 0; K_tmp < 3; K_tmp++) {
      c_Sw22[i + 3 * K_tmp] =
          (Sw22[i] * Sw22[3 * K_tmp] + c1 * Sw22[3 * K_tmp + 1]) +
          c3 * Sw22[3 * K_tmp + 2];
    }
    A44[i] = 0.0;
  }
  A44[6] = -0.0;
  A44[12] = dv44[2];
  A44[18] = -dv44[1];
  A44[7] = -dv44[2];
  A44[13] = -0.0;
  A44[19] = dv44[0];
  A44[8] = dv44[1];
  A44[14] = -dv44[0];
  A44[20] = -0.0;
  for (int i{0}; i < 6; i++) {
    r = _mm_loadu_pd(&b_dw22[3 * i]);
    r1 = _mm_loadu_pd(&b_Sw22[3 * i]);
    K_tmp = 6 * (i + 4);
    _mm_storeu_pd(&A44[K_tmp], _mm_add_pd(r, r1));
    b_i = 3 * i + 2;
    A44[K_tmp + 2] = b_dw22[b_i] + b_Sw22[b_i];
  }
  A44[3] = dv44[0];
  r = _mm_loadu_pd(&dv[0]);
  r1 = _mm_loadu_pd(&c_Sw22[0]);
  _mm_storeu_pd(&A44[9], _mm_add_pd(r, r1));
  A44[11] = -dw44[1] + c_Sw22[2];
  A44[4] = dv44[1];
  r = _mm_loadu_pd(&dv[3]);
  r1 = _mm_loadu_pd(&c_Sw22[3]);
  _mm_storeu_pd(&A44[15], _mm_add_pd(r, r1));
  A44[17] = dw44[0] + c_Sw22[5];
  A44[5] = dv44[2];
  r = _mm_loadu_pd(&dv[6]);
  r1 = _mm_loadu_pd(&c_Sw22[6]);
  _mm_storeu_pd(&A44[21], _mm_add_pd(r, r1));
  A44[23] = c_Sw22[8];
  for (int i{0}; i < 6; i++) {
    b_i = 6 * (i + 4);
    A44[b_i + 3] = 0.0;
    A44[b_i + 4] = 0.0;
    A44[b_i + 5] = 0.0;
  }
  // i=5;
  Sw22[0] = 0.0;
  Sw22[3] = -w55[2];
  Sw22[6] = w55[1];
  Sw22[1] = w55[2];
  Sw22[4] = 0.0;
  Sw22[7] = -w55[0];
  Sw22[2] = -w55[1];
  Sw22[5] = w55[0];
  Sw22[8] = 0.0;
  b_w22[0] = w55[0];
  b_w22[3] = w55[1];
  b_w22[6] = w55[2];
  b_w22[9] = 0.0;
  b_w22[12] = 0.0;
  b_w22[15] = 0.0;
  b_w22[1] = 0.0;
  b_w22[4] = w55[0];
  b_w22[7] = 0.0;
  b_w22[10] = w55[1];
  b_w22[13] = w55[2];
  b_w22[16] = 0.0;
  b_w22[2] = 0.0;
  b_w22[5] = 0.0;
  b_w22[8] = w55[0];
  b_w22[11] = 0.0;
  b_w22[14] = w55[1];
  b_w22[17] = w55[2];
  b_dw22[0] = dw55[0];
  b_dw22[3] = dw55[1];
  b_dw22[6] = dw55[2];
  b_dw22[9] = 0.0;
  b_dw22[12] = 0.0;
  b_dw22[15] = 0.0;
  b_dw22[1] = 0.0;
  b_dw22[4] = dw55[0];
  b_dw22[7] = 0.0;
  b_dw22[10] = dw55[1];
  b_dw22[13] = dw55[2];
  b_dw22[16] = 0.0;
  b_dw22[2] = 0.0;
  b_dw22[5] = 0.0;
  b_dw22[8] = dw55[0];
  b_dw22[11] = 0.0;
  b_dw22[14] = dw55[1];
  b_dw22[17] = dw55[2];
  dv[0] = 0.0;
  dv[3] = -dw55[2];
  dv[6] = dw55[1];
  dv[1] = dw55[2];
  dv[4] = 0.0;
  dv[7] = -dw55[0];
  dv[2] = -dw55[1];
  dv[5] = dw55[0];
  dv[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    s1 = Sw22[i];
    c1 = Sw22[i + 3];
    c3 = Sw22[i + 6];
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      b_Sw22[i + 3 * K_tmp] =
          (s1 * b_w22[3 * K_tmp] + c1 * b_w22[3 * K_tmp + 1]) +
          c3 * b_w22[3 * K_tmp + 2];
    }
    for (K_tmp = 0; K_tmp < 3; K_tmp++) {
      c_Sw22[i + 3 * K_tmp] =
          (Sw22[i] * Sw22[3 * K_tmp] + c1 * Sw22[3 * K_tmp + 1]) +
          c3 * Sw22[3 * K_tmp + 2];
    }
    A55[i] = 0.0;
  }
  A55[6] = -0.0;
  A55[12] = dv55[2];
  A55[18] = -dv55[1];
  A55[7] = -dv55[2];
  A55[13] = -0.0;
  A55[19] = dv55[0];
  A55[8] = dv55[1];
  A55[14] = -dv55[0];
  A55[20] = -0.0;
  for (int i{0}; i < 6; i++) {
    r = _mm_loadu_pd(&b_dw22[3 * i]);
    r1 = _mm_loadu_pd(&b_Sw22[3 * i]);
    K_tmp = 6 * (i + 4);
    _mm_storeu_pd(&A55[K_tmp], _mm_add_pd(r, r1));
    b_i = 3 * i + 2;
    A55[K_tmp + 2] = b_dw22[b_i] + b_Sw22[b_i];
  }
  A55[3] = dv55[0];
  r = _mm_loadu_pd(&dv[0]);
  r1 = _mm_loadu_pd(&c_Sw22[0]);
  _mm_storeu_pd(&A55[9], _mm_add_pd(r, r1));
  A55[11] = -dw55[1] + c_Sw22[2];
  A55[4] = dv55[1];
  r = _mm_loadu_pd(&dv[3]);
  r1 = _mm_loadu_pd(&c_Sw22[3]);
  _mm_storeu_pd(&A55[15], _mm_add_pd(r, r1));
  A55[17] = dw55[0] + c_Sw22[5];
  A55[5] = dv55[2];
  r = _mm_loadu_pd(&dv[6]);
  r1 = _mm_loadu_pd(&c_Sw22[6]);
  _mm_storeu_pd(&A55[21], _mm_add_pd(r, r1));
  A55[23] = c_Sw22[8];
  for (int i{0}; i < 6; i++) {
    b_i = 6 * (i + 4);
    A55[b_i + 3] = 0.0;
    A55[b_i + 4] = 0.0;
    A55[b_i + 5] = 0.0;
  }
  // i=6;
  Sw22[0] = 0.0;
  Sw22[3] = -w66[2];
  Sw22[6] = w66[1];
  Sw22[1] = w66[2];
  Sw22[4] = 0.0;
  Sw22[7] = -w66[0];
  Sw22[2] = -w66[1];
  Sw22[5] = w66[0];
  Sw22[8] = 0.0;
  b_w22[0] = w66[0];
  b_w22[3] = w66[1];
  b_w22[6] = w66[2];
  b_w22[9] = 0.0;
  b_w22[12] = 0.0;
  b_w22[15] = 0.0;
  b_w22[1] = 0.0;
  b_w22[4] = w66[0];
  b_w22[7] = 0.0;
  b_w22[10] = w66[1];
  b_w22[13] = w66[2];
  b_w22[16] = 0.0;
  b_w22[2] = 0.0;
  b_w22[5] = 0.0;
  b_w22[8] = w66[0];
  b_w22[11] = 0.0;
  b_w22[14] = w66[1];
  b_w22[17] = w66[2];
  b_dw22[0] = dw66[0];
  b_dw22[3] = dw66[1];
  b_dw22[6] = dw66[2];
  b_dw22[9] = 0.0;
  b_dw22[12] = 0.0;
  b_dw22[15] = 0.0;
  b_dw22[1] = 0.0;
  b_dw22[4] = dw66[0];
  b_dw22[7] = 0.0;
  b_dw22[10] = dw66[1];
  b_dw22[13] = dw66[2];
  b_dw22[16] = 0.0;
  b_dw22[2] = 0.0;
  b_dw22[5] = 0.0;
  b_dw22[8] = dw66[0];
  b_dw22[11] = 0.0;
  b_dw22[14] = dw66[1];
  b_dw22[17] = dw66[2];
  dv[0] = 0.0;
  dv[3] = -dw66[2];
  dv[6] = dw66[1];
  dv[1] = dw66[2];
  dv[4] = 0.0;
  dv[7] = -dw66[0];
  dv[2] = -dw66[1];
  dv[5] = dw66[0];
  dv[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    s1 = Sw22[i];
    c1 = Sw22[i + 3];
    c3 = Sw22[i + 6];
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      b_Sw22[i + 3 * K_tmp] =
          (s1 * b_w22[3 * K_tmp] + c1 * b_w22[3 * K_tmp + 1]) +
          c3 * b_w22[3 * K_tmp + 2];
    }
    for (K_tmp = 0; K_tmp < 3; K_tmp++) {
      c_Sw22[i + 3 * K_tmp] =
          (Sw22[i] * Sw22[3 * K_tmp] + c1 * Sw22[3 * K_tmp + 1]) +
          c3 * Sw22[3 * K_tmp + 2];
    }
    A66[i] = 0.0;
  }
  A66[6] = -0.0;
  A66[12] = dv66[2];
  A66[18] = -dv66[1];
  A66[7] = -dv66[2];
  A66[13] = -0.0;
  A66[19] = dv66[0];
  A66[8] = dv66[1];
  A66[14] = -dv66[0];
  A66[20] = -0.0;
  for (int i{0}; i < 6; i++) {
    r = _mm_loadu_pd(&b_dw22[3 * i]);
    r1 = _mm_loadu_pd(&b_Sw22[3 * i]);
    K_tmp = 6 * (i + 4);
    _mm_storeu_pd(&A66[K_tmp], _mm_add_pd(r, r1));
    b_i = 3 * i + 2;
    A66[K_tmp + 2] = b_dw22[b_i] + b_Sw22[b_i];
  }
  A66[3] = dv66[0];
  r = _mm_loadu_pd(&dv[0]);
  r1 = _mm_loadu_pd(&c_Sw22[0]);
  _mm_storeu_pd(&A66[9], _mm_add_pd(r, r1));
  A66[11] = -dw66[1] + c_Sw22[2];
  A66[4] = dv66[1];
  r = _mm_loadu_pd(&dv[3]);
  r1 = _mm_loadu_pd(&c_Sw22[3]);
  _mm_storeu_pd(&A66[15], _mm_add_pd(r, r1));
  A66[17] = dw66[0] + c_Sw22[5];
  A66[5] = dv66[2];
  r = _mm_loadu_pd(&dv[6]);
  r1 = _mm_loadu_pd(&c_Sw22[6]);
  _mm_storeu_pd(&A66[21], _mm_add_pd(r, r1));
  A66[23] = c_Sw22[8];
  for (int i{0}; i < 6; i++) {
    b_i = 6 * (i + 4);
    A66[b_i + 3] = 0.0;
    A66[b_i + 4] = 0.0;
    A66[b_i + 5] = 0.0;
  }
  // i=7;
  Sw22[0] = 0.0;
  Sw22[3] = -w77[2];
  Sw22[6] = w77[1];
  Sw22[1] = w77[2];
  Sw22[4] = 0.0;
  Sw22[7] = -w77[0];
  Sw22[2] = -w77[1];
  Sw22[5] = w77[0];
  Sw22[8] = 0.0;
  b_w22[0] = w77[0];
  b_w22[3] = w77[1];
  b_w22[6] = w77[2];
  b_w22[9] = 0.0;
  b_w22[12] = 0.0;
  b_w22[15] = 0.0;
  b_w22[1] = 0.0;
  b_w22[4] = w77[0];
  b_w22[7] = 0.0;
  b_w22[10] = w77[1];
  b_w22[13] = w77[2];
  b_w22[16] = 0.0;
  b_w22[2] = 0.0;
  b_w22[5] = 0.0;
  b_w22[8] = w77[0];
  b_w22[11] = 0.0;
  b_w22[14] = w77[1];
  b_w22[17] = w77[2];
  b_dw22[0] = dw77[0];
  b_dw22[3] = dw77[1];
  b_dw22[6] = dw77[2];
  b_dw22[9] = 0.0;
  b_dw22[12] = 0.0;
  b_dw22[15] = 0.0;
  b_dw22[1] = 0.0;
  b_dw22[4] = dw77[0];
  b_dw22[7] = 0.0;
  b_dw22[10] = dw77[1];
  b_dw22[13] = dw77[2];
  b_dw22[16] = 0.0;
  b_dw22[2] = 0.0;
  b_dw22[5] = 0.0;
  b_dw22[8] = dw77[0];
  b_dw22[11] = 0.0;
  b_dw22[14] = dw77[1];
  b_dw22[17] = dw77[2];
  dv[0] = 0.0;
  dv[3] = -dw77[2];
  dv[6] = dw77[1];
  dv[1] = dw77[2];
  dv[4] = 0.0;
  dv[7] = -dw77[0];
  dv[2] = -dw77[1];
  dv[5] = dw77[0];
  dv[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    s1 = Sw22[i];
    c1 = Sw22[i + 3];
    c3 = Sw22[i + 6];
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      b_Sw22[i + 3 * K_tmp] =
          (s1 * b_w22[3 * K_tmp] + c1 * b_w22[3 * K_tmp + 1]) +
          c3 * b_w22[3 * K_tmp + 2];
    }
    for (K_tmp = 0; K_tmp < 3; K_tmp++) {
      c_Sw22[i + 3 * K_tmp] =
          (Sw22[i] * Sw22[3 * K_tmp] + c1 * Sw22[3 * K_tmp + 1]) +
          c3 * Sw22[3 * K_tmp + 2];
    }
    A77[i] = 0.0;
  }
  A77[6] = -0.0;
  A77[12] = dv77[2];
  A77[18] = -dv77[1];
  A77[7] = -dv77[2];
  A77[13] = -0.0;
  A77[19] = dv77[0];
  A77[8] = dv77[1];
  A77[14] = -dv77[0];
  A77[20] = -0.0;
  for (int i{0}; i < 6; i++) {
    r = _mm_loadu_pd(&b_dw22[3 * i]);
    r1 = _mm_loadu_pd(&b_Sw22[3 * i]);
    K_tmp = 6 * (i + 4);
    _mm_storeu_pd(&A77[K_tmp], _mm_add_pd(r, r1));
    b_i = 3 * i + 2;
    A77[K_tmp + 2] = b_dw22[b_i] + b_Sw22[b_i];
  }
  A77[3] = dv77[0];
  r = _mm_loadu_pd(&dv[0]);
  r1 = _mm_loadu_pd(&c_Sw22[0]);
  _mm_storeu_pd(&A77[9], _mm_add_pd(r, r1));
  A77[11] = -dw77[1] + c_Sw22[2];
  A77[4] = dv77[1];
  r = _mm_loadu_pd(&dv[3]);
  r1 = _mm_loadu_pd(&c_Sw22[3]);
  _mm_storeu_pd(&A77[15], _mm_add_pd(r, r1));
  A77[17] = dw77[0] + c_Sw22[5];
  A77[5] = dv77[2];
  r = _mm_loadu_pd(&dv[6]);
  r1 = _mm_loadu_pd(&c_Sw22[6]);
  _mm_storeu_pd(&A77[21], _mm_add_pd(r, r1));
  A77[23] = c_Sw22[8];
  for (int i{0}; i < 6; i++) {
    b_i = 6 * (i + 4);
    A77[b_i + 3] = 0.0;
    A77[b_i + 4] = 0.0;
    A77[b_i + 5] = 0.0;
  }
  // i=8;
  Sw22[0] = 0.0;
  Sw22[3] = -w88[2];
  Sw22[6] = w88[1];
  Sw22[1] = w88[2];
  Sw22[4] = 0.0;
  Sw22[7] = -w88[0];
  Sw22[2] = -w88[1];
  Sw22[5] = w88[0];
  Sw22[8] = 0.0;
  b_w22[0] = w88[0];
  b_w22[3] = w88[1];
  b_w22[6] = w88[2];
  b_w22[9] = 0.0;
  b_w22[12] = 0.0;
  b_w22[15] = 0.0;
  b_w22[1] = 0.0;
  b_w22[4] = w88[0];
  b_w22[7] = 0.0;
  b_w22[10] = w88[1];
  b_w22[13] = w88[2];
  b_w22[16] = 0.0;
  b_w22[2] = 0.0;
  b_w22[5] = 0.0;
  b_w22[8] = w88[0];
  b_w22[11] = 0.0;
  b_w22[14] = w88[1];
  b_w22[17] = w88[2];
  b_dw22[0] = dw88[0];
  b_dw22[3] = dw88[1];
  b_dw22[6] = dw88[2];
  b_dw22[9] = 0.0;
  b_dw22[12] = 0.0;
  b_dw22[15] = 0.0;
  b_dw22[1] = 0.0;
  b_dw22[4] = dw88[0];
  b_dw22[7] = 0.0;
  b_dw22[10] = dw88[1];
  b_dw22[13] = dw88[2];
  b_dw22[16] = 0.0;
  b_dw22[2] = 0.0;
  b_dw22[5] = 0.0;
  b_dw22[8] = dw88[0];
  b_dw22[11] = 0.0;
  b_dw22[14] = dw88[1];
  b_dw22[17] = dw88[2];
  dv[0] = 0.0;
  dv[3] = -dw88[2];
  dv[6] = dw88[1];
  dv[1] = dw88[2];
  dv[4] = 0.0;
  dv[7] = -dw88[0];
  dv[2] = -dw88[1];
  dv[5] = dw88[0];
  dv[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    s1 = Sw22[i];
    c1 = Sw22[i + 3];
    c3 = Sw22[i + 6];
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      b_Sw22[i + 3 * K_tmp] =
          (s1 * b_w22[3 * K_tmp] + c1 * b_w22[3 * K_tmp + 1]) +
          c3 * b_w22[3 * K_tmp + 2];
    }
    for (K_tmp = 0; K_tmp < 3; K_tmp++) {
      c_Sw22[i + 3 * K_tmp] =
          (Sw22[i] * Sw22[3 * K_tmp] + c1 * Sw22[3 * K_tmp + 1]) +
          c3 * Sw22[3 * K_tmp + 2];
    }
    A88[i] = 0.0;
  }
  A88[6] = -0.0;
  A88[12] = dv88[2];
  A88[18] = -dv88[1];
  A88[7] = -dv88[2];
  A88[13] = -0.0;
  A88[19] = dv88[0];
  A88[8] = dv88[1];
  A88[14] = -dv88[0];
  A88[20] = -0.0;
  for (int i{0}; i < 6; i++) {
    r = _mm_loadu_pd(&b_dw22[3 * i]);
    r1 = _mm_loadu_pd(&b_Sw22[3 * i]);
    K_tmp = 6 * (i + 4);
    _mm_storeu_pd(&A88[K_tmp], _mm_add_pd(r, r1));
    b_i = 3 * i + 2;
    A88[K_tmp + 2] = b_dw22[b_i] + b_Sw22[b_i];
  }
  A88[3] = dv88[0];
  r = _mm_loadu_pd(&dv[0]);
  r1 = _mm_loadu_pd(&c_Sw22[0]);
  _mm_storeu_pd(&A88[9], _mm_add_pd(r, r1));
  A88[11] = -dw88[1] + c_Sw22[2];
  A88[4] = dv88[1];
  r = _mm_loadu_pd(&dv[3]);
  r1 = _mm_loadu_pd(&c_Sw22[3]);
  _mm_storeu_pd(&A88[15], _mm_add_pd(r, r1));
  A88[17] = dw88[0] + c_Sw22[5];
  A88[5] = dv88[2];
  r = _mm_loadu_pd(&dv[6]);
  r1 = _mm_loadu_pd(&c_Sw22[6]);
  _mm_storeu_pd(&A88[21], _mm_add_pd(r, r1));
  A88[23] = c_Sw22[8];
  for (int i{0}; i < 6; i++) {
    b_i = 6 * (i + 4);
    A88[b_i + 3] = 0.0;
    A88[b_i + 4] = 0.0;
    A88[b_i + 5] = 0.0;
  }
  // -----------------------------------------------------------------------------
  // identification of matrix X(i,i+1)F(i+1)
  // -------------------------------------------------------------------------------
  // p(i,i+1)x=T(i,i+1)(1,4);
  // p(i,i+1)y=T(i,i+1)(2,4);
  // p(i,i+1)z=T(i,i+1)(3,4);
  // Sp(i,i+1)=[0 -p(i,i+1)z p(i,i+1)y;p(i,i+1)z 0 -p(i,i+1)x;-p(i,i+1)y
  // p(i,i+1)x 0]; X(i,i+1)=[R(i,i+1) null1; R(i,i+1)*Sp(i,i+1) R(i,i+1)];
  // X(i,i+1)F(i+1)=(inv(X(i,i+1)).';
  // i=1:
  // X01F1=(inv(X01)).';
  // i=2:
  for (int i{0}; i < 3; i++) {
    for (K_tmp = 0; K_tmp < 3; K_tmp++) {
      b_K_tmp = K_tmp << 2;
      s1 = T12[i + b_K_tmp];
      T12_tmp = K_tmp + 6 * i;
      b_T12[T12_tmp] = s1;
      b_T12[T12_tmp + 3] = 0.0;
      T12_tmp = K_tmp + 6 * (i + 3);
      b_T12[T12_tmp] = (0.0 * T12[b_K_tmp] + 0.0 * T12[b_K_tmp + 1]) +
                       0.0 * T12[b_K_tmp + 2];
      b_T12[T12_tmp + 3] = s1;
    }
  }
  coder::inv(b_T12, X12F2);
  // i=3:
  // i=4:
  for (int i{0}; i < 3; i++) {
    s1 = b_a[i];
    K_tmp = static_cast<int>(b_a[i + 3]);
    c1 = b_a[i + 6];
    b_K_tmp = static_cast<int>(c_a[i]);
    c3 = c_a[i + 3];
    s4 = c_a[i + 6];
    for (b_i = 0; b_i < 3; b_i++) {
      a_tmp = b_i << 2;
      c4 = T23[i + a_tmp];
      T12_tmp = b_i + 6 * i;
      b_T12[T12_tmp] = c4;
      b_T12[T12_tmp + 3] = 0.0;
      T12_tmp = b_i + 6 * (i + 3);
      b_T12[T12_tmp] =
          (s1 * T23[a_tmp] + static_cast<double>(K_tmp) * T23[a_tmp + 1]) +
          c1 * T23[a_tmp + 2];
      b_T12[T12_tmp + 3] = c4;
      Sw22[i + 3 * b_i] =
          (static_cast<double>(b_K_tmp) * T34[a_tmp] + c3 * T34[a_tmp + 1]) +
          s4 * T34[a_tmp + 2];
    }
  }
  coder::inv(b_T12, X23F3);
  // i=5:
  for (int i{0}; i < 3; i++) {
    s1 = d_a[i];
    c1 = d_a[i + 3];
    c3 = d_a[i + 6];
    for (K_tmp = 0; K_tmp < 3; K_tmp++) {
      b_K_tmp = K_tmp << 2;
      s4 = T34[i + b_K_tmp];
      T12_tmp = K_tmp + 6 * i;
      b_T12[T12_tmp] = s4;
      b_T12[T12_tmp + 3] = 0.0;
      T12_tmp = K_tmp + 6 * (i + 3);
      b_i = i + 3 * K_tmp;
      b_T12[T12_tmp] = Sw22[b_i];
      b_T12[T12_tmp + 3] = s4;
      Sw22[b_i] =
          (s1 * T45[b_K_tmp] + c1 * T45[b_K_tmp + 1]) + c3 * T45[b_K_tmp + 2];
    }
  }
  coder::inv(b_T12, X34F4);
  for (int i{0}; i < 3; i++) {
    s1 = T45[i];
    b_T12[6 * i] = s1;
    b_T12[6 * i + 3] = 0.0;
    T12_tmp = 6 * (i + 3);
    b_T12[T12_tmp] = Sw22[i];
    b_T12[T12_tmp + 3] = s1;
    s1 = T45[i + 4];
    b_T12[6 * i + 1] = s1;
    b_T12[6 * i + 4] = 0.0;
    b_T12[T12_tmp + 1] = Sw22[i + 3];
    b_T12[T12_tmp + 4] = s1;
    s1 = T45[i + 8];
    b_T12[6 * i + 2] = s1;
    b_T12[6 * i + 5] = 0.0;
    b_T12[T12_tmp + 2] = Sw22[i + 6];
    b_T12[T12_tmp + 5] = s1;
  }
  coder::inv(b_T12, X45F5);
  // i=6:
  for (int i{0}; i < 3; i++) {
    for (K_tmp = 0; K_tmp < 3; K_tmp++) {
      b_K_tmp = K_tmp << 2;
      s1 = T56[i + b_K_tmp];
      T12_tmp = K_tmp + 6 * i;
      b_T12[T12_tmp] = s1;
      b_T12[T12_tmp + 3] = 0.0;
      T12_tmp = K_tmp + 6 * (i + 3);
      b_T12[T12_tmp] = (0.0 * T56[b_K_tmp] + 0.0 * T56[b_K_tmp + 1]) +
                       0.0 * T56[b_K_tmp + 2];
      b_T12[T12_tmp + 3] = s1;
    }
  }
  coder::inv(b_T12, X56F6);
  // i=7:
  for (int i{0}; i < 3; i++) {
    K_tmp = static_cast<int>(e_a[i]);
    s1 = e_a[i + 3];
    c1 = e_a[i + 6];
    for (b_K_tmp = 0; b_K_tmp < 3; b_K_tmp++) {
      b_i = b_K_tmp << 2;
      c3 = T67[i + b_i];
      T12_tmp = b_K_tmp + 6 * i;
      b_T12[T12_tmp] = c3;
      b_T12[T12_tmp + 3] = 0.0;
      T12_tmp = b_K_tmp + 6 * (i + 3);
      b_T12[T12_tmp] =
          (static_cast<double>(K_tmp) * T67[b_i] + s1 * T67[b_i + 1]) +
          c1 * T67[b_i + 2];
      b_T12[T12_tmp + 3] = c3;
    }
  }
  coder::inv(b_T12, X67F7);
  // i=8:
  // ------------------------------------------------------------------------------
  // identification of observation matrix K=X*A
  // -------------------------------------------------------------------------------
  // Definition of the first row of the observation matrix
  // Definition of the second row of the observation matrix
  // Definition of the third row of the observation matrix
  // Definition of the forth row of the observation matrix
  // Definition of the 5th row of the observation matrix
  for (int i{0}; i < 6; i++) {
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 = 0.0;
      c1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        b_i = i + 6 * b_K_tmp;
        a_tmp = b_K_tmp + 6 * K_tmp;
        s1 += X12F2[b_i] * X23F3[a_tmp];
        c1 += X23F3[b_i] * X34F4[a_tmp];
      }
      b_i = i + 6 * K_tmp;
      X24F4[b_i] = c1;
      X13F3_tmp[b_i] = s1;
    }
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += X24F4[i + 6 * b_K_tmp] * X45F5[b_K_tmp + 6 * K_tmp];
      }
      X25F5[i + 6 * K_tmp] = s1;
    }
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += X25F5[i + 6 * b_K_tmp] * X56F6[b_K_tmp + 6 * K_tmp];
      }
      X26F6[i + 6 * K_tmp] = s1;
    }
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 = 0.0;
      c1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        b_i = i + 6 * b_K_tmp;
        a_tmp = b_K_tmp + 6 * K_tmp;
        s1 += X26F6[b_i] * X67F7[a_tmp];
        c1 += X34F4[b_i] * X45F5[a_tmp];
      }
      b_i = i + 6 * K_tmp;
      X35F5[b_i] = c1;
      X27F7[b_i] = s1;
    }
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += X35F5[i + 6 * b_K_tmp] * X56F6[b_K_tmp + 6 * K_tmp];
      }
      X36F6[i + 6 * K_tmp] = s1;
    }
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 = 0.0;
      c1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        b_i = i + 6 * b_K_tmp;
        a_tmp = b_K_tmp + 6 * K_tmp;
        s1 += X36F6[b_i] * X67F7[a_tmp];
        c1 += X45F5[b_i] * X56F6[a_tmp];
      }
      b_i = i + 6 * K_tmp;
      X46F6[b_i] = c1;
      X37F7[b_i] = s1;
    }
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 = 0.0;
      c1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        c3 = X67F7[b_K_tmp + 6 * K_tmp];
        b_i = i + 6 * b_K_tmp;
        s1 += X46F6[b_i] * c3;
        c1 += X56F6[b_i] * c3;
      }
      b_i = i + 6 * K_tmp;
      X57F7[b_i] = c1;
      X47F7[b_i] = s1;
    }
  }
  // Definition of the 6th row of the observation matrix
  // Definition of the 7th row of the observation matrix
  //  %Definition of the 8th row of the observation matrix
  // K=[K1;K2;K3;K4;K5;K6;K7;K8];
  b_w22[0] = w11[0];
  b_w22[3] = w11[1];
  b_w22[6] = w11[2];
  b_w22[9] = 0.0;
  b_w22[12] = 0.0;
  b_w22[15] = 0.0;
  b_w22[1] = 0.0;
  b_w22[4] = w11[0];
  b_w22[7] = 0.0;
  b_w22[10] = w11[1];
  b_w22[13] = w11[2];
  b_w22[16] = 0.0;
  b_w22[2] = 0.0;
  b_w22[5] = 0.0;
  b_w22[8] = w11[0];
  b_w22[11] = 0.0;
  b_w22[14] = w11[1];
  b_w22[17] = w11[2];
  b_dw22[0] = s2;
  b_dw22[3] = c2;
  b_dw22[6] = s3;
  b_dw22[9] = 0.0;
  b_dw22[12] = 0.0;
  b_dw22[15] = 0.0;
  b_dw22[1] = 0.0;
  b_dw22[4] = s2;
  b_dw22[7] = 0.0;
  b_dw22[10] = c2;
  b_dw22[13] = s3;
  b_dw22[16] = 0.0;
  b_dw22[2] = 0.0;
  b_dw22[5] = 0.0;
  b_dw22[8] = s2;
  b_dw22[11] = 0.0;
  b_dw22[14] = c2;
  b_dw22[17] = s3;
  dv[0] = 0.0;
  dv[3] = -s3;
  dv[6] = c2;
  dv[1] = s3;
  dv[4] = 0.0;
  dv[7] = -s2;
  dv[2] = -c2;
  dv[5] = s2;
  dv[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    s1 = Sw11[i];
    c1 = Sw11[i + 3];
    c3 = Sw11[i + 6];
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      b_Sw22[i + 3 * K_tmp] =
          (s1 * b_w22[3 * K_tmp] + c1 * b_w22[3 * K_tmp + 1]) +
          c3 * b_w22[3 * K_tmp + 2];
    }
    for (K_tmp = 0; K_tmp < 3; K_tmp++) {
      Sw22[i + 3 * K_tmp] =
          (Sw11[i] * Sw11[3 * K_tmp] + c1 * Sw11[3 * K_tmp + 1]) +
          c3 * Sw11[3 * K_tmp + 2];
    }
    dv1[i] = 0.0;
  }
  dv1[6] = -0.0;
  dv1[12] = dv11[2];
  dv1[18] = -dv11[1];
  dv1[7] = -dv11[2];
  dv1[13] = -0.0;
  dv1[19] = dv11[0];
  dv1[8] = dv11[1];
  dv1[14] = -dv11[0];
  dv1[20] = -0.0;
  for (int i{0}; i < 6; i++) {
    r = _mm_loadu_pd(&b_dw22[3 * i]);
    r1 = _mm_loadu_pd(&b_Sw22[3 * i]);
    K_tmp = 6 * (i + 4);
    _mm_storeu_pd(&dv1[K_tmp], _mm_add_pd(r, r1));
    b_K_tmp = 3 * i + 2;
    dv1[K_tmp + 2] = b_dw22[b_K_tmp] + b_Sw22[b_K_tmp];
  }
  dv1[3] = dv11[0];
  r = _mm_loadu_pd(&dv[0]);
  r1 = _mm_loadu_pd(&Sw22[0]);
  _mm_storeu_pd(&dv1[9], _mm_add_pd(r, r1));
  dv1[11] = -c2 + Sw22[2];
  dv1[4] = dv11[1];
  r = _mm_loadu_pd(&dv[3]);
  r1 = _mm_loadu_pd(&Sw22[3]);
  _mm_storeu_pd(&dv1[15], _mm_add_pd(r, r1));
  dv1[17] = s2 + Sw22[5];
  dv1[5] = dv11[2];
  r = _mm_loadu_pd(&dv[6]);
  r1 = _mm_loadu_pd(&Sw22[6]);
  _mm_storeu_pd(&dv1[21], _mm_add_pd(r, r1));
  dv1[23] = Sw22[8];
  for (int i{0}; i < 6; i++) {
    K_tmp = 6 * (i + 4);
    dv1[K_tmp + 3] = 0.0;
    dv1[K_tmp + 4] = 0.0;
    dv1[K_tmp + 5] = 0.0;
    f_a[i] = 0.0;
    g_a[i] = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      b_i = i_a[K_tmp];
      a_tmp = K_tmp + 6 * i;
      f_a[i] += static_cast<double>(b_i) * X12F2[a_tmp];
      g_a[i] += static_cast<double>(b_i) * X13F3_tmp[a_tmp];
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += X13F3_tmp[i + 6 * b_K_tmp] * X34F4[b_K_tmp + 6 * K_tmp];
      }
      b_T12[i + 6 * K_tmp] = s1;
    }
  }
  for (int i{0}; i < 6; i++) {
    s1 = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 += static_cast<double>(i_a[K_tmp]) * b_T12[K_tmp + 6 * i];
    }
    h_a[i] = s1;
  }
  for (int i{0}; i < 6; i++) {
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += X13F3_tmp[i + 6 * b_K_tmp] * X34F4[b_K_tmp + 6 * K_tmp];
      }
      b_T12[i + 6 * K_tmp] = s1;
    }
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += b_T12[i + 6 * b_K_tmp] * X45F5[b_K_tmp + 6 * K_tmp];
      }
      X12F2[i + 6 * K_tmp] = s1;
    }
  }
  for (int i{0}; i < 6; i++) {
    j_a[i] = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      j_a[i] += static_cast<double>(i_a[K_tmp]) * X12F2[K_tmp + 6 * i];
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += X13F3_tmp[i + 6 * b_K_tmp] * X34F4[b_K_tmp + 6 * K_tmp];
      }
      b_T12[i + 6 * K_tmp] = s1;
    }
  }
  for (int i{0}; i < 6; i++) {
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += b_T12[i + 6 * b_K_tmp] * X45F5[b_K_tmp + 6 * K_tmp];
      }
      X12F2[i + 6 * K_tmp] = s1;
    }
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += X12F2[i + 6 * b_K_tmp] * X56F6[b_K_tmp + 6 * K_tmp];
      }
      b_T12[i + 6 * K_tmp] = s1;
    }
  }
  for (int i{0}; i < 6; i++) {
    s1 = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 += static_cast<double>(i_a[K_tmp]) * b_T12[K_tmp + 6 * i];
    }
    k_a[i] = s1;
  }
  for (int i{0}; i < 6; i++) {
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += X13F3_tmp[i + 6 * b_K_tmp] * X34F4[b_K_tmp + 6 * K_tmp];
      }
      b_T12[i + 6 * K_tmp] = s1;
    }
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += b_T12[i + 6 * b_K_tmp] * X45F5[b_K_tmp + 6 * K_tmp];
      }
      X12F2[i + 6 * K_tmp] = s1;
    }
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += X12F2[i + 6 * b_K_tmp] * X56F6[b_K_tmp + 6 * K_tmp];
      }
      b_T12[i + 6 * K_tmp] = s1;
    }
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += b_T12[i + 6 * b_K_tmp] * X67F7[b_K_tmp + 6 * K_tmp];
      }
      X12F2[i + 6 * K_tmp] = s1;
    }
  }
  for (int i{0}; i < 6; i++) {
    l_a[i] = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      l_a[i] += static_cast<double>(i_a[K_tmp]) * X12F2[K_tmp + 6 * i];
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += X13F3_tmp[i + 6 * b_K_tmp] * X34F4[b_K_tmp + 6 * K_tmp];
      }
      b_T12[i + 6 * K_tmp] = s1;
    }
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += b_T12[i + 6 * b_K_tmp] * X45F5[b_K_tmp + 6 * K_tmp];
      }
      X13F3_tmp[i + 6 * K_tmp] = s1;
    }
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += X13F3_tmp[i + 6 * b_K_tmp] * X56F6[b_K_tmp + 6 * K_tmp];
      }
      b_T12[i + 6 * K_tmp] = s1;
    }
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += b_T12[i + 6 * b_K_tmp] * X67F7[b_K_tmp + 6 * K_tmp];
      }
      X13F3_tmp[i + 6 * K_tmp] = s1;
    }
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += X13F3_tmp[i + 6 * b_K_tmp] * b[b_K_tmp + 6 * K_tmp];
      }
      b_T12[i + 6 * K_tmp] = s1;
    }
  }
  for (int i{0}; i < 6; i++) {
    s1 = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 += static_cast<double>(i_a[K_tmp]) * b_T12[K_tmp + 6 * i];
    }
    m_a[i] = s1;
  }
  for (int i{0}; i < 10; i++) {
    s1 = 0.0;
    c1 = 0.0;
    c3 = 0.0;
    s4 = 0.0;
    c4 = 0.0;
    s5 = 0.0;
    c5 = 0.0;
    s6 = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      b_K_tmp = K_tmp + 6 * i;
      s1 += static_cast<double>(i_a[K_tmp]) * dv1[b_K_tmp];
      c1 += f_a[K_tmp] * A22[b_K_tmp];
      c3 += g_a[K_tmp] * A33[b_K_tmp];
      s4 += h_a[K_tmp] * A44[b_K_tmp];
      c4 += j_a[K_tmp] * A55[b_K_tmp];
      s5 += k_a[K_tmp] * A66[b_K_tmp];
      c5 += l_a[K_tmp] * A77[b_K_tmp];
      s6 += m_a[K_tmp] * A88[b_K_tmp];
    }
    u_a[i] = s6;
    t_a[i] = c5;
    s_a[i] = s5;
    r_a[i] = c4;
    q_a[i] = s4;
    p_a[i] = c3;
    o_a[i] = c1;
    n_a[i] = s1;
  }
  for (int i{0}; i < 6; i++) {
    f_a[i] = 0.0;
    g_a[i] = 0.0;
    h_a[i] = 0.0;
    j_a[i] = 0.0;
    k_a[i] = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      b_i = i_a[K_tmp];
      a_tmp = K_tmp + 6 * i;
      f_a[i] += static_cast<double>(b_i) * X23F3[a_tmp];
      g_a[i] += static_cast<double>(b_i) * X24F4[a_tmp];
      h_a[i] += static_cast<double>(b_i) * X25F5[a_tmp];
      j_a[i] += static_cast<double>(b_i) * X26F6[a_tmp];
      k_a[i] += static_cast<double>(b_i) * X27F7[a_tmp];
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += X27F7[i + 6 * b_K_tmp] * b[b_K_tmp + 6 * K_tmp];
      }
      b_T12[i + 6 * K_tmp] = s1;
    }
  }
  for (int i{0}; i < 6; i++) {
    s1 = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 += static_cast<double>(i_a[K_tmp]) * b_T12[K_tmp + 6 * i];
    }
    l_a[i] = s1;
  }
  for (int i{0}; i < 10; i++) {
    s1 = 0.0;
    c1 = 0.0;
    c3 = 0.0;
    s4 = 0.0;
    c4 = 0.0;
    s5 = 0.0;
    c5 = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      b_K_tmp = K_tmp + 6 * i;
      s1 += static_cast<double>(i_a[K_tmp]) * A22[b_K_tmp];
      c1 += f_a[K_tmp] * A33[b_K_tmp];
      c3 += g_a[K_tmp] * A44[b_K_tmp];
      s4 += h_a[K_tmp] * A55[b_K_tmp];
      c4 += j_a[K_tmp] * A66[b_K_tmp];
      s5 += k_a[K_tmp] * A77[b_K_tmp];
      c5 += l_a[K_tmp] * A88[b_K_tmp];
    }
    cb_a[i] = c5;
    bb_a[i] = s5;
    ab_a[i] = c4;
    y_a[i] = s4;
    x_a[i] = c3;
    w_a[i] = c1;
    v_a[i] = s1;
  }
  for (int i{0}; i < 6; i++) {
    f_a[i] = 0.0;
    g_a[i] = 0.0;
    h_a[i] = 0.0;
    j_a[i] = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      b_i = i_a[K_tmp];
      a_tmp = K_tmp + 6 * i;
      f_a[i] += static_cast<double>(b_i) * X34F4[a_tmp];
      g_a[i] += static_cast<double>(b_i) * X35F5[a_tmp];
      h_a[i] += static_cast<double>(b_i) * X36F6[a_tmp];
      j_a[i] += static_cast<double>(b_i) * X37F7[a_tmp];
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += X37F7[i + 6 * b_K_tmp] * b[b_K_tmp + 6 * K_tmp];
      }
      b_T12[i + 6 * K_tmp] = s1;
    }
  }
  for (int i{0}; i < 6; i++) {
    s1 = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 += static_cast<double>(i_a[K_tmp]) * b_T12[K_tmp + 6 * i];
    }
    k_a[i] = s1;
  }
  for (int i{0}; i < 10; i++) {
    s1 = 0.0;
    c1 = 0.0;
    c3 = 0.0;
    s4 = 0.0;
    c4 = 0.0;
    s5 = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      b_K_tmp = K_tmp + 6 * i;
      s1 += static_cast<double>(i_a[K_tmp]) * A33[b_K_tmp];
      c1 += f_a[K_tmp] * A44[b_K_tmp];
      c3 += g_a[K_tmp] * A55[b_K_tmp];
      s4 += h_a[K_tmp] * A66[b_K_tmp];
      c4 += j_a[K_tmp] * A77[b_K_tmp];
      s5 += k_a[K_tmp] * A88[b_K_tmp];
    }
    ib_a[i] = s5;
    hb_a[i] = c4;
    gb_a[i] = s4;
    fb_a[i] = c3;
    eb_a[i] = c1;
    db_a[i] = s1;
  }
  for (int i{0}; i < 6; i++) {
    f_a[i] = 0.0;
    g_a[i] = 0.0;
    h_a[i] = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      b_i = i_a[K_tmp];
      a_tmp = K_tmp + 6 * i;
      f_a[i] += static_cast<double>(b_i) * X45F5[a_tmp];
      g_a[i] += static_cast<double>(b_i) * X46F6[a_tmp];
      h_a[i] += static_cast<double>(b_i) * X47F7[a_tmp];
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += X47F7[i + 6 * b_K_tmp] * b[b_K_tmp + 6 * K_tmp];
      }
      b_T12[i + 6 * K_tmp] = s1;
    }
  }
  for (int i{0}; i < 6; i++) {
    s1 = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 += static_cast<double>(i_a[K_tmp]) * b_T12[K_tmp + 6 * i];
    }
    j_a[i] = s1;
  }
  for (int i{0}; i < 10; i++) {
    s1 = 0.0;
    c1 = 0.0;
    c3 = 0.0;
    s4 = 0.0;
    c4 = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      b_K_tmp = K_tmp + 6 * i;
      s1 += static_cast<double>(i_a[K_tmp]) * A44[b_K_tmp];
      c1 += f_a[K_tmp] * A55[b_K_tmp];
      c3 += g_a[K_tmp] * A66[b_K_tmp];
      s4 += h_a[K_tmp] * A77[b_K_tmp];
      c4 += j_a[K_tmp] * A88[b_K_tmp];
    }
    nb_a[i] = c4;
    mb_a[i] = s4;
    lb_a[i] = c3;
    kb_a[i] = c1;
    jb_a[i] = s1;
  }
  for (int i{0}; i < 6; i++) {
    f_a[i] = 0.0;
    g_a[i] = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      b_i = i_a[K_tmp];
      a_tmp = K_tmp + 6 * i;
      f_a[i] += static_cast<double>(b_i) * X56F6[a_tmp];
      g_a[i] += static_cast<double>(b_i) * X57F7[a_tmp];
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += X57F7[i + 6 * b_K_tmp] * b[b_K_tmp + 6 * K_tmp];
      }
      b_T12[i + 6 * K_tmp] = s1;
    }
  }
  for (int i{0}; i < 6; i++) {
    s1 = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 += static_cast<double>(i_a[K_tmp]) * b_T12[K_tmp + 6 * i];
    }
    h_a[i] = s1;
  }
  for (int i{0}; i < 10; i++) {
    s1 = 0.0;
    c1 = 0.0;
    c3 = 0.0;
    s4 = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      b_K_tmp = K_tmp + 6 * i;
      s1 += static_cast<double>(i_a[K_tmp]) * A55[b_K_tmp];
      c1 += f_a[K_tmp] * A66[b_K_tmp];
      c3 += g_a[K_tmp] * A77[b_K_tmp];
      s4 += h_a[K_tmp] * A88[b_K_tmp];
    }
    rb_a[i] = s4;
    qb_a[i] = c3;
    pb_a[i] = c1;
    ob_a[i] = s1;
  }
  for (int i{0}; i < 6; i++) {
    f_a[i] = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      f_a[i] += static_cast<double>(i_a[K_tmp]) * X67F7[K_tmp + 6 * i];
      s1 = 0.0;
      for (b_K_tmp = 0; b_K_tmp < 6; b_K_tmp++) {
        s1 += X67F7[i + 6 * b_K_tmp] * b[b_K_tmp + 6 * K_tmp];
      }
      b_T12[i + 6 * K_tmp] = s1;
    }
  }
  for (int i{0}; i < 6; i++) {
    s1 = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      s1 += static_cast<double>(i_a[K_tmp]) * b_T12[K_tmp + 6 * i];
    }
    g_a[i] = s1;
  }
  for (int i{0}; i < 10; i++) {
    s1 = 0.0;
    c1 = 0.0;
    c3 = 0.0;
    s4 = 0.0;
    c4 = 0.0;
    for (K_tmp = 0; K_tmp < 6; K_tmp++) {
      b_K_tmp = K_tmp + 6 * i;
      b_i = i_a[K_tmp];
      s1 += static_cast<double>(b_i) * A66[b_K_tmp];
      s5 = A77[b_K_tmp];
      c1 += f_a[K_tmp] * s5;
      c5 = A88[b_K_tmp];
      c3 += g_a[K_tmp] * c5;
      s4 += static_cast<double>(b_i) * s5;
      c4 += static_cast<double>(b_i) * c5;
    }
    int c_K_tmp;
    int d_K_tmp;
    K[7 * i] = n_a[i];
    b_i = 7 * (i + 10);
    K[b_i] = o_a[i];
    a_tmp = 7 * (i + 20);
    K[a_tmp] = p_a[i];
    b_K_tmp = 7 * (i + 30);
    K[b_K_tmp] = q_a[i];
    T12_tmp = 7 * (i + 40);
    K[T12_tmp] = r_a[i];
    K_tmp = 7 * (i + 50);
    K[K_tmp] = s_a[i];
    c_K_tmp = 7 * (i + 60);
    K[c_K_tmp] = t_a[i];
    d_K_tmp = 7 * (i + 70);
    K[d_K_tmp] = u_a[i];
    K[7 * i + 1] = 0.0;
    K[b_i + 1] = v_a[i];
    K[a_tmp + 1] = w_a[i];
    K[b_K_tmp + 1] = x_a[i];
    K[T12_tmp + 1] = y_a[i];
    K[K_tmp + 1] = ab_a[i];
    K[c_K_tmp + 1] = bb_a[i];
    K[d_K_tmp + 1] = cb_a[i];
    K[7 * i + 2] = 0.0;
    K[b_i + 2] = 0.0;
    K[a_tmp + 2] = db_a[i];
    K[b_K_tmp + 2] = eb_a[i];
    K[T12_tmp + 2] = fb_a[i];
    K[K_tmp + 2] = gb_a[i];
    K[c_K_tmp + 2] = hb_a[i];
    K[d_K_tmp + 2] = ib_a[i];
    K[7 * i + 3] = 0.0;
    K[b_i + 3] = 0.0;
    K[a_tmp + 3] = 0.0;
    K[b_K_tmp + 3] = jb_a[i];
    K[T12_tmp + 3] = kb_a[i];
    K[K_tmp + 3] = lb_a[i];
    K[c_K_tmp + 3] = mb_a[i];
    K[d_K_tmp + 3] = nb_a[i];
    K[7 * i + 4] = 0.0;
    K[b_i + 4] = 0.0;
    K[a_tmp + 4] = 0.0;
    K[b_K_tmp + 4] = 0.0;
    K[T12_tmp + 4] = ob_a[i];
    K[K_tmp + 4] = pb_a[i];
    K[c_K_tmp + 4] = qb_a[i];
    K[d_K_tmp + 4] = rb_a[i];
    K[7 * i + 5] = 0.0;
    K[b_i + 5] = 0.0;
    K[a_tmp + 5] = 0.0;
    K[b_K_tmp + 5] = 0.0;
    K[T12_tmp + 5] = 0.0;
    K[K_tmp + 5] = s1;
    K[c_K_tmp + 5] = c1;
    K[d_K_tmp + 5] = c3;
    K[7 * i + 6] = 0.0;
    K[b_i + 6] = 0.0;
    K[a_tmp + 6] = 0.0;
    K[b_K_tmp + 6] = 0.0;
    K[T12_tmp + 6] = 0.0;
    K[K_tmp + 6] = 0.0;
    K[c_K_tmp + 6] = s4;
    K[d_K_tmp + 6] = c4;
  }
  // size(K);
  // KLFRanka = K(1:7,71:80);
}

// End of code generation (Observation_matrix_Franka.cpp)
