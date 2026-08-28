//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// inv.cpp
//
// Code generation for function 'inv'
//

// Include files
#include "inv.h"
#include <cmath>

// Function Definitions
namespace coder {
void inv(const double x[36], double y[36])
{
  double b_x[36];
  double smax;
  int i;
  int jA;
  int jp1j;
  int kAcol;
  int x_tmp;
  signed char ipiv[6];
  signed char p[6];
  for (i = 0; i < 36; i++) {
    y[i] = 0.0;
    b_x[i] = x[i];
  }
  for (i = 0; i < 6; i++) {
    ipiv[i] = static_cast<signed char>(i + 1);
  }
  for (int j{0}; j < 5; j++) {
    int b_tmp;
    int mmj_tmp;
    mmj_tmp = 4 - j;
    b_tmp = j * 7;
    jp1j = b_tmp + 2;
    jA = 6 - j;
    kAcol = 0;
    smax = std::abs(b_x[b_tmp]);
    for (int k{2}; k <= jA; k++) {
      double s;
      s = std::abs(b_x[(b_tmp + k) - 1]);
      if (s > smax) {
        kAcol = k - 1;
        smax = s;
      }
    }
    if (b_x[b_tmp + kAcol] != 0.0) {
      if (kAcol != 0) {
        jA = j + kAcol;
        ipiv[j] = static_cast<signed char>(jA + 1);
        for (int k{0}; k < 6; k++) {
          kAcol = j + k * 6;
          smax = b_x[kAcol];
          x_tmp = jA + k * 6;
          b_x[kAcol] = b_x[x_tmp];
          b_x[x_tmp] = smax;
        }
      }
      i = (b_tmp - j) + 6;
      for (int b_i{jp1j}; b_i <= i; b_i++) {
        b_x[b_i - 1] /= b_x[b_tmp];
      }
    }
    jA = b_tmp;
    for (kAcol = 0; kAcol <= mmj_tmp; kAcol++) {
      smax = b_x[(b_tmp + kAcol * 6) + 6];
      if (smax != 0.0) {
        i = jA + 8;
        jp1j = (jA - j) + 12;
        for (x_tmp = i; x_tmp <= jp1j; x_tmp++) {
          b_x[x_tmp - 1] += b_x[((b_tmp + x_tmp) - jA) - 7] * -smax;
        }
      }
      jA += 6;
    }
  }
  for (i = 0; i < 6; i++) {
    p[i] = static_cast<signed char>(i + 1);
  }
  for (int k{0}; k < 5; k++) {
    signed char i1;
    i1 = ipiv[k];
    if (i1 > k + 1) {
      jA = p[i1 - 1];
      p[i1 - 1] = p[k];
      p[k] = static_cast<signed char>(jA);
    }
  }
  for (int k{0}; k < 6; k++) {
    x_tmp = 6 * (p[k] - 1);
    y[k + x_tmp] = 1.0;
    for (int j{k + 1}; j < 7; j++) {
      i = (j + x_tmp) - 1;
      if (y[i] != 0.0) {
        jp1j = j + 1;
        for (int b_i{jp1j}; b_i < 7; b_i++) {
          jA = (b_i + x_tmp) - 1;
          y[jA] -= y[i] * b_x[(b_i + 6 * (j - 1)) - 1];
        }
      }
    }
  }
  for (int j{0}; j < 6; j++) {
    jA = 6 * j;
    for (int k{5}; k >= 0; k--) {
      kAcol = 6 * k;
      i = k + jA;
      smax = y[i];
      if (smax != 0.0) {
        y[i] = smax / b_x[k + kAcol];
        for (int b_i{0}; b_i < k; b_i++) {
          x_tmp = b_i + jA;
          y[x_tmp] -= y[i] * b_x[b_i + kAcol];
        }
      }
    }
  }
}

} // namespace coder

// End of code generation (inv.cpp)
