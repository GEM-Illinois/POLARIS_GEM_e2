#ifndef APPROX_REAL_CBMC_H
#define APPROX_REAL_CBMC_H
#include <stdint.h>

typedef __CPROVER_fixedbv[32][16] ApproxReal;

const ApproxReal ONE = (ApproxReal) 1;
const ApproxReal APPROX_PI = (ApproxReal) 3.1415863037109375;  // pi ~ 205887/65536
const ApproxReal APPROX_PI_2 = (ApproxReal) 1.57080078125;  // pi/2 ~ 102944/65536

typedef __CPROVER_fixedbv[64][32] fix32_t;

ApproxReal nondet_real();
// Provide models of apprximated functions for signed fixedbv[32][16]
static inline ApproxReal approx_abs(ApproxReal x) {
  return (x >=0)? x : -x;
}
static inline ApproxReal approx_mul(ApproxReal x, ApproxReal y) {
  // NOTE: We only check overflow. x * y can still loss precision.
  fix32_t ret_32 = ((fix32_t) x) * ((fix32_t) y);
  __CPROVER_postcondition(-32768 <= ret_32 && ret_32 < 32768, "no overflow for multiplication");
  return x * y;
}
static inline ApproxReal approx_modulo(ApproxReal y_fix16, ApproxReal x_fix16) {
  fix32_t y_fix32 = (fix32_t) y_fix16;
  fix32_t x_fix32 = (fix32_t) x_fix16;

  y_fix32 = y_fix32 * 65536;
  x_fix32 = x_fix32 * 65536;
  int32_t y_int32 = (int32_t) y_fix32;
  int32_t x_int32 = (int32_t) x_fix32;
  __CPROVER_assert((y_fix32 == (fix32_t) y_int32) && (x_fix32 == (fix32_t) x_int32), "lossless conversion from fix32 to int32");

  fix32_t rem_fix32 = (fix32_t) (y_int32 % x_int32);
  rem_fix32 = rem_fix32 / 65536;

  // lossless conversion
  ApproxReal rem_fix16 = (ApproxReal) rem_fix32;
  __CPROVER_postcondition(rem_fix32 == (fix32_t) rem_fix16, "lossless conversion from fix32 to fix16");
  return rem_fix16;
}
static inline ApproxReal approx_asin(ApproxReal x) {
  /**
   * For -1 <= x <= 1,
   *     asin(x) = sum ((2n)!*x^(2n+1)) / (2^2n*(n!)^2*(2n+1))
   *             = x + x^3/6 + 3x^5/40 + 5x^7/112 + 35x^9/1152 + ...
   * Error is bounded by TODO
   */
  __CPROVER_precondition(-ONE <= x && x <= ONE, "-1 <= x <= 1");
  ApproxReal ret = 0;
  const ApproxReal x_sq = x*x;
  ApproxReal x_term = x;
  ret += x_term;
  x_term *= x_sq;  // x^3
  ret += x_term / 6;
  x_term *= x_sq;  // x^5
  ret += x_term * 3 / 40;
  x_term *= x_sq;  // x^7
  ret += x_term * 5 / 112;
  x_term *= x_sq;  // x^9
  ret += x_term * 35 / 1152;
  return ret;
}
static inline ApproxReal fix16_atan_impl(ApproxReal x) {
  /**
   * Approximation of arctan using the first `n` terms of Taylor expansion.
   * For 0 <= x <= 1,
   *     atan(x) = x - x^3/3 + x^5/5 - x^7/7 + ...
   * Error is bounded by
   *     |err| <= x^(2n+1) / (2n+1)
   */
  __CPROVER_precondition(0 <= x && x <= ONE, "0 <= x <= 1");
  const ApproxReal x_sq = x*x;

  ApproxReal x_term = x;
  ApproxReal ret = 0;
  ret += x_term;
  x_term *= x_sq;  // x^3
  ret -= x_term / 3;
  x_term *= x_sq;  // x^5
  ret += x_term / 5;
  x_term *= x_sq;  // x^7
  ret -= x_term / 7;
  x_term *= x_sq;  // x^9
  ret += x_term / 9;
  x_term *= x_sq;  // x^11
  ret -= x_term / 11;
  x_term *= x_sq;  // x^13
  ret += x_term / 13;
  x_term *= x_sq;  // x^15
  ret -= x_term / 15;
  return ret;
}
static inline ApproxReal approx_atan(ApproxReal x) {
  /**
   * For |x| > 1, use atan(|x|) = pi/2 - atan(1/|x|)
   * For x < 0, use atan(x) = -atan(|x|)
   */
  ApproxReal abs_x = approx_abs(x);

  ApproxReal ret = (abs_x <= ONE)?
      fix16_atan_impl(abs_x) : (APPROX_PI_2 - fix16_atan_impl(ONE / abs_x));

  return (x >= 0)? ret : -ret;
}
static inline ApproxReal approx_atan2(ApproxReal y, ApproxReal x) {
  if (x > 0) {
    return approx_atan(y / x);
  } else {
    // TODO model when x is negative
    __CPROVER_assert(0, "not implemented case is unreachable.");
    return nondet_real();
  }
}

static inline ApproxReal normalize_angle(ApproxReal angle) {
  ApproxReal tempAngle = approx_modulo(angle, (APPROX_PI*2));
  if(tempAngle > APPROX_PI)
    tempAngle -= (APPROX_PI*2);
  else if(tempAngle < -APPROX_PI)
    tempAngle += (APPROX_PI*2);
  __CPROVER_postcondition(-APPROX_PI <= tempAngle && tempAngle <= APPROX_PI, "normalized angle is in [-pi, pi]");
  return tempAngle;
}

static inline ApproxReal approx_sin(ApproxReal x) {
  /**
   * Approximation of sine using the first few terms of Taylor expansion.
   * For -pi <= x <= pi,
   *    sin(x) = x - x^3/(3!) + x^5/(5!) - x^7/(7!) + ...
   */
  ApproxReal x_term = normalize_angle(x);
  const ApproxReal x_sq = x_term*x_term;

  ApproxReal ret = 0;
  ret += x_term;
  x_term *= x_sq;  // x^3
  ret -= x_term / 6;
  x_term *= x_sq;  // x^5
  ret += x_term / 120;
  x_term *= x_sq;  // x^7
  ret -= x_term / 5040;
  return ret;
}

static inline ApproxReal approx_cos(ApproxReal x) {
  /**
   * Approximation of cosine
   *    cos(x) = sin(x + pi/2)
   */
   return approx_sin(x + APPROX_PI/2);
}

static inline ApproxReal approx_tan(ApproxReal x) {
  /**
   * Approximation of tangent
   *    tan(x) = TODO
   */
   return approx_sin(x) / approx_cos(x);
}

#endif  // APPROX_REAL_CBMC_H
