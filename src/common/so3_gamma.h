// pimax_slam.pi.dll -- src/common/so3_gamma.h  (from the c03_estimator draft)
//
// SO(3) "Gamma" functions and their Jacobians used by the Pimax closed-form IMU preintegration in
// ceres_backend/imu_error.cpp (callers: propagation 0x1800427f0, redoPreintegration 0x180045750).
// FILE NAME / NAMESPACE / FUNCTION NAMES ARE GUESSES -- the binary has no symbols and no __FILE__
// string for these (they never assert/log).  They sit (as inline/COMDAT code) in the imu_error object.
//
// Notation (verified numerically against finite differences):
//   theta = |phi|, u = phi/theta, K = hat(u), s = sin(theta)/theta, c = cos(theta), a = (1-c)/theta^2
//   Gamma_n(phi) = sum_k hat(phi)^k / (k+n)!      (Gamma_0 = exp, Gamma_1 = left Jacobian Jl)
//   The functions below return the *scaled* n! * Gamma_n(phi) (identity coefficient 1).
//   Each one switches to an alternating series (Sophus::alternatingSeries, 0x1800409e0) when
//   theta < eps; eps is passed by const reference from caller constants:
//     1e-5 (0x1803AF4F8)  0.001 (0x1803AF500)  0.02 (0x1803AF508)  0.06 (0x1803AF510)  0.1 (0x1803AF518)
//
// Eigen evaluation notes: every function first writes Identity into the result, then assigns/adds.
// scalar*matrix creates CwiseNullaryOp constants (0x180022AF0); MSVC builds the right operand of a
// '+' first, which is why the constants of the K^2 term are created before those of the K term.
#pragma once

#include <cmath>

#include <Eigen/Core>

#include "common/sophus/so3ex_base.h"

namespace pimax {
namespace totem {

using Sophus::alternatingSeries;  // 0x1800409e0

// ----------------------------------------------------------------------------------------------
// n! * Gamma_n(phi)
// ----------------------------------------------------------------------------------------------

// 0x180040ad0  Gamma_1 = left Jacobian Jl(phi).  Callers: 0x1800427f0, 0x180045750 (eps 1e-5).
inline Eigen::Matrix3d gamma1(const Eigen::Vector3d& phi, const double& eps) {
  Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
  const double theta = phi.norm();
  const Eigen::Matrix3d K = Sophus::SO3d::hat(phi.normalized());
  if (theta < eps) {
    const double b = alternatingSeries(theta, 3, 1) * theta;  // 1 - sin(t)/t
    const double a = alternatingSeries(theta, 2, 0) * theta;  // (1 - cos(t))/t
    G += a * K + b * K * K;
  } else {
    const double a = (1.0 - std::cos(theta)) / theta;
    const double b = 1.0 - std::sin(theta) / theta;
    G += a * K + b * K * K;
  }
  return G;
}

// 0x180040fe0  2 * Gamma_2.  Callers: 0x1800427f0, 0x180045750 (eps 0.001).
inline Eigen::Matrix3d gamma2(const Eigen::Vector3d& phi, const double& eps) {
  Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
  const double theta = phi.norm();
  const Eigen::Matrix3d K = Sophus::SO3d::hat(phi.normalized());
  if (theta < eps) {
    const double b = alternatingSeries(theta, 4, 1) * theta;
    const double a = alternatingSeries(theta, 3, 0) * theta;
    G += 2.0 * (a * K + b * K * K);
  } else {
    const double A = 2.0 * (1.0 - std::sin(theta) / theta) / theta;
    const double B = 1.0 - 2.0 * ((1.0 - std::cos(theta)) / (theta * theta));
    G += A * K + B * K * K;
  }
  return G;
}

// 0x180041600  6 * Gamma_3.  Callers: 0x1800427f0, 0x180045750 (eps 0.02).
inline Eigen::Matrix3d gamma3(const Eigen::Vector3d& phi, const double& eps) {
  Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
  const double theta = phi.norm();
  const Eigen::Matrix3d K = Sophus::SO3d::hat(phi.normalized());
  if (theta < eps) {
    const double b = alternatingSeries(theta, 5, 1) * theta;
    const double a = alternatingSeries(theta, 4, 0) * theta;
    G += 6.0 * (a * K + b * K * K);
  } else {
    const double theta2 = theta * theta;
    const double A = (1.0 - 2.0 * ((1.0 - std::cos(theta)) / theta2)) * 3.0 / theta;
    const double B = 1.0 - (1.0 - std::sin(theta) / theta) * 6.0 / theta2;
    G += A * K + B * K * K;
  }
  return G;
}

// 0x180041c40  right Jacobian Jr(phi) = Jl(-phi).  Caller: 0x180045750 (eps 1e-5).
inline Eigen::Matrix3d gamma1Right(const Eigen::Vector3d& phi, const double& eps) {
  return gamma1(-phi, eps);
}

// ----------------------------------------------------------------------------------------------
// Jacobians.  Common shape (4 scalar coefficients e1..e4, identical closed forms in both families):
//
//   family B ("dGammaV"):  J = e2*hat(-v) + (e4*K + e3*K*K) * (v*u^T) + e1*(u*v^T - 2*v*u^T + (u.v)*I)
//       == d( n!*Gamma_n(phi) * v ) / d phi   EXACTLY (checked numerically for n = 1..4).
//   family A ("dGammaTV"): J = e2*hat(v)  + (e4*K + e3*K*K) * ((u.v)*I) + e1*(u*v^T - 2*(u.v)*I + v*u^T)
//       This is NOT an exact derivative of anything simple: it agrees with d(n!*Gamma_n(phi)^T v)/dphi
//       only in the hat(v), (u.v)K^2 and u*v^T coefficients.  Probably a Pimax derivation error --
//       keep it bit-exact.  TODO(verify) intent/name.
//
// Small-angle branch: e1 = t*S(n+2,0), e2 = S(n+1,0), e3 = ((n+2)*S(n+4,2) - t*S(n+3,1))*t,
//   e4 = ((n+1)*S(n+3,1) - t*S(n+2,0))*t with S(m,p) = alternatingSeries(t, m, p);
//   result then multiplied by n! (no multiply for n = 1).
// Closed branch per n (s = sin t / t, c = cos t, a = (1-c)/t^2, t2 = t*t):
//   n=1: e1 = (1-s)/t                       e2 = a
//        e3 = (3s - (c+2))/t                 e4 = s - 2a
//   n=2: e1 = (1-2a)/t                       e2 = 2(1-s)/t2
//        e3 = (-1 - s + 4a)*(2/t)            e4 = (3s - (c+2))*(2/t2)
//   n=3: e1 = (1 - 6(1-s)/t2)/t              e2 = 3(1-2a)/t2
//        e3 = ((3c+12)/t2 - 1 - (15/t2)s)*(2/t)   e4 = (-1 - s + 4a)*(6/t2)
//   n=4: e1 = (1 - 12(1-2a)/t2)/t            e2 = 4(1 - 6(1-s)/t2)/t2
//        e3 = ((s + 2 - 6a)*(12/t2) - 1)*(2/t)    e4 = ((3c+12)/t2 - 1 - (15/t2)s)*(8/t2)
// The scalar values are computed in the order e1, e2, e3, e4 (the order of the constant objects).
// ----------------------------------------------------------------------------------------------

// ---- family A ---------------------------------------------------------------------------------

// 0x18002ef20  family A, n = 2.  Callers: 0x1800427f0, 0x180045750 (eps 0.02).
inline Eigen::Matrix3d dGammaTV2(const Eigen::Vector3d& phi, const Eigen::Vector3d& v, const double& eps) {
  Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
  const double theta = phi.norm();
  const Eigen::Vector3d u = phi.normalized();
  const Eigen::Matrix3d K = Sophus::SO3d::hat(u);
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const double uv = u.dot(v);
  const Eigen::Matrix3d uvI = uv * I;
  const Eigen::Matrix3d Hv = Sophus::SO3d::hat(v);
  const Eigen::Matrix3d W = u * v.transpose() - 2.0 * uvI + v * u.transpose();
  if (theta < eps) {
    const double s30 = alternatingSeries(theta, 3, 0);
    const double s40 = alternatingSeries(theta, 4, 0);
    const double s51 = alternatingSeries(theta, 5, 1);
    const double s62 = alternatingSeries(theta, 6, 2);
    const double e1 = theta * s40;
    const double e2 = s30;
    const double e3 = (s62 * 4.0 - theta * s51) * theta;
    const double e4 = (s51 * 3.0 - theta * s40) * theta;
    J = e2 * Hv + (e4 * K + e3 * K * K) * uvI + e1 * W;
    J *= 2.0;
  } else {
    const double theta2 = theta * theta;
    const double s = std::sin(theta) / theta;
    const double c = std::cos(theta);
    const double a = (1.0 - c) / theta2;
    const double e1 = (1.0 - 2.0 * a) / theta;
    const double e2 = 2.0 * (1.0 - s) / theta2;
    const double e3 = (-1.0 - s + a * 4.0) * (2.0 / theta);
    const double e4 = (s * 3.0 - (c + 2.0)) * (2.0 / theta2);
    J = e2 * Hv + (e4 * K + e3 * K * K) * uvI + e1 * W;
  }
  return J;
}

// 0x1800319c0  family A, n = 3.  Callers: 0x1800427f0, 0x180045750 (eps 0.06).
inline Eigen::Matrix3d dGammaTV3(const Eigen::Vector3d& phi, const Eigen::Vector3d& v, const double& eps) {
  Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
  const double theta = phi.norm();
  const Eigen::Vector3d u = phi.normalized();
  const Eigen::Matrix3d K = Sophus::SO3d::hat(u);
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const double uv = u.dot(v);
  const Eigen::Matrix3d uvI = uv * I;
  const Eigen::Matrix3d Hv = Sophus::SO3d::hat(v);
  const Eigen::Matrix3d W = u * v.transpose() - 2.0 * uvI + v * u.transpose();
  if (theta < eps) {
    const double s40 = alternatingSeries(theta, 4, 0);
    const double s50 = alternatingSeries(theta, 5, 0);
    const double s61 = alternatingSeries(theta, 6, 1);
    const double s72 = alternatingSeries(theta, 7, 2);
    const double e1 = theta * s50;
    const double e2 = s40;
    const double e3 = (s72 * 5.0 - theta * s61) * theta;
    const double e4 = (s61 * 4.0 - theta * s50) * theta;
    J = e2 * Hv + (e4 * K + e3 * K * K) * uvI + e1 * W;
    J *= 6.0;
  } else {
    const double theta2 = theta * theta;
    const double s = std::sin(theta) / theta;
    const double c = std::cos(theta);
    const double a = (1.0 - c) / theta2;
    const double e1 = (1.0 - (1.0 - s) * 6.0 / theta2) / theta;
    const double e2 = (1.0 - 2.0 * a) * 3.0 / theta2;
    const double e3 = ((c * 3.0 + 12.0) / theta2 - 1.0 - 15.0 / theta2 * s) * (2.0 / theta);
    const double e4 = (-1.0 - s + a * 4.0) * (6.0 / theta2);
    J = e2 * Hv + (e4 * K + e3 * K * K) * uvI + e1 * W;
  }
  return J;
}

// 0x1800344e0  family A, n = 4.  Callers: 0x1800427f0, 0x180045750 (eps 0.1).
inline Eigen::Matrix3d dGammaTV4(const Eigen::Vector3d& phi, const Eigen::Vector3d& v, const double& eps) {
  Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
  const double theta = phi.norm();
  const Eigen::Vector3d u = phi.normalized();
  const Eigen::Matrix3d K = Sophus::SO3d::hat(u);
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const double uv = u.dot(v);
  const Eigen::Matrix3d uvI = uv * I;
  const Eigen::Matrix3d Hv = Sophus::SO3d::hat(v);
  const Eigen::Matrix3d W = u * v.transpose() - 2.0 * uvI + v * u.transpose();
  if (theta < eps) {
    const double s50 = alternatingSeries(theta, 5, 0);
    const double s60 = alternatingSeries(theta, 6, 0);
    const double s71 = alternatingSeries(theta, 7, 1);
    const double s82 = alternatingSeries(theta, 8, 2);
    const double e1 = theta * s60;
    const double e2 = s50;
    const double e3 = (s82 * 6.0 - theta * s71) * theta;
    const double e4 = (s71 * 5.0 - theta * s60) * theta;
    J = e2 * Hv + (e4 * K + e3 * K * K) * uvI + e1 * W;
    J *= 24.0;
  } else {
    const double theta2 = theta * theta;
    const double s = std::sin(theta) / theta;
    const double c = std::cos(theta);
    const double a = (1.0 - c) / theta2;
    const double e1 = (1.0 - (1.0 - 2.0 * a) * 12.0 / theta2) / theta;
    const double e2 = (1.0 - (1.0 - s) * 6.0 / theta2) * 4.0 / theta2;
    const double e3 = ((s + 2.0 - a * 6.0) * (12.0 / theta2) - 1.0) * (2.0 / theta);
    const double e4 = ((c * 3.0 + 12.0) / theta2 - 1.0 - 15.0 / theta2 * s) * (8.0 / theta2);
    J = e2 * Hv + (e4 * K + e3 * K * K) * uvI + e1 * W;
  }
  return J;
}

// ---- family B (exact d(n! Gamma_n(phi) v)/dphi) -----------------------------------------------

// 0x180035ad0  family B, n = 1 (= d(Jl(phi) v)/dphi).  Caller: 0x180045750 (eps 0.001).
// NOTE: series call order differs from the other functions: S(3,0), S(4,1), S(2,0), S(5,2).
inline Eigen::Matrix3d dGammaV1(const Eigen::Vector3d& phi, const Eigen::Vector3d& v, const double& eps) {
  Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
  const double theta = phi.norm();
  const Eigen::Vector3d u = phi.normalized();
  const Eigen::Matrix3d K = Sophus::SO3d::hat(u);
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d vu = v * u.transpose();
  const Eigen::Matrix3d Hv = Sophus::SO3d::hat(-v);
  const double uv = u.dot(v);
  const Eigen::Matrix3d R = vu.transpose() - 2.0 * vu + uv * I;
  if (theta < eps) {
    const double s30 = alternatingSeries(theta, 3, 0);
    const double s41 = alternatingSeries(theta, 4, 1);
    const double s20 = alternatingSeries(theta, 2, 0);
    const double s52 = alternatingSeries(theta, 5, 2);
    const double e1 = theta * s30;
    const double e2 = s20;
    const double e3 = (s52 * 3.0 - theta * s41) * theta;
    const double e4 = (s41 * 2.0 - theta * s30) * theta;
    J = e2 * Hv + (e4 * K + e3 * K * K) * vu + e1 * R;
  } else {
    const double s = std::sin(theta) / theta;
    const double c = std::cos(theta);
    const double a = (1.0 - c) / (theta * theta);
    const double e1 = (1.0 - s) / theta;
    const double e2 = a;
    const double e3 = (s * 3.0 - (c + 2.0)) / theta;
    const double e4 = s - 2.0 * a;
    J = e2 * Hv + (e4 * K + e3 * K * K) * vu + e1 * R;
  }
  return J;
}

// 0x180030480  family B, n = 2.  Caller: 0x180045750 (eps 0.02).
inline Eigen::Matrix3d dGammaV2(const Eigen::Vector3d& phi, const Eigen::Vector3d& v, const double& eps) {
  Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
  const double theta = phi.norm();
  const Eigen::Vector3d u = phi.normalized();
  const Eigen::Matrix3d K = Sophus::SO3d::hat(u);
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d vu = v * u.transpose();
  const Eigen::Matrix3d Hv = Sophus::SO3d::hat(-v);
  const double uv = u.dot(v);
  const Eigen::Matrix3d R = vu.transpose() - 2.0 * vu + uv * I;
  if (theta < eps) {
    const double s30 = alternatingSeries(theta, 3, 0);
    const double s40 = alternatingSeries(theta, 4, 0);
    const double s51 = alternatingSeries(theta, 5, 1);
    const double s62 = alternatingSeries(theta, 6, 2);
    const double e1 = theta * s40;
    const double e2 = s30;
    const double e3 = (s62 * 4.0 - theta * s51) * theta;
    const double e4 = (s51 * 3.0 - theta * s40) * theta;
    J = e2 * Hv + (e4 * K + e3 * K * K) * vu + e1 * R;
    J *= 2.0;
  } else {
    const double theta2 = theta * theta;
    const double s = std::sin(theta) / theta;
    const double c = std::cos(theta);
    const double a = (1.0 - c) / theta2;
    const double e1 = (1.0 - 2.0 * a) / theta;
    const double e2 = 2.0 * (1.0 - s) / theta2;
    const double e3 = (-1.0 - s + a * 4.0) * (2.0 / theta);
    const double e4 = (s * 3.0 - (c + 2.0)) * (2.0 / theta2);
    J = e2 * Hv + (e4 * K + e3 * K * K) * vu + e1 * R;
  }
  return J;
}

// 0x180032f70  family B, n = 3.  Caller: 0x180045750 (eps 0.06).
inline Eigen::Matrix3d dGammaV3(const Eigen::Vector3d& phi, const Eigen::Vector3d& v, const double& eps) {
  Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
  const double theta = phi.norm();
  const Eigen::Vector3d u = phi.normalized();
  const Eigen::Matrix3d K = Sophus::SO3d::hat(u);
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d vu = v * u.transpose();
  const Eigen::Matrix3d Hv = Sophus::SO3d::hat(-v);
  const double uv = u.dot(v);
  const Eigen::Matrix3d R = vu.transpose() - 2.0 * vu + uv * I;
  if (theta < eps) {
    const double s40 = alternatingSeries(theta, 4, 0);
    const double s50 = alternatingSeries(theta, 5, 0);
    const double s61 = alternatingSeries(theta, 6, 1);
    const double s72 = alternatingSeries(theta, 7, 2);
    const double e1 = theta * s50;
    const double e2 = s40;
    const double e3 = (s72 * 5.0 - theta * s61) * theta;
    const double e4 = (s61 * 4.0 - theta * s50) * theta;
    J = e2 * Hv + (e4 * K + e3 * K * K) * vu + e1 * R;
    J *= 6.0;
  } else {
    const double theta2 = theta * theta;
    const double s = std::sin(theta) / theta;
    const double c = std::cos(theta);
    const double a = (1.0 - c) / theta2;
    const double e1 = (1.0 - (1.0 - s) * 6.0 / theta2) / theta;
    const double e2 = (1.0 - 2.0 * a) * 3.0 / theta2;
    const double e3 = ((c * 3.0 + 12.0) / theta2 - 1.0 - 15.0 / theta2 * s) * (2.0 / theta);
    const double e4 = (-1.0 - s + a * 4.0) * (6.0 / theta2);
    J = e2 * Hv + (e4 * K + e3 * K * K) * vu + e1 * R;
  }
  return J;
}

}  // namespace totem
}  // namespace pimax
