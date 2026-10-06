// pimax_slam.pi.dll -- src/frontend/pose_local_parameterization.h  (from draft c08)
//
// VINS-Mono style local parameterizations used by the
// VINS-like IMU initialisation in FrameProcessorBase (0x180110490, c09+):
//   pimax::totem::PoseLocalParameterization   (vtable 0x1803B2DA8)  -- VINS-Mono, identical
//   pimax::totem::InitGravityLocalParameter   (vtable 0x1803B2E38)  -- pimax-new (S^2 gravity, 2 dof,
//                                                                      VINS RefineGravity math)
// NOT to be confused with pimax::totem::ceres_backend::PoseLocalParameterization (OKVIS-derived,
// vtables 0x1803ADF58/0x1803ADF90, ceres_backend chunk).
//
// Both classes are header-only: their virtual functions are COMDATs emitted into
// frame_processor_base.obj (first user: 0x180110490 does `new PoseLocalParameterization` and
// `new InitGravityLocalParameter`, each via Eigen's aligned operator new = malloc 8 bytes,
// 0x180011350) -- hence EIGEN_MAKE_ALIGNED_OPERATOR_NEW: the scalar deleting destructor
// 0x1800EC0A0 (shared by both classes, ICF) releases with free() and not with operator delete.
//
// vtable PoseLocalParameterization 0x1803B2DA8:
//   [0] 0x1800EC0A0 scalar deleting dtor (ICF-shared)   [1] 0x1800F1DD0 Plus
//   [2] 0x1800ECF10 ComputeJacobian                     [3] 0x1801B9350 ceres MultiplyByJacobian
//   [4] 0x18008DA00 GlobalSize (return 7; ICF)          [5] 0x18001A920 LocalSize (return 6; ICF)
// vtable InitGravityLocalParameter 0x1803B2E38:
//   [0] 0x1800EC0A0 scalar deleting dtor                [1] 0x1800F1A60 Plus
//   [2] 0x18001A620 ComputeJacobian (ICF-folded with an identical function of an earlier object,
//       presumably ceres_backend's GravityLocalParameterization -- not in this chunk)
//   [3] 0x1801B9350 ceres MultiplyByJacobian            [4] 0x18001A910 GlobalSize (return 3; ICF)
//   [5] 0x180014A10 LocalSize (return 2; ICF-folded with boost codecvt_null<wchar_t>::do_encoding)
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include "frontend/utility.h"
#include "ceres_backend/gravity_local_parameterization.hpp"   // TangentBasis (0x18001ACE0)

namespace pimax {
namespace totem {

class PoseLocalParameterization : public ceres::LocalParameterization
{
 public:
  // 0x1800F1DD0 -- upstream-identical (VINS-Mono pose_local_parameterization.cpp).
  // Binary order: dq = deltaQ(delta+3) first (x,y via divpd by {2.0,2.0} 0x1803B6DC0, z * 0.5,
  // w = 1.0), then p = _p + dp, then q = (_q * dq).normalized() (normalize guarded by norm2 > 0).
  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override
  {
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();

    return true;
  }

  // 0x1800ECF10 -- upstream-identical. 7x6 row-major: first 36 doubles zeroed, six 1.0 on the
  // diagonal (stride 7), then the last row (6 doubles) zeroed.
  bool ComputeJacobian(const double* x, double* jacobian) const override
  {
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
  }

  int GlobalSize() const override { return 7; }  // 0x18008DA00
  int LocalSize() const override { return 6; }   // 0x18001A920

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Gravity direction parameterization on S^2 (keeps |g|): x = g (3), delta = dg (2) in the
// tangent basis of g. Pimax-new; the math is VINS-Mono's RefineGravity update
//   g0 = (g0 + lxly * dg).normalized() * G.norm()
// with G.norm() replaced by the norm of the current estimate.
class InitGravityLocalParameter : public ceres::LocalParameterization
{
 public:
  // 0x1800F1A60. Binary order: copy g (3 doubles); MatrixXd lxly(3,2) (free(nullptr) +
  // aligned malloc of 6 doubles, dims {3,2} from 0x1803AE930); lxly = TangentBasis(g)
  // (0x18001ACE0 returns a MatrixXd by value, move-assigned = pointer swap, old buffer freed);
  // dg = Vector2d(delta) (aligned stack copy); norm = |g| (sqrtpd of squaredNorm);
  // asserts lxly.cols()==2 (Product.h:0x62) and lxly.rows()==3 (CwiseBinaryOp.h:0x74);
  // sum = lxly.col(0)*dg(0) + lxly.col(1)*dg(1) + g (coefficient-wise, lazy product);
  // normalized() (guarded by squaredNorm > 0); * norm; store; free(lxly).
  // The norm is computed before the product/normalize: right-to-left evaluation of
  // operator*(normalized-expr, scalar).
  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override
  {
    Eigen::Vector3d g(x[0], x[1], x[2]);
    Eigen::MatrixXd lxly(3, 2);
    lxly = TangentBasis(g);
    Eigen::Vector2d dg(delta[0], delta[1]);
    Eigen::Map<Eigen::Vector3d> g_plus(x_plus_delta);
    g_plus = (g + lxly * dg).normalized() * g.norm();
    return true;
  }

  // 0x18001A620 (outside this chunk; folded). Reconstructed from its pseudocode for completeness:
  // MatrixXd lxly(3,2); lxly = TangentBasis(g); then the 3x2 is written row-major into jacobian
  // (DenseBase::resize assert "rows == this->rows() && cols == this->cols()" on the assignment).
  bool ComputeJacobian(const double* x, double* jacobian) const override
  {
    Eigen::Vector3d g(x[0], x[1], x[2]);
    Eigen::MatrixXd lxly(3, 2);
    lxly = TangentBasis(g);
    Eigen::Map<Eigen::Matrix<double, 3, 2, Eigen::RowMajor>> j(jacobian);
    j = lxly;  // TODO(verify) exact statement (owner: chunk containing 0x18001A620)
    return true;
  }

  int GlobalSize() const override { return 3; }  // 0x18001A910
  int LocalSize() const override { return 2; }   // 0x180014A10

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace totem
}  // namespace pimax
