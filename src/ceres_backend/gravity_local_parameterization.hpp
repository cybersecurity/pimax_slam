// pimax_slam.pi.dll -- src/ceres_backend/gravity_local_parameterization.hpp
//
// [pimax-new] Local parameterization of the gravity vector g_W: global 3, local 2.  The manifold is
// the 2-sphere of radius |g|; g is perturbed in the tangent plane spanned by TangentBasis(g)
// (VINS-Mono style) and re-projected to the original norm.
// All members are header-inline; their out-of-line copies were emitted in ceres_map.obj
// (notes c01 inline_bodies, c02 "GravityLocalParameterization statics").
//
// Layout (24 bytes, Map+0x538): +0 vptr ceres::LocalParameterization (vtable 0x1803ADFB8),
// +8 vptr LocalParamizationAdditionalInterfaces (0x1803ADFF0), +0x10 bool use_minimal_jacobians_.
// The deleting dtor is 0x18001A550 (ICF with HomogeneousPointLocalParameterization, uses free())
// -> EIGEN_MAKE_ALIGNED_OPERATOR_NEW.
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres/local_parameterization.h>

#include "ceres_backend/local_parameterization_additional_interfaces.hpp"

namespace pimax {
namespace totem {

/// 0x18001ACE0 (out-of-line inline, first emitted in ceres_map.obj).  VINS-Mono
/// InitialAlignment TangentBasis, verbatim: returns a dynamic 3x2 matrix [b c].
/// VINS takes `Vector3d&`; a const reference is used here so that the Map<const Vector3d>
/// callers bind through a temporary.  TODO(verify) exact name/scope/parameter.
inline Eigen::MatrixXd TangentBasis(const Eigen::Vector3d& g0)
{
  Eigen::Vector3d b, c;
  Eigen::Vector3d a = g0.normalized();
  Eigen::Vector3d tmp(0, 0, 1);
  if (a == tmp)
    tmp << 1, 0, 0;
  b = (tmp - a * (a.transpose() * tmp)).normalized();
  c = a.cross(b);
  Eigen::MatrixXd bc(3, 2);
  bc.block<3, 1>(0, 0) = b;
  bc.block<3, 1>(0, 1) = c;
  return bc;
}
// (namespace pimax::totem, not ceres_backend: the frontend's InitGravityLocalParameter
//  (frontend/pose_local_parameterization.h) calls the same COMDAT.)

namespace ceres_backend {

class GravityLocalParameterization : public ::ceres::LocalParameterization,
                                     public LocalParamizationAdditionalInterfaces
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ~GravityLocalParameterization() override = default;   // 0x18001A550 (ICF)

  // 0x18001A960 (also GravityParameterBlock::plus, slot 5)
  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override
  {
    return plus(x, delta, x_plus_delta);
  }

  // 0x18001A930 (also GravityParameterBlock::minus, slot 7)
  bool Minus(const double* x, const double* x_plus_delta, double* delta) const override
  {
    return minus(x, x_plus_delta, delta);
  }

  // 0x18001A620
  bool ComputeJacobian(const double* x, double* jacobian) const override
  {
    return plusJacobian(x, jacobian);
  }

  // 0x18001A780
  bool ComputeLiftJacobian(const double* x, double* jacobian) const override
  {
    return liftJacobian(x, jacobian);
  }

  // 0x18001A8E0 (Pimax: 0 while the flag is set)
  int GlobalSize() const override { return use_minimal_jacobians_ ? 0 : 3; }
  // 0x180014A10 (ICF "return 2", shared with boost codecvt_null<wchar_t>::do_encoding)
  int LocalSize() const override { return 2; }

  // 0x18001D5D0
  // x_plus_delta = |x| * normalize(x + B(x) * delta)
  static bool plus(const double* x, const double* delta, double* x_plus_delta)
  {
    Eigen::Map<const Eigen::Vector3d> g(x);
    Eigen::Map<const Eigen::Vector2d> dg(delta);
    Eigen::Map<Eigen::Vector3d> g_plus(x_plus_delta);

    Eigen::MatrixXd B(3, 2);          // DenseStorage resize: free(0) + aligned_malloc(6*8)
    B = TangentBasis(g);              // move-assign (swap) + free of the 3x2 temp
    const double norm = g.norm();
    // Eigen evaluates `g + B * dg` as tmp = g; tmp += B * dg (lazy coeff product).
    g_plus = (g + B * dg).normalized() * norm;   // normalized(): divide only if squaredNorm > 0
    return true;
  }

  // 0x18001CF70
  // delta = B(x)^T * (x_plus_delta - x)
  static bool minus(const double* x, const double* x_plus_delta, double* delta)
  {
    Eigen::Map<const Eigen::Vector3d> g(x);
    Eigen::Map<const Eigen::Vector3d> g_plus(x_plus_delta);
    Eigen::Map<Eigen::Vector2d> dg(delta);

    Eigen::MatrixXd B(3, 2);
    B = TangentBasis(g);
    // product evaluated into an aligned Vector2d temporary, then copied to delta
    dg = B.transpose() * (g_plus - g);
    return true;
  }

  // Inlined into ComputeJacobian (0x18001A620) and GravityParameterBlock::plusJacobian
  // (0x18002B800): identical code sequences (Vector3d copy of x; MatrixXd B(3,2); B = TangentBasis).
  static bool plusJacobian(const double* x, double* jacobian)
  {
    Eigen::Map<Eigen::Matrix<double, 3, 2, Eigen::RowMajor>> J(jacobian);
    Eigen::Vector3d g = Eigen::Map<const Eigen::Vector3d>(x);
    Eigen::MatrixXd basis;
    basis.resize(3, 2);          // the binary does free(nullptr) + malloc(6 doubles) first
    basis = TangentBasis(g);
    J = basis;
    return true;
  }

  // Inlined into ComputeLiftJacobian (0x18001A780) and GravityParameterBlock::liftJacobian
  // (0x18002A560).  Transpose-aliasing assert (Transpose.h:0x1B6) present.
  static bool liftJacobian(const double* x, double* jacobian)
  {
    Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J(jacobian);
    Eigen::Vector3d g = Eigen::Map<const Eigen::Vector3d>(x);
    Eigen::MatrixXd basis;
    basis.resize(3, 2);
    basis = TangentBasis(g);
    J = basis.transpose();
    return true;
  }
};

static_assert(sizeof(GravityLocalParameterization) == 0x18, "");

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
