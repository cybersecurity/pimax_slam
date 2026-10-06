// pimax_slam.pi.dll -- src/ceres_backend/homogeneous_point_local_parameterization.hpp
//
// Upstream behaviour (svo_ceres_backend homogeneous_point_local_parameterization.{hpp,cpp}), but
// Pimax has no homogeneous_point_local_parameterization.obj: every virtual is emitted in
// estimator.obj / ceres_map.obj, so the bodies are header-inline here (notes c03 §1b).
// An instance lives in ceres_backend::Map at +0x508.  Landmarks are General3DParameterBlocks with
// the Trivial parameterization, so this class is only instantiated, never used.
//
// vtable 0x1803ADEF8 (ceres::LocalParameterization part):
//   [0] 0x18001A550 dtor   [1] 0x18002DFC0 Plus   [2] 0x18002DF00 ComputeJacobian
//   [3] 0x1801B9350 ceres::LocalParameterization::MultiplyByJacobian (library default)
//   [4] 0x18001A8F0 GlobalSize (flag ? 0 : 4)   [5] 0x18001A910 LocalSize (3, ICF)
// vtable 0x1803ADF30 (LocalParamizationAdditionalInterfaces part):
//   [0] 0x18001A538 dtor thunk   [1] 0x18002DF90 Minus   [2] 0x18002DF40 ComputeLiftJacobian
//   [3] 0x180052D90 verify
// EIGEN_MAKE_ALIGNED_OPERATOR_NEW: the deleting dtor 0x18001A550 uses free() (c01).
#pragma once

#include <Eigen/Core>
#include <ceres/local_parameterization.h>

#include "ceres_backend/local_parameterization_additional_interfaces.hpp"

namespace pimax {
namespace totem {
namespace ceres_backend {

/// \brief Local parameterisation of a homogeneous point [x,y,z,w]^T.
class HomogeneousPointLocalParameterization :
    public ::ceres::LocalParameterization,
    public LocalParamizationAdditionalInterfaces
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual ~HomogeneousPointLocalParameterization() = default;

  // 0x18002DFC0 (static plus() inlined)
  virtual bool Plus(const double* x, const double* delta,
                    double* x_plus_delta) const
  {
    return plus(x, delta, x_plus_delta);
  }

  // 0x18002DF90 (static minus() inlined)
  virtual bool Minus(const double* x, const double* x_plus_delta,
                     double* delta) const
  {
    return minus(x, x_plus_delta, delta);
  }

  // 0x18002DF00
  virtual bool ComputeJacobian(const double* x, double* jacobian) const
  {
    return plusJacobian(x, jacobian);
  }

  // 0x18002DF40
  virtual bool ComputeLiftJacobian(const double* x, double* jacobian) const
  {
    return liftJacobian(x, jacobian);
  }

  static bool plus(const double* x, const double* delta, double* x_plus_delta)
  {
    Eigen::Map<const Eigen::Vector3d> delta_(delta);
    Eigen::Map<const Eigen::Vector4d> x_(x);
    Eigen::Map<Eigen::Vector4d> x_plus_delta_(x_plus_delta);

    x_plus_delta_ = x_ + Eigen::Vector4d(delta_[0], delta_[1], delta_[2], 0);

    return true;
  }

  static bool plusJacobian(const double*, double* jacobian)
  {
    Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor> > Jp(jacobian);

    Jp.setZero();
    Jp.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();

    return true;
  }

  static bool minus(const double* x, const double* x_plus_delta, double* delta)
  {
    Eigen::Map<Eigen::Vector3d> delta_(delta);
    Eigen::Map<const Eigen::Vector4d> x_(x);
    Eigen::Map<const Eigen::Vector4d> x_plus_delta_(x_plus_delta);

    // DEBUG_CHECK(fabs((x_plus_delta_-x_)[3])<1e-12) compiled out
    delta_ = (x_plus_delta_ - x_).head<3>();

    return true;
  }

  static bool liftJacobian(const double*, double* jacobian)
  {
    Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor> > Jp(jacobian);

    Jp.setZero();
    Jp.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();

    return true;
  }

  /// \brief The parameter block dimension.  0x18001A8F0 (Pimax: 0 while the flag is set)
  virtual int GlobalSize() const
  {
    return use_minimal_jacobians_ ? 0 : 4;
  }

  /// \brief The parameter block local dimension.  0x18001A910 (ICF "return 3")
  virtual int LocalSize() const
  {
    return 3;
  }
};

static_assert(sizeof(HomogeneousPointLocalParameterization) == 0x18, "");

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
