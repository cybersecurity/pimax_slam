// pimax_slam.pi.dll -- src/ceres_backend/pose_local_parameterization.hpp
//
// Fork of svo_ceres_backend/include/svo/ceres_backend/pose_local_parameterization.hpp.
// Bodies in pose_local_parameterization.cpp (draft c05; object 0x18008C980..0x18008D960).
// Pimax: minus/plusJacobian/liftJacobian normalise their quaternions (see c05 notes).
//
// Layout (24 bytes, member of Map at +0x520):
//   +0x00 vptr ceres::LocalParameterization              (vtable 0x1803ADF58)
//   +0x08 vptr LocalParamizationAdditionalInterfaces     (vtable 0x1803ADF90)
//   +0x10 bool LocalParamizationAdditionalInterfaces::use_minimal_jacobians_ (pimax-new)
// vtable 0x1803ADF58: 0 dtor 0x18001A5E0 (plain sized operator delete -> no aligned new, as
//   upstream), 1 Plus 0x18008CC40, 2 ComputeJacobian 0x18008C980, 3 MultiplyByJacobian (ceres
//   default 0x1801B9350), 4 GlobalSize 0x18001A900 (flag ? 0 : 7), 5 LocalSize 0x18001A920 (6)
// vtable 0x1803ADF90: 0 dtor thunk 0x18001A544, 1 Minus 0x18008CC20,
//   2 ComputeLiftJacobian 0x18008C990, 3 verify 0x180052D90
#pragma once

#include <ceres/local_parameterization.h>

#include "ceres_backend/local_parameterization_additional_interfaces.hpp"

namespace pimax {
namespace totem {
namespace ceres_backend {

/// \brief Pose local parameterisation, i.e. for orientation dq(dalpha) x q_bar.
class PoseLocalParameterization : public ::ceres::LocalParameterization,
    public LocalParamizationAdditionalInterfaces
{
 public:
  /// \brief Trivial destructor.
  virtual ~PoseLocalParameterization() = default;

  /// 0x18008CC40 (identical code is also PoseParameterBlock::plus, slot 5)
  virtual bool Plus(const double* x, const double* delta,
                    double* x_plus_delta) const;

  /// 0x18008CC20 (also PoseParameterBlock::minus, slot 7)
  virtual bool Minus(const double* x, const double* x_plus_delta,
                     double* delta) const;

  /// 0x18008C980 (also PoseParameterBlock::plusJacobian, slot 6)
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;

  /// 0x18008C990 (liftJacobian() inlined)
  virtual bool ComputeLiftJacobian(const double* x, double* jacobian) const;

  // provide these as static for easy use elsewhere:
  /// 0x18008D270
  static bool plus(const double* x, const double* delta, double* x_plus_delta);
  /// 0x18008D570
  static bool plusJacobian(const double* x, double* jacobian);
  /// 0x18008CEE0
  static bool minus(const double* x, const double* x_plus_delta, double* delta);
  /// 0x18008CC60 (also PoseParameterBlock::liftJacobian through 0x18008DAF0;
  /// called directly by ReprojectionError / GroundPlaneError)
  static bool liftJacobian(const double* x, double* jacobian);

  /// \brief The parameter block dimension.  0x18001A900 (Pimax: 0 while the flag is set)
  virtual int GlobalSize() const
  {
    return use_minimal_jacobians_ ? 0 : 7;
  }

  /// \brief The parameter block local dimension.  0x18001A920 (ICF "return 6")
  virtual int LocalSize() const
  {
    return 6;
  }
};

static_assert(sizeof(PoseLocalParameterization) == 0x18, "");

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
