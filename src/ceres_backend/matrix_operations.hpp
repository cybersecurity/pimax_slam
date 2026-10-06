// pimax_slam.pi.dll -- src/ceres_backend/matrix_operations.hpp
//
// The svo_vio_common helpers used by the ceres backend (upstream svo/vio_common/matrix.hpp and
// svo/vio_common/matrix_operations.hpp).  Only the ceres backend uses them (callers of
// 0x180016490: ReprojectionError 0x18000C490/0x18000E0A0, GroundPlaneError 0x18002D390,
// ImuError 0x18003AE40; of 0x1800455A0/0x1800453F0: ImuError and PoseLocalParameterization).
// The original header path is unknown (no __FILE__); it is kept with the backend.
// TODO(verify) path/name.  Namespace pimax::totem (upstream: svo).
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace pimax {
namespace totem {

// -------------------------------------------------------------------------------------------
// matrix.hpp -- Pimax uses double here (upstream FloatType, which is float in this fork).
inline Eigen::Matrix3d skewSymmetric(const double w1,
                                     const double w2,
                                     const double w3)
{
  return (Eigen::Matrix3d() <<
           0.0, -w3,  w2,
           w3,  0.0, -w1,
          -w2,  w1,  0.0).finished();
}

/// 0x180016490 (COMDAT, first emitted by ceres_backend_interface.obj)
inline Eigen::Matrix3d skewSymmetric(const Eigen::Ref<const Eigen::Vector3d>& w)
{
  return skewSymmetric(w(0), w(1), w(2));
}

// -------------------------------------------------------------------------------------------
// matrix_operations.hpp.  Pimax writes these with the comma initializer (CommaInitializer.h
// asserts, 16 x operator,(const double&) = 0x18003A1F0) instead of upstream's 16 element
// assignments; values identical.

//! Plus matrix for a quaternion. q_AB x q_BC = plus(q_AB) * q_BC.coeffs().
// 0x1800455A0
inline Eigen::Matrix4d quaternionPlusMatrix(const Eigen::Quaterniond& q_AB)
{
  const Eigen::Vector4d& q = q_AB.coeffs();
  Eigen::Matrix4d Q;
  Q <<  q[3], -q[2],  q[1],  q[0],
        q[2],  q[3], -q[0],  q[1],
       -q[1],  q[0],  q[3],  q[2],
       -q[0], -q[1], -q[2],  q[3];
  return Q;
}

//! Opposite-Plus matrix for a quaternion q_AB x q_BC = oplus(q_BC) * q_AB.coeffs().
// 0x1800453F0
inline Eigen::Matrix4d quaternionOplusMatrix(const Eigen::Quaterniond& q_BC)
{
  const Eigen::Vector4d& q = q_BC.coeffs();
  Eigen::Matrix4d Q;
  Q <<  q[3],  q[2], -q[1],  q[0],
       -q[2],  q[3],  q[0],  q[1],
        q[1], -q[0],  q[3],  q[2],
       -q[0], -q[1], -q[2],  q[3];
  return Q;
}

}  // namespace totem
}  // namespace pimax
