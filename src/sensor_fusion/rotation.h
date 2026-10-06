// src/sensor_fusion/rotation.h (TODO(verify) path) -- identical to LedObjectPoseEstimator
// ctrl_three_dof/rotation.h: port of Cardboard SDK util/rotation.{h,cc} (only the parts used).
// Quaternion stored as (x, y, z, w) in a Vector4.
#pragma once

#include <cmath>
#include <limits>
#include "sensor_fusion/vector.h"

namespace pimax {
namespace ThreeDof {

class Rotation {
 public:
  typedef Vector<4> QuaternionType;
  typedef Vector<3> VectorType;

  Rotation() : quat_(0.0, 0.0, 0.0, 1.0) {}

  static Rotation FromQuaternion(const QuaternionType& quat) {
    Rotation r;
    r.quat_ = Normalized(quat);
    return r;
  }

  // 0x1801A5B20
  static Rotation RotateInto(const VectorType& from, const VectorType& to);

  // 0x1801A5A10
  void GetAxisAndAngle(VectorType* axis, double* angle) const;

 private:
  QuaternionType quat_;
};

// 0x1801A5B20
inline Rotation Rotation::RotateInto(const VectorType& from, const VectorType& to) {
  // 2.220446049250313e-14 (0x1803BF510)
  static const double kTolerance = std::numeric_limits<double>::epsilon() * 100;

  const double norm_u_norm_v = std::sqrt(Dot(from, from) * Dot(to, to));
  double real_part = norm_u_norm_v + Dot(from, to);
  VectorType w;
  if (real_part < kTolerance * norm_u_norm_v) {
    real_part = 0.0;
    w = (std::abs(from[0]) > std::abs(from[2]))
            ? VectorType(-from[1], from[0], 0)
            : VectorType(0, -from[2], from[1]);
  } else {
    w = Cross(from, to);
  }
  return FromQuaternion(QuaternionType(w[0], w[1], w[2], real_part));
}

// 0x1801A5A10
inline void Rotation::GetAxisAndAngle(VectorType* axis, double* angle) const {
  VectorType vec(quat_[0], quat_[1], quat_[2]);
  if (Normalize(&vec) != 0.0) {
    *angle = 2.0 * std::acos(quat_[3]);
    *axis = vec;
  } else {
    *axis = VectorType(1.0, 0.0, 0.0);
    *angle = 0.0;
  }
}

}  // namespace ThreeDof
}  // namespace pimax
