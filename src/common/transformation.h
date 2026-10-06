// pimax_slam.pi.dll -- src/common/transformation.h
//
// Pimax fork of svo_common/include/svo/common/transformation.h.
//
//  * Transformation is minkindr's QuatTransformationTemplate<double> (Pimax-patched copy in
//    third_party/minkindr: operator* and inverse() renormalise their result, see
//    third_party/minkindr/PIMAX_PATCH.md).  sizeof == 64 (Quaterniond @+0 xyzw, Vector3d @+32).
//  * Quaternion is Eigen::Quaterniond in this fork (upstream: kindr RotationQuaternion).
//    Evidence: members typed `Quaternion` upstream are left uninitialised by their owners' ctors
//    (Frame +0xC0, AbstractInitialization +176/+208), quaternion inversion divides by the squared
//    norm (Eigen) instead of conjugating (kindr) in setRotationIncrementPrior/optimizePose
//    (c10 notes), and `Quaternion(C_imu_world)` in ImuProcessor::getInitialAttitude has no
//    isValidRotationMatrix CHECK (quaternion_assign_impl 0x180020AF0 only).
//    TODO(verify): a Pimax wrapper class with the same behaviour would be indistinguishable.
//  * quaternionExp(): the rotation-vector -> quaternion map inlined (identically) into
//    PoseLocalParameterization::plus 0x18008D270, FrameProcessorBase::getMotionPrior 0x18010A690
//    and ImuProcessor::getRelativeRotationPrior 0x18012AB60: the minkindr/Grassia formula with a
//    FIXED small-angle threshold of 1e-12 (0x1803AF290) and no unit-norm CHECK.  minkindr's own
//    RotationQuaternion::exp (threshold eps^(1/4), CHECK_NEAR in the (w,x,y,z) ctor) is still
//    used unchanged by Transformation::exp in PoseOptimizer::update 0x18013F830 (0x18012FE00,
//    0x180132B70).  TODO(verify) real name/home of this helper.
#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <kindr/minimal/quat-transformation.h>

namespace pimax {
namespace totem {

using Transformation = kindr::minimal::QuatTransformationTemplate<double>;
using TransformationF = kindr::minimal::QuatTransformationTemplate<float>;
using Quaternion = Eigen::Quaterniond;

/// Rotation vector -> unit quaternion (Grassia), threshold 1e-12.  Result is NOT normalised and
/// NOT checked; every caller renormalises after the product.
inline Eigen::Quaterniond quaternionExp(const Eigen::Vector3d& dx)
{
  const double theta = dx.norm();
  double na;
  if (theta < 1e-12)
  {
    static const double one_over_48 = 1.0 / 48.0;   // 0.02083333333333333
    na = 0.5 + (theta * theta) * one_over_48;
  }
  else
  {
    na = std::sin(theta * 0.5) / theta;
  }
  const double ct = std::cos(theta * 0.5);
  return Eigen::Quaterniond(ct, dx[0] * na, dx[1] * na, dx[2] * na);
}

} // namespace totem
} // namespace pimax
