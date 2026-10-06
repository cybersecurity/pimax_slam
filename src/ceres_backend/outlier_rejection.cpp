// pimax_slam.pi.dll -- src/ceres_backend/outlier_rejection.cpp  (draft c05)
//
// Object 0x18008A080..0x18008AD40.  Contents:
//   0x18008A080  lib: Eigen::Quaternion<float>(const Matrix3f&) (quaternionbase_assign_substitution)
//   0x18008A270  lib: minkindr QuatTransformationTemplate<double>::cast<float>()
//   0x18008A460  lib: minkindr RotationQuaternionTemplate<double>::cast<float>()
//   0x18008A610  lib: minkindr RotationQuaternionTemplate<float>(const Eigen::Quaternionf&)
//                     (Pimax minkindr copy: EPS-based CHECKs "(squaredNorm()) <= 1+EPS", line 73)
//   0x18008A7C0  project: OutlierRejection::removeOutliers   <-- below
//   0x18008ACB0  project (inline helper, out-of-line copy): depthInFrame (free inline function in
//                outlier_rejection.hpp; the out-of-line copy is emitted where it is not inlined)
#include "ceres_backend/outlier_rejection.hpp"

#include <cmath>

#include "common/frame.h"
#include "common/point.h"

namespace pimax {
namespace totem {

// 0x18008A7C0  upstream-modified (rewritten, see header).
//
// Frame members used (offsets in Frame, see common/frame.h):
//   T_f_w_ (+0x40), num_features_ (+0x228), f_vec_ (+0x240, float 3xN), level_vec_ (+0x270),
//   landmark_vec_ (+0x2A8), track_id_vec_ (+0x2C0); Point::pos_ is an Eigen::Vector3f at +4.
// Frame::w2c (0x180095450) / Frame::f2c (0x1800955C0) project through cam_->project3().
void OutlierRejection::removeOutliers(Frame& frame,
                                      size_t& /*n_deleted_edges*/,
                                      size_t& /*n_deleted_corners*/,
                                      std::vector<int>& /*deleted_points*/,
                                      int& n_inliers) const
{
  const TransformationF T_cam_world = frame.T_cam_world().cast<float>();

  for (size_t i = 0; i < frame.num_features_; ++i)
  {
    if (frame.track_id_vec_(i) > -1)
    {
      const Eigen::Vector3d xyz_world = frame.landmark_vec_[i]->pos_.cast<double>();
      const Eigen::Vector2d px_landmark = frame.w2c(xyz_world);
      const Eigen::Vector2d px_feature =
          frame.f2c(frame.f_vec_.col(static_cast<int>(i)).cast<double>());

      double unwhitened_error = (px_landmark - px_feature).norm();
      unwhitened_error *= 1.0 / (1 << frame.level_vec_(static_cast<int>(i)));

      if (unwhitened_error <= 2.5)
      {
        const float depth = depthInFrame(T_cam_world, frame.landmark_vec_[i]->pos_);
        if (depth > 0 && depth <= 10.0f)
        {
          ++n_inliers;
        }
      }
    }
  }
}

}  // namespace totem
}  // namespace pimax
