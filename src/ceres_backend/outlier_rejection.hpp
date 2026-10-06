// pimax_slam.pi.dll -- src/ceres_backend/outlier_rejection.hpp  (from draft c05)
//
// Object 0x18008A080..0x18008AD40 sits between marginalization_error.obj and pose_error.obj.
// Its only project function (0x18008A7C0) is called from the backend interface (0x180013680,
// chunk c01) right before LOGD("Outlier rejection: removed  %lu edgelets and  %lu  corners.\n"),
// i.e. it is the Pimax version of svo::OutlierRejection::removeOutliers().  By link order the
// source is probably src/ceres_backend/outlier_rejection.cpp (alphabetically between
// marginalization_error and pose_error).  TODO(verify) file name / class layout.
//
// Pimax rewrote the function completely: nothing is removed any more (n_deleted_edges,
// n_deleted_corners and deleted_points are never touched, `this` is never read); it only COUNTS
// the features whose landmark reprojects within 2.5 px (level-scaled) of the measured bearing
// and lies 0 < depth <= 10 m in front of the camera.  Upstream `ignore_fixed` was replaced by an
// `int&` output.
#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "common/types.h"          // Frame (fwd)
#include "common/transformation.h" // TransformationF (= QuatTransformationTemplate<float>)

namespace pimax {
namespace totem {

/// z-coordinate of T * p, computed with the mixed float/double expression found in the binary
/// (out-of-line copy 0x18008ACB0 in outlier_rejection.obj, called by
/// FrameProcessorBase::initializeImu 0x180110490; inlined in removeOutliers() and
/// FrameProcessorBase::updateGroundPlane 0x1800F2070).  Float products, double 1.0/2.0 terms,
/// result rounded to float.  A free inline function here (c05 had it as a static member, c09 as a
/// free function).  TODO(verify) name/location.
inline float depthInFrame(const TransformationF& T, const Eigen::Vector3f& p)
{
  const Eigen::Quaternionf& q = T.getRotation().toImplementation();
  return (1.0 - 2.0 * (q.y() * q.y() + q.x() * q.x())) * p.z()
      + 2.0 * ((q.z() * q.y() + q.w() * q.x()) * p.y()
               + (q.z() * q.x() - q.w() * q.y()) * p.x())
      + T.getPosition().z();
}

class OutlierRejection
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// inlined into the CeresBackendInterface ctor 0x180008CD0 (malloc(8) + threshold store)
  explicit OutlierRejection(double reproj_err_threshold)
      : reproj_err_threshold_(reproj_err_threshold)
  {}
  ~OutlierRejection() = default;

  /// 0x18008A7C0  upstream-modified (see file comment).
  void removeOutliers(Frame& frame,
                      size_t& n_deleted_edges,
                      size_t& n_deleted_corners,
                      std::vector<int>& deleted_points,
                      int& n_inliers) const;

 private:
  double reproj_err_threshold_;   // +0  TODO(verify): never read by the Pimax code
};

static_assert(sizeof(OutlierRejection) == 8, "OutlierRejection size (malloc(8) in 0x180008CD0)");

}  // namespace totem
}  // namespace pimax
