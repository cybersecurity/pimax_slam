// Reconstructed from pimax_slam.pi.dll (chunk c18_tail_vikit), object camera_geometry_base.obj
// (0x1801B3C00 .. 0x1801B3F38). Upstream: rpg_svo_pro_open/vikit/vikit_cameras/src/camera_geometry_base.cpp
#include "vikit/cameras/camera_geometry_base.h"

#include <string>
#include <utility>

#include <glog/logging.h>

namespace vk {
namespace cameras {

// 0x1801B3C00  -- upstream-identical
// (label_ default-constructed, camera_type_ left uninitialised, mask_ = cv::Mat())
CameraGeometryBase::CameraGeometryBase(const int width, const int height)
  : width_(width)
  , height_(height)
{}

// 0x1801B3C50  CameraGeometryBase::`scalar deleting destructor'  -- upstream-identical
// (= default; destroys mask_ then label_)

// 0x1801B3CE0  -- upstream-modified: both CHECK_NOTNULL() calls removed
// (no "'out_bearing_vectors' Must be non NULL" / "'success' Must be non NULL" strings in the image).
// Calls the single-keypoint overload through the vtable (slot 2, +0x10).
void CameraGeometryBase::backProject3(
      const Eigen::Ref<const Eigen::Matrix2Xd>& keypoints,
      Eigen::Matrix3Xd* out_bearing_vectors, std::vector<bool>* success) const
{
  const int num_keypoints = keypoints.cols();
  out_bearing_vectors->resize(Eigen::NoChange, num_keypoints);
  success->resize(num_keypoints);

  for (int i = 0; i < num_keypoints; ++i) {
    Eigen::Vector3d bearing_vector;
    (*success)[i] = backProject3(keypoints.col(i), &bearing_vector);
    out_bearing_vectors->col(i) = bearing_vector;
  }
}

// 0x1801B3F30  -- upstream-modified: CHECK_EQ(height_, mask.rows), CHECK_EQ(width_, mask.cols),
// CHECK_EQ(mask.type(), CV_8UC1) removed; body is a plain cv::Mat copy-assignment.
// Only caller: calibration loader 0x18015E250 (sets a 640x480 CV_8U mask after drawing circles).
void CameraGeometryBase::setMask(const cv::Mat& mask) {
  mask_ = mask;
}

} // namespace cameras
} // namespace vk
