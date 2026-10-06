// pimax_slam.pi.dll -- src/direct/patch_warp.h
// Pimax fork of svo_direct/include/svo/direct/patch_warp.h (chunk c07)
// Only getWarpMatrixAffine / getBestSearchLevel / warpAffine exist in the binary
// (patch_warp.cpp object = [0x1800B0450, 0x1800B1480)).  getWarpMatrixAffineHomography,
// warpPixelwise, createPatchNoWarp, createPatchNoWarpInterpolated were removed (or never
// referenced and dropped by /OPT:REF -- they would have external linkage, so "removed" is the
// likelier explanation; TODO(verify)).
#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include "common/types.h"
#include "common/camera_fwd.h"
#include "common/transformation.h"

namespace pimax {
namespace totem {
namespace warp {

using AffineTransformation2 = Eigen::Matrix2d;

// 0x1800B0600
void getWarpMatrixAffine(
    const CameraPtr& cam_ref,
    const CameraPtr& cam_cur,
    const Eigen::Ref<Keypoint>& px_ref,
    const Eigen::Ref<BearingVector>& f_ref,
    const double depth_ref,
    const Transformation& T_cur_ref,
    const int level_ref,
    AffineTransformation2* A_cur_ref);

// 0x1800B05B0
int getBestSearchLevel(
    const AffineTransformation2& A_cur_ref,
    const int max_level);

// 0x1800B0C10
bool warpAffine(
    const AffineTransformation2& A_cur_ref,
    const cv::Mat& img_ref,
    const Eigen::Ref<Keypoint>& px_ref,
    const int level_ref,
    const int search_level,
    const int halfpatch_size,
    uint8_t* patch);

} // namespace warp
} // namespace totem
} // namespace pimax
