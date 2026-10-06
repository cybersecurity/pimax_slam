// pimax_slam.pi.dll -- src/direct/feature_alignment.h
// Pimax fork of svo_direct/include/svo/direct/feature_alignment.h (chunk c06; object range
// 0x1800A3900 .. 0x1800AA150).
// Only align1D (0x1800A63E0) and align2D (0x1800A84F0) exist in the image; align2D_SSE2,
// align2D_NEON, alignPyr2D and alignPyr2DVec are absent (unreferenced or deleted).
#pragma once

#include <vector>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include "common/types.h"   // FloatType == float: Keypoint == Vector2f, GradientVector == Vector2f

namespace pimax {
namespace totem {
namespace feature_alignment {

/// Pimax: completely rewritten (IRLS with Gaussian spatial weights, Sobel gradients, residual
/// clamping, SVD pseudo-inverse fallback, step halving, chi2-based termination, patch-mean
/// sanity check).  Signature unchanged.
bool align1D(
    const cv::Mat& cur_img,
    const Eigen::Ref<GradientVector>& dir,                  // direction in which the patch is allowed to move
    uint8_t* ref_patch_with_border,
    uint8_t* ref_patch,
    const int n_iter,
    const bool affine_est_offset,
    const bool affine_est_gain,
    Keypoint* cur_px_estimate,
    double* h_inv = nullptr);

/// Pimax: float literals; Hinv replaced by an LDLT solve.
bool align2D(
    const cv::Mat& cur_img,
    uint8_t* ref_patch_with_border,
    uint8_t* ref_patch,
    const int n_iter,
    const bool affine_est_offset,
    const bool affine_est_gain,
    Keypoint& cur_px_estimate,
    bool no_simd = false,
    std::vector<Eigen::Vector2f>* each_step=nullptr);

} // namespace feature_alignment
} // namespace totem
} // namespace pimax
