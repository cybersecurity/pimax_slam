// pimax_slam.pi.dll -- src/direct/matcher.h
//
// Pimax fork of svo_direct/include/svo/direct/matcher.h.  Reconciled from c07 (layout, tail of
// matcher.cpp), c06 (head of matcher.cpp, inlined ctor in the DepthFilter ctor 0x18009F280:
// defaults, sizeof 352, the extra Vector3f at +332, float depth out-parameters, findLocalMatch)
// and c13 (same defaults from the inlined ctor in StereoTriangulation::compute 0x180147110,
// std::vector<Matcher> stride 352).
//
// sizeof(Matcher) == 352 (0x160): malloc(0x160) + memset (value-initialisation of the implicit
// default ctor) in the DepthFilter ctor, then the default member initialisers below
// (+4 = 10, +8 = 2.0, +16 = 100, +24..+26 = 1, +32 = 0.5, +41/+42 = 1, +48 = 2.5, +320..+343 = 0).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <Eigen/Core>

#include "common/types.h"          // Pimax: FloatType == float (Keypoint/BearingVector/GradientVector are float)
#include "common/camera_fwd.h"
#include "common/transformation.h"

namespace pimax {
namespace totem {

class Point;
class Frame;
struct FeatureWrapper;

namespace patch_score {
template <int HALF_PATCH_SIZE> class ZMSSD;
}

class Matcher
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const int kHalfPatchSize = 4;
  static const int kPatchSize = 8;

  typedef patch_score::ZMSSD<kHalfPatchSize> PatchScore;
  typedef std::shared_ptr<Matcher> Ptr;

  struct Options
  {
    bool align_1d = false;                      // +0
    int align_max_iter = 10;                    // +4   (used: findMatchDirect / findLocalMatch)
    double max_epi_length_optim = 2.0;          // +8
    size_t max_epi_search_steps = 100;          // +16  (no longer read by scanEpipolarUnitPlane)
    bool subpix_refinement = true;              // +24
    bool epi_search_edgelet_filtering = true;   // +25
    bool scan_on_unit_sphere = true;            // +26
    double epi_search_edgelet_max_angle = 0.5;  // +32  Pimax: upstream 0.7
    bool verbose = false;                       // +40
    bool use_affine_warp_ = true;               // +41  (no longer read)
    bool affine_est_offset_ = true;             // +42
    bool affine_est_gain_ = false;              // +43
    double max_patch_diff_ratio = 2.5;          // +48  Pimax: upstream 2.0
  } options_;                                   // 56 bytes

  enum class MatchResult {
    kSuccess,            // 0
    kFailScore,          // 1
    kFailTriangulation,  // 2
    kFailVisibility,     // 3
    kFailWarp,           // 4
    kFailAlignment,      // 5
    kFailRange,          // 6
    kFailAngle,          // 7
    kFailCloseView,      // 8
    kFailLock,           // 9
    kFailTooFar          // 10
  };

  uint8_t patch_[kPatchSize * kPatchSize];                         // +56
  uint8_t patch_with_border_[(kPatchSize + 2) * (kPatchSize + 2)]; // +120
  Eigen::Matrix2d A_cur_ref_;          // +224 affine warp matrix
  Eigen::Vector2d epi_image_;          // +256 vector from epipolar start to end on the image plane
  double epi_length_pyramid_;          // +272 |px_A - px_B| / 2^search_level
  // Pimax: two extra epipolar lengths written by findEpipolarMatchDirect (0x1800AE820) and read
  // by scanEpipolarUnitPlane: |px_A - px_C| / 2^lvl and |px_C - px_B| / 2^lvl, C = projection at
  // the current depth estimate.  Names are ours.  TODO(verify)
  double epi_length_pyramid_ca_;       // +280
  double epi_length_pyramid_cb_;       // +288
  double h_inv_;                       // +296 hessian of 1d image alignment along epipolar line
  int search_level_;                   // +304
  bool reject_;                        // +308
  Keypoint px_cur_;                    // +312 (Vector2f)
  BearingVector f_cur_ = BearingVector::Zero();              // +320 (Vector3f)
  /// [pimax] f_cur_ before normalisation (findEpipolarMatchDirect); read by
  /// StereoTriangulation::triangulatePoint 0x180145470.  c12 took it for a Vector2f "uv".
  /// TODO(verify) name
  BearingVector f_cur_unnormalized_ = BearingVector::Zero();  // +332

  Matcher() = default;
  ~Matcher() = default;

  /// 0x1800AF5E0
  MatchResult findMatchDirect(
      const Frame& ref_frame,
      const Frame& cur_frame,
      const FeatureWrapper& ref_ftr,
      const FloatType& ref_depth,
      Keypoint& px_cur);

  /// 0x1800AE820.  Pimax: the depth out-parameter is float.  The upstream overload without
  /// T_cur_ref is not in the image.
  MatchResult findEpipolarMatchDirect(
      const Frame& ref_frame,
      const Frame& cur_frame,
      const Transformation& T_cur_ref,
      const FeatureWrapper& ref_ftr,
      const double d_estimate_inv,
      const double d_min_inv,
      const double d_max_inv,
      FloatType& depth);

  /// 0x1800AF4B0 (align1D/align2D dispatch; upstream-identical apart from float keypoints)
  MatchResult findLocalMatch(
      const Frame& frame,
      const Eigen::Ref<GradientVector>& direction,
      const int patch_level,
      Keypoint& px_cur);

  /// 0x1800AFD70.  Pimax: upstream scanEpipolarLine / scanEpipolarUnitSphere /
  /// isPatchWithinImage / getResultString are not in the image.
  void scanEpipolarUnitPlane(
      const Frame& frame,
      const Eigen::Vector3d& A,
      const Eigen::Vector3d& B,
      const Eigen::Vector3d& C,
      const PatchScore& patch_score,
      const int patch_level,
      Keypoint* image_best,
      int* zmssd_best);

  /// 0x1800B0270 (upstream-identical)
  bool updateZMSSD(
      const Frame& frame,
      const Eigen::Vector2i& pxi,
      const int patch_level,
      const PatchScore& patch_score,
      int* zmssd_best);

  static void layout_check();
};

namespace matcher_utils {

/// 0x1800AE610 (+ lambda 0x1800AE150).  Pimax: closed form with the upstream least-squares solve
/// as fallback; float depth.
Matcher::MatchResult depthFromTriangulation(
    const Transformation& T_search_ref,
    const Eigen::Vector3d& f_ref,
    const Eigen::Vector3d& f_cur,
    FloatType* depth);

} // namespace matcher_utils

inline void Matcher::layout_check()
{
  static_assert(sizeof(Matcher::Options) == 56, "sizeof(Matcher::Options)");
  static_assert(offsetof(Matcher::Options, align_max_iter) == 4, "Options::align_max_iter");
  static_assert(offsetof(Matcher::Options, epi_search_edgelet_max_angle) == 32, "Options::epi_search_edgelet_max_angle");
  static_assert(offsetof(Matcher::Options, affine_est_offset_) == 42, "Options::affine_est_offset_");
  static_assert(offsetof(Matcher::Options, max_patch_diff_ratio) == 48, "Options::max_patch_diff_ratio");
  static_assert(sizeof(Matcher) == 352, "sizeof(Matcher)");
  static_assert(offsetof(Matcher, patch_) == 56, "Matcher::patch_");
  static_assert(offsetof(Matcher, patch_with_border_) == 120, "Matcher::patch_with_border_");
  static_assert(offsetof(Matcher, A_cur_ref_) == 224, "Matcher::A_cur_ref_");
  static_assert(offsetof(Matcher, epi_image_) == 256, "Matcher::epi_image_");
  static_assert(offsetof(Matcher, epi_length_pyramid_) == 272, "Matcher::epi_length_pyramid_");
  static_assert(offsetof(Matcher, epi_length_pyramid_ca_) == 280, "Matcher::epi_length_pyramid_ca_");
  static_assert(offsetof(Matcher, epi_length_pyramid_cb_) == 288, "Matcher::epi_length_pyramid_cb_");
  static_assert(offsetof(Matcher, h_inv_) == 296, "Matcher::h_inv_");
  static_assert(offsetof(Matcher, search_level_) == 304, "Matcher::search_level_");
  static_assert(offsetof(Matcher, reject_) == 308, "Matcher::reject_");
  static_assert(offsetof(Matcher, px_cur_) == 312, "Matcher::px_cur_");
  static_assert(offsetof(Matcher, f_cur_) == 320, "Matcher::f_cur_");
  static_assert(offsetof(Matcher, f_cur_unnormalized_) == 332, "Matcher::f_cur_unnormalized_");
}

} // namespace totem
} // namespace pimax
