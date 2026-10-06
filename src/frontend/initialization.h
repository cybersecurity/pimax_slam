// pimax_slam.pi.dll -- src/frontend/initialization.h  (from draft c11; factory values c14)
//
// pimax::totem::AbstractInitialization / StereoInit.
// Fork of svo/include/svo/initialization.h.
// Only StereoInit survives in the binary (makeInitializer has a single case).
#pragma once

#include <memory>
#include <vector>

#include <cstddef>
#include <utility>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "common/transformation.h"
#include "common/types.h"
#include "frontend/stereo_triangulation.h"

namespace pimax {
namespace totem {

class FeatureTracker;               // tracker/feature_tracker.h (A1)
struct FeatureTrackerOptions;       // tracker/feature_tracker.h (A1)
struct DetectorOptions;             // direct/feature_detection_types.h (A1)
using FeatureTrackerUniquePtr = std::unique_ptr<FeatureTracker>;

// TODO(verify): enumerator list. The binary only proves kStereo == 0 (makeInitializer FATALs on
// any non-zero init_type). Upstream had kHomography first.
enum class InitializerType {
  kStereo = 0,
};

// 64 bytes (copied as 4 x 16 bytes into AbstractInitialization+16).
// Upstream fields minus expected_avg_depth / init_min_depth_error (both removed).
// Factory values (loadInitializationOptions 0x18015DB40): {0, 30.0, 0.5, 45, 2.0, 50, 70, 2.0};
// the defaults below are upstream's (not observable in the binary).
struct InitializationOptions {
  InitializerType init_type = InitializerType::kStereo;  // +0
  double init_min_disparity = 50.0;                      // +8
  double init_disparity_pivot_ratio = 0.5;               // +16
  size_t init_min_features = 100;                        // +24  (read by StereoInit::addFrameBundle)
  double init_min_features_factor = 2.5;                 // +32
  size_t init_min_tracked = 50;                          // +40
  size_t init_min_inliers = 40;                          // +48
  double reproj_error_thresh = 2.0;                      // +56
};

enum class InitResult { kFailure, kNoKeyframe, kTracking, kSuccess };

/// Bootstrapping the map from the first two views.
/// sizeof 280 (StereoInit: 320). MSVC pads the vfptr to 16 because of the Eigen members, so
/// options_ starts at +16.
class AbstractInitialization {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BearingVectors = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;
  using Ptr = std::shared_ptr<AbstractInitialization>;
  using UniquePtr = std::unique_ptr<AbstractInitialization>;
  using FeatureMatches = std::vector<std::pair<size_t, size_t>>;

  InitializationOptions options_;        // +16
  FeatureTrackerUniquePtr tracker_;      // +80
  FrameBundlePtr frames_ref_;            // +88
  Transformation T_cur_from_ref_;        // +112 (identity from kindr default ctor)
  Quaternion R_ref_world_;               // +176 (Eigen, left uninitialised)
  Quaternion R_cur_world_;               // +208 (Eigen, left uninitialised)
  Eigen::Vector3d t_ref_cur_;            // +240
  bool have_rotation_prior_ = false;     // +264
  bool have_translation_prior_ = false;  // +265
  // have_depth_prior_ removed (ctor and reset() only touch the 16-bit word at +264)
  double depth_at_current_frame_ = 1.0;  // +272

  AbstractInitialization(const InitializationOptions& init_options,
                         const FeatureTrackerOptions& tracker_options,
                         const DetectorOptions& detector_options,
                         const CameraBundlePtr& cams);                           // 0x18012B320

  virtual ~AbstractInitialization();                                             // vtbl[0] 0x18012B8E0 (body 0x18012B860)

  virtual InitResult addFrameBundle(const FrameBundlePtr& frames_cur) = 0;       // vtbl[1] _purecall

  virtual void reset();                                                          // vtbl[2] 0x18012C030

  inline void setAbsoluteOrientationPrior(const Quaternion& R_cam_world) {
    R_cur_world_ = R_cam_world;
    have_rotation_prior_ = true;
  }

  inline void setTranslationPrior(const Eigen::Vector3d& t_ref_cur) {
    t_ref_cur_ = t_ref_cur;
    have_translation_prior_ = true;
  }

 private:
  friend struct InitializationLayoutCheck;
};

class StereoInit : public AbstractInitialization {
 public:
  StereoInit(const InitializationOptions& init_options, const FeatureTrackerOptions& tracker_options,
             const DetectorOptions& detector_options, const CameraBundlePtr& cams);   // 0x18012B4E0

  virtual ~StereoInit() = default;                                                // vtbl[0] 0x18012BAB0

  virtual InitResult addFrameBundle(const FrameBundlePtr& frames_cur) override;   // vtbl[1] 0x18012BB60

  std::unique_ptr<StereoTriangulation> stereo_;   // +288
  std::shared_ptr<AbstractDetector> detector_;    // +296
};

static_assert(sizeof(InitializationOptions) == 64, "sizeof(InitializationOptions)");

struct InitializationLayoutCheck
{
  static_assert(sizeof(AbstractInitialization) == 288, "dsize 280, 16-aligned");
  static_assert(offsetof(AbstractInitialization, options_) == 16, "");
  static_assert(offsetof(AbstractInitialization, tracker_) == 80, "");
  static_assert(offsetof(AbstractInitialization, frames_ref_) == 88, "");
  static_assert(offsetof(AbstractInitialization, T_cur_from_ref_) == 112, "");
  static_assert(offsetof(AbstractInitialization, R_ref_world_) == 176, "");
  static_assert(offsetof(AbstractInitialization, R_cur_world_) == 208, "");
  static_assert(offsetof(AbstractInitialization, t_ref_cur_) == 240, "");
  static_assert(offsetof(AbstractInitialization, have_rotation_prior_) == 264, "");
  static_assert(offsetof(AbstractInitialization, depth_at_current_frame_) == 272, "");
  static_assert(sizeof(StereoInit) == 320, "malloc(0x140)");
  static_assert(offsetof(StereoInit, stereo_) == 288, "");
  static_assert(offsetof(StereoInit, detector_) == 296, "");
};

namespace initialization_utils {

AbstractInitialization::UniquePtr makeInitializer(const InitializationOptions& init_options,
                                                  const FeatureTrackerOptions& tracker_options,
                                                  const DetectorOptions& detector_options,
                                                  const CameraBundlePtr& camera_array);   // 0x18012BF30

}  // namespace initialization_utils

}  // namespace totem
}  // namespace pimax
