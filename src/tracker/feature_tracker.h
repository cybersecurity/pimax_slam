// pimax_slam.pi.dll -- src/tracker/feature_tracker.h (TODO(verify) path; the object is linked after sensor_fusion
// and before the third-party `fast` objects, consistent with src/tracker/feature_tracker.cpp)
// -- rpg_svo_pro_open svo_tracker/include/svo/tracker/feature_tracker.h, upstream-identical.
// Only the ctor, reset() and resetTerminatedTracks() survived /OPT:REF (used by
// AbstractInitialization, 0x18012B320 / 0x18012C030).
//
// Layout (sizeof == 152):
//   +0   FeatureTrackerOptions options_   (72 bytes, copied member-wise)
//   +72  const size_t bundle_size_
//   +80  std::vector<std::shared_ptr<AbstractDetector>> detectors_
//   +104 std::vector<FeatureTracks, aligned_allocator> active_tracks_
//   +128 std::vector<FeatureTracks, aligned_allocator> terminated_tracks_
#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

// From other chunks / upstream svo headers:
//   svo::FeatureTrackerOptions (svo/tracker/feature_tracking_types.h, upstream layout:
//     int klt_max_level +0, int klt_min_level +4, std::vector<int> klt_patch_sizes +8,
//     int klt_max_iter +32, double klt_min_update_squared +40,
//     bool klt_template_is_first_observation +48, size_t min_tracks_to_detect_new_features +56,
//     bool reset_before_detection +64),
//   FeatureTrack (32 bytes: int track_id_; std::vector<FeatureRef> feature_track_),
//   FeatureTracks = std::vector<FeatureTrack, Eigen::aligned_allocator<FeatureTrack>>,
//   AbstractDetector (grid_ at +88, closeness_check_grid_ at +160; resetGrid() inline),
//   DetectorOptions, CameraBundlePtr (= std::shared_ptr<vk::cameras::NCamera>),
//   feature_detection_utils::makeDetector (0x1800AD840).
#include "common/camera.h"
#include "direct/feature_detection.h"
#include "direct/feature_detection_types.h"
#include "tracker/feature_tracking_types.h"

namespace pimax {
namespace totem {

class FeatureTracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<FeatureTracker> Ptr;

  FeatureTracker() = delete;

  // 0x1801A7DF0
  FeatureTracker(const FeatureTrackerOptions& options, const DetectorOptions& detector_options,
                 const CameraBundlePtr& cams);

  void resetActiveTracks();      // inlined into reset()
  void resetTerminatedTracks();  // 0x1801A80F0
  void reset();                  // 0x1801A8030

  FeatureTrackerOptions options_;
  const size_t bundle_size_;
  std::vector<std::shared_ptr<AbstractDetector>> detectors_;
  std::vector<FeatureTracks, Eigen::aligned_allocator<FeatureTracks>> active_tracks_;
  std::vector<FeatureTracks, Eigen::aligned_allocator<FeatureTracks>> terminated_tracks_;

  static void layout_check();
};

inline void FeatureTracker::layout_check()
{
  static_assert(sizeof(FeatureTracker) == 152, "sizeof(FeatureTracker)");
  static_assert(offsetof(FeatureTracker, bundle_size_) == 72, "FeatureTracker::bundle_size_");
  static_assert(offsetof(FeatureTracker, detectors_) == 80, "FeatureTracker::detectors_");
  static_assert(offsetof(FeatureTracker, active_tracks_) == 104, "FeatureTracker::active_tracks_");
  static_assert(offsetof(FeatureTracker, terminated_tracks_) == 128, "FeatureTracker::terminated_tracks_");
}

}  // namespace totem
}  // namespace pimax
