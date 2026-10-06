// pimax_slam.pi.dll -- src/tracker/feature_tracker.cpp (TODO(verify) path) -- rpg_svo_pro_open
// svo_tracker/src/feature_tracker.cpp, upstream-identical for the three surviving functions.
// Library instantiations emitted here:
//   0x1801A7AD0 std::vector<std::shared_ptr<AbstractDetector>>::_Emplace_reallocate (push_back(&&))
//   0x1801A7D00 std::vector<FeatureTracks, Eigen::aligned_allocator<...>>::vector(size_t)
#include "tracker/feature_tracker.h"

#include "direct/feature_detection_utils.h"

namespace pimax {
namespace totem {

// 0x1801A7DF0
FeatureTracker::FeatureTracker(const FeatureTrackerOptions& options,
                               const DetectorOptions& detector_options,
                               const CameraBundlePtr& cams)
    : options_(options),
      bundle_size_(cams->getNumCameras()),
      active_tracks_(bundle_size_),
      terminated_tracks_(bundle_size_) {
  for (size_t i = 0; i < cams->getNumCameras(); ++i) {
    detectors_.push_back(feature_detection_utils::makeDetector(  // 0x1800AD840
        detector_options, cams->getCameraShared(i)));             // 0x1801B41A0
  }
}

// inlined into reset()
void FeatureTracker::resetActiveTracks() {
  for (auto& track : active_tracks_) track.clear();
}

// 0x1801A80F0
void FeatureTracker::resetTerminatedTracks() {
  for (auto& track : terminated_tracks_) track.clear();
}

// 0x1801A8030
void FeatureTracker::reset() {
  resetActiveTracks();
  resetTerminatedTracks();
  for (auto& detector : detectors_) detector->resetGrid();  // OccupandyGrid2D::reset 0x1800AAB80 x2
}

}  // namespace totem
}  // namespace pimax
