// pimax_slam.pi.dll -- src/frontend/frame_processor.h  (from draft c07)
//
// Pimax fork of svo/include/svo/frame_handler_stereo.h
// class pimax::totem::FrameProcessor (vtable 0x1803B2A90), sizeof == 4080 (0xFF0).
// Base: FrameProcessorBase (frame_processor_base.h, sizeof 4032 == 0xFC0; reconciled centrally,
// see notes/c07_direct.md "FrameProcessorBase layout (from its ctor 0x1800DFDF0)").
#pragma once

#include <memory>
#include <vector>
#include <string>

#include "frontend/frame_processor_base.h"
#include "frontend/stereo_triangulation.h"

namespace pimax {
namespace totem {

using StereoTriangulationPtr = std::shared_ptr<StereoTriangulation>;

class FrameProcessor : public FrameProcessorBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<FrameProcessor> Ptr;

  // 0x1800B2460
  FrameProcessor(
      const BaseOptions& base_options,
      const DepthFilterOptions& depth_filter_options,
      const DetectorOptions& feature_detector_options,
      const InitializationOptions& init_options,
      const StereoTriangulationOptions& stereo_options,
      const ReprojectorOptions& reprojector_options,
      const FeatureTrackerOptions& tracker_options,
      const CameraBundlePtr& stereo_camera,
      const std::string& device_sn,         // forwarded to FrameProcessorBase (unused there);
                                            // c14: make_shared<FrameProcessor>(..., device_sn, loc_mode)
      const uint8_t& loc_mode);             // forwarded to FrameProcessorBase (stored at +464)

  // 0x1800B29B0 (scalar deleting dtor; the dtor body is implicit)
  virtual ~FrameProcessor() = default;

  // SVO Modules:
  StereoTriangulationPtr stereo_triangulation_;   // +4032

public:  // TODO(verify) access; SLAMManager/factory only use the base API
  /// Pipeline implementation. Called by base class.
  virtual UpdateResult processFrameBundle() override;     // vtbl slot 2  0x1800B4970

  /// Reset the frame handler.
  virtual void resetAll() override;                        // vtbl slot 3  0x1800B5760

  virtual UpdateResult processFirstFrame();                // vtbl slot 7  0x1800B4180
  virtual UpdateResult processFrame();                     // vtbl slot 8  0x1800B4290
  // Pimax: takes the per-bundle input value FrameProcessorBase+3112 (set by addFrameBundle from
  // its 3rd argument); < 1 enables the "stereo-from-last-keyframe" triangulation.
  virtual UpdateResult makeKeyframe(int input_value);      // vtbl slot 9  0x1800B2F80

  // Pimax-new (0x1800B49A0): reject feature tracks between last_frames_ and new_frames_ that are
  // fundamental-matrix outliers.  Name is ours.
  void removeOutliersByFundamentalMat();

  int forced_kf_count_ = 0;                  // +4048 (++ every time low_match_count_ > 3 in processFrame)
  int low_match_count_ = 0;                  // +4052 (consecutive bundles with < 150 candidates)
  std::vector<FramePtr> last_df_keyframes_;  // +4056 last list returned by DepthFilter (0x18009FC10)

  static void layout_check();
};

inline void FrameProcessor::layout_check()
{
  static_assert(sizeof(FrameProcessor) == 4080, "sizeof(FrameProcessor) (make_shared block 0x1000)");
  static_assert(offsetof(FrameProcessor, stereo_triangulation_) == 4032, "");
  static_assert(offsetof(FrameProcessor, forced_kf_count_) == 4048, "");
  static_assert(offsetof(FrameProcessor, low_match_count_) == 4052, "");
  static_assert(offsetof(FrameProcessor, last_df_keyframes_) == 4056, "");
}

} // namespace totem
} // namespace pimax
