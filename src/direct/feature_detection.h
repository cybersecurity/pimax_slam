// pimax_slam.pi.dll -- src/direct/feature_detection.h
// Pimax fork of svo_direct/include/svo/direct/feature_detection.h (chunk c06; object range
// 0x1800AA150 .. 0x1800AAC30).
// Only AbstractDetector and FastGradDetector exist in the image (makeDetector always builds a
// FastGradDetector).  vtables: AbstractDetector 0x1803B2720 [0x1800AA4F0 deleting dtor,
// _purecall], FastGradDetector 0x1803B27D0 [0x1800AA4F0, 0x1800AA6E0 detect].
// sizeof(FastGradDetector) == 0xE8 (plain operator new, no aligned new).
#pragma once

#include <cstddef>
#include <memory>

#include <opencv2/core/core.hpp>
#include "common/types.h"
#include "common/camera_fwd.h"
#include "common/occupancy_grid_2d.h"
#include "direct/feature_detection_types.h"

namespace pimax {
namespace totem {

class AbstractDetector
{
public:
  typedef std::shared_ptr<AbstractDetector> Ptr;

  DetectorOptions options_;                       // +8

  /// Default constructor (0x1800AA150, upstream-identical).
  AbstractDetector(
      const DetectorOptions& options,
      const CameraPtr& cam);

  /// Default destructor (scalar deleting dtor 0x1800AA4F0).
  virtual ~AbstractDetector() = default;

  // no copy
  AbstractDetector& operator=(const AbstractDetector&) = delete;
  AbstractDetector(const AbstractDetector&) = delete;

  virtual void detect(
      const ImgPyr& img_pyr,
      const cv::Mat& mask,
      const size_t max_n_features,
      Keypoints& px_vec,
      Scores& score_vec,
      Levels& level_vec,
      Gradients& grad_vec,
      FeatureTypes& types_vec) = 0;

  inline void resetGrid()
  {
    grid_.reset();
    closeness_check_grid_.reset();
  }

  // occupancy for the current feature type
  OccupandyGrid2D grid_;                          // +88
  // this is to additionally check whether detected features are near some exiting ones
  // useful for having mutliple detectors
  OccupandyGrid2D closeness_check_grid_;          // +160

  static void layout_check();
};

class FastGradDetector : public AbstractDetector
{
public:
  using AbstractDetector::AbstractDetector; // default constructor
  virtual ~FastGradDetector() = default;

  // 0x1800AA6E0
  virtual void detect(
      const ImgPyr& img_pyr,
      const cv::Mat& mask,
      const size_t max_n_features,
      Keypoints& px_vec,
      Scores& score_vec,
      Levels& level_vec,
      Gradients& grad_vec,
      FeatureTypes& types_vec) override;
};

inline void AbstractDetector::layout_check()
{
  static_assert(sizeof(AbstractDetector) == 0xE8, "sizeof(AbstractDetector)");
  static_assert(offsetof(AbstractDetector, options_) == 8, "AbstractDetector::options_");
  static_assert(offsetof(AbstractDetector, grid_) == 88, "AbstractDetector::grid_");
  static_assert(offsetof(AbstractDetector, closeness_check_grid_) == 160, "AbstractDetector::closeness_check_grid_");
  static_assert(sizeof(FastGradDetector) == 0xE8, "sizeof(FastGradDetector)");
}

} // namespace totem
} // namespace pimax
