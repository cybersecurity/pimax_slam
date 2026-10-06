// feature_detection.cpp -- Pimax fork of svo_direct/src/feature_detection.cpp
// Chunk c06_common_depthfilter, object range 0x1800AA150 .. 0x1800AAC30.

#include "direct/feature_detection.h"

#include <cmath>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <vikit/cameras/camera_geometry_base.h>
#include "direct/feature_detection_utils.h"

namespace pimax {
namespace totem {

namespace fd_utils = feature_detection_utils;

//------------------------------------------------------------------------------
// 0x1800AA150 (upstream-identical)
AbstractDetector::AbstractDetector(
    const DetectorOptions& options,
    const CameraPtr& cam)
  : options_(options)
  , grid_(options_.cell_size,
          std::ceil(static_cast<double>(cam->imageWidth())/options_.cell_size),
          std::ceil(static_cast<double>(cam->imageHeight())/options_.cell_size))
  , closeness_check_grid_(options_.cell_size/options_.sec_grid_fineness,
                    std::ceil(options_.sec_grid_fineness * static_cast<double>(cam->imageWidth())/options_.cell_size),
                    std::ceil(options_.sec_grid_fineness * static_cast<double>(cam->imageHeight())/options_.cell_size))
{}

//------------------------------------------------------------------------------
// 0x1800AA6E0
void FastGradDetector::detect(
    const ImgPyr& img_pyr,
    const cv::Mat& mask,
    const size_t max_n_features,
    Keypoints& px_vec,
    Scores& score_vec,
    Levels& level_vec,
    Gradients& grad_vec,
    FeatureTypes& types_vec)
{
  {
    // Detect fast corners.
    // Pimax: two FAST passes (threshold_primary, then threshold_secondary if too few strong
    // corners); the mask is applied inside fastDetector; corners must beat threshold_secondary.
    Corners corners(
          grid_.n_cols*grid_.n_rows,
          Corner(0, 0, options_.threshold_secondary, 0, 0.0f));
    fd_utils::fastDetector(
          img_pyr, options_.threshold_primary, options_.border,
          options_.min_level, options_.max_level, mask, corners, grid_);

    size_t n_strong = 0;
    for(const Corner& c : corners)
    {
      if(c.score > options_.threshold_primary)
        ++n_strong;
    }
    if(n_strong < max_n_features && n_strong > 8)
    {
      fd_utils::fastDetector(
            img_pyr, options_.threshold_secondary, options_.border,
            options_.min_level, options_.max_level, mask, corners, grid_);
    }
    fd_utils::fillFeatures(
          corners, FeatureType::kCorner, options_.threshold_secondary,
          max_n_features, px_vec, score_vec, level_vec, grad_vec, types_vec, grid_);
  }

  if(!options_.disable_edgelets)
  {
    int max_features = static_cast<int>(max_n_features) - px_vec.cols();
    if(max_features > 0)
    {
      // Detect edgelets.
      Corners corners(
            grid_.n_cols * grid_.n_rows,
            Corner(0, 0, options_.threshold_edgelet, 0, 0.0f));
      fd_utils::edgeletDetector_V2(
            img_pyr, options_.threshold_edgelet, options_.border,
            options_.min_level, options_.max_level, corners, grid_);
      fd_utils::fillFeatures(
            corners, FeatureType::kEdgelet, options_.threshold_edgelet,
            max_features, px_vec, score_vec, level_vec, grad_vec, types_vec, grid_);
    }
  }

  resetGrid();
}

} // namespace totem
} // namespace pimax
