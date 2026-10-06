// pimax_slam.pi.dll -- src/direct/feature_detection_utils.h
// Pimax fork of svo_direct/include/svo/direct/feature_detection_utils.h (chunk c06; object range
// 0x1800AAC30 .. 0x1800ADD10).
// Only the functions present in the image are declared.
#pragma once

#include <array>
#include <memory>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include "common/types.h"
#include "common/camera_fwd.h"
#include "common/occupancy_grid_2d.h"
#include "direct/feature_detection_types.h"

namespace pimax {
namespace totem {

class AbstractDetector;
using AbstractDetectorPtr = std::shared_ptr<AbstractDetector>;

namespace feature_detection_utils {

/// 0x1800AD840 -- Pimax: always a FastGradDetector (detector_type ignored).
AbstractDetectorPtr makeDetector(
    const DetectorOptions& options,
    const CameraPtr& cam);

/// 0x1800ACB30 -- Pimax: no mask parameter, stable sort.
void fillFeatures(const Corners& corners,
    const FeatureType& type,
    const double& threshold,
    const size_t max_n_features,
    Keypoints& keypoints,
    Scores& scores,
    Levels& levels,
    Gradients& gradients,
    FeatureTypes& types,
    OccupandyGrid2D& grid);

/// 0x1800AC5E0 -- Pimax: mask parameter, saturation rejection.
void fastDetector(
    const ImgPyr& img_pyr,
    const int threshold,
    const int border,
    const size_t min_level,
    const size_t max_level,
    const cv::Mat& mask,
    Corners& corners,
    OccupandyGrid2D& grid);

/// 0x1800AC200 -- Pimax: no Gaussian blur, cv::parallel_for_ over rows.
void edgeletDetector_V2(
    const ImgPyr& img_pyr,
    const int threshold,
    const int border,
    const int min_level,
    const int max_level,
    Corners& corners,
    OccupandyGrid2D& grid);

/// 0x1800AD4A0 -- Pimax: float histogram and float result.
float getAngleAtPixelUsingHistogram(
    const cv::Mat& img,
    const Eigen::Vector2i& px,
    const size_t halfpatch_size);

namespace angle_hist {

constexpr size_t n_bins = 36;
using AngleHistogram = std::array<float, n_bins>;   // Pimax: float (upstream double)

void angleHistogram(
    const cv::Mat& img, int x, int y, int halfpatch_size, AngleHistogram& hist);

bool gradientAndMagnitudeAtPixel(
    const cv::Mat& img, int x, int y, double* mag, double* angle);

/// 0x1800ADA80
void smoothOrientationHistogram(
    AngleHistogram& hist);

double getDominantAngle(
    const AngleHistogram& hist);

} // namespace angle_hist

} // namespace feature_detection_utils
} // namespace totem
} // namespace pimax
