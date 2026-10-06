// pimax_slam.pi.dll -- src/direct/feature_detection_types.h
// Pimax fork of svo_direct/include/svo/direct/feature_detection_types.h (chunk c06).
#pragma once

#include <cstddef>
#include <vector>
#include "common/types.h"

namespace pimax {
namespace totem {

/// 20 bytes (vector<Corner> stride 20 in 0x1800AA6E0 / 0x1800ACB30).
struct Corner
{
  int x;        ///< +0  x-coordinate of corner in the image.
  int y;        ///< +4  y-coordinate of corner in the image.
  int level;    ///< +8  pyramid level of the corner.
  float score;  ///< +12 shi-tomasi score of the corner.
  float angle;  ///< +16 for gradient-features: dominant gradient angle.

  Corner(int _x, int _y, float _score, int _level, float _angle)
    : x(_x), y(_y), level(_level), score(_score), angle(_angle)
  {}
};
using Corners = std::vector<Corner>;

enum class DetectorType
{
  kFast,            ///< Fast Corner Detector by Edward Rosten
  kGrad,            ///< Gradient detector for edgelets
  kFastGrad,        ///< Combined Fast and Gradient detector
  kShiTomasi,       ///< Shi-Tomasi detector
  kShiTomasiGrad,   ///< Combined Shi-Tomasi, fast and Gradient detector
  kGridGrad,        ///< Gradient detector with feature grid.
  kAll,             ///< Every pixel is a feature!
  kGradHuangMumford,///< 'Natural' edges (see Huang CVPR'99)
  kCanny,           ///< Canny edge detector
  kSobel            ///< Sobel edge detector
};

/// 80 bytes, copied into AbstractDetector::options_ (+8) with 5 xmm moves (0x1800AA150).
/// Defaults: the values written by 0x18015DAD0 (other chunk; looks like the default ctor /
/// option loader).  detector_type is NOT written there (makeDetector ignores it anyway).
/// Pimax changes vs upstream: new fields +40 / +48 (names are ours), cell_size 20,
/// threshold_primary 20, threshold_secondary 10 (now the FAST fallback threshold),
/// threshold_shitomasi 50.  TODO(verify) all names marked "Pimax-new".
struct DetectorOptions
{
  size_t cell_size = 20;                 // +0
  int max_level = 2;                     // +8
  int min_level = 0;                     // +12
  int border = 8;                        // +16
  DetectorType detector_type;            // +20 (not initialised by 0x18015DAD0)
  double threshold_primary = 20.0;       // +24 FAST threshold (1st pass)
  double threshold_secondary = 10.0;     // +32 FAST threshold (2nd pass) and corner score threshold
  bool disable_edgelets = false;         // +40 Pimax-new: skip the edgelet detector
  double threshold_edgelet = 200.0;      // +48 Pimax-new: edgelet (gradient magnitude) threshold
  int sampling_level = 0;                // +56
  int level = 0;                         // +60
  size_t sec_grid_fineness = 1;          // +64
  double threshold_shitomasi = 50.0;     // +72
};

static_assert(sizeof(Corner) == 20, "sizeof(Corner)");
static_assert(sizeof(DetectorOptions) == 80, "sizeof(DetectorOptions)");
static_assert(offsetof(DetectorOptions, detector_type) == 20, "DetectorOptions::detector_type");
static_assert(offsetof(DetectorOptions, threshold_primary) == 24, "DetectorOptions::threshold_primary");
static_assert(offsetof(DetectorOptions, disable_edgelets) == 40, "DetectorOptions::disable_edgelets");
static_assert(offsetof(DetectorOptions, threshold_edgelet) == 48, "DetectorOptions::threshold_edgelet");
static_assert(offsetof(DetectorOptions, sampling_level) == 56, "DetectorOptions::sampling_level");
static_assert(offsetof(DetectorOptions, sec_grid_fineness) == 64, "DetectorOptions::sec_grid_fineness");
static_assert(offsetof(DetectorOptions, threshold_shitomasi) == 72, "DetectorOptions::threshold_shitomasi");

} // namespace totem
} // namespace pimax
