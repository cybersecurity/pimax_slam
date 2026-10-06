// pimax_slam.pi.dll -- src/frontend/stereo_triangulation.h  (drafts c12 + c13 merged)
//
// pimax::totem::StereoTriangulation -- fork of svo/stereo_triangulation.h.
// sizeof 0x90 (FrameProcessor: operator new(0x90) + _Ref_count_resource; StereoInit: unique_ptr):
//   +0   StereoTriangulationOptions options_ (32 bytes, copied)
//   +32  DetectorPtr feature_detector_      (shared_ptr; detect = vtable slot 1)
//   +48  cv::Mat mask_                       Pimax: mask with the existing features blotted out
// Functions: ctor 0x180145320, triangulate 0x180145470, computeStd 0x180146360 (c12 bodies),
// compute 0x180147110 (c13 body).  Names: c12's `triangulate`/`computeStd` (body owner) are used;
// c13's drafts called them triangulatePoint/computeStdDev.
#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

#include "common/feature_wrapper.h"
#include "common/transformation.h"
#include "common/types.h"

namespace pimax {
namespace totem {

class AbstractDetector;
using DetectorPtr = std::shared_ptr<AbstractDetector>;
class Matcher;

/// 32 bytes.  Factory values (loadStereoOptions 0x18015E130): 120, 0.3, 1.0, 0.05.
/// Defaults below = the StereoInit ctor values {120, 1/3, 1.0, 0.02} (c11, 0x1803B71B0..).
struct StereoTriangulationOptions
{
  size_t triangulate_n_features = 120;   // +0
  double mean_depth_inv = 1.0 / 3.0;     // +8
  double min_depth_inv = 1.0 / 1.0;      // +16
  double max_depth_inv = 1.0 / 50.0;     // +24
};
static_assert(sizeof(StereoTriangulationOptions) == 32, "");

class StereoTriangulation
{
 public:
  typedef std::shared_ptr<StereoTriangulation> Ptr;

  StereoTriangulationOptions options_;   // +0
  DetectorPtr feature_detector_;         // +32
  cv::Mat mask_;                         // +48 [pimax-new]  TODO(verify) name

  /// 0x180145320
  StereoTriangulation(const StereoTriangulationOptions& options,
                      const DetectorPtr& feature_detector);
  ~StereoTriangulation() = default;

  /// 0x180147110.  Pimax adds `is_init`: true from StereoInit (0x18012BB60), false from the frame
  /// processor (0x1800B2F80).
  void compute(const FramePtr& frame0, const FramePtr& frame1, bool is_init);

  /// 0x180145470 [pimax-new] (`this` unused).  DLT triangulation with P0 = [I|0], P1 = [R|t];
  /// returns the depth / a reprojection measure >= 0 on success, negative codes on failure.
  double triangulate(const FramePtr& frame0, const FramePtr& frame1,
                     const Transformation& T_f1f0,
                     const Eigen::Matrix<double, 3, 4>& P0,
                     const Eigen::Matrix<double, 3, 4>& P1,
                     const Matcher& matcher,
                     const FeatureWrapper& ref_ftr,
                     Eigen::Vector3d& x3D) const;

  /// 0x180146360 [pimax-new] sqrt(sum(pow(v - mean, 2.0)) / v.size()).  TODO(verify) name/linkage.
  double computeStd(const std::vector<double>& values, double mean);

 private:
  friend struct StereoTriangulationLayoutCheck;
};

struct StereoTriangulationLayoutCheck
{
  static_assert(sizeof(StereoTriangulation) == 0x90, "sizeof(StereoTriangulation)");
  static_assert(offsetof(StereoTriangulation, feature_detector_) == 32, "");
  static_assert(offsetof(StereoTriangulation, mask_) == 48, "");
};

}  // namespace totem
}  // namespace pimax
