// pimax_slam.pi.dll -- src/loop_closing/map_alignment.h  (from draft c16)
//
// pimax::totem::MapAlignmentSE3 (upstream svo_online_loopclosing/map_alignment.h).
// Only the constructor survives in the binary (0x1801975b0, map_alignment.obj); every other method
// was dropped by /OPT:REF.  The opengv point vectors of upstream are gone (layout below).
// sizeof == 0xA0 (make_shared<MapAlignmentSE3> allocates 0xB0 in 0x18015cee0).
#pragma once

#include <memory>

#include <Eigen/Core>
#include <cstddef>

#include "common/transformation.h"
#include "common/types.h"   // Positions (float 3xN)

namespace pimax {
namespace totem {

/// factory (getLoopClosingModule 0x18015CEE0): {8, 40.0}
struct MapAlignmentOptions
{
  int ransac3d_min_pts;            // +0  (factory: 8)
  double ransac3d_inlier_percent;  // +8  (factory: 40.0)
};

class MapAlignmentSE3
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MapAlignmentSE3(const MapAlignmentOptions& map_alignment_options);   // 0x1801975b0
  ~MapAlignmentSE3() {}

  typedef std::shared_ptr<MapAlignmentSE3> Ptr;

  inline int getMinRansacPts() { return ransac3d_min_pts_; }

private:
  Positions points_new_;               // +0
  Positions points_old_;               // +16
  int num_points_ = 0;                 // +32
  int num_points_ransac_ = 0;          // +36
  const int max_num_points_;           // +40 (= 100)
  int ransac3d_min_pts_;               // +44
  double ransac3d_inlier_percent_;     // +48
  MapAlignmentOptions options_;        // +56
  Transformation t_rel_combined_;      // +80 (identity)
  Eigen::VectorXd ransac_inliers_;     // +144

  friend struct MapAlignmentSE3LayoutCheck;
};

struct MapAlignmentSE3LayoutCheck
{
  static_assert(sizeof(MapAlignmentSE3) == 0xA0, "sizeof(MapAlignmentSE3)");
  static_assert(offsetof(MapAlignmentSE3, num_points_) == 32, "");
  static_assert(offsetof(MapAlignmentSE3, max_num_points_) == 40, "");
  static_assert(offsetof(MapAlignmentSE3, options_) == 56, "");
  static_assert(offsetof(MapAlignmentSE3, t_rel_combined_) == 80, "");
  static_assert(offsetof(MapAlignmentSE3, ransac_inliers_) == 144, "");
};

}  // namespace totem
}  // namespace pimax
