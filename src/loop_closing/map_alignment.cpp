// pimax_slam.pi.dll -- src/loop_closing/map_alignment.cpp  (draft c16_loop_closing)
//
// map_alignment.obj: only the constructor survived /OPT:REF (0x1801975b0).
// upstream-identical (except for the removed opengv members, see map_alignment.h).
#include "loop_closing/map_alignment.h"

namespace pimax {
namespace totem {

// 0x1801975b0
MapAlignmentSE3::MapAlignmentSE3(const MapAlignmentOptions& map_alignment_options)
  : num_points_(0), max_num_points_(100), options_(map_alignment_options)
{
  points_old_.conservativeResize(Eigen::NoChange, max_num_points_);
  points_new_.conservativeResize(Eigen::NoChange, max_num_points_);
  ransac3d_min_pts_ = options_.ransac3d_min_pts;
  ransac3d_inlier_percent_ = options_.ransac3d_inlier_percent;
}

}  // namespace totem
}  // namespace pimax
