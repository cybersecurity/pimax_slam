// pimax_slam.pi.dll -- src/frontend/reprojector.h  (from draft c12; option layout from c14's factory)
//
// pimax::totem::Reprojector (upstream svo/include/svo/reprojector.h).
// Heavily modified by Pimax: candidate validity flag + grid NMS, depth/height gating,
// ORB-style rotation-consistency check with IC_Angle, extended statistics, no occupancy grid use.
#pragma once

#include <memory>
#include <vector>
#include <cstddef>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

#include "common/feature_wrapper.h"
#include "common/frame.h"
#include "common/occupancy_grid_2d.h"
#include "common/point.h"
#include "common/types.h"

namespace pimax {
namespace totem {

class Matcher;

/// Reprojector config parameters (96 bytes; copied verbatim by the ctor 0x1801412F0).
/// Layout = upstream + one Pimax size_t (loadReprojectorOptions 0x18015E0B0 stores +0 100, +8 100,
/// +16 100, +24 20, +32 19, +40 1, +48 -1.0, +56 0, +64 200.0f, +68 1, +69 1, +70 0, +72 50,
/// +80 20, +88 50.0).  projectMapInFrame 0x180119260 reads +32 as max_n_kfs (c10), so the extra
/// field sits at +24 (or +16; TODO(verify) which of cell_size/extra is which).  c12 had guessed
/// the extra field at +56, c14 at +32 -- both contradicted by the +32 read.
/// Fields read by c12 code (sure): +0, +8, +64 (float), +69, +70.  Defaults are upstream's.
struct ReprojectorOptions
{
  size_t max_n_features_per_frame = 120;     // +0  sure (used when !need_imu_init)
  size_t max_map_features_per_frame = 120;   // +8  sure (used when need_imu_init)
  size_t cell_size = 30;                     // +16
  size_t pimax_unknown_24 = 0;               // +24 [pimax-new] factory 20  TODO(verify) name/use
  size_t max_n_kfs = 5;                      // +32 sure (projectMapInFrame); factory 19
  bool reproject_unconverged_seeds = true;   // +40
  double max_unconverged_seeds_ratio = -1.0; // +48
  size_t min_required_features = 0;          // +56
  FloatType seed_sigma2_thresh = 200;        // +64 sure (float!)
  bool remove_unconstrained_points = true;   // +68
  bool affine_est_offset = true;             // +69 sure
  bool affine_est_gain = false;              // +70 sure
  bool use_kfs_from_global_map = false;      // +71 (not written by the factory)
  size_t max_fixed_landmarks = 50;           // +72
  size_t max_n_global_kfs = 20;              // +80
  double fixed_lm_grid_size = 50;            // +88
};
static_assert(sizeof(ReprojectorOptions) == 96, "ReprojectorOptions must be 96 bytes");

/// sizeof == 0xF0 (allocated with Eigen aligned new(0xF0) in FrameProcessorBase ctor).
class Reprojector
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<Reprojector> Ptr;

  Reprojector(const ReprojectorOptions& options,
              size_t camera_index);

  ~Reprojector() = default;

  ReprojectorOptions options_;                         // +0

  struct Statistics                                    // 48 bytes
  {
    inline void reset()
    {
      n_matches = 0;
      n_trials = 0;
      sum_lm_succeeded_reproj = 0;
      sum_lm_obs = 0;
      n_lm_matches = 0;
      n_seed_matches = 0;
    }
    void add(const Statistics s)
    {
      n_matches += s.n_matches;
      n_trials += s.n_trials;
      sum_lm_succeeded_reproj += s.sum_lm_succeeded_reproj;
      sum_lm_obs += s.sum_lm_obs;
      n_lm_matches += s.n_lm_matches;
      n_seed_matches += s.n_seed_matches;
    }
    inline double successRate()
    {
      if (n_trials == 0)
      {
        return 0.0;
      }
      return n_matches / (1.0 * n_trials);
    }
    size_t n_matches = 0;                // +0
    size_t n_trials = 0;                 // +8
    size_t sum_lm_succeeded_reproj = 0;  // +16 Pimax: sum of point->n_succeeded_reproj_ over matched landmarks
    size_t sum_lm_obs = 0;               // +24 Pimax: sum of point->obs_.size() over matched landmarks
    size_t n_lm_matches = 0;             // +32 Pimax: matches that came from a landmark
    size_t n_seed_matches = 0;           // +40 Pimax: matches that came from a seed
  };
  Statistics stats_;                                   // +96
  Statistics fixed_lm_stats_;                          // +144 (unused in this build)

  /// A candidate is a point that projects into the image plane and for which we
  /// will search a maching feature in the image. 72 bytes.
  struct Candidate
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool valid = true;    //!< +0  Pimax: cleared by grid NMS (filterCandidatesByGrid)
    FramePtr ref_frame;   //!< +8  Reference frame.
    size_t ref_index;     //!< +24 Feature index in reference frame.
    Keypoint cur_px;      //!< +32 Projected 2D pixel location in current frame (float).
    int n_reproj = 0;     //!< +40 Number of previously successful projections for quality.
    Score score;          //!< +44 ref_frame->score_vec_(ref_index) (float).
    FeatureType type;     //!< +48 Type of feature to determine quality.
    size_t n_obs;         //!< +56
    float depth;          //!< +64 Pimax: depth of the point in the current frame (sort key).

    Candidate() = default;

    Candidate(const FramePtr _ref_frame, const size_t _ref_index,
              const Keypoint& _cur_px, const int _n_reproj,
              const Score _score, const FeatureType& _type,
              const size_t _n_obs, const double _depth)
      : ref_frame(_ref_frame)
      , ref_index(_ref_index)
      , cur_px(_cur_px)
      , n_reproj(_n_reproj)
      , score(_score)
      , type(_type)
      , n_obs(_n_obs)
      , depth(static_cast<float>(_depth))
    { ; }
  };
  using Candidates = std::vector<Candidate>;

  std::unique_ptr<OccupandyGrid2D> fixed_landmark_grid_;   // +192 (never created)
  std::unique_ptr<OccupandyGrid2D> grid_;                  // +200 (never created in this build!)
  Candidates candidates_;                                  // +208
  size_t camera_index_;                                    // +232

  /// Project points seed in close_kfs into frame.
  /// Pimax: extra flag = FrameProcessorBase+0xD80 (true until "imu_initial true").
  void reprojectFrames(
      const FramePtr &frame,
      const std::vector<FramePtr>& close_kfs,
      std::vector<PointPtr>& trash_points,
      const bool& need_imu_init);

 private:
  friend struct ReprojectorLayoutCheck;
};

static_assert(sizeof(Reprojector::Statistics) == 48, "");
static_assert(sizeof(Reprojector::Candidate) == 72, "");
struct ReprojectorLayoutCheck
{
  static_assert(sizeof(Reprojector) == 0xF0, "sizeof(Reprojector)");
  static_assert(offsetof(Reprojector, stats_) == 96, "");
  static_assert(offsetof(Reprojector, fixed_lm_stats_) == 144, "");
  static_assert(offsetof(Reprojector, fixed_landmark_grid_) == 192, "");
  static_assert(offsetof(Reprojector, grid_) == 200, "");
  static_assert(offsetof(Reprojector, candidates_) == 208, "");
  static_assert(offsetof(Reprojector, camera_index_) == 232, "");
  static_assert(offsetof(ReprojectorOptions, max_n_kfs) == 32, "");
  static_assert(offsetof(ReprojectorOptions, seed_sigma2_thresh) == 64, "");
  static_assert(offsetof(ReprojectorOptions, affine_est_offset) == 69, "");
  static_assert(offsetof(ReprojectorOptions, max_fixed_landmarks) == 72, "");
};

namespace reprojector_utils {

  // 0x180143DB0 Pimax-new
  void filterCandidatesByGrid(
      Reprojector::Candidates& candidates,
      Reprojector::Candidates& filtered);

  // 0x180142A60
  void matchCandidates(const FramePtr& frame,
      const size_t max_n_features_per_frame,
      const bool affine_est_offset,
      const bool affine_est_gain,
      Reprojector::Candidates& candidates,
      OccupandyGrid2D* grid,            // unused (binary passes grid_.get(), which is null)
      Reprojector::Statistics& stats,
      const FloatType seed_sigma2_thresh=200);

  // 0x1801422D0
  bool matchCandidate(
      const FramePtr& frame,
      Reprojector::Candidate& c,
      Matcher& matcher,
      FeatureWrapper& feature,
      float* angle_ref,
      float* angle_cur,
      const std::vector<int>& umax,
      const FloatType seed_sigma2_thresh=200);

  // 0x180141C40
  bool getCandidate(
      const FramePtr& cur_frame,
      const FramePtr& ref_frame,
      const size_t& ref_index,
      Reprojector::Candidate& candidate);

  // 0x180142190 Pimax-new
  bool checkPatchExposure(
      const FramePtr& ref_frame,
      const FramePtr& cur_frame,
      const Keypoint& px_cur);

  // 0x1801415B0 Pimax-new (ORB-SLAM IC_Angle)
  float IC_Angle(const cv::Mat& image, const Keypoint& pt, const std::vector<int>& u_max);

  // 0x180141B00 Pimax-new (ORB-SLAM umax table, HALF_PATCH_SIZE 15)
  std::vector<int> computeUmax();

} // namespace reprojector_utils
} // namespace totem
} // namespace pimax
