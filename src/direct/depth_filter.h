// pimax_slam.pi.dll -- src/direct/depth_filter.h
// Pimax fork of svo_direct/include/svo/direct/depth_filter.h (chunk c06; object range
// 0x18009DBE0 .. 0x1800A3900).
// sizeof(DepthFilter) == 0x160 (aligned new at 0x1800E1648), vtable 0x1803B1E80 (1 slot:
// scalar deleting dtor 0x18009FBC0).
#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include "common/types.h"          // FloatType == float, FramePtr, FrameBundlePtr, SeedState ...
#include "common/camera_fwd.h"     // CameraBundle
#include "common/transformation.h"
#include "direct/matcher.h"
#include "direct/feature_detection_types.h"   // DetectorOptions (passed by value)

namespace pimax {
namespace totem {

class Map;
struct FeatureWrapper;

/// Depth-filter config parameters.  56 bytes, copied verbatim into DepthFilter::options_ (+8).
/// Pimax: the two thresholds are FloatType (float) instead of double -- everything else keeps
/// the upstream order (offsets verified: +0 float used as threshold, +25 threaded flag,
/// +27 scan_epi_unit_sphere, +48/+49 affine flags).  Defaults are upstream's; the values used
/// at run time come from the factory (other chunk).  TODO(verify) defaults.
struct DepthFilterOptions
{
  FloatType seed_convergence_sigma2_thresh = 200.0;      // +0
  FloatType mappoint_convergence_sigma2_thresh = 500.0;  // +4  (no reader in this chunk)
  bool use_inverse_depth = true;                         // +8
  size_t max_search_level = 2;                           // +16
  bool verbose = false;                                  // +24
  bool use_threaded_depthfilter = true;                  // +25
  bool update_3d_point = true;                           // +26
  bool scan_epi_unit_sphere = false;                     // +27
  size_t max_n_seeds_per_frame = 200;                    // +32
  size_t max_map_seeds_per_frame = 200;                  // +40
  bool affine_est_offset = true;                         // +48
  bool affine_est_gain = false;                          // +49
  bool extra_map_points = false;                         // +50
};

/// Depth filter implements the Bayesian Update proposed in:
/// "Video-based, Real-Time Multi View Stereo" by G. Vogiatzis and C. Hernandez.
class DepthFilter
{
protected:

  /// Job for multi-threading.  80 bytes (deque<Job> allocates 0x50 per element).
  struct Job
  {
    enum Type { UPDATE, SEED_INIT } type;     // +0
    FrameBundlePtr frame_bundle;              // +8   Pimax-new: SEED_INIT works on a whole bundle
    FramePtr cur_frame;                       // +24
    FramePtr ref_frame;                       // +40
    size_t ref_frame_seed_index;              // +56
    float min_depth, max_depth, mean_depth;   // +64 +68 +72  (only ever set to 1.0f, never read)
    // ~Job(): implicit, emitted at 0x18009FAF0 (releases ref_frame, cur_frame, frame_bundle)

    /// Default constructor
    Job()
      : cur_frame(nullptr), ref_frame(nullptr)
    {}

    /// Constructor for seed update (inlined in updateSeeds 0x1800A3270)
    Job(const FramePtr& _cur_frame, const FramePtr& _ref_frame, const size_t _ref_index)
      : type(UPDATE)
      , cur_frame(_cur_frame)
      , ref_frame(_ref_frame)
      , ref_frame_seed_index(_ref_index)
    {}

    /// Constructor for seed initialization (inlined in addKeyframe 0x18009FE90).
    /// TODO(verify): the depths might be explicit 1.0f arguments at the call site instead.
    Job(const FrameBundlePtr& _frame_bundle)
      : type(SEED_INIT), frame_bundle(_frame_bundle), cur_frame(nullptr), ref_frame(nullptr)
      , min_depth(1.0f), max_depth(1.0f), mean_depth(1.0f)
    {}
  };

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<DepthFilter> Ptr;
  typedef std::mutex mutex_t;
  typedef std::unique_lock<mutex_t> ulock_t;
  typedef std::queue<Job> JobQueue;

  DepthFilterOptions options_;                                 // +8

  /// Default Constructor (0x18009F650).  Pimax: detector/cams are ignored (no feature detector
  /// inside the depth filter any more); DetectorOptions is passed by value (caller copies 0x50
  /// bytes).  TODO(verify) by-value.
  DepthFilter(
      const DepthFilterOptions& options,
      DetectorOptions detector,
      const CameraBundlePtr& cams);

  /// Constructor (0x18009F280)
  DepthFilter(
      const DepthFilterOptions& options);

  /// Destructor stops thread if necessary (0x18009F940, deleting dtor 0x18009FBC0).
  virtual ~DepthFilter();

  /// Start this thread when seed updating should be in a parallel thread (inlined in ctor).
  void startThread();

  /// Stop the parallel thread that is running (0x1800A2730).
  void stopThread();

  /// Pimax: seeds are initialized for a whole frame bundle; the frames-without-seeds lists are
  /// pruned against the keyframes of the map (0x18009FE90).
  void addKeyframe(
      const FrameBundlePtr& frame_bundle,
      const Map& map);

  /// Resets all jobs of the parallel thread (0x1800A2300).
  void reset();

  /// test
  Matcher& getMatcher() { return *matcher_; }

  /// Update seeds (0x1800A3270)
  size_t updateSeeds(
      const std::vector<FramePtr>& frames_with_seeds,
      const FramePtr& new_frame);

  /// Pimax-new (0x18009FC10): copies the list of reference frames that had no seeds left when
  /// updateSeeds() was called.  Returns false if that list is empty.  TODO(verify) name; the
  /// decorated name sorts between ??_GDepthFilter and ?_Growmap@deque inside the object, i.e.
  /// it starts with an upper-case letter (Pimax naming style).
  bool GetFramesWithoutSeeds(std::vector<FramePtr>& frames);

protected:
  mutex_t jobs_mut_;                                           // +64
  JobQueue jobs_;                                              // +144
  std::deque<FramePtr> frames_without_seeds_;                  // +184 Pimax-new TODO(verify) name
  std::vector<FramePtr> frames_without_seeds_vec_;             // +224 Pimax-new TODO(verify) name
  std::condition_variable jobs_condvar_;                       // +248
  std::unique_ptr<std::thread> thread_;                        // +320
  bool quit_thread_ = false;                                   // +328
  Matcher::Ptr matcher_;                                       // +336
  // Pimax: upstream feature_detector_mut_ / feature_detector_ / sec_feature_detector_ removed.

  /// A thread that is continuously updating the seeds (0x1800A3630).
  void updateSeedsLoop();

public:
  static void layout_check();
};

inline void DepthFilter::layout_check()
{
  static_assert(sizeof(DepthFilterOptions) == 56, "sizeof(DepthFilterOptions)");
  static_assert(offsetof(DepthFilterOptions, use_threaded_depthfilter) == 25, "DepthFilterOptions::use_threaded_depthfilter");
  static_assert(offsetof(DepthFilterOptions, scan_epi_unit_sphere) == 27, "DepthFilterOptions::scan_epi_unit_sphere");
  static_assert(offsetof(DepthFilterOptions, affine_est_offset) == 48, "DepthFilterOptions::affine_est_offset");
  static_assert(sizeof(Job) == 80, "sizeof(DepthFilter::Job)");
  static_assert(offsetof(Job, frame_bundle) == 8, "Job::frame_bundle");
  static_assert(offsetof(Job, cur_frame) == 24, "Job::cur_frame");
  static_assert(offsetof(Job, ref_frame) == 40, "Job::ref_frame");
  static_assert(offsetof(Job, ref_frame_seed_index) == 56, "Job::ref_frame_seed_index");
  static_assert(offsetof(Job, min_depth) == 64, "Job::min_depth");
  static_assert(sizeof(DepthFilter) == 0x160, "sizeof(DepthFilter)");
  static_assert(offsetof(DepthFilter, options_) == 8, "DepthFilter::options_");
  static_assert(offsetof(DepthFilter, jobs_mut_) == 64, "DepthFilter::jobs_mut_");
  static_assert(offsetof(DepthFilter, jobs_) == 144, "DepthFilter::jobs_");
  static_assert(offsetof(DepthFilter, frames_without_seeds_) == 184, "DepthFilter::frames_without_seeds_");
  static_assert(offsetof(DepthFilter, frames_without_seeds_vec_) == 224, "DepthFilter::frames_without_seeds_vec_");
  static_assert(offsetof(DepthFilter, jobs_condvar_) == 248, "DepthFilter::jobs_condvar_");
  static_assert(offsetof(DepthFilter, thread_) == 320, "DepthFilter::thread_");
  static_assert(offsetof(DepthFilter, quit_thread_) == 328, "DepthFilter::quit_thread_");
  static_assert(offsetof(DepthFilter, matcher_) == 336, "DepthFilter::matcher_");
}

namespace depth_filter_utils {

/// Pimax: detects FAST corners itself on img_pyr_[0] of every camera and fills all frames of
/// the bundle (0x1800A0AB0).
void initializeSeeds(
    const FrameBundlePtr& frame_bundle);

/// Update Seed (0x1800A2AC0)
bool updateSeed(
    const Frame& cur_frame,
    Frame& ref_frame,
    const size_t& seed_index,
    Matcher& matcher,
    const FloatType sigma2_convergence_threshold,
    const bool check_visibility = true,
    const bool check_convergence = false,
    const bool use_vogiatzis_update = true);

/// 0x1800A2810
bool updateFilterVogiatzis(
    const FloatType z,
    const FloatType tau2,
    const FloatType z_range,
    Eigen::Ref<SeedState>& seed);

/// Inlined into updateSeed.  TODO(verify): only the NaN test survives in the binary, which
/// means the state is updated on a copy (by-value parameter).
bool updateFilterGaussian(
    const FloatType z,
    const FloatType tau2,
    SeedState seed);

/// Compute the uncertainty of the measurement (0x1800A0530).  Pimax: float maths.
FloatType computeTau(
    const Transformation& T_ref_cur,
    const BearingVector& f,
    const FloatType z,
    const FloatType px_error_angle);

} // namespace depth_filter_utils

} // namespace totem
} // namespace pimax
