// pimax_slam.pi.dll -- src/ceres_backend/ceres_backend_interface.hpp  (drafts c01 + c00 + c13)
//
// pimax::totem::CeresBackendInterface
//
// Fork of rpg_svo_pro_open svo_ceres_backend/include/svo/ceres_backend_interface.hpp.
// Pimax removed the optimization thread, the motion detector, the publisher, the loop-closure
// fixation logic and the AbstractBundleAdjustment base class (the object has NO vtable; it is
// created with std::make_shared in interface/ceres_backend_factory.cpp 0x180158F70, object size
// 0x4F0 = 1264 bytes). Optimization runs synchronously from FrameProcessor via optimize().
//
// Field offsets were taken from the ctor 0x180008CD0, the dtor 0x180009950 (both outside the
// c01 address range) and from every method in 0x180010440..0x18001B4D0.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "common/types.h"                 // FramePtr, FrameBundlePtr, PointPtr, Transformation, ...
#include "ceres_backend/estimator.hpp"    // pimax::totem::Estimator (+ BackendId, MarginalizationTiming, ImuParameters)
#include "ceres_backend/outlier_rejection.hpp"

namespace vk { class PerformanceMonitor; }

namespace pimax {
namespace totem {

class ImuProcessor;     // upstream svo::ImuHandler
class Map;              // frontend map (svo::Map); not used by any reconstructed method

/// Upstream svo/abstract_bundle_adjustment.h.  Pimax dropped AbstractBundleAdjustment; the type
/// survives as CeresBackendInterface::type_ (+0x3D0, = kCeres) and is copied into
/// FrameProcessorBase::bundle_adjustment_type_ (+3424; the frontend only tests != 0 / == 2).
enum class BundleAdjustmentType : int
{
  kNone = 0,
  kGtsam = 1,
  kCeres = 2
};

// size 64 (copied as 4 x 16 bytes in the ctor). Values are the ones hard-coded by the factory
// 0x180158F70 (the factory does not read any config); in-class defaults are the upstream ones.
struct CeresBackendInterfaceOptions
{
  // stores in the factory 0x180158F70: +0 qword, +8 movsd, +16 word 0x100, +24 qword, +32 byte,
  // +40 movsd, +48 qword, +56 byte
  size_t min_num_obs = 2u;                              // +0   factory: 2 (only printed in a VLOG)
  double min_parallax_thresh = 2.0 / 180 * M_PI;        // +8   factory: 0x3FA1DF46A2529D39 (2 deg)
  bool only_use_corners = false;                        // +16  factory: false (unused)
  bool use_zero_motion_detection = true;                // +17  factory: true  (unused, no motion detector)
  size_t backend_zero_motion_check_n_frames = 5;        // +24  factory: 5     (unused)
  bool use_outlier_rejection = true;                    // +32  factory: true
  double outlier_rejection_px_threshold = 2.0;          // +40  factory: 2.75
  size_t min_added_measurements = 10u;                  // +48  factory: 5
  bool skip_optimization_when_tracking_bad = false;     // +56  factory: true
  // TODO(verify): upstream refine_extrinsics / extrinsics_*_sigma were removed (struct is 64 bytes)
};

// size 56 (copied as 3 x 16 + 8 bytes in the ctor)
struct CeresBackendOptions
{
  double max_iteration_time = -1;                       // +0   factory: -1.0
  int num_iterations = 3;                               // +8   factory: 5   (dword store, c7 45 ..)
  int num_threads = 2;                                  // +12  factory: 1   (dword store)
  bool verbose = false;                                 // +16  factory: false (passed to Estimator::optimize)
  bool marginalize = true;                              // +17  factory: true
  size_t num_keyframes = 5;                             // +24  factory: 8   (qword store 48 c7 45 e7 08;
                                                        //      c01 sees the low dword read at +88)
  size_t num_imu_frames = 3u;                           // +32  factory: 1
  bool remove_marginalization_term_after_correction_ = false;  // +40 factory: true (unused here)
  bool recalculate_imu_terms_after_loop = false;        // +41  factory: true (unused here)
  size_t remove_fixation_min_num_fixed_landmarks_ = 10u;// +48  factory: 10 -> backend_.min_num_3d_points_for_fixation_
};

class CeresBackendInterface
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<CeresBackendInterface> Ptr;

  // 0x180008CD0 (outside c01): CeresBackendInterface(options, optimizer_options, camera_bundle)
  CeresBackendInterface(const CeresBackendInterfaceOptions& options,
                        const CeresBackendOptions& optimizer_options,
                        const CameraBundlePtr& camera_bundle);
  // 0x180009950 (outside c01)
  ~CeresBackendInterface();

  // 0x180013480
  void loadMapFromBundleAdjustment(const FrameBundlePtr& new_frames,
                                   const FrameBundlePtr& last_frames,
                                   const std::shared_ptr<Map>& map,
                                   bool& have_motion_prior,
                                   const Eigen::Vector3d& gravity_prior,
                                   bool add_states_flag);
  // 0x180011700
  void bundleAdjustment(const FrameBundlePtr& frame_bundle, int unused_stage,
                        uint8_t tracking_mode);
  // 0x180013680 (pimax: body of upstream optimizationLoop + frontend update, synchronous)
  void optimize(const FrameBundlePtr& frame_bundle, int frame_count, uint8_t tracking_mode);
  // 0x1800149F0
  void reset();
  // 0x180012B20 (name unknown; always called right after reset())
  void clearBackend();
  // 0x180015870
  void setCorrectionInWorld(const Transformation& w_T_correction);
  // 0x180015A80 (pimax: by const reference -- the callee never releases a parameter copy)
  void setImu(const std::shared_ptr<ImuProcessor>& imu_handler);
  // 0x180015DC0
  void setPerformanceMonitor(const std::string& trace_dir);

  CeresBackendInterfaceOptions options_;                // +0    (64)
  CeresBackendOptions optimizer_options_;               // +64   (56)

public:  // Pimax: the frontend reads +96/+120/+160/+164/+976 and writes +1241 directly (c07/c09/c10)
  // 0x180010440
  void addLandmarksAndObservationsToBackend(const FramePtr& frame, double speed);
  // 0x180011170
  bool addStatesAndInertialMeasurementsToBackend(const FrameBundlePtr& frame_bundle,
                                                 bool imu_stationary,
                                                 const Eigen::Vector3d& gravity_prior,
                                                 bool add_states_flag);
  // 0x1800167C0
  void updateActiveKeyframes();

  // pimax-new: candidate observation collected for non-keyframe bundles in bundleAdjustment.
  // 32 bytes {FramePtr, size_t, int, int}; copied (not moved) into the vectors.
  struct ObsCandidate
  {
    FramePtr frame;      // +0
    size_t kp_idx;       // +16
    int track_id;        // +24
    int cam_id;          // +28  Frame::cam_id_ (+20)
  };

  std::deque<FramePtr> active_keyframes_;               // +120  (MSVC deque = 40 bytes incl. proxy)
  bool imu_motion_detector_stationary_ = false;         // +160  copied from FrameBundle +228
  int num_outliers_removed_ = 0;                        // +164  sum of OutlierRejection counts
  Estimator backend_;                                   // +176  (768 bytes, ends at +944)
  std::shared_ptr<ImuProcessor> imu_handler_;           // +944
  size_t no_motion_counter_ = 0;                        // +960  (never used; no dtor)  TODO(verify)
  std::unique_ptr<OutlierRejection> outlier_rejection_; // +968  (8-byte object, malloc/free => aligned new)
  BundleAdjustmentType type_ = BundleAdjustmentType::kNone;  // +976 set to kCeres (2) in the ctor body
  BundleId last_added_nframe_imu_ = -1;                 // +980
  BundleId last_added_nframe_images_ = -1;              // +984
  int64_t last_added_frame_stamp_ns_ = 0;               // +992
  bool skip_optimization_once_ = false;                 // +1000
  std::deque<FramePtr> active_frames_;                  // +1008 non-keyframes (pimax-new)
  // +1048 pimax-new: track id -> "observation count" for the current frame bundle.
  // NOTE (quirk): new entries are initialized with Frame::cam_id_ (+20), not with 1.
  std::map<int, int> obs_count_map_;                    // +1048
  std::mutex w_T_correction_mut_;                       // +1064 (80 bytes)
  Transformation w_T_correction_to_apply_;              // +1152 (16-aligned)
  bool is_w_T_valid_ = false;                           // +1216
  int num_optimizations_ = 0;                           // +1220 incremented per optimize()
  std::shared_ptr<vk::PerformanceMonitor> g_permon_backend_;  // +1224
  bool unknown_flag_1240_ = false;                      // +1240 only ever cleared  TODO(verify)
  bool imu_init_pending_ = false;                       // +1241 written by FrameProcessor (0x1800FB287)
  int correction_steps_ = 1;                            // +1244
  int correction_step_idx_ = 0;                         // +1248
  double correction_step_size_ = 0.001;                 // +1256

  friend struct CeresBackendInterfaceLayoutCheck;
};                                                      // sizeof == 1264

static_assert(sizeof(CeresBackendInterfaceOptions) == 0x40, "");
static_assert(offsetof(CeresBackendInterfaceOptions, outlier_rejection_px_threshold) == 0x28, "");
static_assert(offsetof(CeresBackendInterfaceOptions, skip_optimization_when_tracking_bad) == 0x38, "");
static_assert(sizeof(CeresBackendOptions) == 0x38, "");
static_assert(offsetof(CeresBackendOptions, num_threads) == 0x0C, "");
static_assert(offsetof(CeresBackendOptions, num_keyframes) == 0x18, "");
static_assert(offsetof(CeresBackendOptions, num_imu_frames) == 0x20, "");
static_assert(offsetof(CeresBackendOptions, recalculate_imu_terms_after_loop) == 0x29, "");
static_assert(offsetof(CeresBackendOptions, remove_fixation_min_num_fixed_landmarks_) == 0x30, "");

struct CeresBackendInterfaceLayoutCheck
{
  static_assert(sizeof(CeresBackendInterface) == 0x4F0, "");
  static_assert(offsetof(CeresBackendInterface, optimizer_options_) == 64, "");
  static_assert(offsetof(CeresBackendInterface, active_keyframes_) == 120, "");
  static_assert(offsetof(CeresBackendInterface, imu_motion_detector_stationary_) == 160, "");
  static_assert(offsetof(CeresBackendInterface, num_outliers_removed_) == 164, "");
  static_assert(offsetof(CeresBackendInterface, backend_) == 176, "");
  static_assert(offsetof(CeresBackendInterface, imu_handler_) == 944, "");
  static_assert(offsetof(CeresBackendInterface, no_motion_counter_) == 960, "");
  static_assert(offsetof(CeresBackendInterface, outlier_rejection_) == 968, "");
  static_assert(offsetof(CeresBackendInterface, type_) == 976, "");
  static_assert(offsetof(CeresBackendInterface, last_added_nframe_imu_) == 980, "");
  static_assert(offsetof(CeresBackendInterface, last_added_nframe_images_) == 984, "");
  static_assert(offsetof(CeresBackendInterface, last_added_frame_stamp_ns_) == 992, "");
  static_assert(offsetof(CeresBackendInterface, skip_optimization_once_) == 1000, "");
  static_assert(offsetof(CeresBackendInterface, active_frames_) == 1008, "");
  static_assert(offsetof(CeresBackendInterface, obs_count_map_) == 1048, "");
  static_assert(offsetof(CeresBackendInterface, w_T_correction_mut_) == 1064, "");
  static_assert(offsetof(CeresBackendInterface, w_T_correction_to_apply_) == 1152, "");
  static_assert(offsetof(CeresBackendInterface, is_w_T_valid_) == 1216, "");
  static_assert(offsetof(CeresBackendInterface, num_optimizations_) == 1220, "");
  static_assert(offsetof(CeresBackendInterface, g_permon_backend_) == 1224, "");
  static_assert(offsetof(CeresBackendInterface, unknown_flag_1240_) == 1240, "");
  static_assert(offsetof(CeresBackendInterface, imu_init_pending_) == 1241, "");
  static_assert(offsetof(CeresBackendInterface, correction_steps_) == 1244, "");
  static_assert(offsetof(CeresBackendInterface, correction_step_idx_) == 1248, "");
  static_assert(offsetof(CeresBackendInterface, correction_step_size_) == 1256, "");
};

}  // namespace totem
}  // namespace pimax
