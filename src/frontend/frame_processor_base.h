// pimax_slam.pi.dll -- src/frontend/frame_processor_base.h
//
// pimax::totem::FrameProcessorBase -- Pimax fork of svo/frame_handler_base.h (svo::FrameHandlerBase).
// Single reconciliation of the four partial drafts (c07 ctor layout, c08 dtor/ground plane,
// c09 addImageBundle/addFrameBundle/IMU init, c10 tail of the object) and of c14's interface
// accesses; every offset below was re-checked against the ctor 0x1800DFDF0 (all member
// initialisations are visible there).  sizeof == 4032 (FrameProcessor members start at +4032).
// See notes/integration_A2.md for the name choices (draft names that differ are listed there).
//
// vtable 0x1803B2E70 (FrameProcessorBase) / 0x1803B2A90 (FrameProcessor):
//   [0] scalar deleting dtor 0x1800EBFB0 (free(): EIGEN_MAKE_ALIGNED_OPERATOR_NEW); dtor body
//       0x1800E46A0 ("FrameProcessorBase resetBackend": resetBackend() is inlined into it)
//   [1] setFirstFrames 0x180127550   [2] processFrameBundle = 0 (_purecall)
//   [3] resetAll 0x18011B370 (inline { resetVisionFrontendCommon(); })
//   [4] resetBackend 0x18011B380      [5] setTrackingQuality 0x180128260
//   [6] getMotionPrior 0x18010A690
//   FrameProcessor only: [7] processFirstFrame, [8] processFrame, [9] makeKeyframe(int)
// Upstream needNewKf / setDetectorOccupiedCells are no longer virtual; no CallbackHost base.
#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vikit/timer.h>

#include "common/frame.h"
#include "common/imu_calibration.h"   // ImuMeasurements
#include "common/point.h"
#include "common/transformation.h"
#include "common/types.h"
#include "ceres_backend/ceres_backend_interface.hpp"   // BundleAdjustmentType
#include "frontend/global.h"
#include "frontend/visual_imu_alignment.h"             // ImageFrame (all_image_frame_ value)
#include "plane/plane.h"                                // Plane (232 B), LmkPositionMap

namespace pimax {
namespace totem {

class Reprojector;
struct ReprojectorOptions;
class PoseOptimizer;
class DepthFilter;
struct DepthFilterOptions;
struct DetectorOptions;
class AbstractInitialization;
struct InitializationOptions;
struct FeatureTrackerOptions;
class Map;
class LoopClosing;
class Mesher;
class ImuProcessor;
struct ReprojectResult;

using MapPtr = std::shared_ptr<Map>;

enum class Stage : int
{
  kPaused,          ///< Stage at the beginning and after reset
  kInitializing,    ///< Stage until the first frame with enough features is found
  kTracking,        ///< Stage when SVO is running and everything is well
  kRelocalization   ///< Stage when SVO looses tracking and it tries to relocalize
};
/// 0x18047DDB0 (default std::hash, not EnumClassHash; draft c00 frame_processor_base_globals.cpp)
extern const std::unordered_map<Stage, std::string> kStageName;

enum class TrackingQuality : int
{
  kInsufficient,
  kBad,
  kGood
};
/// 0x18047DDF0
extern const std::unordered_map<TrackingQuality, std::string> kTrackingQualityName;

enum class UpdateResult : int
{
  kDefault,
  kKeyframe,
  kFailure
};
/// 0x18047DE30
extern const std::unordered_map<UpdateResult, std::string> kUpdateResultName;

enum class KeyframeCriterion : int
{
  DOWNLOOKING,
  FORWARD
};

/// 256 bytes, copied member-wise into FrameProcessorBase+24 by the ctor.  Defaults = the inlined
/// default ctor in loadBaseOptions 0x18015D6E0 (Pimax differs from upstream: max_n_kfs 200,
/// quality_max_fts_drop 150, use_imu true, img_align_max_num_features int, est_illumination
/// gain/offset ... see below, grid_size 20).  The factory then overwrites most fields (c14 notes).
struct BaseOptions
{
  size_t max_n_kfs = 200;                              // +0
  KeyframeCriterion kfselect_criterion = KeyframeCriterion::DOWNLOOKING;  // +8 (dword)
  double kfselect_min_dist = 0.12;                     // +16
  size_t kfselect_numkfs_upper_thresh = 110;           // +24
  size_t kfselect_numkfs_lower_thresh = 80;            // +32
  double kfselect_min_dist_metric = 0.5;               // +40
  double kfselect_min_angle = 5.0;                     // +48
  int kfselect_min_num_frames_between_kfs = 2;         // +56
  double kfselect_min_disparity = -1;                  // +64
  double kfselect_backend_max_time_sec = 3.0;          // +72
  double init_map_scale = 1.0;                         // +80
  bool init_use_att_and_depth = false;                 // +88
  size_t img_align_max_level = 4;                      // +96  (Frame pyramid levels - 1)
  size_t img_align_min_level = 2;                      // +104
  bool img_align_robustification = false;              // +112
  double img_align_prior_lambda_rot = 0.0;             // +120
  double img_align_prior_lambda_trans = 0.0;           // +128
  int img_align_max_num_features = 0;                  // +136 (Pimax: int, upstream size_t)
  bool img_align_use_distortion_jacobian = false;      // +140
  bool img_align_est_illumination_gain = false;        // +141
  bool img_align_est_illumination_offset = false;      // +142
  double poseoptim_thresh = 2.0;                       // +144 (FPB+168)
  double poseoptim_prior_lambda = 0.0;                 // +152 (FPB+176)
  bool poseoptim_using_unit_sphere = false;            // +160 (FPB+184; ctor: PoseOptimizer err_type 1)
  int structure_optimization_max_pts = 20;             // +164 (FPB+188)
  std::string trace_dir = "/tmp";                      // +168 (FPB+192)
  size_t quality_min_fts = 50;                         // +200 (FPB+224)
  int quality_max_fts_drop = 150;                      // +208 (FPB+232; upstream 40)
  size_t relocalization_max_trials = 100;              // +216
  bool use_imu = true;                                 // +224 (upstream false)
  bool update_seeds_with_old_keyframes = false;        // +225
  bool use_async_reprojectors = false;                 // +226 (FPB+250)
  bool trace_statistics = false;                       // +227 (FPB+251)
  double backend_scale_stable_thresh = 0.02;           // +232
  double global_map_lc_timeout_sec_ = 3.0;             // +240 (FPB+264)
  /// [pimax-new] +248 (FPB+272): cell size of the occupancy grid (FPB+3080: (w/size)*(h/size)
  /// cells; also the last Frame ctor argument).  TODO(verify) name (c07 grid_size, c10
  /// grid_cell_size, c14 unknown_248).
  uint16_t grid_size = 20;
};

/// [pimax-new] 96 bytes at FrameProcessorBase+536; ctor 0x1800E1ED0 (zero), copy 0x18015A8A0.
/// Filled from <SFConfig><Stateinit .../> of device_calibration.xml by the factory (c14).
/// wBias/aBias are copied into ImuProcessor::omega_bias_/acc_bias_ by the resets (c10).
struct ImuParams
{
  Eigen::Vector3f wBias = Eigen::Vector3f::Zero();       // +0  "wBias"
  Eigen::Vector3f aBias = Eigen::Vector3f::Zero();       // +12 "aBias"
  Eigen::Vector3f ka = Eigen::Vector3f::Zero();          // +24 "ka"
  Eigen::Vector3f kg = Eigen::Vector3f::Zero();          // +36 "kg"
  Eigen::Vector3f na = Eigen::Vector3f::Zero();          // +48 "na"
  Eigen::Vector3f ng = Eigen::Vector3f::Zero();          // +60 "ng"
  Eigen::Vector3f unknown_72 = Eigen::Vector3f::Zero();  // +72 not written by the parser
  double delta = 0.0;                                    // +88 "delta"
};
static_assert(sizeof(ImuParams) == 96, "sizeof(ImuParams)");

/// 448 bytes: the pose/state block handed to the C API (FrameProcessorBase+784, guarded by
/// output_mutex_ +696; SLAMManager keeps three more).  Ctor 0x1800E2B10 (Isometry identities,
/// zero vectors, confidence 0, status 0, tracking_state 4, backend_static 0), copy-assignment
/// 0x180167790.  Names: c09 (writer, addImageBundle) where it had one, else c14.
struct PoseState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp = 0.0;                                           // +0   frame timestamp_ / 1e9
  Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();    // +16  (c14 T_odom_imu)
  Eigen::Isometry3d T_map_world = Eigen::Isometry3d::Identity();    // +144 (c14 T_world_odom;
                                                                    //       setIdentity 0x1800F1380)
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();               // +272 FrameBundle imu_vel_w_
  Eigen::Vector3d gyr_bias = Eigen::Vector3d::Zero();               // +296
  Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();               // +320 (zeroed if |ba| > 0.5)
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();       // +344 (SLAMManager)
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();    // +368 (SLAMManager)
  Eigen::Vector3d unknown_392 = Eigen::Vector3d::Zero();            // +392
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();                // +416 FrameBundle gravity_
  float confidence = 0.0f;                                          // +440 1.0f when tracking
  uint8_t status = 0;                                               // +444 0 tracking; 1/2/3 IMU init
  uint8_t tracking_state = 4;                                       // +445 4, 0 if bundle relocalized
  bool backend_static = false;                                      // +446 CeresBackendInterface+160
                                                                    //      (c14 "reset")
};
static_assert(sizeof(PoseState) == 448, "sizeof(PoseState)");
static_assert(offsetof(PoseState, T_map_world) == 144, "");
static_assert(offsetof(PoseState, velocity) == 272, "");
static_assert(offsetof(PoseState, gravity) == 416, "");
static_assert(offsetof(PoseState, confidence) == 440, "");
static_assert(offsetof(PoseState, status) == 444, "");

/// Running first/second moments (24 B); three at +2824 (x,y,z gyro of imu_window_).
/// mean() has an out-of-line copy 0x1801181B0; add/remove/stddev are inlined (checkImuMotion).
struct RunningStats
{
  size_t n = 0;        // +0
  double sum = 0.0;    // +8
  double sum_sq = 0.0; // +16

  void add(const double x)
  {
    ++n;
    sum += x;
    sum_sq += x * x;
  }
  void remove(const double x)
  {
    --n;
    sum -= x;
    sum_sq -= x * x;
  }
  // 0x1801181B0
  double mean() const
  {
    if (n == 0)
      return 0.0;
    return sum / static_cast<double>(n);
  }
  // sample standard deviation (inlined into checkImuMotion 0x1800FE320)
  double stddev() const
  {
    if (n <= 1)
      return 0.0;
    return std::sqrt((sum_sq - sum * sum / static_cast<double>(n)) / static_cast<double>(n - 1));
  }
};
static_assert(sizeof(RunningStats) == 24, "");

/// Result of projectMapInFrame() (returned by value, 72 bytes, zero initialised).  Consumer:
/// FrameProcessor::processFrame ("# tracking_result ...").  Field names are ours (B2): named
/// after the Reprojector::Statistics fields they are summed from (c10: ave_success_num /
/// n_stat136 / n_stat128 / ave_stat120; c07: n_points / stat24 / stat32 / stat48_ratio).
struct ReprojectResult
{
  size_t n_matches = 0;                   // +0   sum stats_.n_matches
  size_t n_trials = 0;                    // +8   sum stats_.n_trials (c07 n_points)
  float  ave_success_num = 0.f;           // +16  sum stats_.sum_lm_succeeded_reproj / n_matches
                                          //      ("ave_success_num %f")
  size_t n_seed_matches = 0;              // +24  sum stats_.n_seed_matches (caller adds it to
                                          //      n_matches)
  size_t n_lm_matches = 0;                // +32  sum stats_.n_lm_matches
  size_t unused40 = 0;                    // +40
  float  ave_lm_obs = 0.f;                // +48  sum stats_.sum_lm_obs / n_matches
  size_t unused56 = 0;                    // +56
  size_t unused64 = 0;                    // +64
};
static_assert(sizeof(ReprojectResult) == 72, "");

class FrameProcessorBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// 0x1800DFDF0.  Pimax: no IMU/backend/global-map setup; the 9th parameter (FrameProcessor
  /// forwards its std::string device_sn there, c14 make_shared<FrameProcessor>) is unused; the
  /// 10th is stored at +464 and `if (!loc_mode) loadPriorPosition(&T_prior_, &prior_position_loaded_)`.
  FrameProcessorBase(const BaseOptions& base_options,
                     const ReprojectorOptions& reprojector_options,
                     const DepthFilterOptions& depthfilter_options,
                     const DetectorOptions& detector_options,
                     const InitializationOptions& init_options,
                     const FeatureTrackerOptions& tracker_options,
                     const CameraBundlePtr& cameras,
                     const std::string& device_sn,     // unused  TODO(verify) type
                     const uint8_t& loc_mode);

  virtual ~FrameProcessorBase();                                           // [0] 0x1800E46A0
  virtual void setFirstFrames(const std::vector<FramePtr>& first_frames);  // [1] 0x180127550
  virtual UpdateResult processFrameBundle() = 0;                           // [2]
  virtual void resetAll() { resetVisionFrontendCommon(); }                 // [3] 0x18011B370
  virtual void resetBackend();                                             // [4] 0x18011B380
  virtual void setTrackingQuality(const size_t num_observations);          // [5] 0x180128260
  virtual void getMotionPrior(const bool use_velocity_in_frame);           // [6] 0x18010A690

  // ---- input (c09) ------------------------------------------------------------------------
  /// 0x1800FB650 (called by SLAMManager::ProcessLoop).  exposures/gains: the HeadsetImage
  /// uint32 fields (c14 ImageBundle); c09 drafted std::vector<int>.  TODO(verify) element type.
  bool addImageBundle(std::vector<cv::Mat>& imgs,
                      const std::vector<uint32_t>& exposure_times,
                      const std::vector<uint32_t>& gains,
                      const uint64_t& timestamp,
                      const ImuMeasurements& imu_measurements,
                      const int& frame_flag,
                      const Eigen::Quaternionf& imu_rotation,
                      const bool has_imu_rotation,
                      const double imu_rotation_timestamp);
  /// 0x1800FA6B0 ("New Frame Bundle received: %d")
  bool addFrameBundle(const FrameBundlePtr& frame_bundle,
                      const Eigen::Quaternionf& imu_rotation,
                      const bool has_imu_rotation,
                      const double imu_rotation_timestamp);
  /// 0x1800FE320
  bool checkImuMotion(const ImuMeasurements& imu_measurements);
  /// 0x180110490 (VINS-like visual-inertial initialisation, 20 KB)
  bool initializeImu(const Eigen::Quaternionf& imu_rotation,
                     const bool has_imu_rotation,
                     const double imu_rotation_timestamp);
  /// 0x180115F00 ("<trace_dir>/pimax_prior_position.txt", "Prior position loaded from %s")
  void loadPriorPosition(Transformation* T_prior, bool* loaded);
  /// 0x180125040 (writes the same file; called by the dtor)
  void savePriorPosition(const Transformation& T_world_imu);

  // ---- setters used by the interface (c10) -------------------------------------------------
  void setBundleAdjuster(const std::shared_ptr<CeresBackendInterface>& ba);   // 0x1801274A0
  void setRotationPrior(const Eigen::Quaterniond& R_imu_world);               // 0x180128210
  void setRotationIncrementPrior(const Eigen::Quaterniond& R_lastimu_newimu); // 0x1801280A0
  /// 0x1801277B0 ("C_imu_world is not orthogonal matrix.")
  void setInitialPose(const FrameBundlePtr& frame_bundle,
                      const Eigen::Quaternionf& R_imu_world_att,
                      bool use_attitude);

  // ---- tracking (c10) ----------------------------------------------------------------------
  size_t optimizePose(bool use_weighted_prior);                                       // 0x180118740
  ReprojectResult projectMapInFrame();                                                // 0x180119260
  void optimizeStructure(const FrameBundlePtr& frames, int max_n_pts, int max_iter);  // 0x180118D80
  /// 0x1801188C0 (prior-map / consecutive variant; c07 "optimizeStructureInPriorMap")
  void optimizeStructureConsecutive(const FrameBundlePtr& frames, int max_n_pts, int max_iter);
  void upgradeSeedsToFeatures(const FramePtr& frame);                                 // 0x180128FA0
  void resetVisionFrontendCommon();                                                   // 0x18011B420
  void resetVisionFrontendCommonWhenSetStart();                                       // 0x18011B8F0
  void reLocalize();                                              // 0x18011AA90 "input reLocalize"
  /// 0x18011BCA0 tracking-health watchdog ("Reset b_m_lost_r ...").  TODO(verify) name.
  void checkTrackingHealth();

  // ---- ground plane / mesh (c08) -------------------------------------------------------------
  // The binary passes raw object pointers to 0x1800F2070 / 0x1800F5170 -> reference parameters.
  void updateGroundPlane(const Frame& frame);                      // 0x1800F2070
  void refinePlanes(std::vector<Plane>& planes);                   // 0x1800F3430
  bool refitWallPlane(Plane& plane);                               // 0x1800F4190
  void mergeSimilarPlanes(std::vector<Plane>& planes);             // 0x1800EC2B0
  void selectGroundPlane();                                        // 0x1800F64A0
  void estimateGroundPlane(const FrameBundle& frame_bundle);       // 0x1800F5170
  bool shouldRemoveKeyframe(const FramePtr& frame);                // 0x1800F4BD0

  /// always inlined in the binary: sets loss_without_correction_ and
  /// LoopClosing::setRecoveryMode(recovery) (LoopClosing +1928/+1929).  Defined in
  /// frame_processor_base.cpp (needs the complete LoopClosing).
  void setRecovery(const bool recovery);
  inline bool isInRecovery() const { return loss_without_correction_; }

  // upstream FrameHandlerBase inline getters (used by SLAMManager, c14; no out-of-line copy)
  inline FrameBundlePtr getLastFrames() const { return last_frames_; }
  inline const CameraBundlePtr& getNCamera() const { return cams_; }

  // ---- data (all public: FrameProcessor, SLAMManager and the factory access them directly) ---
  // +8: MSVC pads the vfptr slot to 16 bytes (the class is 16-aligned); c07/c10's "+8 member"
  // is that padding.
  /// +16: = last_frames_->numLandmarks() or CeresBackendInterface::num_outliers_removed_ (+164);
  /// "< 20" counts low_q_num_ (c09 num_landmarks_last_, c10 num_tracked_last_)
  size_t num_tracked_last_ = 0;                          // +16
  BaseOptions options_;                                  // +24
  CameraBundlePtr cams_;                                 // +280
  FrameBundlePtr new_frames_;                            // +296
  FrameBundlePtr last_frames_;                           // +312
  FrameBundlePtr last_last_frames_;                      // +328 (Pimax)
  FrameBundlePtr last_kf_frames_;                        // +344
  cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));   // +360
  Eigen::Vector3d t_lastimu_newimu_;                     // +376 (not initialised)
  Transformation T_world_imuinit;                        // +400 identity (inline stores)
  /// +464: ctor's last argument (c14 loc_mode_, c10 slam_mode_, c09 ba_mode_, c07
  /// skip_prior_position_); 0 => load the prior position; reLocalize tests != 2;
  /// bundleAdjustment() receives it.
  uint8_t loc_mode_;                                     // +464
  std::vector<std::unique_ptr<Reprojector>> reprojectors_;   // +472
  std::unique_ptr<PoseOptimizer> pose_optimizer_;            // +496
  std::unique_ptr<DepthFilter> depth_filter_;                // +504
  std::unique_ptr<AbstractInitialization> initializer_;      // +512
  std::shared_ptr<ImuProcessor> imu_handler_;                // +520 (c14 imu_processor_)
  ImuParams imu_params_;                                     // +536 (ctor 0x1800E1ED0)
  // tracking-health watchdog counters (checkTrackingHealth; log names in quotes; c10)
  int low_q_num_ = 0;                        // +632  "low_q_num"
  int low_m_r_num_ = 0;                      // +636  "low_m_r_num" (FrameProcessor: match ratio < 0.1)
  int vel_fly_num_ = 0;                      // +640  bundle velocity > 4.5 m/s
  int dark_t_ = 0;                           // +644  "dark_t_"
  int bright_t_ = 0;                         // +648
  int feature_d_m_ = 0;                      // +652  "feature_d_m"
  int v_fast_times_ = 0;                     // +656  "v_fast_times_"
  int low_m_ba_ = 0;                         // +660  "low_m_ba"
  int low_m_f_ = 0;                          // +664  "low_m_f"
  int b_dist_ = 0;                           // +668  "b_dist"
  int low_marks_ = 0;                        // +672  "low_marks"
  int low_c_ = 0;                            // +676  "low_c"
  int e_f_f_ = 0;                            // +680  "e_f_f"
  bool p_diff_ = false;                      // +684  "p_diff" (c09: set on a big position jump)
  int c_kf_ = 0;                             // +688  "c_kf"
  std::mutex output_mutex_;                  // +696  guards output_ (C API)
  PoseState output_;                         // +784  (ctor 0x1800E2B10)
  uint8_t unknown_1232_[104];                // +1232 never initialised/destroyed  TODO(verify)
  std::mutex plane_mutex_;                   // +1336 guards +1416..+1443 (c14 ground mutex)
  bool plane_valid_ = false;                 // +1416
  Eigen::Vector3f plane_imu_pos_ = Eigen::Vector3f::Zero();   // +1420
  Eigen::Vector3f plane_center_ = Eigen::Vector3f::Zero();    // +1432
  std::mutex loc_mutex_;                     // +1448 (c14 loc mutex)
  uint8_t loc_state_ = 0;                    // +1528 (c14 loc state; c09 flag_1528_)
  std::mutex mutex_1536_;                    // +1536 TODO(verify) name/use
  bool is_destructing_ = false;              // +1616 (dtor sets it first; c08)  TODO(verify) name
  std::shared_ptr<LoopClosing> lc_;          // +1624
  int64_t unknown_1640_ = 0;                 // +1640 zeroed, never destroyed  TODO(verify)
  bool saved_gyro_bias_valid_ = false;       // +1648 (c09)
  Eigen::Vector3d saved_gyro_bias_ = Eigen::Vector3d::Zero();   // +1656
  std::vector<std::vector<PointPtr>> trash_points_;            // +1680 per camera
  std::vector<std::set<int>> track_ids_last_;                  // +1704 per camera
  std::vector<std::set<int>> track_ids_last_last_;             // +1728 per camera
  std::vector<Eigen::Vector3d> trajectory_positions_;          // +1752
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
      trajectory_poses_a_;                                     // +1776 (keyframe bundles)
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
      trajectory_poses_b_;                                     // +1800 (non-keyframe bundles)
  std::condition_variable trajectory_cv_;                      // +1824
  std::mutex trajectory_mutex_;                                // +1896
  std::ofstream ofs_1976_;                                     // +1976 (never opened)  TODO(verify)
  std::ofstream ofs_2240_;                                     // +2240
  std::ofstream ofs_2504_;                                     // +2504
  float unknown_2768_ = 0.01f;                                 // +2768 TODO(verify)
  float unknown_2772_ = 0.0075f;                               // +2772
  float unknown_2776_ = 0.006f;                                // +2776
  ImuMeasurements imu_window_;               // +2784 stationarity window (<= 500 samples)
  RunningStats gyr_stat_[3];                 // +2824
  float acc_std_threshold_ = 0.001f;         // +2896 (unused)  TODO(verify) name
  float gyro_std_threshold_ = 8e-5f;         // +2900
  /// +2912 relocalisation / loop-closure world correction (c09 T_map_world_); built with the
  /// checking kindr RotationQuaternion ctor 0x1800089C0.
  Transformation T_world_correction_{
      kindr::minimal::RotationQuaternionTemplate<double>(Eigen::Quaterniond::Identity()),
      Eigen::Vector3d::Zero()};
  Stage stage_ = Stage::kPaused;             // +2976
  bool set_reset_ = false;                   // +2980 (c09 set_start_, c14 need_reset_)
  MapPtr map_;                               // +2984 (= unique_ptr<Map>(new Map) in the ctor)
  size_t num_obs_last_ = 0;                  // +3000
  TrackingQuality tracking_quality_ = TrackingQuality::kInsufficient;   // +3008
  UpdateResult update_res_ = UpdateResult::kDefault;                    // +3012
  size_t frame_counter_ = 0;                 // +3016
  double depth_median_;                      // +3024 (not initialised)
  std::vector<std::vector<FramePtr>> overlap_kfs_;        // +3032
  std::vector<std::vector<FramePtr>> last_overlap_kfs_;   // +3056
  std::vector<bool> grid_occupancy_;         // +3080 (passed to the Frame ctor)
  /// +3112 per-bundle int from addImageBundle (c09 frame_flag_, c07 input_value_ = makeKeyframe
  /// argument, c10 exposure_level_ ">75" in the watchdog).
  int frame_flag_ = 0;
  int redundant_kf_count_ = 0;               // +3116 bundles with frames_[0]->is_redundant_kf_
  bool have_rotation_prior_ = false;         // +3120
  Eigen::Quaterniond R_imu_world_;           // +3136 (Eigen, not initialised)
  Eigen::Quaterniond R_imulast_world_;       // +3168
  bool prior_position_loaded_ = false;       // +3200 (c09 have_prior_pose_)
  Transformation T_prior_;                   // +3216 (identity, inline stores)
  bool have_prior_bias_ = false;             // +3280
  Eigen::Vector3d prior_acc_bias_ = Eigen::Vector3d::Zero();    // +3288
  Eigen::Vector3d prior_gyro_bias_ = Eigen::Vector3d::Zero();   // +3312
  bool have_motion_prior_ = false;           // +3336
  Transformation T_newimu_lastimu_prior_;    // +3344 (out-of-line default ctor 0x180008930)
  std::shared_ptr<CeresBackendInterface> bundle_adjustment_;    // +3408
  BundleAdjustmentType bundle_adjustment_type_ = BundleAdjustmentType::kNone;   // +3424
  double last_kf_time_sec_ = -1.0;           // +3432
  bool global_map_has_initial_ba_ = false;   // +3440
  bool loss_without_correction_ = false;     // +3441
  double last_good_tracking_time_sec_ = -1.0;   // +3448
  /// +3456 true until the visual-inertial init succeeded ("imu_initial true" clears it; c09
  /// imu_initial_)
  bool imu_not_initialized_ = true;
  std::map<double, FrameBundlePtr> frame_bundle_map_;        // +3464 (VI init; c10 kf_bundles_)
  std::map<double, ImageFrame> all_image_frame_;             // +3480 (VI init, node 0x1F0)
  std::vector<FrameBundlePtr> bundle_buffer_;                // +3496 (VI init, <= 50)
  std::map<double, Eigen::Quaternionf> imu_rotation_buffer_; // +3520 (<= 10; c10 attitude_history_)
  int keyframe_counter_ = 0;                 // +3536
  size_t max_image_frames_ = 20;             // +3544
  size_t window_size_ = 6;                   // +3552 (raised to backend window + 2)
  int keyframe_step_ = 3;                    // +3560
  double keyframe_min_dist_ = 0.02;          // +3568
  Eigen::Vector3f imu_init_pos_ = Eigen::Vector3f::Zero();   // +3576 (.z() = c10 ground_height_)
  std::shared_ptr<Mesher> mesher_;           // +3592 (make_shared<Mesher>() in the ctor body)
  LmkPositionMap lmk_points_;                // +3608 (c08)
  std::vector<Plane> planes_;                // +3672
  std::vector<Plane> other_planes_;          // +3696
  bool mesh_enabled_ = true;                 // +3720
  int mesh_update_count_ = 0;                // +3724 (updateGroundPlane while < 500)
  cv::Rect2f mesh_roi_;                      // +3728 {0, 0, w, h} set in the ctor body
  bool ground_valid_ = false;                // +3744
  int ground_confirmations_ = 0;             // +3748
  int ground_miss_count_ = 0;                // +3752
  Eigen::Vector3d ground_normal_{0.0, 0.0, 1.0};   // +3760
  double ground_distance_ = 0.0;             // +3784
  double ground_sigma_ = 0.08;               // +3792
  double ground_area_ = 0.0;                 // +3800
  std::vector<cv::Point2f> ground_polygon_;  // +3808
  bool ground_rel_init_ = false;             // +3832
  int ground_rel_bundle_id_ = -1;            // +3836
  Eigen::Vector3d ground_rel_normal_b_{0.0, 0.0, 1.0};   // +3840
  Eigen::Vector3d ground_rel_normal_w_{0.0, 0.0, 1.0};   // +3864
  double ground_rel_sigma_ = 0.15;           // +3888
  std::vector<int> ground_rel_ids_;          // +3896
  // relocalisation state (c10 names)
  int reLoc_times_ = 0;                      // +3920
  int max_reLoc_times_ = 300;                // +3924
  bool reset_loop_closing_pending_ = false;  // +3928 (c10 reloc_flag_3928_)
  int unknown_3932_ = 0;                     // +3932
  bool reloc_enabled_ = true;                // +3936 (c09 reloc_disabled_)
  int reloc_session_count_ = 1;              // +3940 (c09 reset_count_)
  int unknown_3944_ = -1;                    // +3944
  bool unknown_3948_ = false;                // +3948
  double reloc_success_interval_ = 2.0;      // +3952
  double last_reloc_success_time_ = -1.0;    // +3960
  double reloc_try_interval_ = 0.01;         // +3968
  double last_reloc_try_time_ = -1.0;        // +3976
  vk::Timer reloc_timer_;                    // +3984 (c09 imu_init_timer_; ctor: now(), 0, 0)
  double t1_voImuInit_ = -1.0;               // +4008 (c09 imu_init_time_sec_)
  double t2_reLocSuc_ = -1.0;                // +4016

  static void layout_check();
};

inline void FrameProcessorBase::layout_check()
{
  static_assert(sizeof(BaseOptions) == 256, "sizeof(BaseOptions)");
  static_assert(offsetof(BaseOptions, img_align_max_num_features) == 136, "");
  static_assert(offsetof(BaseOptions, poseoptim_using_unit_sphere) == 160, "");
  static_assert(offsetof(BaseOptions, structure_optimization_max_pts) == 164, "");
  static_assert(offsetof(BaseOptions, trace_dir) == 168, "");
  static_assert(offsetof(BaseOptions, quality_max_fts_drop) == 208, "");
  static_assert(offsetof(BaseOptions, use_imu) == 224, "");
  static_assert(offsetof(BaseOptions, global_map_lc_timeout_sec_) == 240, "");
  static_assert(offsetof(BaseOptions, grid_size) == 248, "");

  static_assert(sizeof(FrameProcessorBase) == 4032, "sizeof(FrameProcessorBase)");
  static_assert(offsetof(FrameProcessorBase, num_tracked_last_) == 16, "");
  static_assert(offsetof(FrameProcessorBase, options_) == 24, "");
  static_assert(offsetof(FrameProcessorBase, cams_) == 280, "");
  static_assert(offsetof(FrameProcessorBase, new_frames_) == 296, "");
  static_assert(offsetof(FrameProcessorBase, last_frames_) == 312, "");
  static_assert(offsetof(FrameProcessorBase, last_last_frames_) == 328, "");
  static_assert(offsetof(FrameProcessorBase, last_kf_frames_) == 344, "");
  static_assert(offsetof(FrameProcessorBase, clahe_) == 360, "");
  static_assert(offsetof(FrameProcessorBase, t_lastimu_newimu_) == 376, "");
  static_assert(offsetof(FrameProcessorBase, T_world_imuinit) == 400, "");
  static_assert(offsetof(FrameProcessorBase, loc_mode_) == 464, "");
  static_assert(offsetof(FrameProcessorBase, reprojectors_) == 472, "");
  static_assert(offsetof(FrameProcessorBase, pose_optimizer_) == 496, "");
  static_assert(offsetof(FrameProcessorBase, depth_filter_) == 504, "");
  static_assert(offsetof(FrameProcessorBase, initializer_) == 512, "");
  static_assert(offsetof(FrameProcessorBase, imu_handler_) == 520, "");
  static_assert(offsetof(FrameProcessorBase, imu_params_) == 536, "");
  static_assert(offsetof(FrameProcessorBase, low_q_num_) == 632, "");
  static_assert(offsetof(FrameProcessorBase, dark_t_) == 644, "");
  static_assert(offsetof(FrameProcessorBase, e_f_f_) == 680, "");
  static_assert(offsetof(FrameProcessorBase, p_diff_) == 684, "");
  static_assert(offsetof(FrameProcessorBase, c_kf_) == 688, "");
  static_assert(offsetof(FrameProcessorBase, output_mutex_) == 696, "");
  static_assert(offsetof(FrameProcessorBase, output_) == 784, "");
  static_assert(offsetof(FrameProcessorBase, unknown_1232_) == 1232, "");
  static_assert(offsetof(FrameProcessorBase, plane_mutex_) == 1336, "");
  static_assert(offsetof(FrameProcessorBase, plane_valid_) == 1416, "");
  static_assert(offsetof(FrameProcessorBase, plane_imu_pos_) == 1420, "");
  static_assert(offsetof(FrameProcessorBase, plane_center_) == 1432, "");
  static_assert(offsetof(FrameProcessorBase, loc_mutex_) == 1448, "");
  static_assert(offsetof(FrameProcessorBase, loc_state_) == 1528, "");
  static_assert(offsetof(FrameProcessorBase, mutex_1536_) == 1536, "");
  static_assert(offsetof(FrameProcessorBase, is_destructing_) == 1616, "");
  static_assert(offsetof(FrameProcessorBase, lc_) == 1624, "");
  static_assert(offsetof(FrameProcessorBase, unknown_1640_) == 1640, "");
  static_assert(offsetof(FrameProcessorBase, saved_gyro_bias_valid_) == 1648, "");
  static_assert(offsetof(FrameProcessorBase, saved_gyro_bias_) == 1656, "");
  static_assert(offsetof(FrameProcessorBase, trash_points_) == 1680, "");
  static_assert(offsetof(FrameProcessorBase, track_ids_last_) == 1704, "");
  static_assert(offsetof(FrameProcessorBase, track_ids_last_last_) == 1728, "");
  static_assert(offsetof(FrameProcessorBase, trajectory_positions_) == 1752, "");
  static_assert(offsetof(FrameProcessorBase, trajectory_poses_a_) == 1776, "");
  static_assert(offsetof(FrameProcessorBase, trajectory_poses_b_) == 1800, "");
  static_assert(offsetof(FrameProcessorBase, trajectory_cv_) == 1824, "");
  static_assert(offsetof(FrameProcessorBase, trajectory_mutex_) == 1896, "");
  static_assert(offsetof(FrameProcessorBase, ofs_1976_) == 1976, "");
  static_assert(offsetof(FrameProcessorBase, ofs_2240_) == 2240, "");
  static_assert(offsetof(FrameProcessorBase, ofs_2504_) == 2504, "");
  static_assert(offsetof(FrameProcessorBase, unknown_2768_) == 2768, "");
  static_assert(offsetof(FrameProcessorBase, imu_window_) == 2784, "");
  static_assert(offsetof(FrameProcessorBase, gyr_stat_) == 2824, "");
  static_assert(offsetof(FrameProcessorBase, acc_std_threshold_) == 2896, "");
  static_assert(offsetof(FrameProcessorBase, gyro_std_threshold_) == 2900, "");
  static_assert(offsetof(FrameProcessorBase, T_world_correction_) == 2912, "");
  static_assert(offsetof(FrameProcessorBase, stage_) == 2976, "");
  static_assert(offsetof(FrameProcessorBase, set_reset_) == 2980, "");
  static_assert(offsetof(FrameProcessorBase, map_) == 2984, "");
  static_assert(offsetof(FrameProcessorBase, num_obs_last_) == 3000, "");
  static_assert(offsetof(FrameProcessorBase, tracking_quality_) == 3008, "");
  static_assert(offsetof(FrameProcessorBase, update_res_) == 3012, "");
  static_assert(offsetof(FrameProcessorBase, frame_counter_) == 3016, "");
  static_assert(offsetof(FrameProcessorBase, depth_median_) == 3024, "");
  static_assert(offsetof(FrameProcessorBase, overlap_kfs_) == 3032, "");
  static_assert(offsetof(FrameProcessorBase, last_overlap_kfs_) == 3056, "");
  static_assert(offsetof(FrameProcessorBase, grid_occupancy_) == 3080, "");
  static_assert(offsetof(FrameProcessorBase, frame_flag_) == 3112, "");
  static_assert(offsetof(FrameProcessorBase, redundant_kf_count_) == 3116, "");
  static_assert(offsetof(FrameProcessorBase, have_rotation_prior_) == 3120, "");
  static_assert(offsetof(FrameProcessorBase, R_imu_world_) == 3136, "");
  static_assert(offsetof(FrameProcessorBase, R_imulast_world_) == 3168, "");
  static_assert(offsetof(FrameProcessorBase, prior_position_loaded_) == 3200, "");
  static_assert(offsetof(FrameProcessorBase, T_prior_) == 3216, "");
  static_assert(offsetof(FrameProcessorBase, have_prior_bias_) == 3280, "");
  static_assert(offsetof(FrameProcessorBase, prior_acc_bias_) == 3288, "");
  static_assert(offsetof(FrameProcessorBase, prior_gyro_bias_) == 3312, "");
  static_assert(offsetof(FrameProcessorBase, have_motion_prior_) == 3336, "");
  static_assert(offsetof(FrameProcessorBase, T_newimu_lastimu_prior_) == 3344, "");
  static_assert(offsetof(FrameProcessorBase, bundle_adjustment_) == 3408, "");
  static_assert(offsetof(FrameProcessorBase, bundle_adjustment_type_) == 3424, "");
  static_assert(offsetof(FrameProcessorBase, last_kf_time_sec_) == 3432, "");
  static_assert(offsetof(FrameProcessorBase, global_map_has_initial_ba_) == 3440, "");
  static_assert(offsetof(FrameProcessorBase, loss_without_correction_) == 3441, "");
  static_assert(offsetof(FrameProcessorBase, last_good_tracking_time_sec_) == 3448, "");
  static_assert(offsetof(FrameProcessorBase, imu_not_initialized_) == 3456, "");
  static_assert(offsetof(FrameProcessorBase, frame_bundle_map_) == 3464, "");
  static_assert(offsetof(FrameProcessorBase, all_image_frame_) == 3480, "");
  static_assert(offsetof(FrameProcessorBase, bundle_buffer_) == 3496, "");
  static_assert(offsetof(FrameProcessorBase, imu_rotation_buffer_) == 3520, "");
  static_assert(offsetof(FrameProcessorBase, keyframe_counter_) == 3536, "");
  static_assert(offsetof(FrameProcessorBase, max_image_frames_) == 3544, "");
  static_assert(offsetof(FrameProcessorBase, window_size_) == 3552, "");
  static_assert(offsetof(FrameProcessorBase, keyframe_step_) == 3560, "");
  static_assert(offsetof(FrameProcessorBase, keyframe_min_dist_) == 3568, "");
  static_assert(offsetof(FrameProcessorBase, imu_init_pos_) == 3576, "");
  static_assert(offsetof(FrameProcessorBase, mesher_) == 3592, "");
  static_assert(offsetof(FrameProcessorBase, lmk_points_) == 3608, "");
  static_assert(offsetof(FrameProcessorBase, planes_) == 3672, "");
  static_assert(offsetof(FrameProcessorBase, other_planes_) == 3696, "");
  static_assert(offsetof(FrameProcessorBase, mesh_enabled_) == 3720, "");
  static_assert(offsetof(FrameProcessorBase, mesh_update_count_) == 3724, "");
  static_assert(offsetof(FrameProcessorBase, mesh_roi_) == 3728, "");
  static_assert(offsetof(FrameProcessorBase, ground_valid_) == 3744, "");
  static_assert(offsetof(FrameProcessorBase, ground_confirmations_) == 3748, "");
  static_assert(offsetof(FrameProcessorBase, ground_miss_count_) == 3752, "");
  static_assert(offsetof(FrameProcessorBase, ground_normal_) == 3760, "");
  static_assert(offsetof(FrameProcessorBase, ground_distance_) == 3784, "");
  static_assert(offsetof(FrameProcessorBase, ground_sigma_) == 3792, "");
  static_assert(offsetof(FrameProcessorBase, ground_area_) == 3800, "");
  static_assert(offsetof(FrameProcessorBase, ground_polygon_) == 3808, "");
  static_assert(offsetof(FrameProcessorBase, ground_rel_init_) == 3832, "");
  static_assert(offsetof(FrameProcessorBase, ground_rel_bundle_id_) == 3836, "");
  static_assert(offsetof(FrameProcessorBase, ground_rel_normal_b_) == 3840, "");
  static_assert(offsetof(FrameProcessorBase, ground_rel_normal_w_) == 3864, "");
  static_assert(offsetof(FrameProcessorBase, ground_rel_sigma_) == 3888, "");
  static_assert(offsetof(FrameProcessorBase, ground_rel_ids_) == 3896, "");
  static_assert(offsetof(FrameProcessorBase, reLoc_times_) == 3920, "");
  static_assert(offsetof(FrameProcessorBase, max_reLoc_times_) == 3924, "");
  static_assert(offsetof(FrameProcessorBase, reset_loop_closing_pending_) == 3928, "");
  static_assert(offsetof(FrameProcessorBase, reloc_enabled_) == 3936, "");
  static_assert(offsetof(FrameProcessorBase, reloc_session_count_) == 3940, "");
  static_assert(offsetof(FrameProcessorBase, unknown_3944_) == 3944, "");
  static_assert(offsetof(FrameProcessorBase, unknown_3948_) == 3948, "");
  static_assert(offsetof(FrameProcessorBase, reloc_success_interval_) == 3952, "");
  static_assert(offsetof(FrameProcessorBase, last_reloc_try_time_) == 3976, "");
  static_assert(offsetof(FrameProcessorBase, reloc_timer_) == 3984, "");
  static_assert(offsetof(FrameProcessorBase, t1_voImuInit_) == 4008, "");
  static_assert(offsetof(FrameProcessorBase, t2_reLocSuc_) == 4016, "");
}

// ---- free helpers of frame_processor_base.cpp, called from several of its draft parts ----------
// 0x1800CB4B0 collectTrashPoints<PointPtr>: a function template ("??$" COMDAT among the
// templates of the object), defined (and only used) in frame_processor_base.cpp.
/// 0x1800FDEE0: position distance (float) and rotation angle [deg] between two poses.
void computePoseDifference(double* trans_diff, double* rot_diff_deg,
                           const Transformation& T_a, const Transformation& T_b);
/// 0x1800FE140: shoelace area of a plane polygon (c09 name; c08 called it polygonArea).
double computePolygonArea(const Plane& plane);
/// 0x180115430: slerp between (t0, q0) and (t1, q1) evaluated at t (c09 name; c10 called it
/// interpolateQuaternion).
Eigen::Quaternionf slerpByTime(const Eigen::Quaternionf& q0, const Eigen::Quaternionf& q1,
                               double t0, double t1, double t);

}  // namespace totem
}  // namespace pimax
