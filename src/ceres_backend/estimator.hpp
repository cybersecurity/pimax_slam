// pimax_slam.pi.dll -- src/ceres_backend/estimator.hpp  (drafts c02 + c03 + c01 merged)
//
// Upstream: reference/rpg_svo_pro_open/svo_ceres_backend/include/svo/ceres_backend/estimator.hpp
// Namespace svo -> pimax::totem, svo::ceres_backend -> pimax::totem::ceres_backend.
//
// This header merges the layout recorded by chunk c02 (ctor 0x180022EF0 / dtor 0x1800239A0) with
// what the functions of chunk c03 (0x180027AE0..0x1800427F0) access, plus c01's evidence for
// +0xB8 (min_num_3d_points_for_fixation_, written by the CeresBackendInterface ctor 0x180008CD0).
// sizeof(Estimator) == 0x300 (CeresBackendInterface +0xB0 .. +0x3B0).
// Inline members whose out-of-line copies landed elsewhere are in estimator_impl.hpp
// (addObservation 0x180010C20, isPointInEstimator 0x180013420, setKeyframe).  Offsets are relative to the
// Estimator object (CeresBackendInterface embeds it at +0xB0 = 176, see 0x180012B20).
// "(sure)" = accessed with a clear meaning in c03.
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <ostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <ceres/ceres.h>

#include "common/imu_calibration.h"    // ImuMeasurements
#include "common/types.h"
#include "ceres_backend/ceres_map.hpp"
#include "ceres_backend/estimator_types.hpp"
#include "ceres_backend/marginalization_error.hpp"

namespace pimax {
namespace totem {

typedef std::shared_ptr<const FrameBundle> FrameBundleConstPtr;

// -------------------------------------------------------------------------------------------------
// States (sizeof 0x50) -- upstream-identical layout. removeState/findSlot are inlined into
// applyMarginalizationStrategy (0x180027AE0) and printStates (0x18002B9D0); the DEBUG_CHECK of
// addState is gone (see c02, 0x180026060).
struct States
{
  // ordered from oldest to newest.
  std::vector<BackendId> ids;      // +0x00 (Estimator+0x140)
  std::vector<bool> is_keyframe;   // +0x18 (Estimator+0x158; _Mysize at Estimator+0x170)
  std::vector<double> timestamps;  // +0x38 (Estimator+0x178)

  States() = default;

  // 0x180026060 (out-of-line inline) -- upstream-modified (DEBUG_CHECK removed)
  void addState(BackendId id, bool keyframe, double timestamp)
  {
    ids.push_back(id);
    is_keyframe.push_back(keyframe);
    timestamps.push_back(timestamp);
  }

  bool removeState(BackendId id)
  {
    auto slot = findSlot(id);
    if (slot.second)
    {
      ids.erase(ids.begin() + slot.first);
      is_keyframe.erase(is_keyframe.begin() + slot.first);   // 0x180029980 vector<bool>::erase
      timestamps.erase(timestamps.begin() + slot.first);
      return true;
    }
    return false;
  }

  std::pair<size_t, bool> findSlot(BackendId id) const
  {
    for (size_t i = 0; i < ids.size(); ++i)
    {
      if (ids[i] == id)
      {
        return std::make_pair(i, true);
      }
    }
    return std::make_pair(0, false);
  }
};

// -------------------------------------------------------------------------------------------------
// MarginalizationTiming -- upstream-identical.  names_ is the static vector at 0x18047DA40
// (filled by a .text$di initializer of estimator.obj with the six names below).
// reset() is inlined in 0x180027AE0 (map<string,double>::operator[] inlined as lower_bound +
// node insert with value 0.0); add() calls map::operator[] 0x180024750.
// TODO(verify): the reset loop does not copy the key string (upstream `for (const auto k : names_)`
// copies) -> Pimax probably wrote `const auto& k`.
struct MarginalizationTiming
{
  static std::vector<std::string> names_;   // 0x18047DA40
  std::map<std::string, double> named_timing_;

  MarginalizationTiming()
  {
    for (const auto& k : names_)
    {
      named_timing_.emplace(std::make_pair(k, 0.0));
    }
  }

  inline void reset()
  {
    for (const auto& k : names_)
    {
      named_timing_[k] = 0.0;
    }
  }

  inline double get(const std::string& name) const
  {
    return named_timing_.at(name);
  }

  inline void add(const std::string& name, const double sec)
  {
    named_timing_[name] = sec;
  }
};

// -------------------------------------------------------------------------------------------------
/// [pimax-new] relative ground-plane constraint between two NFrames (72 bytes at Estimator+0x268).
/// Filled by the frontend plane detector 0x1800F5170 ("ground-plane relative normal candidates
/// ...") through Estimator::setGroundPlaneConstraint (0x18002C530); consumed by
/// Estimator::addGroundPlaneError (0x180025930, c02) which builds a GroundPlaneError between the
/// poses of bundle_id_0 and bundle_id_1.  Field names TODO(verify).
struct GroundPlaneConstraint
{
  bool valid = false;                                        // +0x00 (+0x268)
  int bundle_id_0 = -1;                                      // +0x04 (+0x26C) previous bundle
  int bundle_id_1 = -1;                                      // +0x08 (+0x270) current bundle
  Eigen::Vector3d normal_0 = Eigen::Vector3d(0.0, 0.0, 1.0); // +0x10 (+0x278) plane normal in body 0
  Eigen::Vector3d normal_1 = Eigen::Vector3d(0.0, 0.0, 1.0); // +0x28 (+0x290) plane normal in body 1
  double sigma = 0.15;                                       // +0x40 (+0x2A8) clamped to [0.05,0.25]
};

//! The estimator class (Pimax fork of svo::Estimator).
class Estimator
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Estimator();                                                        // 0x180023430 (c02)
  explicit Estimator(std::shared_ptr<ceres_backend::Map> map_ptr);    // 0x180022EF0 (c02)
  ~Estimator();                                                       // 0x1800239A0 (c02)

  // ---- chunk c02 (declared for completeness) ----------------------------------------------------
  void addCameraBundle(const CameraBundlePtr& camera_rig);            // 0x180025810
  void addImu(const ImuParameters& imu_parameters);                   // 0x180025BE0
  bool addStates(const FrameBundleConstPtr& frame_bundle,
                 const ImuMeasurements& imu_measurements,
                 const double& timestamp,
                 Eigen::Vector3f& gravity_out,
                 bool fix_pose_from_frame,
                 bool skip_imu_propagation,
                 const Eigen::Vector3d& gravity_init,
                 bool states_from_frame);                             // 0x180026100
  bool addLandmark(const PointPtr& landmark);                         // 0x180025C80
  bool addVelocityPrior(BackendId nframe_id,
                        const Eigen::Vector3d& velocity,
                        double sigma,
                        const Eigen::Vector3d& bias_acc,
                        const Eigen::Vector3d& bias_gyr);             // 0x180027470
  void addGroundPlaneError();                                         // 0x180025930

  // ---- chunk c03 -------------------------------------------------------------------------------
  /// [pimax-new] full reset of the sliding window (called by CeresBackendInterface::reset
  /// 0x180012B20 before resetMap()).
  void reset();                                                       // 0x180029610
  /// [pimax-new] new ceres map, counters and fixation cleared.
  void resetMap();                                                    // 0x18002C350

  /// [pimax-new] ground plane constraint setter / reset (frontend 0x1800F5170, 0x18011B420).
  void setGroundPlaneConstraint(const GroundPlaneConstraint& constraint);   // 0x18002C530
  void resetGroundPlaneConstraint();                                  // 0x1800298A0
  void removeGroundPlaneErrors();                                     // 0x18002C080

  bool applyMarginalizationStrategy(size_t num_keyframes, size_t num_imu_frames,
                                    MarginalizationTiming* timing = nullptr);  // 0x180027AE0

  void printStates(BackendId pose_id, std::ostream& buffer) const;    // 0x18002B9D0

  /// Pimax: 4th argument of the call in CeresBackendInterface (0x180013680) is never read
  /// (r9 unused); upstream had (num_iter, num_threads, verbose).
  void optimize(size_t num_iter, bool verbose, bool unused = false);  // 0x18002A6D0

  void updateAllActivePoints() const;                                 // 0x18002CF00

  bool get_T_WS(BackendId id, Transformation& T_WS) const;            // 0x18002A4F0
  bool getSpeedAndBias(BackendId id, SpeedAndBias& speed_and_bias) const;   // 0x180029F30

  void setOldestFrameFixed();                                         // 0x18002C7B0

  /// [pimax-new] overwrite the pose of `id` and zero the velocity of its speed/bias block.
  /// Name TODO(verify); called by CeresBackendInterface (0x180013680) when backend+0xA0 is set.
  void setPoseEstimateAndZeroVelocity(const BackendId& id, const Transformation& T_WS);  // 0x18002CC70

  // ---- inline (estimator_impl.hpp) ---------------------------------------------------------------
  /// 0x180010C20 (out-of-line copy in ceres_backend_interface.obj).  Pimax: extra nframe_id
  /// argument; the measurement is the projection of the current landmark estimate.
  ceres::ResidualBlockId addObservation(const FramePtr& frame, const size_t keypoint_idx,
                                        const BackendId& nframe_id);
  /// 0x180013420 (out-of-line copy)
  bool isPointInEstimator(const int id) const;
  /// always inlined (bundleAdjustment)
  void setKeyframe(BackendId nframe_id, bool is_keyframe);

  // ---- inline helpers --------------------------------------------------------------------------
  inline bool hasPrior() const
  {
    return marginalization_error_ptr_ ? true : false;
  }

  void resetPrior()   // inlined in 0x180027AE0 (upstream-identical body)
  {
    marginalization_error_ptr_.reset(
          new ceres_backend::MarginalizationError(*map_ptr_.get()));
  }

  /// Pimax: CHECK removed (upstream: CHECK(it == end) << size << ", " << id).
  inline void checkAndAddToSet(const uint64_t id, std::set<uint64_t>* id_set)
  {
    id_set->insert(id);
  }

  /// Pimax: CHECK removed (upstream: CHECK(it != end) ...).  Quirk: erases end() when the id is
  /// not registered (UB in the original as well).
  inline void checkAndDeleteFromSet(const uint64_t id, std::set<uint64_t>* id_set)
  {
    auto it = std::find(id_set->begin(), id_set->end(), id);   // 0x180021F50
    id_set->erase(it);
  }

  // 0x18002BFB0 (out-of-line copy of the inline; std::set<uint64_t>::insert)
  inline void registerFixedFrame(const uint64_t param_id)
  {
    checkAndAddToSet(param_id, &fixed_frame_parameter_ids_);
  }

  inline void deRegisterFixedFrame(const uint64_t param_id)
  {
    checkAndDeleteFromSet(param_id, &fixed_frame_parameter_ids_);
  }

  inline bool hasFixedPose() const
  {
    return !fixed_frame_parameter_ids_.empty();
  }

  /// upstream estimator.hpp inline (always inlined; CeresBackendInterface::bundleAdjustment
  /// 0x180011700 reads Estimator+0x110 = landmarks_map_.size()).
  inline size_t numLandmarks() const
  {
    return landmarks_map_.size();
  }

 private:
  bool removeObservation(ceres::ResidualBlockId residual_block_id);   // 0x18002C100

  std::pair<Transformation, bool> getPoseEstimate(BackendId id) const;       // 0x180029B80
  std::pair<SpeedAndBias, bool> getSpeedAndBiasEstimate(BackendId id) const; // 0x180029F90

 public:   // TODO(verify) access specifiers; layout is what matters
  // ---- members (offset | evidence) --------------------------------------------------------------
  std::map<uint64_t, uint64_t> unknown_map_0_;              // +0x000 (c02) TODO(verify)
  std::map<uint64_t, std::vector<uint64_t, Eigen::aligned_allocator<uint64_t>>>
      unknown_map_10_;                                      // +0x010 (c02) TODO(verify)
  bool is_reinit_ = false;                                  // +0x020 (c02)
  Eigen::Matrix<double, 9, 1> reinit_speed_bias_;           // +0x028 (c02)
  Transformation reinit_T_WS_;                              // +0x070 (c02)
  double reinit_timestamp_start_;                           // +0x0B0 (c02)
  /// +0x0B8: not initialised by the Estimator ctor; written by the CeresBackendInterface ctor
  /// (0x180008CD0) from optimizer_options_.remove_fixation_min_num_fixed_landmarks_ (c01).
  size_t min_num_3d_points_for_fixation_;                   // +0x0B8 (c01)

  /// [pimax] upstream's local vectors of applyMarginalizationStrategy became members; cleared at
  /// the start of every call (0x180027AE0).  (sure)
  std::vector<BackendId> marginalize_pose_frames_;          // +0x0C0
  std::vector<BackendId> marginalize_all_but_pose_frames_;  // +0x0D8
  std::vector<BackendId> all_linearized_frames_;            // +0x0F0

  PointMap landmarks_map_;                                  // +0x108 (sure) size at +0x110
  CameraBundlePtr camera_rig_;                              // +0x118 (c02)
  std::vector<BackendId> constant_extrinsics_ids_;          // +0x128 (sure, printStates)
  States states_;                                           // +0x140 (sure)
  std::shared_ptr<ceres_backend::Map> map_ptr_;             // +0x190 (sure)
  ExtrinsicsEstimationParametersVec extrinsics_estimation_parameters_;  // +0x1A0 (c02)
  ImuParameters imu_parameters_;                            // +0x1B8 (c02)
  std::shared_ptr<ceres::LossFunction> cauchy_loss_function_ptr_;       // +0x238 (c02)
  std::shared_ptr<ceres::LossFunction> huber_loss_function_ptr_;        // +0x248 (c02)
  std::shared_ptr<ceres::LossFunction> ground_plane_loss_function_ptr_; // +0x258 (c02)
  GroundPlaneConstraint ground_plane_;                      // +0x268 (sure)
  std::vector<ceres::ResidualBlockId> ground_plane_residual_ids_;       // +0x2B0 (sure)
  std::shared_ptr<ceres_backend::MarginalizationError> marginalization_error_ptr_;  // +0x2C8 (sure)
  ceres::ResidualBlockId marginalization_residual_id_;      // +0x2D8 (sure)
  std::set<uint64_t> fixed_frame_parameter_ids_;            // +0x2E0 (sure) size at +0x2E8
  uint64_t gravity_parameter_block_id_ = static_cast<uint64_t>(-2);  // +0x2F0 (c02)
  /// [pimax-new] number of optimize() calls; landmark gating starts after 10 calls; reset to 0
  /// by resetMap().  Name TODO(verify).  (sure)
  int optimize_count_ = 0;                                  // +0x2F8
  // upstream ceres_callback_ and fixed_landmark_parameter_ids_ do not exist in Pimax
  // (no setOptimizationTimeLimit / needPoseFixation in the binary).
};

static_assert(sizeof(States) == 0x50, "States size");
static_assert(sizeof(GroundPlaneConstraint) == 0x48, "GroundPlaneConstraint size");
static_assert(offsetof(GroundPlaneConstraint, bundle_id_0) == 0x04, "");
static_assert(offsetof(GroundPlaneConstraint, bundle_id_1) == 0x08, "");
static_assert(offsetof(GroundPlaneConstraint, normal_0) == 0x10, "");
static_assert(offsetof(GroundPlaneConstraint, normal_1) == 0x28, "");
static_assert(offsetof(GroundPlaneConstraint, sigma) == 0x40, "");

static_assert(sizeof(Estimator) == 0x300, "Estimator size");
static_assert(offsetof(Estimator, unknown_map_10_) == 0x010, "");
static_assert(offsetof(Estimator, is_reinit_) == 0x020, "");
static_assert(offsetof(Estimator, reinit_speed_bias_) == 0x028, "");
static_assert(offsetof(Estimator, reinit_T_WS_) == 0x070, "");
static_assert(offsetof(Estimator, reinit_timestamp_start_) == 0x0B0, "");
static_assert(offsetof(Estimator, min_num_3d_points_for_fixation_) == 0x0B8, "");
static_assert(offsetof(Estimator, marginalize_pose_frames_) == 0x0C0, "");
static_assert(offsetof(Estimator, marginalize_all_but_pose_frames_) == 0x0D8, "");
static_assert(offsetof(Estimator, all_linearized_frames_) == 0x0F0, "");
static_assert(offsetof(Estimator, landmarks_map_) == 0x108, "");
static_assert(offsetof(Estimator, camera_rig_) == 0x118, "");
static_assert(offsetof(Estimator, constant_extrinsics_ids_) == 0x128, "");
static_assert(offsetof(Estimator, states_) == 0x140, "");
static_assert(offsetof(Estimator, map_ptr_) == 0x190, "");
static_assert(offsetof(Estimator, extrinsics_estimation_parameters_) == 0x1A0, "");
static_assert(offsetof(Estimator, imu_parameters_) == 0x1B8, "");
static_assert(offsetof(Estimator, cauchy_loss_function_ptr_) == 0x238, "");
static_assert(offsetof(Estimator, huber_loss_function_ptr_) == 0x248, "");
static_assert(offsetof(Estimator, ground_plane_loss_function_ptr_) == 0x258, "");
static_assert(offsetof(Estimator, ground_plane_) == 0x268, "");
static_assert(offsetof(Estimator, ground_plane_residual_ids_) == 0x2B0, "");
static_assert(offsetof(Estimator, marginalization_error_ptr_) == 0x2C8, "");
static_assert(offsetof(Estimator, marginalization_residual_id_) == 0x2D8, "");
static_assert(offsetof(Estimator, fixed_frame_parameter_ids_) == 0x2E0, "");
static_assert(offsetof(Estimator, gravity_parameter_block_id_) == 0x2F0, "");
static_assert(offsetof(Estimator, optimize_count_) == 0x2F8, "");

}  // namespace totem
}  // namespace pimax

#include "ceres_backend/estimator_impl.hpp"
