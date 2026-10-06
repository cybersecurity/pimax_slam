// pimax_slam.pi.dll -- src/ceres_backend/estimator.cpp
//
// __FILE__ = "E:\code_codex\pimax_slam\beta111_5a7902_dll\src\ceres_backend\estimator.cpp"
// Upstream: reference/rpg_svo_pro_open/svo_ceres_backend/src/estimator.cpp
// (namespace svo -> pimax::totem, svo::ceres_backend -> pimax::totem::ceres_backend).
//
// Merged from the phase-1 drafts
//   c00 draft/c00_globals/ceres_backend/estimator_globals.cpp  (MarginalizationTiming::names_)
//   c02 draft/c02_ceres_map/ceres_backend/estimator_c02.cpp     (0x180020440..0x180027AE0: ctors,
//       dtor, addCameraBundle, addImu, addStates, addLandmark, addGroundPlaneError, addVelocityPrior)
//   c03 draft/c03_estimator/ceres_backend/estimator.cpp         (0x180027AE0..: the rest)
// Inline members whose out-of-line copies lie in estimator.obj are in the headers:
//   States::addState 0x180026060, registerFixedFrame 0x18002BFB0 (estimator.hpp),
//   isFinite 0x180027A20, MapPoint::getTriangulationParallax 0x18002A160 (estimator_types.hpp),
//   Frame::T_world_imu 0x180024D50 / Frame::pos 0x18002B960 (common/frame.h),
//   GravityParameterBlock / General3DParameterBlock / GroundPlaneError / HomogeneousPoint LP virtuals.
//
// glog line numbers of the binary (forced with #line; they only appear in log output):
//   1168 VLOG(20) "Marginalizing following parameter blocks..."   1176 VLOG(20) "removing block with id "
//   1184/1185/1193/1194 VLOG(21) "marginalizeOut ..."
//   1454 LOG(INFO) "optimize: frame lock failed: expired"
//   1506 LOG(INFO) "residuals: frame lock failed: expired"     1509 LOG(INFO) "cant find observations item"
//   1581/1586 LOG(INFO) summary / states
// The Pimax logger (LOGE = 0x18000C2C0) is common/logger.h.
//
// Test switch: -DPIMAX_SLAM_TEST_DETERMINISTIC replaces the 25 ms Ceres wall-clock budget of
// optimize() by 1e9 s (test/make_deterministic_orig.py patches the original DLL the same way).
// The normal build is unchanged.

#include "ceres_backend/estimator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <ceres/ceres.h>
#include <glog/logging.h>
#include <vikit/timer.h>

#include "ceres_backend/ceres_map.hpp"
#include "ceres_backend/general_3d_parameter_block.hpp"
#include "ceres_backend/gravity_parameter_block.hpp"
#include "ceres_backend/ground_plane_error.hpp"
#include "ceres_backend/imu_error.hpp"
#include "ceres_backend/marginalization_error.hpp"
#include "ceres_backend/pose_error.hpp"
#include "ceres_backend/pose_parameter_block.hpp"
#include "ceres_backend/reprojection_error.hpp"
#include "ceres_backend/speed_and_bias_error.hpp"
#include "ceres_backend/speed_and_bias_parameter_block.hpp"
#include "common/frame.h"
#include "common/logger.h"   // LOGE
#include "common/point.h"

namespace pimax {
namespace totem {

using ErrorType = ceres_backend::ErrorType;

// 0x18047DA40.  Dynamic initializer 0x180001460 (.CRT$XCU #6), atexit 0x1803A4820 -- upstream-
// identical (svo_ceres_backend/src/estimator.cpp:59): six std::string temporaries are built, a
// 6-element buffer (operator new(0xC0)) is allocated and each element copy-constructed (0x180008BF0).
std::vector<std::string> MarginalizationTiming::names_ {
  "0_mag_pre_iterate", "1_mag_collection_non_pose_terms",
  "2_marg_collect_poses", "3_actual_marginalization",
  "4_marg_update_errors", "5_finish"
};

// =================================================================================================
// 0x180022EF0  -- upstream-modified: the map pointer is moved in, an extra HuberLoss(1.5) for
//                 ground-plane residuals, huber loss uses delta 0.5 (upstream 1.0), all other
//                 members take their in-class defaults (see estimator.hpp).
// Constructor if a ceres map is already available.
Estimator::Estimator(
    std::shared_ptr<ceres_backend::Map> map_ptr)
  : map_ptr_(std::move(map_ptr)),
    cauchy_loss_function_ptr_(new ceres::CauchyLoss(1)),
    huber_loss_function_ptr_(new ceres::HuberLoss(0.5)),
    ground_plane_loss_function_ptr_(new ceres::HuberLoss(1.5)),
    marginalization_residual_id_(0)
{}

// =================================================================================================
// 0x180023430  -- upstream-modified: reserves the state buffers.
// The default constructor.
Estimator::Estimator()
  : Estimator(std::make_shared<ceres_backend::Map>())
{
  states_.ids.reserve(10);
  states_.is_keyframe.reserve(10);
  states_.timestamps.reserve(10);
}

// =================================================================================================
// 0x1800239A0  -- upstream-identical (compiler-generated member destruction only)
Estimator::~Estimator()
{}

// =================================================================================================
// 0x180025810  -- upstream-modified: no extrinsics-parameter vector, no CHECKs, no loop.
//                 TODO(verify) name (caller 0x180008CD0).
void Estimator::addCameraBundle(const CameraBundlePtr& camera_rig)
{
  camera_rig_ = camera_rig;
  constant_extrinsics_ids_.resize(camera_rig_->getNumCameras());
}

// =================================================================================================
// 0x180025BE0  -- upstream-modified: a single IMU, plain copy, no return value.
//                 TODO(verify) name (caller 0x180015A80).
void Estimator::addImu(const ImuParameters& imu_parameters)
{
  imu_parameters_ = imu_parameters;
}

// =================================================================================================
// 0x180026100  -- upstream-modified (heavily):
//   * Pimax extra in/out parameters (gravity_out, three bools, gravity_init),
//   * gravity is a state (GravityParameterBlock, id gravity_parameter_block_id_ = -2) that is
//     added with the first frame and is the 5th parameter block of every ImuError,
//   * ImuError is constructed before propagation and propagation() is a member with gravity,
//   * "numUsedImuMeasurements less 1" via LOGE instead of LOG(ERROR) "numUsedImuMeasurements=",
//   * the first speed&bias block is set constant after its prior,
//   * extrinsics are always constant, use the Pimax NCamera::get_T_B_C(i) 0x1801B41E0 (+24 vector,
//     = upstream get_T_C_B(i).inverse()) and iterate
//     camera_rig_->getNumCameras(); no temporal-extrinsics branch,
//   * no reinit branch, no initPoseFromImu, no VLOG, no DEBUG_CHECKs.
// Parameter names TODO(verify) (caller CeresBackendInterface 0x180011170: fix_pose_from_frame =
// CeresBackendInterface::imu_init_pending_ (+0x4D9), skip_imu_propagation = imu_stationary).
// Quirk: with fix_pose_from_frame==true on a non-first state the ImuError stays null and is
// still passed to addResidualBlock.
bool Estimator::addStates(const FrameBundleConstPtr& frame_bundle,
                          const ImuMeasurements& imu_measurements,
                          const double& timestamp,
                          Eigen::Vector3f& gravity_out,
                          bool fix_pose_from_frame,
                          bool skip_imu_propagation,
                          const Eigen::Vector3d& gravity_init,
                          bool states_from_frame)
{
  BackendId nframe_id = createNFrameId(frame_bundle->getBundleId());   // FrameBundle+0xFC

  double last_timestamp = 0;
  Transformation T_WS;
  SpeedAndBias speed_and_bias = SpeedAndBias::Zero();
  std::shared_ptr<ceres_backend::ImuError> imu_error;

  if (fix_pose_from_frame)
  {
    T_WS = frame_bundle->at(0)->T_world_imu();
    T_WS.getRotation().normalize();
    speed_and_bias.setZero();
    speed_and_bias.head<3>() = frame_bundle->imu_vel_w_;          // FrameBundle+0x90
    speed_and_bias.segment<3>(3) = frame_bundle->imu_gyr_bias_;   // FrameBundle+0xA8
    speed_and_bias.segment<3>(6) = frame_bundle->imu_acc_bias_;   // FrameBundle+0xC0
    if (!states_.ids.empty())
    {
      last_timestamp = states_.timestamps.back();
    }
    gravity_out = gravity_init.cast<float>();
  }
  else
  {
    last_timestamp = states_.timestamps.back();   // quirk: no emptiness check
    // get the previous states
    BackendId T_WS_id = states_.ids.back();
    BackendId speed_and_bias_id = changeIdType(T_WS_id, IdType::ImuStates);
    T_WS =
        std::static_pointer_cast<ceres_backend::PoseParameterBlock>(
          map_ptr_->parameterBlockPtr(T_WS_id.asInteger()))->estimate();
    speed_and_bias =
        std::static_pointer_cast<ceres_backend::SpeedAndBiasParameterBlock>(
          map_ptr_->parameterBlockPtr(
            speed_and_bias_id.asInteger()))->estimate();

    imu_error = std::make_shared<ceres_backend::ImuError>(imu_measurements,
                                                          imu_parameters_,
                                                          last_timestamp,
                                                          timestamp,
                                                          speed_and_bias);

    // fetched once before the branch (the temporary shared_ptr is moved out by the
    // static_pointer_cast and released in each branch)
    Eigen::Vector3d gravity =
        std::static_pointer_cast<ceres_backend::GravityParameterBlock>(
          map_ptr_->parameterBlockPtr(gravity_parameter_block_id_))->estimate();
    gravity_out = gravity.cast<float>();
    if (skip_imu_propagation)
    {
      speed_and_bias.head<3>() = Eigen::Vector3d::Zero();
    }
    else
    {
      int num_used_imu_measurements =
          imu_error->propagation(imu_measurements, imu_parameters_, T_WS, speed_and_bias,
                                 last_timestamp, timestamp, gravity, nullptr, nullptr);
      if (num_used_imu_measurements < 1)
      {
        LOGE("numUsedImuMeasurements less 1\n");
        return false;
      }
    }

    if (states_from_frame)
    {
      T_WS = frame_bundle->frames_[0]->T_world_imu();            // no bounds check (sure)
      speed_and_bias.head<3>() = frame_bundle->imu_vel_w_;
      speed_and_bias.segment<3>(6) = frame_bundle->imu_acc_bias_;
      speed_and_bias.segment<3>(3) = frame_bundle->imu_gyr_bias_;
    }
  }

  // gravity state (first frame only)
  if (states_.ids.empty())
  {
    Eigen::Vector3d gravity(0.0, 0.0, imu_parameters_.g);
    if (gravity_init.norm() > 0.0)
    {
      gravity = gravity_init;
    }
    std::shared_ptr<ceres_backend::GravityParameterBlock> gravity_parameter_block =
        std::make_shared<ceres_backend::GravityParameterBlock>(gravity,
                                                               gravity_parameter_block_id_);
    map_ptr_->addParameterBlock(gravity_parameter_block,
                                ceres_backend::Map::Gravity);   // result ignored
  }

  // add the pose states
  std::shared_ptr<ceres_backend::PoseParameterBlock> pose_parameter_block =
      std::make_shared<ceres_backend::PoseParameterBlock>(T_WS, nframe_id.asInteger());
  if (!map_ptr_->addParameterBlock(pose_parameter_block,
                                   ceres_backend::Map::Pose6d))
  {
    return false;
  }
  if (fix_pose_from_frame)
  {
    map_ptr_->setParameterBlockConstant(nframe_id.asInteger());
  }
  // first state is flagged as keyframe (evaluated before the push)
  states_.addState(nframe_id, states_.ids.empty(), timestamp);

  // add IMU states
  BackendId speed_and_bias_id = changeIdType(nframe_id, IdType::ImuStates);
  std::shared_ptr<ceres_backend::SpeedAndBiasParameterBlock>
      speed_and_bias_parameter_block =
      std::make_shared<ceres_backend::SpeedAndBiasParameterBlock>(speed_and_bias,
                                                                  speed_and_bias_id.asInteger());
  if (!map_ptr_->addParameterBlock(speed_and_bias_parameter_block,
                                   ceres_backend::Map::Trivial))
  {
    return false;
  }

  // add initial prior or IMU errors
  if (states_.ids.size() == 1)
  {
    // let's add a prior
    Eigen::Matrix<double,6,6> information = Eigen::Matrix<double,6,6>::Zero();
    information(5,5) = 1.0e8;
    information(0,0) = 1.0e8;
    information(1,1) = 1.0e8;
    information(2,2) = 1.0e8;
    std::shared_ptr<ceres_backend::PoseError > pose_error =
        std::make_shared<ceres_backend::PoseError>(T_WS, information);
    map_ptr_->addResidualBlock(pose_error, nullptr, pose_parameter_block);
    registerFixedFrame(pose_parameter_block->id());

    // get these from parameter file
    const double sigma_bg = imu_parameters_.sigma_bg;
    const double sigma_ba = imu_parameters_.sigma_ba;
    std::shared_ptr<ceres_backend::SpeedAndBiasError > speed_and_bias_error =
        std::make_shared<ceres_backend::SpeedAndBiasError>(
          speed_and_bias, 1.0, sigma_bg*sigma_bg, sigma_ba*sigma_ba);
    // add to map
    map_ptr_->addResidualBlock(
          speed_and_bias_error,
          nullptr,
          map_ptr_->parameterBlockPtr(speed_and_bias_id.asInteger()));
    map_ptr_->setParameterBlockConstant(
          map_ptr_->parameterBlockPtr(speed_and_bias_id.asInteger())->id());   // [pimax-new]
  }
  else
  {
    const BackendId last_nframe_id = states_.ids[states_.ids.size() - 2];
    map_ptr_->addResidualBlock(
          imu_error,
          nullptr,
          map_ptr_->parameterBlockPtr(last_nframe_id.asInteger()),
          map_ptr_->parameterBlockPtr(
            changeIdType(last_nframe_id, IdType::ImuStates).asInteger()),
          map_ptr_->parameterBlockPtr(nframe_id.asInteger()),
          map_ptr_->parameterBlockPtr(speed_and_bias_id.asInteger()),
          map_ptr_->parameterBlockPtr(gravity_parameter_block_id_));
  }

  // Now deal with extrinsics (always constant)
  std::vector<BackendId> cur_bundle_extrinsics_ids = constant_extrinsics_ids_;
  if (states_.ids.size() == 1)
  {
    for (size_t i = 0; i < camera_rig_->getNumCameras(); ++i)
    {
      const Transformation T_S_C = camera_rig_->get_T_B_C(i);   // 0x1801B41E0
      cur_bundle_extrinsics_ids[i] = changeIdType(nframe_id, IdType::Extrinsics, i);
      std::shared_ptr<ceres_backend::PoseParameterBlock> extrinsics_parameter_block =
          std::make_shared<ceres_backend::PoseParameterBlock>(
            T_S_C, cur_bundle_extrinsics_ids[i].asInteger());
      if (!map_ptr_->addParameterBlock(extrinsics_parameter_block,
                                       ceres_backend::Map::Pose6d))
      {
        return false;
      }
      map_ptr_->setParameterBlockConstant(cur_bundle_extrinsics_ids[i].asInteger());
    }
    constant_extrinsics_ids_ = cur_bundle_extrinsics_ids;
  }

  return true;
}

// =================================================================================================
// 0x180027470  -- upstream-modified: bias arguments, speed&bias = [velocity, bias_gyr, bias_acc],
//                 bias information 1e6 on bottomRightCorner<6,6> (upstream: 0.0 on the
//                 (buggy) bottomLeftCorner<6,6>), no DEBUG_CHECK.
bool Estimator::addVelocityPrior(BackendId nframe_id,
                                 const Eigen::Vector3d& velocity,
                                 double sigma,
                                 const Eigen::Vector3d& bias_acc,
                                 const Eigen::Vector3d& bias_gyr)
{
  SpeedAndBias speed_and_bias;
  speed_and_bias.head<3>() = velocity;
  speed_and_bias.segment<3>(3) = bias_gyr;
  speed_and_bias.tail<3>() = bias_acc;   // TODO(verify) exact expression; all three copies are
                                         // inline 3-double moves in the binary
  const double speed_information = 1.0 / (sigma * sigma);
  const double bias_information = 1.0e6;
  Eigen::Matrix<double, 9, 9> information;
  information.setIdentity();
  information.topLeftCorner<3, 3>() *= speed_information;
  information.bottomRightCorner<6, 6>() *= bias_information;
  std::shared_ptr<ceres_backend::SpeedAndBiasError> prior =
      std::make_shared<ceres_backend::SpeedAndBiasError>(speed_and_bias, information);
  ceres::ResidualBlockId id =
      map_ptr_->addResidualBlock(
        prior,
        nullptr,
        map_ptr_->parameterBlockPtr(
          changeIdType(nframe_id, IdType::ImuStates).asInteger()));
  return id != nullptr;
}

// =================================================================================================
// 0x180025C80  -- upstream-modified: General3DParameterBlock (Trivial parameterization) instead
//                 of HomogeneousPointParameterBlock, float position cast to double, no set_fixed
//                 argument, no DEBUG_CHECK.
// Add a landmark.
bool Estimator::addLandmark(const PointPtr& landmark)
{
  // track id is the same as point id
  BackendId landmark_backend_id = createLandmarkId(landmark->id());

  std::shared_ptr<ceres_backend::General3DParameterBlock>
      point_parameter_block =
      std::make_shared<ceres_backend::General3DParameterBlock>(
        landmark->pos().cast<double>(), landmark_backend_id.asInteger(), true);
  if (!map_ptr_->addParameterBlock(point_parameter_block,
                                   ceres_backend::Map::Trivial))
  {
    return false;
  }

  // add landmark to map
  landmarks_map_.emplace_hint(
        landmarks_map_.end(),
        landmark_backend_id, MapPoint(landmark));
  landmark->in_ba_graph_ = true;   // Point+0x80
  return true;
}

// =================================================================================================
// 0x180025930  -- pimax-new: (re)adds the ground-plane residual between two NFrames.
void Estimator::addGroundPlaneError()
{
  removeGroundPlaneErrors();
  if (ground_plane_.valid)
  {
    const BackendId id_0 = createNFrameId(ground_plane_.bundle_id_0);
    const BackendId id_1 = createNFrameId(ground_plane_.bundle_id_1);
    if (map_ptr_->parameterBlockExists(id_0.asInteger()) &&
        map_ptr_->parameterBlockExists(id_1.asInteger()) &&
        !map_ptr_->isParameterBlockConstant(id_0.asInteger()) &&
        !map_ptr_->isParameterBlockConstant(id_1.asInteger()))
    {
      std::shared_ptr<ceres_backend::GroundPlaneError> ground_plane_error =
          std::make_shared<ceres_backend::GroundPlaneError>(ground_plane_.normal_0,
                                                            ground_plane_.normal_1,
                                                            ground_plane_.sigma);
      ceres::ResidualBlockId id = map_ptr_->addResidualBlock(
            ground_plane_error,
            ground_plane_loss_function_ptr_.get(),
            map_ptr_->parameterBlockPtr(id_0.asInteger()),
            map_ptr_->parameterBlockPtr(id_1.asInteger()));
      if (id)
      {
        ground_plane_residual_ids_.push_back(id);
      }
    }
  }
}

// =================================================================================================
// 0x180029610  -- pimax-new.  Called from CeresBackendInterface::clearBackend (0x180012B20),
//                 followed by resetMap() (0x18002C350).
void Estimator::reset()
{
  states_.ids.clear();
  states_.is_keyframe.clear();
  states_.timestamps.clear();
  states_ = States();                      // 0x1800245A0 (States move-assignment)
  landmarks_map_.clear();
  resetGroundPlaneConstraint();            // inlined (see 0x1800298A0)
  ground_plane_residual_ids_.clear();
  if (marginalization_error_ptr_)
  {
    marginalization_error_ptr_.reset();
  }
}

// =================================================================================================
// 0x1800298A0  -- pimax-new (also inlined into reset()/resetMap()/setGroundPlaneConstraint()).
//                 Callers: frontend 0x1800F5170 (fewer than 8 plane candidates), 0x18011B420.
void Estimator::resetGroundPlaneConstraint()
{
  ground_plane_ = GroundPlaneConstraint();   // valid=false, ids=-1, normals (0,0,1), sigma 0.15
}

// =================================================================================================
// 0x180029B80  -- upstream-modified: the `#ifndef NDEBUG` branch (asserts are on in this build) but
//                 without the CHECK; dynamic_pointer_cast result is dereferenced unchecked.
std::pair<Transformation, bool> Estimator::getPoseEstimate(BackendId id) const
{
  if (!map_ptr_->parameterBlockExists(id.asInteger()))
  {
    return std::make_pair(Transformation(), false);
  }
  std::shared_ptr<ceres_backend::ParameterBlock> base_ptr =
      map_ptr_->parameterBlockPtr(id.asInteger());
  if (base_ptr != nullptr)
  {
    std::shared_ptr<ceres_backend::PoseParameterBlock> block_ptr =
        std::dynamic_pointer_cast<ceres_backend::PoseParameterBlock>(base_ptr);
    return std::make_pair(block_ptr->estimate(), true);   // PoseParameterBlock::estimate 0x18008DA10
  }
  return std::make_pair(Transformation(), false);
}

// =================================================================================================
// 0x180029F30  -- upstream-identical
// Get speeds and IMU biases for a given pose ID.
bool Estimator::getSpeedAndBias(BackendId id,
                                SpeedAndBias& speed_and_bias) const
{
  bool success;
  BackendId sab_id = changeIdType(id, IdType::ImuStates);
  std::tie(speed_and_bias, success) = getSpeedAndBiasEstimate(sab_id);
  return success;
}

// =================================================================================================
// 0x180029F90  -- upstream-modified: like getPoseEstimate (dynamic_pointer_cast, no CHECK);
//                 SpeedAndBiasParameterBlock::estimate() is virtual (vtable +0x68) and returns a
//                 reference (ICF-folded with parameters()).
std::pair<SpeedAndBias, bool> Estimator::getSpeedAndBiasEstimate(BackendId id) const
{
  if (!map_ptr_->parameterBlockExists(id.asInteger()))
  {
    return std::make_pair(SpeedAndBias(), false);
  }
  std::shared_ptr<ceres_backend::ParameterBlock> base_ptr =
      map_ptr_->parameterBlockPtr(id.asInteger());
  if (base_ptr != nullptr)
  {
    std::shared_ptr<ceres_backend::SpeedAndBiasParameterBlock> block_ptr =
        std::dynamic_pointer_cast<ceres_backend::SpeedAndBiasParameterBlock>(base_ptr);
    return std::make_pair(block_ptr->estimate(), true);
  }
  return std::make_pair(SpeedAndBias(), false);
}

// =================================================================================================
// 0x18002A4F0  -- upstream-identical (DEBUG_CHECK compiled out)
bool Estimator::get_T_WS(BackendId id,
                         Transformation& T_WS) const
{
  bool success;
  std::tie(T_WS, success) = getPoseEstimate(id);
  return success;
}

// =================================================================================================
// 0x18002C100  -- upstream-modified: the landmark id is taken with Map::parameterBlockIdOfResidual
//                 (0x18001D3A0, CHECKs inside), observations is an unordered_map keyed by the
//                 residual id (single erase instead of the scan), and the result of
//                 removeResidualBlock is returned (upstream: always true).
bool Estimator::removeObservation(ceres::ResidualBlockId residual_block_id)
{
  const BackendId landmarkId(map_ptr_->parameterBlockIdOfResidual(residual_block_id, 1));
  // remove in landmarksMap
  MapPoint& map_point = landmarks_map_.at(landmarkId);
  map_point.observations.erase(reinterpret_cast<uint64_t>(residual_block_id));
  // remove residual block
  return map_ptr_->removeResidualBlock(residual_block_id);
}

/**
 * @brief Does a vector contain a certain element.  (upstream; inlined everywhere)
 */
template<class T>
bool vectorContains(const std::vector<T>& vector, const T & query)
{
  for (size_t i = 0; i < vector.size(); ++i)
  {
    if (vector[i] == query)
    {
      return true;
    }
  }
  return false;
}

// =================================================================================================
// 0x180027AE0  -- upstream-modified (heavily):
//   * early return if the window holds <= num_imu_frames states (no per-step rend check);
//   * the three frame lists are Estimator members (+0xC0/+0xD8/+0xF0), cleared on entry;
//   * keyframes beyond num_keyframes: the pose of the newest kept keyframe is fetched with
//     get_T_WS and only "get_T_WS failed" is logged on failure (the pose itself is unused);
//   * the old marginalization residual is removed AFTER the frame loop;
//   * no keep_parameter_blocks vector (MarginalizationError::marginalizeOut takes one argument);
//   * no extrinsics handling (estimate_temporal_extrinsics_ removed);
//   * landmark loop: no fixed_position branch, no CHECK(residuals.size() != 0), the first pass does
//     not test the residual type;
//   * no "Marginalizing out state with id" VLOG;
//   * VLOG(21) H_/parameter_block_infos_ sizes around marginalizeOut (two unused now() calls);
//   * fixation: `if (!hasFixedPose()) setOldestFrameFixed();` (no needPoseFixation()).
bool Estimator::applyMarginalizationStrategy(
    size_t num_keyframes, size_t num_imu_frames, MarginalizationTiming* timing)
{
  vk::Timer timer;
  if (timing)
  {
    timing->reset();
    timer.start();
  }

  if (states_.ids.size() <= num_imu_frames)
  {
    // nothing to do.
    return true;
  }

  // keep the newest numImuFrames
  std::vector<BackendId>::reverse_iterator rit_id = states_.ids.rbegin();
  std::vector<bool>::reverse_iterator rit_keyframe =
      states_.is_keyframe.rbegin();
  for (size_t k = 0; k < num_imu_frames; ++k)
  {
    ++rit_id;
    ++rit_keyframe;
  }

  // distinguish if we marginalize everything or everything but pose
  marginalize_pose_frames_.clear();
  marginalize_all_but_pose_frames_.clear();
  all_linearized_frames_.clear();
  size_t counted_keyframes = 0;
  BackendId newest_keyframe_id;   // first keyframe outside the IMU window
  // Note: rit is now pointing to the first frame not in the sliding window
  // => Either the first keyframe or the frame falling out of the sliding window.
  while (rit_id != states_.ids.rend())
  {
    // we marginalize in two cases
    //   * a frame outside the imu window but is not a keyframe
    //   * the oldest keyframe when we have enough keyframe
    if (*rit_keyframe)
    {
      if (counted_keyframes < num_keyframes)
      {
        if (counted_keyframes == 0)
        {
          newest_keyframe_id = *rit_id;
        }
        counted_keyframes++;
      }
      else
      {
        const BackendId id = *rit_id;
        Transformation T_WS_newest_keyframe;
        if (!get_T_WS(newest_keyframe_id, T_WS_newest_keyframe))
        {
          LOGE("get_T_WS failed\n");
        }
        // TODO(verify): whatever used T_WS_newest_keyframe was optimised away (result unused).
        marginalize_pose_frames_.push_back(id);
      }
    }
    else
    {
      marginalize_pose_frames_.push_back(*rit_id);
    }

    // for all the frames outside the IMU window, we only keep the pose
    marginalize_all_but_pose_frames_.push_back(*rit_id);
    all_linearized_frames_.push_back(*rit_id);
    ++rit_id;// check the next frame
    ++rit_keyframe;
  }

  // remove linear marginalizationError, if existing
  if (marginalization_error_ptr_ && marginalization_residual_id_)
  {
    bool success = map_ptr_->removeResidualBlock(marginalization_residual_id_);
    marginalization_residual_id_ = 0;
    if (!success)
      return false;
  }

  // these will keep track of what we want to marginalize out.
  std::vector<uint64_t> parameter_blocks_to_be_marginalized;

  if (!hasPrior())
  {
    resetPrior();
  }

  if (timing)
  {
    timing->add(std::string("0_mag_pre_iterate"), timer.stop());
    timer.start();
  }

  // marginalize everything but pose:
  for (size_t k = 0; k < marginalize_all_but_pose_frames_.size(); ++k)
  {
    // Add all IMU error terms.
    uint64_t speed_and_bias_id =
        changeIdType(marginalize_all_but_pose_frames_[k], IdType::ImuStates).asInteger();
    if (!map_ptr_->parameterBlockExists(speed_and_bias_id))
    {
      continue; // already marginalized.
    }
    if (map_ptr_->parameterBlockPtr(speed_and_bias_id)->fixed())
    {
      continue; // Do not remove fixed blocks.
    }
    parameter_blocks_to_be_marginalized.push_back(speed_and_bias_id);

    // Get all residuals connected to this state.
    ceres_backend::Map::ResidualBlockCollection residuals =
        map_ptr_->residuals(speed_and_bias_id);
    for (size_t r = 0; r < residuals.size(); ++r)
    {
      if (residuals[r].error_interface_ptr->typeInfo() != ErrorType::kReprojectionError)
      {
        marginalization_error_ptr_->addResidualBlock(
              residuals[r].residual_block_id);
      }
    }
  }
  if (timing)
  {
    timing->add(std::string("1_mag_collection_non_pose_terms"), timer.stop());
    timer.start();
  }

  // marginalize ONLY pose now:
  // For frames whose poses are marginalized, also deal with landmarks
  for (size_t k = 0; k < marginalize_pose_frames_.size(); ++k)
  {
    // schedule removal
    parameter_blocks_to_be_marginalized.push_back(
          marginalize_pose_frames_[k].asInteger());

    // add remaing error terms
    ceres_backend::Map::ResidualBlockCollection residuals =
        map_ptr_->residuals(marginalize_pose_frames_[k].asInteger());

    // pose
    for (size_t r = 0; r < residuals.size(); ++r)
    {
      ErrorType cur_t = residuals[r].error_interface_ptr->typeInfo();
      if(cur_t == ErrorType::kPoseError)
      {
        // avoids linearising initial pose error
        map_ptr_->removeResidualBlock(residuals[r].residual_block_id);
        deRegisterFixedFrame(marginalize_pose_frames_[k].asInteger());
        continue;
      }

      if (cur_t != ErrorType::kReprojectionError)
      {
        // we make sure no reprojection errors are yet included.
        marginalization_error_ptr_->addResidualBlock(
              residuals[r].residual_block_id);
      }
    }

    // this is the id of the oldest frame in the sliding window
    const BackendId current_kf_id = all_linearized_frames_.at(0);
    // If the frame dropping out of the sliding window is not a keyframe, then
    // the observations are deleted. If it is, then the landmarks visible in
    // the oldest keyframe but not the newest one are marginalized.
    {
      for(PointMap::iterator pit = landmarks_map_.begin();
          pit != landmarks_map_.end();)
      {
        ceres_backend::Map::ResidualBlockCollection residuals =
            map_ptr_->residuals(pit->first.asInteger());

        // First loop: check if we can skip
        bool skip_landmark = true;
        bool visible_in_imu_window = false;
        bool just_delete = false;
        bool marginalize = true;
        bool error_term_added = false;
        size_t obs_count = 0;
        for (size_t r = 0; r < residuals.size(); ++r)
        {
          // Pimax: no typeInfo() test in this pass
          BackendId pose_id(
                map_ptr_->parameterBlockIdOfResidual(residuals[r].residual_block_id, 0));

          // if the landmark is visible inthe frame to marginalize
          if(vectorContains(marginalize_pose_frames_, pose_id))
          {
            skip_landmark = false;
          }

          // the landmark is still visible in the IMU window, we keep it
          if(pose_id >= current_kf_id)
          {
            marginalize = false;
            visible_in_imu_window = true;
          }

          if(vectorContains(all_linearized_frames_, pose_id))
          {
            obs_count++;
          }
        }

        // the landmark is not affected by the marginalization
        if(skip_landmark)
        {
          pit++;
          continue;
        }

        // Second loop: actually collect residuals to marginalize
        for (size_t r = 0; r < residuals.size(); ++r)
        {
          if (residuals[r].error_interface_ptr->typeInfo() == ErrorType::kReprojectionError)
          {
            BackendId pose_id(
                  map_ptr_->parameterBlockIdOfResidual(residuals[r].residual_block_id, 0));
            const bool is_pose_to_be_margin =
                vectorContains(marginalize_pose_frames_, pose_id);
            const bool is_pose_in_sliding_window =
                vectorContains(all_linearized_frames_, pose_id);

            if((is_pose_to_be_margin && visible_in_imu_window )||
               (!is_pose_in_sliding_window && !visible_in_imu_window) ||
               is_pose_to_be_margin)
            {
              // ok, let's ignore the observation.
              removeObservation(residuals[r].residual_block_id);
              residuals.erase(residuals.begin() + r);
              r--;
            }
            else if(!visible_in_imu_window && is_pose_in_sliding_window)
            {
              // TODO: consider only the sensible ones for marginalization
              if(obs_count < 2)
              {
                removeObservation(residuals[r].residual_block_id);
                residuals.erase(residuals.begin() + r);
                r--;
              }
              else
              {
                // add information to be considered in marginalization later.
                error_term_added = true;
                // the residual term is deleted from the map as well
                marginalization_error_ptr_->addResidualBlock(
                      residuals[r].residual_block_id, false);
              }
            }

            // check anything left
            if (residuals.size() == 0)
            {
              just_delete = true;
              marginalize = false;
            }
          }
        }

        // now we deal with parameter blocks
        if(just_delete)
        {
          map_ptr_->removeParameterBlock(pit->first.asInteger());
          pit->second.point->in_ba_graph_ = false;   // Point+0x80
          pit = landmarks_map_.erase(pit);
          continue;
        }

        if(marginalize && error_term_added)
        {
          parameter_blocks_to_be_marginalized.push_back(pit->first.asInteger());
          pit->second.point->in_ba_graph_ = false;
          pit = landmarks_map_.erase(pit);
          continue;
        }

        pit++;
      } // loop of landmark map
    }

    // update book-keeping and go to the next frame
    states_.removeState(marginalize_pose_frames_[k]);
  }
  if (timing)
  {
    timing->add(std::string("2_marg_collect_poses"), timer.stop());
    timer.start();
  }

  // now apply the actual marginalization
  if(parameter_blocks_to_be_marginalized.size() > 0)
  {
    if (FLAGS_v >= 20)
    {
      std::stringstream s;
      s << "Marginalizing following parameter blocks:\n";
      for (uint64_t id : parameter_blocks_to_be_marginalized)
      {
        s << BackendId(id) << "\n";
      }
#line 1168
      VLOG(20) << s.str();
    }

    // clean parameter blocks --> some get lost in marginalization term during
    // loop closures
    for(std::vector<uint64_t>::iterator it =
        parameter_blocks_to_be_marginalized.begin();
        it != parameter_blocks_to_be_marginalized.end();)
    {
      if(!marginalization_error_ptr_->isInMarginalizationTerm(*it))
      {
#line 1176
        VLOG(20) << "removing block with id " << *it;
        it = parameter_blocks_to_be_marginalized.erase(it);
        continue;
      }
      ++it;
    }

#line 1184
    VLOG(21) << "marginalizeOut previous H_ size = " << marginalization_error_ptr_->H_.rows();
    VLOG(21) << "marginalizeOut previous parameter_block_infos_ size = " << marginalization_error_ptr_->parameter_block_infos_.size();
    // TODO(verify): two Clock::now() calls (0x180014730) bracket marginalizeOut; their values are
    // not used in the binary (timing code that was compiled out).
    auto t_marginalize_start = std::chrono::high_resolution_clock::now();
    marginalization_error_ptr_->marginalizeOut(parameter_blocks_to_be_marginalized);
    auto t_marginalize_end = std::chrono::high_resolution_clock::now();
    (void)t_marginalize_start;
    (void)t_marginalize_end;
#line 1193
    VLOG(21) << "marginalizeOut after H_ size = " << marginalization_error_ptr_->H_.rows();
    VLOG(21) << "marginalizeOut after parameter_block_infos_ size = " << marginalization_error_ptr_->parameter_block_infos_.size();
  }
  if (timing)
  {
    timing->add(std::string("3_actual_marginalization"), timer.stop());
    timer.start();
  }

  // update error computation
  if(parameter_blocks_to_be_marginalized.size() > 0)
  {
    marginalization_error_ptr_->updateErrorComputation();
  }
  if (timing)
  {
    timing->add(std::string("4_marg_update_errors"), timer.stop());
    timer.start();
  }

  // add the marginalization term again
  if(marginalization_error_ptr_->num_residuals()==0)
  {
    marginalization_error_ptr_.reset();
  }
  if (marginalization_error_ptr_)
  {
    std::vector<std::shared_ptr<ceres_backend::ParameterBlock> > parameter_block_ptrs;
    marginalization_error_ptr_->getParameterBlockPtrs(parameter_block_ptrs);
    marginalization_residual_id_ = map_ptr_->addResidualBlock(
          marginalization_error_ptr_, nullptr, parameter_block_ptrs);
    if (!marginalization_residual_id_)
    {
      return false;
    }
  }

  if(!hasFixedPose())
  {
    // finally fix the first pose properly
    setOldestFrameFixed();
  }
  if (timing)
  {
    timing->add(std::string("5_finish"), timer.stop());
  }

  return true;
}

// =================================================================================================
// 0x18002A6D0  -- upstream-modified (heavily, Pimax landmark gating / outlier handling):
//   * options: DENSE_SCHUR, DOGLEG, max_num_iterations = num_iter, max_solver_time_in_seconds =
//     0.025 (new); minimizer_progress_to_stdout / SILENT as upstream;
//   * after 10 calls: landmarks with parallax < 1 deg, < 2 point observations or
//     point->n_consecutive_obs_ (+0x78) < 2 are set constant for this solve; with >= 50 landmarks
//     also those with > 5 BA observations, inlier ratio > 0.9 or > 20 observing bundles;
//   * addGroundPlaneError() before, removeGroundPlaneErrors() after the solve;
//   * landmark update: drop the landmark if its distance to the origin jumped by > 1 m and an
//     observing frame is still in the window; per-observation chi2 (> 2.5 outlier, < 1.0 inlier)
//     bookkeeping on the svo::Point; landmark dropped if < 20% of its residuals survive.
void Estimator::optimize(size_t num_iter, bool verbose, bool /*unused*/)
{
  // assemble options
  map_ptr_->options.linear_solver_type = ceres::DENSE_SCHUR;
  map_ptr_->options.trust_region_strategy_type = ceres::DOGLEG;
  map_ptr_->options.max_num_iterations = num_iter;
#ifdef PIMAX_SLAM_TEST_DETERMINISTIC
  // test build only: no wall-clock budget (deterministic A/B runs)
  map_ptr_->options.max_solver_time_in_seconds = 1e9;
#else
  map_ptr_->options.max_solver_time_in_seconds = 0.025;   // 0x18002A763: 0x3F9999999999999A
#endif

  if (verbose)
  {
    map_ptr_->options.minimizer_progress_to_stdout = true;
  }
  else
  {
    map_ptr_->options.logging_type = ceres::LoggingType::SILENT;
    map_ptr_->options.minimizer_progress_to_stdout = false;
  }

  if (optimize_count_ <= 10)
  {
    ++optimize_count_;
  }
  else
  {
    for (auto& id_and_map_point : landmarks_map_)
    {
      const double parallax = id_and_map_point.second.getTriangulationParallax();
      const PointPtr& point = id_and_map_point.second.point;
      if (parallax < 0.017453292519943295 ||             // 1 deg (0x1803AF2A0)
          point->obs_.size() < 2 ||                       // Point+0x20 (unordered_map size)
          point->n_consecutive_obs_ < 2)                  // Point+0x78 int (c03 "ba_obs_frames_")
      {
        map_ptr_->setParameterBlockConstant(id_and_map_point.first.asInteger());
        id_and_map_point.second.fixed_position = true;
      }
    }

    if (landmarks_map_.size() >= 50)
    {
      for (auto& id_and_map_point : landmarks_map_)
      {
        id_and_map_point.second.parallax =
            id_and_map_point.second.getTriangulationParallax();
        const PointPtr& point = id_and_map_point.second.point;
        if (static_cast<int>(id_and_map_point.second.observations.size()) > 5 ||
            static_cast<double>(point->ba_inlier_count_) /
            static_cast<double>(point->ba_total_count_ + 1) > 0.9 ||   // 0x1803AF2C0
            static_cast<int>(point->ba_bundle_ids_.size()) > 20)
        {
          map_ptr_->setParameterBlockConstant(id_and_map_point.first.asInteger());
          id_and_map_point.second.fixed_position = true;
        }
      }
    }
  }

  addGroundPlaneError();   // 0x180025930

  // Map::setFlag 0x18001A950 -> applyFlag 0x18001A980: hand ceres the minimal Jacobians.  The
  // (empty) vector argument is not used by the callee.  Its type is a vector of 24-byte vectors
  // (the caller destroys the elements with ~vector 0x180019E70) -- TODO(verify) element type.
  std::vector<std::vector<uint64_t>> unused_groups;
  map_ptr_->setFlag(unused_groups, true);

  // call solver
  map_ptr_->solve();   // ceres::Solve 0x1801C06B0 (options +0, problem_ +0x3F8, summary +0x1F0)

  removeGroundPlaneErrors();

  // update landmarks
  ceres_backend::Map::ResidualBlockCollection residuals;
  for (PointMap::iterator it = landmarks_map_.begin(); it != landmarks_map_.end();)
  {
    if (it->second.fixed_position)
    {
      map_ptr_->setParameterBlockVariable(it->first.asInteger());
      it->second.fixed_position = false;
    }

    std::shared_ptr<ceres_backend::General3DParameterBlock> block =
        std::static_pointer_cast<ceres_backend::General3DParameterBlock>(
          map_ptr_->parameterBlockPtr(it->first.asInteger()));
    const Eigen::Vector3d new_position = block->estimate();   // virtual, vtable +0x68
    const double distance_change =
        std::fabs(new_position.norm() - it->second.hom_coordinates.head<3>().norm());
    const bool jumped = distance_change > 1.0;

    // The locked frame is released before the landmark is removed (binary order), hence the flag.
    bool remove_landmark = false;
    for (auto& obs : it->second.observations)
    {
      Transformation T_WS;
      if (!obs.second.frame.expired())
      {
        FramePtr frame = obs.second.frame.lock();
        if (jumped && get_T_WS(createNFrameId(frame->bundle_id_), T_WS))   // Frame+0x20
        {
          remove_landmark = true;
          break;
        }
      }
      else
      {
#line 1454
        LOG(INFO) << "optimize: frame lock failed: expired";
      }
    }
    if (remove_landmark)
    {
      map_ptr_->removeParameterBlock(it->first.asInteger());
      PointPtr& point = it->second.point;
      point->in_ba_graph_ = false;          // +0x80
      point->ba_inlier_count_ = 0;          // +0x70
      point->ba_total_count_ = 0;           // +0x74
      point->ba_bundle_ids_.clear();        // +0x60 std::set<int>
      it = landmarks_map_.erase(it);
      continue;
    }

    // update coordinates
    it->second.hom_coordinates.head<3>() = new_position;
    it->second.hom_coordinates[3] = 1.0;

    map_ptr_->residuals(it->first.asInteger(), residuals);   // 0x18001EA40 (clears first)
    const size_t num_residuals = residuals.size();
    for (ceres_backend::Map::ResidualBlockSpec& residual : residuals)
    {
      if (residual.error_interface_ptr->typeInfo() != ErrorType::kReprojectionError)
      {
        continue;
      }

      FramePtr frame;
      size_t keypoint_index = 0;
      bool found = false;
      auto obs_it = it->second.observations.find(
            reinterpret_cast<uint64_t>(residual.residual_block_id));
      if (obs_it != it->second.observations.end())
      {
        if (!obs_it->second.frame.expired())
        {
          frame = obs_it->second.frame.lock();
          keypoint_index = obs_it->second.keypoint_index_;
          found = true;
          if (frame->track_id_vec_(keypoint_index) == -1)   // Frame+0x2C0 (Eigen::VectorXi)
          {
            removeObservation(residual.residual_block_id);
            continue;
          }
        }
        else
        {
#line 1506
          LOG(INFO) << "residuals: frame lock failed: expired";
        }
      }
      else
      {
#line 1509
        LOG(INFO) << "cant find observations item";
      }

      std::shared_ptr<ceres_backend::ReprojectionError> reprojection_error =
          std::static_pointer_cast<ceres_backend::ReprojectionError>(
            residual.error_interface_ptr);
      // ReprojectionError vtable slot 5 (+0x28): weightedError() = const Vector2d& at +0x50, the
      // last weighted residual written by EvaluateWithMinimalJacobians (0x18000C490).
      const double chi2 = reprojection_error->weightedError().squaredNorm();
      if (frame)
      {
        if (chi2 > 2.5 && found)   // 0x1803AF2C8
        {
          removeObservation(residual.residual_block_id);
          if (frame->track_id_vec_(keypoint_index) > -1)
          {
            const PointPtr& point = frame->landmark_vec_[keypoint_index];   // Frame+0x2A8
            --point->n_succeeded_reproj_;   // +0x54
            ++point->n_failed_reproj_;      // +0x50
            if (frame->is_keyframe_)        // Frame+0xB0
            {
              point->removeObservation(frame->id_);   // 0x18009BBE0, Frame+0x10
              point->ba_inlier_count_ = 0;
              point->ba_total_count_ = 0;
              point->ba_bundle_ids_.clear();
            }
            frame->track_id_vec_(keypoint_index) = -1;
            frame->landmark_vec_[keypoint_index] = nullptr;
          }
          continue;
        }

        const PointPtr& point = frame->landmark_vec_[keypoint_index];
        point->ba_bundle_ids_.insert(frame->bundle_id_);
        ++point->ba_total_count_;
        if (chi2 < 1.0)
        {
          ++point->ba_inlier_count_;
        }
      }
    }
    residuals.clear();

    // keep the landmark only if at least 20% of its residual blocks survived (float ratio)
    const size_t num_remaining =
        map_ptr_->id_to_residual_block_multimap_.count(it->first.asInteger());   // Map+0x488
    if (static_cast<float>(num_remaining) / static_cast<float>(num_residuals) < 0.2)  // 0x1803ADDC0
    {
      map_ptr_->removeParameterBlock(it->first.asInteger());
      it->second.point->in_ba_graph_ = false;
      it = landmarks_map_.erase(it);
    }
    else
    {
      ++it;
    }
  }

  // summary output
  if (verbose)
  {
#line 1581
    LOG(INFO) << map_ptr_->summary.FullReport();
    std::stringstream s;
    for (const auto& id : states_.ids)
    {
      printStates(id, s);
    }
#line 1586
    LOG(INFO) << s.str();
  }
}

// =================================================================================================
// 0x18002B9D0  -- upstream-modified: estimate_temporal_extrinsics_ branch removed (always the
//                 constant extrinsics ids).
// Prints state information to buffer.
void Estimator::printStates(BackendId pose_id, std::ostream& buffer) const
{
  auto slot = states_.findSlot(pose_id);
  if (!slot.second)
  {
    buffer << "Tried to print info on pose with ID " << pose_id
           << " which is not part of the estimator." << std::endl;
    return;
  }
  buffer << "Pose. ID: " << pose_id
         << " - Keyframe: " << (states_.is_keyframe[slot.first] ? "yes" : "no")
      << " - Timestamp: " << states_.timestamps[slot.first]
      << ":\n";
  buffer << getPoseEstimate(pose_id).first << "\n";   // operator<< 0x180020CE0

  BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);
  auto sab = getSpeedAndBiasEstimate(speed_and_bias_id);
  if (sab.second)
  {
    buffer << "Speed and Bias. ID: " << speed_and_bias_id << ":\n";
    buffer << sab.first.transpose() << "\n";            // 0x180021190
  }
  std::vector<BackendId> extrinsics_id = constant_extrinsics_ids_;
  for (size_t i = 0; i < extrinsics_id.size(); ++i)
  {
    auto extrinsics = getPoseEstimate(extrinsics_id[i]);
    if (extrinsics.second)
    {
      buffer << "Extrinsics. ID: " << extrinsics_id[i] << ":\n";
      buffer << extrinsics.first << "\n";
    }
  }
  buffer << "-------------------------------------------" << std::endl;
}

// =================================================================================================
// 0x18002C080  -- pimax-new
void Estimator::removeGroundPlaneErrors()
{
  for (ceres::ResidualBlockId id : ground_plane_residual_ids_)
  {
    map_ptr_->removeResidualBlock(id);
  }
  ground_plane_residual_ids_.clear();
}

// =================================================================================================
// 0x18002C350  -- pimax-new.  Called from CeresBackendInterface::clearBackend (0x180012B20) after
//                 reset().
void Estimator::resetMap()
{
  map_ptr_ = std::make_shared<ceres_backend::Map>();   // Map(bool flag = false) 0x180018E90
  optimize_count_ = 0;
  resetGroundPlaneConstraint();
  ground_plane_residual_ids_.clear();
  marginalization_residual_id_ = 0;
  fixed_frame_parameter_ids_.clear();
}

// =================================================================================================
// 0x18002C530  -- pimax-new.  Caller: frontend plane detector 0x1800F5170 (which already clamps
//                 its own sigma to [0.08, 0.18]).  Quirk: the norms are computed before the
//                 validity test and divided by directly (no normalize()).
void Estimator::setGroundPlaneConstraint(const GroundPlaneConstraint& constraint)
{
  ground_plane_ = constraint;
  const double norm_0 = ground_plane_.normal_0.norm();
  const double norm_1 = ground_plane_.normal_1.norm();
  if (ground_plane_.valid &&
      ground_plane_.bundle_id_0 >= 0 &&
      ground_plane_.bundle_id_1 >= 0 &&
      ground_plane_.bundle_id_0 != ground_plane_.bundle_id_1 &&
      norm_0 > 1e-12 &&
      norm_1 > 1e-12 &&
      isFinite(ground_plane_.normal_0) &&    // 0x180027A20
      isFinite(ground_plane_.normal_1))
  {
    ground_plane_.normal_0 /= norm_0;
    ground_plane_.normal_1 /= norm_1;
    ground_plane_.sigma = std::max(0.05, std::min(ground_plane_.sigma, 0.25));
  }
  else
  {
    resetGroundPlaneConstraint();
  }
}

// =================================================================================================
// 0x18002C7B0  -- upstream-identical (registerFixedFrame has no CHECK in Pimax)
void Estimator::setOldestFrameFixed()
{
  Transformation T_WS_0;
  BackendId oldest_id = states_.ids[0];
  get_T_WS(oldest_id, T_WS_0);
  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double,6,6>::Zero();
  information(0, 0) = 1.0e14;
  information(1, 1) = 1.0e14;
  information(2, 2) = 1.0e14;
  information(5, 5) = 1.0e14;
  std::shared_ptr<ceres_backend::PoseError> pose_error =
      std::make_shared<ceres_backend::PoseError>(T_WS_0, information);   // 0x18008BAC0
  map_ptr_->addResidualBlock(
        pose_error, nullptr,
        map_ptr_->parameterBlockPtr(oldest_id.asInteger()));             // 0x18001C370
  registerFixedFrame(oldest_id.asInteger());                             // 0x18002BFB0
}

// =================================================================================================
// 0x18002CC70  -- pimax-new.  Name TODO(verify) (c01: set_T_WS).  Caller CeresBackendInterface::
//                 optimize 0x180013680 while the IMU reports "stationary".
void Estimator::setPoseEstimateAndZeroVelocity(const BackendId& id, const Transformation& T_WS)
{
  std::shared_ptr<ceres_backend::PoseParameterBlock> pose_block =
      std::static_pointer_cast<ceres_backend::PoseParameterBlock>(
        map_ptr_->parameterBlockPtr(id.asInteger()));
  pose_block->setEstimate(T_WS);   // virtual, vtable +0x60

  const BackendId sab_id = changeIdType(id, IdType::ImuStates);
  if (map_ptr_->parameterBlockExists(sab_id.asInteger()))
  {
    std::shared_ptr<ceres_backend::SpeedAndBiasParameterBlock> sab_block =
        std::static_pointer_cast<ceres_backend::SpeedAndBiasParameterBlock>(
          map_ptr_->parameterBlockPtr(sab_id.asInteger()));
    SpeedAndBias sab = sab_block->estimate();   // virtual, vtable +0x68
    sab.head<3>() = Eigen::Vector3d::Zero();
    sab_block->setEstimate(sab);                // virtual, vtable +0x60
  }
}

// =================================================================================================
// 0x18002CF00  -- upstream-modified: no isLandmarkFixed() test (fixed landmarks do not exist in
//                 Pimax); only points with >= 2 observations; Point::pos_ is a Vector3f (+4).
void Estimator::updateAllActivePoints() const
{
  for(auto &id_and_map_point : landmarks_map_)
  {
    const PointPtr& point = id_and_map_point.second.point;
    if (point && point->obs_.size() >= 2)
    {
      // update coordinates
      point->pos_ = id_and_map_point.second.hom_coordinates.head<3>().cast<float>();
    }
  }
}

}  // namespace totem
}  // namespace pimax
