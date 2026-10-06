// pimax_slam.pi.dll -- src/ceres_backend/ceres_backend_interface.cpp  (drafts c01 + c00)
// E:\code_codex\pimax_slam\beta111_5a7902_dll\src\ceres_backend\ceres_backend_interface.cpp
//
// pimax::totem::CeresBackendInterface -- object #1 in link order (TU1).  The ctor (0x180008CD0)
// and dtor (0x180009950) lie in the c00 address range, all other methods in c01
// (0x180010440 .. 0x1800167C0).  See notes/c01_backend_interface.md and notes/integration_B1.md.
//
// IMPORTANT: glog records __LINE__.  The glog statements carry the binary's line numbers through
// #line directives (281, 410, 494-500, 509, 613); keep them when editing.
#include "ceres_backend/ceres_backend_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <glog/logging.h>
#include <vikit/performance_monitor.h>
#include <vikit/timer.h>

#include "common/frame.h"
#include "common/logger.h"
#include "common/point.h"
#include "frontend/imu_processor.h"

namespace pimax {
namespace totem {

// 0x180008CD0   upstream-modified: no MotionDetector, no extrinsics-estimation branch
// (addCameraBundle takes only the bundle), no soft time limit, no FLAGS_*.  All other members get
// their in-class initialisers (deques, flags, -1 ids, identity w_T_correction_to_apply_, 1/0/0.001).
CeresBackendInterface::CeresBackendInterface(
    const CeresBackendInterfaceOptions& options,
    const CeresBackendOptions& optimizer_options,
    const CameraBundlePtr& camera_bundle)
  : options_(options), optimizer_options_(optimizer_options)
{
  type_ = BundleAdjustmentType::kCeres;

  // Setup modules
  if (options_.use_outlier_rejection)
  {
    outlier_rejection_.reset(
        new OutlierRejection(options_.outlier_rejection_px_threshold));
  }

  // Cameras: extrinsics are never estimated in this build.
  backend_.addCameraBundle(camera_bundle);   // 0x180025810

  backend_.min_num_3d_points_for_fixation_ =   // Estimator +0xB8
      optimizer_options_.remove_fixation_min_num_fixed_landmarks_;
}

// 0x180009950 (called from _Ref_count_obj2<CeresBackendInterface>::_Destroy 0x180158F60).
// Body is empty: only member destructors run.
// upstream-modified: upstream calls quitThread() when thread_ != nullptr; there is no thread here.
CeresBackendInterface::~CeresBackendInterface()
{
}


// Get a motion prior for new_frames.
// 0x180013480
void CeresBackendInterface::loadMapFromBundleAdjustment(
    const FrameBundlePtr& new_frames, const FrameBundlePtr& /*last_frames*/,
    const std::shared_ptr<Map>& /*map*/, bool& have_motion_prior,
    const Eigen::Vector3d& gravity_prior, bool add_states_flag)
{
  imu_motion_detector_stationary_ = new_frames->is_static_;

  // Adding new state to backend ---------------------------------------------
  if (addStatesAndInertialMeasurementsToBackend(
          new_frames, new_frames->is_static_, gravity_prior, add_states_flag))
  {
    last_added_nframe_imu_ = new_frames->getBundleId();
    if (!imu_init_pending_)
    {
      // Obtain motion prior ---------------------------------------------------
      Transformation T_WS;
      backend_.get_T_WS(createNFrameId(new_frames->getBundleId()), T_WS);
      new_frames->set_T_W_B(T_WS);
      SpeedAndBias speed_bias;  // NOTE: not initialized (garbage if getSpeedAndBias fails)
      backend_.getSpeedAndBias(createNFrameId(new_frames->getBundleId()), speed_bias);
      // pimax: the velocity is NOT rotated by T_WS (upstream used T_WS.getRotation().rotate)
      new_frames->setIMUState(speed_bias.block<3, 1>(0, 0),
                              speed_bias.block<3, 1>(3, 0),
                              speed_bias.block<3, 1>(6, 0));
      have_motion_prior = true;
    }
  }
  else
  {
    LOGE("Could not add frame bundle  %lu to backend image timestamp  %lu\n",
         new_frames->getBundleId(), new_frames->getMinTimestampNanoseconds());
    have_motion_prior = false;
  }
}

// Add feature correspondences and landmarks to backend
// 0x180011700
void CeresBackendInterface::bundleAdjustment(const FrameBundlePtr& frame_bundle,
                                             int /*unused_stage*/,
                                             uint8_t tracking_mode)
{
  // check for case when IMU measurements could not be added.
  if (last_added_nframe_imu_ == last_added_nframe_images_)
  {
    return;
  }

  size_t max_num_observations = 80u;
  if (tracking_mode == 1 || tracking_mode == 2)
  {
    max_num_observations = 100u;
  }

  const double speed = frame_bundle->imu_vel_w_.norm();

  // Zero velocity prior ------------------------------------------------------
  if (imu_motion_detector_stationary_)
  {
    LOGD("IMU determined stationary, adding prior at time %f\n",
         frame_bundle->at(0)->getTimestampSec());
    if (backend_.addVelocityPrior(createNFrameId(frame_bundle->getBundleId()),
                                  Eigen::Vector3d::Zero(), 0.001,
                                  frame_bundle->imu_acc_bias_,
                                  frame_bundle->imu_gyr_bias_))
    {
      skip_optimization_once_ = true;
    }
    else
    {
      LOGE("Failed to add a zero velocity prior!\n");
    }
  }
  else if (speed < 0.005)
  {
    // return value ignored
    backend_.addVelocityPrior(createNFrameId(frame_bundle->getBundleId()),
                              frame_bundle->imu_vel_w_, 0.001,
                              frame_bundle->imu_acc_bias_,
                              frame_bundle->imu_gyr_bias_);
  }

  size_t num_new_observations = 0;
  std::map<int, int> last_obs_count_map = obs_count_map_;
  obs_count_map_.clear();

  const BackendId nframe_id = createNFrameId(frame_bundle->at(0)->bundleId());
  if (frame_bundle->isKeyframe())
  {
    backend_.setKeyframe(nframe_id, true);
    for (const FramePtr& frame : *frame_bundle)
    {
      active_keyframes_.push_back(frame);
      if (last_obs_count_map.size() > max_num_observations &&
          (imu_motion_detector_stationary_ ||
           (frame_bundle->at(0)->is_redundant_kf_ &&
            backend_.numLandmarks() > max_num_observations)))
      {
        // Enough landmarks: only continue observations of the last bundle.
        for (size_t kp_idx = 0; kp_idx < frame->numFeatures(); ++kp_idx)
        {
          if (frame->level_vec_[kp_idx] > 1)
          {
            continue;
          }
          if (frame->track_id_vec_[kp_idx] == -1)
          {
            continue;
          }
          if (speed < 0.02)
          {
            const Eigen::Vector2d px_lm =
                frame->w2c(frame->landmark_vec_[kp_idx]->pos_.cast<double>());
            const Eigen::Vector2d px_f = frame->f2c(
                frame->f_vec_.col(static_cast<int>(kp_idx)).cast<double>());
            if ((px_lm - px_f).norm() > 4.0)
            {
              continue;
            }
          }
          const PointPtr& point = frame->landmark_vec_[kp_idx];
          if (point && point->obs_.size() > 1)
          {
            if (backend_.isPointInEstimator(frame->track_id_vec_[kp_idx]))
            {
              if (obs_count_map_.find(frame->track_id_vec_[kp_idx]) ==
                  obs_count_map_.end())
              {
                if (last_obs_count_map.find(frame->track_id_vec_[kp_idx]) !=
                        last_obs_count_map.end() &&
                    backend_.addObservation(frame, kp_idx, nframe_id))
                {
                  ++num_new_observations;
                  obs_count_map_.insert(
                      std::make_pair(frame->track_id_vec_[kp_idx], frame->cam_index_));
                }
              }
            }
          }
        }
      }
      else
      {
        addLandmarksAndObservationsToBackend(frame, speed);
      }
    }
  }
  else
  {
    // add observations for landmarks that are still visible
    std::vector<ObsCandidate> candidates;
    for (const FramePtr& frame : *frame_bundle)
    {
      active_frames_.push_back(frame);
      for (size_t kp_idx = 0; kp_idx < frame->numFeatures(); ++kp_idx)
      {
        if (frame->level_vec_[kp_idx] > 1)
        {
          continue;
        }
        if (frame->track_id_vec_[kp_idx] == -1)
        {
          continue;
        }
        // NOTE: landmark_vec_[kp_idx] is dereferenced without a null check here.
        if ((imu_motion_detector_stationary_ ||
             (frame_bundle->at(0)->is_redundant_kf_ &&
              backend_.numLandmarks() > max_num_observations)) &&
            frame->landmark_vec_[kp_idx]->n_consecutive_obs_ < 2)
        {
          continue;
        }
        if (speed < 0.02)
        {
          const Eigen::Vector2d px_lm =
              frame->w2c(frame->landmark_vec_[kp_idx]->pos_.cast<double>());
          const Eigen::Vector2d px_f = frame->f2c(
              frame->f_vec_.col(static_cast<int>(kp_idx)).cast<double>());
          if ((px_lm - px_f).norm() > 4.0)
          {
            continue;
          }
        }
        const PointPtr& point = frame->landmark_vec_[kp_idx];
        if (point && point->obs_.size() > 1)
        {
          if (backend_.isPointInEstimator(frame->track_id_vec_[kp_idx]))
          {
            const ObsCandidate candidate{frame, kp_idx,
                                         frame->track_id_vec_[kp_idx],
                                         frame->cam_index_};
            candidates.push_back(candidate);
          }
        }
      }
    }

    // Landmarks observed in the last bundle first, then the others.
    std::vector<ObsCandidate> candidates_in_last;
    std::vector<ObsCandidate> candidates_new;
    candidates_in_last.reserve(candidates.size());
    candidates_new.reserve(candidates.size());
    for (const ObsCandidate& c : candidates)
    {
      if (last_obs_count_map.find(c.track_id) == last_obs_count_map.end())
      {
        candidates_new.push_back(c);
      }
      else
      {
        candidates_in_last.push_back(c);
      }
    }

    for (const ObsCandidate& c : candidates_in_last)
    {
      if (obs_count_map_.find(c.track_id) != obs_count_map_.end() &&
          backend_.isPointInEstimator(c.track_id))
      {
        continue;
      }
      if (backend_.addObservation(c.frame, c.kp_idx, nframe_id))
      {
        ++num_new_observations;
        obs_count_map_.insert(std::make_pair(c.track_id, c.cam_id));
      }
      if (num_new_observations > max_num_observations)
      {
        break;
      }
    }
    for (const ObsCandidate& c : candidates_new)
    {
      if (obs_count_map_.find(c.track_id) != obs_count_map_.end() &&
          backend_.isPointInEstimator(c.track_id))
      {
        continue;
      }
      if (backend_.addObservation(c.frame, c.kp_idx, nframe_id))
      {
        ++num_new_observations;
        obs_count_map_.insert(std::make_pair(c.track_id, c.cam_id));
      }
      if (num_new_observations > max_num_observations)
      {
        break;
      }
    }
  }

#line 281
  VLOG(10) << "Backend: Added " << num_new_observations
           << " continued observation in non-KF to backend.";

  if (options_.skip_optimization_when_tracking_bad)
  {
    if (frame_bundle->numLandmarksInBA() < options_.min_added_measurements)
    {
      LOGW("Too few visual measurements, skip optimization once.\n");
      skip_optimization_once_ = true;
    }
  }

  last_added_nframe_images_ = frame_bundle->getBundleId();
  last_added_frame_stamp_ns_ = frame_bundle->getMinTimestampNanoseconds();
}

// Add all landmarks and observations of frame (under certain criteria)
// 0x180010440
void CeresBackendInterface::addLandmarksAndObservationsToBackend(
    const FramePtr& frame, double speed)
{
  // Statistics.
  size_t n_skipped_points_parallax = 0;
  size_t n_skipped_few_obs = 0;        // never incremented any more
  size_t n_features_already_in_backend = 0;
  size_t n_new_observations = 0;
  size_t n_new_landmarks = 0;
  size_t n_skipped_not_corner = 0;     // never incremented any more

  // Upstream leftover: declared but unused (its zero-init survives in the binary).
  std::vector<std::pair<size_t, size_t>> kp_idx_to_n_obs_map_fixed_lm;

  const BackendId nframe_id = createNFrameId(frame->bundleId());

  // iterate through all features
  for (size_t kp_idx = 0; kp_idx < frame->numFeatures(); ++kp_idx)
  {
    const PointPtr& point = frame->landmark_vec_[kp_idx];

    // check if feature is associated to landmark
    if (frame->track_id_vec_[kp_idx] == -1)
    {
      continue;
    }
    if (frame->level_vec_[kp_idx] > 1)
    {
      continue;
    }
    // NOTE: point is not null-checked
    if (point->obs_.size() < 2 ||
        (point->obs_.size() > 2 && point->n_consecutive_obs_ < 2))
    {
      continue;
    }
    if (speed < 0.02)
    {
      const Eigen::Vector2d px_lm = frame->w2c(point->pos_.cast<double>());
      const Eigen::Vector2d px_f = frame->f2c(
          frame->f_vec_.col(static_cast<int>(kp_idx)).cast<double>());
      if ((px_lm - px_f).norm() > 4.0)
      {
        continue;
      }
    }

    // check if landmark was already in to backend, if yes just add observation.
    if (backend_.isPointInEstimator(frame->track_id_vec_[kp_idx]))
    {
      auto it = obs_count_map_.find(frame->track_id_vec_[kp_idx]);
      if (it != obs_count_map_.end() && it->second > 1)
      {
        continue;
      }
      ++n_features_already_in_backend;
      if (!backend_.addObservation(frame, kp_idx, nframe_id))
      {
#line 410
        LOG(WARNING) << "Failed to add an observation!";
        continue;
      }
      if (it == obs_count_map_.end())
      {
        obs_count_map_.insert(
            std::make_pair(frame->track_id_vec_[kp_idx], frame->cam_index_));
      }
      else
      {
        ++it->second;
      }
      ++n_new_observations;
    }
    else
    {
      if (point->getTriangulationParallax() < options_.min_parallax_thresh)
      {
        ++n_skipped_points_parallax;
        continue;
      }
      if (!backend_.addLandmark(point))
      {
        LOGE("Failed to add a landmark!\n");
        continue;
      }
      ++n_new_landmarks;
      // add an observation to the landmark
      if (!backend_.addObservation(frame, kp_idx, nframe_id))
      {
        LOGE("Failed to add an observation!\n");
        continue;
      }
      ++n_new_observations;
      if (obs_count_map_.size() > 150u)
      {
        break;
      }
    }
  }  // landmarks

#line 494
  VLOG(6) << "Backend: Added " << n_new_landmarks << " new landmarks";
  VLOG(6) << "Backend: Added " << n_new_observations << " new observations";
  VLOG(6) << "Backend: Observations already in backend: " << n_features_already_in_backend;
  VLOG(6) << "Backend: Adding points. Skipped because less than "
          << options_.min_num_obs << " observations: " << n_skipped_few_obs;
  VLOG(6) << "Backend: Adding points. Skipped because small parallax: " << n_skipped_points_parallax;
  VLOG(6) << "Backend: Adding points. Skipped because not corner: " << n_skipped_not_corner;
}
// 0x180011170
bool CeresBackendInterface::addStatesAndInertialMeasurementsToBackend(
    const FrameBundlePtr& frame_bundle, bool imu_stationary,
    const Eigen::Vector3d& gravity_prior, bool add_states_flag) {
  const double current_frame_bundle_stamp = frame_bundle->getMinTimestampSeconds();
  Eigen::Vector3f state_out = Eigen::Vector3f::Zero();
  if (!backend_.addStates(frame_bundle, frame_bundle->imu_measurements_, current_frame_bundle_stamp, state_out, imu_init_pending_, imu_stationary, gravity_prior, add_states_flag)) {
#line 509
    LOG(ERROR) << "Failed to add state. Will drop frames.";
    return false;
  }
  frame_bundle->gravity_ = state_out;  // FrameBundle +216 (Vector3f)
  return true;
}

// 0x1800149F0
void CeresBackendInterface::reset()
{
  LOGI("Backend: Reset\n");
}

// 0x180012B20 (name unknown; always called right after reset())
void CeresBackendInterface::clearBackend()
{
  backend_.reset();       // 0x180029610
  backend_.resetMap();     // 0x18002C350 (new ceres_backend::Map(false))
  last_added_nframe_images_ = -1;
  active_keyframes_.clear();
  active_frames_.clear();
  obs_count_map_.clear();
  unknown_flag_1240_ = false;
  imu_init_pending_ = false;
  num_optimizations_ = 0;
}

// pimax: synchronous replacement of upstream optimizationLoop() + the map update part of
// loadMapFromBundleAdjustment().
// 0x180013680
void CeresBackendInterface::optimize(const FrameBundlePtr& frame_bundle,
                                     int frame_count, uint8_t tracking_mode)
{
  if (active_keyframes_.size() + active_frames_.size() == 0u)
  {
    return;
  }
  ++num_optimizations_;
  const auto t_start = std::chrono::system_clock::now();

  vk::Timer timer;
  timer.start();
  MarginalizationTiming mag_timing;
  // Marginalization -------------------------------------------------------
  if (optimizer_options_.marginalize)
  {
    int num_keyframes = (frame_count == 2) ? optimizer_options_.num_keyframes - 2
                                           : optimizer_options_.num_keyframes - 4;
    if (tracking_mode == 1)
    {
      num_keyframes = 8;
    }
    if (!backend_.applyMarginalizationStrategy(
            num_keyframes, optimizer_options_.num_imu_frames + 1, &mag_timing))
    {
#line 613
      LOG(ERROR) << "Marginalization failed!";
    }
    updateActiveKeyframes();
  }
  if (g_permon_backend_)
  {
    const double marginalization_time = timer.stop();
    g_permon_backend_->log("marginalization", marginalization_time);
    LOGD("vi-estimator marginalization %f ms\n", marginalization_time * 1000.0);
    for (const auto& k : MarginalizationTiming::names_)
    {
      g_permon_backend_->log(k, mag_timing.get(k));
    }
  }

  // update fixation
  timer.start();
  if (!backend_.hasFixedPose())
  {
    backend_.setOldestFrameFixed();
  }
  if (g_permon_backend_)
  {
    g_permon_backend_->log("fixation", timer.stop());
  }

  // Optimization ----------------------------------------------------------
  timer.start();
  if (skip_optimization_once_ || imu_init_pending_)
  {
    skip_optimization_once_ = false;
  }
  else
  {
    int num_iterations = 5;
    if (num_optimizations_ > 30)
    {
      num_iterations = optimizer_options_.num_iterations;
      if (frame_count > 8)
      {
        num_iterations -= 2;
      }
    }
    backend_.optimize(num_iterations, optimizer_options_.verbose,
                      imu_motion_detector_stationary_);
  }
  if (g_permon_backend_)
  {
    const double ceres_time = timer.stop();
    g_permon_backend_->log("ceres_time", ceres_time);
    LOGD("vi-estimator ceres_time %f ms\n", ceres_time * 1000.0);
  }
  const double total_time_ms =
      std::chrono::duration<double>(std::chrono::system_clock::now() - t_start).count() *
      1000.0;
  if (g_permon_backend_)
  {
    g_permon_backend_->log("tot_time", total_time_ms / 1000.0);
    g_permon_backend_->writeToFile();
  }
  LOGI("vi-estimator total time %.2f ms\n", total_time_ms);

  // Update the newest frame bundle -------------------------------------------
  {
    Transformation T_WS;
    SpeedAndBias speed_and_bias = SpeedAndBias::Zero();
    if (!imu_motion_detector_stationary_)
    {
      for (size_t i = 0; i < frame_bundle->size(); ++i)
      {
        if (i != 0)
        {
          continue;  // only the first frame is processed
        }
        backend_.get_T_WS(createNFrameId(frame_bundle->at(i)->bundleId()), T_WS);
        T_WS.getRotation().normalize();
        const bool success = backend_.getSpeedAndBias(
            createNFrameId(frame_bundle->at(i)->bundleId()), speed_and_bias);
        if (!success)
        {
          LOGE("Could not get speed/bias for frame bundle\n");
        }
        imu_handler_->acc_bias_ = speed_and_bias.tail<3>();      // ImuProcessor +208 (no bias mutex in Pimax)
        imu_handler_->omega_bias_ = speed_and_bias.segment<3>(3); // ImuProcessor +232
        if (speed_and_bias.tail<3>().norm() < 1.0)
        {
          frame_bundle->setIMUState(speed_and_bias.head<3>(),
                                    speed_and_bias.segment<3>(3),
                                    speed_and_bias.tail<3>());
        }
        else
        {
          // implausible accelerometer bias: keep velocity, zero both biases
          frame_bundle->setIMUState(speed_and_bias.head<3>(),
                                    Eigen::Vector3d::Zero(),
                                    Eigen::Vector3d::Zero());
        }
        if (success)
        {
          frame_bundle->at(i)->set_T_w_imu(T_WS);
        }
      }
    }
    else
    {
      // stationary: pin the backend pose to the frontend pose of the first frame
      const BackendId nframe_id = createNFrameId(frame_bundle->getBundleId());
      backend_.setPoseEstimateAndZeroVelocity(nframe_id, frame_bundle->at(0)->T_world_imu());
    }
  }

  // Update keyframes and frames of the active window -------------------------
  Transformation T_WS;
  BundleId last_bundle_id = 0;  // shared by both loops
  for (size_t i = 0; i < active_keyframes_.size(); ++i)
  {
    if (i == 0 || last_bundle_id != active_keyframes_.at(i)->bundleId())
    {
      if (!backend_.get_T_WS(createNFrameId(active_keyframes_.at(i)->bundleId()), T_WS))
      {
        continue;
      }
      T_WS.getRotation().normalize();
      last_bundle_id = active_keyframes_.at(i)->bundleId();
    }
    active_keyframes_.at(i)->set_T_w_imu(T_WS);
  }
  for (size_t i = 0; i < active_frames_.size(); ++i)
  {
    if (i == 0 || last_bundle_id != active_frames_.at(i)->bundleId())
    {
      const bool success =
          backend_.get_T_WS(createNFrameId(active_frames_.at(i)->bundleId()), T_WS);
      T_WS.getRotation().normalize();
      last_bundle_id = active_frames_.at(i)->bundleId();
      if (!success)
      {
        LOGE("Could not get state for frame bundle\n");
      }
    }
    active_frames_.at(i)->set_T_w_imu(T_WS);
  }

  // Update the 3d points in map of the updated keyframes ----------------
  backend_.updateAllActivePoints();

  num_outliers_removed_ = 0;
  // Remove outliers of frame_bundle ----------------------------------------
  if (outlier_rejection_)
  {
    if (frame_bundle)
    {
      size_t n_deleted_edges = 0;
      size_t n_deleted_corners = 0;
      std::vector<int> deleted_points;
      for (const FramePtr& frame : *frame_bundle)
      {
        int n_removed = 0;
        outlier_rejection_->removeOutliers(*frame, n_deleted_edges, n_deleted_corners,
                                           deleted_points, n_removed);
        num_outliers_removed_ += n_removed;
      }
      // pimax: deleted_points are NOT removed from the backend any more
      LOGD("Outlier rejection: removed  %lu edgelets and  %lu  corners.\n",
           n_deleted_edges, n_deleted_corners);
    }
  }
}

// Performance monitor for benchmarking
// 0x180015DC0
void CeresBackendInterface::setPerformanceMonitor(const std::string& trace_dir)
{
  // Initialize Performance Monitor
  g_permon_backend_.reset(new vk::PerformanceMonitor());
  g_permon_backend_->addLog("tot_time");
  g_permon_backend_->addLog("ceres_time");
  g_permon_backend_->addLog("pre_optim_time");
  g_permon_backend_->addLog("marginalization");
  g_permon_backend_->addLog("fixation");
  g_permon_backend_->addLog("n_fixed_lm");
  for (const auto& k : MarginalizationTiming::names_)  // by reference (no string copy)
  {
    g_permon_backend_->addLog(k);
  }
  g_permon_backend_->init("trace_backend", trace_dir);
}

// Set the IMU and the parameters in backend
// 0x180015A80
void CeresBackendInterface::setImu(const std::shared_ptr<ImuProcessor>& imu_handler)
{
  imu_handler_ = imu_handler;

  ImuParameters imu_parameters;
  imu_parameters.a_max = imu_handler_->imu_calib_.saturation_accel_max;
  imu_parameters.g_max = imu_handler_->imu_calib_.saturation_omega_max;
  imu_parameters.sigma_g_c = imu_handler_->imu_calib_.gyro_noise_density;
  imu_parameters.sigma_bg = imu_handler_->imu_init_.omega_bias_sigma;
  imu_parameters.sigma_a_c = imu_handler_->imu_calib_.acc_noise_density;
  imu_parameters.sigma_ba = imu_handler_->imu_init_.acc_bias_sigma;
  imu_parameters.sigma_gw_c = imu_handler_->imu_calib_.gyro_bias_random_walk_sigma;
  imu_parameters.sigma_aw_c = imu_handler_->imu_calib_.acc_bias_random_walk_sigma;
  imu_parameters.g = imu_handler_->imu_calib_.gravity_magnitude;
  imu_parameters.a0 = imu_handler_->acc_bias_;            // ImuProcessor +208, no lock
  imu_parameters.unknown_60 = imu_handler_->omega_bias_;  // ImuProcessor +232, pimax-new field (c01: "g0")
  // rate keeps its default 1000.0; upstream rate/delay_imu_cam assignments removed
  backend_.addImu(imu_parameters);
}

// set correction transformation to be applied (in correction_steps_ increments)
// 0x180015870
void CeresBackendInterface::setCorrectionInWorld(const Transformation& w_T_correction)
{
  correction_steps_ = static_cast<int>(
      std::ceil(w_T_correction.getPosition().norm() / correction_step_size_));
  if (correction_steps_ == 0)
  {
    return;
  }
  correction_step_idx_ = 0;
  std::lock_guard<std::mutex> lock(w_T_correction_mut_);
  w_T_correction_to_apply_ = w_T_correction;
  w_T_correction_to_apply_.getPosition() =
      w_T_correction.getPosition() / static_cast<double>(correction_steps_);
  is_w_T_valid_ = true;
  for (const FramePtr& f : active_keyframes_)
  {
    // NOTE: the full correction (not the per-step one) is accumulated
    f->accumulated_w_T_correction_ = w_T_correction * f->accumulated_w_T_correction_;
  }
}

// Remove frames of the bundle that was marginalized last from both windows.
// 0x1800167C0
void CeresBackendInterface::updateActiveKeyframes()
{
  if (backend_.marginalize_pose_frames_.empty())
  {
    return;
  }
  const BundleId marginalized_bundle_id =
      backend_.marginalize_pose_frames_.front().bundleId();
  active_keyframes_.erase(
      std::remove_if(active_keyframes_.begin(), active_keyframes_.end(),
                     [&](const FramePtr& f) {
                       return f->bundleId() == marginalized_bundle_id;
                     }),
      active_keyframes_.end());
  active_frames_.erase(
      std::remove_if(active_frames_.begin(), active_frames_.end(),
                     [&](const FramePtr& f) {
                       return f->bundleId() == marginalized_bundle_id;
                     }),
      active_frames_.end());
}

}  // namespace totem
}  // namespace pimax
