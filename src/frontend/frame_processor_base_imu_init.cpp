// pimax_slam.pi.dll -- src/frontend/frame_processor_base_imu_init.cpp  (phase B2; draft c09
// frame_processor_base_imu_init.cpp + c07 eulerToRotation)
//
// Part of frontend/frame_processor_base.cpp in the original tree (frame_processor_base.obj);
// split out for size only.  pimax::totem::FrameProcessorBase::initializeImu -- 0x180110490
// (20 090 bytes), pimax-new (VINS-Mono style visual-inertial initialisation built on the SVO
// frontend):
//   A. collect "image frames" (VINS all_image_frame) every few frames / every few cm, each with
//      an IntegrationBase pre-integrated from the merged IMU samples of the skipped bundles;
//   B. once `window_size_` image frames exist: visual BA (Ceres, poses only, landmarks and
//      camera extrinsics constant) over the newest window;
//   C. express the window relative to its first frame and run ImuInitializer::VisualIMUAlignment
//      (0x180157240);
//   D. visual-inertial refinement (Ceres, IMUFactor 0x1800E1950, speeds + gravity free);
//   E. sanity checks on the estimated gravity (global G, written by IMUFactor::Evaluate);
//   F. yaw alignment with the externally supplied IMU orientation (optional);
//   G. transform all frames / landmarks / keyframes by the resulting correction T_x;
//   H. re-flag keyframes of the window, I. replay the newest bundle into the backend,
//   J. reset the bookkeeping, store the init position and time.
// Returns true on success (caller then clears imu_not_initialized_).
//
// Integration (B2) renames applied to the c09 draft: imu_initial_ -> imu_not_initialized_,
// ImuProcessor::gyro_bias_ -> omega_bias_, IntegrationBase reserved0_/reserved1_/G_NORM -> G.x/y/z,
// backend->window_size_ (+96) -> optimizer_options_.num_imu_frames, flag_16_/flag_48_ ->
// use_minimal_jacobians_, Frame T_imu_cam_/T_cam_imu_ -> T_body_cam_/T_cam_body_, ImageFrame
// T_imu_aligned/T_world_imu/velocity -> T_c0_body/T_w_body/V_w (c13), ImuInitParams +
// VisualIMUAlignment(params,...) -> ImuInitializer (c13), FrameBundle T_80_ -> T_W_B_init_,
// have_prior_pose_ -> prior_position_loaded_, ba_mode_ -> loc_mode_, updateFrameBundleState ->
// CeresBackendInterface::optimize, Map::keyframe_ids_ -> sorted_keyframe_ids_, imu_init_timer_ /
// imu_init_time_sec_ -> reloc_timer_ / t1_voImuInit_, Utility::ypr2R(.., bool) 0x1800DDCF0 ->
// eulerToRotation (Rz*Rx*Ry; NOT Utility::ypr2R 0x18017EE10, see frontend/utility.h), the
// checking kindr RotationQuaternion ctors written explicitly.
//
// Members used (FrameProcessorBase offsets):
//   +3464 frame_bundle_map_ (std::map<double, FrameBundlePtr>), +3480 all_image_frame_,
//   +3496 bundle_buffer_, +3520 imu_rotation_buffer_, +3536 keyframe_counter_,
//   +3544 max_image_frames_ (20), +3552 window_size_ (6), +3560 keyframe_step_ (3),
//   +3568 keyframe_min_dist_ (0.02), +3576 imu_init_pos_, +3984 reloc_timer_, +4008 t1_voImuInit_,
//   +1648/+1656 saved_gyro_bias_valid_/saved_gyro_bias_, +3280/+3288/+3312 prior biases.
//
// Quirks kept (c09 notes): IntegrationBase noise members overwritten after construction (the
// noise matrix keeps the defaults); phase I runs once and reads window_bundles[i-1]; phase G
// tests updated_points.count(point->id()) but inserts the track id for window bundles; the
// slerp uses lower_bound(t) without an end() check.
#include "frontend/frame_processor_base.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <ceres/ceres.h>

#include "ceres_backend/ceres_backend_interface.hpp"
#include "ceres_backend/outlier_rejection.hpp"          // depthInFrame 0x18008ACB0
#include "ceres_backend/pose_local_parameterization.hpp"   // ceres_backend::PoseLocalParameterization
#include "ceres_backend/reprojection_error.hpp"         // ceres_backend::ReprojectionError
#include "common/frame.h"
#include "common/logger.h"
#include "common/point.h"
#include "frontend/imu_factor.h"                        // IMUFactor, extern G
#include "frontend/imu_processor.h"
#include "frontend/initialization.h"
#include "frontend/integration_base.h"
#include "frontend/map.h"
#include "frontend/pose_local_parameterization.h"       // pimax::totem::PoseLocalParameterization,
                                                        // InitGravityLocalParameter
#include "frontend/utility.h"                           // Utility::R2ypr
#include "frontend/visual_imu_alignment.h"              // ImuInitializer, ImageFrame

namespace pimax {
namespace totem {

// ============================================================================================
// 0x1800DDCF0  euler angles -> rotation matrix (only caller: initializeImu, "diff yaw %f").
// pimax-new (VINS ypr2R-like but R = Rz(e0) * Rx(e1) * Ry(e2), degrees unless is_rad); a
// function template (COMDAT among the "??$" templates of frame_processor_base.obj).
// Header/name unknown -- TODO(verify) (c07 name).  The three matrices are written element-wise
// (no CommaInitializer calls in the binary, unlike Utility::skewSymmetric).
// ============================================================================================
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 3> eulerToRotation(const Eigen::Matrix<Scalar, 3, 1>& euler, bool is_rad)
{
  Scalar a = euler(0);
  Scalar b = euler(1);
  Scalar c = euler(2);
  if (!is_rad)
  {
    a = a / 180.0 * 3.141592653589793;   // 0x1803B6D38 / 0x1803B1390
    b = b / 180.0 * 3.141592653589793;
    c = c / 180.0 * 3.141592653589793;
  }
  const Scalar ca = std::cos(a), sa = std::sin(a);   // call order in the binary: cos, sin
  Eigen::Matrix<Scalar, 3, 3> Rz;
  Rz(0, 0) = ca;  Rz(1, 0) = sa;  Rz(2, 0) = 0;
  Rz(0, 1) = -sa; Rz(1, 1) = ca;  Rz(2, 1) = 0;
  Rz(0, 2) = 0;   Rz(1, 2) = 0;   Rz(2, 2) = 1;
  const Scalar cb = std::cos(b), sb = std::sin(b);
  Eigen::Matrix<Scalar, 3, 3> Rx;
  Rx(0, 0) = 1;   Rx(1, 0) = 0;   Rx(2, 0) = 0;
  Rx(0, 1) = 0;   Rx(1, 1) = cb;  Rx(2, 1) = sb;
  Rx(0, 2) = 0;   Rx(1, 2) = -sb; Rx(2, 2) = cb;
  const Scalar cc = std::cos(c), sc = std::sin(c);
  Eigen::Matrix<Scalar, 3, 3> Ry;
  Ry(0, 0) = cc;  Ry(1, 0) = 0;   Ry(2, 0) = -sc;
  Ry(0, 1) = 0;   Ry(1, 1) = 1;   Ry(2, 1) = 0;
  Ry(0, 2) = sc;  Ry(1, 2) = 0;   Ry(2, 2) = cc;
  return Rz * Rx * Ry;
}

// 0x180110490
bool FrameProcessorBase::initializeImu(const Eigen::Quaternionf& imu_rotation,
                                       const bool has_imu_rotation,
                                       const double imu_rotation_timestamp)
{
  if (stage_ != Stage::kTracking)
  {
    frame_bundle_map_.clear();
    all_image_frame_.clear();
    bundle_buffer_.clear();
    imu_rotation_buffer_.clear();
    return false;
  }

  if (has_imu_rotation)
  {
    imu_rotation_buffer_.insert(std::make_pair(imu_rotation_timestamp, imu_rotation));
    if (imu_rotation_buffer_.size() > 10)
      imu_rotation_buffer_.erase(imu_rotation_buffer_.begin());
  }

  // ---------------------------------------------------------------------------- A --
  {
    std::shared_ptr<IntegrationBase> pre_integration;
    const double t_cur = new_frames_->at(0)->getTimestampSec();
    if (!frame_bundle_map_.empty())
    {
      ++keyframe_counter_;
      const Eigen::Vector3d p_last =
          std::prev(all_image_frame_.end())->second.T_world_cam.getPosition();
      const Eigen::Vector3d p_cur = new_frames_->at(0)->T_f_w_.inverse().getPosition();
      const double dist = (p_cur - p_last).norm();
      keyframe_step_ = std::max(keyframe_step_, 1);
      bundle_buffer_.push_back(new_frames_);
      if ((keyframe_counter_ % keyframe_step_ != 0 && keyframe_min_dist_ > dist)
          || new_frames_->num_tracked_ < 120)
      {
        while (bundle_buffer_.size() > 50)
          bundle_buffer_.erase(bundle_buffer_.begin());
        return false;
      }
      keyframe_counter_ = 0;

      // merge the IMU samples of all buffered bundles (newest first)
      ImuMeasurements imu_merged;
      for (const FrameBundlePtr& bundle : bundle_buffer_)
      {
        ImuMeasurements imu = bundle->imu_measurements_;
        if (!imu_merged.empty())
        {
          const double t_front = imu_merged.front().timestamp_;
          for (size_t i = imu.size() - 1; i != 0; --i)
          {
            if (t_front < imu[i].timestamp_)
              imu_merged.push_front(imu[i]);
          }
          imu_merged.push_front(imu[0]);
        }
        else
        {
          imu_merged = imu;
        }
      }
      new_frames_->imu_measurements_ = imu_merged;
      new_frames_->last_timestamp_sec_ = bundle_buffer_[0]->last_timestamp_sec_;
      bundle_buffer_.clear();

      // pre-integrate from the oldest (back) to the newest (front) sample
      for (size_t i = imu_merged.size() - 1; i != 0; --i)
      {
        const ImuMeasurement& m = imu_merged[i];
        const Eigen::Vector3d acc = m.linear_acceleration_.cast<double>();
        const Eigen::Vector3d gyr = m.angular_velocity_.cast<double>();
        if (i == imu_merged.size() - 1)
        {
          Eigen::Vector3d ba = imu_handler_->acc_bias_;    // ImuProcessor +208
          Eigen::Vector3d bg = imu_handler_->omega_bias_;  // ImuProcessor +232
          if (have_prior_bias_)
          {
            ba = prior_acc_bias_;
            bg = prior_gyro_bias_;
          }
          else if (saved_gyro_bias_valid_)
          {
            bg = saved_gyro_bias_;
          }
          pre_integration = std::make_shared<IntegrationBase>(acc, gyr, ba, bg);
          // noise parameters are overwritten *after* construction (noise matrix keeps the
          // defaults computed by the ctor)
          pre_integration->G.x() = 0.0;                                              // +0
          pre_integration->G.y() = 0.0;                                              // +8
          pre_integration->G.z() = imu_handler_->imu_calib_.gravity_magnitude;        // +16 <- calib +56
          pre_integration->ACC_N = imu_handler_->imu_calib_.acc_noise_density;          // +24 <- calib +24
          pre_integration->ACC_W = imu_handler_->imu_calib_.acc_bias_random_walk_sigma; // +32 <- calib +48
          pre_integration->GYR_N = imu_handler_->imu_calib_.gyro_noise_density;         // +40 <- calib +16
          pre_integration->GYR_W = imu_handler_->imu_calib_.gyro_bias_random_walk_sigma;// +48 <- calib +40
          if (new_frames_->last_timestamp_sec_ > m.timestamp_)
          {
            const double dt = imu_merged[i - 1].timestamp_ - new_frames_->last_timestamp_sec_;
            if (dt >= 0.0001)
              pre_integration->push_back(dt, acc, gyr);
          }
          continue;
        }
        double dt = imu_merged[i - 1].timestamp_ - m.timestamp_;
        const double t_frame = new_frames_->at(0)->getTimestampSec();
        if (imu_merged[i - 1].timestamp_ > t_frame)
        {
          dt = t_frame - m.timestamp_;
          if (dt < 0.0001)
            continue;
        }
        pre_integration->push_back(dt, acc, gyr);
      }
    }
    else
    {
      ++keyframe_counter_;
    }

    ImageFrame image_frame;
    image_frame.t = new_frames_->at(0)->getTimestampSec();
    image_frame.is_key_frame = new_frames_->is_keyframe_;
    image_frame.pre_integration = pre_integration;
    image_frame.T_world_cam = new_frames_->at(0)->T_f_w_.inverse();
    image_frame.T_world_cam.getRotation().toImplementation().normalize();
    if (all_image_frame_.size() > max_image_frames_)
    {
      frame_bundle_map_.erase(frame_bundle_map_.begin());
      all_image_frame_.erase(all_image_frame_.begin());
    }
    frame_bundle_map_.insert(std::make_pair(t_cur, new_frames_));
    all_image_frame_.insert(std::make_pair(t_cur, image_frame));
  }

  std::shared_ptr<CeresBackendInterface> backend = bundle_adjustment_;
  const size_t n_backend_frames = backend->optimizer_options_.num_imu_frames;   // backend +96 (c09 window_size_)
  window_size_ = std::max(window_size_, n_backend_frames + 2);
  Transformation T_x;
  std::vector<FrameBundlePtr> window_bundles;
  if (all_image_frame_.size() < window_size_)
    return false;

  {
    std::map<double, ImageFrame> local_image_frames;
    auto it = all_image_frame_.end();
    while (it != all_image_frame_.begin())
    {
      local_image_frames.insert(*std::prev(it));
      window_bundles.push_back(frame_bundle_map_[std::prev(it)->first]);
      if (local_image_frames.size() >= window_size_)
        break;
      --it;
    }
    std::reverse(window_bundles.begin(), window_bundles.end());

    // -------------------------------------------------------------------------- B --
    const size_t num_cams = new_frames_->frames_.size();
    {
      std::unordered_map<int, int> landmark_index;
      int num_landmarks = 0;
      for (const FrameBundlePtr& bundle : window_bundles)
      {
        for (size_t j = 0; j < num_cams; ++j)
        {
          const FramePtr frame = bundle->frames_.at(j);
          for (size_t i = 0; i < frame->num_features_; ++i)
          {
            if (frame->track_id_vec_(i) > -1)
            {
              if (landmark_index.find(frame->track_id_vec_(i)) == landmark_index.end())
              {
                const PointPtr& point = frame->landmark_vec_[i];
                const float depth = depthInFrame(frame->T_f_w_.cast<float>(), point->pos_);
                if (depth > 0.0f && depth <= 10.0f)
                  landmark_index[frame->track_id_vec_(i)] = num_landmarks++;
              }
            }
          }
        }
      }

      ceres::Problem problem;
      ceres::LossFunction* loss_function = new ceres::HuberLoss(0.5);
      ceres_backend::PoseLocalParameterization* local_parameterization =
          new ceres_backend::PoseLocalParameterization();
      local_parameterization->use_minimal_jacobians_ = true;   // +16 (c09 flag_16_)

      double* para_ex = new double[7 * num_cams];
      for (size_t j = 0; j < num_cams; ++j)
      {
        const Transformation T_imu_cam = new_frames_->frames_.at(j)->T_body_cam_;
        // float translation (cvtsd2ss/cvtpd2ps at 0x180111AC9..0x180111AF0), double quaternion
        const Eigen::Vector3f t = T_imu_cam.getPosition().cast<float>();
        Eigen::Vector4d q = T_imu_cam.getRotation().toImplementation().coeffs().normalized();
        const Eigen::Vector4d q_copy = q;
        para_ex[7 * j + 0] = t[0];
        para_ex[7 * j + 1] = t[1];
        para_ex[7 * j + 2] = t[2];
        Eigen::Map<Eigen::Vector4d>(para_ex + 7 * j + 3) = q_copy;
        problem.AddParameterBlock(para_ex + 7 * j, 7, local_parameterization);
        problem.SetParameterBlockConstant(para_ex + 7 * j);
      }

      double* para_landmark = new double[3 * num_landmarks];
      double* para_pose = new double[7 * window_bundles.size()];
      std::unordered_map<int, int> landmark_added;
      for (size_t k = 0; k < window_bundles.size(); ++k)
      {
        double* pose = para_pose + 7 * k;
        for (size_t j = 0; j < num_cams; ++j)
        {
          const FramePtr frame = window_bundles[k]->frames_.at(j);
          const Transformation T_world_imu = frame->T_world_imu();
          // float translation (cvtpd2ps at 0x180111DCD..0x180111DF5), double quaternion
          const Eigen::Vector3f t = T_world_imu.getPosition().cast<float>();
          Eigen::Vector4d q = T_world_imu.getRotation().toImplementation().coeffs().normalized();
          const Eigen::Vector4d q_copy = q;
          pose[0] = t[0];
          pose[1] = t[1];
          pose[2] = t[2];
          Eigen::Map<Eigen::Vector4d>(pose + 3) = q_copy;
          problem.AddParameterBlock(pose, 7, local_parameterization);

          for (size_t i = 0; i < frame->num_features_; ++i)
          {
            if (frame->track_id_vec_(i) > -1)
            {
              const Eigen::Vector3f p_w = frame->landmark_vec_[i]->pos_;
              const float depth = depthInFrame(frame->T_f_w_.cast<float>(), p_w);
              if (depth > 0.0f && depth <= 10.0f)
              {
                int track_id = frame->track_id_vec_(i);
                const int idx = landmark_index[track_id];
                if (landmark_added.find(track_id) == landmark_added.end())
                {
                  double* landmark = para_landmark + 3 * idx;
                  landmark[0] = p_w[0];
                  landmark[1] = p_w[1];
                  landmark[2] = p_w[2];
                  problem.AddParameterBlock(landmark, 3);
                  problem.SetParameterBlockConstant(landmark);
                  landmark_added[track_id] = 1;
                }
                Eigen::Matrix2d sqrt_info = Eigen::Matrix2d::Identity();
                sqrt_info *= 1.0 / (1 << frame->level_vec_(i));
                ceres_backend::ReprojectionError* reprojection_error =
                    new ceres_backend::ReprojectionError(
                        frame->cam_, frame->px_vec_.col(i).cast<double>(), sqrt_info);
                reprojection_error->use_minimal_jacobians_ = true;   // +48 (c09 flag_48_)
                problem.AddResidualBlock(
                    reprojection_error, loss_function,
                    std::vector<double*>{pose, para_landmark + 3 * idx, para_ex + 7 * j});
                problem.SetParameterBlockConstant(para_ex + 7 * j);
              }
            }
          }
        }
      }

      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_SCHUR;
      options.minimizer_progress_to_stdout = false;
      options.trust_region_strategy_type = ceres::DOGLEG;
      options.max_num_iterations = 5;
      options.num_threads = 1;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);

      for (size_t k = 0; k < window_bundles.size(); ++k)
      {
        const double* pose = para_pose + 7 * k;
        const Eigen::Vector3d t(pose[0], pose[1], pose[2]);
        const Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);
        const Transformation T_world_imu(t, q.normalized());   // 0x180039830
        for (const FramePtr& frame : window_bundles[k]->frames_)
        {
          frame->T_f_w_ = (T_world_imu * frame->T_body_cam_).inverse();
          frame->T_f_w_.getRotation().toImplementation().normalize();
        }
        double t_k = window_bundles[k]->at(0)->getTimestampSec();
        all_image_frame_[t_k].T_world_cam = window_bundles[k]->at(0)->T_f_w_.inverse();
        local_image_frames[t_k].T_world_cam = window_bundles[k]->at(0)->T_f_w_.inverse();
      }

      for (const FrameBundlePtr& bundle : window_bundles)
      {
        for (size_t j = 0; j < num_cams; ++j)
        {
          const FramePtr frame = bundle->frames_.at(j);
          for (size_t i = 0; i < frame->num_features_; ++i)
          {
            if (frame->track_id_vec_(i) > -1
                && landmark_index.count(frame->landmark_vec_[i]->id()))
            {
              const int idx = landmark_index[frame->track_id_vec_(i)];
              Eigen::Vector3f& pos = frame->landmark_vec_[i]->pos_;
              pos[0] = para_landmark[3 * idx];
              pos[1] = para_landmark[3 * idx + 1];
              pos[2] = para_landmark[3 * idx + 2];
            }
          }
        }
      }
      delete[] para_ex;
      delete[] para_landmark;
      delete[] para_pose;
    }

    // -------------------------------------------------------------------------- C --
    int i_frame = 0;
    Transformation T0;
    for (auto frame_it = local_image_frames.begin(); frame_it != local_image_frames.end();
         ++frame_it)
    {
      Transformation T_i;
      if (i_frame == 0)
        T0 = frame_it->second.T_world_cam.inverse();
      T_i = T0 * frame_it->second.T_world_cam;
      frame_it->second.R = (T_i * new_frames_->at(0)->T_cam_body_).getRotationMatrix();
      frame_it->second.T = T_i.getPosition();
      frame_it->second.T_c0_body =
          new_frames_->at(0)->T_body_cam_ * T_i * new_frames_->at(0)->T_cam_body_;
      frame_it->second.T_i = T_i;
      ++i_frame;
    }

    // ImuInitializer (c13; c09's ImuInitParams + free VisualIMUAlignment), ctor 0x1800E1FD0.
    // G.x/G.y are written with one 16-byte zero store, G.z = gravity magnitude.
    ImuInitializer initializer;
    initializer.G.x() = 0.0;
    initializer.G.y() = 0.0;
    initializer.G.z() = imu_handler_->imu_calib_.gravity_magnitude;
    const Eigen::Matrix3d R_imu_cam = new_frames_->at(0)->T_body_cam_.getRotationMatrix();
    initializer.RIC.assign(&R_imu_cam, &R_imu_cam + 1);                  // 0x1800BA360
    const Eigen::Vector3d t_imu_cam = new_frames_->at(0)->T_body_cam_.getPosition();
    initializer.TIC.assign(&t_imu_cam, &t_imu_cam + 1);                  // 0x1800BA190
    // aligned vector: allocator 0x1800FD8E0 (Eigen aligned malloc), freed with free()
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Bgs(
        local_image_frames.size(), Eigen::Vector3d::Zero());             // 0x1800DFCA0
    Eigen::VectorXd x;
    const Eigen::Vector3d ba_init = imu_handler_->acc_bias_;
    Eigen::Vector3d g;
    if (!initializer.VisualIMUAlignment(local_image_frames, Bgs, g, x, ba_init))   // 0x180157240
      return false;

    T_x = initializer.T_w0_ * new_frames_->at(0)->T_body_cam_ * T0;
    T_x.getRotation().toImplementation().normalize();

    for (const FrameBundlePtr& bundle : window_bundles)
    {
      double t_b = bundle->at(0)->getTimestampSec();
      bundle->T_W_B_init_ = local_image_frames[t_b].T_w_body;
      bundle->imu_vel_w_ = local_image_frames[t_b].V_w;
      bundle->imu_gyr_bias_ = imu_handler_->omega_bias_;
      bundle->imu_acc_bias_ = imu_handler_->acc_bias_;
      if (have_prior_bias_)
      {
        bundle->imu_gyr_bias_ = prior_gyro_bias_;
        bundle->imu_acc_bias_ = prior_acc_bias_;
      }
    }

    // -------------------------------------------------------------------------- D --
    {
      ceres::Problem problem;
      ceres::LossFunction* loss_function = new ceres::HuberLoss(0.5);
      ceres::LocalParameterization* local_parameterization = new pimax::totem::PoseLocalParameterization();
      ceres::LocalParameterization* gravity_parameterization = new InitGravityLocalParameter();
      const size_t n = window_bundles.size();
      double* para_pose = new double[7 * n];
      double* para_speed = new double[3 * n];
      double* para_ba = new double[3 * n];
      double* para_bg = new double[3 * n];
      double para_g[3] = {0.0, 0.0, imu_handler_->imu_calib_.gravity_magnitude};
      problem.AddParameterBlock(para_g, 3, gravity_parameterization);
      {
        const FrameBundlePtr bundle = window_bundles[0];
        const Eigen::Vector3d p = bundle->T_W_B_init_.getPosition();
        const Eigen::Quaterniond q = bundle->T_W_B_init_.getRotation().toImplementation();
        para_pose[0] = p[0];
        para_pose[1] = p[1];
        para_pose[2] = p[2];
        para_pose[3] = q.x();
        para_pose[4] = q.y();
        para_pose[5] = q.z();
        para_pose[6] = q.w();
        problem.AddParameterBlock(para_pose, 7, local_parameterization);
        problem.SetParameterBlockConstant(para_pose);
        para_speed[0] = bundle->imu_vel_w_[0];
        para_speed[1] = bundle->imu_vel_w_[1];
        para_speed[2] = bundle->imu_vel_w_[2];
        para_ba[0] = bundle->imu_acc_bias_[0];
        para_ba[1] = bundle->imu_acc_bias_[1];
        para_ba[2] = bundle->imu_acc_bias_[2];
        para_bg[0] = bundle->imu_gyr_bias_[0];
        para_bg[1] = bundle->imu_gyr_bias_[1];
        para_bg[2] = bundle->imu_gyr_bias_[2];
        problem.AddParameterBlock(para_speed, 3);
        problem.SetParameterBlockConstant(para_speed);
        problem.AddParameterBlock(para_ba, 3);
        problem.SetParameterBlockConstant(para_ba);
        problem.AddParameterBlock(para_bg, 3);
        problem.SetParameterBlockConstant(para_bg);
      }
      for (size_t i = 1; i < window_bundles.size(); ++i)
      {
        const FrameBundlePtr bundle = window_bundles[i];
        double* pose = para_pose + 7 * i;
        double* speed = para_speed + 3 * i;
        double* ba = para_ba + 3 * i;
        double* bg = para_bg + 3 * i;
        const Eigen::Vector3d p = bundle->T_W_B_init_.getPosition();
        const Eigen::Quaterniond q = bundle->T_W_B_init_.getRotation().toImplementation();
        pose[0] = p[0];
        pose[1] = p[1];
        pose[2] = p[2];
        pose[3] = q.x();
        pose[4] = q.y();
        pose[5] = q.z();
        pose[6] = q.w();
        problem.AddParameterBlock(pose, 7, local_parameterization);
        problem.SetParameterBlockConstant(pose);
        speed[0] = bundle->imu_vel_w_[0];
        speed[1] = bundle->imu_vel_w_[1];
        speed[2] = bundle->imu_vel_w_[2];
        ba[0] = bundle->imu_acc_bias_[0];
        ba[1] = bundle->imu_acc_bias_[1];
        ba[2] = bundle->imu_acc_bias_[2];
        bg[0] = bundle->imu_gyr_bias_[0];
        bg[1] = bundle->imu_gyr_bias_[1];
        bg[2] = bundle->imu_gyr_bias_[2];
        problem.AddParameterBlock(speed, 3);
        problem.AddParameterBlock(ba, 3);
        problem.SetParameterBlockConstant(ba);
        problem.AddParameterBlock(bg, 3);
        problem.SetParameterBlockConstant(bg);

        double t_i = bundle->at(0)->getTimestampSec();
        const Eigen::Vector3d ba_i(ba[0], ba[1], ba[2]);
        const Eigen::Vector3d bg_i(bg[0], bg[1], bg[2]);
        local_image_frames[t_i].pre_integration->repropagate(ba_i, bg_i);   // 0x18011AFE0
        IMUFactor* imu_factor =
            new IMUFactor(local_image_frames[t_i].pre_integration, initializer.G);   // 0x1800E1950
        problem.AddResidualBlock(
            imu_factor, loss_function,
            std::vector<double*>{pose - 7, speed - 3, ba - 3, bg - 3, pose, speed, ba, bg, para_g});
        problem.SetParameterBlockConstant(pose - 7);
        problem.SetParameterBlockConstant(pose);
      }

      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_SCHUR;
      options.minimizer_progress_to_stdout = false;
      options.trust_region_strategy_type = ceres::DOGLEG;
      options.max_num_iterations = 5;
      options.num_threads = 1;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);

      for (size_t i = 0; i < window_bundles.size(); ++i)
      {
        const double* pose = para_pose + 7 * i;
        const Eigen::Vector3d speed(para_speed[3 * i], para_speed[3 * i + 1], para_speed[3 * i + 2]);
        const Eigen::Vector3d ba(para_ba[3 * i], para_ba[3 * i + 1], para_ba[3 * i + 2]);
        const Eigen::Vector3d bg(para_bg[3 * i], para_bg[3 * i + 1], para_bg[3 * i + 2]);
        window_bundles[i]->imu_vel_w_ = speed;
        window_bundles[i]->imu_gyr_bias_ = bg;
        window_bundles[i]->imu_acc_bias_ = ba;
        const Eigen::Vector3d p(pose[0], pose[1], pose[2]);
        const Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);
        window_bundles[i]->T_W_B_init_.getPosition() = p;
        window_bundles[i]->T_W_B_init_.getRotation() =
            kindr::minimal::RotationQuaternionTemplate<double>(q);   // checking ctor 0x1800089C0
      }
      delete[] para_pose;
      delete[] para_speed;
      delete[] para_ba;
      delete[] para_bg;
    }
  }

  // ---------------------------------------------------------------------------- E --
  const Eigen::Vector3d g_est = G;
  LOGW("init_gravity x %f y %f z %f norm %f\n", G.x(), G.y(), G.z(), G.norm());
  if (g_est.head<2>().norm() > 0.45)
  {
    LOGW("imu initial gravity estimator is not good\n");
    return false;
  }
  LOGW("imu initial gravity estimator is good\n");
  const double cos_threshold = std::cos(3.5 * M_PI / 180.0);
  const Eigen::Vector3d g_ref = Eigen::Vector3d(0.0, 0.0, 9.80667).normalized();
  const Eigen::Vector3d g_dir = G.normalized();
  const double cos_angle = g_dir.dot(g_ref);
  if (cos_threshold > cos_angle)
  {
    LOGW("imu initial gravity direction error: angle=%f degrees",
         std::acos(cos_angle) * 180.0 / M_PI);
    return false;
  }

  // ---------------------------------------------------------------------------- F --
  Transformation T_wb = T_x * window_bundles.back()->get_T_W_B();
  T_wb.getRotation().toImplementation().normalize();
  const Eigen::Vector3d p_wb = T_wb.getPosition();
  T_wb.getPosition() = Eigen::Vector3d::Zero();
  if (prior_position_loaded_ && T_prior_.getPosition().norm() < 100.0)
    T_wb.getPosition() += T_prior_.getPosition();
  else
    T_wb.getPosition() = p_wb;

  Eigen::Matrix3d R_yaw = Eigen::Matrix3d::Identity();
  if (has_imu_rotation)
  {
    Eigen::Quaternionf q_imu = imu_rotation;
    if (imu_rotation_buffer_.size() > 2)
    {
      const double t = new_frames_->getMinTimestampSeconds();
      auto it_upper = imu_rotation_buffer_.lower_bound(t);
      if (it_upper != imu_rotation_buffer_.begin())
      {
        auto it_lower = std::prev(it_upper);
        q_imu = slerpByTime(it_lower->second, it_upper->second, it_lower->first,
                            it_upper->first, new_frames_->getMinTimestampSeconds());
      }
    }
    const double yaw_vio = Utility::R2ypr(T_wb.getRotationMatrix(), false).x();            // 0x1800F3260
    const double yaw_imu =
        Utility::R2ypr(q_imu.cast<double>().toRotationMatrix(), false).x();
    const double diff_yaw = yaw_imu - yaw_vio;
    LOGI("diff yaw %f\n", diff_yaw);
    R_yaw = eulerToRotation(Eigen::Vector3d(diff_yaw, 0.0, 0.0), false);                   // 0x1800DDCF0
  }
  const Eigen::Matrix3d R_aligned = R_yaw * T_wb.getRotationMatrix();
  T_wb.getRotation() =
      kindr::minimal::RotationQuaternionTemplate<double>(R_aligned);   // 0x1800DEA10 (CHECK isValidRotationMatrix)
  Transformation T_wb_old = T_x * window_bundles.back()->get_T_W_B();
  T_wb_old.getRotation().toImplementation().normalize();
  T_x = T_wb * T_wb_old.inverse() * T_x;
  T_x.getRotation().toImplementation().normalize();

  // ---------------------------------------------------------------------------- G --
  {
  std::unordered_map<int, PointPtr> updated_points;
  std::unordered_map<int, FramePtr> updated_frames;
  initializer_->reset();                                         // vtable slot 2 of +512
  for (const FrameBundlePtr& bundle : window_bundles)
  {
    for (const FramePtr& frame : bundle->frames_)
    {
      for (size_t i = 0; i < frame->num_features_; ++i)
      {
        if (frame->track_id_vec_(i) > -1)
        {
          if (!updated_points.count(frame->landmark_vec_[i]->id()))
          {
            frame->landmark_vec_[i]->pos_ = T_x.cast<float>() * frame->landmark_vec_[i]->pos_;
            updated_points.insert(std::make_pair(frame->track_id_vec_(i), frame->landmark_vec_[i]));
          }
        }
      }
      frame->set_T_w_imu(T_x * frame->T_world_imu());                                       // 0x1801283F0
      updated_frames.insert(std::make_pair(frame->id(), frame));
    }
    bundle->setIMUState(R_yaw * bundle->imu_vel_w_, bundle->imu_gyr_bias_,
                        bundle->imu_acc_bias_);                                               // 0x180127700
  }
  for (auto kf_it = map_->keyframes_.begin(); kf_it != map_->keyframes_.end(); ++kf_it)
  {
    int frame_id = kf_it->second->id();
    if (!updated_frames.count(frame_id))
    {
      updated_frames.insert(std::make_pair(frame_id, kf_it->second));
      kf_it->second->set_T_w_imu(T_x * kf_it->second->T_world_imu());
      const FramePtr& frame = kf_it->second;
      for (size_t i = 0; i < frame->num_features_; ++i)
      {
        if (frame->track_id_vec_(i) > -1 && !updated_points.count(frame->track_id_vec_(i)))
        {
          frame->landmark_vec_[i]->pos_ = T_x.cast<float>() * frame->landmark_vec_[i]->pos_;
          updated_points.insert(std::make_pair(frame->track_id_vec_(i), frame->landmark_vec_[i]));
        }
      }
    }
  }
  for (const FramePtr& frame : last_frames_->frames_)
  {
    int frame_id = frame->id();
    if (!updated_frames.count(frame_id))
    {
      updated_frames.insert(std::make_pair(frame_id, frame));
      frame->set_T_w_imu(T_x * frame->T_world_imu());
      for (size_t i = 0; i < frame->num_features_; ++i)
      {
        if (frame->track_id_vec_(i) > -1 && !updated_points.count(frame->track_id_vec_(i)))
        {
          frame->landmark_vec_[i]->pos_ = T_x.cast<float>() * frame->landmark_vec_[i]->pos_;
          updated_points.insert(std::make_pair(frame->track_id_vec_(i), frame->landmark_vec_[i]));
        }
      }
    }
  }
  for (const FramePtr& frame : last_last_frames_->frames_)
  {
    int frame_id = frame->id();
    if (!updated_frames.count(frame_id))
    {
      updated_frames.insert(std::make_pair(frame_id, frame));
      frame->set_T_w_imu(T_x * frame->T_world_imu());
      for (size_t i = 0; i < frame->num_features_; ++i)
      {
        if (frame->track_id_vec_(i) > -1 && !updated_points.count(frame->track_id_vec_(i)))
        {
          frame->landmark_vec_[i]->pos_ = T_x.cast<float>() * frame->landmark_vec_[i]->pos_;
          updated_points.insert(std::make_pair(frame->track_id_vec_(i), frame->landmark_vec_[i]));
        }
      }
    }
  }
  }  // ~updated_frames, ~updated_points (0x180019920)

  // ---------------------------------------------------------------------------- H --
  // the newest three bundles of the backend window become keyframes, older ones are demoted
  int cnt = 0;
  for (int k = window_bundles.size() - n_backend_frames - 1; k < window_bundles.size(); ++k, ++cnt)
  {
    const FrameBundlePtr& bundle = window_bundles[k];
    if (cnt >= 3)
    {
      if (!bundle->is_keyframe_)
        continue;
      bundle->is_keyframe_ = false;
      for (const FramePtr& frame : window_bundles[k]->frames_)
      {
        frame->is_keyframe_ = (cnt < 3);
        map_->removeKeyframe(frame->id());                                                    // 0x18012E9F0
        auto id_it = std::find(map_->sorted_keyframe_ids_.begin(), map_->sorted_keyframe_ids_.end(), frame->id());
        if (id_it != map_->sorted_keyframe_ids_.end())
          map_->sorted_keyframe_ids_.erase(id_it);                                                   // 0x180108C50
      }
    }
    else
    {
      if (bundle->is_keyframe_)
        continue;
      bundle->is_keyframe_ = true;
      for (const FramePtr& frame : window_bundles[k]->frames_)
      {
        frame->setKeyframe();                                                                 // 0x180096E80
        map_->addKeyframe(frame);                                                             // 0x18012D680
        for (size_t i = 0; i < frame->num_features_; ++i)
        {
          if (frame->track_id_vec_(i) > -1)
          {
            const PointPtr& point = frame->landmark_vec_[i];
            int frame_id = frame->id();
            if (point->obs_.find(frame_id) == point->obs_.end())
            {
              int obs_frame_id = frame->id();
              point->obs_.insert(std::make_pair(obs_frame_id, KeypointIdentifier(frame, i)));
            }
          }
        }
      }
    }
  }

  // ---------------------------------------------------------------------------- I --
  for (size_t i = window_bundles.size() - 1; i < window_bundles.size(); ++i)
  {
    last_last_frames_ = last_frames_;
    last_frames_ = window_bundles[i - 1];
    new_frames_ = window_bundles[i];
    bundle_adjustment_->loadMapFromBundleAdjustment(new_frames_, last_frames_, map_,
                                                    have_motion_prior_, G, true);   // 0x180013480
    bundle_adjustment_->bundleAdjustment(new_frames_, 0, loc_mode_);                // 0x180011700
    if (i > window_bundles.size() - 2)
      bundle_adjustment_->optimize(new_frames_, frame_flag_, loc_mode_);            // 0x180013680
  }

  // ---------------------------------------------------------------------------- J --
  imu_init_pos_ = new_frames_->at(0)->imuPos();
  frame_bundle_map_.clear();
  all_image_frame_.clear();
  imu_rotation_buffer_.clear();
  t1_voImuInit_ = reloc_timer_.stop();      // +4008 (c09 imu_init_time_sec_ / imu_init_timer_)
  reloc_timer_.start();
  return true;
}

}  // namespace totem
}  // namespace pimax
