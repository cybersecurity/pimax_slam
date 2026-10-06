// pimax_slam.pi.dll -- src/frontend/frame_processor_base.cpp  (phase B2)
//
// pimax::totem::FrameProcessorBase -- Pimax fork of svo/src/frame_handler_base.cpp
// (svo::FrameHandlerBase).  Original: E:\code_codex\pimax_slam\beta111_5a7902_dll\src\frontend\
// frame_processor_base.cpp (~2800 lines; glog __LINE__s 2417, 2568, 2619, 2668, 2714, 2770, 2774
// are reproduced with #line below).  The original object frame_processor_base.obj spans
// 0x1800B57D0..0x180129D60; for size the reconstruction is split into three files:
//   frame_processor_base.cpp          globals, ctor/dtor, input, motion prior, resets, setters,
//                                     relocalisation, watchdog, tracking (c00/c07/c08/c09/c10)
//   frame_processor_base_ground.cpp   mesh / plane / ground-plane members (c08)
//   frame_processor_base_imu_init.cpp initializeImu 0x180110490 (c09) + eulerToRotation (c07)
// Drafts merged: c00 frame_processor_base_globals.cpp, c07 frame_processor_base_ctor.cpp,
// c08 frame_processor_base_dtor.cpp, c09 frame_processor_base.cpp, c10 frame_processor_base.cpp.
// Name reconciliation: notes/integration_A2.md section 2.9 and notes/integration_B2.md.
//
// Not reproduced (no source effect / library machinery): the two atexit-only statics of
// pangolin/handler/handler.h this TU included (c00 #39/#40), and the 84 boost::serialization
// singletons of the PlatMap/KeyFrame types that were first instantiated in this object
// (c00 #46..#129; they come with the loop-closing headers, see notes/integration_B2.md).
//
// Logger: LOGD = 0x18000C120, LOGI = 0x18000F500, LOGW = 0x18000F6A0, LOGE = 0x18000C2C0.

#include "frontend/frame_processor_base.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <glog/logging.h>
#include <vikit/performance_monitor.h>

#include "ceres_backend/ceres_backend_interface.hpp"
#include "common/frame.h"
#include "common/logger.h"
#include "common/point.h"
#include "direct/depth_filter.h"
#include "direct/feature_detection_utils.h"
#include "frontend/imu_factor.h"        // extern Eigen::Vector3d G
#include "frontend/imu_processor.h"
#include "frontend/initialization.h"
#include "frontend/map.h"
#include "frontend/pose_optimizer.h"
#include "frontend/reprojector.h"
#include "loop_closing/loop_closing.h"
#include "plane/mesher.h"
#include "plane/plane.h"

namespace pimax {
namespace totem {

// =============================================================================================
// globals (c00 frame_processor_base_globals.cpp; .CRT$XCU #42..#45 of TU30)
// =============================================================================================

// definition of global and static variables which were declared in the header
PerformanceMonitorPtr g_permon;  // 0x18047DDA0 (std::shared_ptr<vk::PerformanceMonitor>, atexit only)

// Upstream (svo/src/frame_handler_base.cpp:56-75) uses EnumClassHash; Pimax uses the default
// std::hash<Enum> (the insert helper 0x1800D0090 hashes the 4 enum bytes with FNV-1a).
const std::unordered_map<Stage, std::string> kStageName  // 0x18047DDB0 (init 0x180002410)
{{Stage::kPaused, "Paused"}, {Stage::kInitializing, "Initializing"},
  {Stage::kTracking, "Tracking"}, {Stage::kRelocalization, "Reloc"}};

const std::unordered_map<TrackingQuality, std::string> kTrackingQualityName  // 0x18047DDF0
{{TrackingQuality::kInsufficient, "Insufficient"},
  {TrackingQuality::kBad, "Bad"}, {TrackingQuality::kGood, "Good"}
};

const std::unordered_map<UpdateResult, std::string> kUpdateResultName  // 0x18047DE30
{{UpdateResult::kDefault, "Default"}, {UpdateResult::kKeyframe, "KF"},
  {UpdateResult::kFailure, "Failure"}};

// 0x18047ED98: VINS-Mono global gravity (frontend/imu_factor.h).  Zero-initialised .bss, no
// dynamic initializer; written by every IMUFactor::Evaluate, read by initializeImu and passed to
// CeresBackendInterface::loadMapFromBundleAdjustment.  TODO(verify) definition site (A2 placed it
// here; the address lies between the frame_processor_base / imu_processor .bss blocks).
Eigen::Vector3d G;

// =============================================================================================
// file-local helpers
// =============================================================================================

// 0x1800CB4B0 (caller projectMapInFrame 0x180119260).  pimax-new.  A function template: its
// COMDAT sorts among the "??$" templates of the object (between ??$cast and ??$defaultEnsure).
// The set is returned by move (two return paths => no NRVO, the binary swaps the tree heads).
// TODO(verify) name (c10's), template-ness and exact spelling.
template <typename T>
std::set<T> collectTrashPoints(bool is_static, const std::vector<std::vector<T>>& trash_points)
{
  if (is_static)
  {
    std::set<T> trash_set;
    for (const std::vector<T>& point_vec : trash_points)
    {
      for (const T& point : point_vec)
        trash_set.insert(point);
    }
    return trash_set;
  }
  return std::set<T>();
}


//------------------------------------------------------------------------------
// 0x1800FDEE0  (only caller: checkTrackingHealth 0x18011BCA0)
// pimax-new: translation distance (computed in float) and rotation angle in degrees.
void computePoseDifference(double* trans_diff, double* rot_diff_deg,
                           const Transformation& T_a, const Transformation& T_b)
{
  const Eigen::Vector3f p_a = T_a.getPosition().cast<float>();
  const Eigen::Vector3f p_b = T_b.getPosition().cast<float>();
  *trans_diff = (p_a - p_b).norm();
  const Eigen::Matrix3d R_a = T_a.getRotationMatrix();
  const Eigen::Matrix3d R_b = T_b.getRotationMatrix();
  *rot_diff_deg = std::acos(std::max(-1.0, std::min(1.0, ((R_a.transpose() * R_b).trace() - 1.0) * 0.5)))
                  * 180.0 / M_PI;
}

//------------------------------------------------------------------------------
// 0x1800FE140  (only caller: updateGroundPlane 0x1800F2070, result stored in Plane::area_)
// pimax-new: shoelace area of the plane polygon in its (x, y) coordinates.
// (c08 called it polygonArea; c09 drafted the vertices as cv::Point3f `pt` -- the vertex type
// is PolygonVertex {int lmk_id; Vector3f pos} (plane/plane.h), same bytes +4/+8.)
double computePolygonArea(const Plane& plane)
{
  const int n = static_cast<int>(plane.polygon_.size());
  if (n < 3)
    return 0.0;
  double area = 0.0;
  for (int i = 0; i < n; ++i)
  {
    const Eigen::Vector3f& p0 = plane.polygon_[i].pos;
    const Eigen::Vector3f& p1 = plane.polygon_[(i + 1) % n].pos;
    area += p0.x() * p1.y() - p1.x() * p0.y();   // float products, double accumulation
  }
  return std::fabs(area) * 0.5;
}

//------------------------------------------------------------------------------
// 0x180115430  (callers: initializeImu 0x180110490, setInitialPose 0x1801277B0)
// pimax-new: Eigen slerp at the normalised time (t - t0) / (t1 - t0) (alpha as float).
// (c10 called it interpolateQuaternion.)
Eigen::Quaternionf slerpByTime(const Eigen::Quaternionf& q0, const Eigen::Quaternionf& q1,
                               double t0, double t1, double t)
{
  return q0.slerp(static_cast<float>((t - t0) / (t1 - t0)), q1);
}

// =============================================================================================
// construction / destruction
// =============================================================================================

// 0x1800DFDF0
// upstream-modified.  All member initialisation (default member initialisers of
// frame_processor_base.h, in offset order) happens before the body.  Differences to
// svo::FrameHandlerBase::FrameHandlerBase:
//  * no need_new_kf_ std::bind, no imu/backend/global-map setup, no loop closing here;
//  * map_ is created from a unique_ptr (_Ref_count_resource<Map*, default_delete<Map>>),
//    the mesher from std::make_shared<Mesher>() (_Ref_count_obj2<Mesher>);
//  * mesh ROI from the image size, occupancy grid (options_.grid_size), per-camera containers;
//  * the 9th argument is unused; the 10th is stored at +464 and, when 0, the prior position file
//    is loaded (0x180115F00, "/pimax_prior_position.txt").
FrameProcessorBase::FrameProcessorBase(
    const BaseOptions& base_options,
    const ReprojectorOptions& reprojector_options,
    const DepthFilterOptions& depthfilter_options,
    const DetectorOptions& detector_options,
    const InitializationOptions& init_options,
    const FeatureTrackerOptions& tracker_options,
    const CameraBundlePtr& cameras,
    const std::string& /*device_sn*/,
    const uint8_t& loc_mode)
  : options_(base_options)
  , cams_(cameras)
  , map_(std::unique_ptr<Map>(new Map))                                 // 0x18012CE00
{
  mesher_ = std::make_shared<Mesher>();                                  // 0x18019C020

  {
    // binary order: +12 (height) is read before +8 (width)
    const int height = cams_->getCameraShared(0)->imageHeight();
    const int width = cams_->getCameraShared(0)->imageWidth();
    mesh_roi_ = cv::Rect2f(0.f, 0.f, static_cast<float>(width), static_cast<float>(height));
  }

  if (options_.trace_statistics)
  {
    // Initialize Performance Monitor (upstream FrameHandlerBase::initPerformanceMonitor,
    // inlined here; TODO(verify) whether it is a separate inline member in the Pimax source)
    g_permon = std::unique_ptr<vk::PerformanceMonitor>(new vk::PerformanceMonitor());
    g_permon->addTimer("pyramid_creation");
    g_permon->addTimer("sparse_img_align");
    g_permon->addTimer("reproject");
    g_permon->addTimer("reproject_kfs");
    g_permon->addTimer("reproject_candidates");
    g_permon->addTimer("frontend_time");
    g_permon->addTimer("frontend_update_map_from_ba");
    g_permon->addTimer("frontend_prepare_data_for_ba");
    g_permon->addTimer("feature_track_show");
    g_permon->addTimer("makeKeyframe_time");
    g_permon->addLog("timestamp");
    g_permon->addLog("img_align_n_tracked");
    g_permon->addLog("n_candidates");
    g_permon->addLog("dropout");
    g_permon->init("trace_frontend", options_.trace_dir);
  }

  // init modules
  reprojectors_.reserve(cams_->numCameras());
  for (size_t camera_idx = 0; camera_idx < cams_->numCameras(); ++camera_idx)
  {
    // the binary moves a std::unique_ptr temporary into the vector (source nulled after the
    // push).  TODO(verify) exact spelling.
    reprojectors_.push_back(std::unique_ptr<Reprojector>(
        new Reprojector(reprojector_options, camera_idx)));             // 0x1801412F0 (0xF0)
  }
  pose_optimizer_ = std::unique_ptr<PoseOptimizer>(
      new PoseOptimizer(PoseOptimizer::getDefaultSolverOptions()));     // 0x18013B2B0, ctor 0x180132D90
  if (options_.poseoptim_using_unit_sphere)
    pose_optimizer_->setErrorType(PoseOptimizer::ErrorType::kBearingVectorDiff);   // PoseOptimizer+968 = 1

  // the 80-byte DetectorOptions is copied before the allocation (by-value parameter)
  DetectorOptions detector_options2 = detector_options;
  depth_filter_ = std::unique_ptr<DepthFilter>(
      new DepthFilter(depthfilter_options, detector_options2, cams_));     // 0x18009F650 (0x160 bytes)

  initializer_ = initialization_utils::makeInitializer(
      init_options, tracker_options, detector_options, cams_);            // 0x18012BF30

  overlap_kfs_.resize(cams_->numCameras());
  last_overlap_kfs_.resize(cams_->numCameras());

  // QUIRK: division by zero if grid_size == 0; the bits are cleared twice.
  const int n_cells = (cams_->getCameraShared(0)->imageHeight() / options_.grid_size)
                      * (cams_->getCameraShared(0)->imageWidth() / options_.grid_size);
  grid_occupancy_.resize(n_cells, false);
  for (int i = 0; i < n_cells; ++i)
    grid_occupancy_[i] = false;

  trash_points_.resize(cams_->numCameras());
  track_ids_last_.resize(cams_->numCameras());
  track_ids_last_last_.resize(cams_->numCameras());

  loc_mode_ = loc_mode;
  if (!loc_mode)
    loadPriorPosition(&T_prior_, &prior_position_loaded_);              // 0x180115F00
}

// 0x1800E46A0 (scalar deleting dtor 0x1800EBFB0: calls this, then free() unless flag 4).
// upstream-modified (svo FrameHandlerBase::~FrameHandlerBase only logged "SVO destructor invoked").
// resetBackend() is a virtual call in a destructor => statically bound and fully inlined
// (identical to 0x18011B380, hence the "FrameProcessorBase resetBackend" string here).
// Implicit member destruction follows in reverse declaration order (mesher_ already released).
FrameProcessorBase::~FrameProcessorBase()
{
  is_destructing_ = true;     // +1616   TODO(verify) name
  resetBackend();
  savePriorPosition(T_world_correction_ * T_prior_);   // 0x180009B80 (kindr operator*), 0x180125040
  mesher_.reset();
}

// =============================================================================================
// input (c09)
// =============================================================================================

//------------------------------------------------------------------------------
// 0x1800FB650
// upstream: FrameHandlerBase::addImageBundle (modified: CLAHE for dark stereo pairs, Pimax
// Frame ctor, IMU stationarity detection, C-API output state, ground plane, loop closing).
bool FrameProcessorBase::addImageBundle(std::vector<cv::Mat>& imgs,
                                        const std::vector<uint32_t>& exposure_times,
                                        const std::vector<uint32_t>& gains,
                                        const uint64_t& timestamp,
                                        const ImuMeasurements& imu_measurements,
                                        const int& frame_flag,
                                        const Eigen::Quaternionf& imu_rotation,
                                        const bool has_imu_rotation,
                                        const double imu_rotation_timestamp)
{
  frame_flag_ = frame_flag;

  // Pimax: equalise dark image pairs (4 cameras = 2 stereo pairs).
  if (imgs.size() == 4)
  {
    for (size_t i = 0; i < imgs.size(); i += 2)
    {
      if (cv::mean(imgs[i])[0] < 25.0 && cv::mean(imgs[i + 1])[0] < 25.0)
      {
        cv::Mat img0;
        clahe_->apply(imgs[i], img0);
        imgs[i] = img0;
        cv::Mat img1;
        clahe_->apply(imgs[i + 1], img1);
        imgs[i + 1] = img1;
      }
    }
  }

  if (last_frames_)
  {
    // check if the timestamp is valid (signed compare, no CHECK(!frames_.empty()))
    if (last_frames_->frames_[0]->getTimestampNSec() >= static_cast<int64_t>(timestamp))
    {
      LOGE("Dropping frame: timestamp older than last frame.\n");
      return false;
    }
  }
  else
  {
    // at first iteration initialize tracing if enabled
    if (options_.trace_statistics && bundle_adjustment_)
      bundle_adjustment_->setPerformanceMonitor(options_.trace_dir);   // 0x180015DC0
  }
  if (options_.trace_statistics)
  {
    SVO_START_TIMER("pyramid_creation");
  }

  std::vector<FramePtr> frames;
  for (size_t i = 0; i < imgs.size(); ++i)
  {
    // n_pyr_levels = options_.img_align_max_level (+120 of this, passed unchanged)
    FramePtr frame = std::make_shared<Frame>(
          cams_->getCameraShared(i), imgs[i], timestamp, options_.img_align_max_level,
          static_cast<int>(i), exposure_times[i], gains[i], grid_occupancy_,
          options_.grid_size);                                            // 0x180091DA0
    // NCamera::get_T_C_B (0x1801B41F0, +0 vector) / get_T_B_C (0x1801B41E0, +24 vector).
    // The binary evaluates get_T_B_C first, then stores T_cam_body_ (+320), T_body_cam_ (+256).
    frame->set_T_cam_imu(cams_->get_T_C_B(i), cams_->get_T_B_C(i));
    frame->setNFrameIndex(i);                                             // Frame +36
    frames.push_back(std::move(frame));
  }
  FrameBundlePtr frame_bundle = std::make_shared<FrameBundle>(frames);   // 0x1800928D0
  frame_bundle->imu_measurements_ = imu_measurements;                    // deque operator= 0x1800E5AB0
  if (last_frames_)
  {
    frame_bundle->last_timestamp_sec_ = last_frames_->getMinTimestampSeconds();  // *1e-9
  }
  const bool imu_moving = checkImuMotion(frame_bundle->imu_measurements_);
  if (imu_not_initialized_ && !imu_moving)
  {
    // device at rest during IMU initialisation: use the mean gyro reading as gyro bias
    imu_handler_->omega_bias_ = Eigen::Vector3d(gyr_stat_[0].mean(),
                                                gyr_stat_[1].mean(),
                                                gyr_stat_[2].mean());   // ImuProcessor +232
  }
  frame_bundle->is_static_ = !imu_moving;                                // FrameBundle +228
  if (options_.trace_statistics)
  {
    SVO_STOP_TIMER("pyramid_creation");
  }

  // Process frame bundle.
  const bool res = addFrameBundle(frame_bundle, imu_rotation, has_imu_rotation,
                                  imu_rotation_timestamp);

  // QUIRK: four copies are taken (and only frame0 is used); at(1..3) throw for < 4 cameras
  const FramePtr frame0 = frame_bundle->at(0);
  const FramePtr frame1 = frame_bundle->at(1);
  const FramePtr frame2 = frame_bundle->at(2);
  const FramePtr frame3 = frame_bundle->at(3);

  const Eigen::Vector3d p_world_imu = frame0->imuPos().cast<double>();
  Eigen::Isometry3d T_world_imu;
  T_world_imu.translation() = p_world_imu;
  std::unique_lock<std::mutex> trajectory_lock(trajectory_mutex_);
  trajectory_lock.unlock();
  T_world_imu.linear() = Eigen::Quaterniond(frame0->T_world_imu().getRotation().w(),
                                            frame0->T_world_imu().getRotation().x(),
                                            frame0->T_world_imu().getRotation().y(),
                                            frame0->T_world_imu().getRotation().z())
                             .normalized().toRotationMatrix();
  Eigen::Isometry3d T_map_world;
  T_map_world.translation() = T_world_correction_.getPosition();
  T_map_world.linear() = T_world_correction_.getRotationMatrix();

  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    output_.timestamp = frame0->getTimestampSec();   // timestamp_ / 1e9 (division)
    if (imu_not_initialized_)
    {
      output_.confidence = 0.0f;
      output_.status = 1;
      output_.tracking_state = 4;
      if (dark_t_ > 1)
        output_.status = 2;
      if (bright_t_ > 1)
        output_.status = 3;
    }
    else
    {
      output_.T_world_imu = T_world_imu;
      output_.T_map_world = T_map_world;
      output_.velocity = frame_bundle->imu_vel_w_;
      output_.gyr_bias = frame_bundle->imu_gyr_bias_;
      output_.acc_bias = frame_bundle->imu_acc_bias_;
      if (output_.acc_bias.norm() > 0.5)
      {
        output_.acc_bias = Eigen::Vector3d::Zero();
        output_.gyr_bias = Eigen::Vector3d::Zero();
      }
      output_.backend_static = bundle_adjustment_->imu_motion_detector_stationary_;   // backend +160
      output_.gravity = frame_bundle->gravity_.cast<double>();
      output_.status = 0;
      output_.tracking_state = 4;
      output_.confidence = (stage_ == Stage::kTracking) ? 1.0f : 0.0f;
      if (frame_bundle->is_relocalized_)
        output_.tracking_state = 0;
    }
  }

  {
    std::lock_guard<std::mutex> lock(loc_mutex_);
    if (frame_bundle->is_relocalized_ && loc_mode_ == 2 && !loc_state_)
      loc_state_ = 1;
  }

  // Both results are unused (dead code apart from the Eigen alignment asserts).
  const Eigen::Vector3d p_map_imu = (T_map_world * T_world_imu).translation();
  const Eigen::Quaterniond q_map_imu((T_map_world * T_world_imu).linear());
  (void)p_map_imu;
  (void)q_map_imu;

  if (bundle_adjustment_ && !imu_not_initialized_ &&
      (stage_ == Stage::kTracking || stage_ == Stage::kRelocalization))
  {
    estimateGroundPlane(*frame_bundle);                                      // 0x1800F5170
    bundle_adjustment_->optimize(frame_bundle, frame_flag_, loc_mode_);      // 0x180013680
    const Eigen::Vector3d p_new = frame_bundle->get_T_W_B().getPosition();
    if ((p_new - p_world_imu).norm() > 0.15 && !frame_bundle->is_relocalized_)
    {
      p_diff_ = true;
      LOGW("big_position_diff %f\n", (p_new - p_world_imu).norm());
    }
  }

  if (options_.trace_statistics)
  {
    SVO_START_TIMER("feature_track_show");   // never stopped (upstream remnant)
  }

  Eigen::Isometry3d T_world_imu_pose;
  T_world_imu_pose.translation() = frame0->imuPos().cast<double>();
  const Eigen::Quaterniond q_world_imu(frame0->T_world_imu().getRotation().w(),
                                       frame0->T_world_imu().getRotation().x(),
                                       frame0->T_world_imu().getRotation().y(),
                                       frame0->T_world_imu().getRotation().z());
  T_world_imu_pose.linear() = q_world_imu.normalized().toRotationMatrix();
  const Eigen::Vector3d t_log = T_world_imu_pose.translation();

  trajectory_lock.lock();
  if (stage_ == Stage::kTracking)
  {
    trajectory_positions_.emplace_back(frame0->imuPos().cast<double>());
    if (frame_bundle->is_keyframe_)
      trajectory_poses_a_.push_back(T_world_imu_pose);
    else
      trajectory_poses_b_.push_back(T_world_imu_pose);
  }
  trajectory_lock.unlock();

  if (stage_ == Stage::kTracking && !imu_not_initialized_)
  {
    prior_position_loaded_ = true;
    T_prior_ = frame_bundle->get_T_W_B();
    prior_acc_bias_ = frame_bundle->imu_acc_bias_;
    prior_gyro_bias_ = frame_bundle->imu_gyr_bias_;
  }

  LOGD("low frequency pose timestamp %lu translation x %f y %f z %f q w %f x %f y %f z %f\n",
       frame0->getTimestampNSec(), t_log.x(), t_log.y(), t_log.z(),
       q_world_imu.w(), q_world_imu.x(), q_world_imu.y(), q_world_imu.z());

  if (stage_ == Stage::kTracking && !imu_not_initialized_ && frame0->numLandmarks() > 10 &&
      !frame_bundle->is_static_)
  {
    if (mesh_enabled_ && last_last_frames_ && mesh_update_count_ < 500)
    {
      updateGroundPlane(*last_last_frames_->at(0));                         // 0x1800F2070
      ++mesh_update_count_;
    }
  }

  std::unique_lock<std::mutex> plane_lock(plane_mutex_);
  if (stage_ != Stage::kTracking || imu_not_initialized_)
  {
    plane_valid_ = false;
  }
  else
  {
    plane_valid_ = false;
    double max_area = 0.0;
    for (Plane plane : planes_)       // copied (0x1800E2910 / ~Plane 0x1800E5230)
    {
      if (plane.area_ > max_area)
      {
        max_area = plane.area_;
        if (plane.area_ > 0.3)
        {
          plane_valid_ = true;
          plane_imu_pos_ = T_world_correction_.cast<float>() * frame0->imuPos();   // 0x18008A270 / 0x1800932D0
          plane_center_ = T_world_correction_.cast<float>() * plane.centroid_;
          if (std::fabs(plane_imu_pos_.z() - plane_center_.z()) > 3.0f)
            plane_valid_ = false;
        }
      }
    }
  }
  plane_lock.unlock();

  if (lc_ && last_last_frames_ && !imu_not_initialized_ && !reloc_enabled_)
  {
    if (reset_loop_closing_pending_)
    {
      lc_->resetLoopClosing();                                               // 0x18018C3A0
      reset_loop_closing_pending_ = false;
    }
    if (!last_frames_->is_static_ && frame_flag_ < 5)
    {
      double trans_diff = 0.0;
      double rot_diff = 0.0;
      lc_->computePoseDiff(&trans_diff, &rot_diff, last_frames_->get_T_W_B(),
                           last_last_frames_->get_T_W_B());                  // 0x180186F30
      if (trans_diff > 0.01)
        lc_->addFrameToPR(last_last_frames_, T_world_correction_);           // 0x180185E40
    }
  }
  return res;
}

//------------------------------------------------------------------------------
// 0x1800FA6B0
// upstream: FrameHandlerBase::addFrameBundle (heavily modified).
bool FrameProcessorBase::addFrameBundle(const FrameBundlePtr& frame_bundle,
                                        const Eigen::Quaternionf& imu_rotation,
                                        const bool has_imu_rotation,
                                        const double imu_rotation_timestamp)
{
  LOGD("New Frame Bundle received: %d\n", frame_bundle->getBundleId());

  // Pimax: keep the last few externally supplied IMU orientations while uninitialised.
  if (has_imu_rotation && imu_not_initialized_)
  {
    imu_rotation_buffer_.insert(std::make_pair(imu_rotation_timestamp, imu_rotation));
    if (imu_rotation_buffer_.size() > 10)
      imu_rotation_buffer_.erase(imu_rotation_buffer_.begin());
  }

  // ---------------------------------------------------------------------------
  // Prepare processing.
  if (update_res_ == UpdateResult::kFailure || set_reset_)
  {
    // Temporary copy rotation prior. (upstream also saved R_imu_world_)
    const bool have_rotation_prior = have_rotation_prior_;
    resetAll();
    have_rotation_prior_ = have_rotation_prior;
    setInitialPose(frame_bundle, imu_rotation, has_imu_rotation);
    stage_ = Stage::kInitializing;
  }

  if (options_.trace_statistics)
  {
    SVO_LOG("timestamp", frame_bundle->at(0)->getTimestampNSec());
    SVO_START_TIMER("frontend_time");
  }

  // ---------------------------------------------------------------------------
  // Add to pipeline.
  new_frames_ = frame_bundle;
  ++frame_counter_;

  if (options_.trace_statistics)
  {
    SVO_START_TIMER("frontend_update_map_from_ba");
  }
  if (!imu_not_initialized_ && bundle_adjustment_)
  {
    bundle_adjustment_->loadMapFromBundleAdjustment(new_frames_, last_frames_, map_,
                                                    have_motion_prior_, G, false);  // 0x180013480
    num_tracked_last_ = bundle_adjustment_->num_outliers_removed_;   // int at backend +164
  }
  if (options_.trace_statistics)
  {
    SVO_STOP_TIMER("frontend_update_map_from_ba");
  }

  // handle motion prior
  if (have_motion_prior_)
  {
    have_rotation_prior_ = true;
    R_imu_world_ = new_frames_->frames_[0]->T_imu_world().getRotation().toImplementation();
    if (last_frames_)
    {
      T_newimu_lastimu_prior_ =
          new_frames_->frames_[0]->T_imu_world() * last_frames_->get_T_W_B();
      have_motion_prior_ = true;
    }
  }
  else
  {
    // Predict pose of new frame using motion prior.
    if (last_frames_)
    {
      if (stage_ == Stage::kTracking)
      {
        num_tracked_last_ = last_frames_->numLandmarks();                     // 0x180095180
        LOGD("Predict pose of new image using motion prior.\n");
        getMotionPrior(false);

        // set initial pose estimate (Pimax: last_frames_->at(0) for every camera)
        const Transformation T_newimu_lastworld =
            T_newimu_lastimu_prior_ * last_frames_->at(0)->T_imu_world();
        for (size_t i = 0; i < new_frames_->size(); ++i)
        {
          new_frames_->at(i)->T_f_w_ = new_frames_->at(i)->T_cam_imu() * T_newimu_lastworld;
          new_frames_->at(i)->T_f_w_.getRotation().normalize();
        }
      }
    }
  }

  // Perform tracking.
  update_res_ = processFrameBundle();

  // Pimax: count consecutive observations of every tracked landmark (by bundle id, Frame +32).
  const int frame_id = new_frames_->at(0)->bundleId();
  for (const FramePtr& frame : new_frames_->frames_)
  {
    for (size_t i = 0; i < frame->num_features_; ++i)
    {
      if (frame->track_id_vec_(i) != -1)
      {
        const PointPtr& point = frame->landmark_vec_[i];
        if (point->last_obs_bundle_id_ < frame_id)             // Point +124
        {
          if (frame_id - point->last_obs_bundle_id_ == 1)
            ++point->n_consecutive_obs_;                       // Point +120
          else
            point->n_consecutive_obs_ = 1;
          point->last_obs_bundle_id_ = frame_id;
        }
      }
    }
  }

  // handle motion prior (Pimax: repeated after tracking)
  if (have_motion_prior_)
  {
    have_rotation_prior_ = true;
    R_imu_world_ = new_frames_->frames_[0]->T_imu_world().getRotation().toImplementation();
    if (last_frames_)
    {
      T_newimu_lastimu_prior_ =
          new_frames_->frames_[0]->T_imu_world() * last_frames_->get_T_W_B();
      have_motion_prior_ = true;
    }
  }

  if (!imu_not_initialized_ && bundle_adjustment_ &&
      (stage_ == Stage::kTracking || stage_ == Stage::kRelocalization))
  {
    reLocalize();
    const FramePtr& cur_frame = new_frames_->at(0);   // result unused (removed log?)
    (void)cur_frame;
  }
  checkTrackingHealth();

  if (options_.trace_statistics)
  {
    SVO_START_TIMER("frontend_prepare_data_for_ba");
  }
  // We start the backend first, since it is the most time critical
  if (!imu_not_initialized_ && bundle_adjustment_ &&
      (stage_ == Stage::kTracking || stage_ == Stage::kRelocalization))
  {
    // if we have loop closing module, see whether there is any correction we can do
    if (lc_)
    {
      std::lock_guard<std::mutex> lock(lc_->lc_info_lock_);   // LoopClosing +1568
      if (lc_->hasCorrectionInfo())                    // deque size at +1680
      {
        Transformation w_T_correction;
        lc_->consumeOldestCorrection(&w_T_correction);
        bundle_adjustment_->setCorrectionInWorld(w_T_correction);    // 0x180015870
        setRecovery(false);
      }
    }
    bundle_adjustment_->bundleAdjustment(new_frames_, frame_flag_, loc_mode_);  // 0x180011700
  }
  if (options_.trace_statistics)
  {
    SVO_STOP_TIMER("frontend_prepare_data_for_ba");
  }

  // Pimax: visual-inertial initialisation
  if (bundle_adjustment_ && imu_not_initialized_ && last_frames_ && stage_ == Stage::kTracking)
  {
    if (initializeImu(imu_rotation, has_imu_rotation, imu_rotation_timestamp))
    {
      LOGI("imu_initial true\n");
      imu_not_initialized_ = false;
    }
    bundle_adjustment_->imu_init_pending_ = imu_not_initialized_;          // backend +1241
  }

  // ---------------------------------------------------------------------------
  // Finish pipeline.
  if (last_frames_ && stage_ == Stage::kTracking)
  {
    // Set translation motion prior for next frame.
    t_lastimu_newimu_ = new_frames_->at(0)->T_imu_world().getRotation().rotate(
        new_frames_->at(0)->imuPos().cast<double>() -
        last_frames_->at(0)->imuPos().cast<double>());
  }

  // Statistics.
  num_obs_last_ = new_frames_->numTrackedFeatures();   // 0x180095360
  if (stage_ == Stage::kTracking)
  {
    if (isInRecovery())
    {
      if ((new_frames_->getMinTimestampSeconds() - last_good_tracking_time_sec_)
          < options_.global_map_lc_timeout_sec_)
      {
        setRecovery(false);
        last_good_tracking_time_sec_ = new_frames_->getMinTimestampSeconds();
      }
    }
    else
    {
      last_good_tracking_time_sec_ = new_frames_->getMinTimestampSeconds();
    }
  }

  // Set last frame.
  last_last_frames_ = last_frames_;
  last_frames_ = new_frames_;
  new_frames_.reset();

  // Reset if we should.
  if (set_reset_)
  {
    t_lastimu_newimu_ = Eigen::Vector3d::Zero();
    resetVisionFrontendCommonWhenSetStart();
    if (bundle_adjustment_)
      resetBackend();
    if (!imu_not_initialized_)
    {
      if (lc_)
      {
        LOGW("lc_->resetReLocalize()\n");
        lc_->resetReLocalize();                                  // 0x18018C4E0
      }
      reset_loop_closing_pending_ = true;
      reloc_enabled_ = true;
      ++reloc_session_count_;
    }
    imu_not_initialized_ = true;
    reloc_timer_.start();                                      // vk::Timer at +3984
  }

  // Reset rotation prior.
  have_rotation_prior_ = false;
  R_imulast_world_ = R_imu_world_;

  // Reset motion prior
  have_motion_prior_ = false;
  T_newimu_lastimu_prior_.setIdentity();

  // tracing
  if (options_.trace_statistics)
  {
    SVO_LOG("dropout", static_cast<int>(update_res_));
    SVO_STOP_TIMER("frontend_time");
    g_permon->writeToFile();                                   // 0x1801B58E0
  }
  return true;
}

//------------------------------------------------------------------------------
// always inlined in the binary (addFrameBundle: +3441 = 0, LoopClosing +1928/+1929 = 0 with
// one 16-bit store).  upstream-identical (FrameHandlerBase::setRecovery).
void FrameProcessorBase::setRecovery(const bool recovery)
{
  loss_without_correction_ = recovery;
  if (lc_)
  {
    lc_->setRecoveryMode(recovery);
  }
}

//------------------------------------------------------------------------------
// 0x18010A690  (vtable slot 6 of FrameProcessorBase and FrameProcessor)
// upstream: FrameHandlerBase::getMotionPrior (modified: IMU branch uses the FrameBundle IMU
// deque instead of imu_timestamps_ns_/imu_measurements_, iterates newest->oldest, aborts on
// non-increasing timestamps, uses the last bundle's gyro bias).
// QUIRK: with the newest-first deque the dt test fails at once ("IMU timestamps need to be
// strictly increasing.") and the pair (1,0) is never used.
void FrameProcessorBase::getMotionPrior(const bool /*use_velocity_in_frame*/)
{
  if (have_rotation_prior_)
  {
    LOGI("Get motion prior from provided rotation prior.\n");
    T_newimu_lastimu_prior_ =
        Transformation(R_imulast_world_ * R_imu_world_.inverse(), t_lastimu_newimu_).inverse();
    have_motion_prior_ = true;
  }
  else if (new_frames_->imu_measurements_.size() > 2)
  {
    LOGW("Get motion prior from integrated IMU measurements.\n");
    const ImuMeasurements& imu_measurements = new_frames_->imu_measurements_;
    const Eigen::Vector3d gyro_bias = last_frames_->imu_gyr_bias_;   // FrameBundle +168
    Eigen::Quaterniond delta_R = Eigen::Quaterniond::Identity();
    const size_t num_measurements = imu_measurements.size();
    for (size_t m_idx = num_measurements - 1u; m_idx != 1u; --m_idx)
    {
      const double delta_t_seconds =
          imu_measurements[m_idx].timestamp_ - imu_measurements[m_idx - 1u].timestamp_;
      if (delta_t_seconds < 1e-6)
      {
        LOGW("IMU timestamps need to be strictly increasing.\n");
        return;
      }
      const Eigen::Vector3d w =
          (imu_measurements[m_idx].angular_velocity_.cast<double>() - gyro_bias) * delta_t_seconds;
      // 1e-12-threshold exp inlined (common/transformation.h quaternionExp)
      const Eigen::Quaterniond R_incr = quaternionExp(w);
      delta_R = (delta_R * R_incr).normalized();
    }
    T_newimu_lastimu_prior_ = Transformation(delta_R, t_lastimu_newimu_).inverse();
    have_motion_prior_ = true;
  }
  else if (options_.poseoptim_prior_lambda > 0
           || options_.img_align_prior_lambda_rot > 0
           || options_.img_align_prior_lambda_trans > 0)
  {
    LOGD("Get motion prior by assuming constant velocity.\n");
    // binary: identity Eigen quaternion through the checking RotationQuaternion ctor 0x1800089C0
    T_newimu_lastimu_prior_ =
        Transformation(Eigen::Quaterniond::Identity(), t_lastimu_newimu_).inverse();
    have_motion_prior_ = true;
  }
}

//------------------------------------------------------------------------------
// 0x1800FE320
// pimax-new: sliding-window (<= 500 samples) gyro standard-deviation test.
// Returns true when the IMU is moving (QUIRK: also while the window holds < 200 samples).
bool FrameProcessorBase::checkImuMotion(const ImuMeasurements& imu_measurements)
{
  // oldest sample first (the bundle's deque is newest-first): reverse iterators, the binary passes
  // first = {end offset}, last = {begin offset} to deque::insert 0x1800D0A40
  imu_window_.insert(imu_window_.end(), imu_measurements.rbegin(), imu_measurements.rend());
  for (const ImuMeasurement& m : imu_measurements)
  {
    gyr_stat_[0].add(m.angular_velocity_.x());
    gyr_stat_[1].add(m.angular_velocity_.y());
    gyr_stat_[2].add(m.angular_velocity_.z());
  }
  while (imu_window_.size() > 500)
  {
    const ImuMeasurement& front = imu_window_.front();
    gyr_stat_[0].remove(front.angular_velocity_.x());
    gyr_stat_[1].remove(front.angular_velocity_.y());
    gyr_stat_[2].remove(front.angular_velocity_.z());
    imu_window_.pop_front();
  }
  if (imu_window_.size() < 200)
    return true;

  static const float kStdScale = std::sqrt(0.001);   // thread-safe static (0x18047DE98 / guard DE9C)
  const bool still = gyro_std_threshold_ > gyr_stat_[0].stddev() * kStdScale
                  && gyro_std_threshold_ > gyr_stat_[1].stddev() * kStdScale
                  && gyro_std_threshold_ > gyr_stat_[2].stddev() * kStdScale;
  return !still;
}

//------------------------------------------------------------------------------
// 0x180115F00   (called from the ctor 0x1800DFDF0 when loc_mode == 0:
//                loadPriorPosition(&T_prior_, &prior_position_loaded_))
// pimax-new.  Also writes the position into output_.T_world_imu.translation() (+896).
void FrameProcessorBase::loadPriorPosition(Transformation* T_prior, bool* loaded)
{
  const std::string path = options_.trace_dir + "/pimax_prior_position.txt";
  std::ifstream ifs(path);
  if (!ifs.is_open())
  {
    *loaded = false;
    return;
  }
  Eigen::Vector3f p;
  ifs >> p.x() >> p.y() >> p.z();
  if (ifs.fail())
  {
    std::cerr << "Failed to read prior position from file!" << std::endl;
    LOGE("Failed to read prior position from file!\n");
    *loaded = false;
  }
  else
  {
    const Eigen::Vector3d p_d = p.cast<double>();
    T_prior->getPosition() = p_d;
    output_.T_world_imu.translation() = p_d;     // +896
    *loaded = true;
    LOGW("Prior position loaded from %s\n", path.c_str());
  }
  ifs.close();
}

//------------------------------------------------------------------------------
// 0x180125040
// pimax-new: write the IMU position of the corrected pose; called from the dtor with
// T_world_correction_ * T_prior_.
void FrameProcessorBase::savePriorPosition(const Transformation& T_world_imu)
{
  const Eigen::Vector3d position = T_world_imu.getPosition();
  const std::string file_path = options_.trace_dir + "/pimax_prior_position.txt";
  LOGW("save pimax_prior_position.txt\n");
  std::ofstream file(file_path);
  if (file.is_open())
  {
    file << position.x() << " " << position.y() << " " << position.z() << std::endl;
    file.close();
  }
  else
  {
    std::cerr << "Failed to open file for saving prior position!" << std::endl;
  }
}

// =============================================================================================
// setters (c10)
// =============================================================================================

//------------------------------------------------------------------------------
// 0x180127550 (vtable slot 1)
// upstream setFirstFrames(); Map::addKeyframe lost its 2nd argument.
void FrameProcessorBase::setFirstFrames(const std::vector<FramePtr>& first_frames)
{
  resetAll();
  last_frames_.reset(new FrameBundle(first_frames));
  for (const FramePtr& f : last_frames_->frames_)
  {
    f->setKeyframe();
    map_->addKeyframe(f);
  }
  stage_ = Stage::kTracking;
}

//------------------------------------------------------------------------------
// 0x1801274A0
// upstream setBundleAdjuster() + null check.
void FrameProcessorBase::setBundleAdjuster(const std::shared_ptr<CeresBackendInterface>& ba)
{
  bundle_adjustment_ = ba;
  if (bundle_adjustment_)
    bundle_adjustment_type_ = bundle_adjustment_->type_;    // ba+976 (upstream getType())
  else
    bundle_adjustment_type_ = BundleAdjustmentType::kNone;
}

//------------------------------------------------------------------------------
// 0x1801277B0
// upstream setInitialPose() rewritten: gravity from the newest IMU sample in the bundle
// (needs > 10 samples), fixed reference axis z (no p/p_alternative switch), orthogonality check,
// optional external attitude (interpolated from imu_rotation_buffer_).  No rotation-prior branch.
void FrameProcessorBase::setInitialPose(const FrameBundlePtr& frame_bundle,
                                        const Eigen::Quaternionf& R_imu_world_att,
                                        bool use_attitude)
{
  if (frame_bundle->imu_measurements_.size() > 10)
  {
    LOGI("Set initial pose: Use inertial measurements in frame to get gravity\n");
    const Eigen::Vector3d g =
        frame_bundle->imu_measurements_.back().linear_acceleration_.cast<double>();
    const Eigen::Vector3d z = g.normalized(); // imu measures positive-z when static
    Eigen::Vector3d p(0, 0, 1);
    Eigen::Vector3d y = z.cross(p);
    y.normalize();
    const Eigen::Vector3d x = y.cross(z);
    Eigen::Matrix3d C_imu_world; // world unit vectors in imu coordinates
    C_imu_world.col(0) = x;
    C_imu_world.col(1) = y;
    C_imu_world.col(2) = z;
    if (std::fabs(C_imu_world.determinant() - 1.0) > 1e-5
        || !C_imu_world.col(0).isApprox(C_imu_world.col(1).cross(C_imu_world.col(2)), 1e-12))
    {
      LOGW("C_imu_world is not orthogonal matrix.\n");
      for (size_t i = 0; i < frame_bundle->frames_.size(); ++i)
      {
        frame_bundle->frames_.at(i)->T_f_w_ = cams_->get_T_C_B(i) * T_world_imuinit.inverse();
        frame_bundle->frames_.at(i)->T_f_w_.getRotation().normalize();
      }
    }
    else
    {
      Eigen::Quaterniond q_imu_world(C_imu_world);
      if (use_attitude)
      {
        q_imu_world = R_imu_world_att.cast<double>();
        if (imu_rotation_buffer_.size() > 2)
        {
          const double t = frame_bundle->frames_[0]->getTimestampSec();
          auto it = imu_rotation_buffer_.lower_bound(t);
          if (it != imu_rotation_buffer_.begin())
          {
            // QUIRK: `it` may be end() (all stored attitudes older than the frame); the
            // header node's key/value are then read (MSVC: uninitialised storage).
            auto prev = std::prev(it);
            Eigen::Quaternionf q = slerpByTime(prev->second, it->second,
                                               prev->first, it->first, t);
            q.normalize();
            q_imu_world = q.cast<double>();
          }
        }
      }
      for (size_t i = 0; i < frame_bundle->frames_.size(); ++i)
      {
        Transformation T_imu_world(Transformation::Rotation(q_imu_world), Eigen::Vector3d::Zero());
        frame_bundle->at(i)->T_f_w_ = cams_->get_T_C_B(i) * T_imu_world;
        frame_bundle->at(i)->T_f_w_.getRotation().normalize();
      }
    }
  }
  else
  {
    LOGW("Set initial pose: set such that T_imu_world is identity.\n");
    for (size_t i = 0; i < frame_bundle->frames_.size(); ++i)
    {
      frame_bundle->frames_.at(i)->T_f_w_ = cams_->get_T_C_B(i) * T_world_imuinit.inverse();
      frame_bundle->frames_.at(i)->T_f_w_.getRotation().normalize();
    }
  }
}

//------------------------------------------------------------------------------
// 0x1801280A0
// upstream setRotationIncrementPrior(); VLOG -> LOGI.  The priors are plain Eigen::Quaterniond in
// the fork: inverse() divides by the squared norm (Eigen), the product has no kindr renormalisation.
void FrameProcessorBase::setRotationIncrementPrior(const Eigen::Quaterniond& R_lastimu_newimu)
{
  LOGI("Set rotation increment prior.\n");
  R_imu_world_ = R_lastimu_newimu.inverse() * R_imulast_world_;
  have_rotation_prior_ = true;
}

//------------------------------------------------------------------------------
// 0x180128210
// upstream setRotationPrior(); VLOG -> LOGI.
void FrameProcessorBase::setRotationPrior(const Eigen::Quaterniond& R_imu_world)
{
  LOGI("Set rotation prior.\n");
  R_imu_world_ = R_imu_world;
  have_rotation_prior_ = true;
}

//------------------------------------------------------------------------------
// 0x180128260 (vtable slot 5)
// upstream setTrackingQuality(): the "less than quality_min_fts" warning was removed, the drop test
// uses the bundle-id distance to the last keyframe bundle instead of last_frames_->isKeyframe(),
// and a feature-drop counter (feature_d_m_) feeds the watchdog.
// QUIRK: last_kf_frames_ is dereferenced without a check; "%lu" with an int and two spaces.
void FrameProcessorBase::setTrackingQuality(const size_t num_observations)
{
  tracking_quality_ = TrackingQuality::kGood;
  if (num_observations < options_.quality_min_fts)
  {
    tracking_quality_ = TrackingQuality::kInsufficient;
  }
  const int feature_drop = static_cast<int>(num_obs_last_) - num_observations;
  // seeds are extracted at keyframe,
  // so the number is not indicative of tracking quality
  if (std::abs(last_kf_frames_->getBundleId() - new_frames_->getBundleId()) > 4 &&
      feature_drop > options_.quality_max_fts_drop)
  {
    LOGW("Lost %lu  features!\n", feature_drop);
    tracking_quality_ = TrackingQuality::kInsufficient;
  }
  if (feature_drop > options_.quality_max_fts_drop)
    ++feature_d_m_;
  else
    feature_d_m_ = 0;
}

// =============================================================================================
// tracking (c10)
// =============================================================================================

//------------------------------------------------------------------------------
// 0x180118740
// upstream optimizePose(); trace/permon removed, SVO_DEBUG_STREAM -> LOGD, run() got an extra flag.
size_t FrameProcessorBase::optimizePose(bool use_weighted_prior)
{
  // pose optimization
  // optimize the pose of the frame in such a way, that the projection of all feature world
  // coordinates is not far off the position of the feature points within the frame.
  pose_optimizer_->reset();   // inlined: chi2_=1e10, mu_=mu_init, nu_=nu_init, n_meas_=0,
                              //          n_iter_=0, iter_=0, stop_=false, have_prior_=false
  if (have_motion_prior_)
  {
    LOGD("Apply prior to pose optimization\n");
    pose_optimizer_->setRotationPrior(
        new_frames_->get_T_W_B().getRotation().toImplementation().inverse(),
        options_.poseoptim_prior_lambda);
  }
  size_t sfba_n_edges_final =
      pose_optimizer_->run(new_frames_, options_.poseoptim_thresh, use_weighted_prior);

  // QUIRK: nObs is printed with %f but passed as size_t.
  LOGD("PoseOptimizer:\n\t ErrInit = %f\n\t ErrFin = %f\n\t nObs = %f",
       pose_optimizer_->stats_.reproj_error_before,
       pose_optimizer_->stats_.reproj_error_after,
       sfba_n_edges_final);
  return sfba_n_edges_final;
}

// =============================================================================================
// resets (c10)
// =============================================================================================

// 0x18011B370 (vtable slot 3 of FrameProcessorBase) is the header-inline
//   virtual void resetAll() { resetVisionFrontendCommon(); }

//------------------------------------------------------------------------------
// 0x18011B380 (vtable slot 4; also inlined into the dtor 0x1800E46A0)
// upstream resetBackend(): the backend is only reset, never released; IMU statistics cleared.
void FrameProcessorBase::resetBackend()
{
  if (bundle_adjustment_ && bundle_adjustment_type_ != BundleAdjustmentType::kNone)
  {
    bundle_adjustment_->reset();          // 0x1800149F0 LOGI("Backend: Reset\n")
    bundle_adjustment_->clearBackend();   // 0x180012B20
  }
  imu_window_.clear();
  for (RunningStats& s : gyr_stat_)
    s = RunningStats();
  LOGI("FrameProcessorBase resetBackend\n");
}

//------------------------------------------------------------------------------
// 0x18011B420
// upstream resetVisionFrontendCommon(): no reloc_keyframe_/sparse_img_align_, no set_start_,
// restores the IMU biases from imu_params_, clears Pimax state (VI-init buffers, track-id sets,
// watchdog counters, ground plane, backend ground-plane constraint).
// QUIRK: last_last_frames_, last_kf_frames_ and the relocalisation state are not touched.
void FrameProcessorBase::resetVisionFrontendCommon()
{
  stage_ = Stage::kPaused;
  tracking_quality_ = TrackingQuality::kInsufficient;
  set_reset_ = false;
  num_obs_last_ = 0;
  t_lastimu_newimu_ = Eigen::Vector3d::Zero();
  have_motion_prior_ = false;
  T_newimu_lastimu_prior_.setIdentity();
  have_rotation_prior_ = false;
  for (auto& frame_vec : overlap_kfs_)
  {
    frame_vec.clear();
  }
  imu_handler_->acc_bias_ = imu_params_.aBias.cast<double>();
  imu_handler_->omega_bias_ = imu_params_.wBias.cast<double>();
  keyframe_counter_ = 0;
  frame_bundle_map_.clear();
  all_image_frame_.clear();
  bundle_buffer_.clear();

  new_frames_.reset();
  last_frames_.reset();
  map_->reset();

  depth_filter_->reset();
  initializer_->reset();

  for (auto& point_vec : trash_points_)
    point_vec.clear();
  for (auto& ids : track_ids_last_)
    ids.clear();
  for (auto& ids : track_ids_last_last_)
    ids.clear();

  low_q_num_ = 0;
  low_m_r_num_ = 0;
  vel_fly_num_ = 0;
  feature_d_m_ = 0;
  v_fast_times_ = 0;
  num_tracked_last_ = 0;
  low_m_ba_ = 0;
  low_m_f_ = 0;
  c_kf_ = 0;
  b_dist_ = 0;
  low_marks_ = 0;

  lmk_points_.clear();
  planes_.clear();
  other_planes_.clear();
  mesh_enabled_ = true;
  mesh_update_count_ = 0;
  ground_valid_ = false;
  ground_confirmations_ = 0;
  ground_miss_count_ = 0;
  ground_normal_ = Eigen::Vector3d(0.0, 0.0, 1.0);
  ground_distance_ = 0.0;
  ground_sigma_ = 0.08;
  ground_polygon_.clear();
  ground_area_ = 0.0;
  ground_rel_init_ = false;
  ground_rel_bundle_id_ = -1;
  ground_rel_normal_b_ = Eigen::Vector3d(0.0, 0.0, 1.0);
  ground_rel_normal_w_ = Eigen::Vector3d(0.0, 0.0, 1.0);
  ground_rel_sigma_ = 0.15;
  ground_rel_ids_.clear();

  if (bundle_adjustment_)
    bundle_adjustment_->backend_.resetGroundPlaneConstraint();   // 0x1800298A0 on (ba+176)

  LOGI("resetVisionFrontendCommon\n");
}

//------------------------------------------------------------------------------
// 0x18011B8F0
// pimax-new: like resetVisionFrontendCommon() but keeps set_reset_, c_kf_, the ground-plane state
// and the backend.  Called from addFrameBundle when set_reset_ is true.
void FrameProcessorBase::resetVisionFrontendCommonWhenSetStart()
{
  stage_ = Stage::kPaused;
  tracking_quality_ = TrackingQuality::kInsufficient;
  num_obs_last_ = 0;
  t_lastimu_newimu_ = Eigen::Vector3d::Zero();
  have_motion_prior_ = false;
  T_newimu_lastimu_prior_.setIdentity();
  have_rotation_prior_ = false;
  for (auto& frame_vec : overlap_kfs_)
  {
    frame_vec.clear();
  }
  imu_handler_->acc_bias_ = imu_params_.aBias.cast<double>();
  imu_handler_->omega_bias_ = imu_params_.wBias.cast<double>();
  keyframe_counter_ = 0;
  frame_bundle_map_.clear();
  all_image_frame_.clear();
  bundle_buffer_.clear();

  new_frames_.reset();
  last_frames_.reset();
  map_->reset();

  depth_filter_->reset();
  initializer_->reset();

  for (auto& point_vec : trash_points_)
    point_vec.clear();
  for (auto& ids : track_ids_last_)
    ids.clear();
  for (auto& ids : track_ids_last_last_)
    ids.clear();

  low_q_num_ = 0;
  low_m_r_num_ = 0;
  vel_fly_num_ = 0;
  feature_d_m_ = 0;
  v_fast_times_ = 0;
  num_tracked_last_ = 0;
  low_m_ba_ = 0;
  low_m_f_ = 0;
  b_dist_ = 0;
  low_marks_ = 0;

  LOGI("resetVisionFrontendCommonWhenSetStart\n");
}

// =============================================================================================
// relocalisation / watchdog (c10)
// =============================================================================================

//------------------------------------------------------------------------------
// 0x18011AA90
// pimax-new: relocalisation against the loaded PlatMap through the loop-closing module.
// c10 names -> LoopClosing (c16): enable_reloc_ (+953) = options_.use_plat_map,
// plat_map_->keyframes_.size() ((lc_+24)->+96) = plat_map_->kf_map_.size(),
// reLocalize(info*) 0x18018B600 = getReLocCorrection, addReLocFrames 0x180186A50 = reLocalize,
// reloc_kf_id_ (+16) = map_id_, info.T_correction / kf_id = w_T_new_old_ / map_id_.
void FrameProcessorBase::reLocalize()
{
  LOGI("input reLocalize\n");
  if (!last_frames_ || !last_last_frames_ || imu_not_initialized_)
    return;

  const FramePtr& frame0 = new_frames_->frames_.at(0);
  if (frame0->mean_intensity_ < 25.0 || frame0->numLandmarks() < 30)   // Frame 0x180118580
    return;
  if (!lc_)
    return;
  if (!lc_->options_.use_plat_map || reloc_session_count_ <= 0)      // lc_+953
    return;

  if (lc_->plat_map_->kf_map_.size() == 0)                           // (lc_+24)->(+96)
  {
    reloc_enabled_ = false;
    reLoc_times_ = 0;
    return;
  }

  // NOTE: written as !(diff < interval) -- the binary proceeds when the difference is NaN.
  if (!imu_not_initialized_ && reloc_enabled_
      && !(new_frames_->frames_[0]->getTimestampSec() - last_reloc_success_time_
           < reloc_success_interval_)
      && !(last_frames_->frames_[0]->getTimestampSec() - last_reloc_try_time_
           < reloc_try_interval_))
  {
    ReLocCorrectionInfo reloc_info;
    if (lc_->getReLocCorrection(&reloc_info))                       // 0x18018B600
    {
      LOGW("reLoc_correction_info_.size(): %d\n", lc_->reLoc_correction_info_.size());
      T_world_correction_ = reloc_info.w_T_new_old_ * T_world_correction_;
      new_frames_->is_relocalized_ = true;                          // FrameBundle+64
      last_reloc_success_time_ = new_frames_->frames_[0]->getTimestampSec();
      lc_->map_id_ = reloc_info.map_id_;                            // lc_+16
      reloc_enabled_ = false;
      reLoc_times_ = 0;
      lc_->reLoc_correction_info_.clear();                          // lc_+232 (std::deque)
      t2_reLocSuc_ = reloc_timer_.stop();                           // 0x18002CC00
      LOGW("reLocalize success, INFO=ReLocTimer||t1_voImuInit=%f||t2_reLocSuc=%f\n",
           t1_voImuInit_, t2_reLocSuc_);
    }
    else
    {
      lc_->reLocalize(last_frames_, T_world_correction_);           // 0x180186A50
      ++reLoc_times_;
      last_reloc_try_time_ = last_frames_->frames_[0]->getTimestampSec();
      if (reLoc_times_ > max_reLoc_times_ && loc_mode_ != 2)
      {
        LOGI("reloc fail, reLoc_times_ %d > max_reLoc_times_ %d\n", reLoc_times_, max_reLoc_times_);
        reloc_enabled_ = false;
        reLoc_times_ = 0;
      }
    }
  }
}

//------------------------------------------------------------------------------
// 0x18011BCA0
// pimax-new: tracking-health watchdog, requests a reset (set_reset_) when tracking looks broken.
// TODO(verify) function name.  FrameBundle counters by address: 0x180095180 numLandmarks (track
// ids), 0x180095240 numLandmarksInBA, 0x1800952D0 numTrackedEdgelets, 0x1800953F0
// numTrackedLandmarks (c10 called them numTrackedIds / - / numTrackedLandmarks / numLandmarks).
// QUIRK: dark_t_, bright_t_, low_marks_ and low_c_ survive a reset; "acc_b_n" is always 0.0.
void FrameProcessorBase::checkTrackingHealth()
{
  double max_brightness = 0.0;
  double min_brightness = 255.0;
  for (const FramePtr& frame : new_frames_->frames_)
  {
    max_brightness = std::max(max_brightness, frame->mean_intensity_);
    min_brightness = std::min(min_brightness, frame->mean_intensity_);
  }
  if (max_brightness < 15.0)
    ++dark_t_;
  else
    dark_t_ = 0;
  if (min_brightness > 220.0)
    ++bright_t_;
  else
    bright_t_ = 0;

  if (!last_frames_ || !last_last_frames_)
    return;

  const int64_t t_diff_ns = std::abs(new_frames_->frames_[0]->getTimestampNSec()
                                     - last_frames_->frames_[0]->getTimestampNSec());
  int low_q_num = 0;
  int vel_fly_num = 0;
  if (stage_ == Stage::kTracking)
  {
    if (num_tracked_last_ < 20)
      low_q_num = ++low_q_num_;
    else
      low_q_num_ = 0;
    if (new_frames_->imu_vel_w_.norm() > 4.5)
      vel_fly_num = ++vel_fly_num_;
    else
      vel_fly_num_ = 0;
  }
  else
  {
    low_q_num_ = 0;
    vel_fly_num_ = 0;
  }
  const int low_m_r_num = low_m_r_num_;

  // speed between last and new pose
  double dist = 0.0;
  double angle = 0.0;
  const Transformation T_W_B_last = last_frames_->frames_[0]->T_world_imu();
  const Transformation T_W_B_new = new_frames_->frames_[0]->T_world_imu();
  computePoseDifference(&dist, &angle, T_W_B_new, T_W_B_last);
  bool b_big_draft = false;
  if (dist / (new_frames_->frames_[0]->getTimestampSec()
              - last_frames_->frames_[0]->getTimestampSec() + 1e-6) > 4.5)
  {
    if (v_fast_times_++ >= 12)
      b_big_draft = true;
  }
  else
  {
    v_fast_times_ = 0;
  }
  if (!b_big_draft && feature_d_m_ > 6)
    b_big_draft = true;

  const bool b_light = stage_ == Stage::kTracking && (dark_t_ > 6 || bright_t_ > 6);

  if (imu_not_initialized_ || last_frames_->is_static_ || last_frames_->numLandmarksInBA() >= 10)
    low_m_ba_ = 0;
  else
    ++low_m_ba_;

  // number of cameras without any tracked feature
  int n_empty_frames = 0;
  for (size_t cam = 0; cam < new_frames_->frames_.size(); ++cam)
  {
    if (new_frames_->frames_.at(cam)->numLandmarks() == 0)   // Frame 0x180118580 (inlined here)
      ++n_empty_frames;
  }
  if (n_empty_frames < 3)
    e_f_f_ = 0;
  else
    ++e_f_f_;

  if (new_frames_->is_keyframe_)
  {
    if (new_frames_->low_feature_kf_ && new_frames_->imu_vel_w_.norm() > 0.1)
      ++low_m_f_;
    else
      low_m_f_ = 0;
  }
  if (new_frames_->is_keyframe_ && stage_ == Stage::kTracking && !imu_not_initialized_
      && !new_frames_->is_static_)
    ++c_kf_;
  else
    c_kf_ = 0;

  if (static_cast<double>(new_frames_->numTrackedEdgelets())             // 0x1800952D0
        > static_cast<double>(new_frames_->numTrackedLandmarks()) * 0.8  // 0x1800953F0
      && stage_ == Stage::kTracking)
    ++b_dist_;
  else
    b_dist_ = 0;

  if (new_frames_->numLandmarks() >= 10)          // FrameBundle 0x180095180 (track ids > -1)
    low_marks_ = 0;
  else
    ++low_marks_;

  double delt_z = 0.0;
  if (!imu_not_initialized_)
  {
    const Transformation T_W_B = new_frames_->frames_.at(0)->T_world_imu();
    delt_z = std::abs(imu_init_pos_.z() - static_cast<float>(T_W_B.getPosition().z()));   // float
  }

  // common points between the new and the last-last bundle
  std::set<int> new_point_ids;
  for (const FramePtr& frame : new_frames_->frames_)
  {
    for (size_t i = 0; i < frame->num_features_; ++i)
    {
      if (frame->track_id_vec_(i) != -1)
        new_point_ids.insert(frame->landmark_vec_[i]->id());
    }
  }
  std::set<int> last_last_point_ids;
  for (const FramePtr& frame : last_last_frames_->frames_)
  {
    for (size_t i = 0; i < frame->num_features_; ++i)
    {
      if (frame->track_id_vec_(i) != -1)
        last_last_point_ids.insert(frame->landmark_vec_[i]->id());
    }
  }
  int n_common = 0;
  for (const int id : new_point_ids)
  {
    if (last_last_point_ids.find(id) != last_last_point_ids.end())
      ++n_common;
  }
  if (!imu_not_initialized_ && new_frames_->imu_vel_w_.norm() > 1.0 && n_common < 30)
    ++low_c_;
  else
    low_c_ = 0;

  if (t_diff_ns > 165000000
      || low_q_num > 10
      || vel_fly_num > 12
      || low_m_r_num > 5
      || b_big_draft
      || b_light
      || frame_flag_ > 75
      || low_m_ba_ > 10
      || low_m_f_ > 6
      || b_dist_ > 6
      || low_marks_ > 6
      || delt_z > 5.0
      || low_c_ > 10
      || e_f_f_ > 6
      || p_diff_
      || c_kf_ > 10
      || (imu_not_initialized_ && new_frames_->numLandmarks() <= 10))
  {
    LOGW("Reset b_m_lost_r %d, t_diff %f, b_t_less %d, low_q_num %d, b_vel_fly %d low_m_r_num %d "
         "b_big_draft %d feature_d_m %d v_fast_times_%d dark_t_ %d low_m_ba %d acc_b_n %f "
         "low_m_f %d b_dist %d low_marks %d delt_z %f low_c %d e_f_f %d p_diff %d c_kf %d\n",
         t_diff_ns > 165000000,
         static_cast<double>(t_diff_ns) * 1e-6,
         low_q_num > 10,
         low_q_num_,
         vel_fly_num > 12,
         low_m_r_num_,
         b_big_draft,
         feature_d_m_,
         v_fast_times_,
         dark_t_,
         low_m_ba_,
         0.0,
         low_m_f_,
         b_dist_,
         low_marks_,
         delt_z,
         low_c_,
         e_f_f_,
         p_diff_,
         c_kf_);
    if (imu_not_initialized_ && new_frames_->numLandmarks() <= 10)
    {
      LOGW("Reset vo reset\n");
    }
    set_reset_ = true;
    low_q_num_ = 0;
    low_m_r_num_ = 0;
    vel_fly_num_ = 0;
    feature_d_m_ = 0;
    v_fast_times_ = 0;
    num_tracked_last_ = 0;
    low_m_ba_ = 0;
    low_m_f_ = 0;
    c_kf_ = 0;
    b_dist_ = 0;
    e_f_f_ = 0;
    p_diff_ = false;
  }
}

// =============================================================================================
// map projection / structure (c10; glog __LINE__s of the original file)
// =============================================================================================

//------------------------------------------------------------------------------
// 0x180119260   (glog lines 2417, 2568; async worker lambda body 0x1800E61D0)
// upstream projectMapInFrame(), heavily modified:
//  * static bundles (new_frames_->is_static_) use half of the reprojector's max_n_kfs and, unless
//    the last KF bundle was a keyframe or the IMU is not initialised yet, reuse last_overlap_kfs_.
//  * the last keyframe bundle's frame is always prepended to the overlap list.
//  * trash points go to the member trash_points_ (not a local).
//  * features whose track id disappeared in the last frame but existed the frame before are dropped.
//  * returns a ReprojectResult instead of the number of features.
// QUIRKS: the "reproject" timer is never stopped; last_kf_frames_ is not null-checked; trash
// points are only deleted from the map for static bundles; the async path dedupes/sorts the
// overlap keyframes, the synchronous path does not.
ReprojectResult FrameProcessorBase::projectMapInFrame()
{
  ReprojectResult result;
#line 2417
  VLOG(40) << "Project map in frame.";
  if (options_.trace_statistics)
  {
    SVO_START_TIMER("reproject");   // no matching stop in this function
  }

  // compute overlap keyframes
  if (!new_frames_->is_static_ || last_kf_frames_->is_keyframe_ || imu_not_initialized_)
  {
    for (size_t camera_idx = 0; camera_idx < cams_->numCameras(); ++camera_idx)
    {
      std::unique_ptr<Reprojector>& cur_reprojector = reprojectors_.at(camera_idx);
      size_t max_n_kfs = cur_reprojector->options_.max_n_kfs;      // Reprojector+32
      if (new_frames_->is_static_)
        max_n_kfs = max_n_kfs >> 1;
      overlap_kfs_.at(camera_idx).clear();
      map_->getClosestNKeyframesWithOverlap(
            new_frames_->at(camera_idx),
            max_n_kfs,
            &overlap_kfs_.at(camera_idx));
    }
  }
  else
  {
    overlap_kfs_ = last_overlap_kfs_;
  }

  if (options_.use_async_reprojectors)
  {
    // start reprojection workers
    std::vector<std::future<void>> reprojector_workers;
    for (size_t camera_idx = 0; camera_idx < cams_->numCameras(); ++camera_idx)
    {
      // lambda body: 0x1800E61D0
      auto func = [this, camera_idx]()
      {
        std::vector<FramePtr>& kfs = overlap_kfs_.at(camera_idx);
        kfs.insert(kfs.begin(), last_kf_frames_->at(camera_idx));
        std::unordered_set<FramePtr> unique_kfs(overlap_kfs_.at(camera_idx).begin(),
                                                overlap_kfs_.at(camera_idx).end());
        overlap_kfs_.at(camera_idx).assign(unique_kfs.begin(), unique_kfs.end());
        std::sort(overlap_kfs_.at(camera_idx).begin(), overlap_kfs_.at(camera_idx).end(),
                  [](const FramePtr& lhs, const FramePtr& rhs) { return lhs->id_ > rhs->id_; });
        reprojectors_.at(camera_idx)->reprojectFrames(
              new_frames_->at(camera_idx), overlap_kfs_.at(camera_idx),
              trash_points_.at(camera_idx), imu_not_initialized_);
      };
      reprojector_workers.push_back(std::async(std::launch::async, func));
    }

    // make sure all of them are finished
    for (size_t i = 0; i < reprojector_workers.size(); ++i)
      reprojector_workers[i].get();
  }
  else
  {
    for (size_t camera_idx = 0; camera_idx < cams_->numCameras(); ++camera_idx)
    {
      std::vector<FramePtr>& kfs = overlap_kfs_.at(camera_idx);
      kfs.insert(kfs.begin(), last_kf_frames_->at(camera_idx));
      reprojectors_.at(camera_idx)->reprojectFrames(
            new_frames_->at(camera_idx), overlap_kfs_.at(camera_idx),
            trash_points_.at(camera_idx), imu_not_initialized_);
    }
  }
  last_overlap_kfs_ = overlap_kfs_;

  // Drop features whose track id was seen two frames ago but not in the last frame.
  std::vector<std::set<int>> cur_track_ids;
  cur_track_ids.resize(cams_->numCameras());
  for (size_t camera_idx = 0; camera_idx < cams_->numCameras(); ++camera_idx)
  {
    FramePtr frame = new_frames_->at(camera_idx);
    for (size_t i = 0; i < frame->num_features_; ++i)
    {
      if (frame->track_id_vec_(i) == -1)
        continue;
      const int track_id = frame->track_id_vec_(i);
      std::set<int>::const_iterator it_last_last =
          track_ids_last_last_.at(camera_idx).find(track_id);
      std::set<int>::const_iterator it_last = track_ids_last_.at(camera_idx).find(track_id);
      if (it_last_last != track_ids_last_last_[camera_idx].end()
          && it_last == track_ids_last_[camera_idx].end()
          && (last_frames_->is_static_ || redundant_kf_count_ > 6))
      {
        if (frame->landmark_vec_[i])
          trash_points_.at(camera_idx).push_back(frame->landmark_vec_[i]);
        frame->landmark_vec_[i] = nullptr;
        frame->track_id_vec_(i) = -1;
      }
      else
      {
        cur_track_ids.at(camera_idx).insert(track_id);
      }
    }
  }
  track_ids_last_last_.swap(track_ids_last_);
  track_ids_last_.swap(cur_track_ids);

  // Effectively clear the points that were discarded by the reprojectors
  // QUIRK: only for static bundles; otherwise the trash points are just forgotten.
  std::set<PointPtr> trash_set = collectTrashPoints(new_frames_->is_static_, trash_points_);
  if (new_frames_->is_static_)
  {
    for (const FramePtr& frame : new_frames_->frames_)
    {
      for (size_t i = 0; i < frame->num_features_; ++i)
      {
        if (frame->track_id_vec_(i) == -1)
          continue;
        PointPtr point = frame->landmark_vec_.at(i);
        if (trash_set.find(point) != trash_set.end())
        {
          frame->landmark_vec_.at(i) = nullptr;
          frame->track_id_vec_(i) = -1;
        }
      }
    }
    if (!frame_bundle_map_.empty())
    {
      for (auto it = frame_bundle_map_.begin(); it != frame_bundle_map_.end(); ++it)
      {
        FrameBundlePtr bundle = it->second;
        for (FramePtr frame : bundle->frames_)
        {
          for (size_t i = 0; i < frame->num_features_; ++i)
          {
            if (frame->track_id_vec_(i) == -1)
              continue;
            PointPtr point = frame->landmark_vec_.at(i);
            if (trash_set.find(point) != trash_set.end())
            {
              frame->landmark_vec_.at(i) = nullptr;
              frame->track_id_vec_(i) = -1;
            }
          }
        }
      }
    }
    for (PointPtr point : trash_set)
      map_->safeDeletePoint(point);
  }
  for (auto& point_vec : trash_points_)
    point_vec.clear();

  // Count the total number of trials and matches for all reprojectors
  Reprojector::Statistics cumul_stats_;
  Reprojector::Statistics cumul_stats_global_map;
  for (const std::unique_ptr<Reprojector>& reprojector : reprojectors_)
  {
    cumul_stats_.n_matches += reprojector->stats_.n_matches;                              // +96
    cumul_stats_.n_trials += reprojector->stats_.n_trials;                                // +104
    cumul_stats_.sum_lm_succeeded_reproj += reprojector->stats_.sum_lm_succeeded_reproj;  // +112
    cumul_stats_.sum_lm_obs += reprojector->stats_.sum_lm_obs;                            // +120
    cumul_stats_.n_lm_matches += reprojector->stats_.n_lm_matches;                        // +128
    cumul_stats_.n_seed_matches += reprojector->stats_.n_seed_matches;                    // +136
    cumul_stats_global_map.n_matches += reprojector->fixed_lm_stats_.n_matches;           // +144
  }

#line 2568
  VLOG(40) << "Reprojection:" << "\t nPoints = " << cumul_stats_.n_trials << "\t\t nMatches = "
      << cumul_stats_.n_matches;

  size_t n_total_ftrs = cumul_stats_.n_matches +
      (cumul_stats_global_map.n_matches <= 10 ? 0 : cumul_stats_global_map.n_matches);

  if (n_total_ftrs < options_.quality_min_fts)
  {
    LOGW("Not enough matched features: %lu\n", n_total_ftrs);
  }

  result.n_matches = cumul_stats_.n_matches;
  result.n_trials = cumul_stats_.n_trials;
  result.ave_success_num = static_cast<float>(cumul_stats_.sum_lm_succeeded_reproj)
      / static_cast<float>(cumul_stats_.n_matches);
  result.n_seed_matches = cumul_stats_.n_seed_matches;
  result.n_lm_matches = cumul_stats_.n_lm_matches;
  result.ave_lm_obs = static_cast<float>(cumul_stats_.sum_lm_obs)
      / static_cast<float>(cumul_stats_.n_matches);
  return result;
}

//------------------------------------------------------------------------------
// 0x180118D80   (glog line 2619)
// upstream optimizeStructure(): no nth_element / max_n_pts limit any more (max_n_pts only gates),
// no last_structure_optim_ update, no omni check (always on unit plane), track_id instead of the
// landmark null check, and a set prevents optimising a point twice over the bundle.
void FrameProcessorBase::optimizeStructure(const FrameBundlePtr& frames, int max_n_pts, int max_iter)
{
#line 2619
  VLOG(40) << "Optimize structure.";
  // some feature points will be optimized w.r.t keyframes they were observed
  // in the way that their projection error into all other keyframes is minimzed

  if (max_n_pts == 0)
    return; // don't return if max_n_pts == -1, this means we optimize ALL points

  std::unordered_set<PointPtr> optimized_points;
  for (const FramePtr& frame : frames->frames_)
  {
    std::deque<PointPtr> pts;
    for (size_t i = 0; i < frame->num_features_; ++i)
    {
      if (optimized_points.find(frame->landmark_vec_[i]) != optimized_points.end())
        continue;
      if (frame->track_id_vec_(i) == -1 || isEdgelet(frame->type_vec_[i]))
        continue;
      if (frame->landmark_vec_[i]->obs_.size() >= 2u)
      {
        pts.push_back(frame->landmark_vec_[i]);
        optimized_points.insert(frame->landmark_vec_[i]);
      }
    }
    for (const PointPtr& point : pts)
    {
      point->optimize(max_iter, false);
    }
  }
}

//------------------------------------------------------------------------------
// 0x1801188C0   (glog line 2668)
// Pimax variant of optimizeStructure() that only optimises points tracked in >= 2 consecutive
// bundles (Point::n_consecutive_obs_ +0x78).  c07 called it optimizeStructureInPriorMap.
void FrameProcessorBase::optimizeStructureConsecutive(const FrameBundlePtr& frames,
                                                       int max_n_pts, int max_iter)
{
#line 2668
  VLOG(40) << "Optimize structure.";
  // some feature points will be optimized w.r.t keyframes they were observed
  // in the way that their projection error into all other keyframes is minimzed

  if (max_n_pts == 0)
    return; // don't return if max_n_pts == -1, this means we optimize ALL points

  std::unordered_set<PointPtr> optimized_points;
  for (const FramePtr& frame : frames->frames_)
  {
    std::deque<PointPtr> pts;
    for (size_t i = 0; i < frame->num_features_; ++i)
    {
      if (optimized_points.find(frame->landmark_vec_[i]) != optimized_points.end())
        continue;
      if (frame->track_id_vec_(i) == -1 || isEdgelet(frame->type_vec_[i]))
        continue;
      if (frame->landmark_vec_[i]->obs_.size() >= 2u &&
          frame->landmark_vec_[i]->n_consecutive_obs_ >= 2)
      {
        pts.push_back(frame->landmark_vec_[i]);
        optimized_points.insert(frame->landmark_vec_[i]);
      }
    }
    for (const PointPtr& point : pts)
    {
      point->optimize(max_iter, false);
    }
  }
}

//------------------------------------------------------------------------------
// 0x180128FA0   (glog lines 2714, 2770, 2774)
// upstream upgradeSeedsToFeatures(): track ids instead of landmark pointers, no map points, no
// fixed landmarks (no CHECK), FeatureType edgelet/corner only, seed positions in float.
void FrameProcessorBase::upgradeSeedsToFeatures(const FramePtr& frame)
{
#line 2714
  VLOG(40) << "Upgrade seeds to features";
  size_t update_count = 0;
  size_t unconverged_cnt = 0;
  for (size_t i = 0; i < frame->num_features_; ++i)
  {
    if (frame->track_id_vec_(i) > -1)
    {
      const FeatureType& type = frame->type_vec_[i];
      if (type == FeatureType::kEdgelet || type == FeatureType::kCorner)
      {
        frame->landmark_vec_[i]->addObservation(frame, i);
      }
    }
    else if (frame->seed_ref_vec_[i].keyframe)
    {
      // Pimax isUnconvergedSeed(): kEdgeletSeed || kCornerSeed (binary: type < 2; upstream also
      // counts kMapPointSeed)
      if (frame->type_vec_[i] == FeatureType::kEdgeletSeed
          || frame->type_vec_[i] == FeatureType::kCornerSeed)
      {
        unconverged_cnt++;
      }
      SeedRef& ref = frame->seed_ref_vec_[i];

      // In multi-camera case, it might be that we already created a 3d-point
      // for this seed previously when processing another frame from the bundle.
      PointPtr point = ref.keyframe->landmark_vec_[ref.seed_id];
      if (point == nullptr)
      {
        // That's not the case. Therefore, create a new 3d point.
        Eigen::Vector3f xyz_world =
            ref.keyframe->T_world_cam().cast<float>() *
            ref.keyframe->getSeedPosInFrame(ref.seed_id);
        point = std::make_shared<Point>(xyz_world);
        ref.keyframe->landmark_vec_[ref.seed_id] = point;
        ref.keyframe->track_id_vec_(ref.seed_id) = point->id();
        point->addObservation(ref.keyframe, ref.seed_id);
      }

      // add reference to current frame.
      frame->landmark_vec_[i] = point;
      frame->track_id_vec_(i) = point->id();
      point->addObservation(frame, i);
      if (isCorner(ref.keyframe->type_vec_[ref.seed_id]))
      {
        ref.keyframe->type_vec_[ref.seed_id] = FeatureType::kCorner;
        frame->type_vec_[i] = FeatureType::kCorner;
      }
      else if (isEdgelet(ref.keyframe->type_vec_[ref.seed_id]))
      {
        ref.keyframe->type_vec_[ref.seed_id] = FeatureType::kEdgelet;
        frame->type_vec_[i] = FeatureType::kEdgelet;

        // Update the edgelet direction.
        float angle = feature_detection_utils::getAngleAtPixelUsingHistogram(
            frame->img_pyr_[frame->level_vec_(i)],
            (frame->px_vec_.col(i) / (1 << frame->level_vec_(i))).cast<int>(),
            4u);
        frame->grad_vec_.col(i) = GradientVector(std::cos(angle),
                                                 std::sin(angle));
      }
      ++update_count;
    }

    // when using the feature-wrapper, we might copy some old references?
    frame->seed_ref_vec_[i].keyframe.reset();
    frame->seed_ref_vec_[i].seed_id = -1;
  }
#line 2770
  VLOG(5) << "NEW KEYFRAME: Updated "
          << update_count << " seeds to features in reference frame, "
          << "including " << unconverged_cnt << " unconverged points.\n";
  const double ratio = (1.0 * unconverged_cnt) / update_count;
  if (ratio > 0.2)
  {
#line 2774
    LOG(WARNING) << ratio * 100 << "% updated seeds are unconverged.";
  }
}

}  // namespace totem
}  // namespace pimax
