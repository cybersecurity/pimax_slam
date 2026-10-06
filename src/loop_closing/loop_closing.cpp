// pimax_slam.pi.dll -- src/loop_closing/loop_closing.cpp
//
// pimax::totem::LoopClosing (loop_closing.obj, TU47).  Very loosely derived from
// rpg_svo_pro_open/svo_online_loopclosing/src/loop_closing.cpp.
// Sources: draft c00_globals/loop_closing/loop_closing_globals.cpp (namespace-scope globals),
// draft c15_platmap/loop_closing/loop_closing_ctor.cpp (ctor 0x18017f730 / dtor 0x180181ae0),
// draft c16_loop_closing/loop_closing/loop_closing.cpp (0x180185e40 .. 0x180196b60; function order
// below = address order in the binary).  KeyFrame / PlatMap bodies: loop_closing/platmap.cpp.
//
// Conventions:
//   * LOGD/LOGI/LOGW/LOGE = pimax::g_logger (common/logger.h).  glog VLOG/LOG kept as glog; the
//     __LINE__ of every VLOG(40) site is forced with #line to the value found in the binary
//     (the following #line restores the physical numbering).
//   * Every `std::unique_lock ... lock.unlock()` pair reproduces the `if (!_Pmtx) throw; unlock`
//     sequences seen in the binary; plain scope exits reproduce the destructor unlocks.
#include "loop_closing/loop_closing.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iterator>
#include <sstream>
#include <tuple>
#include <unordered_map>

#include <ceres/ceres.h>
#include <glog/logging.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <vikit/cameras/camera_geometry.h>
#include <vikit/cameras/camera_geometry_base.h>
#include <vikit/cameras/equidistant_distortion.h>
#include <vikit/cameras/ncamera.h>
#include <vikit/cameras/pinhole_projection.h>
#include <vikit/timer.h>

#include "ceres_backend/ceres_map.hpp"                    // ceres_backend::Map
#include "ceres_backend/general_3d_parameter_block.hpp"
#include "ceres_backend/pose_local_parameterization.hpp"
#include "ceres_backend/pose_parameter_block.hpp"
#include "ceres_backend/reprojection_error.hpp"
#include "common/feature_wrapper.h"                       // FeatureWrapper, SeedRef
#include "common/frame.h"                                 // Frame, FrameBundle, frame_utils
#include "common/logger.h"
#include "direct/matcher.h"                               // Matcher
#include "frontend/utility.h"                             // Utility::R2ypr / ypr2R(.., bool)
#include "loop_closing/beblid.h"

namespace pimax {
namespace totem {

// ===============================================================================================
// Namespace-scope globals (draft c00 loop_closing_globals.cpp).  .CRT$XCU order of TU47:
// #154 Eigen marker, #155 kStrToScaleRetMap (init 0x180002DF0, atexit 0x1803A62F0),
// #156 kStrToGlobalMapType (init 0x180002BF0, atexit 0x1803A6260), #157 kPlatMapVersion
// (init 0x180002BB0, atexit 0x1803A61F0) -- i.e. definition order below.
// Upstream (svo_online_loopclosing/src/loop_closing.cpp) uses std::map<std::string, ...>;
// Pimax uses std::unordered_map<std::string, ...> (insert-range helper 0x18017E530).
// ===============================================================================================
std::unordered_map<std::string, LCScaleRetMethod> kStrToScaleRetMap{  // 0x18047EEE0
  { std::string("CommonLM"), LCScaleRetMethod::kCommonLandmarks },
  { std::string("MixedKP"), LCScaleRetMethod::kMixedKeyPoints },
  { std::string("None"), LCScaleRetMethod::kNone }
};

std::unordered_map<std::string, GlobalMapType> kStrToGlobalMapType{  // 0x18047EE90
  { std::string("BuiltInPoseGraph"), GlobalMapType::kBuiltInPoseGraph },
  { std::string("ExternalGlobalMap"), GlobalMapType::kExternalGlobalMap },
  { std::string("None"), GlobalMapType::kNone }
};

// 0x18046A200 (std::string in .data: size/capacity constant-initialised, the 5 bytes "1.0.0"
// (0x1803bdd9c) written by the initializer).  Compared against "map_version_" of the YAML index
// in PlatMap::loadIndex 0x180183320.  c00 had it `static`; it is extern (platmap.h) because
// PlatMap's bodies live in platmap.cpp.  TODO(verify) name.
const std::string kPlatMapVersion = "1.0.0";

// ---------------------------------------------------------------------------------------
// 0x18017f730   upstream-modified
//
// Differences to upstream:
//  * extra parameters map_tag / mode; mode stored at +1216, map tag at +1224
//    (kNormal: caller tag, kLabMap/kLabLoc: "Lab"; lab modes multiply max_kf_num by 5,
//    kLabLoc also disables saving);
//  * K_ / D_ are collected for EVERY camera (K_list_, D_list_) instead of camera 0 only;
//    T_C_B_/T_B_C_ are NOT initialised from the bundle any more (stay identity);
//  * mask_ = camera 0 mask (clone), aruco detector parameters + DICT_4X4_1000;
//  * no CHECKs on the string->enum maps (operator[] on unordered_maps), no CHECK_GE on
//    ignored_past_frames, no Pgo creation;
//  * "exec rm -r <image_log_base_path>*" only if enable_image_logging;
//  * starts the worker threads, then creates the PlatMap and (unless kLabMap) loads it.
//    NOTE: threads are started BEFORE plat_map_ exists (quirk, kept).
LoopClosing::LoopClosing(const LoopClosureOptions& loopclosure_options,
                         const CameraBundlePtr& cams, const std::string& map_tag,
                         LoopClosingMode mode)
  : cams_(cams)
  , options_(loopclosure_options)
  , orb_(cv::ORB::create(500, 1.2f, 4, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20))
  , beblid_(BEBLID::create(256, 0.75f))   // 0x180170570 (out-of-line, by value; c15 notes: make_shared)
{
  mode_ = mode;
  switch (mode)
  {
    case LoopClosingMode::kNormal:
      map_tag_ = map_tag;
      break;
    case LoopClosingMode::kLabMap:
      map_tag_ = "Lab";
      options_.max_kf_num *= 5;
      break;
    case LoopClosingMode::kLabLoc:
      options_.max_kf_num *= 5;
      save_map_enabled_ = false;
      map_tag_ = "Lab";
      break;
  }

  std::stringstream path;
  path << options_.voc_path << options_.voc_name;
  std::string voc_path_full = path.str();
  voc_ = OrbVocabulary(voc_path_full);   // upstream: voc_ = loadVoc(voc_path_full);

  for (size_t i = 0; i < cams->getNumCameras(); ++i)   // NOTE: parameter, not cams_
  {
    cv::Mat K;
    Eigen::VectorXd D;
    Eigen::VectorXd intrinsics = cams->getCameraShared(i)->getIntrinsicParameters();
    K = (cv::Mat_<double>(3, 3) << intrinsics(0), 0, intrinsics(2), 0, intrinsics(1),
         intrinsics(3), 0, 0, 1);
    D = cams->getCameraShared(i)->getDistortionParameters();
    K_.push_back(K);
    D_.push_back(D);
  }
  mask_ = cams->getCameraShared(0)->getMask().clone();

  aruco_params_ = cv::aruco::DetectorParameters::create();
  aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);

  scale_retrieval_approach_ = kStrToScaleRetMap[options_.scale_ret_app];
  global_map_type_ = kStrToGlobalMapType[options_.global_map_type];

  if (options_.enable_image_logging)
  {
    int sys = system(("exec rm -r " + options_.image_log_base_path + "*").c_str());
    (void)sys;
  }

  startAllThread();

  if (options_.use_plat_map)
  {
    plat_map_ = std::make_shared<PlatMap>();
    plat_map_->max_kf_num_ = options_.max_kf_num;
    plat_map_->K_list_ = K_;
    plat_map_->D_list_ = D_;

    std::string bk_file = options_.map_path + map_tag_ + options_.map_name + ".bk";
    std::ifstream bk_stream(bk_file);
    if (bk_stream.good())
    {
      // NOTE: message says "not existed" but is printed when the backup DOES exist.
      LOGE("platMap_orborb_K8L4.bin.bk is not existed\n");
      remove(bk_file.c_str());
      std::string map_file = options_.map_path + map_tag_ + options_.map_name;
      std::string index_file = options_.map_path + map_tag_ + options_.map_index_name;
      remove(map_file.c_str());
      remove(index_file.c_str());
    }

    if (mode == LoopClosingMode::kLabMap)
    {
      LOGW("kLabMap mode, not load map, only generate map\n");
    }
    else
    {
      if (mode == LoopClosingMode::kLabLoc)
      {
        LOGW("kLabLoc mode, load map to reloc, not save map\n");
      }
      else if (mode == LoopClosingMode::kNormal)
      {
        LOGW("kNormal mode, load map to reloc, and save map\n");
      }
      if (loadIndex())
      {
        load();
      }
    }
  }
}

// ---------------------------------------------------------------------------------------
// 0x180181ae0 (scalar deleting dtor 0x180182910 frees with free(): aligned operator new)
// upstream-modified: upstream dtor is empty.
LoopClosing::~LoopClosing()
{
  stopAllThread();
}

// 0x180180cd0  LoopClosureOptions::LoopClosureOptions(const LoopClosureOptions&): implicit
//              (copy of options_ in the ctor above; 656 bytes, 9 std::string members).

// ===============================================================================================
// 0x180185e40  addFrameToPR -- upstream-modified (heavily):
//  * takes both frames of the stereo bundle and an odometry->world offset,
//  * a pose-cell key map (pose_key_map_) suppresses keyframes in already visited cells,
//  * frame quality gates (num_features_, mean intensity 25..220),
//  * no BoW here (moved to runPROnLatestKeyframe), no detached threads: the loop-closing worker
//    is woken through lc_cond_,
//  * kf_list_ is capped at options_.max_kf_num (oldest bundle erased).
// ===============================================================================================
void LoopClosing::addFrameToPR(const FrameBundlePtr& last_frames, const Transformation& T_w_odom)
{
  std::unique_lock<std::mutex> lock(completed_flags_mutex_);
  if (!completed_flags_.empty() && !completed_flags_.back())
  {
    return;
  }
  lock.unlock();

  for (size_t i = 0; i < 2; ++i)
  {
    FramePtr frame = last_frames->at(i);
    const std::string key = getPoseKey(T_w_odom * frame->T_world_imu());
    if ((pose_key_map_.find(key) != pose_key_map_.end() && !frame->is_keyframe_) ||
        frame->num_features_ < options_.min_num_features ||
        frame->mean_intensity_ < 25.0 || frame->mean_intensity_ > 220.0)
    {
      return;
    }
  }

  std::vector<KeyFramePtr> cur_kfs;
  for (size_t i = 0; i < 2; ++i)
  {
    const FramePtr& frame = last_frames->at(i);
    // KeyFrame(nframe_id = bundle id (FrameBundle+0xFC), cam_id = Frame+0x14 (cam_index_),
    //          frame_id = Frame+0x10, map_id)
    KeyFramePtr kf = std::make_shared<KeyFrame>(last_frames->getBundleId(), frame->cam_index_,
                                                frame->id_, map_id_);
    if (!kf)   // quirk: null check on the make_shared result is in the binary
    {
      return;
    }
    cur_kfs.push_back(kf);
  }
  KeyFramePtr cur_kf = cur_kfs[0];

  for (size_t i = 0; i < 2; ++i)
  {
    FramePtr frame = last_frames->at(i);
    svoFrameToKeyframe(frame, cur_kfs[i].get(), T_w_odom, false);
  }

  std::string key = getPoseKey(T_w_odom * last_frames->at(0)->T_world_imu());
  ++svo_keyframe_count_;
  cur_loop_check_viz_info_.clear();

  std::unique_lock<std::mutex> kf_lock(kf_list_mutex_);
  const int n_kf_list = kf_list_.size();
  kf_lock.unlock();

  if (svo_keyframe_count_ == 1 || n_kf_list < 2)
  {
    kf_lock.lock();
    kf_list_.push_back(cur_kfs);
    kf_lock.unlock();
    pose_key_map_.insert({key, true});

    lock.lock();
    completed_flags_.push_back(false);
    lock.unlock();
    run_lc_on_this_frame_ = true;
    lc_cond_.notify_one();
    last_run_lc_frame_trackIDs_ = cur_kf->svo_trackIDsvector_;
  }
  else
  {
    lock.lock();
    const bool last_finished = completed_flags_.back();
    lock.unlock();

    FramePtr frame0 = last_frames->at(0);
    if (last_finished)
    {
      if (commonLandMarkCheck(last_run_lc_frame_trackIDs_, cur_kf->svo_trackIDsvector_,
                              options_.beta))
      {
        bool run_lc_on_this_frame;
        if (frame0->is_keyframe_)
        {
          run_lc_on_this_frame = true;
          last_run_lc_frame_trackIDs_ = cur_kf->svo_trackIDsvector_;
        }
        else
        {
          run_lc_on_this_frame = false;
        }

        kf_lock.lock();
        if (kf_list_.size() > options_.max_kf_num)
        {
          kf_list_.erase(kf_list_.begin());
        }
        kf_list_.push_back(cur_kfs);
        pose_key_map_.insert({key, true});
        cumulative_distance_ += (kf_list_.back()[0]->T_w_c_.getPosition() -
                                 kf_list_[kf_list_.size() - 2][0]->T_w_c_.getPosition())
                                    .norm();
        kf_lock.unlock();
        prox_dist_thresh_ =
            options_.proximity_dist_ratio * cumulative_distance_ + options_.proximity_offset;

        lock.lock();
        completed_flags_.push_back(false);
        lock.unlock();
        run_lc_on_this_frame_ = run_lc_on_this_frame;
        lc_cond_.notify_one();
      }
    }
    else
    {
#line 1351
      VLOG(40) << "########## WARNING: Last thread still running ###########";   // line 1351
#line 336
    }
  }
}

// ===============================================================================================
// 0x180186a50  reLocalize -- pimax-new.  Queues the KeyFrames of the new bundle for the
// relocalization worker (reloc_busy_ is a one-shot gate cleared by the worker / resetReLocalize).
// ===============================================================================================
void LoopClosing::reLocalize(const FrameBundlePtr& new_frames, const Transformation& T_w_odom)
{
  if (!options_.use_plat_map)
  {
    return;
  }
  if (!plat_map_ready_)
  {
    LOGI("INFO=ReLoc plat map not be ready.\n");
    return;
  }
  bool expected = false;
  if (!reloc_busy_.compare_exchange_strong(expected, true))
  {
    return;
  }

  try
  {
    {
      std::unique_lock<std::mutex> lock(reloc_frames_mutex_);
      reloc_kf_list_.clear();
      const size_t n_frames = new_frames->frames_.size();   // read once (cached in the binary)
      for (int i = 0; i < n_frames; ++i)
      {
        const FramePtr& frame = new_frames->at(i);
        KeyFramePtr kf = std::make_shared<KeyFrame>(new_frames->getBundleId(), frame->cam_index_,
                                                    frame->id_, map_id_);
        svoFrameToKeyframe(new_frames->at(i), kf.get(), T_w_odom, false);
        reloc_kf_list_.push_back(kf);
      }
    }
    reloc_cond_.notify_one();
  }
  catch (const cv::Exception& e)
  {
    LOGE("ReLocalize: failed to prepare frames, OpenCV exception code=%d: %s\n", e.code, e.what());
    clearReLocFrames();
    reloc_busy_ = false;
    return;
  }
  catch (const std::bad_alloc& e)
  {
    LOGE("ReLocalize: failed to prepare frames, allocation failure: %s\n", e.what());
    clearReLocFrames();
    reloc_busy_ = false;
    return;
  }
  catch (const std::exception& e)
  {
    LOGE("ReLocalize: failed to prepare frames: %s\n", e.what());
    clearReLocFrames();
    reloc_busy_ = false;
    return;
  }
  catch (...)
  {
    LOGE("ReLocalize: failed to prepare frames, unknown exception\n");
    clearReLocFrames();
    reloc_busy_ = false;
  }
}

// ===============================================================================================
// 0x180186cb0  file-static helper -- pimax-new.  cv::putText with FONT_HERSHEY_SIMPLEX, LINE_8.
// ===============================================================================================
static void drawText(cv::Mat& img, const std::string& text, cv::Point org, double font_scale,
                     cv::Scalar color, int thickness)
{
  cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness, cv::LINE_8,
              false);
}

// ===============================================================================================
// 0x180186f30  computePoseDiff -- pimax-new.  Called by the frontend (0x1800fb650) with
// (T_world_imu(new bundle), T_world_imu(last bundle)); addFrameToPR runs only if dist > 0.01.
// Translation distance is computed in FLOAT (cast before the subtraction).
// ===============================================================================================
void LoopClosing::computePoseDiff(double* dist, double* angle_deg, const Transformation& T_1,
                                  const Transformation& T_2)
{
  const Eigen::Vector3f t_1 = T_1.getPosition().cast<float>();
  const Eigen::Vector3f t_2 = T_2.getPosition().cast<float>();
  *dist = (t_1 - t_2).norm();
  const Eigen::Matrix3d R_1 = T_1.getRotationMatrix();
  const Eigen::Matrix3d R_2 = T_2.getRotationMatrix();
  *angle_deg =
      std::acos(std::max(-1.0, std::min(1.0, ((R_1.transpose() * R_2).trace() - 1.0) * 0.5))) *
      180.0 / 3.141592653589793;
}

// ===============================================================================================
// 0x1801872e0  clearReLocFrames -- pimax-new (used by the reLocalize catch handlers; the same body
// is inlined into reLocalizeThread).
// ===============================================================================================
void LoopClosing::clearReLocFrames()
{
  std::unique_lock<std::mutex> lock(reloc_frames_mutex_);
  reloc_kf_list_.clear();
  lock.unlock();
}

// ===============================================================================================
// 0x180187370  backProject -- pimax-new.  Batch back-projection through the camera (vtable slot 1 =
// CameraGeometryBase::backProject3(const Ref<const Matrix2Xd>&, Matrix3Xd*, vector<bool>*)),
// result narrowed to float.  `px` and `cam` are taken BY VALUE (destroyed in the callee).
// ===============================================================================================
void LoopClosing::backProject(Eigen::Matrix2Xd px, CameraPtr cam, Eigen::Matrix3Xf* f)
{
  std::vector<bool> success;
  Eigen::Matrix3Xd bearings;
  cam->backProject3(px, &bearings, &success);
  *f = bearings.cast<float>();
}

// ===============================================================================================
// 0x180187830  extractAndConvert -- upstream-modified:
//  * Twc = T_w_odom * T_world_cam(), timestamp first,
//  * keeps features whose pyramid level is <= 1 and (unless use_all_features) that are tracked
//    (track id != -1); no null check of landmark_vec_[i] (quirk),
//  * landmark position is the float Point::pos_ cast to double, expressed in the camera frame,
//  * "landmark ids" and "track ids" both receive track_id_vec_(i),
//  * bearings / depths / feature types / original indices are no longer extracted.
// ===============================================================================================
void LoopClosing::extractAndConvert(const FramePtr& frame, double* timestamp_sec,
                                    Transformation* Twc, std::vector<cv::Point2f>* keypoints,
                                    std::vector<cv::Point3f>* landmarks_in_cam,
                                    std::vector<int>* landmark_ids, std::vector<int>* track_ids,
                                    const Transformation& T_w_odom, const bool use_all_features)
{
  *timestamp_sec = frame->getTimestampSec();   // (double)timestamp_ns(+240) / 1e9
  *Twc = T_w_odom * frame->T_world_cam();
  for (size_t i = 0; i < frame->landmark_vec_.size(); ++i)
  {
    if (!use_all_features && frame->track_id_vec_(i) == -1)
    {
      continue;
    }
    if (frame->level_vec_(i) <= 1)
    {
      keypoints->emplace_back(frame->px_vec_(0, i), frame->px_vec_(1, i));
      const Eigen::Vector3d p_c = frame->T_f_w_ * frame->landmark_vec_[i]->pos_.cast<double>();
      landmarks_in_cam->emplace_back(p_c(0), p_c(1), p_c(2));
      landmark_ids->push_back(frame->track_id_vec_(i));
      track_ids->push_back(frame->track_id_vec_(i));
    }
  }
}

// ===============================================================================================
// 0x180187c90  svoFrameToKeyframe -- upstream-modified: clears the SVO feature info in place
// (clearSVOFeatureInfo inlined, reduced field set), copies ids, the IMU-camera extrinsic
// (Frame::T_body_cam_, +256) into KeyFrame+192 and a deep copy of pyramid level 0.
// ===============================================================================================
void LoopClosing::svoFrameToKeyframe(const FramePtr& frame, KeyFrame* kf,
                                     const Transformation& T_w_odom, const bool use_all_features)
{
  kf->svo_keypointsvector_.clear();
  kf->svo_landmarksvector_cam_.clear();
  kf->svo_landmark_ids_.clear();
  kf->svo_trackIDsvector_.clear();
  kf->svo_features_mat_.release();
  kf->svo_features_.clear();
  kf->svo_node_ids_.clear();

  kf->frame_id_ = frame->id_;
  kf->cam_id_ = frame->cam_index_;    // Frame+0x14 (c16 draft: "bundle_id_", A1: cam_index_)
  kf->T_unk_192_ = frame->T_imu_cam();   // Frame::T_body_cam_ (+0x100) = T_b_c
  kf->keyframe_image_ = frame->img().clone();
  extractAndConvert(frame, &kf->timestamp_sec_abs_, &kf->T_w_c_, &kf->svo_keypointsvector_,
                    &kf->svo_landmarksvector_cam_, &kf->svo_landmark_ids_,
                    &kf->svo_trackIDsvector_, T_w_odom, use_all_features);
}

// ===============================================================================================
// 0x180187e30  getPoseKey -- pimax-new.  Discretises a pose into "x_y_z_yaw_pitch_roll" cells.
// (Utility::R2ypr is inlined: degrees.)
// ===============================================================================================
std::string LoopClosing::getPoseKey(const Transformation& T)
{
  const Eigen::Vector3d t = T.getPosition();
  const Eigen::Vector3d ypr = Utility::R2ypr(T.getRotationMatrix());
  const double pos_res = options_.key_pos_resolution + 0.000001;
  const int ix = static_cast<int>(std::floor(t.x() / pos_res));
  const int iy = static_cast<int>(std::floor(t.y() / pos_res));
  const int iz = static_cast<int>(std::floor(t.z() / pos_res));
  const double ang_res = options_.key_ang_resolution + 0.000001;
  const int iyaw = static_cast<int>(std::floor(ypr(0) / ang_res));
  const int ipitch = static_cast<int>(std::floor(ypr(1) / ang_res));
  const int iroll = static_cast<int>(std::floor(ypr(2) / ang_res));
  return std::to_string(ix) + "_" + std::to_string(iy) + "_" + std::to_string(iz) + "_" +
         std::to_string(iyaw) + "_" + std::to_string(ipitch) + "_" + std::to_string(iroll);
}

// ===============================================================================================
// 0x180188830  loopClosingThread -- pimax-new (replaces upstream's detached std::thread per KF).
// ===============================================================================================
void LoopClosing::loopClosingThread()
{
  while (true)
  {
    std::unique_lock<std::mutex> lock(lc_mutex_);
    lc_cond_.wait(lock);
    if (lc_stop_)
    {
      return;
    }
    runPROnLatestKeyframe(options_.ignored_past_frames, run_lc_on_this_frame_);
  }
}

// ===============================================================================================
// 0x180188900  load -- pimax-new.  Loads <map_path><map_tag><map_name>; on a std::exception falls
// back to the portable archive "<...>.pba".  Logs success even after the fallback.
// ===============================================================================================
void LoopClosing::load()
{
  plat_map_ready_ = false;
  std::string path = options_.map_path + map_tag_ + options_.map_name;
  std::ifstream ifs(path);
  const bool fail = !ifs.good();
  ifs.close();
  if (fail)
  {
    LOGE("Could not open platMap file %s\n", path.c_str());
    return;
  }
  try
  {
    plat_map_->load(path);
  }
  catch (const std::exception&)
  {
    plat_map_->load_bin(path + ".pba");
  }
  plat_map_ready_ = true;
  LOGW("re-localization, load %s success\n", path.c_str());
  LOGW("re-localization, map size %d\n", plat_map_->kf_map_.size());
}

// ===============================================================================================
// 0x180188ce0  loadIndex -- pimax-new.  map_id_ becomes (largest indexed map id) + 1.
// In kLabLoc mode the tag index is read from the SAME yaml file.
// ===============================================================================================
bool LoopClosing::loadIndex()
{
  std::string path = options_.map_path + map_tag_ + options_.map_index_name;
  std::ifstream ifs(path);
  if (!ifs.is_open())
  {
    LOGE("Could not open platMap index file %s\n", path.c_str());
    map_id_ = 0;
    plat_map_ready_ = false;
    return false;
  }
  ifs.seekg(0, std::ios::end);
  if (ifs.tellg() == 0)
  {
    LOGE("platMap index file is empty %s\n", path.c_str());
    ifs.close();
    map_id_ = 0;
    plat_map_ready_ = false;
    return false;
  }
  ifs.close();
  if (!plat_map_->loadIndex(path))
  {
    map_id_ = 0;
    plat_map_ready_ = false;
    return false;
  }
  map_id_ = (--plat_map_->map_index_.end())->first + 1;
  if (mode_ == LoopClosingMode::kLabLoc && !loadTagIndex(path))
  {
    LOGW("Could not load tag file %s\n", path.c_str());
  }
  return true;
}

// ===============================================================================================
// 0x1801890e0  loadSavePlatMapThread -- pimax-new.
// ===============================================================================================
void LoopClosing::loadSavePlatMapThread()
{
  while (true)
  {
    std::unique_lock<std::mutex> lock(platmap_mutex_);
    if (platmap_stop_)
    {
      return;
    }
    platmap_cond_.wait(lock);
    if (platmap_cmd_ == 0)
    {
      load();
    }
    if (platmap_cmd_ == 1)
    {
      save();
    }
    if (platmap_cmd_ == 2)
    {
      save();
      return;
    }
  }
}

// ===============================================================================================
// 0x1801891b0  loadTagIndex -- pimax-new.  Reads "tag_index_" (map_id_, frame_id_, board_idx,
// T[3], Q[4]) into tag_load_.  NOTE: map_id_/frame_id_ are overwritten per entry (last wins).
// ===============================================================================================
bool LoopClosing::loadTagIndex(const std::string& path)
{
  cv::FileStorage fs;
  fs.open(path, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);
  if (!fs.isOpened())
  {
    LOGE("Failed to open file %s\n", path.c_str());
    return false;
  }
  cv::FileNode tag_index = fs["tag_index_"];
  if (tag_index.empty() || tag_index.size() == 0)
  {
    return false;
  }
  for (cv::FileNodeIterator it = tag_index.begin(); it != tag_index.end(); ++it)
  {
    cv::FileNode node = *it;
    tag_load_.map_id_ = (int)node["map_id_"];
    tag_load_.frame_id_ = (int)node["frame_id_"];
    tag_load_.board_idx.push_back((int)node["board_idx"]);
    const double tx = (double)node["T"][0];
    const double ty = (double)node["T"][1];
    const double tz = (double)node["T"][2];
    tag_load_.T.push_back(Eigen::Vector3d(tx, ty, tz));
    Eigen::Vector4d q;
    q(0) = (double)node["Q"][0];
    q(1) = (double)node["Q"][1];
    q(2) = (double)node["Q"][2];
    q(3) = (double)node["Q"][3];
    tag_load_.Q.push_back(q);
  }
  tag_load_.valid = true;
  LOGI("success load tag index\n");
  return true;
}

// ===============================================================================================
// 0x1801899c0  bundleAdjustKfList -- pimax-new.  Called by resetReLocalize when the load/save
// worker is told to exit (platmap_cmd_ == 2).  Ceres BA over the session's bundles:
//   * per camera: extrinsic T_b_c (KeyFrame+192) block, constant; camera model rebuilt from the
//     PlatMap K/D lists as CameraGeometry<Pinhole<Equidistant>> 640x480,
//   * per bundle: T_w_b = kfs[0]->T_w_c_ * T_b_c^-1 block (variable),
//   * per tracked landmark with positive depth: world point block, CONSTANT,
//   * ReprojectionError (2,7,3,7) with CauchyLoss(1) -> only the bundle poses move.
// Afterwards every KeyFrame gets T_w_c_ = T_w_b * T_b_c (re-normalised) and its camera-frame
// landmarks are recomputed from the (unchanged) world points.
// ===============================================================================================
bool LoopClosing::bundleAdjustKfList(std::vector<std::vector<KeyFramePtr>>& kf_list)
{
  if (kf_list.size() == 0)
  {
    return false;
  }
  const auto t_start = std::chrono::steady_clock::now();   // result unused in the binary
  (void)t_start;

  const int n_cams = kf_list[0].size();
  std::vector<double> fx(n_cams);
  std::vector<double> fy(n_cams);
  std::vector<double> cx(n_cams);
  std::vector<double> cy(n_cams);
  std::vector<CameraPtr> cams(n_cams);
  for (int i = 0; i < n_cams; ++i)
  {
    fx[i] = plat_map_->K_list_[i].at<double>(0, 0);
    fy[i] = plat_map_->K_list_[i].at<double>(1, 1);
    cx[i] = plat_map_->K_list_[i].at<double>(0, 2);
    cy[i] = plat_map_->K_list_[i].at<double>(1, 2);
    const Eigen::VectorXd& D = plat_map_->D_list_[i];
    cams[i] = std::make_shared<vk::cameras::CameraGeometry<
        vk::cameras::PinholeProjection<vk::cameras::EquidistantDistortion>>>(
        640, 480,
        vk::cameras::PinholeProjection<vk::cameras::EquidistantDistortion>(
            fx[i], fy[i], cx[i], cy[i],
            vk::cameras::EquidistantDistortion(D(0), D(1), D(2), D(3), D(4), D(5))));
  }

  // track id -> index of the world point block
  std::unordered_map<int, int> landmark_index;
  int n_points = 0;
  for (const auto& kfs : kf_list)
  {
    for (const auto& kf : kfs)
    {
      const int n_lm = kf->svo_landmarksvector_cam_.size();
      for (int j = 0; j < n_lm; ++j)
      {
        const int track_id = kf->svo_trackIDsvector_[j];
        if (track_id > -1 && landmark_index.find(track_id) == landmark_index.end() &&
            kf->svo_landmarksvector_cam_[j].z > 0.0)
        {
          landmark_index[kf->svo_trackIDsvector_[j]] = n_points++;
        }
      }
    }
  }

  ceres::Problem problem;
  ceres::LossFunction* loss_function = new ceres::CauchyLoss(1.0);
  ceres::LocalParameterization* local_parameterization =
      new ceres_backend::PoseLocalParameterization();

  double* cam_poses = new double[7 * n_cams];
  for (int i = 0; i < n_cams; ++i)
  {
    const Transformation T_b_c = kf_list[0][i]->T_unk_192_;
    const Eigen::Quaterniond q = T_b_c.getEigenQuaternion().normalized();
    double* p = cam_poses + 7 * i;
    p[0] = T_b_c.getPosition().x();
    p[1] = T_b_c.getPosition().y();
    p[2] = T_b_c.getPosition().z();
    p[3] = q.x();
    p[4] = q.y();
    p[5] = q.z();
    p[6] = q.w();
    problem.AddParameterBlock(p, 7, local_parameterization);
    problem.SetParameterBlockConstant(p);
  }

  double* points = new double[3 * n_points];
  double* bundle_poses = new double[7 * kf_list.size()];
  std::map<int, int> point_added;
  for (size_t b = 0; b < kf_list.size(); ++b)
  {
    std::vector<KeyFramePtr> kfs = kf_list[b];
    const Transformation T_w_b = kfs[0]->T_w_c_ * kfs[0]->T_unk_192_.inverse();
    const Eigen::Quaterniond q = T_w_b.getEigenQuaternion().normalized();
    double* pose = bundle_poses + 7 * b;
    pose[0] = T_w_b.getPosition().x();
    pose[1] = T_w_b.getPosition().y();
    pose[2] = T_w_b.getPosition().z();
    pose[3] = q.x();
    pose[4] = q.y();
    pose[5] = q.z();
    pose[6] = q.w();
    problem.AddParameterBlock(pose, 7, local_parameterization);

    const int n_kfs = kfs.size();
    for (int c = 0; c < n_kfs; ++c)
    {
      const KeyFramePtr& kf = kfs[c];
      const size_t n_lm = kf->svo_landmarksvector_cam_.size();
      for (size_t j = 0; j < n_lm; ++j)
      {
        if (kf->svo_trackIDsvector_[j] > -1)
        {
          const cv::Point3f& pc = kf->svo_landmarksvector_cam_[j];
          const Eigen::Vector3d p_c(pc.x, pc.y, pc.z);
          const Eigen::Vector3d p_w = kf->T_w_c_ * p_c;
          if (p_c.z() > 0.0)
          {
            int track_id = kf->svo_trackIDsvector_[j];
            const int idx = landmark_index[track_id];
            if (point_added.find(track_id) == point_added.end())
            {
              double* pt = points + 3 * idx;
              pt[0] = p_w.x();
              pt[1] = p_w.y();
              pt[2] = p_w.z();
              problem.AddParameterBlock(pt, 3);
              problem.SetParameterBlockConstant(pt);
              point_added[track_id] = 1;
            }
            const Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
            Eigen::Vector2d obs;
            obs(0) = kf->svo_keypointsvector_[j].x;
            obs(1) = kf->svo_keypointsvector_[j].y;
            ceres::CostFunction* cost_function =
                new ceres_backend::ReprojectionError(cams[c], obs, information);
            double* cam_pose = cam_poses + 7 * c;
            std::vector<double*> parameter_blocks{pose, points + 3 * idx, cam_pose};
            problem.AddResidualBlock(cost_function, loss_function, parameter_blocks);
            problem.SetParameterBlockConstant(cam_pose);
          }
        }
      }
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.max_num_iterations = 15;
  options.num_threads = 1;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  for (size_t b = 0; b < kf_list.size(); ++b)
  {
    const double* pose = bundle_poses + 7 * b;
    const Eigen::Vector3d t(pose[0], pose[1], pose[2]);
    const Eigen::Quaterniond q = Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5]).normalized();
    const Transformation T_w_b(q, t);   // RotationQuaternion(q) built in place (norm CHECK)
    const int n_kfs = kf_list[b].size();
    for (int c = 0; c < n_kfs; ++c)
    {
      kf_list[b][c]->T_w_c_ = T_w_b * kf_list[b][c]->T_unk_192_;
      kf_list[b][c]->T_w_c_.getRotation().normalize();
    }
  }

  for (auto& kfs : kf_list)
  {
    const int n_kfs = kfs.size();
    for (int c = 0; c < n_kfs; ++c)
    {
      const size_t n_lm = kfs[c]->svo_landmarksvector_cam_.size();
      for (size_t j = 0; j < n_lm; ++j)
      {
        const KeyFramePtr& kf = kfs[c];
        int& track_id = kf->svo_trackIDsvector_[j];
        if (track_id > -1)
        {
          if (landmark_index.find(track_id) != landmark_index.end())
          {
            const int idx = landmark_index[kf->svo_trackIDsvector_[j]];
            const Eigen::Vector3d p_w(points[3 * idx], points[3 * idx + 1], points[3 * idx + 2]);
            const Eigen::Vector3d p_c = kfs[c]->T_w_c_.inverse() * p_w;
            kfs[c]->svo_landmarksvector_cam_[j].x = p_c.x();
            kfs[c]->svo_landmarksvector_cam_[j].y = p_c.y();
            kfs[c]->svo_landmarksvector_cam_[j].z = p_c.z();
          }
        }
      }
    }
  }

  delete[] cam_poses;
  delete[] points;
  delete[] bundle_poses;
  return true;
}

// ===============================================================================================
// 0x18018b3d0  reLocalizeThread -- pimax-new.  Besides running the relocalization it pushes the
// session into the PlatMap (and saves it) once 200 bundles have been collected.
// ===============================================================================================
void LoopClosing::reLocalizeThread()
{
  while (!reloc_stop_)
  {
    std::unique_lock<std::mutex> lock(reloc_mutex_);
    reloc_cond_.wait(lock);
    if (reloc_stop_)
    {
      return;
    }
    if (!platmap_built_)
    {
      std::unique_lock<std::mutex> kf_lock(kf_list_mutex_);
      if (kf_list_.size() >= 200)
      {
        plat_map_->UpdateMap(kf_list_, map_id_);
        kf_lock.unlock();
        platmap_built_ = true;
        save();
      }
      else
      {
        kf_lock.unlock();
      }
    }
    if (reloc_busy_)
    {
      runReLocalization();
    }
    clearReLocFrames();   // inlined in the binary
    reloc_busy_ = false;
  }
}

// ===============================================================================================
// 0x18018b600  getReLocCorrection -- pimax-new.  Needs >= 5 queued corrections; groups them by
// translation norm (10 cm bins), ranks groups by mean weight (1/confidence, capped at 1e6) and
// returns the best group's first correction with the translation replaced by the group "mean".
// QUIRK (kept): the mean loop never advances its iterator -> it averages N copies of the first
// element, i.e. the translation is that element's own translation.
// ===============================================================================================
bool LoopClosing::getReLocCorrection(ReLocCorrectionInfo* info)
{
  using Group = std::multimap<float, int, std::greater<float>>;

  std::deque<ReLocCorrectionInfo> corrections;
  {
    std::lock_guard<std::mutex> lock(reloc_info_lock_);
    corrections = reLoc_correction_info_;
  }
  const size_t n_corrections = corrections.size();
  if (n_corrections < 5)
  {
    return false;
  }

  std::unordered_map<int, Group> groups;
  for (int i = 0; i < (int)n_corrections; ++i)
  {
    const float weight = std::min(1.0f / corrections[i].confidence_, 1000000.0f);
    const Transformation T = corrections[i].w_T_new_old_;
    int key = static_cast<int>(T.getPosition().norm() * 100.0 / 10.0);
    if (groups.find(key) != groups.end())
    {
      groups[key].insert(std::make_pair(weight, i));
    }
    else
    {
      groups[key] = {std::make_pair(weight, i)};
    }
  }

  std::vector<std::pair<int, Group>> sorted_groups(groups.begin(), groups.end());
  // comparator body 0x1801826F0 (std::sort helpers 0x18017B7E0..0x18017D330)
  std::sort(sorted_groups.begin(), sorted_groups.end(),
            [](const std::pair<int, Group>& a, const std::pair<int, Group>& b) {
              float sum_a = 0.f;
              for (const auto& e : a.second)
              {
                sum_a += e.first;
              }
              float sum_b = 0.f;
              for (const auto& e : b.second)
              {
                sum_b += e.first;
              }
              if (a.second.size() != 0 && b.second.size() != 0)
              {
                return sum_a / a.second.size() > sum_b / b.second.size();
              }
              return a.second.size() > b.second.size();
            });
  Group best = sorted_groups[0].second;

  static int best_size =
      getenv("PIMAX_LC_BSTSIZE") ? std::stoi(getenv("PIMAX_LC_BSTSIZE")) : 4;
  if (best.size() < static_cast<size_t>(best_size))
  {
    return false;
  }

  Eigen::Vector3d t_sum = Eigen::Vector3d::Zero();
  auto it = best.begin();
  for (size_t k = 0; k < best.size(); ++k)
  {
    t_sum += corrections[it->second].w_T_new_old_.getPosition();   // `it` is never advanced
  }
  *info = corrections[best.begin()->second];
  info->w_T_new_old_.getPosition() = t_sum / static_cast<double>(best.size());
  reloc_success_ = true;
  LOGW("Best reloc candidate size: %d\n", best.size());
  return true;
}

// ===============================================================================================
// 0x18018c070  recovery_kf -- pimax-new (free function, called by PlatMap::load / load_bin with
// the PlatMap*).  Rebuilds the mixed_* arrays when they are inconsistent with bow+svo features.
// ===============================================================================================
void recovery_kf(PlatMap* plat_map)
{
  for (auto& kfs : plat_map->kf_list_)
  {
    for (auto& kf : kfs)
    {
      if (kf->num_bow_features_ + kf->svo_features_.size() != kf->mixed_features_.size())
      {
        kf->mixed_keypoints_.clear();
        kf->mixed_keypoints_.insert(kf->mixed_keypoints_.end(), kf->bow_keypoints_.begin(),
                                    kf->bow_keypoints_.end());
        kf->mixed_keypoints_.insert(kf->mixed_keypoints_.end(), kf->svo_keypointsvector_.begin(),
                                    kf->svo_keypointsvector_.end());
        kf->mixed_features_.clear();
        kf->mixed_features_.insert(kf->mixed_features_.end(), kf->bow_features_.begin(),
                                   kf->bow_features_.end());
        kf->mixed_features_.insert(kf->mixed_features_.end(), kf->svo_features_.begin(),
                                   kf->svo_features_.end());
        kf->mixed_node_ids_.clear();
        kf->mixed_node_ids_.insert(kf->mixed_node_ids_.end(), kf->bow_node_ids_.begin(),
                                   kf->bow_node_ids_.end());
        kf->mixed_node_ids_.insert(kf->mixed_node_ids_.end(), kf->svo_node_ids_.begin(),
                                   kf->svo_node_ids_.end());
      }
    }
  }
}

// ===============================================================================================
// 0x18018c3a0  resetLoopClosing -- pimax-new.
// ===============================================================================================
void LoopClosing::resetLoopClosing()
{
  LOGI("resetLoopClosing\n");
  std::unique_lock<std::mutex> lock(completed_flags_mutex_);
  completed_flags_.clear();
  lock.unlock();
  std::unique_lock<std::mutex> kf_lock(kf_list_mutex_);
  kf_list_.clear();
  kf_lock.unlock();
  pose_key_map_.clear();
  svo_keyframe_count_ = 0;
}

// ===============================================================================================
// 0x18018c4e0  resetReLocalize -- pimax-new.  Ends the current session: if enough bundles were
// collected, drops the map with the fewest keyframes when too many maps are stored, merges the
// session into the PlatMap (BA first when the load/save worker is about to exit) and asks the
// worker to save.  map_id_ always advances.
// ===============================================================================================
void LoopClosing::resetReLocalize()
{
  LOGW("resetReLocalize start kf_list_loop.size() %d\n", kf_list_.size());
  std::unique_lock<std::mutex> kf_lock(kf_list_mutex_);
  const int n_kf_list = kf_list_.size();
  kf_lock.unlock();

  if (static_cast<size_t>(n_kf_list) >= options_.min_kf_to_save_map)
  {
    if (options_.use_plat_map)
    {
      PlatMap* plat_map = plat_map_.get();
      if (plat_map->map_index_.find(map_id_) == plat_map->map_index_.end() &&
          plat_map->map_index_.size() > options_.max_num_maps)
      {
        auto oldest = std::min_element(
            plat_map->map_index_.begin(), plat_map->map_index_.end(),
            [](const std::pair<const int, MapIndex>& a, const std::pair<const int, MapIndex>& b) {
              return a.second.kf_nums_ < b.second.kf_nums_;
            });
        int del_map_id = oldest->first;
        if (plat_map->kf_map_.find(del_map_id) == plat_map->kf_map_.end())
        {
          LOGW("INFO=PlatMap delete fail, as map_id %d not exists.\n", del_map_id);
        }
        else
        {
          plat_map->kf_map_.erase(del_map_id);
          plat_map->Map2Vec(del_map_id);
          // NOTE: "map_size" prints kf_list_.size() and "kf_size" prints map_index_.size().
          LOGI("INFO=PlatMap delete success||map_size=%d||kf_size=%d.\n",
               plat_map->kf_list_.size(), plat_map->map_index_.size());
        }
      }
      {
        std::lock_guard<std::mutex> lock(kf_list_mutex_);
        if (!kf_list_.empty())
        {
          if (platmap_cmd_ == 2)
          {
            bundleAdjustKfList(kf_list_);
          }
          plat_map_->UpdateMap(kf_list_, map_id_);
        }
      }
      plat_map_ready_ = true;
      platmap_cmd_ = 1;
      platmap_cond_.notify_one();
    }
    ++map_id_;
    reloc_busy_ = false;
    reloc_success_ = false;
    LOGI("resetReLocalize end\n");
  }
  else
  {
    ++map_id_;
    reloc_busy_ = false;
  }
}

// ===============================================================================================
// 0x18018c850  runPROnLatestKeyframe -- upstream-modified (heavily).  Runs on the loop-closing
// worker with the PlatMap mutex held.  Only builds the database entries of the newest bundle
// (BoW + SVO descriptors); the actual loop detection is gone.  Both parameters are unused.
// ===============================================================================================
void LoopClosing::runPROnLatestKeyframe(const size_t /*n_ignored_latest*/,
                                        const bool /*run_lc_on_this_frame*/)
{
  std::lock_guard<std::mutex> map_lock(plat_map_->mtx_);
  vk::Timer timer_total;
  vk::Timer timer_each;
  timer_total.start();
  timer_each.start();

  std::vector<KeyFramePtr> cur_kfs;
  {
    std::unique_lock<std::mutex> lock(kf_list_mutex_);
    if (kf_list_.empty())
    {
      return;
    }
    cur_kfs = kf_list_.back();
    lock.unlock();
  }
  const int current_frame_ID = cur_kfs[0]->NframeID_;
  for (auto& kf : cur_kfs)
  {
    kf->lc_frame_count_ = ++lc_frame_count_;
  }

  if (options_.enable_image_logging)
  {
    int cam_idx = 0;
    for (auto& kf : cur_kfs)
    {
      const std::string img_path = options_.image_log_base_path +
                                   std::to_string(current_frame_ID) + "_" +
                                   std::to_string(cam_idx) + ".jpg";
      cv::imwrite(img_path, kf->keyframe_image_);
      ++cam_idx;
    }
  }

  timer_each.stop();
#line 658
  VLOG(40) << "INFO=LC-1 data preprocess finish";   // line 658
#line 1170
  timer_each.start();
  for (auto& kf : cur_kfs)
  {
    extractBoWFeaturesFromImage(kf->keyframe_image_, &kf->bow_keypoints_, &kf->bow_features_);
    kf->num_bow_features_ = kf->bow_features_.size();
  }

  bool bow_features_too_few = false;
  int min_bow_features = 10000000;
  size_t landmark_nums = 0;
  for (auto& kf : cur_kfs)
  {
    const size_t n_bow = kf->bow_keypoints_.size();
    bow_features_too_few |= n_bow < options_.min_bow_features;
    min_bow_features = std::min(min_bow_features, static_cast<int>(n_bow));
    landmark_nums += kf->svo_landmarksvector_cam_.size();
  }
  if (bow_features_too_few)
  {
#line 694
    VLOG(40) << "INFO=LC-2 bow features is less||bow_features_num=" << min_bow_features
             << std::endl;   // line 694
#line 1193
    {
      std::lock_guard<std::mutex> lock(kf_list_mutex_);
      kf_list_.pop_back();
    }
    std::unique_lock<std::mutex> lock(completed_flags_mutex_);
    completed_flags_.back() = true;
    lock.unlock();
    return;
  }

  timer_each.stop();
#line 707
  VLOG(40) << "INFO=LC-2 feature extract finish||landmarkNums=" << landmark_nums;   // line 707
#line 1207
  timer_each.start();
  for (auto& kf : cur_kfs)
  {
    createBOW(kf->bow_features_, voc_, &kf->vec_bow_, &kf->bow_node_ids_);
    updateSVOPointsDescriptors(kf, false);
    if (!options_.enable_image_logging)
    {
      kf->keyframe_image_.release();
    }
    timer_each.stop();
#line 723
    VLOG(40) << "INFO=LC-3 feature encoding finish";   // line 723
#line 1220
  }

  if (options_.skip_loop_detection)
  {
    std::unique_lock<std::mutex> lock(completed_flags_mutex_);
    completed_flags_.back() = true;
    lock.unlock();
    return;
  }

  {
    std::unique_lock<std::mutex> kf_lock(kf_list_mutex_);
    const size_t n_kf_list = kf_list_.size();
    if (n_kf_list >= 2)
    {
      for (size_t i = 0; i < kf_list_.back().size(); ++i)
      {
        // result unused (upstream: score_expected for the BoW check, now dead)
        compareBOWs(kf_list_.back()[i]->vec_bow_, kf_list_[kf_list_.size() - 2][i]->vec_bow_,
                    voc_);
      }
    }
  }
  std::unique_lock<std::mutex> lock(completed_flags_mutex_);
  completed_flags_.back() = true;
  lock.unlock();
}

// ===============================================================================================
// 0x18018d770  runReLocalization -- pimax-new (26 KB).  Runs on the relocalization worker for the
// two KeyFrames queued by reLocalize():
//   ReLoc-1  take the queued stereo pair,
//   ReLoc-2  BoW + SVO descriptors (only if missing), abort if any frame has too few SVO keypoints,
//   ReLoc-3  BoW vectors,
//   ReLoc-4  score EVERY PlatMap bundle (sum of per-camera DBoW2 scores, no threshold) into a
//            multimap sorted by descending score ("spatial grouping" step is empty),
//   ReLoc-5  for the best min(n, PIMAX_LC_TM, options.reloc_max_candidates) candidates:
//            ORB ratio-test matching cur0<->map0 and map0<->map1, map stereo triangulation
//            (DLT on bearings), solvePnPRansac (K = I) -> T_map0_cur0, direct epipolar matching of
//            the inliers cur0 -> cur1, ceres refinement of T_map0_cur0 on both cameras and, if
//            converged and close to the PnP result, a yaw-only map<-odom correction is queued.
// Environment tunables (read at every call): PIMAX_LC_HPS (4, only printed), PIMAX_LC_RE (2,
// PnP reprojection error), PIMAX_LC_DT (18, max squared px distance), PIMAX_LC_VNT (10, min valid
// epipolar matches), PIMAX_LC_OIN (20, only printed), PIMAX_LC_RATIO (80 -> 0.8 ratio test),
// PIMAX_LC_SNT (5, min matches / min triangulated points), PIMAX_LC_DRAW (0, debug window),
// PIMAX_LC_TM (10, max candidates).
// ===============================================================================================
void LoopClosing::runReLocalization()
{
  vk::Timer timer_total;
  vk::Timer timer_each;
  timer_total.start();
  timer_each.start();

  std::vector<KeyFramePtr> cur_kfs;
  {
    std::unique_lock<std::mutex> lock(reloc_frames_mutex_);
    if (reloc_kf_list_.size() < 2)
    {
      return;
    }
    cur_kfs.push_back(reloc_kf_list_[0]);
    cur_kfs.push_back(reloc_kf_list_[1]);
  }

  size_t landmark_nums = 0;
  for (auto& kf : cur_kfs)
  {
    landmark_nums += kf->svo_landmarksvector_cam_.size();
  }
  timer_each.stop();
#line 1952
  VLOG(40) << "INFO=ReLoc-1 choice frame is finish||landmark_nums=" << landmark_nums
           << std::endl;   // line 1952
#line 1295
  timer_each.start();

  for (auto& kf : cur_kfs)
  {
    if (kf->bow_features_.empty())
    {
      extractBoWFeaturesFromImage(kf->keyframe_image_, &kf->bow_keypoints_, &kf->bow_features_);
      kf->num_bow_features_ = kf->bow_features_.size();
    }
    if (kf->mixed_features_.empty())
    {
      updateSVOPointsDescriptors(kf, true);
    }
  }
  for (int i = 0; i < cur_kfs.size(); ++i)   // int index, unsigned compare (as in the binary)
  {
    const size_t n_keypoints = cur_kfs[i]->svo_keypointsvector_.size();
#line 1970
    VLOG(40) << "[Reloc-2] Frame index: " << i << ", keypoints feature num: " << n_keypoints
             << std::endl;   // line 1970
#line 1316
    if (n_keypoints < options_.min_bow_features)
    {
      return;
    }
  }
  timer_each.stop();
#line 1981
  VLOG(40) << "INFO=ReLoc-2 feature extract finish" << std::endl;   // line 1981
#line 1325
  timer_each.start();

  for (auto& kf : cur_kfs)
  {
    if (kf->vec_bow_.empty())
    {
      createBOW(kf->bow_features_, voc_, &kf->vec_bow_, &kf->bow_node_ids_);
    }
  }
  timer_each.stop();
#line 1992
  VLOG(40) << "INFO=RecLoc-3 feature encoding finish";   // line 1992
#line 1338
  timer_each.start();

  const double score_expected = options_.reloc_expected_score;
  last_reloc_kfs_ = cur_kfs;

  // ---- ReLoc-4: image query ----------------------------------------------------------------
  std::multimap<double, size_t, std::greater<double>> candidates;
  const size_t n_map_bundles = plat_map_->kf_list_.size();
  for (size_t i = 0; i < n_map_bundles; ++i)
  {
    if (plat_map_->kf_list_[i].size() >= cur_kfs.size())
    {
      double score = 0.0;
      size_t c = 0;
      for (auto& kf : cur_kfs)
      {
        score += compareBOWs(kf->vec_bow_, plat_map_->kf_list_[i][c]->vec_bow_, voc_);
        ++c;
      }
      candidates.insert(std::make_pair(score, i));
#line 2029
      VLOG(40) << "INFO=ReLoc-4 query score is pass||query_id=" << cur_kfs[0]->NframeID_
               << "||candidate_id=" << plat_map_->kf_list_[i][0]->NframeID_ << "||score="
               << score << "||param_min_score=" << options_.reloc_min_score
               << "||param_expected_score=" << score_expected;   // line 2029
#line 1364
    }
  }
  timer_each.stop();
#line 2035
  VLOG(40) << "INFO=ReLoc-4 image query finish||query_pass=" << candidates.size();   // line 2035
#line 1370
  const double t_spatial_grouping = timer_each.stop();
#line 2066
  VLOG(40) << "INFO=ReLoc-4.3 spatial grouping finish\n";   // line 2066
#line 1374
  timer_each.start();

  // ---- tunables -------------------------------------------------------------------------------
  const int hps = getenv("PIMAX_LC_HPS") ? std::stoi(getenv("PIMAX_LC_HPS")) : 4;
  const int reproj_err = getenv("PIMAX_LC_RE") ? std::stoi(getenv("PIMAX_LC_RE")) : 2;
  const int dist_thresh = getenv("PIMAX_LC_DT") ? std::stoi(getenv("PIMAX_LC_DT")) : 18;
  const int valid_num_thresh = getenv("PIMAX_LC_VNT") ? std::stoi(getenv("PIMAX_LC_VNT")) : 10;
  const int oin = getenv("PIMAX_LC_OIN") ? std::stoi(getenv("PIMAX_LC_OIN")) : 20;
  const int ratio_percent = getenv("PIMAX_LC_RATIO") ? std::stoi(getenv("PIMAX_LC_RATIO")) : 80;
  const int min_match_num = getenv("PIMAX_LC_SNT") ? std::stoi(getenv("PIMAX_LC_SNT")) : 5;
  const int draw = getenv("PIMAX_LC_DRAW") ? std::stoi(getenv("PIMAX_LC_DRAW")) : 0;
  const int top_n = getenv("PIMAX_LC_TM") ? std::stoi(getenv("PIMAX_LC_TM")) : 10;
  const float ratio = ratio_percent / 100.0f;
  static bool param_printed = false;
  if (!param_printed)
  {
    LOGW("[Reloc param] %d, %d, %d, %d, %d, %f\n", hps, reproj_err, dist_thresh,
         valid_num_thresh, oin, ratio);
    param_printed = true;
  }

  int n_checked = 0;
  const int n_query_pass = candidates.size();
  const int max_candidates = std::min(n_query_pass, std::min(top_n, options_.reloc_max_candidates));

  cv::BFMatcher matcher(cv::NORM_HAMMING, false);
  CameraPtr cam0;
  CameraPtr cam1;
  cv::Mat query_descriptors;
  bool query_descriptors_ready = false;
  Transformation T_c1_c0;
  Transformation T_c0_c1;
  Eigen::Matrix<double, 3, 4> P0;
  Eigen::Matrix<double, 3, 4> P1;
  cv::Mat K;
  cv::Mat dist_coeffs;
  bool cameras_ready = false;
  std::unique_ptr<Frame> frame0;
  std::unique_ptr<Frame> frame1;
  std::unique_ptr<Matcher> feature_matcher;

  for (auto it = candidates.begin(); it != candidates.end() && n_checked < max_candidates;
       ++it, ++n_checked)
  {
    const size_t cand_idx = it->second;
    if (cand_idx >= plat_map_->kf_list_.size())
    {
      continue;
    }
    const std::vector<KeyFramePtr>& cand_kfs = plat_map_->kf_list_[cand_idx];
    if (cand_kfs.size() < cur_kfs.size())
    {
      continue;
    }
    bool enough_bow = true;
    for (const auto& kf : cand_kfs)
    {
      enough_bow = (kf->bow_keypoints_.size() >= options_.reloc_min_bow_keypoints) ? enough_bow
                                                                                    : false;
    }
    if (!enough_bow)
    {
      continue;
    }

    // ---- ORB matching ------------------------------------------------------------------------
    if (!query_descriptors_ready)
    {
      cv::vconcat(cur_kfs[0]->bow_features_, query_descriptors);
      query_descriptors_ready = true;
    }
    std::vector<std::vector<cv::DMatch>> knn_cur_map0;
    std::vector<std::vector<cv::DMatch>> knn_map0_map1;
    cv::Mat map0_descriptors;
    cv::Mat map1_descriptors;
    cv::vconcat(cand_kfs[0]->bow_features_, map0_descriptors);
    cv::vconcat(cand_kfs[1]->bow_features_, map1_descriptors);
    matcher.knnMatch(query_descriptors, map0_descriptors, knn_cur_map0, 2);
    matcher.knnMatch(map0_descriptors, map1_descriptors, knn_map0_map1, 2);

    std::vector<cv::DMatch> good_map0_map1;
    std::vector<cv::DMatch> good_cur_map0;
    for (const auto& m : knn_cur_map0)
    {
      if (m.size() > 1 && ratio * m[1].distance > m[0].distance)
      {
        good_cur_map0.push_back(m[0]);
      }
    }
    for (const auto& m : knn_map0_map1)
    {
      if (m.size() > 1 && m[1].distance * 0.9 > m[0].distance)
      {
        good_map0_map1.push_back(m[0]);
      }
    }
    std::unordered_map<int, int> map0_to_map1;
    for (const auto& m : good_map0_map1)
    {
      map0_to_map1[m.queryIdx] = m.trainIdx;
    }
    // (cur0 kp, map0 kp, map1 kp)
    std::vector<std::tuple<cv::Point2f, cv::Point2f, cv::Point2f>> matches;
    for (const auto& m : good_cur_map0)
    {
      int map0_idx = m.trainIdx;
      if (map0_to_map1.find(map0_idx) != map0_to_map1.end())
      {
        matches.emplace_back(cur_kfs[0]->bow_keypoints_[m.queryIdx],
                             cand_kfs[0]->bow_keypoints_[map0_idx],
                             cand_kfs[1]->bow_keypoints_[map0_to_map1[map0_idx]]);
      }
    }
    if (matches.size() < static_cast<size_t>(min_match_num))
    {
      continue;
    }

    if (!cameras_ready)
    {
      cam0 = cams_->getCameraShared(0);
      cam1 = cams_->getCameraShared(1);
      T_c1_c0 = cur_kfs[1]->T_w_c_.inverse() * cur_kfs[0]->T_w_c_;
      T_c0_c1 = T_c1_c0.inverse();
      P0 = Eigen::Matrix<double, 3, 4>::Identity();
      P1.block<3, 3>(0, 0) = T_c1_c0.getRotationMatrix();
      P1.col(3) = T_c1_c0.getPosition();
      K = cv::Mat::eye(3, 3, CV_32F);
      dist_coeffs = cv::Mat::zeros(4, 1, CV_32F);
      cameras_ready = true;
    }

    // ---- stereo triangulation in the map frame (map cam0) ------------------------------------
    Eigen::Matrix2Xd px_map0;
    Eigen::Matrix2Xd px_map1;
    Eigen::Matrix2Xd px_cur;
    Eigen::Matrix3Xf f_map0;
    Eigen::Matrix3Xf f_map1;
    Eigen::Matrix3Xf f_cur;
    px_map0.resize(2, matches.size());
    px_map1.resize(2, matches.size());
    px_cur.resize(2, matches.size());
    f_map0.resize(3, matches.size());
    f_map1.resize(3, matches.size());
    f_cur.resize(3, matches.size());
    int64_t col = 0;
    for (const auto& m : matches)
    {
      px_map0.col(col) = Eigen::Vector2d(std::get<1>(m).x, std::get<1>(m).y);
      px_map1.col(col) = Eigen::Vector2d(std::get<2>(m).x, std::get<2>(m).y);
      px_cur.col(col) = Eigen::Vector2d(std::get<0>(m).x, std::get<0>(m).y);
      ++col;
    }
    backProject(px_map0, cam0, &f_map0);
    backProject(px_map1, cam1, &f_map1);
    backProject(px_cur, cam0, &f_cur);

    std::vector<cv::Point3f> pts3d;
    std::vector<cv::Point2f> pts2d;
    std::vector<cv::Point2f> px_cur_kept;
    std::vector<Eigen::Vector3f> f_cur_kept;
    const int n_matches = matches.size();
    for (int i = 0; i < n_matches; ++i)
    {
      Eigen::Vector3d p_map0;
      const bool ok = triangulate(f_map0.col(i), f_map1.col(i), cam0, cam1, T_c1_c0, P0, P1,
                                  &p_map0);
      if (p_map0.z() <= 15.0 && ok)
      {
        const Eigen::Vector3f fc = f_cur.col(i);
        pts3d.emplace_back(p_map0.x(), p_map0.y(), p_map0.z());
        pts2d.emplace_back(fc(0), fc(1));
        px_cur_kept.emplace_back(px_cur.col(i).x(), px_cur.col(i).y());
        f_cur_kept.emplace_back(f_cur.col(i));
      }
    }
    if (pts3d.size() < static_cast<size_t>(min_match_num))
    {
      continue;
    }

    // ---- PnP: T_map0_cur0 ------------------------------------------------------------------------
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat R_wc;
    cv::Mat t_wc;
    std::vector<int> inliers;
    cv::solvePnPRansac(pts3d, pts2d, K, dist_coeffs, rvec, tvec, false, 200,
                       static_cast<float>(reproj_err), 0.999, inliers, cv::SOLVEPNP_ITERATIVE);
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    R_wc = R.inv();
    t_wc = -R.inv() * tvec;
    Eigen::Vector3d eigenTVecWorld;
    Eigen::Matrix3d eigenRMatWorld;
    cv::cv2eigen(t_wc, eigenTVecWorld);
    cv::cv2eigen(R_wc, eigenRMatWorld);
    if (eigenTVecWorld.norm() > 2.0)
    {
      LOGW("eigenTVecWorld norm gt 2\n");
      continue;
    }
    const Transformation T_map0_cur0(eigenTVecWorld, Eigen::Quaterniond(eigenRMatWorld));

    std::vector<cv::Point3f> inlier_pts3d;
    std::vector<cv::Point3f> pts_cur0;
    std::vector<cv::Point2f> inlier_px;
    std::vector<Eigen::Vector3f> inlier_f;
    for (int idx : inliers)
    {
      inlier_pts3d.push_back(pts3d[idx]);
      inlier_px.push_back(px_cur_kept[idx]);
      inlier_f.push_back(f_cur_kept[idx]);
    }
    Transformation T_cur0_map0 = T_map0_cur0.inverse();
    T_cur0_map0.getRotation().normalize();
    Transformation T_cur1_map0 = T_c1_c0 * T_cur0_map0;
    T_cur1_map0.getRotation().normalize();

    std::vector<cv::Point2f> reproj_cur1;
    for (const auto& P : inlier_pts3d)
    {
      Eigen::Vector3d P_map0;
      P_map0 << P.x, P.y, P.z;
      const Eigen::Vector3d p_cur1 = T_cur1_map0.transform(P_map0);
      const Eigen::Vector3d p_cur0 = T_cur0_map0.transform(P_map0);
      Eigen::Vector2d px_cur1;
      cam1->project3(p_cur1, &px_cur1, nullptr);
      pts_cur0.emplace_back(p_cur0.x(), p_cur0.y(), p_cur0.z());
      reproj_cur1.emplace_back(px_cur1.x(), px_cur1.y());
    }

    // ---- direct epipolar matching cur0 -> cur1 ---------------------------------------------------
    int n_valid = 0;
    float total_dist = 0.f;
    std::vector<cv::Point2f> tracked_cur1;
    tracked_cur1.reserve(inlier_px.size());
    LOGI("inliner_pixels size: %d\n", inlier_px.size());
    if (!feature_matcher)
    {
      frame0 = std::make_unique<Frame>(0, static_cast<int64_t>(cur_kfs[0]->timestamp_sec_abs_ *
                                                                1000000000.0),
                                       cam0, cur_kfs[0]->T_w_c_);
      frame1 = std::make_unique<Frame>(1, static_cast<int64_t>(cur_kfs[1]->timestamp_sec_abs_ *
                                                                1000000000.0),
                                       cam1, cur_kfs[1]->T_w_c_);
      frame_utils::createImgPyramid(cur_kfs[0]->keyframe_image_, 1, frame0->img_pyr_);
      frame_utils::createImgPyramid(cur_kfs[1]->keyframe_image_, 1, frame1->img_pyr_);
      feature_matcher = std::make_unique<Matcher>();
      feature_matcher->options_.max_epi_search_steps = 500;
      feature_matcher->options_.subpix_refinement = true;
    }
    for (size_t k = 0; k < inlier_px.size(); ++k)
    {
      FloatType depth = pts_cur0[k].z;   // float (findEpipolarMatchDirect writes it back)
      if (depth <= 0.0 || depth > 50.0)
      {
        tracked_cur1.push_back(cv::Point2f(-1.f, -1.f));
        continue;
      }
      FeatureType type = FeatureType::kCorner;
      Keypoint px(inlier_px[k].x, inlier_px[k].y);
      BearingVector f = inlier_f[k];
      BearingVector f_normalized = f.normalized();
      GradientVector grad = GradientVector::Zero();
      Score score = 0;
      Level level = 0;
      PointPtr landmark;
      SeedRef seed_ref(FramePtr(), -1);
      int track_id = -1;
      // FeatureWrapper(type, px, f, f_raw, grad, score, level, landmark, seed_ref, track_id),
      // out-of-line copy 0x18017F430: verified (B4) that the un-normalized bearing is stored in
      // `f` (+0x20) and the normalized one in `f_raw` (+0x38) -- swapped w.r.t. their meaning,
      // kept as in the binary.  type = kCorner (7).
      FeatureWrapper ref_ftr(type, px, f, f_normalized, grad, score, level, landmark, seed_ref,
                             track_id);
      if (feature_matcher->findEpipolarMatchDirect(*frame0, *frame1, T_c1_c0, ref_ftr,
                                                   1.0 / depth, 1.0 / (depth * 1.2),
                                                   1.0 / (depth * 0.8), depth) !=
          Matcher::MatchResult::kSuccess)
      {
        tracked_cur1.push_back(cv::Point2f(-1.f, -1.f));
        continue;
      }
      const cv::Point2f px_matched(feature_matcher->px_cur_(0), feature_matcher->px_cur_(1));
      const float dx = px_matched.x - reproj_cur1[k].x;
      const float dy = px_matched.y - reproj_cur1[k].y;
      const float d2 = dy * dy + dx * dx;
      if (d2 <= static_cast<float>(dist_thresh))
      {
        total_dist += d2;
        ++n_valid;
        tracked_cur1.push_back(px_matched);
      }
      else
      {
        tracked_cur1.push_back(cv::Point2f(-1.f, -1.f));
      }
    }

    if (n_valid >= valid_num_thresh)
    {
      // ---- ceres refinement of T_map0_cur0 on both cameras ------------------------------------
      std::shared_ptr<ceres_backend::Map> map_ptr = std::make_shared<ceres_backend::Map>(false);
      ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);   // never used (leak)
      (void)loss_function;
      Transformation T_s_c0;
      Transformation T_w_s;
      T_s_c0.getRotation() =
          kindr::minimal::RotationQuaternion(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
      T_w_s = T_map0_cur0;
      std::shared_ptr<ceres_backend::PoseParameterBlock> extrinsics0 =
          std::make_shared<ceres_backend::PoseParameterBlock>(T_s_c0, 0);
      std::shared_ptr<ceres_backend::PoseParameterBlock> extrinsics1 =
          std::make_shared<ceres_backend::PoseParameterBlock>(T_c0_c1, 1);
      map_ptr->addParameterBlock(extrinsics0, ceres_backend::Map::Pose6d);
      // Map::setParameterBlockConstant(shared_ptr) / Map::solve() are inline in ceres_map.hpp;
      // their out-of-line copies 0x180195AB0 / 0x180195B50 are emitted in loop_closing.obj.
      map_ptr->setParameterBlockConstant(extrinsics0);
      map_ptr->addParameterBlock(extrinsics1, ceres_backend::Map::Pose6d);
      map_ptr->setParameterBlockConstant(extrinsics1);
      std::shared_ptr<ceres_backend::PoseParameterBlock> pose =
          std::make_shared<ceres_backend::PoseParameterBlock>(T_w_s, 2);
      map_ptr->addParameterBlock(pose, ceres_backend::Map::Pose6d);
      for (size_t kk = 0; kk < inlier_px.size(); ++kk)
      {
        Eigen::Vector2d obs1;
        obs1(0) = tracked_cur1[kk].x;
        obs1(1) = tracked_cur1[kk].y;
        if (obs1(0) >= 0.0 && obs1(1) >= 0.0)
        {
          Eigen::Vector2d obs0;
          obs0(0) = inlier_px[kk].x;
          obs0(1) = inlier_px[kk].y;
          uint64_t landmark_id = kk + 10;
          const Eigen::Vector3d lm(inlier_pts3d[kk].x, inlier_pts3d[kk].y, inlier_pts3d[kk].z);
          std::shared_ptr<ceres_backend::General3DParameterBlock> landmark_block =
              std::make_shared<ceres_backend::General3DParameterBlock>(lm, landmark_id);
          map_ptr->addParameterBlock(landmark_block, ceres_backend::Map::Trivial);
          map_ptr->setParameterBlockConstant(landmark_block);
          // information defaults to Identity inside make_shared (ctor default argument)
          std::shared_ptr<ceres_backend::ReprojectionError> error0 =
              std::make_shared<ceres_backend::ReprojectionError>(cam0, obs0);
          std::vector<std::shared_ptr<ceres_backend::ParameterBlock>> blocks0{
              pose, landmark_block, extrinsics0};
          map_ptr->addResidualBlock(error0, nullptr, blocks0);
          std::shared_ptr<ceres_backend::ReprojectionError> error1 =
              std::make_shared<ceres_backend::ReprojectionError>(cam1, obs1);
          std::vector<std::shared_ptr<ceres_backend::ParameterBlock>> blocks1{
              pose, landmark_block, extrinsics1};
          map_ptr->addResidualBlock(error1, nullptr, blocks1);
        }
      }
      try
      {
        map_ptr->options.linear_solver_type = ceres::DENSE_SCHUR;
        map_ptr->options.minimizer_progress_to_stdout = false;
        map_ptr->options.trust_region_strategy_type = ceres::DOGLEG;
        map_ptr->options.max_num_iterations = 100;
        map_ptr->options.num_threads = 1;
        map_ptr->options.function_tolerance = 1e-6;
        map_ptr->solve();
      }
      catch (const std::exception& e)
      {
        LOGW("ceres solve error: %s\n", e.what());
        continue;
      }
      catch (...)
      {
        LOGW("ceres solve error\n");
        continue;
      }

      if (map_ptr->summary.termination_type == ceres::CONVERGENCE)
      {
        const Transformation T_w_s_opt = pose->estimate();
        if ((T_w_s_opt.getPosition() - T_map0_cur0.getPosition()).norm() <= 0.2)
        {
          Transformation T_correction =
              cand_kfs[0]->T_w_c_ * pose->estimate() * cur_kfs[0]->T_w_c_.inverse();
          T_correction.getRotation().normalize();
          // keep only the yaw of the correction
          const Eigen::Vector3d ypr = Utility::R2ypr(T_correction.getRotationMatrix(), false);   // 0x1800f3260
          T_correction.getRotation() = kindr::minimal::RotationQuaternion(
              Utility::ypr2R(Eigen::Vector3d(ypr(0), 0.0, 0.0), false));   // 0x18017ee10
          const float confidence = total_dist / n_valid;
          LOGW("Adding correction info: map_id: %d, timestamp: %f, confidence: %f, correctT: %f, "
               "%f, %f, %f, %f, %f\n",
               cand_kfs[0]->map_id_, cur_kfs[0]->timestamp_sec_abs_, confidence,
               T_correction.getPosition().x(), T_correction.getPosition().y(),
               T_correction.getPosition().z(), T_correction.getRotation().x(),
               T_correction.getRotation().y(), T_correction.getRotation().z());
          {
            std::lock_guard<std::mutex> lock(reloc_info_lock_);
            reLoc_correction_info_.emplace_back(cand_kfs[0]->map_id_,
                                                cur_kfs[0]->timestamp_sec_abs_, confidence,
                                                T_correction);
          }
          if (draw)
          {
            cv::Mat img_cur0;
            cv::Mat img_cur1;
            cv::Mat img_map0;
            cv::Mat img_map1;
            cv::Mat show_cur0;
            cv::Mat show_cur1;
            cv::Mat show_map0;
            cv::Mat show_map1;
            img_cur0 = cur_kfs[0]->keyframe_image_;
            img_cur1 = cur_kfs[1]->keyframe_image_;
            img_map0 = plat_map_->kf_list_[cand_idx][0]->keyframe_image_;
            img_map1 = plat_map_->kf_list_[cand_idx][1]->keyframe_image_;
            cv::cvtColor(img_cur0, show_cur0, cv::COLOR_GRAY2BGR);
            cv::cvtColor(img_cur1, show_cur1, cv::COLOR_GRAY2BGR);
            cv::cvtColor(img_map0, show_map0, cv::COLOR_GRAY2BGR);
            cv::cvtColor(img_map1, show_map1, cv::COLOR_GRAY2BGR);
            drawText(show_cur0, "Realtime Image cam 0", cv::Point(20, 40), 1.0,
                     cv::Scalar(0, 0, 255), 2);
            drawText(show_cur1, "Realtime Image cam 1", cv::Point(20, 40), 1.0,
                     cv::Scalar(0, 0, 255), 2);
            drawText(show_map0, "Map Image cam 0", cv::Point(20, 40), 1.0, cv::Scalar(0, 0, 255),
                     2);
            drawText(show_map1, "Map Image cam 1", cv::Point(20, 40), 1.0, cv::Scalar(0, 0, 255),
                     2);
            drawText(show_cur1, "valid number " + std::to_string(n_valid), cv::Point(20, 80), 1.0,
                     cv::Scalar(0, 0, 255), 2);
            drawText(show_cur1,
                     "avg distance " + std::to_string(n_valid > 0 ? total_dist / n_valid : 0.0f),
                     cv::Point(20, 120), 1.0, cv::Scalar(0, 0, 255), 2);
            drawText(show_cur1, "total_distance " + std::to_string(total_dist),
                     cv::Point(20, 160), 1.0, cv::Scalar(0, 0, 255), 2);

            cv::Mat top;
            cv::Mat bottom;
            cv::Mat unused;
            std::vector<cv::KeyPoint> kps_cur0;
            std::vector<cv::KeyPoint> kps_cur1;
            std::vector<cv::KeyPoint> kps_map0;
            std::vector<cv::KeyPoint> kps_map1;
            for (const auto& p : cur_kfs[0]->bow_keypoints_)
            {
              kps_cur0.emplace_back(p.x, p.y, -1);
            }
            for (const auto& p : cur_kfs[1]->bow_keypoints_)
            {
              kps_cur1.emplace_back(p.x, p.y, -1);
            }
            for (const auto& p : plat_map_->kf_list_[cand_idx][0]->bow_keypoints_)
            {
              kps_map0.emplace_back(p.x, p.y, -1);
            }
            for (const auto& p : plat_map_->kf_list_[cand_idx][1]->bow_keypoints_)
            {
              kps_map1.emplace_back(p.x, p.y, -1);
            }
            for (size_t mm = 0; mm < tracked_cur1.size(); ++mm)
            {
              const float ty = tracked_cur1[mm].y;
              const float tx = tracked_cur1[mm].x;
              const float ry = reproj_cur1[mm].y;
              const float rx = reproj_cur1[mm].x;
              if (tx >= 0.0 && ty >= 0.0)
              {
                cv::circle(show_cur1, cv::Point((int)tx, (int)ty), 3, cv::Scalar(255, 0, 0), -1,
                           8, 0);
                cv::circle(show_cur1, cv::Point((int)rx, (int)ry), 3, cv::Scalar(0, 255, 0), -1,
                           8, 0);
              }
            }
            cv::hconcat(show_cur0, show_map0, top);
            cv::hconcat(show_cur1, show_map1, bottom);
            cv::Mat all;
            cv::vconcat(top, bottom, all);
            for (size_t i = 0; i < inlier_px.size() && i < reproj_cur1.size(); ++i)
            {
              const cv::Point2f& p0 = inlier_px[i];
              const cv::Point2f& p1 = tracked_cur1[i];
              if (p1.x >= 0.0 && p1.y >= 0.0)
              {
                const float rows = static_cast<float>(show_cur1.rows);
                cv::line(all, cv::Point((int)p0.x, (int)p0.y),
                         cv::Point((int)p1.x, (int)(p1.y + rows)), cv::Scalar(255, 0, 0), 1, 8,
                         0);
              }
            }
            cv::Mat all_copy = all.clone();
            cv::imshow("Good Matches " + std::to_string(std::distance(candidates.begin(), it)),
                       all);
            cv::waitKey(0);
          }
        }
      }
    }
  }

  if (!options_.keep_reloc_images)
  {
    for (auto& kf : cur_kfs)
    {
      kf->keyframe_image_.release();
    }
  }
  const double t_rerank = timer_each.stop();
#line 2638
  VLOG(40) << "INFO=ReLoc-5 image re-rank finish||rerank_pass=0, cost time: "
           << t_rerank - t_spatial_grouping << std::endl;   // line 2638
#line 1882
  timer_each.stop();
  timer_total.stop();
}

// ===============================================================================================
// 0x180193e00  save -- pimax-new.  Saves only if the PlatMap holds more than 20 bundles and saving
// is enabled (not kLabLoc).  plat_map_ready_ is false while saving.
// ===============================================================================================
void LoopClosing::save()
{
  plat_map_ready_ = false;
  if (plat_map_->kf_list_.size() > 20 && save_map_enabled_)
  {
    saveIndex();
    std::string path = options_.map_path + map_tag_ + options_.map_name;
    plat_map_->save(path);
    LOGI("re-localization, save plat map success\n");
  }
  plat_map_ready_ = true;
}

// ===============================================================================================
// 0x180194020  saveIndex -- pimax-new.  In kLabMap mode with a detected tag the tag index is
// written into the SAME yaml (overwriting PlatMap::saveIndex's output, see saveTagIndex).
// ===============================================================================================
bool LoopClosing::saveIndex()
{
  plat_map_ready_ = false;
  std::string path = options_.map_path + map_tag_ + options_.map_index_name;
  plat_map_->saveIndex(path);
  if (tag_save_.valid && mode_ == LoopClosingMode::kLabMap)
  {
    saveTagIndex(path);
  }
  return true;
}

// ===============================================================================================
// 0x180194200  saveTagIndex -- pimax-new.  Re-reads "map_index_" from the yaml and REWRITES the
// file with map_index_ + tag_index_ only.  QUIRKS (kept): "map_version_" is dropped (loadIndex will
// then refuse the map), and the tag keys carry a trailing space ("map_id_ ", "frame_id_ ",
// "board_idx ", "T ", "Q ") so loadTagIndex (which looks up "map_id_" etc.) cannot read them back.
// ===============================================================================================
bool LoopClosing::saveTagIndex(const std::string& path)
{
  cv::FileStorage fs;
  fs.open(path, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);
  if (!fs.isOpened())
  {
    LOGE("Failed to open file %s\n", path.c_str());
    return false;
  }
  cv::FileNode map_index_node = fs["map_index_"];
  if (map_index_node.empty() || map_index_node.size() == 0)
  {
    return false;
  }
  std::map<int, MapIndex> map_index;
  for (cv::FileNodeIterator it = map_index_node.begin(); it != map_index_node.end(); ++it)
  {
    cv::FileNode node = *it;
    MapIndex index;
    index.map_id_ = (int)node["map_id_"];
    index.kf_nums_ = (int)node["kf_nums_"];
    index.newest_timestamp_ = (double)node["newest_timestamp_"];
    index.startIdx_ = (int)node["startIdx_"];
    index.endIdx_ = (int)node["endIdx_"];
    map_index[index.map_id_] = index;
  }

  fs.open(path, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
  fs << "map_index_" << "[";
  for (const auto& kv : map_index)
  {
    const MapIndex index = kv.second;
    fs << "{";
    fs << "map_id_" << kv.first;
    fs << "kf_nums_" << index.kf_nums_;
    fs << "newest_timestamp_" << index.newest_timestamp_;
    fs << "startIdx_" << index.startIdx_;
    fs << "endIdx_" << index.endIdx_ << "}";
  }
  fs << "]";
  fs << "tag_index_" << "[";
  for (int k = 0; k < tag_save_.board_idx.size(); ++k)
  {
    fs << "{";
    fs << "map_id_ " << tag_save_.map_id_;
    fs << "frame_id_ " << tag_save_.frame_id_;
    fs << "board_idx " << tag_save_.board_idx[k];
    fs << "T " << "[" << tag_save_.T[k](0) << tag_save_.T[k](1) << tag_save_.T[k](2);
    fs << "]";
    fs << "Q " << "[" << tag_save_.Q[k](0) << tag_save_.Q[k](1) << tag_save_.Q[k](2)
       << tag_save_.Q[k](3);
    fs << "]";
    fs << "}";
  }
  fs << "]";
  fs.release();
  LOGI("success save tag index\n");
  return true;
}

// ===============================================================================================
// 0x180195b70  startAllThread -- pimax-new.  Called at the end of the ctor.
// ===============================================================================================
void LoopClosing::startAllThread()
{
  if (lc_thread_)
  {
    LOGE("LoopClosing: Thread already started!\n");
    return;
  }
  LOGI("LoopClosing: Start thread.\n");
  lc_thread_ = std::unique_ptr<std::thread>(new std::thread(&LoopClosing::loopClosingThread, this));

  if (reloc_thread_)
  {
    LOGE("ReLocalize: Thread already started!\n");
    return;
  }
  LOGI("ReLocalize: Start thread.\n");
  reloc_thread_ = std::unique_ptr<std::thread>(new std::thread(&LoopClosing::reLocalizeThread, this));

  if (load_save_thread_)
  {
    LOGE("LoadSavePlatMap: Thread already started!\n");
    return;
  }
  LOGI("LoadSavePlatMap: Start thread.\n");
  load_save_thread_ =
      std::unique_ptr<std::thread>(new std::thread(&LoopClosing::loadSavePlatMapThread, this));
}

// ===============================================================================================
// 0x180195e60  stopAllThread -- pimax-new.  Called first in the dtor.  The load/save worker is
// stopped through resetReLocalize() with platmap_cmd_ = 2 (=> BA + UpdateMap + final save).
// ===============================================================================================
void LoopClosing::stopAllThread()
{
  if (lc_thread_)
  {
    LOGI("LoopClosing: interrupt and join thread... \n");
    lc_stop_ = true;
    lc_cond_.notify_all();
    if (lc_thread_->joinable())
    {
      lc_thread_->join();
    }
    lc_thread_.reset();
  }
  if (reloc_thread_)
  {
    LOGI("ReLocalize: interrupt and join thread... \n");
    reloc_stop_ = true;
    reloc_cond_.notify_all();
    if (reloc_thread_->joinable())
    {
      LOGI("ReLocalize: before join thread... \n");
      reloc_thread_->join();
    }
    reloc_thread_.reset();
  }
  LOGI("LoadSavePlatMap: before save thread join\n");
  if (load_save_thread_)
  {
    LOGI("LoadSavePlatMap: interrupt and join thread... \n");
    plat_map_ready_ = false;
    platmap_cmd_ = 2;
    resetReLocalize();
    platmap_cond_.notify_all();
    platmap_stop_ = true;
    LOGI("before thread_load_save_->joinable()\n");
    if (load_save_thread_->joinable())
    {
      load_save_thread_->join();
    }
    load_save_thread_.reset();
  }
  LOGI("stopAllThread finsh!!!\n");
  std::unique_lock<std::mutex> lock(kf_list_mutex_);
  kf_list_.clear();
  lock.unlock();
  plat_map_->kf_list_.clear();
  plat_map_->kf_map_.clear();
  ::pimax::g_logger.Close();   // inlined: if (m_file.is_open()) m_file.close()
}

// ===============================================================================================
// 0x1801963d0  triangulate -- pimax-new.  Linear (DLT) triangulation of one stereo match given as
// bearing vectors; only their x,y components are used (no division by z -- quirk).
// Returns false on degenerate solution / negative depth / projection failure in cam0.
// QUIRK (kept): a point behind cam1 returns TRUE without projecting into cam1.
// ===============================================================================================
bool LoopClosing::triangulate(const Eigen::Vector3f& f0, const Eigen::Vector3f& f1,
                              CameraPtr cam0, CameraPtr cam1, const Transformation& T_c1_c0,
                              const Eigen::Matrix<double, 3, 4>& P0,
                              const Eigen::Matrix<double, 3, 4>& P1, Eigen::Vector3d* p_c0)
{
  const double x0 = f0(0);
  const double y0 = f0(1);
  const double x1 = f1(0);
  const double y1 = f1(1);
  Eigen::Matrix4d A;
  A.row(0) = x0 * P0.row(2) - P0.row(0);
  A.row(1) = y0 * P0.row(2) - P0.row(1);
  A.row(2) = x1 * P1.row(2) - P1.row(0);
  A.row(3) = y1 * P1.row(2) - P1.row(1);
  Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
  const Eigen::Vector4d X = svd.matrixV().col(3);
  if (std::fabs(X(3)) < 0.00000001)
  {
    return false;
  }
  *p_c0 = X.head(3) / X(3);
  if (p_c0->z() <= 0.0)
  {
    return false;
  }
  const Eigen::Matrix3d R_c1_c0 = T_c1_c0.getRotationMatrix();
  if (R_c1_c0.row(2).dot(*p_c0) + P1(2, 3) <= 0.0)
  {
    return false;
  }
  Eigen::Vector2d px0;
  if (p_c0->z() < 0.0)
  {
    return false;
  }
  if (!cam0->project3(*p_c0, &px0, nullptr).isKeypointVisible())   // status_ != 0
  {
    return false;
  }
  const Eigen::Vector3d p_c1 = T_c1_c0.getRotationMatrix() * (*p_c0) + P1.col(3);
  Eigen::Vector2d px1;
  if (p_c1.z() < 0.0)
  {
    return true;
  }
  if (!cam1->project3(p_c1, &px1, nullptr).isKeypointVisible())
  {
    return false;
  }
  return true;
}

// ===============================================================================================
// 0x180196b60  updateSVOPointsDescriptors -- upstream-modified: takes the KeyFramePtr (not an index
// into kf_list_), null-checked, reduced extractFeaturesFromSVOKeypoints signature.
// ===============================================================================================
void LoopClosing::updateSVOPointsDescriptors(const KeyFramePtr& kf, const bool replace_mixed_features)
{
  if (!kf)
  {
    return;
  }
  extractFeaturesFromSVOKeypoints(kf->keyframe_image_, &kf->svo_landmarksvector_cam_,
                                  &kf->svo_landmark_ids_, &kf->svo_trackIDsvector_,
                                  &kf->svo_keypointsvector_, &kf->svo_features_,
                                  &kf->svo_features_mat_);
  kf->svo_node_ids_.clear();
  getNodeID(kf->svo_features_, voc_, 3, &kf->svo_node_ids_);

  if (replace_mixed_features)
  {
    kf->mixed_keypoints_.clear();
    kf->mixed_features_.clear();
    kf->mixed_node_ids_.clear();
  }
  kf->mixed_keypoints_.insert(kf->mixed_keypoints_.end(), kf->bow_keypoints_.begin(),
                              kf->bow_keypoints_.end());
  kf->mixed_keypoints_.insert(kf->mixed_keypoints_.end(), kf->svo_keypointsvector_.begin(),
                              kf->svo_keypointsvector_.end());
  kf->mixed_features_.insert(kf->mixed_features_.end(), kf->bow_features_.begin(),
                             kf->bow_features_.end());
  kf->mixed_features_.insert(kf->mixed_features_.end(), kf->svo_features_.begin(),
                             kf->svo_features_.end());
  kf->mixed_node_ids_.insert(kf->mixed_node_ids_.end(), kf->bow_node_ids_.begin(),
                             kf->bow_node_ids_.end());
  kf->mixed_node_ids_.insert(kf->mixed_node_ids_.end(), kf->svo_node_ids_.begin(),
                             kf->svo_node_ids_.end());
}

}  // namespace totem
}  // namespace pimax
