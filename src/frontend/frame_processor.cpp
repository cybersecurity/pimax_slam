// pimax_slam.pi.dll -- src/frontend/frame_processor.cpp  (phase B2; draft c07)
//
// Pimax fork of svo/src/frame_handler_stereo.cpp (svo::FrameHandlerStereo).
// Object [0x1800B1480, 0x1800B57D0).  MSVC emitted the COMDATs of this object sorted by
// decorated name; the project functions are:
//   0x1800B2460 ??0FrameProcessor            0x1800B29B0 ??_GFrameProcessor (scalar deleting dtor,
//                                                         implicit dtor body)
//   0x1800B2F80 ?makeKeyframe                0x1800B4180 ?processFirstFrame
//   0x1800B4290 ?processFrame                0x1800B4970 ?processFrameBundle
//   0x1800B49A0 ?re...  (removeOutliersByFundamentalMat, name is ours; sorts between
//               processFrameBundle and resetAll)
//   0x1800B5760 ?resetAll
// Everything else in the object is library template code (see notes/c07_direct.md).
//
// The original object also includes pangolin/handler/handler.h (two atexit-only statics, c00
// TU29) and defines an unreferenced, constant-initialised static std::string (0x18046A170);
// neither is reproduced (no Pangolin in this build).  TODO(verify)
//
// Logging: LOGD 0x18000C120, LOGI 0x18000F500 (logger at 0x18046A000).
//
// Integration (B2) renames applied to the c07 draft (see notes/integration_B2.md):
//   counter_632_ -> low_q_num_, low_match_ratio_frames_ -> low_m_r_num_, input_value_ ->
//   frame_flag_, map_mode_ (+3424) -> bundle_adjustment_type_ (== kCeres), map_mode_flag_ (+3456)
//   -> imu_not_initialized_, prior_map_ (+3408) -> bundle_adjustment_ (its active_keyframes_
//   deque at +120), img_mean_ -> mean_intensity_, is_stationary_ -> is_static_,
//   Map::checkOverlap -> Map::allKeyPointsVisible, Map::keyframe_ids_ -> sorted_keyframe_ids_,
//   frame_utils::computeSceneDepth -> frame_utils::getSceneDepth, DepthFilter::getKeyframes ->
//   GetFramesWithoutSeeds, optimizeStructureInPriorMap -> optimizeStructureConsecutive,
//   ReprojectResult fields (n_points -> n_trials, stat24 -> n_seed_matches, stat32 ->
//   n_lm_matches, stat48_ratio -> ave_lm_obs).

#include "frontend/frame_processor.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <opencv2/calib3d.hpp>

#include "ceres_backend/ceres_backend_interface.hpp"
#include "common/frame.h"
#include "common/logger.h"
#include "common/point.h"
#include "direct/depth_filter.h"
#include "direct/feature_detection.h"
#include "direct/feature_detection_utils.h"
#include "frontend/initialization.h"
#include "frontend/map.h"
#include "frontend/pose_optimizer.h"
#include "frontend/reprojector.h"
#include "frontend/stereo_triangulation.h"

namespace pimax {
namespace totem {

namespace {

// Inlined twice in makeKeyframe (0x1800B3041.. and 0x1800B3DD0..): the quaternion angle
// 2*acos(|q1.q2|) (0 if |1-|q1.q2|| < 1e-6).  The binary applies the abs mask twice (andps x2)
// -> fabs(fabs(dot)).  TODO(verify) whether this is a free helper in a Pimax header (no
// out-of-line copy exists).
inline double quaternionAngle(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2)
{
  const double d = std::fabs(std::fabs(q1.dot(q2)));
  if (std::fabs(1.0 - d) < 1e-6)
    return 0.0;
  return 2.0 * std::acos(d);
}

} // namespace

// 0x1800B2460
// upstream-modified: two extra trailing ctor arguments forwarded to FrameProcessorBase;
// stereo_triangulation_ is assigned from a unique_ptr (control block is
// _Ref_count_resource<StereoTriangulation*, default_delete<>> at 0x1803B2BC8).
FrameProcessor::FrameProcessor(
    const BaseOptions& base_options,
    const DepthFilterOptions& depth_filter_options,
    const DetectorOptions& feature_detector_options,
    const InitializationOptions& init_options,
    const StereoTriangulationOptions& stereo_options,
    const ReprojectorOptions& reprojector_options,
    const FeatureTrackerOptions& tracker_options,
    const CameraBundlePtr& stereo_camera,
    const std::string& device_sn,
    const uint8_t& loc_mode)
  : FrameProcessorBase(
        base_options, reprojector_options, depth_filter_options,
        feature_detector_options, init_options, tracker_options, stereo_camera,
        device_sn, loc_mode)
{
  // init initializer
  stereo_triangulation_ = std::unique_ptr<StereoTriangulation>(
      new StereoTriangulation(
          stereo_options,
          feature_detection_utils::makeDetector(
              feature_detector_options, cams_->getCameraShared(0))));
}

// 0x1800B4970
// upstream-identical
UpdateResult FrameProcessor::processFrameBundle()
{
  UpdateResult res = UpdateResult::kFailure;
  if (stage_ == Stage::kTracking)
    res = processFrame();
  else if (stage_ == Stage::kInitializing)
    res = processFirstFrame();
  return res;
}

// 0x1800B4180
// upstream-modified:
//  * every frame of the bundle is made a keyframe (upstream: at(0) and at(1)); Map::addKeyframe has
//    no "ceres" flag; per frame the Pimax scene-depth helper 0x180094890 is called (it stores
//    min/median depth into Frame+0x1A8/+0x1AC) instead of getSceneDepth on frame 0.
//  * depth_filter_->addKeyframe(new_frames_, *map_) takes the whole bundle and the map.
//  * on failure stage_ is (re)set to kInitializing and kFailure is returned (upstream: kDefault +
//    SVO_ERROR_STREAM).
//  * last_kf_frames_ = new_frames_.
UpdateResult FrameProcessor::processFirstFrame()
{
  if (initializer_->addFrameBundle(new_frames_) == InitResult::kFailure)
  {
    stage_ = Stage::kInitializing;
    return UpdateResult::kFailure;
  }

  low_q_num_ = 0;                                      // base +632
  for (const FramePtr& frame : new_frames_->frames_)
  {
    frame->setKeyframe();
    map_->addKeyframe(frame);
    frame_utils::getSceneDepth(frame);                 // 0x180094890, bool result ignored
  }
  new_frames_->is_keyframe_ = true;                    // FrameBundle+248
  depth_filter_->addKeyframe(new_frames_, *map_);

  LOGI("Init: Selected first frame.\n");
  stage_ = Stage::kTracking;
  tracking_quality_ = TrackingQuality::kGood;
  last_kf_frames_ = new_frames_;
  return UpdateResult::kKeyframe;
}

// 0x1800B4290
// upstream-modified (heavily): no sparse image alignment here; projectMapInFrame returns a
// statistics struct; Pimax fundamental-matrix outlier rejection; IMU-only pose propagation while
// the bundle is flagged stationary; custom keyframe selection (match ratio / landmark ratio /
// candidate counter / bundle-id distance / 0.4 s spacing / small map).  With the Ceres backend
// and an initialised IMU (bundle_adjustment_type_ == kCeres && !imu_not_initialized_) the pose
// optimisation result is ignored (the backend owns the pose), structure is optimised with the
// consecutive-observation variant, and there is no early keyframe exit.
UpdateResult FrameProcessor::processFrame()
{
  // ---------------------------------------------------------------------------
  // tracking
  const ReprojectResult res = projectMapInFrame();     // 0x180119260
  const size_t n_matches = res.n_matches;
  const size_t n_points = res.n_trials;
  const float match_ratio = static_cast<float>(n_matches) / static_cast<float>(n_points);
  const float lm_match_ratio = static_cast<float>(res.n_lm_matches) / static_cast<float>(n_matches);
  size_t n_tracked_features = n_matches + res.n_seed_matches;

  // QUIRK: size_t / int values passed for %d.
  LOGD("# tracking_result: %d, %d, %f, %f, %f, %d, %d\n",
       n_matches, n_points, match_ratio, res.ave_lm_obs, lm_match_ratio,
       map_->size(), new_frames_->frames_[0]->id());
  LOGD("ave_success_num %f\n", res.ave_success_num);

  if (match_ratio < 0.1)
    ++low_m_r_num_;                                    // base +636
  else
    low_m_r_num_ = 0;

  new_frames_->num_tracked_ = n_tracked_features;      // FrameBundle+232
  if (n_tracked_features < options_.quality_min_fts && !new_frames_->is_static_)
  {
    new_frames_->low_feature_kf_ = true;               // FrameBundle+229
    LOGI("feature less to quality_min_fts, force stereo triangulation to recover\n");
    optimizeStructure(new_frames_, options_.structure_optimization_max_pts, 5);   // 0x180118D80
    return makeKeyframe(frame_flag_);
  }

  removeOutliersByFundamentalMat();

  if (bundle_adjustment_type_ == BundleAdjustmentType::kCeres && !imu_not_initialized_)   // +3424 / +3456
  {
    if (!new_frames_->is_static_)
    {
      if (new_frames_->numLandmarks() > 10)
        optimizePose(false);                           // 0x180118740, result ignored
      optimizeStructureConsecutive(new_frames_, 10, 5);   // 0x1801188C0
    }
  }
  else
  {
    if (new_frames_->is_static_)
    {
      // keep the IMU pose of the previous bundle
      const Transformation T_world_imu = last_frames_->at(0)->T_world_imu();
      for (int i = 0; i < new_frames_->size(); ++i)   // int i, unsigned 64-bit compare in the binary
      {
        const FramePtr& frame = new_frames_->at(i);
        frame->T_f_w_ = (T_world_imu * frame->T_imu_cam()).inverse();
        frame->T_f_w_.getRotation().normalize();       // TODO(verify): explicit or inside set_T_w_imu
      }
    }
    else if (new_frames_->numLandmarks() > 10)
    {
      n_tracked_features = optimizePose(true);
    }

    if (n_tracked_features < options_.quality_min_fts
        || (new_frames_->frames_[0]->getTimestampNSec()
            - last_kf_frames_->frames_[0]->getTimestampNSec()) / 1000000000.0 > 0.25)
    {
      return makeKeyframe(frame_flag_);
    }
    optimizeStructure(new_frames_, options_.structure_optimization_max_pts, 5);
  }

  setTrackingQuality(n_tracked_features);
  if (tracking_quality_ == TrackingQuality::kInsufficient)
    return makeKeyframe(frame_flag_);

  // ---------------------------------------------------------------------------
  // select keyframe
  const float min_match_ratio = (n_matches > 300) ? 0.65f : 0.72f;
  if (n_points < 150)
    ++low_match_count_;
  bool force_kf = false;
  if (low_match_count_ > 3)
  {
    ++forced_kf_count_;
    force_kf = true;
  }
  const FramePtr& last_kf = last_kf_frames_->frames_[0];
  const FramePtr& cur = new_frames_->frames_[0];
  const int max_bundle_gap = last_kf->bundleId() + 33;
  const bool spacing_ok =
      (cur->getTimestampNSec() - last_kf->getTimestampNSec()) / 1000000000.0 > 0.4f
      && !new_frames_->is_static_;
  if (((min_match_ratio > match_ratio || res.ave_lm_obs < 3.5f || lm_match_ratio < 0.8f
        || force_kf || cur->bundleId() >= max_bundle_gap) && spacing_ok)
      || map_->size() < 8)
  {
    LOGD("New keyframe selected.");
    low_match_count_ = 0;
    return makeKeyframe(frame_flag_);
  }

  for (size_t i = 0; i < new_frames_->size(); ++i)
    depth_filter_->updateSeeds(overlap_kfs_.at(i), new_frames_->at(i));
  return UpdateResult::kDefault;
}

// 0x1800B2F80
// upstream-modified (rewritten).  Returns kKeyframe always.
//  1. a bundle whose pose is within 5 cm / 0.3 rad of the last keyframe bundle and whose key
//     points are all visible in it (Map 0x18012E930 on cam 0 or 1) marks all its frames
//     "redundant" (Frame+0xEA); 3 consecutive redundant keyframes, >= 200 landmarks or a
//     stationary bundle skip triangulation.
//  2. otherwise stereo triangulation on (0,1) [+ (2,3), and (0,2)/(1,3) when < 150 landmarks for
//     4-camera rigs], plus temporal triangulation against the last keyframe bundle when
//     input_value < 1 and < 100 landmarks; images with mean intensity (Frame+0xE0) <= 15 are
//     skipped.
//  3. depth filter: addKeyframe(bundle) + updateSeeds per camera; keyframes the depth filter
//     reports since the last call are offered to FrameProcessorBase 0x1800F4BD0 and removed from
//     the map when it returns true.
//  4. map size limit: with the Ceres backend (bundle_adjustment_type_ == kCeres) redundant
//     keyframes that the backend's active keyframes (+120) do not reference are removed first,
//     else Map::removeOldestKeyframe.
UpdateResult FrameProcessor::makeKeyframe(int input_value)
{
  if (last_kf_frames_)
  {
    const FramePtr& cur0 = new_frames_->frames_[0];
    const FramePtr& kf0 = last_kf_frames_->frames_[0];
    const double dist = (cur0->T_f_w_.getPosition() - kf0->T_f_w_.getPosition()).norm();
    const double angle = quaternionAngle(kf0->T_f_w_.getRotation().toImplementation(),
                                         cur0->T_f_w_.getRotation().toImplementation());
    if ((map_->allKeyPointsVisible(new_frames_->frames_[0], last_kf_frames_->frames_[0])
         || map_->allKeyPointsVisible(new_frames_->frames_[1], last_kf_frames_->frames_[1]))
        && dist < 0.05 && angle < 0.3)
    {
      for (const FramePtr& frame : new_frames_->frames_)
        frame->is_redundant_kf_ = true;              // Frame+234
    }
  }

  if (new_frames_->at(0)->is_redundant_kf_)
    ++redundant_kf_count_;                           // base +3116
  else
    redundant_kf_count_ = 0;

  if (new_frames_->numLandmarks() >= 200
      || new_frames_->is_static_
      || redundant_kf_count_ >= 3)
  {
    for (const FramePtr& frame : new_frames_->frames_)
    {
      frame->setKeyframe();
      map_->addKeyframe(frame);
      upgradeSeedsToFeatures(frame);                 // 0x180128FA0
      frame_utils::getSceneDepth(frame);             // 0x180094890
    }
  }
  else
  {
    // ---------------------------------------------------------------------------
    // stereo triangulation
    if (new_frames_->at(0)->mean_intensity_ > 15.0 && new_frames_->at(1)->mean_intensity_ > 15.0)
      stereo_triangulation_->compute(new_frames_->at(0), new_frames_->at(1), false);
    if (new_frames_->size() == 4)
    {
      if (new_frames_->at(2)->mean_intensity_ > 15.0 && new_frames_->at(3)->mean_intensity_ > 15.0)
        stereo_triangulation_->compute(new_frames_->at(2), new_frames_->at(3), false);
      if (new_frames_->numLandmarks() < 150)
      {
        if (new_frames_->at(0)->mean_intensity_ > 15.0 && new_frames_->at(2)->mean_intensity_ > 15.0)
          stereo_triangulation_->compute(new_frames_->at(0), new_frames_->at(2), false);
        if (new_frames_->at(1)->mean_intensity_ > 15.0 && new_frames_->at(3)->mean_intensity_ > 15.0)
          stereo_triangulation_->compute(new_frames_->at(1), new_frames_->at(3), false);
      }
    }

    // temporal triangulation against the last keyframe bundle
    if (input_value < 1)
    {
      double dist = 0.0;
      if (new_frames_->numLandmarks() < 100)
      {
        // NOTE: last_kf_frames_ is dereferenced before the null check below (quirk).
        const Eigen::Vector3d t_kf = last_kf_frames_->at(0)->T_f_w_.getPosition();
        const Eigen::Vector3d t_cur = new_frames_->at(0)->T_f_w_.getPosition();
        dist = (t_cur - t_kf).norm();
        if (last_kf_frames_)
        {
          if (new_frames_->at(0)->mean_intensity_ > 15.0 && dist > 0.01)
            stereo_triangulation_->compute(last_kf_frames_->at(0), new_frames_->at(0), false);
          if (last_kf_frames_ && new_frames_->at(1)->mean_intensity_ > 15.0 && dist > 0.01)
            stereo_triangulation_->compute(last_kf_frames_->at(1), new_frames_->at(1), false);
        }
      }
      // QUIRK: at(2)/at(3) without a size() == 4 check (throws on 2-camera rigs).
      if (new_frames_->numLandmarks() < 100 && last_kf_frames_)
      {
        if (new_frames_->at(2)->mean_intensity_ > 15.0 && dist > 0.01)
          stereo_triangulation_->compute(last_kf_frames_->at(2), new_frames_->at(2), false);
        if (last_kf_frames_ && new_frames_->at(3)->mean_intensity_ > 15.0 && dist > 0.01)
          stereo_triangulation_->compute(last_kf_frames_->at(3), new_frames_->at(3), false);
      }
    }

    optimizeStructure(new_frames_, options_.structure_optimization_max_pts, 5);

    // ---------------------------------------------------------------------------
    // new keyframe selected
    for (size_t i = 0; i < new_frames_->size(); ++i)
    {
      new_frames_->at(i)->setKeyframe();
      map_->addKeyframe(new_frames_->at(i));
      upgradeSeedsToFeatures(new_frames_->at(i));
    }
    for (size_t i = 0; i < new_frames_->size(); ++i)
      frame_utils::getSceneDepth(new_frames_->at(i));
  }

  new_frames_->is_keyframe_ = true;
  depth_filter_->addKeyframe(new_frames_, *map_);
  for (size_t i = 0; i < new_frames_->size(); ++i)
    depth_filter_->updateSeeds(overlap_kfs_.at(i), new_frames_->at(i));

  // keyframes the depth filter found without seeds since the last call
  std::vector<FramePtr> df_keyframes;
  std::vector<FramePtr> new_df_keyframes;
  const bool have_df_keyframes = depth_filter_->GetFramesWithoutSeeds(df_keyframes);   // 0x18009FC10
  if (!last_df_keyframes_.empty() && have_df_keyframes)
  {
    const int last_id = last_df_keyframes_.back()->id();
    for (int k = static_cast<int>(df_keyframes.size()) - 1; k >= 0; --k)
    {
      if (last_id == df_keyframes[k]->id())
        break;
      new_df_keyframes.push_back(df_keyframes[k]);
    }
  }
  if (!df_keyframes.empty())
    last_df_keyframes_ = df_keyframes;
  for (const FramePtr& kf : new_df_keyframes)
  {
    if (shouldRemoveKeyframe(kf))                    // FrameProcessorBase 0x1800F4BD0
    {
      map_->removeKeyframe(kf->id());
      auto& ids = map_->sorted_keyframe_ids_;        // std::deque<int> at Map+128
      auto it = std::find(ids.begin(), ids.end(), kf->id());
      if (it != ids.end())
        ids.erase(it);
    }
  }

  // ---------------------------------------------------------------------------
  // if limited number of keyframes, remove one
  while (map_->size() > options_.max_n_kfs)
  {
    if (bundle_adjustment_type_ != BundleAdjustmentType::kCeres)
    {
      map_->removeOldestKeyframe();
      continue;
    }

    std::shared_ptr<CeresBackendInterface> backend = bundle_adjustment_;   // base +3408
    std::set<int> backend_kf_ids;
    for (const FramePtr& frame : backend->active_keyframes_)   // std::deque<FramePtr> at +120
      backend_kf_ids.insert(frame->id());

    std::vector<int> redundant_ids;
    std::unordered_map<int, FramePtr> candidates;
    for (const auto& kv : map_->keyframes_)        // std::map<int, FramePtr> at Map+0
    {
      if (kv.second->is_redundant_kf_ && backend_kf_ids.find(kv.first) == backend_kf_ids.end())
        redundant_ids.push_back(kv.first);
      else if (backend_kf_ids.find(kv.first) == backend_kf_ids.end())
        candidates.insert(kv);
    }

    for (const auto& kv : candidates)
    {
      for (size_t i = 0; i < new_frames_->size(); ++i)
      {
        const FramePtr& kf = kv.second;
        const FramePtr& frame = new_frames_->frames_[i];
        const double dist = (frame->T_f_w_.getPosition() - kf->T_f_w_.getPosition()).norm();
        const double angle = quaternionAngle(frame->T_f_w_.getRotation().toImplementation(),
                                             kf->T_f_w_.getRotation().toImplementation());
        if (dist < 0.05 && angle < 0.3)
          kf->is_redundant_kf_ = true;
      }
    }

    for (size_t i = 0; i < new_frames_->size(); ++i)
    {
      if (redundant_ids.size() > new_frames_->size())
      {
        std::sort(redundant_ids.begin(), redundant_ids.end());
        map_->removeKeyframe(redundant_ids[0]);
        auto& ids = map_->sorted_keyframe_ids_;
        auto it = std::find(ids.begin(), ids.end(), redundant_ids[0]);
        if (it != ids.end())
        {
          ids.erase(it);
          redundant_ids.erase(redundant_ids.begin());
        }
      }
      else
      {
        map_->removeOldestKeyframe();
      }
    }
  }

  last_kf_frames_ = new_frames_;
  return UpdateResult::kKeyframe;
}

// 0x1800B49A0
// pimax-new: for each camera, match features of new_frames_ and last_frames_ by track id
// (only features with track id != -1 and pyramid level (Frame +0x270) <= 1), run a RANSAC
// fundamental matrix (1.5 px, 0.99) and drop the outliers: track id := -1, observation removed
// from the point if the frame is a keyframe, landmark reset.
void FrameProcessor::removeOutliersByFundamentalMat()
{
  if (!last_frames_)
    return;

  for (size_t i = 0; i < new_frames_->size(); ++i)
  {
    FramePtr cur_frame = new_frames_->at(i);
    FramePtr last_frame = last_frames_->at(i);
    if (cur_frame->num_features_ < 15 || last_frame->num_features_ < 15)
      continue;

    std::vector<size_t> cur_idx;
    std::vector<size_t> last_idx;
    std::vector<cv::Point2f> cur_pts;
    std::vector<cv::Point2f> last_pts;
    std::unordered_map<int, std::vector<size_t>> track_to_last_idx;

    for (size_t j = 0; j < last_frame->num_features_; ++j)
    {
      if (last_frame->track_id_vec_(j) != -1 && last_frame->level_vec_(j) <= 1)
        track_to_last_idx[last_frame->track_id_vec_(j)].push_back(j);
    }

    for (size_t j = 0; j < cur_frame->num_features_; ++j)
    {
      if (cur_frame->track_id_vec_(j) != -1 && cur_frame->level_vec_(j) <= 1)
      {
        const int track_id = cur_frame->track_id_vec_(j);
        if (track_to_last_idx.find(track_id) != track_to_last_idx.end())
        {
          const std::vector<size_t>& idx = track_to_last_idx[track_id];
          if (!idx.empty())
          {
            const size_t k = idx[0];
            cur_idx.push_back(j);
            last_idx.push_back(k);
            cur_pts.emplace_back(cur_frame->px_vec_(0, j), cur_frame->px_vec_(1, j));
            last_pts.emplace_back(last_frame->px_vec_(0, k), last_frame->px_vec_(1, k));
          }
        }
      }
    }

    if (cur_pts.size() >= 15)
    {
      std::vector<uchar> status(cur_pts.size());
      cv::findFundamentalMat(cur_pts, last_pts, status, cv::FM_RANSAC, 1.5, 0.99);
      for (size_t k = 0; k < cur_pts.size(); ++k)
      {
        if (!status[k])
        {
          cur_frame->track_id_vec_(cur_idx[k]) = -1;
          // QUIRK: the landmark may be null here.
          if (cur_frame->is_keyframe_)
            cur_frame->landmark_vec_[cur_idx[k]]->removeObservation(cur_frame->id());   // 0x18009BBE0
          cur_frame->landmark_vec_[cur_idx[k]].reset();
        }
      }
    }
  }
}

// 0x1800B5760
// upstream-modified: logs, resets the redundant-keyframe counter (+3116) and the depth-filter
// keyframe list (upstream set backend_scale_initialized_ = true).
void FrameProcessor::resetAll()
{
  LOGI("resetAll\n");
  redundant_kf_count_ = 0;
  last_df_keyframes_.clear();
  resetVisionFrontendCommon();                       // 0x18011B420
}

} // namespace totem
} // namespace pimax
