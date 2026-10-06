// pimax_slam.pi.dll -- src/frontend/map.cpp  (draft c11, phase B3)
//
// pimax::totem::Map -- fork of svo/src/map.cpp.
// Original file: E:\code_codex\pimax_slam\beta111_5a7902_dll\src\frontend\map.cpp
// Object TU33, code 0x18012C0B0..0x18012EFE0 (incl. the STL instantiations of
// getClosestNKeyframesWithOverlap).
// glog line numbers seen in the binary: LOG(WARNING) 32 (removeKeyframe), VLOG(100) 87
// (addKeyframe) -- forced with #line.
// The in-object order of the functions looks like decorated-name order (??0, ??1, addKeyframe,
// getClosestN..., getOverlapKeyframes, <0x18012E930>, removeKeyframe, removeOldestKeyframe, reset,
// safeDeletePoint), which suggests that the real name of 0x18012E930 sorts between
// "getOverlapKeyframes" and "removeKeyframe"; "allKeyPointsVisible" (invented by c11) is kept
// because the frame processor calls it.  TODO(verify) name.  (Weak hint only: other objects,
// e.g. stereo_triangulation.obj, do not follow that order.)
#include "frontend/map.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <unordered_set>

#include <glog/logging.h>
#include <vikit/cameras/camera_geometry_base.h>

#include "common/camera.h"
#include "common/logger.h"

namespace pimax {
namespace totem {

namespace {

// Angle between two rotations from the quaternion dot product. Also inlined in the frame
// processor (0x1800B2F80), so it most likely lives in a shared header.
// TODO(verify): real name / home. Neither Eigen 3.3 (d>=1 -> 0) nor Eigen 3.4 (atan2) matches.
inline double quaternionAngle(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
  const double d = std::fabs(q1.coeffs().dot(q2.coeffs()));
  if (std::fabs(1.0 - d) < 1e-6)   // 0x1803AF298
    return 0.0;
  return 2.0 * std::acos(d);       // binary: acos(d) + acos(d)
}

}  // namespace

// 0x18012CE00
// upstream-identical (std::map head node new(0x38), deque proxy new(0x10), mutex init).
Map::Map() : last_added_kf_id_(-1) {}

// 0x18012CEB0
// upstream-modified: SVO_INFO_STREAM -> LOGI; reset() also clears last_removed_kf_.
Map::~Map() {
  reset();
  LOGI("Map destructed\n");
}

// 0x18012ED20 (also inlined into ~Map)
// upstream-modified: additionally resets last_removed_kf_.
void Map::reset() {
  keyframes_.clear();
  sorted_keyframe_ids_.clear();
  last_added_kf_id_ = -1;
  points_to_delete_.clear();
  last_removed_kf_.reset();
}

// 0x18012E9F0
// upstream-modified: features are selected by track_id_vec_(i) > -1 (instead of
// landmark_vec_[i] != nullptr), and the frame's landmark reference and track id are cleared
// (upstream had this commented out).
void Map::removeKeyframe(const int frame_id) {
  auto it_kf = keyframes_.find(frame_id);
  if (it_kf == keyframes_.end()) {
#line 32
    LOG(WARNING) << "Cannot find the keyframe with id " << frame_id   // map.cpp:32
                 << ", will not do anything.";
    return;
  }
  const FramePtr& frame = it_kf->second;
  last_removed_kf_ = frame;
  for (size_t i = 0; i < frame->num_features_; ++i) {
    if (frame->track_id_vec_(i) > -1) {
      frame->landmark_vec_[i]->removeObservation(frame_id);   // 0x18009BBE0
      frame->landmark_vec_[i] = nullptr;
      frame->track_id_vec_(i) = -1;
    }
  }
  keyframes_.erase(it_kf);
}

// 0x18012EC70
// upstream-identical.
void Map::removeOldestKeyframe() {
  removeKeyframe(sorted_keyframe_ids_.front());
  sorted_keyframe_ids_.pop_front();
}

// 0x18012EDE0
// upstream-modified: null-pointer guard; obs_ is an unordered_map<int, KeypointIdentifier>;
// SVO_ERROR_STREAM -> LOGE; the caller's pointer is reset at the end.
void Map::safeDeletePoint(PointPtr& pt) {
  if (pt) {
    // Delete references to mappoints in all keyframes
    for (auto& obs : pt->obs_) {
      if (const FramePtr& frame = obs.second.frame.lock()) {
        frame->deleteLandmark(obs.second.keypoint_index_);   // 0x180094460
      } else {
        LOGE("could not lock weak_ptr<Frame> in Map::safeDeletePoint\n");
      }
    }
    pt->obs_.clear();
    pt.reset();
  }
}

// 0x18012D680
// upstream-modified: no temporal_map argument, id always appended to sorted_keyframe_ids_.
void Map::addKeyframe(const FramePtr& new_keyframe) {
#line 87
  VLOG(100) << "Adding keyframe to map. Frame-Id = " << new_keyframe->id();   // map.cpp:87

  keyframes_.insert(std::make_pair(new_keyframe->id(), new_keyframe));
  last_added_kf_id_ = new_keyframe->id();
  sorted_keyframe_ids_.push_back(new_keyframe->id());
}

// 0x18012E1B0
// upstream-modified:
//  * keyframes are visited newest-id first (reverse iteration of the ordered map);
//  * key points are Vector3f (converted to double for isVisible);
//  * the ids of keyframes added by the visibility test are remembered; if at most 20 were found,
//    a second pass adds every other keyframe that is within 0.5 (position) and 1.0 rad
//    (rotation) of `frame` (pimax-new).
void Map::getOverlapKeyframes(const FramePtr& frame,
                              std::vector<std::pair<FramePtr, double>>* close_kfs) const {
  const Eigen::Vector3d cur_pos = frame->T_f_w_.getPosition();
  const Eigen::Quaterniond cur_q = frame->T_f_w_.getRotation().toImplementation();
  std::unordered_set<int> added_kf_ids;

  for (auto kf = keyframes_.rbegin(); kf != keyframes_.rend(); ++kf) {
    const double dist = (cur_pos - kf->second->T_f_w_.getPosition()).norm();
    // check first if Point is visible in the Keyframe, use therefore KeyPoints
    for (const auto& keypoint : kf->second->key_pts_) {
      if (keypoint.first == -1)
        continue;

      if (frame->isVisible(keypoint.second.cast<double>())) {
        close_kfs->push_back(std::make_pair(kf->second, dist));
        added_kf_ids.insert(kf->first);
        break;  // this keyframe has an overlapping field of view -> add to close_kfs
      }
    }
  }

  if (added_kf_ids.size() <= 20) {
    for (auto kf = keyframes_.rbegin(); kf != keyframes_.rend(); ++kf) {
      if (added_kf_ids.find(kf->first) != added_kf_ids.end())
        continue;
      const Eigen::Vector3d kf_pos = kf->second->T_f_w_.getPosition();
      const double dist = (cur_pos - kf_pos).norm();
      const double angle =
          quaternionAngle(cur_q, kf->second->T_f_w_.getRotation().toImplementation());
      if (std::fabs(dist) < 0.5 && std::fabs(angle) < 1.0) {
        close_kfs->push_back(std::make_pair(kf->second, dist));
      }
    }
  }
}

// 0x18012D900
// upstream-modified (pimax ranking):
//  * CHECK_NOTNULL removed;
//  * every overlapping keyframe is re-scored: the principal point (cx, cy; stored as float) of the
//    keyframe camera is back-projected, pushed 50 units along the ray, transformed into the
//    current frame and projected with the current camera (the ProjectionResult is ignored);
//    keyframes whose point is behind the camera (z < 0) or whose back-projection fails are dropped;
//    score = 100 * distance + |projected - principal point| (pixels);
//  * nth_element puts keyframes with the current frame's camera index first (Frame +0x14 =
//    cam_index_; c11 called it bundle_id_, see integration_A1.md), then by score.
void Map::getClosestNKeyframesWithOverlap(const FramePtr& cur_frame, const size_t& num_frames,
                                          std::vector<FramePtr>* close_kfs) const {
  std::vector<std::pair<FramePtr, double>> overlap_kfs;
  getOverlapKeyframes(cur_frame, &overlap_kfs);
  if (overlap_kfs.empty())
    return;

  std::vector<std::pair<FramePtr, double>> scored_kfs;
  for (const auto& kf : overlap_kfs) {
    const float cx = kf.first->cam_->getIntrinsicParameters()(2);
    const float cy = kf.first->cam_->getIntrinsicParameters()(3);
    const Eigen::Vector2d px_center(cx, cy);
    Eigen::Vector3d f;
    if (!kf.first->cam_->backProject3(px_center, &f))
      continue;

    // T_f_w_.inverseTransform(50 f) with an explicitly renormalised inverse rotation:
    // q_inv = q.inverse() (Eigen: conj/|q|^2, zero on |q|^2 <= 0), then normalized(), then
    // rotate (50 f - t). TODO(verify): source spelling (maybe a Pimax kindr inverseRotate).
    const Eigen::Quaterniond q_world_kf =
        kf.first->T_f_w_.getRotation().toImplementation().inverse().normalized();
    const Eigen::Vector3d p_world = q_world_kf * (f * 50.0 - kf.first->T_f_w_.getPosition());
    const Eigen::Vector3d p_cur = cur_frame->T_f_w_ * p_world;
    if (p_cur.z() >= 0.0) {
      Eigen::Vector2d px;
      cur_frame->cam_->project3(p_cur, &px, nullptr);
      const double score = kf.second * 100.0 + (px - px_center).norm();
      scored_kfs.emplace_back(kf.first, score);
    }
  }

  size_t N = std::min(num_frames, scored_kfs.size());
  // Extract closest N frames.
  std::nth_element(scored_kfs.begin(), scored_kfs.begin() + N, scored_kfs.end(),
                   [cur_frame](const std::pair<FramePtr, double>& lhs,
                               const std::pair<FramePtr, double>& rhs) {
                     if (lhs.first->cam_index_ == cur_frame->cam_index_) {
                       if (rhs.first->cam_index_ != cur_frame->cam_index_)
                         return true;
                     } else if (rhs.first->cam_index_ == cur_frame->cam_index_) {
                       return false;
                     }
                     return lhs.second < rhs.second;
                   });
  scored_kfs.resize(N);

  close_kfs->reserve(num_frames);
  std::transform(scored_kfs.begin(), scored_kfs.end(), std::back_inserter(*close_kfs),
                 [](const std::pair<FramePtr, double>& p) { return p.first; });
}

// 0x18012E930
// pimax-new. Counts the valid key points (first != -1) of `kf` that are visible in `frame`.
bool Map::allKeyPointsVisible(const FramePtr& frame, const FramePtr& kf) const {
  int n_visible = 0;
  for (const auto& keypoint : kf->key_pts_) {
    if (keypoint.first != -1) {
      if (frame->isVisible(keypoint.second.cast<double>()))
        ++n_visible;
    }
  }
  return n_visible == 5;
}

}  // namespace totem
}  // namespace pimax
