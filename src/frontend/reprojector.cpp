// pimax_slam.pi.dll -- src/frontend/reprojector.cpp  (draft c12, phase B3)
//
// pimax::totem::Reprojector + reprojector_utils -- fork of svo/src/reprojector.cpp, heavily
// rewritten by Pimax (candidate validity flag + 16 px grid NMS, depth/height gating, ORB-style
// rotation-consistency check with IC_Angle, extended statistics, no occupancy grid).
// Original file: E:\code_codex\pimax_slam\beta111_5a7902_dll\src\frontend\reprojector.cpp
// Object TU35, code 0x18013FA30..0x180144340.  glog line: VLOG(10) 121 in reprojectFrames
// (forced with #line).  Frame::getSeedDepth (COMDAT 0x1801420C0 emitted in this object) is the
// inline member in common/frame.h.
// Depth gates use a hand-written "z of R(q)*p + t" with DOUBLE literals 1.0/2.0 (mixed precision
// in the float instance) - see depthInFrameT() below (same value as the float
// pimax::totem::depthInFrame of ceres_backend/outlier_rejection.hpp; the operand orders differ
// only by commutation of two-term sums).
#include "frontend/reprojector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include <glog/logging.h>
#include <opencv2/core.hpp>

#include "common/camera.h"
#include "common/logger.h"
#include "common/seed.h"
#include "direct/depth_filter.h"
#include "direct/matcher.h"

namespace pimax {
namespace totem {
namespace {
// Inlined in getCandidate (T=double) and reprojectFrames (T=float). Exact op order of the binary:
// ((1.0 - 2.0*(x*x + y*y)) * p.z + 2.0*((z*x - w*y)*p.x + (z*y + w*x)*p.y)) + t.z
template<typename T>
inline double depthInFrameT(const kindr::minimal::QuatTransformationTemplate<T>& T_f_w,
                           const Eigen::Matrix<T, 3, 1>& p)
{
  const Eigen::Quaternion<T>& q = T_f_w.getRotation().toImplementation();
  return (1.0 - 2.0*(q.x()*q.x() + q.y()*q.y())) * p.z()
      + 2.0*((q.z()*q.x() - q.w()*q.y())*p.x() + (q.z()*q.y() + q.w()*q.x())*p.y())
      + T_f_w.getPosition().z();
}
} // namespace

// 0x1801412F0  upstream-identical (ctor; 240-byte object, see reprojector.h)
Reprojector::Reprojector(
    const ReprojectorOptions& options,
    size_t camera_index)
  : options_(options)
  , camera_index_(camera_index)
{}

// Pimax reprojectFrames() vs upstream: see notes/c12_poseopt_reprojector.md ("reprojectFrames").
// 0x180143380  upstream-modified (rewritten, see notes/c12_poseopt_reprojector.md)
void Reprojector::reprojectFrames(
    const FramePtr& cur_frame,
    const std::vector<FramePtr>& visible_kfs,
    std::vector<PointPtr>& trash_points,
    const bool& need_imu_init)
{
  // Pimax: nothing to match in a (nearly) black image.
  if(cur_frame->mean_intensity_ < 10.0)
    return;

  const size_t max_total_n_features = need_imu_init ?
        options_.max_map_features_per_frame : options_.max_n_features_per_frame;
  cur_frame->resizeFeatureStorage(max_total_n_features);
  stats_.reset();

  // Reproject all map points of the closest N kfs with overlap.
  candidates_.clear();
  std::unordered_set<int> projected_point_ids;
  for(const FramePtr& ref_frame : visible_kfs)
  {
    if(!ref_frame)
      continue;
    const float ref_imu_height = ref_frame->T_world_imu().getPosition().z();
    // Try to reproject each map point that the other KF observes
    for(size_t i = 0; i < ref_frame->num_features_; ++i)
    {
      if(ref_frame->track_id_vec_(i) == -1
         || ref_frame->type_vec_[i] == FeatureType::kOutlier)
        continue;

      const PointPtr& point = ref_frame->landmark_vec_[i];

      // make sure we project a point only once
      if(projected_point_ids.find(point->id_) != projected_point_ids.end())
        continue;

      // Pimax: once gravity is known, ignore points more than 2 m below the reference IMU.
      if(ref_imu_height - 2.0 <= point->pos_.z() || need_imu_init)
      {
        // first check if the point is valid.
        if((point->n_failed_reproj_ > 30
            && point->n_succeeded_reproj_ < point->n_failed_reproj_)
           || point->obs_.size() < 2)
        {
          trash_points.push_back(point);
        }
        else
        {
          // Pimax: only points in (0, 10] m depth of the reference frame.
          const FloatType depth_in_ref =
              depthInFrameT(ref_frame->T_f_w_.cast<FloatType>(), point->pos_);
          if(depth_in_ref > 0.0 && depth_in_ref <= 10.0)
          {
            Candidate candidate;
            if(reprojector_utils::getCandidate(cur_frame, ref_frame, i, candidate))
            {
              candidates_.push_back(candidate);
              projected_point_ids.insert(point->id_);
            }
          }
        }
      }
    }
  }

  // Pimax: converged seeds are always added to the same candidate list.
  for(const FramePtr& ref_frame : visible_kfs)
  {
    for(size_t i = 0; i < ref_frame->num_features_; ++i)
    {
      if(isConvergedCornerEdgeletSeed(ref_frame->type_vec_[i]))
      {
        Candidate candidate;
        if(reprojector_utils::getCandidate(cur_frame, ref_frame, i, candidate))
          candidates_.push_back(candidate);
      }
    }
  }

  Statistics lm_stats;
#line 121
  VLOG(10) << "all candidates num: " << candidates_.size() << std::endl;
  std::sort(candidates_.begin(), candidates_.end(),
            [](const Candidate& lhs, const Candidate& rhs)
  {
    if(lhs.type > rhs.type
       || (lhs.type == rhs.type && lhs.n_obs > rhs.n_obs)
       || (lhs.type == rhs.type && lhs.n_obs == rhs.n_obs && rhs.depth > lhs.depth))
      return true;
    return false;
  });
  Candidates filtered_candidates;
  reprojector_utils::filterCandidatesByGrid(candidates_, filtered_candidates);
  reprojector_utils::matchCandidates(
        cur_frame, max_total_n_features,
        options_.affine_est_offset, options_.affine_est_gain,
        filtered_candidates, grid_.get(), lm_stats, options_.seed_sigma2_thresh);
  stats_.add(lm_stats);
}

namespace reprojector_utils {

namespace {
constexpr int HALF_PATCH_SIZE = 15;          // ORB-SLAM
constexpr int HISTO_LENGTH = 30;             // ORB-SLAM rotation histogram
constexpr float kRotFactor = HISTO_LENGTH / 360.0f;   // 0.083333336f (ORB-SLAM3 convention, degrees)

// Inlined into matchCandidates (0x180142A60). Copy of ORBmatcher::ComputeThreeMaxima with every
// comparison inverted: it returns the three LEAST populated bins.
void computeThreeMinima(std::vector<int>* histo, const int L, int& ind1, int& ind2, int& ind3)
{
  int min1 = std::numeric_limits<int>::max();
  int min2 = std::numeric_limits<int>::max();
  int min3 = std::numeric_limits<int>::max();

  for(int i = 0; i < L; i++)
  {
    const int s = histo[i].size();
    if(s < min1)
    {
      min3 = min2;
      min2 = min1;
      min1 = s;
      ind3 = ind2;
      ind2 = ind1;
      ind1 = i;
    }
    else if(s < min2)
    {
      min3 = min2;
      min2 = s;
      ind3 = ind2;
      ind2 = i;
    }
    else if(s < min3)
    {
      min3 = s;
      ind3 = i;
    }
  }

  if(min2 > 0.1f*(float)min1)
  {
    ind2 = -1;
    ind3 = -1;
  }
  else if(min3 > 0.1f*(float)min1)
  {
    ind3 = -1;
  }
}
} // namespace

// 0x1801415B0  Pimax-new (ORB-SLAM ORBextractor.cc IC_Angle, verbatim)
float IC_Angle(const cv::Mat& image, const Keypoint& pt, const std::vector<int>& u_max)
{
  int m_01 = 0, m_10 = 0;

  const uchar* center = &image.at<uchar>(cvRound(pt.y()), cvRound(pt.x()));

  // Treat the center line differently, v=0
  for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
    m_10 += u * center[u];

  // Go line by line in the circular patch
  int step = (int)image.step1();
  for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
  {
    // Proceed over the two lines
    int v_sum = 0;
    int d = u_max[v];
    for (int u = -d; u <= d; ++u)
    {
      int val_plus = center[u + v*step], val_minus = center[u - v*step];
      v_sum += (val_plus - val_minus);
      m_10 += u * (val_plus + val_minus);
    }
    m_01 += v * v_sum;
  }

  return cv::fastAtan2((float)m_01, (float)m_10);
}

// 0x180141B00  Pimax-new (ORB-SLAM ORBextractor ctor umax table; called once from a static init)
std::vector<int> computeUmax()
{
  std::vector<int> umax(HALF_PATCH_SIZE + 1);

  int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * std::sqrt(2.f) / 2 + 1);   // = 11
  int vmin = cvCeil(HALF_PATCH_SIZE * std::sqrt(2.f) / 2);                // = 11
  const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;
  for (v = 0; v <= vmax; ++v)
    umax[v] = cvRound(std::sqrt(hp2 - v * v));

  // Make sure we are symmetric
  for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
  {
    while (umax[v0] == umax[v0 + 1])
      ++v0;
    umax[v] = v0;
    ++v0;
  }
  return umax;
}

// 0x180141C40  upstream-modified
bool getCandidate(
    const FramePtr& cur_frame,
    const FramePtr& ref_frame,
    const size_t& ref_index,
    Reprojector::Candidate& candidate)
{
  Eigen::Vector3d xyz_world = Eigen::Vector3d::Zero();
  int n_reproj = 0;
  const bool has_landmark = ref_frame->track_id_vec_(ref_index) > -1;   // Pimax: by track id
  if(has_landmark)
  {
    const PointPtr& point = ref_frame->landmark_vec_[ref_index];
    xyz_world = point->pos().cast<double>();
    n_reproj =
        point->n_succeeded_reproj_ - point->n_failed_reproj_;
  }
  else
  {
    // Pimax: seeds that failed too often are skipped (score_vec_ is used as a failure score).
    if(ref_frame->score_vec_(ref_index) > 10.0f)
      return false;
    xyz_world = ref_frame->T_world_cam() *
        ref_frame->getSeedPosInFrame(ref_index).cast<double>();
  }

  // Pimax: depth gate in the current frame.
  const double depth = depthInFrameT(cur_frame->T_f_w_, xyz_world);
  if(depth <= 0.0 || depth > 10.0)
    return false;

  // upstream projectPointAndCheckVisibility() reduced to Frame::isVisible (which has the 16 px margin)
  Eigen::Vector2d px;
  if(!cur_frame->isVisible(xyz_world, &px))
  {
    if(!has_landmark)
      ref_frame->score_vec_(ref_index) += 3.0f;
    return false;
  }

  candidate = Reprojector::Candidate(
        ref_frame, ref_index, px.cast<FloatType>(), n_reproj, ref_frame->score_vec_(ref_index),
        ref_frame->type_vec_[ref_index],
        has_landmark ? ref_frame->landmark_vec_[ref_index]->obs_.size() : 0u,
        depth);

  return true;
}

// 0x180142190  Pimax-new
// false if the pixel is masked, or - when either frame is flagged too dark/too bright - if every
// pixel of the 5x5 window around px_cur in the current image is < 10 or > 240.
bool checkPatchExposure(
    const FramePtr& ref_frame,
    const FramePtr& cur_frame,
    const Keypoint& px_cur)
{
  const int x = static_cast<int>(px_cur.x());
  const int y = static_cast<int>(px_cur.y());
  if(!cur_frame->getMask().at<uchar>(y, x))
    return false;
  if(!cur_frame->is_too_dark_ && !cur_frame->is_too_bright_)
    return true;
  if(!ref_frame->is_too_dark_ && !ref_frame->is_too_bright_)
    return true;

  const cv::Mat& img = cur_frame->img();
  const int x_min = std::max(x - 2, 0);
  const int x_max = std::min(x + 2, img.cols - 1);
  const int y_min = std::max(y - 2, 0);
  const int y_max = std::min(y + 2, img.rows - 1);
  const int width = x_max - x_min + 1;
  const int n_total = width * (y_max - y_min + 1);
  int n_bad = 0;
  for(int v = y_min; v <= y_max; ++v)
  {
    const uchar* row = img.ptr<uchar>(v) + x_min;
    for(int u = 0; u < width; ++u)
    {
      if(row[u] < 10 || row[u] > 240)
        ++n_bad;
    }
  }
  return n_bad != n_total;
}

// 0x1801422D0  upstream-modified
bool matchCandidate(
    const FramePtr& frame,
    Reprojector::Candidate& c,
    Matcher& matcher,
    FeatureWrapper& feature,
    float* angle_ref,
    float* angle_cur,
    const std::vector<int>& umax,
    const FloatType seed_sigma2_thresh)
{
  *angle_ref = 0.0f;
  *angle_cur = 0.0f;
  GradientVector grad_ref;

  // direct matching (Pimax: CHECKs replaced by LOGE + return false)
  if(c.ref_frame.get() == nullptr)
  {
    LOGE("c.ref_frame.get() null\n");
    return false;
  }
  if(c.ref_index >= c.ref_frame->num_features_)
  {
    LOGE("c.ref_index >=c.ref_frame->num_features_\n");
    return false;
  }
  int track_id = -1;
  if(c.ref_frame->track_id_vec_(c.ref_index) == -1)
  {
    FeatureWrapper ref_ftr = c.ref_frame->getFeatureWrapper(c.ref_index);
    if(isConvergedCornerEdgeletSeed(c.type))
    {
      const FloatType ref_depth = c.ref_frame->getSeedDepth(c.ref_index);
      Matcher::MatchResult res =
          matcher.findMatchDirect(*c.ref_frame, *frame, ref_ftr, ref_depth,
                                  c.cur_px);
      if(res != Matcher::MatchResult::kSuccess)
      {
        c.ref_frame->score_vec_(c.ref_index) += 1.0f;   // Pimax: seed failure score
        return false;
      }
      if(!checkPatchExposure(c.ref_frame, frame, matcher.px_cur_))
        return false;
    }
    else if(isUnconvergedCornerEdgeletSeed(c.type))
    {
      if(!depth_filter_utils::updateSeed(
           *frame, *c.ref_frame, c.ref_index, matcher, seed_sigma2_thresh,
           false, false))
      {
        return false;
      }
    }
    else
    {
      return false;   // upstream: map-point seeds / CHECK(false) << "Seed type unknown"
    }

    grad_ref = ref_ftr.grad;
    track_id = ref_ftr.track_id;
    feature.seed_ref.keyframe = c.ref_frame;
    feature.seed_ref.seed_id = c.ref_index;
  }
  else
  {
    const PointPtr& point = c.ref_frame->landmark_vec_[c.ref_index];
    FramePtr ref_frame;
    size_t ref_feature_index;
    if(!point->getCloseViewObs(frame->pos().cast<double>(), ref_frame, ref_feature_index))
    {
      return false;
    }
    FeatureWrapper ref_ftr = ref_frame->getFeatureWrapper(ref_feature_index);
    if(ref_ftr.landmark.get() == nullptr)
    {
      LOGE("ref_ftr.landmark.get()\n");
      return false;
    }
    const FloatType ref_depth = (ref_frame->pos() - ref_ftr.landmark->pos()).norm();
    Matcher::MatchResult res = matcher.findMatchDirect(
          *ref_frame, *frame, ref_ftr, ref_depth, c.cur_px);
    if(res != Matcher::MatchResult::kSuccess)
    {
      point->n_failed_reproj_++;
      return false; // TODO(cfo): We should return match result and write in statistics.
    }
    if(!checkPatchExposure(ref_frame, frame, matcher.px_cur_))
      return false;

    // Pimax: orientation of reference and matched patch for the rotation-consistency check.
    *angle_ref = 0.0f;
    *angle_cur = 0.0f;
    if(ref_ftr.px.x() > 16.0f && ref_ftr.px.x() < 624.0f
       && ref_ftr.px.y() > 16.0f && ref_ftr.px.y() < 464.0f
       && matcher.px_cur_.x() > 16.0f && matcher.px_cur_.x() < 624.0f
       && matcher.px_cur_.y() > 16.0f && matcher.px_cur_.y() < 464.0f)
    {
      // QUIRK (keep): the reference angle is measured in c.ref_frame's image at the pixel of the
      // close-view frame's observation (ref_frame), which is generally a different keyframe.
      *angle_ref = IC_Angle(c.ref_frame->img(), ref_ftr.px, umax);
      *angle_cur = IC_Angle(frame->img(), matcher.px_cur_, umax);
    }

    point->n_succeeded_reproj_ += 1;
    grad_ref = ref_ftr.grad;
    feature.landmark = point;
    track_id = point->id();
  }

  // Set edgelet direction, check if is consistent.
  if(isEdgelet(c.type))
  {
    GradientVector g_predicted = (matcher.A_cur_ref_.cast<FloatType>() * grad_ref).normalized();
    feature.grad = g_predicted;
  }

  // Here we add a reference in the feature and frame to the 3D point, the other way
  // round is only done if this frame is selected as keyframe.
  feature.type = c.type;
  feature.px = matcher.px_cur_;
  feature.f = matcher.f_cur_;
  feature.level = matcher.search_level_;
  feature.track_id = track_id;
  feature.score = c.score;
  // This assumes that the feature points to the first free slot
  // i.e., the wrapper is got from getEmptyFeatureWrapper function
  frame->invmu_sigma2_a_b_vec_.col(frame->numFeatures()) =
      c.ref_frame->invmu_sigma2_a_b_vec_.col(c.ref_index);
  // Pimax: in_ba_graph_vec_ copy removed
  return true;
}

// 0x180142A60  upstream-modified
void matchCandidates(
    const FramePtr& frame,
    const size_t max_n_features_per_frame,
    const bool affine_est_offset,
    const bool affine_est_gain,
    Reprojector::Candidates& candidates,
    OccupandyGrid2D* /*grid*/,          // Pimax: grid occupancy no longer used
    Reprojector::Statistics& stats,
    const FloatType seed_sigma2_thresh)
{
  Matcher matcher;
  matcher.options_.affine_est_offset_ = affine_est_offset;
  matcher.options_.affine_est_gain_ = affine_est_gain;

  static std::vector<int> umax = computeUmax();                 // guard 0x18047ED88, obj 0x18047ED70
  thread_local static std::vector<int> rot_hist[HISTO_LENGTH];  // TLS
  for(int k = 0; k < HISTO_LENGTH; k++)
  {
    rot_hist[k].clear();
    rot_hist[k].reserve(500);
  }

  int n_rot = 0;
  int i = 0;
  for(Reprojector::Candidate& candidate : candidates)
  {
    ++i;
    const PointPtr point = candidate.ref_frame->landmark_vec_[static_cast<int>(candidate.ref_index)];

    ++stats.n_trials;
    FeatureWrapper feature_wrapper = frame->getEmptyFeatureWrapper();
    float angle_ref, angle_cur;
    if(matchCandidate(frame, candidate, matcher, feature_wrapper, &angle_ref, &angle_cur,
                      umax, seed_sigma2_thresh))
    {
      if(point)
      {
        stats.sum_lm_succeeded_reproj += point->n_succeeded_reproj_;
        stats.sum_lm_obs += point->obs_.size();
        ++stats.n_lm_matches;
      }
      else
      {
        ++stats.n_seed_matches;
      }
      ++stats.n_matches;
      ++frame->num_features_;
      if(max_n_features_per_frame > 0
         && frame->num_features_ >= max_n_features_per_frame)
      {
        break;
      }

      // Pimax: ORB-style rotation histogram (only landmark matches produce angles).
      if(angle_ref != 0.0f && angle_cur != 0.0f)
      {
        float rot = angle_ref - angle_cur;
        if(rot < 0.0f)
          rot += 360.0f;
        int bin = static_cast<int>(std::round(rot * kRotFactor));
        if(bin == HISTO_LENGTH)
          bin = 0;
        if(bin < 0 || bin > HISTO_LENGTH)
        {
          LOGW(" bin < 0 or bin > 30, bin value is %d \n", bin);
        }
        else
        {
          rot_hist[bin].push_back(static_cast<int>(frame->num_features_ - 1));
          ++n_rot;
        }
      }
    }
  }

  if(n_rot > 5)
  {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;
    computeThreeMinima(rot_hist, HISTO_LENGTH, ind1, ind2, ind3);
    for(int k = 0; k < HISTO_LENGTH; k++)
    {
      if(k == ind1 || k == ind2 || k == ind3)
      {
        for(size_t j = 0, jend = rot_hist[k].size(); j < jend; j++)
        {
          const int idx = rot_hist[k][j];
          if(frame->track_id_vec_(idx) != -1)
          {
            size_t index = idx;
            frame->deleteLandmark(index);   // 0x180094460 (c12: "removeLandmark")
          }
        }
      }
    }
  }
  candidates.erase(candidates.begin(), candidates.begin()+i);
}

// 0x180143DB0  Pimax-new: 16 px grid non-maximum suppression on the (sorted) candidates.
// Keeps a candidate if no better (earlier, still valid) candidate lies within 16 px.
void filterCandidatesByGrid(
    Reprojector::Candidates& candidates,
    Reprojector::Candidates& filtered)
{
  filtered.reserve(200);
  std::unordered_map<int, std::vector<int>> grid;
  grid.reserve(candidates.size());
  for(int i = 0; i < static_cast<int>(candidates.size()); ++i)
  {
    const int gx = static_cast<int>(std::floor(candidates[i].cur_px.x() * 0.0625f));
    const int gy = static_cast<int>(std::floor(candidates[i].cur_px.y() * 0.0625f));
    grid[gx * 10000 + gy].push_back(i);
  }

  for(size_t i = 0; i < candidates.size(); ++i)
  {
    Reprojector::Candidate& c = candidates[i];
    if(!c.valid)
      continue;
    filtered.push_back(c);
    const int gx = static_cast<int>(std::floor(c.cur_px.x() * 0.0625f));
    const int gy = static_cast<int>(std::floor(c.cur_px.y() * 0.0625f));
    for(int dx = -1; dx <= 1; ++dx)
    {
      for(int dy = -1; dy <= 1; ++dy)
      {
        const auto it = grid.find((gx + dx) * 10000 + gy + dy);
        if(it == grid.end())
          continue;
        for(const int j : it->second)
        {
          if(j == static_cast<int>(i))
            continue;
          Reprojector::Candidate& other = candidates[j];
          if(!other.valid)
            continue;
          const float ddx = c.cur_px.x() - other.cur_px.x();
          const float ddy = c.cur_px.y() - other.cur_px.y();
          if(ddy * ddy + ddx * ddx < 256.0f)
            other.valid = false;
        }
      }
    }
  }
}

} // namespace reprojector_utils

// 0x1801420C0  Frame::getSeedDepth: inline member of common/frame.h (its COMDAT is emitted in
// this object because getCandidate/matchCandidate are its first users).

} // namespace totem
} // namespace pimax
