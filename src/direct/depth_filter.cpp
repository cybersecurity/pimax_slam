// depth_filter.cpp -- Pimax fork of svo_direct/src/depth_filter.cpp
// Chunk c06_common_depthfilter, object range 0x18009DBE0 .. 0x1800A3900.
//
// Heavily modified vs upstream (see notes/c06_common_depthfilter.md, "DepthFilter"):
//  * no feature detector inside the depth filter; initializeSeeds() runs FAST itself on every
//    camera of a FrameBundle and cross-marks the occupancy grids of the other cameras;
//  * float (FloatType) seed maths, different constants, no glog;
//  * the thread loop moves jobs out of the queue and calls Sleep(0) after every job;
//  * frames whose seeds are all gone are remembered (deque + vector) and skipped.

#include <windows.h>                   // Sleep(0) in updateSeedsLoop (direct KERNEL32 import)

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>                    // Matrix3d::inverse()
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>         // cv::circle
#include <fast/fast.h>                 // fast::fast_xy, fast_corner_detect_10, fast_corner_score_10, fast_nonmax_3x3
#include <vikit/math_utils.h>          // vk::normPdf

#include "common/logger.h"             // LOGD/LOGI/LOGW/LOGE
#include "common/camera.h"
#include "common/frame.h"
#include "common/seed.h"
#include "direct/depth_filter.h"
#include "direct/matcher.h"
#include "frontend/map.h"

namespace pimax {
namespace totem {

// 0x18009F280
DepthFilter::DepthFilter(
    const DepthFilterOptions& options)
  : options_(options)
  , matcher_(new Matcher())
{
  // Pimax: no "DepthFilter: created." message.
  matcher_->options_.scan_on_unit_sphere = options.scan_epi_unit_sphere;
  matcher_->options_.affine_est_offset_ = options.affine_est_offset;
  matcher_->options_.affine_est_gain_ = options.affine_est_gain;
  if(options_.use_threaded_depthfilter)
    startThread();
}

// 0x18009F650
DepthFilter::DepthFilter(
    const DepthFilterOptions& options,
    DetectorOptions /*detector_options*/,
    const std::shared_ptr<CameraBundle>& /*cams*/)
  : DepthFilter(options)
{
  // Pimax: upstream created feature_detector_ / sec_feature_detector_ here; removed.
}

// 0x18009F940  (stopThread() inlined; scalar deleting dtor 0x18009FBC0)
DepthFilter::~DepthFilter()
{
  stopThread();
}

// inlined into 0x18009F280
void DepthFilter::startThread()
{
  if(thread_)
  {
    LOGE("DepthFilter: Thread already started!\n");
    return;
  }
  LOGI("DepthFilter: Start thread.\n");
  thread_.reset(new std::thread(&DepthFilter::updateSeedsLoop, this));
}

// 0x1800A2730
void DepthFilter::stopThread()
{
  if(thread_ != nullptr)
  {
    LOGI("DepthFilter: interrupt and join thread... \n");
    quit_thread_ = true;
    jobs_condvar_.notify_all();
    if(thread_->joinable())
      thread_->join();
    thread_.reset();
  }
  LOGI("DepthFilter: end\n");
}

// 0x18009FE90
void DepthFilter::addKeyframe(
    const FrameBundlePtr& frame_bundle,
    const Map& map)
{
  // allocate memory for new features: one slot per occupancy-grid cell of the camera image.
  for(const FramePtr& frame : frame_bundle->frames_)
  {
    const cv::Mat& img = frame->img_pyr_[0];
    frame->resizeFeatureStorage(
        frame->num_features_ +
        img.rows / frame->grid_cell_size_ * (img.cols / frame->grid_cell_size_));
  }

  if(thread_ == nullptr)
  {
    depth_filter_utils::initializeSeeds(frame_bundle);
    return;
  }

  ulock_t lock(jobs_mut_);

  // clear all other jobs, this one has priority
  JobQueue empty_queue;
  jobs_.swap(empty_queue);

  // Pimax: forget frames-without-seeds that are no longer keyframes in the map.
  if(map.keyframes_.size() != frames_without_seeds_.size())
  {
    auto not_in_map = [&map](const FramePtr& f) {
      return map.keyframes_.find(f->id_) == map.keyframes_.end();
    };
    frames_without_seeds_.erase(
          std::remove_if(frames_without_seeds_.begin(), frames_without_seeds_.end(), not_in_map),
          frames_without_seeds_.end());
    frames_without_seeds_vec_.erase(
          std::remove_if(frames_without_seeds_vec_.begin(), frames_without_seeds_vec_.end(), not_in_map),
          frames_without_seeds_vec_.end());
  }

  jobs_.push(Job(frame_bundle));
  jobs_condvar_.notify_all();
}

// 0x1800A2300
void DepthFilter::reset()
{
  ulock_t lock(jobs_mut_);
  JobQueue empty_queue;
  jobs_.swap(empty_queue);
  frames_without_seeds_.clear();
  frames_without_seeds_vec_.clear();
  // Pimax: no "DepthFilter: RESET." message.
}

// 0x18009FC10
bool DepthFilter::GetFramesWithoutSeeds(std::vector<FramePtr>& frames)
{
  frames.clear();
  ulock_t lock(jobs_mut_);
  if(frames_without_seeds_vec_.empty())
    return false;
  frames = frames_without_seeds_vec_;
  return true;
}

// 0x1800A3630  (thread entry; std::thread invoker is StartAddress 0x18009E4F0)
void DepthFilter::updateSeedsLoop()
{
  while(true)
  {
    // wait for new jobs
    Job job;
    {
      ulock_t lock(jobs_mut_);
      while(jobs_.empty() && !quit_thread_)
        jobs_condvar_.wait(lock);

      if(quit_thread_)
        return;

      job = std::move(jobs_.front());     // Pimax: moved, not copied
      jobs_.pop();
    } // release lock

    // process jobs
    if(job.type == Job::SEED_INIT)
    {
      // Pimax: no feature_detector_mut_ any more.
      depth_filter_utils::initializeSeeds(job.frame_bundle);
    }
    else if(job.type == Job::UPDATE)
    {
      // We get higher precision (10x in the synthetic blender dataset)
      // when we keep updating seeds even though they are converged until
      // the frame handler selects a new keyframe.
      depth_filter_utils::updateSeed(
            *job.cur_frame, *job.ref_frame, job.ref_frame_seed_index, *matcher_,
            options_.seed_convergence_sigma2_thresh, true, false);
    }
    Sleep(0);   // Pimax-new
  }
}

// 0x1800A3270
size_t DepthFilter::updateSeeds(
    const std::vector<FramePtr>& ref_frames_with_seeds,
    const FramePtr& cur_frame)
{
  size_t n_success = 0;
  // Pimax: map-point threshold (mappoint_convergence_sigma2_thresh) is not used any more.
  const FloatType cur_thresh = options_.seed_convergence_sigma2_thresh;
  if(thread_ == nullptr)
  {
    for(const FramePtr& ref_frame : ref_frames_with_seeds)
    {
      for(size_t i = 0; i < ref_frame->num_features_; ++i)
      {
        if(isSeed(ref_frame->type_vec_[i]))
        {
          if(depth_filter_utils::updateSeed(
               *cur_frame, *ref_frame, i, *matcher_, cur_thresh, true, false))
          {
            ++n_success;
          }
        }
      }
    }
    // Pimax: no "updated n Seeds successfully" debug message.
  }
  else
  {
    ulock_t lock(jobs_mut_);
    for(const FramePtr& ref_frame : ref_frames_with_seeds)
    {
      // Pimax: skip reference frames that are already known to have no seeds left.
      if(std::find_if(frames_without_seeds_.begin(), frames_without_seeds_.end(),
                      [&ref_frame](const FramePtr& f) { return f->id_ == ref_frame->id_; })
         != frames_without_seeds_.end())
        continue;

      int n_jobs = 0;
      for(size_t i = 0; i < ref_frame->num_features_; ++i)
      {
        if(isSeed(ref_frame->type_vec_[i]))
        {
          ++n_jobs;
          jobs_.push(Job(cur_frame, ref_frame, i));
        }
      }
      if(n_jobs < 1)
      {
        frames_without_seeds_.push_back(ref_frame);
        frames_without_seeds_vec_.push_back(ref_frame);
      }
    }
    jobs_condvar_.notify_all();
  }
  return n_success;
}

namespace depth_filter_utils {

namespace {

// 0x1800A0520 (6 bytes): std::sort comparator, passed as a function pointer (by value args).
bool compareCornerScore(std::pair<int, fast::fast_xy> a, std::pair<int, fast::fast_xy> b)
{
  return a.first > b.first;
}

// 0x1800A0750: FAST-10 corners with score >= 20, 3x3 non-max suppressed, sorted by
// descending score.  TODO(verify) name.
void detectFastCorners(
    const cv::Mat& img,
    std::vector<std::pair<int, fast::fast_xy>>& corners)
{
  std::vector<int> scores;
  std::vector<fast::fast_xy> fast_corners;
  std::vector<fast::fast_xy> kept_corners;      // filled but never used (Pimax leftover)
  corners.clear();

  // 0x1801AFA30 (fast library, same entry point as feature_detection_utils::fastDetector).
  // TODO(verify) whether the source calls fast_corner_detect_10_sse2 or _10.
  fast::fast_corner_detect_10_sse2(
        (fast::fast_byte*) img.data, img.cols, img.rows, img.step[0], 20, fast_corners);
  std::vector<int> nm_corners;
  fast::fast_corner_score_10(
        (fast::fast_byte*) img.data, img.step[0], fast_corners, 20, scores);
  fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);

  for(auto it = nm_corners.begin(); it != nm_corners.end(); ++it)
  {
    fast::fast_xy& xy = fast_corners.at(*it);
    const int score = scores.at(*it);
    corners.emplace_back(score, xy);
    kept_corners.push_back(xy);
    scores.push_back(score);                 // quirk: appends to the score vector it reads from
  }
  std::sort(corners.begin(), corners.end(), compareCornerScore);
}

// 0x1800A2100 (out of line).  TODO(verify) name.
bool isRotationMatrix(const Eigen::Matrix3d& R)
{
  if(std::fabs(R.determinant() - 1.0) > 0.000001)
    return false;
  return (R * R.transpose() - Eigen::Matrix3d::Identity()).squaredNorm() <= 1e-12;
}

} // anonymous namespace

// 0x1800A0AB0
void initializeSeeds(
    const FrameBundlePtr& frame_bundle)
{
  std::vector<std::pair<int, fast::fast_xy>> corners;
  std::unordered_map<int, std::vector<fast::fast_xy>> cam_corners;

  // 1. detect FAST corners in every camera whose image is bright enough
  for(const FramePtr& frame : frame_bundle->frames_)
  {
    if(frame->mean_intensity_ >= 15.0)
    {
      detectFastCorners(frame->img_pyr_[0], corners);
      for(const auto& c : corners)
        cam_corners[frame->cam_index_].push_back(c.second);
    }
  }

  // 2. mark the cells / mask of the already existing features
  for(const FramePtr& frame : frame_bundle->frames_)
  {
    frame->feature_mask_ = frame->getMask().clone();
    for(size_t i = 0; i < frame->num_features_; ++i)
    {
      const auto px = frame->px_vec_.col(i);
      frame->grid_occupancy_[
          static_cast<int>(px(0) / frame->grid_cell_size_)
          + static_cast<int>(px(1) / frame->grid_cell_size_)
            * (frame->img_pyr_[0].cols / frame->grid_cell_size_)] = true;
      cv::circle(frame->feature_mask_,
                 cv::Point(static_cast<int>(px(0)), static_cast<int>(px(1))),
                 5, cv::Scalar(0), -1, 8, 0);
    }
  }

  // 3. add new seeds
  std::vector<int> visited_cams;
  visited_cams.reserve(4);
  for(const auto& cam_and_corners : cam_corners)
  {
    const int cam_idx = cam_and_corners.first;
    const FramePtr frame = frame_bundle->frames_[cam_idx];
    const size_t n_old = frame->num_features_;
    size_t idx = n_old;
    for(const fast::fast_xy& xy : cam_and_corners.second)
    {
      if(frame->isSaturatedPatch(Eigen::Vector2i(xy.x, xy.y))
         || !(frame->feature_mask_.empty() || frame->feature_mask_.at<uchar>(xy.y, xy.x)))
        continue;

      const int cell = xy.x / frame->grid_cell_size_
          + xy.y / frame->grid_cell_size_ * (frame->img_pyr_[0].cols / frame->grid_cell_size_);
      if(frame->grid_occupancy_[cell])
        continue;
      frame->grid_occupancy_[cell] = true;

      // Pimax: block the same cell in the other cameras (see quirks: the ray is rotated into
      // world, never scaled by a depth, and the cell index uses the ORIGINAL pixel and this
      // camera's grid geometry).
      for(int k = 0; k < frame_bundle->frames_.size(); ++k)
      {
        visited_cams.push_back(cam_idx);
        if(std::find(visited_cams.begin(), visited_cams.end(), k) != visited_cams.end())
          continue;

        const Eigen::Vector2d px_d(xy.x, xy.y);
        Eigen::Vector3d f_raw;
        if(!frame->cam_->backProject3(px_d, &f_raw))
          continue;
        const Eigen::Vector3d f = f_raw.normalized();
        frame->T_f_w_.getRotation().normalize();
        if(frame->T_f_w_.getRotationMatrix().hasNaN()
           || frame->T_f_w_.getPosition().hasNaN()
           || !isRotationMatrix(frame->T_f_w_.getRotationMatrix()))
          continue;
        const Eigen::Vector3d f_w = frame->T_f_w_.getRotationMatrix().inverse() * f;
        const FramePtr& other = frame_bundle->frames_[k];
        const Eigen::Vector3d p_other = other->T_f_w_ * f_w;
        if(p_other(2) >= 0.0)
        {
          Eigen::Vector2d px_other;
          if(other->cam_->project3(p_other, &px_other, nullptr).isKeypointVisible())
          {
            other->grid_occupancy_[
                static_cast<int>(px_d(0) / frame->grid_cell_size_)
                + frame->img_pyr_[0].cols / frame->grid_cell_size_
                  * static_cast<int>(px_d(1) / frame->grid_cell_size_)] = true;
          }
        }
      }

      cv::circle(frame->feature_mask_, cv::Point(xy.x, xy.y), 3, cv::Scalar(0), -1, 8, 0);
      frame->px_vec_(0, idx) = xy.x;
      frame->px_vec_(1, idx) = xy.y;
      const Eigen::Vector2d px(xy.x, xy.y);
      Eigen::Vector3d f;
      if(frame->cam_->backProject3(px, &f))
      {
        frame->f_vec_.col(idx) = f.normalized().cast<FloatType>();
        frame->grad_vec_.col(idx) = GradientVector::Zero();
        frame->score_vec_(idx) = 0;
        frame->level_vec_(idx) = 0;
        frame->type_vec_[idx] = FeatureType::kCornerSeed;
        ++idx;
        frame->num_features_ = idx;
        if(idx > 255)
          break;
      }
    }
    frame->feature_mask_.release();

    const size_t n_new = frame->num_features_ - n_old;

    // initialize seeds
    const FloatType depth_min = std::max(frame->min_depth_ * 0.4f, 0.0f);
    frame->seed_mu_range_ = seed::getMeanRangeFromDepthMinMax(depth_min, depth_min);
    frame->invmu_sigma2_a_b_vec_.block(0, n_old, 1, n_new).setConstant(
          seed::getMeanFromDepth(frame->median_depth_));
    frame->invmu_sigma2_a_b_vec_.block(1, n_old, 1, n_new).setConstant(
          seed::getInitSigma2FromMuRange(frame->seed_mu_range_));
    frame->invmu_sigma2_a_b_vec_.block(2, n_old, 2, n_new).setConstant(10.0f);

    // quirk: prints the number of new seeds + 1
    LOGD("DepthFilter: %s Initialized  %lu Seeds.\n",
         frame->cam_->getLabel().c_str(), idx - n_old + 1);
  }
}

// 0x1800A2AC0
bool updateSeed(
    const Frame& cur_frame,
    Frame& ref_frame,
    const size_t& seed_index,
    Matcher& matcher,
    const FloatType sigma2_convergence_threshold,
    const bool check_visibility,
    const bool check_convergence,
    const bool use_vogiatzis_update)
{
  // Pimax: no "update seed with ref frame" check.
  constexpr double px_noise = 1.0;
  static FloatType px_error_angle = cur_frame.getAngleError(px_noise);

  // check if seed is diverged
  const FeatureType type = ref_frame.type_vec_[seed_index];
  if(type == FeatureType::kOutlier)
  {
    return false;
  }

  // check if already converged  (Pimax: kMapPointSeedConverged no longer listed)
  if((type == FeatureType::kCornerSeedConverged ||
      type == FeatureType::kEdgeletSeedConverged) && check_convergence)
  {
    return false;
  }

  // Create wrappers
  FeatureWrapper ref_ftr = ref_frame.getFeatureWrapper(seed_index);
  Eigen::Ref<SeedState> state = ref_frame.invmu_sigma2_a_b_vec_.col(seed_index);

  // check if point is visible in the current image
  Transformation T_cur_ref = cur_frame.T_f_w_ * ref_frame.T_f_w_.inverse();
  if(check_visibility)
  {
    const Eigen::Vector3d xyz_f(T_cur_ref * (seed::getDepth(state) * ref_ftr.f.cast<double>()));
    if(xyz_f(2) < 0.0)                                   // Pimax-new
      return false;
    Eigen::Vector2d px;
    if(!cur_frame.cam()->project3(xyz_f, &px, nullptr).isKeypointVisible())
      return false;

    // check margin -- Pimax: replaced by a lookup in the camera mask.  The mask is a function
    // static cloned from the camera of the FIRST call (quirk: same mask for every camera).
    const Eigen::Vector2i pxi = px.cast<int>();
    static cv::Mat mask = cur_frame.cam()->getMask().clone();
    if(!mask.at<uchar>(pxi(1), pxi(0)))
      return false;
  }

  // set matcher options
  if(ref_ftr.type == FeatureType::kEdgeletSeed
     || ref_ftr.type == FeatureType::kEdgeletSeedConverged)
    matcher.options_.align_1d = true;
  else
    matcher.options_.align_1d = false;

  // sanity checks  (Pimax: silent early return instead of logging)
  if(std::isnan(seed::sigma2(state)))
    return false;

  // search epipolar line, find match, and triangulate to find new depth z
  FloatType depth = 100.0f;
  Matcher::MatchResult res =
      matcher.findEpipolarMatchDirect(
        ref_frame, cur_frame, T_cur_ref, ref_ftr, seed::getInvDepth(state),
        seed::getInvMinDepth(state), seed::getInvMaxDepth(state), depth);

  if(res != Matcher::MatchResult::kSuccess)
  {
    // Pimax: outlier after too many failures or when the returned depth is far.
    if(!matcher.reject_)
    {
      seed::increaseOutlierProbability(state);
      if(state(3) > 16.5f)
      {
        ref_frame.type_vec_[seed_index] = FeatureType::kOutlier;
        return false;
      }
    }
    if(depth > 50.0f)
      ref_frame.type_vec_[seed_index] = FeatureType::kOutlier;
    return false;
  }

  // compute tau
  const FloatType depth_sigma = computeTau(T_cur_ref.inverse(), ref_ftr.f, depth, px_error_angle);

  // update the estimate
  if(use_vogiatzis_update)
  {
    if(!updateFilterVogiatzis(
         seed::getMeanFromDepth(depth),
         seed::getSigma2FromDepthSigma(depth, depth_sigma),
         ref_frame.seed_mu_range_,
         state))
    {
      ref_ftr.type = FeatureType::kOutlier;
      return false;
    }
  }
  else
  {
    if(!updateFilterGaussian(
         seed::getMeanFromDepth(depth),
         seed::getSigma2FromDepthSigma(depth, depth_sigma),
         state))
    {
      ref_ftr.type = FeatureType::kOutlier;
      return false;
    }
  }

  // check if converged
  if(seed::isConverged(state,
                       ref_frame.seed_mu_range_,
                       sigma2_convergence_threshold))
  {
    if(ref_ftr.type == FeatureType::kCornerSeed)
      ref_ftr.type = FeatureType::kCornerSeedConverged;
    else if(ref_ftr.type == FeatureType::kEdgeletSeed)
      ref_ftr.type = FeatureType::kEdgeletSeedConverged;
    // Pimax: kMapPointSeed branch removed.
  }
  return true;
}

// 0x1800A2810
bool updateFilterVogiatzis(
    const FloatType z, // Measurement
    const FloatType tau2,
    const FloatType mu_range,
    Eigen::Ref<SeedState>& mu_sigma2_a_b)
{
  FloatType& mu = mu_sigma2_a_b(0);
  FloatType& sigma2 = mu_sigma2_a_b(1);
  FloatType& a = mu_sigma2_a_b(2);
  FloatType& b = mu_sigma2_a_b(3);

  const FloatType norm_scale = std::sqrt(sigma2 + tau2);
  if(std::isnan(norm_scale))
  {
    return false;                                 // Pimax: no LOG(WARNING)
  }

  // Pimax: all literals are float (float arithmetic throughout).
  const FloatType s2 = 1.0f/(1.0f/sigma2 + 1.0f/tau2);
  const FloatType m = s2*(mu/sigma2 + z/tau2);
  const FloatType uniform_x = 1.0f/mu_range;
  FloatType C1 = a/(a+b) * vk::normPdf<FloatType>(z, mu, norm_scale);
  FloatType C2 = b/(a+b) * uniform_x;
  const FloatType normalization_constant = C1 + C2;
  C1 /= normalization_constant;
  C2 /= normalization_constant;
  const FloatType f = C1*(a+1.0f)/(a+b+1.0f) + C2*a/(a+b+1.0f);
  const FloatType e = C1*(a+1.0f)*(a+2.0f)/((a+b+1.0f)*(a+b+2.0f))
                    + C2*a*(a+1.0f)/((a+b+1.0f)*(a+b+2.0f));

  // update parameters
  const FloatType mu_new = C1*m+C2*mu;
  sigma2 = C1*(s2 + m*m) + C2*(sigma2 + mu*mu) - mu_new*mu_new;
  mu = mu_new;
  a = (e - f) / (f - e / f);
  b = a * (1.0f - f) / f;

  // Pimax: the "sigma2 < 0 -> restore" and "mu < 0 -> diverged" checks are gone.
  return true;
}

// Inlined into 0x1800A2AC0 (else-branch).  Only the NaN test is observable in the binary.
bool updateFilterGaussian(
    const FloatType z, // Measurement
    const FloatType tau2,
    SeedState mu_sigma2_a_b)
{
  FloatType& mu = mu_sigma2_a_b(0);
  FloatType& sigma2 = mu_sigma2_a_b(1);

  const FloatType denom = (sigma2 + tau2);
  if(std::isnan(denom))
  {
    return false;
  }
  mu = (sigma2 * z + tau2 * mu) / denom;
  sigma2 = sigma2 * tau2 / denom;
  return true;
}

// 0x1800A0530
FloatType computeTau(
      const Transformation& T_ref_cur,
      const BearingVector& f,
      const FloatType z,
      const FloatType px_error_angle)
{
  const BearingVector t = T_ref_cur.getPosition().cast<FloatType>();
  const BearingVector a = f*z-t;
  FloatType t_norm = t.norm();
  FloatType a_norm = a.norm();
  FloatType alpha = std::acos(f.dot(t)/t_norm); // dot product
  FloatType beta = std::acos(a.dot(-t)/(t_norm*a_norm)); // dot product
  FloatType beta_plus = beta + px_error_angle;
  FloatType gamma_plus = M_PI-alpha-beta_plus; // triangle angles sum to PI (double)
  FloatType z_plus = t_norm*std::sin(beta_plus)/std::sin(gamma_plus); // law of sines
  return (z_plus - z); // tau
}

} // namespace depth_filter_utils
} // namespace totem
} // namespace pimax
