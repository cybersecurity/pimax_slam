// pimax_slam.pi.dll -- src/frontend/stereo_triangulation.cpp  (drafts c12 + c13 merged, phase B3)
//
// pimax::totem::StereoTriangulation -- fork of rpg_svo_pro_open/svo/src/stereo_triangulation.cpp,
// heavily modified by Pimax.  Object TU36, code 0x180144340..0x180149A50 (ctor / triangulate /
// computeStd and their Eigen/STL instantiations: c12; compute 0x180147110: c13).
// No glog lines in this object (Pimax logger only).
//
// compute() vs upstream:
//   * new bool argument `is_init`; detector budget 400/256 minus the existing features instead of
//     grid_.size(); the existing features are blotted out of a copy of the mask (this->mask_)
//   * "bright image" pre-check (only when is_init): reject the frame pair when more than 20 % of
//     2*N new corners have a 5x5 neighbourhood that is completely <15 or >130
//   * new scores are NOT copied into frame0->score_vec_ (upstream does)
//   * random_shuffle -> std::shuffle with a fresh std::mt19937(std::random_device()()) per call
//     (PIMAX_SLAM_TEST_DETERMINISTIC: std::mt19937(5489u))
//   * matches are no longer inserted immediately: each match is triangulated with a DLT
//     (triangulate 0x180145470), filtered by depth (0.05 .. 10), by result >= 0 and by a
//     parallax angle >= 0.5 deg; the candidates are buffered and finally only those whose value
//     is within 2 sigma of the mean are inserted into frame0/frame1
//   * Matcher options: max_epi_search_steps 500, subpix_refinement true (same as upstream)
//   * glog VLOG/SVO_ERROR_STREAM replaced by the Pimax logger; the final VLOG(20) is gone
//
// Literal checks (tools/ida_dump.py rd): 100.0 @0x1803B6D30, 0.2 @0x1803ADDC0, pi @0x1803B1390,
// 180.0 @0x1803B6D38, 0.5 @0x1803AF2B8; 10.0f / 0.05f are float compares (comiss).
#include "frontend/stereo_triangulation.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <random>
#include <vector>

#include <Eigen/SVD>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "common/camera.h"
#include "common/frame.h"
#include "common/logger.h"
#include "common/point.h"
#include "direct/feature_detection.h"
#include "direct/matcher.h"

namespace pimax {
namespace totem {

// 0x180145320  upstream-modified (extra cv::Mat member)
StereoTriangulation::StereoTriangulation(
    const StereoTriangulationOptions& options,
    const DetectorPtr& feature_detector)
  : options_(options)
  , feature_detector_(feature_detector)
{ ; }

// 0x180145470  Pimax-new (ORB-SLAM style linear triangulation + checks), called from compute().
// (c13 called it triangulatePoint(..., Vector3d*); the returned value is the depth, which
// compute() buffers as its "error" for the 2-sigma gate.)
// Returns the depth (z in frame0) on success, or a negative error code:
//   -1 degenerate (|w| < 1e-8), -2 behind cam0/cam1 or projection failed,
//   -3 negative depth in cam1 (via R.row(2)), -4 reprojection error in frame0 too large,
//   -5 reprojection error in frame1 too large.
// P0/P1: 3x4 projection matrices in normalised (unit-plane) coordinates (column-major).
// x0/y0: the first two components of ref_ftr.f_raw (FeatureWrapper +56, a Ref into the 3xN
// f_vec_raw_; c12 read it as a Vector2f "uv"), x1/y1: the first two components of
// matcher.f_cur_unnormalized_ (Matcher +332/+336; c12 "uv_cur_").  The un-normalised
// back-projection has z == 1, so these are the unit-plane coordinates.
// `this` is unused in the body.
double StereoTriangulation::triangulate(
    const FramePtr& frame0,
    const FramePtr& frame1,
    const Transformation& T_f1f0,
    const Eigen::Matrix<double, 3, 4>& P0,
    const Eigen::Matrix<double, 3, 4>& P1,
    const Matcher& matcher,
    const FeatureWrapper& ref_ftr,
    Eigen::Vector3d& x3D) const
{
  // float coordinates, promoted to double before the multiplications (cvtss2sd in the binary)
  const double x1 = matcher.f_cur_unnormalized_(0);
  const double y1 = matcher.f_cur_unnormalized_(1);
  const double x0 = ref_ftr.f_raw(0);
  const double y0 = ref_ftr.f_raw(1);

  Eigen::Matrix4d A;
  A.row(0) = x0 * P0.row(2) - P0.row(0);
  A.row(1) = y0 * P0.row(2) - P0.row(1);
  A.row(2) = x1 * P1.row(2) - P1.row(0);
  A.row(3) = y1 * P1.row(2) - P1.row(1);

  Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
  Eigen::Vector4d x3Dh = svd.matrixV().col(3);
  if(std::fabs(x3Dh(3)) < 1e-8)
    return -1.0;

  x3D = x3Dh.head(3) / x3Dh(3);
  if(x3D(2) <= 0.0)
    return -2.0;

  // positive depth in the second camera
  if(T_f1f0.getRotationMatrix().row(2).dot(x3D) + P1(2, 3) <= 0.0)
    return -3.0;

  // reprojection error in the first image
  if(x3D(2) < 0.0)
    return -2.0;
  Eigen::Vector2d px0;
  if(!frame0->cam()->project3(x3D, &px0, nullptr).isKeypointVisible())
    return -2.0;
  const double ex0 = px0(0) - ref_ftr.px(0);
  const double ey0 = px0(1) - ref_ftr.px(1);
  if(ex0 * ex0 + ey0 * ey0 > frame0->level_reproj_thresh_[ref_ftr.level])
    return -4.0;

  // reprojection error in the second image
  const Eigen::Vector3d x3D_c1 = P1.col(3) + T_f1f0.getRotationMatrix() * x3D;
  Eigen::Vector2d px1;
  if(x3D_c1(2) < 0.0)
    return -2.0;
  if(!frame1->cam()->project3(x3D_c1, &px1, nullptr).isKeypointVisible())
    return -2.0;
  const double ex1 = px1(0) - matcher.px_cur_(0);
  const double ey1 = px1(1) - matcher.px_cur_(1);
  if(ex1 * ex1 + ey1 * ey1 > frame1->level_reproj_thresh_[matcher.search_level_])
    return -5.0;

  return x3D(2);
}

// 0x180146360  Pimax-new: standard deviation around a given mean (used by compute()).
double StereoTriangulation::computeStd(const std::vector<double>& values, double mean)
{
  double sum = 0.0;
  for(const double v : values)
    sum += std::pow(v - mean, 2);
  return std::sqrt(sum / values.size());
}

// 0x180147110
void StereoTriangulation::compute(const FramePtr& frame0, const FramePtr& frame1, bool is_init)
{
  // Check if there is something to do
  if (frame0->numLandmarks() >= options_.triangulate_n_features)
  {
    LOGI("Calling stereo triangulation with sufficient number of features has no effect.\n");
    return;
  }
  LOGI("Stereo Init Triangulation: ++++++++++++++++++++ \n");

  // Detect new features.
  Keypoints new_px;
  Scores new_scores;
  Levels new_levels;
  Gradients new_grads;
  FeatureTypes new_types;
  const size_t max_n_features =
      is_init ? 400 - frame0->num_features_ : 256 - frame0->num_features_;
  if (frame0->num_features_ != 0)
  {
    // Pimax: do not detect again on top of features that are already tracked / seeded.
    frame0->getMask().copyTo(mask_);
    for (size_t i = 0; i < frame0->num_features_; ++i)
    {
      // The col(i) Block (and its index assert) is built before the track-id test.
      const auto px = frame0->px_vec_.col(i);
      if (frame0->track_id_vec_(i) != -1 || frame0->seed_ref_vec_[i].keyframe)
      {
        cv::circle(mask_, cv::Point(static_cast<int>(px(0)), static_cast<int>(px(1))), 3,
                   cv::Scalar(0), -1);   // lineType 8, shift 0 (defaults)
      }
    }
    feature_detector_->detect(frame0->img_pyr_, mask_, max_n_features, new_px, new_scores,
                              new_levels, new_grads, new_types);
  }
  else
  {
    feature_detector_->detect(frame0->img_pyr_, frame0->getMask(), max_n_features, new_px,
                              new_scores, new_levels, new_grads, new_types);
  }
  if (new_px.cols() == 0)
  {
    LOGW("Stereo tri: No features.\n");
    return;
  }

  // Pimax: reject bright frames whose new corners mostly sit in saturated / black patches.
  if (is_init && (frame0->is_too_dark_ || frame0->mean_intensity_ > 100.0))
  {
    int bad_count = 0;
    for (int i = 0; i < new_px.cols(); ++i)
    {
      const Keypoint px = new_px.col(i);
      const cv::Mat& img = frame0->img_pyr_[0];
      const int x = static_cast<int>(px(0));
      const int y = static_cast<int>(px(1));
      const int x_min = std::max(0, x - 2);
      const int x_max = std::min(x + 2, img.cols - 1);
      const int y_min = std::max(0, y - 2);
      const int y_max = std::min(y + 2, img.rows - 1);
      int n_total = 0;
      int n_out_of_range = 0;
      for (int r = y_min; r <= y_max; ++r)
      {
        for (int c = x_min; c <= x_max; ++c)
        {
          ++n_total;
          const uchar v = frame0->img_pyr_[0].at<uchar>(r, c);
          if (v < 15 || v > 130)   // compiled as (uint8_t)(v - 15) > 115
            ++n_out_of_range;
        }
      }
      if (n_out_of_range == n_total)
        ++bad_count;
    }
    // int count / float(Index): cvtdq2ps + cvtsi2ss(64-bit) + divss, compared as double.
    const float bad_rate = static_cast<float>(bad_count) / static_cast<float>(2 * new_px.cols());
    if (bad_rate > 0.2)
    {
      LOGE("Stereo Init Triangulation: bad_frate %f > 0.2 \n", bad_rate);
      return;
    }
  }

  // Compute and normalize all bearing vectors.
  Bearings new_f;
  Bearings new_f_raw;
  frame_utils::computeNormalizedBearingVectors(new_px, *frame0->cam(), &new_f, &new_f_raw);

  // Add features to first frame. (long == 32 bit on Windows: n_old/n_new are int.)
  const long n_old = static_cast<long>(frame0->num_features_);
  const long n_new = new_px.cols();
  frame0->resizeFeatureStorage(frame0->num_features_ + static_cast<size_t>(n_new));
  frame0->px_vec_.middleCols(n_old, n_new) = new_px;
  frame0->f_vec_.middleCols(n_old, n_new) = new_f;
  frame0->f_vec_raw_.middleCols(n_old, n_new) = new_f_raw;
  frame0->grad_vec_.middleCols(n_old, n_new) = new_grads;
  // Pimax: score_vec_ is NOT filled (upstream: score_vec_.segment(n_old, n_new) = new_scores).
  frame0->level_vec_.segment(n_old, n_new) = new_levels;
  frame0->num_features_ += static_cast<size_t>(n_new);
  frame0->type_vec_.insert(frame0->type_vec_.begin() + n_old, new_types.cbegin(), new_types.cend());

  // We only want a limited number of features. Therefore, we create a random
  // vector of indices that we will process.
  std::vector<size_t> indices(static_cast<size_t>(n_new));
  std::iota(indices.begin(), indices.end(), n_old);
  long n_corners = std::count_if(new_types.begin(), new_types.end(),
                                 [](const FeatureType& t) { return t == FeatureType::kCorner; });

  // shuffle twice before we prefer corners!
  // TODO(verify): one shared std::random_device vs. two temporaries -- identical code.
  // Two calls of std::_Random_device (0x180148065, 0x1801480EC).  Test build
  // (-DPIMAX_SLAM_TEST_DETERMINISTIC): seed with the std::mt19937 default seed 5489, like
  // test/make_deterministic_orig.py does for the original DLL (std::_Random_device -> 5489).
#ifdef PIMAX_SLAM_TEST_DETERMINISTIC
  std::shuffle(indices.begin(), indices.begin() + n_corners, std::mt19937(5489u));
  std::shuffle(indices.begin() + n_corners, indices.end(), std::mt19937(5489u));
#else
  std::shuffle(indices.begin(), indices.begin() + n_corners, std::mt19937(std::random_device()()));
  std::shuffle(indices.begin() + n_corners, indices.end(), std::mt19937(std::random_device()()));
#endif

  // now for all maximum corners, initialize a new seed
  size_t n_succeded = 0;
  const size_t n_desired = options_.triangulate_n_features - frame0->numLandmarks();
  // note: we checked already at start that n_desired will be larger than 0

  // reserve space for features in second frame
  if (frame1->num_features_ + n_desired > frame1->landmark_vec_.size())
  {
    frame1->resizeFeatureStorage(frame1->num_features_ + n_desired);
  }

  // Pimax leftovers: four locals that are constructed and destroyed but never used
  // (element sizes 8, 8, 144 and 352/16-aligned). TODO(verify) element types.
  std::vector<size_t> unused_vec_a;
  std::vector<size_t> unused_vec_b;
  std::vector<FeatureWrapper> unused_ftrs;
  std::vector<Matcher> unused_matchers;

  Matcher matcher;
  matcher.options_.max_epi_search_steps = 500;
  matcher.options_.subpix_refinement = true;
  Transformation T_f1f0 = frame1->T_f_w_ * frame0->T_world_cam();
  T_f1f0.getRotation().normalize();   // inline renormalisation after the (already normalising)
                                      // operator* 0x180009B80. TODO(verify) exact spelling.

  std::vector<size_t> ref_indices;          // ref feature index of each candidate
  std::vector<PointPtr> new_points;
  std::vector<FeatureWrapper> ref_ftrs;
  std::vector<GradientVector> cur_grads;
  std::vector<Keypoint> cur_pxs;
  std::vector<BearingVector> cur_fs;
  std::vector<double> errors;
  const size_t n_reserve = std::min(indices.size(), n_desired);
  new_points.reserve(n_reserve);
  ref_indices.reserve(n_reserve);
  ref_ftrs.reserve(n_reserve);
  cur_grads.reserve(n_reserve);
  cur_pxs.reserve(n_reserve);
  cur_fs.reserve(n_reserve);
  errors.reserve(n_reserve);

  // Projection matrices for the DLT: P0 = [I|0], P1 = [R_f1f0|t_f1f0].
  Eigen::Matrix<double, 3, 4> P0;
  Eigen::Matrix<double, 3, 4> P1;
  P0.setIdentity();
  P1.leftCols<3>() = T_f1f0.getRotationMatrix();
  P1.col(3) = T_f1f0.getPosition();

  const Transformation T_world_cam0 = frame0->T_world_cam();
  const Eigen::Vector3f cam0_center = T_world_cam0.getPosition().cast<float>();
  const Transformation T_world_cam1 = frame1->T_world_cam();
  const Eigen::Vector3f cam1_center = T_world_cam1.getPosition().cast<float>();

  for (const size_t& i_ref : indices)
  {
    matcher.options_.align_1d = isEdgelet(frame0->type_vec_[i_ref]);
    FloatType depth = 0.0;
    FeatureWrapper ref_ftr = frame0->getFeatureWrapper(i_ref);
    Matcher::MatchResult res = matcher.findEpipolarMatchDirect(
        *frame0, *frame1, T_f1f0, ref_ftr, options_.mean_depth_inv, options_.min_depth_inv,
        options_.max_depth_inv, depth);

    if (res == Matcher::MatchResult::kSuccess)
    {
      // NaN depth passes (comiss/ja + comiss/jnb).
      if (depth > 10.0f || depth <= 0.05f)
        continue;

      Eigen::Vector3d xyz_in_f0;
      const double error = triangulate(frame0, frame1, T_f1f0, P0, P1, matcher, ref_ftr,
                                       xyz_in_f0);
      if (error < 0.0)
        continue;

      const Eigen::Vector3f xyz_world = T_world_cam0.cast<float>() * xyz_in_f0.cast<float>();
      const Eigen::Vector3f dir0 = (cam0_center - xyz_world).normalized();
      const Eigen::Vector3f dir1 = (cam1_center - xyz_world).normalized();
      const double parallax_deg = std::acos(std::fabs(dir1.dot(dir0))) / M_PI * 180.0;
      if (parallax_deg < 0.5)
        continue;

      PointPtr new_point = std::make_shared<Point>(xyz_world);
      new_points.push_back(new_point);
      errors.push_back(error);
      ref_indices.push_back(i_ref);
      GradientVector g = matcher.A_cur_ref_.cast<float>() * ref_ftr.grad;
      cur_grads.push_back(g);
      ref_ftrs.push_back(ref_ftr);
      cur_fs.push_back(matcher.f_cur_);
      cur_pxs.push_back(matcher.px_cur_);
      ++n_succeded;
    }
    if (n_succeded >= n_desired)
      break;
  }

  if (errors.empty())
    return;

  // Keep only candidates whose triangulation error is within 2 sigma of the mean.
  // (stddev == 0 -> 0/0 = NaN -> every candidate is rejected; `<= 2.0` compiled with jb.)
  const double mean = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
  const double stddev = computeStd(errors, mean);
  for (int i = 0; i < static_cast<int>(new_points.size()); ++i)
  {
    if (!new_points[i])
      continue;
    const size_t i_ref = ref_indices[i];
    if (i_ref < frame0->landmark_vec_.size() && (errors[i] - mean) / stddev <= 2.0)
    {
      frame0->landmark_vec_[i_ref] = new_points[i];
      frame0->track_id_vec_(static_cast<int>(i_ref)) = new_points[i]->id();
      new_points[i]->addObservation(frame0, i_ref);

      const int i_cur = static_cast<int>(frame1->num_features_);
      frame1->type_vec_[static_cast<size_t>(i_cur)] = ref_ftrs[i].type;
      frame1->level_vec_[i_cur] = ref_ftrs[i].level;
      frame1->px_vec_.col(i_cur) = cur_pxs[i];
      frame1->f_vec_.col(i_cur) = cur_fs[i];
      frame1->score_vec_[i_cur] = ref_ftrs[i].score;
      frame1->grad_vec_.col(i_cur) = cur_grads[i].normalized();
      frame1->landmark_vec_[static_cast<size_t>(i_cur)] = new_points[i];
      frame1->track_id_vec_(i_cur) = new_points[i]->id();
      new_points[i]->addObservation(frame1, static_cast<size_t>(i_cur));
      frame1->num_features_++;
    }
  }
}

}  // namespace totem
}  // namespace pimax
