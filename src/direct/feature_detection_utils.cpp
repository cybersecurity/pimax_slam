// feature_detection_utils.cpp -- Pimax fork of svo_direct/src/feature_detection_utils.cpp
// Chunk c06_common_depthfilter, object range 0x1800AAC30 .. 0x1800ADD10
// (COMDATs are sorted by decorated name inside the object: std::stable_sort helpers first,
//  then edgeletDetector_V2, fastDetector, fillFeatures, getAngleAtPixelUsingHistogram,
//  makeDetector, cv::parallel_for_ (inline OpenCV wrapper), smoothOrientationHistogram).
// Only the functions present in the image are reconstructed.

#include "direct/feature_detection_utils.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <fast/fast.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "common/frame.h"
#include "direct/feature_detection.h"

namespace pimax {
namespace totem {
namespace feature_detection_utils {

//------------------------------------------------------------------------------
// 0x1800AD840
AbstractDetector::Ptr makeDetector(
    const DetectorOptions& options,
    const CameraPtr& cam)
{
  // Pimax: the detector_type switch is gone; always FastGradDetector.
  // (shared_ptr built from a unique_ptr -> _Ref_count_resource<FastGradDetector*,
  //  default_delete<FastGradDetector>>, vtable 0x1803B2808.)
  return AbstractDetector::Ptr(std::unique_ptr<FastGradDetector>(new FastGradDetector(options, cam)));
}

//------------------------------------------------------------------------------
// 0x1800ACB30
void fillFeatures(const Corners& corners,
    const FeatureType& type,
    const double& threshold,
    const size_t max_n_features,
    Keypoints& keypoints,
    Scores& scores,
    Levels& levels,
    Gradients& gradients,
    FeatureTypes& types,
    OccupandyGrid2D& grid)
{
  // Pimax: no CHECK_EQs, no mask test.

  // copy new features in temporary vectors
  std::vector<Keypoint, Eigen::aligned_allocator<Keypoint>> keypoint_vec;
  std::vector<GradientVector, Eigen::aligned_allocator<GradientVector>> gradient_vec;
  std::vector<Score> score_vec;
  std::vector<Level> level_vec;
  keypoint_vec.reserve(corners.size());
  gradient_vec.reserve(corners.size());
  level_vec.reserve(corners.size());
  score_vec.reserve(corners.size());
  for(const Corner& c : corners)
  {
    if(c.score > threshold)
    {
      // we don't need to check for grid occupancy since we already do that during
      // feature extraction.
      keypoint_vec.emplace_back(c.x, c.y);
      level_vec.emplace_back(c.level);
      score_vec.emplace_back(c.score);
      gradient_vec.emplace_back(std::cos(c.angle), std::sin(c.angle));
      grid.occupancy_[grid.getCellIndex(c.x, c.y)] = true;
      grid.feature_occupancy_[grid.getCellIndex(c.x, c.y)] = Keypoint(c.x, c.y);
    }
  }

  // Sort according to scores (bigger score should be better)
  // Pimax: std::stable_sort (MSVC: insertion sort <= 32, 512-element stack buffer).
  std::vector<size_t> idx(score_vec.size());
  std::iota(idx.begin(), idx.end(), 0u);
  std::stable_sort(idx.begin(), idx.end(), [&score_vec](size_t i1, size_t i2)
            { return score_vec[i1] > score_vec[i2]; });

  // copy temporary in eigen block
  const size_t n_old = keypoints.cols();
  const size_t n_new = std::min(max_n_features, keypoint_vec.size());
  const size_t n_tot = n_old + n_new;

  keypoints.conservativeResize(Eigen::NoChange, n_tot);
  gradients.conservativeResize(Eigen::NoChange, n_tot);
  scores.conservativeResize(n_tot);
  levels.conservativeResize(n_tot);
  types.resize(n_tot, type);

  for(size_t i = 0; i < n_new; ++i)
  {
    const size_t j = idx[i];
    keypoints.col(n_old+i) = keypoint_vec[j];
    gradients.col(n_old+i) = gradient_vec[j];
    scores(n_old+i) = score_vec[j];
    levels(n_old+i) = level_vec[j];
  }
}

//------------------------------------------------------------------------------
// 0x1800AC5E0
void fastDetector(
    const ImgPyr& img_pyr,
    const int threshold,
    const int border,
    const size_t min_level,
    const size_t max_level,
    const cv::Mat& mask,
    Corners& corners,
    OccupandyGrid2D& grid)
{
  // Pimax: no CHECK_EQ / CHECK_LE.
  for(size_t level=min_level; level<=max_level; ++level)
  {
    const int scale = (1<<level);
    std::vector<fast::fast_xy> fast_corners;
    // 0x1801AFA30 (fast library): if(w < 22) generic detector; else if(h < 7) return; else
    // detector.  TODO(verify) which fast entry point the source names.
    fast::fast_corner_detect_10_sse2(
          (fast::fast_byte*) img_pyr[level].data, img_pyr[level].cols,
          img_pyr[level].rows, img_pyr[level].step, threshold, fast_corners);

    // Pimax: drop corners outside the mask (mask is level-0 sized).
    if(!mask.empty())
    {
      for(auto it = fast_corners.begin(); it != fast_corners.end(); )
      {
        if(mask.at<uint8_t>(scale * it->y, scale * it->x))
          ++it;
        else
          it = fast_corners.erase(it);
      }
    }

    std::vector<int> scores, nm_corners;
    fast::fast_corner_score_10((fast::fast_byte*) img_pyr[level].data, img_pyr[level].step,
                               fast_corners, threshold, scores);
    fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);

    const int maxw = img_pyr[level].cols-border;
    const int maxh = img_pyr[level].rows-border;
    for(const int& i : nm_corners)
    {
      fast::fast_xy& xy = fast_corners.at(i);
      if(xy.x < border || xy.y < border || xy.x >= maxw || xy.y >= maxh)
        continue;

      // Pimax: reject corners in (nearly) saturated areas.
      {
        const cv::Mat& img = img_pyr[level];
        const int r = (level == 0) ? 2 : ((level == 1) ? 1 : 0);
        const int x0 = std::max(0, xy.x - r);
        const int x1 = std::min(img.cols - 1, xy.x + r);
        const int y0 = std::max(0, xy.y - r);
        const int y1 = std::min(img.rows - 1, xy.y + r);
        int n_total = 0;
        int n_bright = 0;
        for(int v = y0; v <= y1; ++v)
        {
          for(int u = x0; u <= x1; ++u)
          {
            ++n_total;
            if(img.at<uint8_t>(v, u) > 200)
              ++n_bright;
          }
        }
        if(n_bright > n_total * 0.8)
          continue;
      }

      const size_t k = grid.getCellIndex(xy.x, xy.y, scale);
      if(grid.occupancy_.at(k))
        continue;
      const float score = scores.at(i); //vk::shiTomasiScore(img_pyr[L], xy.x, xy.y);
      if(score > corners.at(k).score)
        corners.at(k) = Corner(xy.x*scale, xy.y*scale, score, level, 0.0f);
    }
  }
}

//------------------------------------------------------------------------------
// 0x1800AC200
void edgeletDetector_V2(
    const ImgPyr& img_pyr,
    const int threshold,
    const int border,
    const int min_level,
    const int max_level,
    Corners& corners,
    OccupandyGrid2D& grid)
{
  // Pimax: no CHECK_EQ.
  constexpr int level = 1;
  constexpr int scale = (1<<level);
  cv::Mat score(img_pyr[level].size(), CV_32FC1, cv::Scalar(0.0f));
  cv::Mat angle(img_pyr[level].size(), CV_8UC1, cv::Scalar(255));

  // compute image first derivative
  cv::Mat img, dx, dy;
  img = img_pyr[level];                          // Pimax: no GaussianBlur
  cv::Scharr(img, dx, CV_16S, 1, 0, 1, 0, cv::BORDER_DEFAULT);
  cv::Scharr(img, dy, CV_16S, 0, 1, 1, 0, cv::BORDER_DEFAULT);

  // compute angle and magnitude in angle direction
  // Pimax: rows in parallel; angle only written for strong pixels.
  const int max_row = dx.rows-border;
  const int max_col = dx.cols-border;
  const float angle_factor = 10.0 / (2.0 * M_PI);   // 1.5915494f, captured by reference
  cv::parallel_for_(cv::Range(border, max_row), [&](const cv::Range& range)  // lambda body: 0x1800ABEB0
  {
    for(int y = range.start; y < range.end; ++y)
    {
      int16_t* p_dx = dx.ptr<int16_t>(y);
      int16_t* p_dy = dy.ptr<int16_t>(y);
      float* p_score = score.ptr<float>(y);
      uint8_t* p_angle = angle.ptr<uint8_t>(y);
      for(int x = border; x < max_col; ++x)
      {
        const float mag = std::sqrt(p_dx[x]*p_dx[x]+p_dy[x]*p_dy[x]);
        if(mag > threshold)
        {
          p_score[x] = mag;
          p_angle[x] = (std::atan2(p_dy[x], p_dx[x]) + M_PI) * angle_factor;
        }
        else
        {
          p_score[x] = 0.0f;
        }
      }
    }
  });

  // 8-neighbor nonmax suppression
  // Pimax: stride in floats (upstream used the byte step as a float offset).
  const int stride = score.step / sizeof(float);
  cv::parallel_for_(cv::Range(border, score.rows-border), [&](const cv::Range& range)  // 0x1800ABB50
  {
    for(int y = range.start; y < range.end; ++y)
    {
      const float* p = &score.at<float>(y,border);
      for(int x=border; x<score.cols-border; ++x, ++p)
      {
        const int k = grid.getCellIndex(x, y, scale);
        if(grid.occupancy_.at(k))
          continue;

        const float* const center=p;
        if(*center<threshold) continue;
        if(*(center+1)>=*center) continue;
        if(*(center-1)>*center) continue;
        const float* const p1=(center+stride);
        const float* const p2=(center-stride);
        if(*p1>=*center) continue;
        if(*p2>*center) continue;
        if(*(p1+1)>=*center) continue;
        if(*(p1-1)>*center) continue;
        if(*(p2+1)>=*center) continue;
        if(*(p2-1)>*center) continue;

        Corner& c = corners.at(k);
        if(*p > c.score)
        {
          c.x=x*scale;
          c.y=y*scale;
          c.level=level-1;
          c.score=*p;
          c.angle = getAngleAtPixelUsingHistogram(img_pyr[level], Eigen::Vector2i(x,y), 4);
        }
      }
    }
  });
  (void)min_level;
  (void)max_level;
}

//------------------------------------------------------------------------------
// 0x1800AD4A0 (angleHistogram, gradientAndMagnitudeAtPixel, getDominantAngle inlined)
float getAngleAtPixelUsingHistogram(
    const cv::Mat& img, const Eigen::Vector2i& px, const size_t halfpatch_size)
{
  angle_hist::AngleHistogram hist;
  hist.fill(0.0);
  angle_hist::angleHistogram(img, px(0), px(1), halfpatch_size, hist);
  angle_hist::smoothOrientationHistogram(hist);
  return angle_hist::getDominantAngle(hist);
}

namespace angle_hist {

// inlined into 0x1800AD4A0
void angleHistogram(
    const cv::Mat& img, int x, int y, int halfpatch_size, AngleHistogram& hist)
{
  constexpr float pi2 = 2.0 * M_PI;              // Pimax: float (bin divisor 6.2831854820...)
  for(int dy = -halfpatch_size; dy <= halfpatch_size; ++dy)
  {
    for(int dx = -halfpatch_size; dx <= halfpatch_size; ++dx)
    {
      double mag, angle;
      if(gradientAndMagnitudeAtPixel(img, x + dx, y + dy, &mag, &angle))
      {
        size_t bin = std::round( n_bins * ( angle + M_PI ) / pi2 );
        bin = (bin < n_bins) ? bin : 0u;
        hist[bin] += mag;
      }
    }
  }
}

// inlined into 0x1800AD4A0 (upstream-identical)
bool gradientAndMagnitudeAtPixel(
    const cv::Mat& img, int x, int y, double* mag, double* angle)
{
  if(y > 0 && y < img.rows-1 && x > 0  &&  x < img.cols-1 )
  {
    double dx = img.at<uint8_t>(y, x+1) - img.at<uint8_t>(y, x-1);
    double dy = img.at<uint8_t>(y+1, x) - img.at<uint8_t>(y-1, x);
    *mag = std::sqrt( dx*dx + dy*dy );
    *angle = std::atan2( dy, dx );
    return true;
  }
  return false;
}

// 0x1800ADA80 (upstream-identical apart from the float histogram)
void smoothOrientationHistogram(AngleHistogram& hist)
{
  double prev = hist[n_bins-1], h0 = hist[0];
  for(size_t i = 0; i < n_bins; ++i)
  {
    double tmp = hist[i];
    hist[i] = 0.25 * prev + 0.5 * hist[i] + 0.25 * ( (i+1 == n_bins) ? h0 : hist[i+1]);
    prev = tmp;
  }
}

// inlined into 0x1800AD4A0
double getDominantAngle(const AngleHistogram& hist)
{
  float max_angle;                               // Pimax: float compare (comiss)
  int max_bin;

  max_angle = hist[0];
  max_bin = 0;
  for(size_t i = 1; i < n_bins; i++ )
  {
    if(hist[i] > max_angle)
    {
      max_angle = hist[i];
      max_bin = i;
    }
  }
  return max_bin * 2.0 * M_PI / n_bins;
}

} // namespace angle_hist

} // namespace feature_detection_utils
} // namespace totem
} // namespace pimax
