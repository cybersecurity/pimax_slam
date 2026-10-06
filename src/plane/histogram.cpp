// pimax_slam.pi.dll -- src/plane/histogram.cpp  (TODO(verify) path)
// Kimera-VIO utils/Histogram.cpp, Pimax-modified.  histogram.obj: 0x1801976B0 .. 0x180198C90.
// First four functions from the c16 draft (ctor/dtor/operator=), the rest from the c17 draft.
#include "plane/histogram.h"

#include <cmath>
#include <string>

namespace pimax {
namespace totem {

// 0x180197bb0
Histogram::Histogram()
  : n_images_(0),
    channels_(nullptr),
    mask_(),
    dims_(0),
    hist_size_(nullptr),
    ranges_(nullptr),
    uniform_(false),
    accumulate_(false),
    histogram_(),
    unknown_f0_(),
    unknown_150_()
{
}

// 0x180197940
Histogram::Histogram(int n_images, const std::vector<int>& channels, cv::Mat mask, int dims,
                     const std::vector<int>& hist_size,
                     const std::vector<std::array<float, 2>>& ranges, bool uniform,
                     bool accumulate)
  : n_images_(n_images),
    channels_(nullptr),
    mask_(mask),
    dims_(dims),
    hist_size_(nullptr),
    ranges_(nullptr),
    uniform_(uniform),
    accumulate_(accumulate),
    histogram_(),
    unknown_f0_(),
    unknown_150_()
{
  channels_ = new int[channels.size()];
  for (size_t i = 0; i < channels.size(); i++)
  {
    channels_[i] = channels.at(i);
  }
  hist_size_ = new int[hist_size.size()];
  for (size_t i = 0; i < hist_size.size(); i++)
  {
    hist_size_[i] = hist_size.at(i);
  }
  ranges_ = new const float*[ranges.size()];
  for (size_t i = 0; i < ranges.size(); i++)
  {
    float* range = new float[2];
    range[0] = ranges.at(i).at(0);
    range[1] = ranges.at(i).at(1);
    ranges_[i] = range;
  }
}

// 0x180197c20
Histogram::~Histogram()
{
  delete[] channels_;
  delete[] hist_size_;
  for (size_t i = 0; i < dims_; i++)
  {
    delete ranges_[i];   // scalar delete (sized 4) on a new float[2] -- as in the binary
  }
  delete[] ranges_;
}

// 0x180197cc0  (histogram_ and the two other Mats are NOT copied; channels_ is sized by dims_)
Histogram& Histogram::operator=(const Histogram& other)
{
  delete[] channels_;
  delete[] hist_size_;
  for (size_t i = 0; i < dims_; i++)
  {
    delete ranges_[i];
  }
  delete[] ranges_;

  dims_ = other.dims_;
  int* channels = new int[dims_];
  for (size_t j = 0; j < dims_; j++)
  {
    channels[j] = other.channels_[j];
  }
  int* hist_size = new int[dims_];
  for (size_t k = 0; k < dims_; k++)
  {
    hist_size[k] = other.hist_size_[k];
  }
  const float** ranges = new const float*[dims_];
  for (size_t i = 0; i < dims_; i++)
  {
    float* range = new float[2];
    range[0] = other.ranges_[i][0];
    range[1] = other.ranges_[i][1];
    ranges[i] = range;
  }

  n_images_ = other.n_images_;
  channels_ = channels;
  mask_ = other.mask_;
  hist_size_ = hist_size;
  ranges_ = ranges;
  uniform_ = other.uniform_;
  accumulate_ = other.accumulate_;
  return *this;
}

/* -------------------------------------------------------------------------- */
// 0x180197E70
// upstream-modified: no LOG(FATAL) for dims_ not in {1,2}; range arrays are not static.
void Histogram::calculateHistogram(const cv::Mat& input, bool log_histogram) {
  if (dims_ == 1) {
    const float* range_hist[] = {ranges_[0]};
    cv::calcHist(&input, n_images_, channels_, mask_, histogram_, static_cast<int>(dims_),
                 hist_size_, range_hist, uniform_, accumulate_);
  } else if (dims_ == 2) {
    const float* range_hist[] = {ranges_[0], ranges_[1]};
    cv::calcHist(&input, n_images_, channels_, mask_, histogram_, static_cast<int>(dims_),
                 hist_size_, range_hist, uniform_, accumulate_);
  }

  if (log_histogram) {
    cv::FileStorage file("histogram_" + std::to_string(dims_) + ".yaml",
                         cv::FileStorage::WRITE);
    file << "Histogram";
    file << histogram_;
  }
}

/* -------------------------------------------------------------------------- */
// inlined into findPeaks (0x180198460); Kimera's CHECK_EQ(dims_, 1) removed.
Histogram::PeakInfo Histogram::peakInfo(int pos, int left_size, int right_size,
                                        float support) const {
  PeakInfo output;
  output.pos_ = pos;
  output.left_size_ = left_size;
  output.right_size_ = right_size;
  output.support_ = support;
  // float arithmetic, widened to double on store
  output.value_ =
      (pos * (ranges_[0][1] - ranges_[0][0]) / hist_size_[0]) + ranges_[0][0];
  return output;
}

/* -------------------------------------------------------------------------- */
// 0x180198460
// upstream-modified: slope uses src2(i) - src2(i - size) (Kimera: src2(i + size) - ...);
// the "end of up-hill" transition also fires for a flat -> falling slope (pre 0 -> cur 1).
std::vector<Histogram::PeakInfo> Histogram::findPeaks(cv::InputArray _src,
                                                      int window_size) const {
  cv::Mat src = _src.getMat();

  cv::Mat slope_mat = src.clone();

  // Transform initial matrix into 1channel, and 1 row matrix
  cv::Mat src2 = src.reshape(1, 1);

  int size = window_size / 2;

  Length up_hill, down_hill;
  std::vector<PeakInfo> output;

  int pre_state = 0;
  int i = size;

  while (i < src2.cols - size) {
    float cur_state = src2.at<float>(i) - src2.at<float>(i - size);

    if (cur_state > 0)
      cur_state = 2;
    else if (cur_state < 0)
      cur_state = 1;
    else
      cur_state = 0;

    // In case you want to check how the slope looks like
    slope_mat.at<float>(i) = cur_state;

    if (pre_state == 0 && cur_state == 2) {
      up_hill.pos1 = i;
    } else if ((pre_state == 2 || pre_state == 0) && cur_state == 1) {
      // TODO(verify) source form; the binary takes this branch for pre_state 0 and 2.
      up_hill.pos2 = i - 1;
      down_hill.pos1 = i;
    }

    if ((pre_state == 1 && cur_state == 2) || (pre_state == 1 && cur_state == 0)) {
      down_hill.pos2 = i - 1;
      int max_pos = up_hill.pos2;
      if (src2.at<float>(up_hill.pos2) < src2.at<float>(down_hill.pos1)) {
        max_pos = down_hill.pos1;
      }

      PeakInfo peak_info = peakInfo(max_pos, up_hill.size(), down_hill.size(),
                                    src2.at<float>(max_pos));

      output.push_back(peak_info);
    }
    i++;
    pre_state = (int)cur_state;
  }
  return output;
}

/* -------------------------------------------------------------------------- */
// 0x180198960
// upstream-modified: new parameters `reference_value` (the camera height, Mesher t_w_c_.z) and
// `is_first_frame`; peaks below min_support are still accepted when their support exceeds
// 10 and (first frame or |value - reference| > 1.0).  No CHECKs, no logging, no drawing.
std::vector<Histogram::PeakInfo> Histogram::getLocalMaximum1D(
    const cv::Size& smooth_size, int window_size, float peak_per, float min_support,
    float reference_value, bool is_first_frame) const {
  cv::Mat src = histogram_.clone();

  std::vector<PeakInfo> output;
  cv::GaussianBlur(src, src, smooth_size, 0);

  std::vector<PeakInfo> peaks = findPeaks(src, window_size);

  double min_val, max_val;
  cv::minMaxLoc(src, &min_val, &max_val);

  for (size_t i = 0; i < peaks.size(); i++) {
    if (peaks[i].support_ > max_val * peak_per &&
        (peaks[i].support_ > min_support ||
         ((is_first_frame || std::fabs(peaks[i].value_ - reference_value) > 1.0) &&
          peaks[i].support_ > 10.0f)) &&
        peaks[i].left_size_ >= 2 && peaks[i].right_size_ >= 2) {
      output.push_back(peaks[i]);
    }
  }
  return output;
}

}  // namespace totem
}  // namespace pimax
