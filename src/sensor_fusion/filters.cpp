#define _USE_MATH_DEFINES
#include "sensor_fusion/filters.h"

#include <cmath>

namespace pimax {
namespace ThreeDof {

namespace {
const double kSecondsFromNanoseconds = 1.0e-9;       // 0x1803ADD98
// NOTE: float literal widened to double: 0.0009500000160187483 (0x1803BF500)
const double kMinTimestepS = 0.00095f;
const double kMaxTimestepS = 1.00f;                   // 1.0 (0x1803ADDD0)
}  // namespace

// 0x1801A4EB0
LowpassFilter::LowpassFilter(double cutoff_freq_hz)
    : time_constant_(1 / (2 * M_PI * cutoff_freq_hz)),  // 1.0 / (f * 6.283185307179586)
      timestamp_most_recent_update_ns_(0),
      initialized_(false),
      filtered_data_() {}

// 0x1801A4EE0  (AddWeightedSample(sample, ts, 1.0) inlined/folded)
void LowpassFilter::AddSample(const Vector3& sample, uint64_t timestamp_ns) {
  AddWeightedSample(sample, timestamp_ns, 1.0);
}

// 0x1801A4FC0
void LowpassFilter::AddWeightedSample(const Vector3& sample, uint64_t timestamp_ns,
                                      double weight) {
  if (!initialized_) {
    filtered_data_ = sample;
    timestamp_most_recent_update_ns_ = timestamp_ns;
    initialized_ = true;
    return;
  }
  if (timestamp_ns < timestamp_most_recent_update_ns_) {
    timestamp_most_recent_update_ns_ = timestamp_ns;
    return;
  }
  const double delta_s =
      static_cast<double>(timestamp_ns - timestamp_most_recent_update_ns_) *
      kSecondsFromNanoseconds;
  if (delta_s <= kMinTimestepS || delta_s > kMaxTimestepS) {
    timestamp_most_recent_update_ns_ = timestamp_ns;
    return;
  }
  const double weighted_delta_secs = weight * delta_s;
  const double alpha = weighted_delta_secs / (time_constant_ + weighted_delta_secs);
  for (int i = 0; i < 3; ++i) {
    filtered_data_[i] = (1 - alpha) * filtered_data_[i] + alpha * sample[i];
  }
  timestamp_most_recent_update_ns_ = timestamp_ns;
}

// 0x1801A50A0
void LowpassFilter::Reset() {
  initialized_ = false;
  filtered_data_ = {0, 0, 0};
}

// 0x1801A50C0
MeanFilter::MeanFilter(size_t filter_size) : filter_size_(filter_size) {}

// 0x1801A5120
void MeanFilter::AddSample(const Vector3& sample) {
  buffer_.push_back(sample);
  if (buffer_.size() > filter_size_) {
    buffer_.pop_front();
  }
}

// 0x1801A51F0
Vector3 MeanFilter::GetFilteredData() const {
  Vector3 mean;
  for (const Vector3& sample : buffer_) {
    mean += sample;
  }
  return mean / static_cast<double>(filter_size_);
}

// 0x1801A54B0
MedianFilter::MedianFilter(size_t filter_size) : filter_size_(filter_size) {}

// 0x1801A5540
void MedianFilter::AddSample(const Vector3& sample) {
  buffer_.push_back(sample);
  norms_.push_back(static_cast<float>(Length(sample)));
  if (buffer_.size() > filter_size_) {
    buffer_.pop_front();
    norms_.pop_front();
  }
}

// 0x1801A5A00
bool MedianFilter::IsValid() const { return buffer_.size() == filter_size_; }

// 0x1801A56E0
// QUIRK (differs from upstream Cardboard): there is no "return zero" fallback;
// the binary looks the median norm up with an iterator walk over norms_ and
// indexes buffer_ with the resulting distance unconditionally (would read
// buffer_[size()] if not found -- impossible in practice since the median is
// one of the values).
Vector3 MedianFilter::GetFilteredData() const {
  std::vector<float> norms(norms_.begin(), norms_.end());
  std::nth_element(norms.begin(), norms.begin() + filter_size_ / 2, norms.end());
  const float median_norm = norms[filter_size_ / 2];
  // TODO(verify): exact source form of the search (std::find vs manual loop).
  const auto it = std::find(norms_.begin(), norms_.end(), median_norm);
  return buffer_[it - norms_.begin()];
}

}  // namespace ThreeDof
}  // namespace pimax
