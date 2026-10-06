// src/sensor_fusion/filters.h (TODO(verify) path) -- identical to LedObjectPoseEstimator
// ctrl_three_dof/filters.h: ports of Cardboard SDK sensors/{lowpass_filter,mean_filter,
// median_filter}.  All out-of-line in the binary (addresses below).
// Compiler-generated dtors: ~MeanFilter 0x1801A45B0, ~MedianFilter 0x1801A45C0,
// std::deque<Vector3>::~deque 0x1801A4450, deque<Vector3>::_Growmap 0x1801A52D0.
#pragma once

#include <algorithm>
#include <cstdint>
#include <deque>
#include <vector>
#include "sensor_fusion/vector.h"

namespace pimax {
namespace ThreeDof {

// sizeof == 48
//   +0  double time_constant_
//   +8  uint64 timestamp_most_recent_update_ns_
//   +16 bool   initialized_
//   +24 Vector3 filtered_data_
class LowpassFilter {
 public:
  explicit LowpassFilter(double cutoff_freq_hz);  // 0x1801A4EB0
  void AddSample(const Vector3& sample, uint64_t timestamp_ns);  // 0x1801A4EE0
  void AddWeightedSample(const Vector3& sample, uint64_t timestamp_ns,
                         double weight);  // 0x1801A4FC0
  uint64_t GetMostRecentTimestampNs() const { return timestamp_most_recent_update_ns_; }
  const Vector3& GetFilteredData() const { return filtered_data_; }
  bool IsInitialized() const { return initialized_; }
  void Reset();  // 0x1801A50A0

 private:
  const double time_constant_;
  uint64_t timestamp_most_recent_update_ns_;
  bool initialized_;
  Vector3 filtered_data_;
};

// sizeof == 48: size_t filter_size_; std::deque<Vector3> buffer_;
class MeanFilter {
 public:
  explicit MeanFilter(size_t filter_size);  // 0x1801A50C0
  void AddSample(const Vector3& sample);    // 0x1801A5120
  bool IsValid() const { return buffer_.size() == filter_size_; }
  Vector3 GetFilteredData() const;          // 0x1801A51F0

 private:
  const size_t filter_size_;
  std::deque<Vector3> buffer_;
};

// sizeof == 88: size_t filter_size_; std::deque<Vector3> buffer_;
//               std::deque<float> norms_;
class MedianFilter {
 public:
  explicit MedianFilter(size_t filter_size);  // 0x1801A54B0
  void AddSample(const Vector3& sample);      // 0x1801A5540
  bool IsValid() const;                       // 0x1801A5A00
  Vector3 GetFilteredData() const;            // 0x1801A56E0

 private:
  const size_t filter_size_;
  std::deque<Vector3> buffer_;
  std::deque<float> norms_;
};

static_assert(sizeof(LowpassFilter) == 48, "sizeof(LowpassFilter)");
static_assert(sizeof(MeanFilter) == 48, "sizeof(MeanFilter)");
static_assert(sizeof(MedianFilter) == 88, "sizeof(MedianFilter)");

}  // namespace ThreeDof
}  // namespace pimax
