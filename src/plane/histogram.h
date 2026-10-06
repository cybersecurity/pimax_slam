// pimax_slam.pi.dll -- src/plane/histogram.h  (TODO(verify) path: Kimera-VIO utils/Histogram.h)
// Reconciled from c16 (ctor 0x180197940, default ctor 0x180197BB0, dtor 0x180197C20, operator=
// 0x180197CC0, vector<PeakInfo>::_Emplace_reallocate 0x1801976B0) and c17 (calculateHistogram,
// findPeaks, getLocalMaximum1D).  histogram.obj sits right before src/plane/mesh.cpp.
// c16 additionally found: mask taken BY VALUE, ranges_ entries (new float[2]) are freed with
// scalar delete, operator= does not copy histogram_ and sizes the copies by dims_.
//
// Pimax changes vs. Kimera-VIO (2019/2020):
//   * no CHECKs / VLOGs / LOG(FATAL) anywhere in the three functions of chunk c17,
//   * dims_ is 64-bit (loaded as qword, printed with the unsigned std::to_string path),
//   * two extra cv::Mat members after histogram_ (+0xF0, +0x150; only seen in ctor/dtor),
//   * calculateHistogram uses non-static range arrays,
//   * findPeaks computes the slope as src(i) - src(i - size) (Kimera: src(i + size) - src(i - size))
//     and treats a falling slope after a flat one (pre 0 -> cur 1) like the end of an up-hill,
//   * getLocalMaximum1D has a different signature (reference value + first-frame flag) and a
//     different acceptance rule; drawing/logging removed.
#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace pimax {
namespace totem {

// sizeof == 0x1B0 (432): Mesher::z_hist_ at Mesher+0x2A0, temporary of 432 bytes in the Mesher ctor.
class Histogram {
 public:
  Histogram();                                            // 0x180197BB0
  Histogram(int n_images, const std::vector<int>& channels, cv::Mat mask, int dims,
            const std::vector<int>& hist_size, const std::vector<std::array<float, 2>>& ranges,
            bool uniform = true, bool accumulate = false);  // 0x180197940
  ~Histogram();                                           // 0x180197C20
  Histogram& operator=(const Histogram& other);           // 0x180197CC0

  // 24 bytes (vector stride 24).
  struct PeakInfo {
    int pos_ = 0;          // +0
    int left_size_ = 0;    // +4
    int right_size_ = 0;   // +8
    float support_ = 0;    // +12
    double value_ = 0;     // +16
    bool operator<(const PeakInfo& rhd) const { return (support_ < rhd.support_); }
    bool operator==(const PeakInfo& rhd) const {
      return (support_ == rhd.support_ && pos_ == rhd.pos_);
    }
  };

  // 0x180197E70
  void calculateHistogram(const cv::Mat& input, bool log_histogram = false);

  // 0x180198960  (Pimax signature)
  std::vector<PeakInfo> getLocalMaximum1D(const cv::Size& smooth_size, int window_size,
                                          float peak_per, float min_support,
                                          float reference_value, bool is_first_frame) const;

 private:
  struct Length {
    int pos1 = 0;
    int pos2 = 0;
    int size() const { return pos2 - pos1 + 1; }
  };

  // inlined into findPeaks
  PeakInfo peakInfo(int pos, int left_size, int right_size, float support) const;
  // 0x180198460
  std::vector<PeakInfo> findPeaks(cv::InputArray _src, int window_size) const;

  int n_images_;            // +0x00
  int* channels_;           // +0x08
  cv::Mat mask_;            // +0x10
  size_t dims_;             // +0x70  (qword; TODO(verify) declared type)
  int* hist_size_;          // +0x78
  const float** ranges_;    // +0x80
  bool uniform_;            // +0x88
  bool accumulate_;         // +0x89
  cv::Mat histogram_;       // +0x90
  cv::Mat unknown_f0_;      // +0xF0  TODO(verify) (Kimera has no such member)
  cv::Mat unknown_150_;     // +0x150 TODO(verify)

  friend struct HistogramLayout;
};

struct HistogramLayout {
  static_assert(sizeof(Histogram) == 0x1B0, "sizeof(Histogram)");
  static_assert(sizeof(Histogram::PeakInfo) == 24, "sizeof(Histogram::PeakInfo)");
  static_assert(offsetof(Histogram, channels_) == 0x08, "Histogram::channels_");
  static_assert(offsetof(Histogram, mask_) == 0x10, "Histogram::mask_");
  static_assert(offsetof(Histogram, dims_) == 0x70, "Histogram::dims_");
  static_assert(offsetof(Histogram, hist_size_) == 0x78, "Histogram::hist_size_");
  static_assert(offsetof(Histogram, ranges_) == 0x80, "Histogram::ranges_");
  static_assert(offsetof(Histogram, uniform_) == 0x88, "Histogram::uniform_");
  static_assert(offsetof(Histogram, accumulate_) == 0x89, "Histogram::accumulate_");
  static_assert(offsetof(Histogram, histogram_) == 0x90, "Histogram::histogram_");
  static_assert(offsetof(Histogram, unknown_f0_) == 0xF0, "Histogram::unknown_f0_");
  static_assert(offsetof(Histogram, unknown_150_) == 0x150, "Histogram::unknown_150_");
};

}  // namespace totem
}  // namespace pimax
