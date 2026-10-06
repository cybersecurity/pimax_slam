// src/sensor_fusion/gyroscope_bias_estimator.h (TODO(verify) path)
// pimax::ThreeDof::GyroscopeBiasEstimator -- same code as LedObjectPoseEstimator's
// pimax::CtrlThreeDof::GyroscopeBiasEstimator (Cardboard SDK port) except
// kMinSumOfWeightsGyroBiasThreshold (1000 here, 250 there).  vtable 0x1803BF4A8 (7 slots):
//   [0] 0x1801A4690 scalar deleting dtor   (~ = 0x1801A4540)
//   [1] 0x1801A4CC0 ProcessGyroscope(const Vector3&, uint64_t)
//   [2] 0x1801A4C60 ProcessGyroscope(const double&, const double&, const double&, uint64_t)
//   [3] 0x1801A4980 ProcessAccelerometer(const Vector3&, uint64_t)
//   [4] 0x1801A4920 ProcessAccelerometer(const double&, const double&, const double&, uint64_t)
//   [5] 0x1801A4760 GetGyroscopeBias() const -> Vector3
//   [6] 0x1801A4780 IsCurrentEstimateValid() const
// MSVC places overloads adjacently in *reverse* declaration order, so the
// (x,y,z) overloads are declared first.  TODO(verify) the overload order.
//
// Layout (sizeof == 384):
//   +0   vptr
//   +8   LowpassFilter accelerometer_lowpass_filter_                          (1.0 Hz)
//   +56  LowpassFilter simulated_gyroscope_from_accelerometer_lowpass_filter_ (0.15f Hz)
//   +104 LowpassFilter gyroscope_lowpass_filter_                              (1.0 Hz)
//   +152 LowpassFilter gyroscope_bias_lowpass_filter_                         (0.15f Hz)
//   +200 std::unique_ptr<IsStaticCounter> accelerometer_static_counter_
//   +208 std::unique_ptr<IsStaticCounter> gyroscope_static_counter_
//   +216 float current_accumulated_weights_gyroscope_bias_
//   +224 MeanFilter   mean_filter_   (5)
//   +272 MedianFilter median_filter_ (5)
//   +360 Vector3 last_mean_filtered_accelerometer_value_
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include "sensor_fusion/filters.h"
#include "sensor_fusion/vector.h"

namespace pimax {
namespace ThreeDof {

class GyroscopeBiasEstimator {
 public:
  GyroscopeBiasEstimator();             // 0x1801A4300
  virtual ~GyroscopeBiasEstimator();    // 0x1801A4540

  // 0x1801A4C60
  virtual void ProcessGyroscope(const double& x, const double& y, const double& z,
                                uint64_t timestamp_ns) {
    ProcessGyroscope(Vector3(x, y, z), timestamp_ns);
  }
  // 0x1801A4CC0
  virtual void ProcessGyroscope(const Vector3& gyroscope_sample, uint64_t timestamp_ns);

  // 0x1801A4920
  virtual void ProcessAccelerometer(const double& x, const double& y, const double& z,
                                    uint64_t timestamp_ns) {
    ProcessAccelerometer(Vector3(x, y, z), timestamp_ns);
  }
  // 0x1801A4980
  virtual void ProcessAccelerometer(const Vector3& accelerometer_sample,
                                    uint64_t timestamp_ns);

  // 0x1801A4760
  virtual Vector3 GetGyroscopeBias() const;
  // 0x1801A4780
  virtual bool IsCurrentEstimateValid() const;

  // 0x1801A4710 (non-virtual; emitted out of line here, called by ThreeDofTracker)
  void GetGyroscopeBias(double& x, double& y, double& z) const {
    const Vector3& b = gyroscope_bias_lowpass_filter_.GetFilteredData();
    x = b[0];
    y = b[1];
    z = b[2];
  }

  void Reset();  // 0x1801A4E60 (inlined into the ctor)

 private:
  class IsStaticCounter {
   public:
    explicit IsStaticCounter(int min_static_frames_threshold)
        : min_static_frames_threshold_(min_static_frames_threshold),
          consecutive_static_frames_(0) {}
    void AppendFrame(bool is_static) {
      if (is_static)
        ++consecutive_static_frames_;
      else
        consecutive_static_frames_ = 0;
    }
    bool IsRecentlyStatic() const {
      return consecutive_static_frames_ >= min_static_frames_threshold_;
    }
    void Reset() { consecutive_static_frames_ = 0; }

   private:
    const int min_static_frames_threshold_;
    int consecutive_static_frames_;
  };

  Vector3 ComputeAngularVelocityFromLatestAccelerometer(double timestep) const;
  bool UpdateGyroscopeBias(const Vector3& gyroscope_sample, uint64_t timestamp_ns);

  LowpassFilter accelerometer_lowpass_filter_;
  LowpassFilter simulated_gyroscope_from_accelerometer_lowpass_filter_;
  LowpassFilter gyroscope_lowpass_filter_;
  LowpassFilter gyroscope_bias_lowpass_filter_;
  std::unique_ptr<IsStaticCounter> accelerometer_static_counter_;
  std::unique_ptr<IsStaticCounter> gyroscope_static_counter_;
  float current_accumulated_weights_gyroscope_bias_;
  MeanFilter mean_filter_;
  MedianFilter median_filter_;
  Vector3 last_mean_filtered_accelerometer_value_;

  friend struct GyroscopeBiasEstimatorLayout;
};

static_assert(sizeof(GyroscopeBiasEstimator) == 384, "GyroscopeBiasEstimator size");
struct GyroscopeBiasEstimatorLayout {
  static_assert(offsetof(GyroscopeBiasEstimator, accelerometer_lowpass_filter_) == 8, "GBE +8");
  static_assert(offsetof(GyroscopeBiasEstimator, simulated_gyroscope_from_accelerometer_lowpass_filter_) == 56, "GBE +56");
  static_assert(offsetof(GyroscopeBiasEstimator, gyroscope_lowpass_filter_) == 104, "GBE +104");
  static_assert(offsetof(GyroscopeBiasEstimator, gyroscope_bias_lowpass_filter_) == 152, "GBE +152");
  static_assert(offsetof(GyroscopeBiasEstimator, accelerometer_static_counter_) == 200, "GBE +200");
  static_assert(offsetof(GyroscopeBiasEstimator, current_accumulated_weights_gyroscope_bias_) == 216, "GBE +216");
  static_assert(offsetof(GyroscopeBiasEstimator, mean_filter_) == 224, "GBE +224");
  static_assert(offsetof(GyroscopeBiasEstimator, median_filter_) == 272, "GBE +272");
  static_assert(offsetof(GyroscopeBiasEstimator, last_mean_filtered_accelerometer_value_) == 360, "GBE +360");
};

}  // namespace ThreeDof
}  // namespace pimax
