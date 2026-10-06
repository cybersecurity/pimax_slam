// src/sensor_fusion/gyroscope_bias_estimator.cpp (TODO(verify) path)
// Port of Cardboard SDK sensors/gyroscope_bias_estimator.cc (same as LedObjectPoseEstimator
// ctrl_three_dof/gyroscope_bias_estimator.cpp except the 1000.0f threshold).
// pimax changes vs. upstream (all visible in the binary):
//   * thresholds stored as float literals (0.03f, 0.15f, 0.3f, 0.00095f),
//   * accumulated bias weight uses weight^2 (both for the filter and the sum),
//   * kMinSumOfWeightsGyroBiasThreshold == 1000 (upstream 25; LedObjectPoseEstimator 250),
//   * IsCurrentEstimateValid() still computes the "correlated with simulated
//     gyro" check but does NOT use it (the Dot() calls remain in the binary,
//     the sqrt/compare were dead-code eliminated),
//   * the simulated angular velocity is rounded through float,
//   * MedianFilter has no zero fallback (see filters.cpp).
#include "sensor_fusion/gyroscope_bias_estimator.h"

#include <algorithm>
#include <cmath>
#include "sensor_fusion/rotation.h"

namespace pimax {
namespace ThreeDof {

namespace {
const float kAccelerometerLowPassCutOffFrequencyHz = 1.0f;                      // 0x1803ADDD0
const float kRotationVelocityBasedAccelerometerLowPassCutOffFrequencyHz = 0.15f;  // 0x1803BF4F0
const float kGyroscopeLowPassCutOffFrequencyHz = 1.0f;
const float kGyroscopeBiasLowPassCutOffFrequencyHz = 0.15f;

const double kEpsilon = 1.0e-8;
const int kFilterWindowSize = 5;
const double kRatioBetweenGyroBiasAndAccel = 1.5;
// 1000.0f (0x1803BF4F8) -- upstream Cardboard 25.0f, LedObjectPoseEstimator 250.0f
const float kMinSumOfWeightsGyroBiasThreshold = 1000.0f;
const double kAccelerometerDeltaStaticThreshold = 0.5;      // 0x1803AF2B8
// float literal widened: 0.029999999329447746 (0x1803BF4E8)
const float kGyroscopeDeltaStaticThreshold = 0.03f;
const float kGyroscopeForBiasThreshold = 0.30f;             // 0x1803BF4E0 (float compare)
const int kStaticFrameDetectionCountThreshold = 50;
const double kMinTimestep = 1;                              // ns; 0x1803ADDD0
}  // namespace

// 0x1801A4300
GyroscopeBiasEstimator::GyroscopeBiasEstimator()
    : accelerometer_lowpass_filter_(kAccelerometerLowPassCutOffFrequencyHz),
      simulated_gyroscope_from_accelerometer_lowpass_filter_(
          kRotationVelocityBasedAccelerometerLowPassCutOffFrequencyHz),
      gyroscope_lowpass_filter_(kGyroscopeLowPassCutOffFrequencyHz),
      gyroscope_bias_lowpass_filter_(kGyroscopeBiasLowPassCutOffFrequencyHz),
      accelerometer_static_counter_(new IsStaticCounter(kStaticFrameDetectionCountThreshold)),
      gyroscope_static_counter_(new IsStaticCounter(kStaticFrameDetectionCountThreshold)),
      current_accumulated_weights_gyroscope_bias_(0.f),
      mean_filter_(kFilterWindowSize),
      median_filter_(kFilterWindowSize),
      last_mean_filtered_accelerometer_value_(0, 0, 0) {
  Reset();
}

// 0x1801A4540
GyroscopeBiasEstimator::~GyroscopeBiasEstimator() {}

// 0x1801A4E60 (out of line in this image: also called by ThreeDofTracker's ctor; inlined in
// the GyroscopeBiasEstimator ctor)
void GyroscopeBiasEstimator::Reset() {
  accelerometer_lowpass_filter_.Reset();
  gyroscope_lowpass_filter_.Reset();
  gyroscope_bias_lowpass_filter_.Reset();
  accelerometer_static_counter_->Reset();
  gyroscope_static_counter_->Reset();
}

// 0x1801A4CC0
void GyroscopeBiasEstimator::ProcessGyroscope(const Vector3& gyroscope_sample,
                                              uint64_t timestamp_ns) {
  gyroscope_lowpass_filter_.AddSample(gyroscope_sample, timestamp_ns);

  const Vector3 smoothed_gyroscope_delta =
      gyroscope_sample - gyroscope_lowpass_filter_.GetFilteredData();

  gyroscope_static_counter_->AppendFrame(Length(smoothed_gyroscope_delta) <
                                         kGyroscopeDeltaStaticThreshold);

  if (gyroscope_static_counter_->IsRecentlyStatic() &&
      accelerometer_static_counter_->IsRecentlyStatic()) {
    if (!UpdateGyroscopeBias(gyroscope_sample, timestamp_ns)) {
      gyroscope_static_counter_->AppendFrame(false);
    }
  } else {
    current_accumulated_weights_gyroscope_bias_ = 0.0f;
  }
}

// 0x1801A4980
void GyroscopeBiasEstimator::ProcessAccelerometer(const Vector3& accelerometer_sample,
                                                  uint64_t timestamp_ns) {
  const uint64_t previous_accel_timestamp_ns =
      accelerometer_lowpass_filter_.GetMostRecentTimestampNs();
  const bool is_low_pass_filter_init = accelerometer_lowpass_filter_.IsInitialized();

  accelerometer_lowpass_filter_.AddSample(accelerometer_sample, timestamp_ns);

  const Vector3 smoothed_accelerometer_delta =
      accelerometer_sample - accelerometer_lowpass_filter_.GetFilteredData();

  accelerometer_static_counter_->AppendFrame(Length(smoothed_accelerometer_delta) <
                                             kAccelerometerDeltaStaticThreshold);

  if (!is_low_pass_filter_init) {
    simulated_gyroscope_from_accelerometer_lowpass_filter_.AddSample({0, 0, 0},
                                                                      timestamp_ns);
    return;
  }

  if (!accelerometer_static_counter_->IsRecentlyStatic()) {
    return;
  }

  median_filter_.AddSample(accelerometer_lowpass_filter_.GetFilteredData());

  if (!median_filter_.IsValid()) {
    mean_filter_.AddSample(accelerometer_lowpass_filter_.GetFilteredData());
    last_mean_filtered_accelerometer_value_ =
        accelerometer_lowpass_filter_.GetFilteredData();
    return;
  }

  mean_filter_.AddSample(median_filter_.GetFilteredData());

  const int64_t diff = timestamp_ns - previous_accel_timestamp_ns;  // cvtsi2sd (signed)
  const double timestep = static_cast<double>(diff);

  simulated_gyroscope_from_accelerometer_lowpass_filter_.AddSample(
      ComputeAngularVelocityFromLatestAccelerometer(timestep), timestamp_ns);
  last_mean_filtered_accelerometer_value_ = mean_filter_.GetFilteredData();
}

// inlined into 0x1801A4980
Vector3 GyroscopeBiasEstimator::ComputeAngularVelocityFromLatestAccelerometer(
    double timestep) const {
  if (timestep < kMinTimestep) {
    return {0, 0, 0};
  }

  const Vector3 mean_of_median = mean_filter_.GetFilteredData();

  const Rotation incremental_rotation =
      Rotation::RotateInto(last_mean_filtered_accelerometer_value_, mean_of_median);

  Vector3 incremental_rotation_axis;
  double incremental_rotation_angle;
  incremental_rotation.GetAxisAndAngle(&incremental_rotation_axis,
                                       &incremental_rotation_angle);

  incremental_rotation_axis *= incremental_rotation_angle / timestep;

  // QUIRK: the binary rounds each component through float (cvtpd2ps/cvtps2pd)
  // before feeding the low-pass filter.  TODO(verify) exact source form
  // (e.g. a float vector type in the return path).
  return Vector3(static_cast<float>(incremental_rotation_axis[0]),
                 static_cast<float>(incremental_rotation_axis[1]),
                 static_cast<float>(incremental_rotation_axis[2]));
}

// inlined into 0x1801A4CC0
bool GyroscopeBiasEstimator::UpdateGyroscopeBias(const Vector3& gyroscope_sample,
                                                 uint64_t timestamp_ns) {
  const float gyroscope_norm = static_cast<float>(Length(gyroscope_sample));
  if (gyroscope_norm >= kGyroscopeForBiasThreshold) {
    return false;
  }

  const float gyroscope_weight =
      std::max(0.0f, 1.0f - gyroscope_norm / kGyroscopeForBiasThreshold);
  const float gyroscope_weight_sq = gyroscope_weight * gyroscope_weight;  // pimax: squared
  gyroscope_bias_lowpass_filter_.AddWeightedSample(
      gyroscope_lowpass_filter_.GetFilteredData(), timestamp_ns, gyroscope_weight_sq);
  current_accumulated_weights_gyroscope_bias_ += gyroscope_weight_sq;
  return true;
}

// 0x1801A4760
Vector3 GyroscopeBiasEstimator::GetGyroscopeBias() const {
  return gyroscope_bias_lowpass_filter_.GetFilteredData();
}

// 0x1801A4780
bool GyroscopeBiasEstimator::IsCurrentEstimateValid() const {
  const Vector3 current_gravity_dir = Normalized(last_mean_filtered_accelerometer_value_);
  const Vector3 gyro_bias_lowpass = gyroscope_bias_lowpass_filter_.GetFilteredData();

  const Vector3 off_gravity_gyro_bias =
      gyro_bias_lowpass -
      current_gravity_dir * Dot(gyro_bias_lowpass, current_gravity_dir);

  const Vector3 gyro_from_accel =
      simulated_gyroscope_from_accelerometer_lowpass_filter_.GetFilteredData();
  // QUIRK: computed but unused (binary keeps only the two Dot() calls).
  const bool isGyroscopeBiasCorrelatedWithSimulatedGyro =
      (Length(gyro_from_accel) * kRatioBetweenGyroBiasAndAccel >
       (Length(off_gravity_gyro_bias) + kEpsilon));
  (void)isGyroscopeBiasCorrelatedWithSimulatedGyro;

  const bool hasEnoughSamples =
      current_accumulated_weights_gyroscope_bias_ > kMinSumOfWeightsGyroBiasThreshold;
  const bool areCountersStatic = gyroscope_static_counter_->IsRecentlyStatic() &&
                                 accelerometer_static_counter_->IsRecentlyStatic();

  const bool isStatic = hasEnoughSamples && areCountersStatic;
  return isStatic;
}

}  // namespace ThreeDof
}  // namespace pimax
