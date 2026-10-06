// src/sensor_fusion/three_dof_tracker.h (TODO(verify) path and class name: no RTTI/vtable)
//
// Headset 3-DoF (orientation-only) fallback tracker, owned by SLAMManager+2832
// (std::unique_ptr, `new` of 0x2A0 bytes + ctor 0x1801A6AC0 in 0x1801663F0).
// Pimax-new: unrelated to LedObjectPoseEstimator's ThreeDofTracker apart from reusing
// GyroscopeBiasEstimator and (a float) ImuFilter.  Float state, Madgwick filter when the
// accelerometer is available, otherwise gyro-only integration from the previous state.
//
// Layout (sizeof == 0x2A0 = 672, alignment 16):
//   +0   std::shared_ptr<ImuFilter> filter_        (ptr +0, ctrl +8)
//   +16  std::mutex mutex_                          (80 bytes; never locked in this chunk)
//   +96  bool ready_                                (c14 name; initialised)
//   +97  bool reset_                                (forces re-initialisation; never set here)
//   +98  bool use_accelerometer_                    (latched once a non-zero accel is seen)
//   +100 Eigen::Vector3f bg_                        gyro bias (initial estimate / bias estimator)
//   +112 Eigen::Vector3f ba_                        accel bias (stored only)
//   +128 State state_                               (80 bytes; ctor 0x1801662D0)
//   +208 ImuSample last_imu_                        (c14 calls it config_)
//   +240 std::vector<ImuSample> init_buffer_
//   +264 8 bytes, zeroed (unknown_264_)            TODO(verify)
//   +272 float gravity_ = 9.80667f                  (unused here)
//   +280 GyroscopeBiasEstimator bias_estimator_     (384 bytes)
//   +664 (padding to 672)
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sensor_fusion/gyroscope_bias_estimator.h"
#include "sensor_fusion/imu_filter.h"

namespace pimax {
namespace ThreeDof {

// Ctor 0x1801662D0 (interface chunk).  sizeof == 80 (Quaternionf forces 16-byte alignment;
// c14's static_assert(sizeof == 72) is wrong: the tracker has state_ at +128 and the next
// member at +208).  The compiler-generated copy 0x1801A6C10 copies 72 bytes.
struct State {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3f p = Eigen::Vector3f::Zero();            // +0
  Eigen::Vector3f v = Eigen::Vector3f::Zero();            // +12
  Eigen::Vector3f ba = Eigen::Vector3f::Zero();           // +24
  Eigen::Vector3f bg = Eigen::Vector3f::Zero();           // +36
  Eigen::Quaternionf q = Eigen::Quaternionf::Identity();  // +48 (x,y,z,w)
  double t = -1.0;                                        // +64
};
static_assert(sizeof(State) == 80, "ThreeDof::State");

// 32 bytes.  NOTE: c14 declares {double t; acc; gyr} -- the code here reads acc at +0,
// gyr at +12 and t at +24 (Propagate/Initialize/InitState), so the order below is the
// binary's.
struct ImuSample {
  Eigen::Vector3f acc;  // +0
  Eigen::Vector3f gyr;  // +12
  double t;             // +24 (seconds)
};
static_assert(sizeof(ImuSample) == 32, "ThreeDof::ImuSample");

class ThreeDofTracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 0x1801A6AC0
  ThreeDofTracker();
  ~ThreeDofTracker() = default;  // inlined into SLAMManager code

  // 0x1801A7A50 (c14: SetImuExtrinsics) -- called with FrameProcessorBase imu params
  // +536 / +548.  TODO(verify) meaning; bg_ is subtracted from the gyro.
  void SetBias(const Eigen::Vector3f& bg, const Eigen::Vector3f& ba);

  // 0x1801A7540 (c14 name)
  void Propagate(const ImuSample& last_imu, const std::vector<ImuSample>& imus,
                 const State& state_in, State* state_out);

 private:
  // 0x1801A7030
  bool Initialize(const std::vector<ImuSample>& imus);
  // 0x1801A7380
  void InitState(const std::vector<ImuSample>& imus, State* state);
  // 0x1801A6CA0
  void PropagateGyroOnly(const ImuSample& last_imu, const std::vector<ImuSample>& imus,
                         const State& state_in, State* state_out);

 public:
  std::shared_ptr<ImuFilter> filter_;   // +0
  std::mutex mutex_;                    // +16
  bool ready_ = false;                  // +96
  bool reset_ = false;                  // +97
  bool use_accelerometer_ = false;      // +98
  Eigen::Vector3f bg_;                  // +100
  Eigen::Vector3f ba_;                  // +112
  State state_;                         // +128
  ImuSample last_imu_;                  // +208
  std::vector<ImuSample> init_buffer_;  // +240
  uint64_t unknown_264_ = 0;            // +264 TODO(verify)
  float gravity_ = 9.80667f;            // +272
  GyroscopeBiasEstimator bias_estimator_;  // +280

  static void layout_check();
};

inline void ThreeDofTracker::layout_check() {
  static_assert(sizeof(ThreeDofTracker) == 0x2A0, "sizeof(ThreeDofTracker)");
  static_assert(offsetof(ThreeDofTracker, mutex_) == 16, "ThreeDofTracker::mutex_");
  static_assert(offsetof(ThreeDofTracker, ready_) == 96, "ThreeDofTracker::ready_");
  static_assert(offsetof(ThreeDofTracker, reset_) == 97, "ThreeDofTracker::reset_");
  static_assert(offsetof(ThreeDofTracker, use_accelerometer_) == 98, "ThreeDofTracker::use_accelerometer_");
  static_assert(offsetof(ThreeDofTracker, bg_) == 100, "ThreeDofTracker::bg_");
  static_assert(offsetof(ThreeDofTracker, ba_) == 112, "ThreeDofTracker::ba_");
  static_assert(offsetof(ThreeDofTracker, state_) == 128, "ThreeDofTracker::state_");
  static_assert(offsetof(ThreeDofTracker, last_imu_) == 208, "ThreeDofTracker::last_imu_");
  static_assert(offsetof(ThreeDofTracker, init_buffer_) == 240, "ThreeDofTracker::init_buffer_");
  static_assert(offsetof(ThreeDofTracker, unknown_264_) == 264, "ThreeDofTracker::unknown_264_");
  static_assert(offsetof(ThreeDofTracker, gravity_) == 272, "ThreeDofTracker::gravity_");
  static_assert(offsetof(ThreeDofTracker, bias_estimator_) == 280, "ThreeDofTracker::bias_estimator_");
  static_assert(offsetof(State, q) == 48, "State::q");
  static_assert(offsetof(State, t) == 64, "State::t");
  static_assert(offsetof(ImuSample, gyr) == 12, "ImuSample::gyr");
  static_assert(offsetof(ImuSample, t) == 24, "ImuSample::t");
}

}  // namespace ThreeDof
}  // namespace pimax
