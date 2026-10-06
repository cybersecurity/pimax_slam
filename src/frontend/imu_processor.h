// pimax_slam.pi.dll -- src/frontend/imu_processor.h  (drafts c10 + c11 + c14 merged)
//
// pimax::totem::ImuProcessor -- fork of svo/imu_handler.h (svo::ImuHandler).
// sizeof 0x2A8 (680): make_shared allocates 0x2B8 in getImuProcessor 0x18015CBD0; no vtable.
// Compared with upstream: IMUHandlerOptions options_ and bias_mut_ are gone (c10's ctor writes the
// biases right after imu_init_), addImuMeasurement returns void and clears the buffer when time
// goes backwards ("t rollback"), the IMU-camera delay correction is gone, and the measurement
// samples are float (common/imu_calibration.h).
// ImuCalibration / ImuInitialization keep the upstream svo_common member order (sizes proven by
// ImuProcessor: imu_calib_ +0, imu_init_ +112, acc_bias_ +208); A1 left them to this header.
#pragma once

#include <cstddef>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdDeque>

#include "common/imu_calibration.h"   // ImuMeasurement, ImuMeasurements
#include "common/transformation.h"    // Quaternion (= Eigen::Quaterniond)
#include "common/types.h"

namespace pimax {
namespace totem {

/// 112 bytes (upstream svo::ImuCalibration member order).  The defaults are upstream's; the values
/// actually used are written by getImuProcessor 0x18015CBD0 (c14): delay 0, max_imu_delta_t 0.01,
/// gyro_noise 0.004, acc_noise 0.02, integration sigma 0, gyro rw 0.0004, acc rw 0.002,
/// gravity 9.80667, coriolis 0, sat accel 156.8, sat omega 35.0, rate 1000.
class ImuCalibration
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<ImuCalibration> Ptr;

  double delay_imu_cam = 0.0;                         // +0  (no longer used: no delay correction)
  double max_imu_delta_t = 0.01;                      // +8
  double gyro_noise_density = 0.00073088444;          // +16
  double acc_noise_density = 0.01883649;              // +24
  double imu_integration_sigma = 0.0;                 // +32
  double gyro_bias_random_walk_sigma = 0.00038765;    // +40
  double acc_bias_random_walk_sigma = 0.012589254;    // +48
  double gravity_magnitude = 9.81007;                 // +56
  Eigen::Vector3d omega_coriolis = Eigen::Vector3d::Zero();   // +64
  double saturation_accel_max = 150;                  // +88
  double saturation_omega_max = 7.8;                  // +96
  double imu_rate = 20;                               // +104

  ImuCalibration() = default;
  ~ImuCalibration() = default;
};
static_assert(sizeof(ImuCalibration) == 112, "sizeof(ImuCalibration)");
static_assert(offsetof(ImuCalibration, max_imu_delta_t) == 8, "");
static_assert(offsetof(ImuCalibration, gravity_magnitude) == 56, "");
static_assert(offsetof(ImuCalibration, omega_coriolis) == 64, "");
static_assert(offsetof(ImuCalibration, imu_rate) == 104, "");

/// 96 bytes (upstream svo::ImuInitialization).  getImuProcessor: velocity/omega_bias/acc_bias 0,
/// sigmas 2.0 / 0.01 / 0.1.
class ImuInitialization
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<ImuInitialization> Ptr;

  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();     // +0
  Eigen::Vector3d omega_bias = Eigen::Vector3d::Zero();   // +24
  Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();     // +48
  double velocity_sigma = 2.0;                             // +72
  double omega_bias_sigma = 0.01;                          // +80
  double acc_bias_sigma = 0.1;                             // +88
};
static_assert(sizeof(ImuInitialization) == 96, "sizeof(ImuInitialization)");

enum class IMUTemporalStatus
{
  kStationary = 0,
  kMoving = 1,
  kUnkown = 2
};

/// 0x18047ECE0, dynamic initializer 0x1800028D0 (atexit 0x1803A5FA0):
///   {kStationary,"Stationary"},{kMoving,"Moving"},{kUnkown,"Unknown"}.
/// Pimax: std::unordered_map with the default std::hash (FNV-1a over the 4 enum bytes, insert-range
/// 0x180129A60), upstream std::map.  Defined in imu_processor.cpp (draft c00
/// imu_processor_globals.cpp).
extern const std::unordered_map<IMUTemporalStatus, std::string> imu_temporal_status_names_;

class ImuProcessor
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<ImuProcessor> Ptr;
  typedef std::mutex mutex_t;
  typedef std::unique_lock<mutex_t> ulock_t;

  /// 0x180129D60 -- upstream ImuHandler ctor without the IMUHandlerOptions argument.
  ImuProcessor(const ImuCalibration& imu_calib, const ImuInitialization& imu_init);
  /// 0x180129EC0 (member destruction only)
  ~ImuProcessor();

  /// 0x180129F40 -- Pimax: returns void; clears the buffer on "t rollback".
  void addImuMeasurement(const ImuMeasurement& measurement);

  /// 0x18012A700
  bool getMeasurements(const double old_cam_timestamp, const double new_cam_timestamp,
                       const bool delete_old_measurements, ImuMeasurements& measurements);

  /// 0x18012A950 (c14 call: getMeasurementsContainingEdges(t, imus, true))
  bool getMeasurementsContainingEdges(const double frame_timestamp, ImuMeasurements& measurements,
                                      const bool remove_measurements);

  /// only inlined into getInitialAttitude (0x18012A2B0)
  bool getClosestMeasurement(const double timestamp, ImuMeasurement& measurement) const;

  /// 0x18012AB60 -- Pimax renormalises the quaternion after every step.
  bool getRelativeRotationPrior(const double old_cam_timestamp, const double new_cam_timestamp,
                                bool delete_old_measurements, Quaternion& R_oldimu_newimu);

  /// 0x18012A2B0 ("ImuProcessor: Could not get initial attitude. No measurements!")
  bool getInitialAttitude(double timestamp, Quaternion& R_imu_world) const;

  /// 0x18012B090 [pimax-new] drop the older half of the buffer once it holds >= 3000 samples.
  /// Name TODO(verify) (c14: "trimMeasurements").
  void limitMeasurementsSize();

  /// Pimax: returns 0.0 when the buffer is empty (upstream dereferenced front()).
  double getLatestTimestamp() const
  {
    ulock_t lock(measurements_mut_);
    return measurements_.empty() ? 0.0 : measurements_.front().timestamp_;
  }

  /// 0x18012B140 -- c14 calls waitTill(t, 0.033).
  bool waitTill(const double timestamp_sec, const double timeout_sec = 1.0);

  // ---- data ------------------------------------------------------------------------------------
  ImuCalibration imu_calib_;            // +0   (FrameProcessorBase reads +16..+56)
  ImuInitialization imu_init_;          // +112
  Eigen::Vector3d acc_bias_;            // +208 (written directly by FrameProcessorBase resets)
  Eigen::Vector3d omega_bias_;          // +232 (written directly by FrameProcessorBase)

 protected:
  mutable mutex_t measurements_mut_;    // +256
  ImuMeasurements measurements_;        // +336 newest measurement at the front
  ImuMeasurements temporal_imu_window_; // +376 (only destroyed)
  std::ofstream ofs_;                   // +416 (264 bytes; only destroyed)

 private:
  friend struct ImuProcessorLayoutCheck;
};

struct ImuProcessorLayoutCheck
{
  static_assert(sizeof(ImuProcessor) == 0x2A8, "sizeof(ImuProcessor)");
  static_assert(offsetof(ImuProcessor, imu_init_) == 112, "");
  static_assert(offsetof(ImuProcessor, acc_bias_) == 208, "");
  static_assert(offsetof(ImuProcessor, omega_bias_) == 232, "");
  static_assert(offsetof(ImuProcessor, measurements_mut_) == 256, "");
  static_assert(offsetof(ImuProcessor, measurements_) == 336, "");
  static_assert(offsetof(ImuProcessor, temporal_imu_window_) == 376, "");
  static_assert(offsetof(ImuProcessor, ofs_) == 416, "");
};

}  // namespace totem
}  // namespace pimax
