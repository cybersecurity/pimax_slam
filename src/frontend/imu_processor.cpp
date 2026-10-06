// pimax_slam.pi.dll -- src/frontend/imu_processor.cpp  (drafts c00 + c10 + c11 merged, phase B3)
//
// pimax::totem::ImuProcessor -- fork of svo/src/imu_handler.cpp (svo::ImuHandler).
// Original file: E:\code_codex\pimax_slam\beta111_5a7902_dll\src\frontend\imu_processor.cpp
// Object TU31: dynamic initializer 0x1800028D0 (imu_temporal_status_names_), code
// 0x180129D60..0x18012B320 (ctor/dtor/addImuMeasurement from c10, the rest from c11).
// glog line numbers seen in the binary: LOG(WARNING) 109, 119; VLOG(20) 342 (forced with #line).
//
// Global Pimax changes vs. upstream ImuHandler:
//  * no IMUHandlerOptions, no bias mutex, no temporal stationary window;
//  * imu_calib_.delay_imu_cam is never subtracted (camera and IMU stamps are already aligned);
//  * most LOG(WARNING) became LOGW (custom logger) with "\n"-terminated strings;
//  * samples are float (ImuMeasurement: double + 2x Vector3f);
//  * the output deque of getMeasurements* is filled with assign() instead of insert(begin()).
#include "frontend/imu_processor.h"

#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>

#include <Windows.h>   // Sleep()

#include <glog/logging.h>
#include <vikit/timer.h>

#include "common/logger.h"
#include "common/transformation.h"   // quaternionExp

namespace pimax {
namespace totem {

// Dynamic initializer 0x1800028D0 (.CRT$XCU #131), atexit 0x1803A5FA0, object at 0x18047ECE0.
// Upstream (svo/src/imu_handler.cpp:78): `const std::map<IMUTemporalStatus, std::string>`;
// Pimax: std::unordered_map with std::hash<IMUTemporalStatus> (4-byte FNV-1a, helper 0x180129A60).
const std::unordered_map<IMUTemporalStatus, std::string> imu_temporal_status_names_
{{IMUTemporalStatus::kStationary, "Stationary"},
  {IMUTemporalStatus::kMoving, "Moving"},
  {IMUTemporalStatus::kUnkown, "Unknown"}};

// ============================================================================================
// 0x180129D60  (c10)
// upstream ImuHandler::ImuHandler() without the IMUHandlerOptions argument.
ImuProcessor::ImuProcessor(
    const ImuCalibration& imu_calib,
    const ImuInitialization& imu_init)
  : imu_calib_(imu_calib)
  , imu_init_(imu_init)
  , acc_bias_(imu_init.acc_bias)
  , omega_bias_(imu_init.omega_bias)
{}

// ============================================================================================
// 0x180129EC0  (c10; only member destruction: ofs_, temporal_imu_window_, measurements_, mutex)
ImuProcessor::~ImuProcessor()
{}

// ============================================================================================
// 0x180129F40  (c10)
// upstream addImuMeasurement(): returns void, clears the buffer when time goes backwards,
// no temporal stationary window.
void ImuProcessor::addImuMeasurement(const ImuMeasurement& m)
{
  ulock_t lock(measurements_mut_);
  if (!measurements_.empty() && measurements_.front().timestamp_ > m.timestamp_)
  {
    LOGE("t rollback\n");
    measurements_.clear();
  }
  measurements_.push_front(m); // new measurement is at the front of the list!
}

// ============================================================================================
// inlined into 0x18012A2B0 (no out-of-line copy in the image)  (c11)
// upstream-modified: no delay correction; warnings -> LOGW, second warning has no value.
bool ImuProcessor::getClosestMeasurement(const double timestamp, ImuMeasurement& measurement) const
{
  ulock_t lock(measurements_mut_);
  if (measurements_.empty())
  {
    LOGW("don't have any imu measurements!\n");
    return false;
  }

  double dt_best = std::numeric_limits<double>::max();
  const double img_timestamp_corrected = timestamp;
  for (const ImuMeasurement& m : measurements_)
  {
    const double dt = std::abs(m.timestamp_ - img_timestamp_corrected);
    if (dt < dt_best)
    {
      dt_best = dt;
      measurement = m;
    }
  }

  if (dt_best > imu_calib_.max_imu_delta_t)
  {
    LOGW("ImuProcessor: getClosestMeasurement: no measurement found closest measurement\n");
    return false;
  }
  return true;
}

// ============================================================================================
// 0x18012A2B0  (c11)
// upstream-modified: getClosestMeasurement inlined (above); on failure LOGW instead of
// LOG(WARNING); the helper axis is fixed to p = (0,0,1) (the p/p_alternative selection is gone);
// VLOG(3) "Initial Rotation" removed.  Quaternion = Eigen::Quaterniond (no kindr CHECK).
bool ImuProcessor::getInitialAttitude(double timestamp, Quaternion& R_imu_world) const
{
  ImuMeasurement m;
  if (!getClosestMeasurement(timestamp, m))
  {
    LOGW("ImuProcessor: Could not get initial attitude. No measurements!\n");
    return false;
  }

  // Set initial coordinate frame based on gravity direction.
  const Eigen::Vector3d g = m.linear_acceleration_.cast<double>();
  const Eigen::Vector3d z = g.normalized();  // imu measures positive-z when static
  const Eigen::Vector3d p(0, 0, 1);          // TODO(verify) literal spelling; values proved by the cross product
  Eigen::Vector3d y = z.cross(p);
  y.normalize();
  const Eigen::Vector3d x = y.cross(z);
  Eigen::Matrix3d C_imu_world;  // world unit vectors in imu coordinates
  C_imu_world.col(0) = x;
  C_imu_world.col(1) = y;
  C_imu_world.col(2) = z;

  R_imu_world = Quaternion(C_imu_world);   // quaternion_assign_impl 0x180020AF0
  return true;
}

// ============================================================================================
// 0x18012A700  (c11)
// upstream-modified: assert(new>old) -> LOGE + return false (checked before locking); no camera
// delay correction; "newest imu measurement is too old" check removed; LOG(WARNING) -> LOGW;
// measurements.insert(begin, it2, it1) -> measurements.assign(it2, it1) (deque::_Assign_range
// 0x180129870).
bool ImuProcessor::getMeasurements(const double old_cam_timestamp, const double new_cam_timestamp,
                                   const bool delete_old_measurements, ImuMeasurements& measurements)
{
  if (!(old_cam_timestamp < new_cam_timestamp))
  {
    LOGE("new_cam_timestamp <= old_cam_timestamp\n");
    return false;
  }
  ulock_t lock(measurements_mut_);
  if (measurements_.empty())
  {
    LOGW("don't have any imu measurements!\n");
    return false;
  }

  const double t1 = old_cam_timestamp;
  const double t2 = new_cam_timestamp;

  // Find the right measurements for integration, note that the newest
  // measurement is at the front of the list!
  ImuMeasurements::iterator it1 = measurements_.end();  // older timestamp
  ImuMeasurements::iterator it2 = measurements_.end();  // newer timestamp
  bool it2_set = false;
  for (ImuMeasurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it)
  {
    if (!it2_set && it->timestamp_ < t2)
    {
      it2 = it;
      it2_set = true;
    }
    if (it->timestamp_ <= t1)
    {
      it1 = it;
      break;
    }
  }

  if (it1 == measurements_.end())
  {
    LOGW("need an older measurement for t1!\n");
    return false;
  }
  if (it2 == measurements_.end())
  {
    LOGW("need an older measurement for t2!\n");
    return false;
  }
  if (it1 == it2)
  {
    LOGW("not enough imu measurements!\n");
    return false;
  }

  // copy affected measurements
  ++it1;  // make sure to copy the last element!
  measurements.assign(it2, it1);

  // change timestamp of oldest measurement
  measurements.back().timestamp_ = t1;

  // delete measurements that will not be used anymore
  if (delete_old_measurements)
  {
    measurements_.erase(it1, measurements_.end());
  }
  return true;
}

// ============================================================================================
// 0x18012A950  (c11)
// upstream-modified: no delay correction; insert -> assign; "need older imu measurements!"
// (LOG(WARNING)) -> LOGW("need older imu data!\n"). The two other warnings stay glog.
bool ImuProcessor::getMeasurementsContainingEdges(const double frame_timestamp,
                                                  ImuMeasurements& extracted_measurements,
                                                  const bool remove_measurements)
{
  ulock_t lock(measurements_mut_);
  if (measurements_.empty())
  {
#line 109
    LOG(WARNING) << "don't have any imu measurements!";   // imu_processor.cpp:109
    return false;
  }

  const double t = frame_timestamp;

  // Find the first measurement newer than frame_timestamp,
  // note that the newest measurement is at the front of the list!
  ImuMeasurements::iterator it = measurements_.begin();
  for (; it != measurements_.end(); ++it)
  {
    if (it->timestamp_ < t)
    {
      if (it == measurements_.begin())
      {
#line 119
        LOG(WARNING) << "need a newer measurement for interpolation!";   // imu_processor.cpp:119
        return false;
      }
      // decrement iterator again to point to element >= t
      --it;
      break;
    }
  }

  // copy affected measurements
  extracted_measurements.assign(it, measurements_.end());

  // check
  if (extracted_measurements.size() < 2)
  {
    LOGW("need older imu data!\n");
    extracted_measurements.clear();   // 0x180041E40
    return false;
  }

  if (remove_measurements)
  {
    // keep it+1 (first sample older than frame_timestamp) for the next interpolation
    measurements_.erase(it + 2, measurements_.end());
  }
  return true;
}

// ============================================================================================
// 0x18012AB60  (c11)
// upstream-modified: no delay correction; samples converted float->double; Eigen quaternion with
// unconditional renormalisation after every product (kindr only renormalised when |q|^2 drifted
// by > 1e-4); exp() threshold 1e-12 (quaternionExp, common/transformation.h).
bool ImuProcessor::getRelativeRotationPrior(const double old_cam_timestamp,
                                            const double new_cam_timestamp,
                                            bool delete_old_measurements,
                                            Quaternion& R_oldimu_newimu)
{
  ImuMeasurements measurements;
  if (!getMeasurements(old_cam_timestamp, new_cam_timestamp, delete_old_measurements, measurements))
    return false;

  // integrate all measurements from t1 to t2
  R_oldimu_newimu.setIdentity();
  ImuMeasurements::reverse_iterator it = measurements.rbegin();
  ImuMeasurements::reverse_iterator it_plus = measurements.rbegin();
  ++it_plus;
  for (; it != measurements.rend(); ++it, ++it_plus)
  {
    double dt = 0.0;
    if (it_plus == measurements.rend())  // only for newest measurement
      dt = new_cam_timestamp - it->timestamp_;
    else
      dt = it_plus->timestamp_ - it->timestamp_;

    const Eigen::Vector3d omega_corrected = it->angular_velocity_.cast<double>() - omega_bias_;
    R_oldimu_newimu = R_oldimu_newimu * quaternionExp(omega_corrected * dt);
    R_oldimu_newimu.normalize();
  }
  return true;
}

// ============================================================================================
// 0x18012B090  (c11)
// pimax-new. Called from the SLAM manager's IMU input path (0x180168640).
// Keeps the newest half once the buffer reaches 3000 samples.
void ImuProcessor::limitMeasurementsSize()
{
  std::lock_guard<mutex_t> lock(measurements_mut_);   // plain _Mtx_lock/_Mtx_unlock, no owns flag
  if (measurements_.size() >= 3000)
  {
    measurements_.erase(measurements_.begin() + measurements_.size() / 2, measurements_.end());
  }
}

// ============================================================================================
// 0x18012B140  (c11)
// upstream-modified: no delay correction; getLatestTimestamp() returns 0 on empty buffer;
// the timeout is measured from the start (no resume()), and is silent (no LOG(ERROR));
// busy loop sleeps 1 ms (Win32 Sleep); VLOG level 50 -> 20.
// (asm: Timer ctor 0x180014730, then an inlined start(); 64-bit ns difference -> cvtsi2sd.)
bool ImuProcessor::waitTill(const double img_timestamp_sec, const double timeout_sec)
{
  vk::Timer wait_time;
  wait_time.start();
  while (this->getLatestTimestamp() < img_timestamp_sec)
  {
    if (wait_time.stop() > timeout_sec)
    {
      return false;
    }
    Sleep(1);
#line 342
    VLOG(20) << "Waiting for imu measurements.";   // imu_processor.cpp:342
  }
  return true;
}

}  // namespace totem
}  // namespace pimax
