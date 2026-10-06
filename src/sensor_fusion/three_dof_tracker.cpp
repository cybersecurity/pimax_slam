// src/sensor_fusion/three_dof_tracker.cpp (TODO(verify) path/class name) -- pimax-new.
#include "sensor_fusion/three_dof_tracker.h"

#include <cmath>

namespace pimax {
namespace ThreeDof {

// 0x1801A6AC0
ThreeDofTracker::ThreeDofTracker() {
  ready_ = false;
  filter_ = std::make_shared<ImuFilter>();
  filter_->setAlgorithmGain(0.005f);  // 0x3BA3D70A
  ba_ = Eigen::Vector3f::Zero();
  bg_ = Eigen::Vector3f::Zero();
  init_buffer_.clear();
  unknown_264_ = 0;
  gravity_ = 9.80667f;               // 0x411CE81F
  bias_estimator_.Reset();           // 0x1801A4E60
}

// 0x1801A7A50
void ThreeDofTracker::SetBias(const Eigen::Vector3f& bg, const Eigen::Vector3f& ba) {
  bg_ = bg;
  ba_ = ba;
}

// 0x1801A7030
// Gyro bias initialisation: accumulate IMU samples until they span > 0.1 s; accept the mean
// gyro as bias when its RMS deviation is < 0.1 rad/s, otherwise drop the oldest batch.
// Returns true immediately when a (non-zero) bias is already known.
bool ThreeDofTracker::Initialize(const std::vector<ImuSample>& imus) {
  float variance = 0.0f;
  if (!use_accelerometer_) {
    if (imus.back().acc.norm() > 0.0) {
      use_accelerometer_ = true;
    }
  }
  if (bg_.norm() > 0.0) {
    return true;
  }
  if (imus.size()) {
    init_buffer_.insert(init_buffer_.end(), imus.begin(), imus.end());
    if (std::fabs(init_buffer_.back().t - init_buffer_.front().t) > 0.1) {
      Eigen::Vector3f mean(0.f, 0.f, 0.f);
      for (const ImuSample& sample : init_buffer_) {
        mean += sample.gyr;
      }
      const float n = static_cast<float>(init_buffer_.size());
      mean = mean / n;
      for (const ImuSample& sample : init_buffer_) {
        variance += (sample.gyr - mean).squaredNorm();
      }
      const float std_dev = std::sqrt(variance / n);
      if (std_dev < 0.1) {
        bg_ = mean;
        init_buffer_.clear();
        return true;
      }
      if (init_buffer_.size() > imus.size()) {
        init_buffer_.erase(init_buffer_.begin(), init_buffer_.begin() + imus.size());
      }
    }
  }
  return false;
}

// 0x1801A7380
// QUIRK: the timestamp goes to the member state_ (overwritten again by Propagate), not to
// *state, whose t keeps the caller's value.
void ThreeDofTracker::InitState(const std::vector<ImuSample>& imus, State* state) {
  state_.t = imus.back().t;
  last_imu_ = imus.back();
  state->p = Eigen::Vector3f::Zero();
  state->q = Eigen::Quaternionf::Identity();
  state->v = Eigen::Vector3f::Zero();
  state->ba = Eigen::Vector3f::Zero();
  state->bg = bg_;
  if (use_accelerometer_) {
    // Initial attitude: rotate the first accelerometer sample onto +Z.
    const Eigen::Quaternionf q =
        Eigen::Quaternionf::FromTwoVectors(imus.front().acc, Eigen::Vector3f(0.f, 0.f, 1.f));
    filter_->setOrientation(q.w(), q.x(), q.y(), q.z());
  }
}

// 0x1801A6CA0
// Gyro-only attitude propagation (mid-point gyro, first-order quaternion increment).
void ThreeDofTracker::PropagateGyroOnly(const ImuSample& last_imu,
                                        const std::vector<ImuSample>& imus,
                                        const State& state_in, State* state_out) {
  Eigen::Vector3f last_gyr = last_imu.gyr;
  double last_time = last_imu.t;
  state_out->q = state_in.q;
  state_out->bg = bg_;
  for (const ImuSample& imu : imus) {
    const float dt = static_cast<float>(imu.t - last_time);
    last_time = imu.t;
    const Eigen::Vector3f gyr = (last_gyr + imu.gyr) * 0.5f - bg_;
    const Eigen::Vector3f half_theta = dt * gyr * 0.5f;
    const double angle = half_theta.norm();
    Eigen::Quaternionf dq;
    if (angle > 0.008726646) {  // 0.5 deg
      dq.w() = std::cos(angle);
      const double s = std::sin(angle);
      dq.x() = half_theta.x() * s / angle;
      dq.y() = half_theta.y() * s / angle;
      dq.z() = half_theta.z() * s / angle;
    } else {
      dq = Eigen::Quaternionf(1.0f, half_theta.x(), half_theta.y(), half_theta.z());
    }
    state_out->q = (state_out->q * dq).normalized();
    last_gyr = imu.gyr;
  }
  state_out->t = imus.back().t;  // QUIRK: reads back() even for an empty vector (never happens)
}

// 0x1801A7540
void ThreeDofTracker::Propagate(const ImuSample& last_imu, const std::vector<ImuSample>& imus,
                                const State& state_in, State* state_out) {
  if (imus.empty()) {
    const double t = state_out->t;
    *state_out = state_;
    state_out->t = t;
    return;
  }

  if (!ready_ || reset_) {
    ready_ = Initialize(imus);
    if (ready_) {
      InitState(imus, state_out);
    }
  } else if (use_accelerometer_) {
    double last_time = last_imu.t;
    for (size_t i = 0; i < imus.size(); i++) {
      const float dt = static_cast<float>(imus[i].t - last_time);
      bias_estimator_.ProcessGyroscope(imus[i].gyr.x(), imus[i].gyr.y(), imus[i].gyr.z(),
                                       static_cast<uint64_t>(imus[i].t * 1000000000.0));
      bias_estimator_.ProcessAccelerometer(imus[i].acc.x(), imus[i].acc.y(), imus[i].acc.z(),
                                           static_cast<uint64_t>(imus[i].t * 1000000000.0));
      if (bias_estimator_.IsCurrentEstimateValid()) {
        double x, y, z;
        bias_estimator_.GetGyroscopeBias(x, y, z);
        bg_ = Eigen::Vector3f(x, y, z);
      }
      filter_->madgwickAHRSupdateIMU(imus[i].gyr.x() - bg_.x(), imus[i].gyr.y() - bg_.y(),
                                     imus[i].gyr.z() - bg_.z(), imus[i].acc.x(),
                                     imus[i].acc.y(), imus[i].acc.z(), dt);
      last_time = imus[i].t;
    }
    float q0, q1, q2, q3;
    filter_->getOrientation(q0, q1, q2, q3);
    state_out->q = Eigen::Quaternionf(q0, q1, q2, q3).normalized();  // normalized() 0x18016D100
    state_out->bg = bg_;
    state_out->t = imus.back().t;
  } else {
    PropagateGyroOnly(last_imu, imus, state_in, state_out);
  }

  state_ = *state_out;  // State::operator= 0x1801A6C10
  last_imu_ = imus.back();
}

}  // namespace ThreeDof
}  // namespace pimax
