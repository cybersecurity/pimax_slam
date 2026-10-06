// src/sensor_fusion/imu_filter.h (TODO(verify) path)
// pimax::ThreeDof::ImuFilter -- Madgwick AHRS (IMU-only), the single-precision ROS
// `imu_filter_madgwick` ImuFilter (float gain/zeta/quaternion, like the pre-2017 ROS version).
// Differences vs. the LedObjectPoseEstimator CtrlThreeDof::ImuFilter (double, gravity
// hard-coded to +Y): everything is float, the world-frame switch is kept and has a fourth
// frame (3) with gravity along +X.
//
// vtable 0x1803BF528: [0] 0x1801A5E20 scalar deleting dtor.
// Created with std::make_shared (_Ref_count_obj2<ImuFilter>, vtable 0x1803BF548, block 0x40
// bytes => sizeof(ImuFilter) == 48).
//
// Layout:
//   +0  vptr
//   +8  float gain_          (ctor 0.05f, set to 0.005f by ThreeDofTracker)
//   +12 float zeta_          (0.001f, unused in the IMU-only update)
//   +16 int   world_frame_   (ctor 2 = NWU)
//   +20 float q0, q1, q2, q3 (w, x, y, z)
//   +36 float w_bx_, w_by_, w_bz_
#pragma once

#include <cmath>
#include <cstddef>

namespace pimax {
namespace ThreeDof {

namespace WorldFrame {
// ROS: enum WorldFrame { ENU, NED, NWU }.  Value 3 is a Pimax addition (gravity +X).
enum WorldFrame { ENU = 0, NED = 1, NWU = 2, XUP = 3 /* TODO(verify) name */ };
}  // namespace WorldFrame

class ImuFilter {
 public:
  ImuFilter();           // 0x1801A5DE0
  virtual ~ImuFilter();  // 0x1801A5E20 (deleting)

  void setAlgorithmGain(float gain) { gain_ = gain; }
  void setDriftBiasGain(float zeta) { zeta_ = zeta; }
  void setWorldFrame(WorldFrame::WorldFrame frame) { world_frame_ = frame; }

  // inlined in ThreeDofTracker::Propagate (0x1801A7540)
  void getOrientation(float& q0, float& q1, float& q2, float& q3) {
    q0 = this->q0;
    q1 = this->q1;
    q2 = this->q2;
    q3 = this->q3;
    float recipNorm = 1 / std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
  }

  // inlined in ThreeDofTracker::InitState (0x1801A7380)
  void setOrientation(float q0, float q1, float q2, float q3) {
    this->q0 = q0;
    this->q1 = q1;
    this->q2 = q2;
    this->q3 = q3;
    w_bx_ = 0;
    w_by_ = 0;
    w_bz_ = 0;
  }

  // 0x1801A61B0
  void madgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,
                             float dt);

 private:
  float gain_;                          // algorithm gain
  float zeta_;                          // gyro drift bias gain
  WorldFrame::WorldFrame world_frame_;  // NWU, ENU, NED (+ Pimax XUP)
  float q0, q1, q2, q3;                 // quaternion
  float w_bx_, w_by_, w_bz_;            //

  friend class ThreeDofTracker;
  friend struct ImuFilterLayout;
};

struct ImuFilterLayout {
  static_assert(offsetof(ImuFilter, gain_) == 8, "ImuFilter::gain_");
  static_assert(offsetof(ImuFilter, zeta_) == 12, "ImuFilter::zeta_");
  static_assert(offsetof(ImuFilter, world_frame_) == 16, "ImuFilter::world_frame_");
  static_assert(offsetof(ImuFilter, q0) == 20, "ImuFilter::q0");
  static_assert(offsetof(ImuFilter, w_bx_) == 36, "ImuFilter::w_bx_");
};

static_assert(sizeof(ImuFilter) == 48, "ImuFilter size");

}  // namespace ThreeDof
}  // namespace pimax
