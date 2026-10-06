// src/sensor_fusion/imu_filter.cpp (TODO(verify) path)
// Madgwick IMU filter -- ROS imu_tools/imu_filter_madgwick src/imu_filter.cpp, float version.
// upstream-identical except: the constructor defaults (gain 0.05f, zeta 0.001f, NWU) and the
// extra world-frame case 3 (gravity +X) in madgwickAHRSupdateIMU.
#include "sensor_fusion/imu_filter.h"

#include <cmath>

namespace pimax {
namespace ThreeDof {

// Fast inverse square-root (magic 0x5F3759DF), inlined everywhere.
static float invSqrt(float x) {
  float xhalf = 0.5f * x;
  union {
    float x;
    int i;
  } u;
  u.x = x;
  u.i = 0x5f3759df - (u.i >> 1);
  /* The next line can be repeated any number of times to increase accuracy */
  u.x = u.x * (1.5f - xhalf * u.x * u.x);
  return u.x;
}

template <typename T>
static inline void normalizeVector(T& vx, T& vy, T& vz) {
  T recipNorm = invSqrt(vx * vx + vy * vy + vz * vz);
  vx *= recipNorm;
  vy *= recipNorm;
  vz *= recipNorm;
}

template <typename T>
static inline void normalizeQuaternion(T& q0, T& q1, T& q2, T& q3) {
  T recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

// 0x1801A6630 (not inlined)
static inline void rotateAndScaleVector(float q0, float q1, float q2, float q3, float _2dx,
                                        float _2dy, float _2dz, float& rx, float& ry,
                                        float& rz) {
  // result is half as long as input
  rx = _2dx * (0.5f - q2 * q2 - q3 * q3) + _2dy * (q0 * q3 + q1 * q2) +
       _2dz * (q1 * q3 - q0 * q2);
  ry = _2dx * (q1 * q2 - q0 * q3) + _2dy * (0.5f - q1 * q1 - q3 * q3) +
       _2dz * (q0 * q1 + q2 * q3);
  rz = _2dx * (q0 * q2 + q1 * q3) + _2dy * (q2 * q3 - q0 * q1) +
       _2dz * (0.5f - q1 * q1 - q2 * q2);
}

static inline void orientationChangeFromGyro(float q0, float q1, float q2, float q3, float gx,
                                             float gy, float gz, float& qDot1, float& qDot2,
                                             float& qDot3, float& qDot4) {
  // Rate of change of quaternion from gyroscope
  // See EQ 12
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
}

// 0x1801A5E50 (not inlined)
static inline void addGradientDescentStep(float q0, float q1, float q2, float q3, float _2dx,
                                          float _2dy, float _2dz, float mx, float my, float mz,
                                          float& s0, float& s1, float& s2, float& s3) {
  float f0, f1, f2;

  // Gradient decent algorithm corrective step
  // EQ 15, 21
  rotateAndScaleVector(q0, q1, q2, q3, _2dx, _2dy, _2dz, f0, f1, f2);

  f0 -= mx;
  f1 -= my;
  f2 -= mz;

  // EQ 22, 34
  // Jt * f
  s0 += (_2dy * q3 - _2dz * q2) * f0 + (-_2dx * q3 + _2dz * q1) * f1 +
        (_2dx * q2 - _2dy * q1) * f2;
  s1 += (_2dy * q2 + _2dz * q3) * f0 + (_2dx * q2 - 2.0f * _2dy * q1 + _2dz * q0) * f1 +
        (_2dx * q3 - _2dy * q0 - 2.0f * _2dz * q1) * f2;
  s2 += (-2.0f * _2dx * q2 + _2dy * q1 - _2dz * q0) * f0 + (_2dx * q1 + _2dz * q3) * f1 +
        (_2dx * q0 + _2dy * q3 - 2.0f * _2dz * q2) * f2;
  s3 += (-2.0f * _2dx * q3 + _2dy * q0 + _2dz * q1) * f0 +
        (-_2dx * q0 - 2.0f * _2dy * q3 + _2dz * q2) * f1 + (_2dx * q1 + _2dy * q2) * f2;
}

// 0x1801A5DE0
ImuFilter::ImuFilter()
    : gain_(0.05f),   // 0x3D4CCCCD
      zeta_(0.001f),  // 0x3A83126F
      world_frame_(WorldFrame::NWU),
      q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
      w_bx_(0.0f), w_by_(0.0f), w_bz_(0.0f) {}

// 0x1801A5E20
ImuFilter::~ImuFilter() {}

// 0x1801A61B0
void ImuFilter::madgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay,
                                      float az, float dt) {
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;

  // Rate of change of quaternion from gyroscope
  orientationChangeFromGyro(q0, q1, q2, q3, gx, gy, gz, qDot1, qDot2, qDot3, qDot4);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalise accelerometer measurement
    normalizeVector(ax, ay, az);

    // Gradient decent algorithm corrective step
    s0 = 0.0;
    s1 = 0.0;
    s2 = 0.0;
    s3 = 0.0;
    switch (world_frame_) {
      case WorldFrame::NED:
        // Gravity: [0, 0, -1]
        addGradientDescentStep(q0, q1, q2, q3, 0.0, 0.0, -2.0, ax, ay, az, s0, s1, s2, s3);
        break;
      case WorldFrame::XUP:
        // pimax: Gravity: [1, 0, 0]
        addGradientDescentStep(q0, q1, q2, q3, 2.0, 0.0, 0.0, ax, ay, az, s0, s1, s2, s3);
        break;
      case WorldFrame::NWU:
      case WorldFrame::ENU:
      default:
        // Gravity: [0, 0, 1]
        addGradientDescentStep(q0, q1, q2, q3, 0.0, 0.0, 2.0, ax, ay, az, s0, s1, s2, s3);
        break;
    }

    normalizeQuaternion(s0, s1, s2, s3);

    // Apply feedback step
    qDot1 -= gain_ * s0;
    qDot2 -= gain_ * s1;
    qDot3 -= gain_ * s2;
    qDot4 -= gain_ * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalise quaternion
  normalizeQuaternion(q0, q1, q2, q3);
}

}  // namespace ThreeDof
}  // namespace pimax
