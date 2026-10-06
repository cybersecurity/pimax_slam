// pimax_slam.pi.dll -- src/common/imu_calibration.h
//
// Pimax fork of svo_common/include/svo/common/imu_calibration.h.
//
//  * ImuMeasurement is 32 bytes with FLOAT samples (c09/c10/c11): timestamp_ (seconds) +0,
//    angular_velocity_ +8, linear_acceleration_ +20.  ImuMeasurements is a deque with
//    Eigen::aligned_allocator (its _Container_proxy is allocated through Eigen's malloc + the
//    "System's malloc returned an unaligned pointer" assert); newest sample at the FRONT.
//  * ImuCalibration / ImuInitialization (upstream also here) are declared by A2 in
//    frontend/imu_processor.h (only the frontend uses them; notes/integration_requests.md).
#pragma once

#include <cstddef>
#include <deque>
#include <iostream>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/StdDeque>

#include "common/types.h"

namespace pimax {
namespace totem {

/// 32 bytes; samples stored as float in the Pimax fork.
struct ImuMeasurement
{
  double timestamp_;                      ///< +0  In seconds.
  Eigen::Vector3f angular_velocity_;      ///< +8
  Eigen::Vector3f linear_acceleration_;   ///< +20
  ImuMeasurement() {}
  ImuMeasurement(
      const double timestamp,
      const Eigen::Vector3f& angular_velocity,
      const Eigen::Vector3f& linear_acceleration)
  : timestamp_(timestamp)
  , angular_velocity_(angular_velocity)
  , linear_acceleration_(linear_acceleration)
  {}
};
typedef std::deque<ImuMeasurement,
Eigen::aligned_allocator<ImuMeasurement> > ImuMeasurements;

static_assert(sizeof(ImuMeasurement) == 32, "sizeof(ImuMeasurement)");
static_assert(offsetof(ImuMeasurement, angular_velocity_) == 8, "ImuMeasurement::angular_velocity_");
static_assert(offsetof(ImuMeasurement, linear_acceleration_) == 20, "ImuMeasurement::linear_acceleration_");
static_assert(sizeof(ImuMeasurements) == 40, "sizeof(ImuMeasurements)");

} // namespace totem
} // namespace pimax
