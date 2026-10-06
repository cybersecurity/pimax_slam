// pimax_slam.pi.dll -- src/interface/ceres_backend_factory.h  (from draft c13)
//
// Pimax port of svo_ros/ceres_backend_factory.h (ROS parameters replaced by constants).  Only
// makeBackend survives (no loadMotionDetectorOptions, no startThread).  The option structs live
// in ceres_backend/ceres_backend_interface.hpp; the factory values are listed there and in
// interface/ceres_backend_factory.cpp (draft c13, VLOG on line 13).
// The namespace spelling (pimax::totem::ceres_backend_factory) is a guess: TODO(verify).
#pragma once

#include <memory>

#include "common/camera_fwd.h"
#include "ceres_backend/ceres_backend_interface.hpp"

namespace pimax {
namespace totem {
namespace ceres_backend_factory {

// 0x180158F70
CeresBackendInterface::Ptr makeBackend(const CameraBundlePtr& camera_bundle);

}  // namespace ceres_backend_factory
}  // namespace totem
}  // namespace pimax
