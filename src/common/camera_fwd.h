// pimax_slam.pi.dll -- src/common/camera_fwd.h  (upstream svo_common camera_fwd.h, namespace pimax::totem)
#pragma once

#include <memory>

namespace vk {
namespace cameras {
class CameraGeometryBase;
class NCamera;
}
}

namespace pimax {
namespace totem {
using Camera = vk::cameras::CameraGeometryBase;
using CameraPtr = std::shared_ptr<Camera>;
using CameraConstPtr = std::shared_ptr<const Camera>;
using CameraBundle = vk::cameras::NCamera;
using CameraBundlePtr = std::shared_ptr<CameraBundle>;
} // namespace totem
} // namespace pimax
