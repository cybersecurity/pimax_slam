// Reconstructed from pimax_slam.pi.dll (chunk c18_tail_vikit).
// Upstream: rpg_svo_pro_open/vikit/vikit_cameras/include/vikit/cameras/ncamera.h
//
// Pimax changes:
//  * second transformation vector T_B_C_ (+24) added; ctor takes it as 2nd argument.
//    (the calibration loader 0x18015E250 pushes T into one vector and T.inverse() into the
//    other; which one is "T_C_B" is decided by the getter names - see notes)  // TODO(verify) naming
//  * no initInternal() / CHECKs in ctor; no CHECK_LT in accessors.
//  * loadFromYaml / printParameters not in the binary (no yaml-cpp) -> omitted.
//
// Layout: sizeof(NCamera) == 104 (0x68; make_shared control block 0x78 at 0x180159A40)
//   +0   TransformationVector T_C_B_   (std::vector<QuatTransformation, aligned_allocator>, 24 B)
//   +24  TransformationVector T_B_C_   (Pimax-added)
//   +48  std::vector<std::shared_ptr<Camera>> cameras_
//   +72  std::string label_            ("PiMax" from the calibration loader)
#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <vikit/cameras/camera_geometry_base.h>

namespace vk {
namespace cameras {

class CameraGeometryBase;
using Camera = CameraGeometryBase;

class NCamera
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<NCamera> Ptr;
  typedef std::shared_ptr<const NCamera> ConstPtr;

protected:
  NCamera() = default;

public:
  // 0x1801B4050
  NCamera(
      const TransformationVector& T_C_B,
      const TransformationVector& T_B_C,
      const std::vector<std::shared_ptr<Camera>>& cameras,
      const std::string& label);

  ~NCamera() {}

  NCamera(const NCamera&) = delete;
  void operator=(const NCamera&) = delete;

  inline size_t getNumCameras() const { return cameras_.size(); }

  /// Get the pose of body frame with respect to the camera i. (0x1801B41F0)
  const Transformation& get_T_C_B(size_t camera_index) const;

  /// Pimax-added: pose of camera i w.r.t. body. (0x1801B41E0)
  const Transformation& get_T_B_C(size_t camera_index) const;

  /// Get all transformations.
  inline const TransformationVector& getTransformationVector() const { return T_C_B_; }

  /// Get the geometry object for camera i.
  inline const Camera& getCamera(size_t camera_index) const { return *cameras_[camera_index]; }

  /// Get the geometry object for camera i. (0x1801B41A0)
  std::shared_ptr<Camera> getCameraShared(size_t camera_index);

  inline size_t numCameras() const { return cameras_.size(); }

  inline const std::vector<std::shared_ptr<Camera>>& getCameraVector() const { return cameras_; }

  inline const std::string& getLabel() const {return label_;}
  inline void setLabel(const std::string& label) {label_ = label;}

private:
  TransformationVector T_C_B_;
  TransformationVector T_B_C_;
  std::vector<std::shared_ptr<Camera>> cameras_;
  std::string label_;

  friend struct NCameraLayout;
};

struct NCameraLayout {
  static_assert(sizeof(NCamera) == 104, "sizeof(NCamera)");
  static_assert(offsetof(NCamera, T_B_C_) == 24, "NCamera::T_B_C_");
  static_assert(offsetof(NCamera, cameras_) == 48, "NCamera::cameras_");
  static_assert(offsetof(NCamera, label_) == 72, "NCamera::label_");
};

} // namespace cameras
} // namespace vk
