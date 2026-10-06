// Reconstructed from pimax_slam.pi.dll (chunk c18_tail_vikit).
// Upstream: rpg_svo_pro_open/vikit/vikit_cameras/include/vikit/cameras/camera_geometry_base.h
//
// Pimax changes vs upstream (see notes/c18_tail_vikit.md):
//  * aslam types replaced by kindr::minimal (Pimax vendors minkindr; svo::Transformation ==
//    kindr::minimal::QuatTransformationTemplate<double>).
//  * No yaml-cpp: loadFromYaml() is not in the binary (and cannot be built without yaml-cpp).
//  * loadMask / isMasked / createRandomKeypoint are not present in the binary (unreferenced or
//    removed - cannot be distinguished). They are omitted here; re-add the upstream bodies if a
//    later chunk needs them. // TODO(verify)
//  * setMask() lost its CHECK_EQs, backProject3(Matrix2Xd) lost its CHECK_NOTNULLs.
//
// Layout (x64 MSVC), sizeof(CameraGeometryBase) = 152 (0x98):
//   +0   vptr  (vk::cameras::CameraGeometryBase::`vftable' @0x1803BFE58, 9 slots)
//   +8   int         width_
//   +12  int         height_
//   +16  std::string label_          (32 bytes)
//   +48  Type        camera_type_    (int enum; NOT initialised by the ctor, as upstream)
//   +56  cv::Mat     mask_           (96 bytes)
// vtable (MSVC groups overloads in reverse declaration order, which is why the
// Matrix2Xd overload lands in slot 1):
//   0 ~CameraGeometryBase (deleting dtor 0x1801B3C50)
//   1 backProject3(Ref<Matrix2Xd>, Matrix3Xd*, vector<bool>*)  0x1801B3CE0
//   2 backProject3(Ref<Vector2d>, Vector3d*)          pure
//   3 project3                                        pure
//   4 printParameters                                 pure
//   5 errorMultiplier                                 pure
//   6 getIntrinsicParameters                          pure
//   7 getDistortionParameters                         pure
//   8 getAngleError                                   pure
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>

#include <kindr/minimal/quat-transformation.h>

namespace vk {

using Transformation = kindr::minimal::QuatTransformationTemplate<double>;
// sizeof(Transformation) == 64 (Quaterniond 32 + Vector3d 24, 16-aligned) -> element stride 64,
// allocated with Eigen::aligned_allocator (plain malloc + EIGEN alignment assert, Memory.h:185).
using TransformationVector =
    std::vector<Transformation, Eigen::aligned_allocator<Transformation>>;
using Quaternion = kindr::minimal::RotationQuaternionTemplate<double>;

namespace cameras {

/// \struct ProjectionResult (upstream identical)
struct ProjectionResult {
  enum class Status {
    KEYPOINT_VISIBLE,
    KEYPOINT_OUTSIDE_IMAGE_BOX,
    POINT_BEHIND_CAMERA,
    PROJECTION_INVALID,
    UNINITIALIZED
  };
  static Status KEYPOINT_VISIBLE;
  static Status KEYPOINT_OUTSIDE_IMAGE_BOX;
  static Status POINT_BEHIND_CAMERA;
  static Status PROJECTION_INVALID;
  static Status UNINITIALIZED;

  constexpr ProjectionResult() : status_(Status::UNINITIALIZED) {}
  constexpr ProjectionResult(Status status) : status_(status) {}

  explicit operator bool() const { return isKeypointVisible(); }
  bool operator==(const ProjectionResult& other) const { return status_ == other.status_; }
  bool operator==(const ProjectionResult::Status& other) const { return status_ == other; }
  friend std::ostream& operator<<(std::ostream& out, const ProjectionResult& state);
  bool isKeypointVisible() const { return (status_ == Status::KEYPOINT_VISIBLE); }
  Status getDetailedStatus() const { return status_; }

 private:
  Status status_;
};

class CameraGeometryBase
{
public:
  // ASLAM_POINTER_TYPEDEFS(CameraGeometryBase);
  typedef std::shared_ptr<CameraGeometryBase> Ptr;
  typedef std::shared_ptr<const CameraGeometryBase> ConstPtr;
  typedef std::unique_ptr<CameraGeometryBase> UniquePtr;

  enum class Type {
    kPinhole = 0,
    kUnifiedProjection = 1,
    kOmni = 2,
    kEqFisheye = 3
  };

  /// Default constructor (0x1801B3C00)
  CameraGeometryBase(const int width, const int height);

  /// (deleting dtor 0x1801B3C50) - upstream identical
  virtual ~CameraGeometryBase() = default;

  /// Computes bearing vector from pixel coordinates. Z-component of the returned
  /// bearing vector is 1.0. IMPORTANT: returned vector is NOT of unit length!
  virtual bool backProject3(
      const Eigen::Ref<const Eigen::Vector2d>& keypoint,
      Eigen::Vector3d* out_point_3d) const = 0;
  /// Override taking multiple keypoints (0x1801B3CE0).
  virtual void backProject3(
      const Eigen::Ref<const Eigen::Matrix2Xd>& keypoints,
      Eigen::Matrix3Xd* out_bearing_vectors, std::vector<bool>* success) const;

  /// Computes pixel coordinates from bearing vector with Jacobian w.r.t. point.
  virtual const ProjectionResult project3(
      const Eigen::Ref<const Eigen::Vector3d>& point_3d,
      Eigen::Vector2d* out_keypoint,
      Eigen::Matrix<double, 2, 3>* out_jacobian_point = nullptr) const = 0;

  /// Print camera info
  virtual void printParameters(std::ostream& out, const std::string& s = "Camera: ") const = 0;

  virtual double errorMultiplier() const = 0;
  virtual Eigen::VectorXd getIntrinsicParameters() const = 0;
  virtual Eigen::VectorXd getDistortionParameters() const = 0;
  virtual double getAngleError(double img_err) const = 0;

  inline Type getType() const { return camera_type_; }
  inline const std::string& getLabel() const { return label_; }
  inline void setLabel(const std::string& label) { label_ = label; }
  uint32_t imageWidth() const { return width_; }
  uint32_t imageHeight() const { return height_; }

  template<typename DerivedKeyPoint>
  bool isKeypointVisible(const Eigen::MatrixBase<DerivedKeyPoint>& keypoint) const;

  template<typename DerivedKeyPoint>
  bool isKeypointVisibleWithMargin(
      const Eigen::MatrixBase<DerivedKeyPoint>& keypoint,
      typename DerivedKeyPoint::Scalar margin) const;

  /// Set the mask (0x1801B3F30). Pimax: no size/type CHECKs.
  void setMask(const cv::Mat& mask);

  inline const cv::Mat& getMask() const { return mask_; }
  inline void clearMask() { mask_ = cv::Mat(); }
  inline bool hasMask() const { return !mask_.empty(); }

protected:
  int width_;
  int height_;
  std::string label_;
  Type camera_type_;
  cv::Mat mask_;

  friend struct CameraGeometryBaseLayout;
};

// PIMAX layout pins (x64 MSVC, see notes/c18_tail_vikit.md).
struct CameraGeometryBaseLayout {
  static_assert(sizeof(CameraGeometryBase) == 152, "sizeof(CameraGeometryBase)");
  static_assert(offsetof(CameraGeometryBase, width_) == 8, "CameraGeometryBase::width_");
  static_assert(offsetof(CameraGeometryBase, height_) == 12, "CameraGeometryBase::height_");
  static_assert(offsetof(CameraGeometryBase, label_) == 16, "CameraGeometryBase::label_");
  static_assert(offsetof(CameraGeometryBase, camera_type_) == 48, "CameraGeometryBase::camera_type_");
  static_assert(offsetof(CameraGeometryBase, mask_) == 56, "CameraGeometryBase::mask_");
};

} // namespace cameras
} // namespace vk

#include "implementation/camera_geometry_base.hpp"
