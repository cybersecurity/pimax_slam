// pimax_slam.pi.dll -- src/ceres_backend/reprojection_error.hpp (from draft c00; header-only)
// Pimax variant of svo_ceres_backend/reprojection_error.hpp
//
// Header-only class: every member function is an inline/COMDAT function; the linker placed them in
// the first object that used them (ceres_backend_interface.obj):
//   ctor 0x180008F50, dtor 0x18000C020 (+thunk 0x18000BE8C), Evaluate 0x18000C460,
//   EvaluateWithMinimalJacobians 0x18000C490, EvaluateMinimal 0x18000E0A0      <- chunk c00
//   setInformation 0x180015C30, getters 0x180012C00..0x180016B50, residualDim/... <- next chunk
//
// Layout (sizeof = 0xF0 = 240, EIGEN_MAKE_ALIGNED_OPERATOR_NEW => free() in deleting dtor):
//   +0x00 vptr (ceres::CostFunction primary)            sure
//   +0x08 std::vector<int32_t> parameter_block_sizes_    sure ({7,3,7})
//   +0x20 int num_residuals_                             sure (2)
//   +0x28 vptr (ErrorInterface)                          sure
//   +0x30 bool ErrorInterface::use_minimal_jacobians_     sure (cleared in ctor, tested in Evaluate)
//   +0x38 int64_t cam_index_                             sure (written by Estimator::addObservation
//                                                        0x180010C20 after make_shared; not set by
//                                                        the ctor; c01 notes)  name TODO(verify)
//   +0x40 measurement_t measurement_                     sure (setMeasurement / measurement())
//   +0x50 mutable measurement_t weighted_error_          sure (written by both evaluate functions)
//   +0x60 mutable measurement_t error_                   sure
//   +0x70 CameraConstPtr camera_geometry_                sure (shared_ptr, project3 = vtbl+0x18)
//   +0x80 covariance_t information_                      sure
//   +0xA0 covariance_t square_root_information_          sure (= LLT(information).matrixL()^T)
//   +0xC0 covariance_t covariance_                       sure (= information.inverse())
//   +0xE0 bool disabled_                                 sure
//   +0xE1 bool point_constant_                           sure (initialised, never read)
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include <ceres/ceres.h>

#include "ceres_backend/error_interface.hpp"
#include "ceres_backend/pose_local_parameterization.hpp"
#include "ceres_backend/reprojection_error_base.hpp"
#include "common/camera.h"  // CameraConstPtr = std::shared_ptr<const vk::cameras::CameraGeometryBase>

namespace pimax {
namespace totem {
namespace ceres_backend {

class ReprojectionError : public ReprojectionErrorBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef ceres::SizedCostFunction<2, 7, 3, 7> base_t;
  static const int kNumResiduals = 2;
  typedef Eigen::Vector2d keypoint_t;

  ReprojectionError(){}

  // 0x180008F50
  // Pimax: the loop-closing BA (0x18017EB00) calls make_shared<ReprojectionError>(cam, obs) and
  // the Identity information is built inside the make_shared instantiation -> default argument
  // (c16 notes).
  ReprojectionError(CameraConstPtr cameraGeometry,
                    const measurement_t& measurement,
                    const covariance_t& information = covariance_t::Identity());

  // 0x18000C020
  virtual ~ReprojectionError()
  {
  }

  // 0x180015DB0
  virtual void setMeasurement(const measurement_t& measurement)
  {
    measurement_ = measurement;
  }

  // upstream had CHECK(camera_geometry != nullptr) here; the Pimax build has no check
  // (inlined into the ctor: plain shared_ptr copy-assignment).
  void setCameraGeometry(
      CameraConstPtr camera_geometry)
  {
    camera_geometry_ = camera_geometry;
  }

  // 0x180015C30
  virtual void setInformation(const covariance_t& information);

  virtual const measurement_t& measurement() const  // 0x180014670
  {
    return measurement_;
  }

  virtual const measurement_t& weightedError() const  // 0x180016B50
  {
    return weighted_error_;
  }

  virtual const measurement_t& error() const  // 0x180013020
  {
    return error_;
  }

  virtual const covariance_t& information() const  // 0x180013030
  {
    return information_;
  }

  virtual const covariance_t& covariance() const  // 0x180012C00
  {
    return covariance_;
  }

  // 0x18000C460
  virtual bool Evaluate(double const* const * parameters, double* residuals,
                        double** jacobians) const;

  // 0x18000C490 (ErrorInterface slot 4). Weights with the scalar square_root_information_(0,0).
  virtual bool EvaluateWithMinimalJacobians(double const* const * parameters,
                                            double* residuals,
                                            double** jacobians,
                                            double** jacobians_minimal) const;

  // 0x18000E0A0 (primary vtable slot 9, Pimax). Minimal jacobians only, full 2x2
  // square-root-information weighting. TODO(verify) name.
  virtual bool EvaluateMinimal(double const* const * parameters,
                               double* residuals,
                               double** jacobians_minimal) const;

  inline void setDisabled(const bool disabled)
  {
    disabled_ = disabled;
  }

  inline void setPointConstant(const bool point_constant)
  {
    point_constant_ = point_constant;
  }

  size_t residualDim() const  // 0x180014A10 (COMDAT-folded with a "return 2" function)
  {
    return kNumResiduals;
  }

  size_t parameterBlocks() const  // 0x180014980
  {
    return parameter_block_sizes().size();
  }

  size_t parameterBlockDim(size_t parameter_block_idx) const  // 0x180014950
  {
    return base_t::parameter_block_sizes().at(parameter_block_idx);
  }

  virtual ErrorType typeInfo() const  // 0x180016740
  {
    return ErrorType::kReprojectionError;
  }

 public:
  /// [pimax-new] +0x38: camera index of the observation (Estimator::addObservation stores
  /// Frame::nframe_index_ here, 0x180010C20).  Not initialised by the ctor.  Name TODO(verify).
  int64_t cam_index_;                            // +0x38

 protected:
  measurement_t measurement_;                    // +0x40
  mutable measurement_t weighted_error_;         // +0x50 (Pimax)
  mutable measurement_t error_;                  // +0x60 (Pimax)
  CameraConstPtr camera_geometry_;               // +0x70
  covariance_t information_;                     // +0x80
  covariance_t square_root_information_;         // +0xA0
  covariance_t covariance_;                      // +0xC0
  bool disabled_ = false;                        // +0xE0
  bool point_constant_ = false;                  // +0xE1

  friend struct ReprojectionErrorLayoutCheck;
};

struct ReprojectionErrorLayoutCheck
{
  static_assert(sizeof(ReprojectionError) == 0xF0, "");
  static_assert(offsetof(ReprojectionError, cam_index_) == 0x38, "");
  static_assert(offsetof(ReprojectionError, measurement_) == 0x40, "");
  static_assert(offsetof(ReprojectionError, weighted_error_) == 0x50, "");
  static_assert(offsetof(ReprojectionError, error_) == 0x60, "");
  static_assert(offsetof(ReprojectionError, camera_geometry_) == 0x70, "");
  static_assert(offsetof(ReprojectionError, information_) == 0x80, "");
  static_assert(offsetof(ReprojectionError, square_root_information_) == 0xA0, "");
  static_assert(offsetof(ReprojectionError, covariance_) == 0xC0, "");
  static_assert(offsetof(ReprojectionError, disabled_) == 0xE0, "");
  static_assert(offsetof(ReprojectionError, point_constant_) == 0xE1, "");
};

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax

#include "ceres_backend/reprojection_error_impl.hpp"
