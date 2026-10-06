// pimax_slam.pi.dll -- src/ceres_backend/pose_error.hpp
//
// Port of svo_ceres_backend/include/svo/ceres_backend/pose_error.hpp.  Bodies in pose_error.cpp
// (draft c05; object 0x18008AD40..0x18008C980).
// Pimax: position (scalar-weighted) + yaw prior: residual = [(p_meas-p)*s00, 0, 0, 2*dq.z*s55];
// information_ and covariance_ removed (setInformation 0x18008C820 does not store them).
// Layout (sizeof == 0x1A0: make_shared<PoseError> allocates 0x1B0, callers 0x180022050 /
// 0x18002C7B0; deleting dtor 0x18008BBE0 uses free()):
//   +0x00 ceres::SizedCostFunction<6,7>  (vtable 0x1803B1318: dtor 0x18008BBE0, Evaluate 0x18003AE00)
//   +0x28 ErrorInterface (vtable 0x1803B1330), +0x30 bool use_minimal_jacobians_
//   +0x40 Transformation measurement_          (q xyzw +0x40, p +0x60)       (sure)
//   +0x80 Matrix<double,6,6> square_root_information_                        (sure)
#pragma once

#include <cstddef>

#include <Eigen/Core>
#include <ceres/sized_cost_function.h>

#include "common/transformation.h"
#include "ceres_backend/error_interface.hpp"

namespace pimax {
namespace totem {
namespace ceres_backend {

/// \brief Absolute error of a pose.  Pimax: position (scalar-weighted) + yaw only.
class PoseError : public ::ceres::SizedCostFunction<6, 7>,
    public ErrorInterface
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef ::ceres::SizedCostFunction<6, 7> base_t;
  static const int kNumResiduals = 6;
  typedef Eigen::Matrix<double, 6, 6> information_t;
  typedef Eigen::Matrix<double, 6, 6> covariance_t;

  PoseError() = default;

  /// 0x18008BAC0
  PoseError(const Transformation& measurement,
            const Eigen::Matrix<double, 6, 6>& information);

  virtual ~PoseError() = default;   // 0x18008BBE0 / thunk 0x18008BBD0

  void setMeasurement(const Transformation& measurement)
  {
    measurement_ = measurement;
  }

  /// 0x18008C820  upstream-modified (information_/covariance_ not stored).
  void setInformation(const information_t& information);

  const Transformation& measurement() const { return measurement_; }

  /// ICF-folded 0x18003AE00 (shared with ImuError / SpeedAndBiasError).
  virtual bool Evaluate(double const* const * parameters, double* residuals,
                        double** jacobians) const;

  /// 0x18008BC30  upstream-modified.
  virtual bool EvaluateWithMinimalJacobians(double const* const * parameters,
                                            double* residuals,
                                            double** jacobians,
                                            double** jacobians_minimal) const;

  /// ICF 0x18001A920 (returns 6)
  size_t residualDim() const
  {
    return kNumResiduals;
  }

  /// ICF 0x180014980
  size_t parameterBlocks() const
  {
    return parameter_block_sizes().size();
  }

  /// ICF 0x180014950
  size_t parameterBlockDim(size_t parameter_block_idx) const
  {
    return base_t::parameter_block_sizes().at(parameter_block_idx);
  }

  /// 0x18008C970: returns 4.
  virtual ErrorType typeInfo() const
  {
    return ErrorType::kPoseError;
  }

 protected:
  Transformation measurement_;                  ///< +0x40 The pose measurement.
  information_t square_root_information_;       ///< +0x80 The 6x6 square root information matrix.

 private:
  friend struct PoseErrorLayoutCheck;
};

struct PoseErrorLayoutCheck
{
  static_assert(sizeof(PoseError) == 0x1A0, "");
  static_assert(offsetof(PoseError, measurement_) == 0x40, "");
  static_assert(offsetof(PoseError, square_root_information_) == 0x80, "");
};

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
