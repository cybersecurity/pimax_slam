// pimax_slam.pi.dll -- src/ceres_backend/speed_and_bias_error.hpp
//
// Port of svo_ceres_backend/include/svo/ceres_backend/speed_and_bias_error.hpp.  Bodies in
// speed_and_bias_error.cpp (draft c05; object 0x18008DB80..0x18008FE60).
// Pimax: each 3-block weighted with a single diagonal entry of sqrt_info; J = -sqrt_info.
// Layout (sizeof 0x590 = 1424; make_shared<SpeedAndBiasError> in 0x180026100 allocates 0x5A0;
// deleting dtor 0x18008EC20 uses free()):
//   +0x000 ceres::SizedCostFunction<9,9>   (vtable 0x1803B1480: dtor 0x18008EC20, Evaluate 0x18003AE00)
//   +0x028 ErrorInterface (vtable 0x1803B1498), +0x030 bool use_minimal_jacobians_
//   +0x038 SpeedAndBias measurement_             (Matrix<double,9,1>, unaligned)      (sure)
//   +0x080 information_t information_            (9x9)                                (sure)
//   +0x308 information_t square_root_information_ (9x9)                               (sure)
#pragma once

#include <cstddef>

#include <Eigen/Core>
#include <ceres/sized_cost_function.h>

#include "ceres_backend/error_interface.hpp"
#include "ceres_backend/estimator_types.hpp"   // SpeedAndBias

namespace pimax {
namespace totem {
namespace ceres_backend {

class SpeedAndBiasError : public ::ceres::SizedCostFunction<9, 9>,
    public ErrorInterface
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef ::ceres::SizedCostFunction<9, 9> base_t;
  static const int kNumResiduals = 9;
  typedef Eigen::Matrix<double, 9, 9> information_t;
  typedef Eigen::Matrix<double, 9, 9> covariance_t;

  SpeedAndBiasError() = default;

  /// 0x18008E8F0
  SpeedAndBiasError(const SpeedAndBias& measurement,
                    const information_t& information);

  /// 0x18008E980  (Estimator::addStates passes speed_variance = 1.0 (0x1803ADDD0))
  SpeedAndBiasError(const SpeedAndBias& measurement, double speed_variance,
                    double gyr_bias_variance, double acc_bias_variance);

  virtual ~SpeedAndBiasError() = default;   // 0x18008EC20 / thunk 0x18008EC08

  void setMeasurement(const SpeedAndBias& measurement)
  {
    measurement_ = measurement;
  }

  /// 0x18008FB10
  void setInformation(const information_t& information);

  const SpeedAndBias& measurement() const { return measurement_; }
  const information_t& information() const { return information_; }

  /// ICF-folded 0x18003AE00
  virtual bool Evaluate(double const* const * parameters, double* residuals,
                        double** jacobians) const;

  /// 0x18008EC70  upstream-modified.
  virtual bool EvaluateWithMinimalJacobians(double const* const * parameters,
                                            double* residuals,
                                            double** jacobians,
                                            double** jacobians_minimal) const;

  /// 0x18008F490 (returns 9; ICF with SpeedAndBiasParameterBlock::dimension)
  size_t residualDim() const
  {
    return kNumResiduals;
  }

  size_t parameterBlocks() const
  {
    return parameter_block_sizes().size();
  }

  size_t parameterBlockDim(size_t parameter_block_idx) const
  {
    return base_t::parameter_block_sizes().at(parameter_block_idx);
  }

  /// 0x18008FE50: returns 2.
  virtual ErrorType typeInfo() const
  {
    return ErrorType::kSpeedAndBiasError;
  }

 protected:
  SpeedAndBias measurement_;                  ///< +0x38 The (9D) measurement.
  information_t information_;                 ///< +0x80 The 9x9 information matrix.
  information_t square_root_information_;     ///< +0x308 The 9x9 square root information matrix.

 private:
  friend struct SpeedAndBiasErrorLayoutCheck;
};

struct SpeedAndBiasErrorLayoutCheck
{
  static_assert(sizeof(SpeedAndBiasError) == 0x590, "");
  static_assert(offsetof(SpeedAndBiasError, measurement_) == 0x38, "");
  static_assert(offsetof(SpeedAndBiasError, information_) == 0x80, "");
  static_assert(offsetof(SpeedAndBiasError, square_root_information_) == 0x308, "");
};

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
