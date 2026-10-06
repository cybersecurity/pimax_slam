// pimax_slam.pi.dll -- src/ceres_backend/speed_and_bias_parameter_block.hpp
//
// Port of svo_ceres_backend/include/svo/ceres_backend/speed_and_bias_parameter_block.hpp --
// upstream-identical.  Ctors in speed_and_bias_parameter_block.cpp (draft c05, object
// 0x18008FE60..0x180090180).  sizeof == 0x68 (ParameterBlock 0x20 + 9 doubles); deleting dtor
// 0x18008FEB0 uses free().
// vtable 0x1803B14D8: 0 dtor 0x18008FEB0, 1/2 parameters() 0x18002B7F0, 3/4 dimension /
//   minimalDimension 0x18008F490 (9), 5 plus 0x1800900B0, 6 plusJacobian 0x18008FEF0,
//   7 minus 0x180090050, 8 liftJacobian 0x18008FEF0 (ICF-folded with plusJacobian),
//   9/10 local parameterization accessors, 11 typeInfo 0x180090140, 12 setEstimate 0x180090110,
//   13 estimate() 0x18002B7F0 (returns &estimate_, folded with parameters()).
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <Eigen/Core>

#include "ceres_backend/parameter_block.hpp"
#include "ceres_backend/estimator_types.hpp"   // SpeedAndBias

namespace pimax {
namespace totem {
namespace ceres_backend {

/// \brief Wraps the parameter block for a speed / IMU biases estimate
class SpeedAndBiasParameterBlock : public ParameterBlock
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef SpeedAndBias estimate_t;

  static constexpr size_t c_dimension = 9;
  static constexpr size_t c_minimal_dimension = 9;

  SpeedAndBiasParameterBlock();

  /// 0x18008FE60
  SpeedAndBiasParameterBlock(const SpeedAndBias& speed_and_bias, uint64_t id);

  /// 0x18008FEB0
  virtual ~SpeedAndBiasParameterBlock() {}

  /// 0x180090110
  virtual void setEstimate(const SpeedAndBias& speed_and_bias)
  {
    estimate_ = speed_and_bias;
  }

  /// 0x18002B7F0 (ICF with parameters())
  virtual const SpeedAndBias& estimate() const { return estimate_; }

  virtual double* parameters() { return estimate_.data(); }
  virtual const double* parameters() const { return estimate_.data(); }

  /// 0x18008F490 (ICF "return 9")
  virtual size_t dimension() const { return c_dimension; }
  virtual size_t minimalDimension() const { return c_minimal_dimension; }

  /// 0x1800900B0
  virtual void plus(const double* x0, const double* Delta_Chi,
                    double* x0_plus_Delta) const
  {
    Eigen::Map<const Eigen::Matrix<double, 9, 1> > x0_(x0);
    Eigen::Map<const Eigen::Matrix<double, 9, 1> > Delta_Chi_(Delta_Chi);
    Eigen::Map<Eigen::Matrix<double, 9, 1> > x0_plus_Delta_(x0_plus_Delta);
    x0_plus_Delta_ = x0_ + Delta_Chi_;
  }

  /// 0x18008FEF0
  virtual void plusJacobian(const double* /*unused: x*/,
                            double* jacobian) const
  {
    Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor> > identity(jacobian);
    identity.setIdentity();
  }

  /// 0x180090050
  virtual void minus(const double* x0, const double* x0_plus_Delta,
                     double* Delta_Chi) const
  {
    Eigen::Map<const Eigen::Matrix<double, 9, 1> > x0_(x0);
    Eigen::Map<Eigen::Matrix<double, 9, 1> > Delta_Chi_(Delta_Chi);
    Eigen::Map<const Eigen::Matrix<double, 9, 1> > x0_plus_Delta_(x0_plus_Delta);
    Delta_Chi_ = x0_plus_Delta_ - x0_;
  }

  /// 0x18008FEF0 (folded)
  virtual void liftJacobian(const double* /*unused: x*/,
                            double* jacobian) const
  {
    Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor> > identity(jacobian);
    identity.setIdentity();
  }

  /// 0x180090140
  virtual std::string typeInfo() const
  {
    return "SpeedAndBiasParameterBlock";
  }

 private:
  SpeedAndBias estimate_;   // +0x20

  friend struct SpeedAndBiasParameterBlockLayoutCheck;
};

struct SpeedAndBiasParameterBlockLayoutCheck
{
  static_assert(sizeof(SpeedAndBiasParameterBlock) == 0x68, "");
  static_assert(offsetof(SpeedAndBiasParameterBlock, estimate_) == 0x20, "");
};

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
