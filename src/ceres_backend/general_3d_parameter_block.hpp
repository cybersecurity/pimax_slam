// pimax_slam.pi.dll -- src/ceres_backend/general_3d_parameter_block.hpp
//
// [pimax-new] Euclidean 3-D parameter block used for landmarks (replaces upstream's
// HomogeneousPointParameterBlock; added to the map with the Trivial parameterization by
// Estimator::addLandmark 0x180025C80).  A 3-D copy of upstream homogeneous_point_parameter_block
// with identity plus/minus.  All members header-inline; emitted in estimator.obj (notes c03 §1b).
//
// vtable 0x1803AF330 (14 slots):
//   [0]  0x18002CFF0 ~General3DParameterBlock (scalar deleting)
//   [1]  0x18002B7F0 parameters() const     [2] 0x18002B7F0 parameters()   (ICF)
//   [3]  0x18001A910 dimension() -> 3       [4] 0x18001A910 minimalDimension() -> 3 (ICF)
//   [5]  0x18002D090 plus                   [6] 0x18002D030 plusJacobian
//   [7]  0x18002D060 minus                  [8] 0x18002D030 liftJacobian (ICF with [6])
//   [9]  0x18002C7A0 setLocalParameterizationPtr   [10] 0x18002A6C0 localParameterizationPtr
//   [11] 0x18002D0D0 typeInfo -> "General3DParameterBlock"
//   [12] 0x18002D0C0 setEstimate(const Eigen::Vector3d&)
//   [13] 0x18002B7F0 estimate() -> const Eigen::Vector3d&  (ICF with parameters())
// Layout: ParameterBlock (0x20), +0x20 Eigen::Vector3d estimate_, +0x38 bool initialized_;
// sizeof 0x40 (make_shared in addLandmark / 0x18017E7D0).
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <Eigen/Core>

#include "ceres_backend/parameter_block.hpp"

namespace pimax {
namespace totem {
namespace ceres_backend {

class General3DParameterBlock : public ParameterBlock
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // deleting dtor 0x18002CFF0 uses free()

  typedef Eigen::Vector3d estimate_t;

  static constexpr size_t c_dimension = 3;
  static constexpr size_t c_minimal_dimension = 3;

  // 0x18002CFB0 -- callers: addLandmark 0x180025C80 and 0x18017E7D0 (both pass true).
  // Store order in the binary: base ctor, estimate_, id_, fixed_=false, initialized_ (last).
  General3DParameterBlock(const Eigen::Vector3d& point, uint64_t id,
                          bool initialized = true)
  {
    setEstimate(point);
    setId(id);
    setInitialized(initialized);
    setFixed(false);
  }

  // 0x18002CFF0
  virtual ~General3DParameterBlock() {}

  // 0x18002D0C0
  virtual void setEstimate(const Eigen::Vector3d& point) { estimate_ = point; }

  void setInitialized(bool initialized) { initialized_ = initialized; }

  // 0x18002B7F0 (ICF)
  virtual const Eigen::Vector3d& estimate() const { return estimate_; }

  bool initialized() const { return initialized_; }

  // 0x18002B7F0 (ICF)
  virtual double* parameters() { return estimate_.data(); }
  virtual const double* parameters() const { return estimate_.data(); }

  // 0x18001A910 (ICF)
  virtual size_t dimension() const { return c_dimension; }
  virtual size_t minimalDimension() const { return c_minimal_dimension; }

  // 0x18002D090
  virtual void plus(const double* x0, const double* Delta_Chi,
                    double* x0_plus_Delta) const
  {
    Eigen::Map<const Eigen::Vector3d> x0_(x0);
    Eigen::Map<const Eigen::Vector3d> delta_(Delta_Chi);
    Eigen::Map<Eigen::Vector3d> x0_plus_Delta_(x0_plus_Delta);
    x0_plus_Delta_ = x0_ + delta_;
  }

  // 0x18002D030 (shared with liftJacobian): 3x3 identity
  virtual void plusJacobian(const double* /*x0*/, double* jacobian) const
  {
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J(jacobian);
    J.setIdentity();
  }

  // 0x18002D060
  virtual void minus(const double* x0, const double* x0_plus_Delta,
                     double* Delta_Chi) const
  {
    Eigen::Map<const Eigen::Vector3d> x0_(x0);
    Eigen::Map<const Eigen::Vector3d> x0_plus_Delta_(x0_plus_Delta);
    Eigen::Map<Eigen::Vector3d> delta_(Delta_Chi);
    delta_ = x0_plus_Delta_ - x0_;
  }

  // 0x18002D030 (ICF with plusJacobian)
  virtual void liftJacobian(const double* /*x0*/, double* jacobian) const
  {
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J(jacobian);
    J.setIdentity();
  }

  // 0x18002D0D0
  virtual std::string typeInfo() const
  {
    return "General3DParameterBlock";
  }

 private:
  Eigen::Vector3d estimate_;   // +0x20
  bool initialized_;           // +0x38

  friend struct General3DParameterBlockLayoutCheck;
};

struct General3DParameterBlockLayoutCheck
{
  static_assert(sizeof(General3DParameterBlock) == 0x40, "");
  static_assert(offsetof(General3DParameterBlock, estimate_) == 0x20, "");
  static_assert(offsetof(General3DParameterBlock, initialized_) == 0x38, "");
};

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
