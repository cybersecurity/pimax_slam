// pimax_slam.pi.dll -- src/ceres_backend/gravity_parameter_block.hpp
//
// [pimax-new] Parameter block holding the gravity vector g_W (3 parameters, 2-D local
// parameterization on the sphere of radius |g|, see GravityLocalParameterization).  It is the 5th
// parameter block of every ImuError (id (uint64_t)-2 = Estimator::gravity_parameter_block_id_).
// All member functions are header-inline; vtable 0x1803AEBB0 and the out-of-line copies are in
// estimator.obj (first user: Estimator::addStates 0x180026100 via make_shared, ctrl block vtable
// 0x1803AEFD8, 0x48 bytes -> sizeof 0x38; deleting dtor 0x180024C80 uses free()).
//
// vtable 0x1803AEBB0 (13 slots):
//   [0]  0x180024C80 ~GravityParameterBlock
//   [1]  0x18002B7F0 parameters() const   [2] 0x18002B7F0 parameters()   (ICF)
//   [3]  0x18001A910 dimension() -> 3     [4] 0x180014A10 minimalDimension() -> 2 (ICF)
//   [5]  0x18001A960 plus                 (= GravityLocalParameterization::Plus, ICF)
//   [6]  0x18002B800 plusJacobian         [7] 0x18001A930 minus (ICF)
//   [8]  0x18002A560 liftJacobian
//   [9]  0x18002C7A0 setLocalParameterizationPtr   [10] 0x18002A6C0 localParameterizationPtr
//   [11] 0x18002CEC0 typeInfo -> "GravityParameterBlock"
//   [12] 0x18002C510 setEstimate(const Eigen::Vector3d&)
//   (no virtual estimate() -- the accessor is non-virtual)
// Layout: ParameterBlock (0x20), +0x20 Eigen::Vector3d estimate_.  sizeof 0x38.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <Eigen/Core>

#include "ceres_backend/gravity_local_parameterization.hpp"
#include "ceres_backend/parameter_block.hpp"

namespace pimax {
namespace totem {
namespace ceres_backend {

class GravityParameterBlock : public ParameterBlock
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Eigen::Vector3d estimate_t;

  static constexpr size_t c_dimension = 3;
  static constexpr size_t c_minimal_dimension = 2;

  /// inlined into make_shared in addStates (0x180026100).  TODO(verify) argument order.
  GravityParameterBlock(const Eigen::Vector3d& gravity, uint64_t id)
  {
    setEstimate(gravity);
    setId(id);
    setFixed(false);
  }

  virtual ~GravityParameterBlock() {}   // 0x180024C80

  // 0x18002C510
  virtual void setEstimate(const Eigen::Vector3d& gravity)
  {
    estimate_ = gravity;
  }

  /// non-virtual (no vtable slot); used by addStates as `->estimate()`.
  const Eigen::Vector3d& estimate() const { return estimate_; }

  // 0x18002B7F0 (ICF)
  virtual double* parameters() { return estimate_.data(); }
  virtual const double* parameters() const { return estimate_.data(); }

  // 0x18001A910 / 0x180014A10 (ICF)
  virtual size_t dimension() const { return c_dimension; }
  virtual size_t minimalDimension() const { return c_minimal_dimension; }

  // 0x18001A960
  virtual void plus(const double* x0, const double* Delta_Chi,
                    double* x0_plus_Delta) const
  {
    GravityLocalParameterization::plus(x0, Delta_Chi, x0_plus_Delta);
  }

  // 0x18002B800
  virtual void plusJacobian(const double* x0, double* jacobian) const
  {
    GravityLocalParameterization::plusJacobian(x0, jacobian);
  }

  // 0x18001A930
  virtual void minus(const double* x0, const double* x0_plus_Delta,
                     double* Delta_Chi) const
  {
    GravityLocalParameterization::minus(x0, x0_plus_Delta, Delta_Chi);
  }

  // 0x18002A560
  virtual void liftJacobian(const double* x0, double* jacobian) const
  {
    GravityLocalParameterization::liftJacobian(x0, jacobian);
  }

  // 0x18002CEC0
  virtual std::string typeInfo() const
  {
    return "GravityParameterBlock";
  }

 private:
  Eigen::Vector3d estimate_;   // +0x20

  friend struct GravityParameterBlockLayoutCheck;
};

struct GravityParameterBlockLayoutCheck
{
  static_assert(sizeof(GravityParameterBlock) == 0x38, "");
  static_assert(offsetof(GravityParameterBlock, estimate_) == 0x20, "");
};

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
