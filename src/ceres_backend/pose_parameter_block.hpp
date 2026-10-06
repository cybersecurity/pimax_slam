// pimax_slam.pi.dll -- src/ceres_backend/pose_parameter_block.hpp
//
// Port of svo_ceres_backend/include/svo/ceres_backend/pose_parameter_block.hpp -- upstream-identical.
// Out-of-line members in pose_parameter_block.cpp (draft c05; object 0x18008D960..0x18008DB80).
// sizeof == 0x58 (ParameterBlock 0x20 + double parameters_[7]); make_shared via
// _Ref_count_obj2<PoseParameterBlock> (vtable 0x1803AF000); deleting dtor 0x18008D9C0 uses free().
// vtable 0x1803B13E0: 0 dtor 0x18008D9C0, 1/2 parameters() 0x18002B7F0, 3 dimension 0x18008DA00 (7),
//   4 minimalDimension 0x18001A920 (6), 5 plus 0x18008CC40, 6 plusJacobian 0x18008C980,
//   7 minus 0x18008CC20, 8 liftJacobian 0x18008DAF0, 9 setLocalParameterizationPtr 0x18002C7A0,
//   10 localParameterizationPtr 0x18002A6C0, 11 typeInfo 0x18008DB40, 12 setEstimate 0x18008DB00.
// (slots 5/6/7 are ICF-folded with the PoseLocalParameterization thunks: same machine code.)
// estimate() (0x18008DA10) is NOT virtual (no slot 13; called directly by the Estimator).
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <Eigen/Core>

#include "common/transformation.h"
#include "ceres_backend/parameter_block.hpp"
#include "ceres_backend/pose_local_parameterization.hpp"

namespace pimax {
namespace totem {
namespace ceres_backend {

/// \brief Wraps the parameter block for a pose estimate
class PoseParameterBlock : public ParameterBlock
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Transformation estimate_t;

  static constexpr size_t c_dimension = 7;
  static constexpr size_t c_minimal_dimension = 6;

  /// \brief Default constructor (assumes not fixed).
  PoseParameterBlock();

  /// 0x18008D960
  PoseParameterBlock(const Transformation& T_WS, uint64_t id);

  /// 0x18008D9C0 (scalar deleting)
  virtual ~PoseParameterBlock();

  /// 0x18008DB00
  virtual void setEstimate(const Transformation& T_WS);

  /// 0x18008DA10 (returns by value, non-virtual)
  Transformation estimate() const;

  virtual double* parameters() { return parameters_; }
  virtual const double* parameters() const { return parameters_; }

  /// 0x18008DA00
  virtual size_t dimension() const { return c_dimension; }
  /// 0x18001A920 (ICF "return 6")
  virtual size_t minimalDimension() const { return c_minimal_dimension; }

  // minimal internal parameterization
  virtual void plus(const double* x0, const double* Delta_Chi,
                    double* x0_plus_Delta) const
  {
    PoseLocalParameterization::plus(x0, Delta_Chi, x0_plus_Delta);
  }

  virtual void plusJacobian(const double* x0, double* jacobian) const
  {
    PoseLocalParameterization::plusJacobian(x0, jacobian);
  }

  virtual void minus(const double* x0, const double* x0_plus_Delta,
                     double* Delta_Chi) const
  {
    PoseLocalParameterization::minus(x0, x0_plus_Delta, Delta_Chi);
  }

  /// 0x18008DAF0
  virtual void liftJacobian(const double* x0, double* jacobian) const
  {
    PoseLocalParameterization::liftJacobian(x0, jacobian);
  }

  /// 0x18008DB40
  virtual std::string typeInfo() const
  {
    return "PoseParameterBlock";
  }

 private:
  double parameters_[c_dimension];   // +0x20  p.xyz, q.xyzw

  friend struct PoseParameterBlockLayoutCheck;
};

struct PoseParameterBlockLayoutCheck
{
  static_assert(sizeof(PoseParameterBlock) == 0x58, "");
  static_assert(offsetof(PoseParameterBlock, parameters_) == 0x20, "");
};

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
