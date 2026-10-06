// pimax_slam.pi.dll -- src/ceres_backend/local_parameterization_additional_interfaces.hpp
//
// Fork of svo_ceres_backend/include/svo/ceres_backend/local_parameterization_additional_interfaces.hpp.
// verify() (0x180052D90) is out-of-line in local_parameterization_additional_interfaces.cpp
// (draft c04).  Secondary-base vtable of every Pimax local parameterization (e.g. 0x1803ADF30):
//   [0] dtor  [1] Minus  [2] ComputeLiftJacobian  [3] verify
//
// Pimax change: one bool at +8 of this sub-object (+0x10 in each 24-byte local
// parameterization).  Set by Map::Map(bool) and by Map::applyFlag (0x18001A980, through
// dynamic_cast<LocalParamizationAdditionalInterfaces*>); while it is set GlobalSize() of all three
// Pimax parameterizations returns 0 (0x18001A8E0/8F0/900: `cmp [rcx+10h],0; cmovnz eax,0`).
// Name TODO(verify) (draft c01: flag_).
#pragma once

#include <cstddef>

namespace pimax {
namespace totem {
namespace ceres_backend {

/// @brief Provides some additional interfaces to ceres' LocalParamization
///        than are needed in the generic marginalisation.
class LocalParamizationAdditionalInterfaces
{
 public:
  /// @brief Trivial destructor.
  virtual ~LocalParamizationAdditionalInterfaces() = default;

  /// @brief Computes the minimal difference between a variable x and a
  ///        perturbed variable x_plus_delta
  virtual bool Minus(const double* x, const double* x_plus_delta,
                     double* delta) const = 0;

  /// @brief Computes the Jacobian from minimal space to naively
  ///        overparameterised space as used by ceres.
  virtual bool ComputeLiftJacobian(const double* x, double* jacobian) const = 0;

  /// @brief Verifies the correctness of a inplementation by means of numeric Jacobians.
  /// 0x180052D90
  virtual bool verify(const double* x_raw,
                      double purturbation_magnitude = 1.0e-6) const;

  /// [pimax-new] +8.  TODO(verify) name.
  bool use_minimal_jacobians_ = false;
};

static_assert(sizeof(LocalParamizationAdditionalInterfaces) == 0x10, "");

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
