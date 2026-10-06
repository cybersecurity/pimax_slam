// pimax_slam.pi.dll -- src/ceres_backend/parameter_block.hpp
//
// Upstream-identical (svo_ceres_backend/include/svo/ceres_backend/parameter_block.hpp),
// namespace pimax::totem::ceres_backend.  (notes c02 "ParameterBlock", c03 §2f, c05 §4.3)
//
// Layout (sure): +0x00 vptr, +0x08 uint64_t id_, +0x10 bool fixed_,
//                +0x18 const ceres::LocalParameterization* local_parameterization_ptr_,
//                +0x20 derived data.
// Vtable (MSVC places overloads in reverse declaration order):
//   0 dtor, 1 parameters() const, 2 parameters(), 3 dimension(), 4 minimalDimension(),
//   5 plus, 6 plusJacobian, 7 minus, 8 liftJacobian,
//   9 setLocalParameterizationPtr (0x18002C7A0), 10 localParameterizationPtr (0x18002A6C0;
//   also ICF-used by ceres' BlockSparseMatrix vtable), 11 typeInfo,
//   then derived virtuals (12 setEstimate, 13 estimate() where virtual).
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <ceres/local_parameterization.h>

namespace pimax {
namespace totem {
namespace ceres_backend {

/// @brief Base class providing the interface for parameter blocks.
class ParameterBlock
{
 public:
  /// @brief Default constructor, assumes not fixed and no local parameterisation.
  ParameterBlock()
      : id_(0),
        fixed_(false),
        local_parameterization_ptr_(0)
  {}

  /// @brief Trivial destructor.
  virtual ~ParameterBlock() = default;

  /// @name Setters
  /// @{
  /// @brief Set parameter block ID
  void setId(uint64_t id) { id_ = id; }

  /// @brief Whether or not this should be optimised at all.
  void setFixed(bool fixed) { fixed_ = fixed; }
  /// @}

  /// @name Getters
  /// @{
  /// @brief Get parameter values.
  virtual double* parameters() = 0;

  /// @brief Get parameter values.
  virtual const double* parameters() const = 0;

  /// @brief Get parameter block ID.
  uint64_t id() const { return id_; }

  /// @brief Get the dimension of the parameter block.
  virtual size_t dimension() const = 0;

  /// @brief The dimension of the internal parameterisation (minimal representation).
  virtual size_t minimalDimension() const = 0;

  /// @brief Whether or not this is optimised at all.
  bool fixed() const { return fixed_; }
  /// @}

  // minimal internal parameterization
  // x0_plus_Delta=Delta_Chi[+]x0
  virtual void plus(const double* x0, const double* Delta_Chi,
                    double* x0_plus_Delta) const = 0;

  /// @brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  virtual void plusJacobian(const double* x0, double* jacobian) const = 0;

  // Delta_Chi=x0_plus_Delta[-]x0
  virtual void minus(const double* x0, const double* x0_plus_Delta,
                     double* Delta_Chi) const = 0;

  /// @brief Computes the Jacobian from minimal space to naively overparameterised space.
  virtual void liftJacobian(const double* x0, double* jacobian) const = 0;

  /// @name Local parameterization
  /// @{
  // 0x18002C7A0
  virtual void setLocalParameterizationPtr(
      const ceres::LocalParameterization* local_parameterization_ptr)
  {
    local_parameterization_ptr_ = local_parameterization_ptr;
  }

  // 0x18002A6C0
  virtual const ceres::LocalParameterization* localParameterizationPtr() const
  {
    return local_parameterization_ptr_;
  }
  /// @}

  /// @brief Return parameter block type as string
  virtual std::string typeInfo() const = 0;

 protected:
  uint64_t id_;                                                     // +0x08
  bool fixed_;                                                      // +0x10
  const ceres::LocalParameterization* local_parameterization_ptr_;  // +0x18

 private:
  friend struct ParameterBlockLayoutCheck;
};

struct ParameterBlockLayoutCheck
{
  static_assert(sizeof(ParameterBlock) == 0x20, "ParameterBlock size");
  static_assert(offsetof(ParameterBlock, id_) == 0x08, "");
  static_assert(offsetof(ParameterBlock, fixed_) == 0x10, "");
  static_assert(offsetof(ParameterBlock, local_parameterization_ptr_) == 0x18, "");
};

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
