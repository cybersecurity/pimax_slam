// pimax_slam.pi.dll -- src/ceres_backend/error_interface.hpp
//
// Pimax fork of svo_ceres_backend/include/svo/ceres_backend/error_interface.hpp
// (namespace svo::ceres_backend -> pimax::totem::ceres_backend).
//
// Differences from upstream (binary evidence, notes c00 §4.1, c02, c05 §4.2):
//   * ErrorType gained kGroundPlaneError (= 7; GroundPlaneError::typeInfo 0x18002DEF0).
//   * kErrorToStr is a std::unordered_map (initializer 0x180001030 in error_interface.cpp,
//     FNV-1a hash over the single uint8 byte); upstream uses std::map.
//   * ErrorInterface has ONE data member: a bool at +8 of the sub-object (= +0x30 in every
//     concrete error, whose ceres::CostFunction base occupies +0x00..+0x28).  The (inlined)
//     ErrorInterface ctor clears it, every Evaluate() tests it:
//        ImuError/PoseError/SpeedAndBiasError::Evaluate (ICF-folded 0x18003AE00):
//            flag ? EvaluateWithMinimalJacobians(p, r, nullptr, J) : (p, r, J, nullptr)
//        ReprojectionError::Evaluate 0x18000C460: flag ? EvaluateMinimal(p, r, J) : ...
//        MarginalizationError::Evaluate 0x180078510: flag ? EvaluateLocal(p, r, J) : ...
//     Written by ceres_backend::Map::applyFlag (0x18001A980) together with the bool of every
//     LocalParamizationAdditionalInterfaces, i.e. "ceres receives tangent-space Jacobians".
//     Name TODO(verify) (drafts: c00 use_minimal_jacobian_, c01 flag_, c03/c04/c05
//     use_minimal_jacobians_ -> the majority name is used).
//
// Vtable (slot order as upstream): 0 dtor, 1 residualDim, 2 parameterBlocks,
// 3 parameterBlockDim, 4 EvaluateWithMinimalJacobians, 5 typeInfo.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>

#include <Eigen/Core>

namespace pimax {
namespace totem {
namespace ceres_backend {

enum class ErrorType : uint8_t
{
  kHomogeneousPointError,   // 0
  kReprojectionError,       // 1  (ReprojectionError::typeInfo 0x180016740)
  kSpeedAndBiasError,       // 2  (0x18008FE50)
  kMarginalizationError,    // 3  (0x180088D00)
  kPoseError,               // 4  (0x18008C970)
  kIMUError,                // 5  (0x180050EE0)
  kRelativePoseError,       // 6
  kGroundPlaneError         // 7  [pimax-new] (0x18002DEF0)
};

/// 0x18047D9F0, defined in error_interface.cpp (dynamic initializer 0x180001030).
extern const std::unordered_map<ErrorType, std::string> kErrorToStr;

/// @brief Simple interface class the errors implemented here should inherit from.
class ErrorInterface
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ErrorInterface() = default;
  virtual ~ErrorInterface() = default;                                        // slot 0

  /// @brief Get dimension of residuals.
  virtual size_t residualDim() const = 0;                                     // slot 1
  /// @brief Get the number of parameter blocks this is connected to.
  virtual size_t parameterBlocks() const = 0;                                 // slot 2
  /// @brief Get the dimension of a parameter block this is connected to.
  virtual size_t parameterBlockDim(size_t parameter_block_idx) const = 0;     // slot 3
  /// @brief This evaluates the error term and additionally computes
  ///        the Jacobians in the minimal internal representation.
  virtual bool EvaluateWithMinimalJacobians(
      double const* const* parameters, double* residuals, double** jacobians,
      double** jacobians_minimal) const = 0;                                  // slot 4
  /// @brief Residual block type as string
  virtual ErrorType typeInfo() const = 0;                                     // slot 5

  /// [pimax-new] +8.  When set, Evaluate() hands ceres the Jacobians w.r.t. the minimal
  /// (local) parameterisation.  Written by Map::applyFlag (0x18001A980).  TODO(verify) name.
  bool use_minimal_jacobians_ = false;
};

static_assert(sizeof(ErrorInterface) == 0x10, "ErrorInterface size");

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
