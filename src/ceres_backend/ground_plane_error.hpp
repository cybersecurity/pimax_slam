// pimax_slam.pi.dll -- src/ceres_backend/ground_plane_error.hpp  (from draft c03; header-only)
//
// [pimax-new] Relative ground-plane constraint between two poses (PoseParameterBlock, 7 params
// [p, q(x,y,z,w)]):  r = w * (R_W1 * n_1 - R_W0 * n_0)  with w = 1 / max(|sigma|, 1e-6).
// The plane normal observed in body frame 0 and in body frame 1 must map to the same world
// direction.  Created by Estimator::addGroundPlaneError (0x180025930, c02) via make_shared
// (ctrl block vtable 0x1803AF0C8, 0x80 bytes -> sizeof(GroundPlaneError) == 0x70).
// All members header-inline (emitted in estimator.obj).
//
// vtables:
//   0x1803AF3E0 (ceres::CostFunction part): [0] 0x180024CC0 dtor (c02)  [1] 0x18002D350 Evaluate
//   0x1803AF3F8 (ErrorInterface part):      [0] 0x18002D338 dtor thunk (this-0x28)
//     [1] 0x18001A910 residualDim -> 3  [2] 0x180014980 parameterBlocks  [3] 0x180014950
//     parameterBlockDim  [4] 0x18002D390 EvaluateWithMinimalJacobians  [5] 0x18002DEF0 typeInfo -> 7
//
// Layout: +0x00 CostFunction vptr, +0x08 parameter_block_sizes_ {7,7}, +0x20 num_residuals_ = 3,
//         +0x28 ErrorInterface vptr, +0x30 bool ErrorInterface::use_minimal_jacobians_ (Pimax;
//         name TODO(verify), see ImuError), +0x38 Vector3d normal_0_, +0x50 Vector3d normal_1_,
//         +0x68 double weight_.
#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/sized_cost_function.h>

#include "ceres_backend/error_interface.hpp"
#include "ceres_backend/estimator_types.hpp"          // isFinite (0x180027A20)
#include "ceres_backend/pose_local_parameterization.hpp"
#include "ceres_backend/matrix_operations.hpp"        // skewSymmetric (0x180016490)

namespace pimax {
namespace totem {
namespace ceres_backend {

class GroundPlaneError :
    public ::ceres::SizedCostFunction<3 /* number of residuals */,
                                      7 /* PoseParameterBlock 0 */,
                                      7 /* PoseParameterBlock 1 */>,
    public ErrorInterface
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef ::ceres::SizedCostFunction<3, 7, 7> base_t;
  static const int kNumResiduals = 3;

  // 0x18002D110
  GroundPlaneError(const Eigen::Vector3d& normal_0, const Eigen::Vector3d& normal_1,
                   double sigma)
    : normal_0_(0.0, 0.0, 1.0),
      normal_1_(0.0, 0.0, 1.0),
      weight_(1.0)
  {
    const double norm_0 = normal_0.norm();
    if (norm_0 > 1e-12 && isFinite(normal_0))
    {
      normal_0_ = normal_0 / norm_0;
    }
    const double norm_1 = normal_1.norm();
    if (norm_1 > 1e-12 && isFinite(normal_1))
    {
      normal_1_ = normal_1 / norm_1;
    }
    // maxsd(1e-6, |sigma|) -> std::max(|sigma|, 1e-6) (NaN propagates)
    weight_ = 1.0 / std::max(std::fabs(sigma), 1e-6);
  }

  virtual ~GroundPlaneError() {}   // 0x180024CC0 (c02)

  // 0x18002D350 -- devirtualised direct call to 0x18002D390 (class/method treated as final,
  // TODO(verify)); same switch as ImuError::Evaluate 0x18003AE00.
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const
  {
    if (use_minimal_jacobians_)
    {
      return GroundPlaneError::EvaluateWithMinimalJacobians(parameters, residuals, nullptr,
                                                            jacobians);
    }
    return GroundPlaneError::EvaluateWithMinimalJacobians(parameters, residuals, jacobians,
                                                          nullptr);
  }

  // 0x18002D390
  virtual bool EvaluateWithMinimalJacobians(double const* const* parameters,
                                            double* residuals,
                                            double** jacobians,
                                            double** jacobians_minimal) const
  {
    // rotate both normals into the world frame (quaternions re-normalised: 0x18002DDA0 =
    // Map<const Quaterniond>::normalized(), vectorised squaredNorm (x²+z²)+(y²+w²))
    const Eigen::Quaterniond q_W0 =
        Eigen::Map<const Eigen::Quaterniond>(parameters[0] + 3).normalized();
    const Eigen::Vector3d n_W0 = q_W0 * normal_0_;
    const Eigen::Quaterniond q_W1 =
        Eigen::Map<const Eigen::Quaterniond>(parameters[1] + 3).normalized();
    const Eigen::Vector3d n_W1 = q_W1 * normal_1_;

    Eigen::Map<Eigen::Vector3d> error(residuals);
    error = (n_W1 - n_W0) * weight_;

    if (jacobians != nullptr || jacobians_minimal != nullptr)
    {
      // minimal (6-D pose tangent [dp, dtheta]) Jacobians, row-major 3x6
      Eigen::Matrix<double, 3, 6, Eigen::RowMajor> J0_minimal;
      J0_minimal.setConstant(0.0);                              // 0x1800074E0(…, 18, 0.0)
      J0_minimal.block<3, 3>(0, 3) = weight_ * skewSymmetric(n_W0);
      Eigen::Matrix<double, 3, 6, Eigen::RowMajor> J1_minimal;
      J1_minimal.setConstant(0.0);
      J1_minimal.block<3, 3>(0, 3) = -weight_ * skewSymmetric(n_W1);

      const Eigen::Matrix<double, 3, 6, Eigen::RowMajor>* J_minimal[2] =
          { &J0_minimal, &J1_minimal };
      for (size_t i = 0; i < 2; ++i)
      {
        if (jacobians_minimal != nullptr && jacobians_minimal[i] != nullptr)
        {
          Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> J_min(jacobians_minimal[i]);
          J_min = *J_minimal[i];
        }
        if (jacobians != nullptr && jacobians[i] != nullptr)
        {
          Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
          PoseLocalParameterization::liftJacobian(parameters[i], J_lift.data());  // 0x18008CC60
          Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> J(jacobians[i]);
          J = *J_minimal[i] * J_lift;   // lazy 3x6 * 6x7 (dot products of 6), see binary
        }
      }
    }
    return true;
  }

  // 0x18001A910 (ICF "return 3")
  virtual size_t residualDim() const { return kNumResiduals; }

  // 0x180014980 (ICF, shared by all errors)
  virtual size_t parameterBlocks() const
  {
    return base_t::parameter_block_sizes().size();
  }

  // 0x180014950 (ICF, shared by all errors)
  virtual size_t parameterBlockDim(size_t parameter_block_idx) const
  {
    return base_t::parameter_block_sizes().at(parameter_block_idx);
  }

  // 0x18002DEF0
  virtual ErrorType typeInfo() const
  {
    return ErrorType::kGroundPlaneError;   // 7
  }

 private:
  Eigen::Vector3d normal_0_;   // +0x38
  Eigen::Vector3d normal_1_;   // +0x50
  double weight_;              // +0x68

  friend struct GroundPlaneErrorLayoutCheck;
};

struct GroundPlaneErrorLayoutCheck
{
  static_assert(sizeof(GroundPlaneError) == 0x70, "");
  static_assert(offsetof(GroundPlaneError, normal_0_) == 0x38, "");
  static_assert(offsetof(GroundPlaneError, normal_1_) == 0x50, "");
  static_assert(offsetof(GroundPlaneError, weight_) == 0x68, "");
};

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
