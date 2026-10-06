// pimax_slam.pi.dll -- src/ceres_backend/marginalization_error.hpp  (drafts c04 + c05)
//
// Pimax fork of svo_ceres_backend/include/svo/ceres_backend/marginalization_error.hpp
// (OKVIS MarginalizationError), namespace pimax::totem::ceres_backend.
//
// Big structural change vs. upstream: addResidualBlock() no longer linearises anything.  It only
// books the new parameter blocks (ParameterBlockInfo, ordering, ceres size bookkeeping) and
// remembers the residual block (id, spec, parameter-block ids).  The actual H_/b0_ resize and the
// Jacobian evaluation + accumulation is deferred to linearizeResidualBlocks() (0x18007B280), which
// marginalizeOut() calls first.  See notes/c05_marginalization.md.
//
// Layout (sizeof == 0x240 == 576, allocated with malloc(0x240) by the estimator @0x180027AE0,
// EIGEN_MAKE_ALIGNED_OPERATOR_NEW; vtables 0x1803B05C0 (CostFunction) / 0x1803B05E0 (ErrorInterface)).
#pragma once

#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <ceres/ceres.h>

#include "ceres_backend/error_interface.hpp"
#include "ceres_backend/parameter_block.hpp"
#include "ceres_backend/ceres_map.hpp"
#include "ceres_backend/general_3d_parameter_block.hpp"

namespace pimax {
namespace totem {

class Estimator;   // friend (Estimator::applyMarginalizationStrategy VLOG(21) reads H_ / parameter_block_infos_)

namespace ceres_backend {

// not sized, in order to be flexible.
class MarginalizationError : public ceres::CostFunction, public ErrorInterface
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef ceres::CostFunction base_t;

  /// Inline dtor, body 0x180077C70 (chunk c04 range), deleting dtor 0x1800784C0 / thunk 0x1800784A8.
  virtual ~MarginalizationError() = default;

  /// 0x1800773E0 (chunk c04 range).  The only constructor present in the binary.
  MarginalizationError(Map& map);

  /// 0x180079D90.  Pimax: returns void, only books the block (no linearisation here).
  void addResidualBlock(ceres::ResidualBlockId residual_block_id, bool keep = false);

  /// 0x180080320.  Pimax: no keep_parameter_blocks argument any more.
  bool marginalizeOut(const std::vector<uint64_t>& parameter_block_ids);

  /// 0x180088D10 (upstream-identical).
  void updateErrorComputation();

  /// 0x18007B1F0.
  void getParameterBlockPtrs(
      std::vector<std::shared_ptr<ceres_backend::ParameterBlock> >& parameter_block_ptrs);

  /// 0x180078510 (CostFunction vtable slot 1).
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const;

  /// 0x180079000 (CostFunction vtable slot 2) [pimax-new virtual].
  /// Jacobians w.r.t. the *local* (minimal) coordinates at the current estimate:
  ///   J_i = J_.block(...) * liftJacobian(x_lin) * plusJacobian(x),  stored rows x minimal_dim.
  /// Selected by Evaluate() when ErrorInterface::use_minimal_jacobians_ is set.
  /// TODO(verify) name.
  virtual bool EvaluateLocal(double const* const* parameters, double* residuals,
                             double** jacobians) const;

  /// 0x180078540 (ErrorInterface vtable slot 4).
  bool EvaluateWithMinimalJacobians(double const* const* parameters,
                                    double* residuals, double** jacobians,
                                    double** jacobians_minimal) const;

  /// 0x180087010 (ErrorInterface slot 1): returns num_residuals() (int, sign-extended).
  size_t residualDim() const
  {
    return base_t::num_residuals();
  }

  /// folded 0x180014980 (ErrorInterface slot 2).
  size_t parameterBlocks() const
  {
    return base_t::parameter_block_sizes().size();
  }

  /// folded 0x180014950 (ErrorInterface slot 3).
  size_t parameterBlockDim(size_t parameter_block_idx) const
  {
    return base_t::parameter_block_sizes().at(parameter_block_idx);
  }

  /// 0x180088D00 (ErrorInterface slot 5): returns 3.
  virtual ErrorType typeInfo() const
  {
    return ErrorType::kMarginalizationError;
  }

  template<typename Derived>
  static bool pseudoInverseSymm(
      const Eigen::MatrixBase<Derived>& a,
      const Eigen::MatrixBase<Derived>& result, double epsilon =
          std::numeric_limits<typename Derived::Scalar>::epsilon(),
      int* rank = 0);

  /// instantiated: Matrix3d 0x18006CA60 (upstream path), MatrixXd 0x18006BF60 (Pimax: LLT first,
  /// see marginalization_error_impl.hpp; c04 correction of c05).
  template<typename Derived>
  static bool pseudoInverseSymmSqrt(
      const Eigen::MatrixBase<Derived>& a,
      const Eigen::MatrixBase<Derived>& result, double epsilon =
          std::numeric_limits<typename Derived::Scalar>::epsilon(),
      int* rank = NULL);

  template<typename Derived, int blockDim>
  static void blockPinverse(
      const Eigen::MatrixBase<Derived>& M_in,
      const Eigen::MatrixBase<Derived>& M_out, double epsilon =
          std::numeric_limits<typename Derived::Scalar>::epsilon());

  template<typename Derived, int blockDim>
  static void blockPinverseSqrt(
      const Eigen::MatrixBase<Derived>& M_in,
      const Eigen::MatrixBase<Derived>& M_out, double epsilon =
          std::numeric_limits<typename Derived::Scalar>::epsilon());

  bool isInMarginalizationTerm(uint64_t param_block_id)
  {
    return parameter_block_id_to_parameter_block_info_idx_.find(param_block_id)
        != parameter_block_id_to_parameter_block_info_idx_.end();
  }

 protected:
  /// [pimax-new] 0x18007B280.  Called at the start of marginalizeOut():
  ///  1. re-computes ordering_idx / index map of the landmark part,
  ///  2. rebuilds H_/b0_ with room for the blocks booked since the last call,
  ///  3. evaluates every booked residual at the linearisation points and adds J^T J / -J^T r,
  ///  4. clears the booking containers/counters.
  /// TODO(verify) name.
  void linearizeResidualBlocks();

  /// 0x18007AA20 (upstream-identical).
  bool computeDeltaChi(double const* const* parameters,
                       Eigen::VectorXd& DeltaChi) const;

  /// instantiated for <MatrixXd x4> at 0x1800732A0 (chunk c04 range).
  template<typename Derived_A, typename Derived_U, typename Derived_W,
      typename Derived_V>
  static void splitSymmetricMatrix(
      const std::vector<std::pair<int, int> >& marginalization_start_idx_and_length_pairs,
      const Eigen::MatrixBase<Derived_A>& A,
      const Eigen::MatrixBase<Derived_U>& U,
      const Eigen::MatrixBase<Derived_W>& W,
      const Eigen::MatrixBase<Derived_V>& V);

  /// instantiated for <VectorXd x3> at 0x1800740F0 (chunk c04 range).
  template<typename Derived_b, typename Derived_b_a, typename Derived_b_b>
  static void splitVector(
      const std::vector<std::pair<int, int> >& marginalization_start_idx_and_length_pairs,
      const Eigen::MatrixBase<Derived_b>& b,
      const Eigen::MatrixBase<Derived_b_a>& b_a,
      const Eigen::MatrixBase<Derived_b_b>& b_b);

  // +0x00 ceres::CostFunction: vptr, parameter_block_sizes_ (+0x08), num_residuals_ (+0x20)
  // +0x28 ErrorInterface: vptr, use_minimal_jacobians_ (+0x30)
  Map* map_ptr_;                                   // +0x038 (sure)
  ceres::ResidualBlockId residual_block_id_;       // +0x040 (sure: zeroed in ctor, never used)

  /// [pimax-new] four dynamic Eigen objects + two 8-byte PODs before H_.  Default constructed
  /// (zeroed) by the ctor, freed by the dtor, never touched by any function found so far.
  /// TODO(verify) names / types / users.
  Eigen::MatrixXd unknown_mat_48_;                 // +0x048
  Eigen::VectorXd unknown_vec_60_;                 // +0x060
  Eigen::MatrixXd unknown_mat_70_;                 // +0x070
  Eigen::VectorXd unknown_vec_88_;                 // +0x088
  double unknown_98_;                              // +0x098 (not initialised by the ctor)
  double unknown_a0_;                              // +0x0A0 (not initialised by the ctor)

  Eigen::MatrixXd H_;              ///< +0x0A8 lhs - Hessian                              (sure)
  Eigen::VectorXd b0_;             ///< +0x0C0 rhs constant part                          (sure)
  Eigen::VectorXd e0_;             ///< +0x0D0 _e0 := pinv(J^T) * _b0                    (sure)
  Eigen::MatrixXd J_;              ///< +0x0E0 Jacobian such that _J^T * J == _H           (sure)
  Eigen::MatrixXd U_;              ///< +0x0F8 (unused, as upstream)
  Eigen::VectorXd S_;              ///< +0x110                                             (sure)
  Eigen::VectorXd S_sqrt_;         ///< +0x120                                             (sure)
  Eigen::VectorXd S_pinv_;         ///< +0x130                                             (sure)
  Eigen::VectorXd S_pinv_sqrt_;    ///< +0x140                                             (sure)
  Eigen::VectorXd p_;              ///< +0x150 (unused, as upstream)
  Eigen::VectorXd p_inv_;          ///< +0x160 (unused, as upstream)
  volatile bool error_computation_valid_;          // +0x170 (sure)

  /// \brief Book-keeping of the ordering.  80 bytes.
  struct ParameterBlockInfo
  {
    uint64_t parameter_block_id;                          // +0x00
    std::shared_ptr<ParameterBlock> parameter_block_ptr;  // +0x08
    size_t ordering_idx;                                  // +0x18
    size_t dimension;                                     // +0x20
    size_t minimal_dimension;                             // +0x28
    size_t local_dimension;                               // +0x30
    std::shared_ptr<double> linearization_point;          // +0x38
    bool is_landmark;                                     // +0x48
    ParameterBlockInfo()
        : parameter_block_id(0),
          parameter_block_ptr(std::shared_ptr<ParameterBlock>()),
          ordering_idx(0),
          dimension(0),
          minimal_dimension(0),
          local_dimension(0),
          is_landmark(false)
    {
    }
    // 0x180077740 (chunk c04 range) -- upstream-identical.
    ParameterBlockInfo(uint64_t parameter_block_id,
                       std::shared_ptr<ParameterBlock> parameter_block_ptr,
                       size_t orderingIdx, bool is_landmark)
        : parameter_block_id(parameter_block_id),
          parameter_block_ptr(parameter_block_ptr),
          ordering_idx(orderingIdx),
          is_landmark(is_landmark)
    {
      dimension = parameter_block_ptr->dimension();
      minimal_dimension = parameter_block_ptr->minimalDimension();
      if (parameter_block_ptr->localParameterizationPtr())
      {
        local_dimension = parameter_block_ptr->localParameterizationPtr()
            ->LocalSize();
      }
      else
      {
        local_dimension = minimal_dimension;
      }
      if (parameter_block_ptr->fixed())
      {
        minimal_dimension = 0;
        local_dimension = 0;
      }
      linearization_point.reset(new double[dimension],
                               std::default_delete<double[]>());
      memcpy(linearization_point.get(), parameter_block_ptr->parameters(),
             dimension * sizeof(double));
    }
  };

  std::vector<ParameterBlockInfo> parameter_block_infos_;           // +0x178 (sure)
  /// Pimax: std::unordered_map (upstream std::map).                   (sure: FNV-1a hash)
  std::unordered_map<uint64_t, size_t>
      parameter_block_id_to_parameter_block_info_idx_;              // +0x190 (sure)

  // ---- [pimax-new] deferred-linearisation bookkeeping (cleared by linearizeResidualBlocks) ----
  /// ids of parameter blocks newly booked by addResidualBlock (push_back only, then clear()).
  std::vector<uint64_t> new_parameter_block_ids_;                   // +0x1D0 (sure)  TODO(verify) name
  /// copy of the Map's ResidualBlockSpec of every booked residual block.
  std::map<ceres::ResidualBlockId, Map::ResidualBlockSpec>
      residual_block_specs_;                                        // +0x1E8 (sure)  TODO(verify) name
  /// booked residual block ids, in booking order.
  std::vector<ceres::ResidualBlockId> residual_block_ids_;          // +0x1F8 (sure)  TODO(verify) name
  /// parameter-block ids of every booked residual block (in the residual's parameter order).
  std::map<ceres::ResidualBlockId, std::vector<uint64_t> >
      residual_block_parameter_ids_;                                // +0x210 (sure)  TODO(verify) name
  uint32_t num_new_dense_blocks_ = 0;                               // +0x220 (sure)  TODO(verify) name
  uint32_t num_new_landmark_blocks_ = 0;                            // +0x224 (sure, write-only)
  uint32_t num_new_dimensions_ = 0;                                 // +0x228 (sure)  sum of new minimal dims

  size_t dense_indices_;  ///< +0x230 size of the dense part of the equation system (sure)
  size_t unknown_238_ = 0;                                          // +0x238 zeroed by ctor, unused

  friend struct MarginalizationErrorLayoutCheck;
  friend class ::pimax::totem::Estimator;
};

struct MarginalizationErrorLayoutCheck
{
  static_assert(sizeof(MarginalizationError) == 0x240, "");
  static_assert(offsetof(MarginalizationError, map_ptr_) == 0x38, "");
  static_assert(offsetof(MarginalizationError, residual_block_id_) == 0x40, "");
  static_assert(offsetof(MarginalizationError, unknown_mat_48_) == 0x48, "");
  static_assert(offsetof(MarginalizationError, unknown_98_) == 0x98, "");
  static_assert(offsetof(MarginalizationError, H_) == 0xA8, "");
  static_assert(offsetof(MarginalizationError, b0_) == 0xC0, "");
  static_assert(offsetof(MarginalizationError, e0_) == 0xD0, "");
  static_assert(offsetof(MarginalizationError, J_) == 0xE0, "");
  static_assert(offsetof(MarginalizationError, U_) == 0xF8, "");
  static_assert(offsetof(MarginalizationError, S_) == 0x110, "");
  static_assert(offsetof(MarginalizationError, p_inv_) == 0x160, "");
  static_assert(offsetof(MarginalizationError, error_computation_valid_) == 0x170, "");
  static_assert(offsetof(MarginalizationError, parameter_block_infos_) == 0x178, "");
  static_assert(offsetof(MarginalizationError, parameter_block_id_to_parameter_block_info_idx_) == 0x190, "");
  static_assert(offsetof(MarginalizationError, new_parameter_block_ids_) == 0x1D0, "");
  static_assert(offsetof(MarginalizationError, residual_block_specs_) == 0x1E8, "");
  static_assert(offsetof(MarginalizationError, residual_block_ids_) == 0x1F8, "");
  static_assert(offsetof(MarginalizationError, residual_block_parameter_ids_) == 0x210, "");
  static_assert(offsetof(MarginalizationError, num_new_dense_blocks_) == 0x220, "");
  static_assert(offsetof(MarginalizationError, num_new_landmark_blocks_) == 0x224, "");
  static_assert(offsetof(MarginalizationError, num_new_dimensions_) == 0x228, "");
  static_assert(offsetof(MarginalizationError, dense_indices_) == 0x230, "");
  static_assert(offsetof(MarginalizationError, unknown_238_) == 0x238, "");
};

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax

#include "ceres_backend/marginalization_error_impl.hpp"
