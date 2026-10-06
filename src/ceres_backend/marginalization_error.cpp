// pimax_slam.pi.dll -- src/ceres_backend/marginalization_error.cpp  (drafts c05 + c04, phase B1)
//
// __FILE__ = "E:\code_codex\pimax_slam\beta111_5a7902_dll\src\ceres_backend\marginalization_error.cpp"
//
// Port of svo_ceres_backend/src/marginalization_error.cpp, edited to match the binary.
// Image layout of this object (it straddles the c04/c05 chunk boundary):
//   0x180053DB0 .. 0x180079D90  (chunk c04 range): Eigen/STL instantiations, the
//       split*/pseudoInverse* template instantiations (bodies in marginalization_error_impl.hpp:
//       splitSymmetricMatrix<MatrixXd x4> 0x1800732A0, splitVector<VectorXd x3> 0x1800740F0,
//       pseudoInverseSymmSqrt<Matrix3d> 0x18006CA60, pseudoInverseSymmSqrt<MatrixXd> 0x18006BF60;
//       all emitted through their uses in marginalizeOut() below), ctor 0x1800773E0,
//       ParameterBlockInfo ctor 0x180077740, dtor body 0x180077C70, Evaluate 0x180078510,
//       EvaluateWithMinimalJacobians 0x180078540, EvaluateLocal 0x180079000.
//   0x180079D90 .. 0x18008A080  (chunk c05): addResidualBlock, computeDeltaChi,
//       getParameterBlockPtrs, linearizeResidualBlocks, marginalizeOut, residualDim (inline),
//       typeInfo (inline), updateErrorComputation and Eigen/STL instantiations.
// c04 and c05 both drafted ctor / Evaluate / EvaluateWithMinimalJacobians / EvaluateLocal; the two
// drafts were token-for-token equivalent (c05 text kept, see notes/integration_B1.md).
//
// No DEBUG_CHECK / check() / LOG anywhere in this object except the single CHECK in
// addResidualBlock (line 120, forced with #line; verified at 0x180079DE1: lea r8d,[r15+78h]).
#include "ceres_backend/marginalization_error.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>

#include <glog/logging.h>

#include "ceres_backend/local_parameterization_additional_interfaces.hpp"

//#define USE_NEW_LINEARIZATION_POINT

namespace pimax {
namespace totem {
namespace ceres_backend {

// Upstream conservativeResize() helpers are gone (no longer used: H_/b0_ are rebuilt with
// setZero() in linearizeResidualBlocks()).

// 0x1800773E0  (chunk c04 range; drafted identically by c04 and c05)  upstream-modified:
//   * error_computation_valid_ etc. as upstream (setMap() inlined; upstream setMap() is not emitted
//     and does not exist in this header),
//   * std::unordered_map index (FNV-1a), Pimax members (default-constructed),
//   * [pimax-new] reserve(100) / reserve(500) / reserve(200) on the bookkeeping vectors.
// 0x180077740 ParameterBlockInfo(id, ptr, ordering, is_landmark) is the out-of-line copy of the
// header-inline ctor (marginalization_error.hpp); 0x180077C70 / 0x1800784C0 / 0x1800784A8 are the
// implicit dtor body / scalar deleting dtor (free(): EIGEN_MAKE_ALIGNED_OPERATOR_NEW) / this-40 thunk.
MarginalizationError::MarginalizationError(Map& map)
{
  // setMap(map) -- inlined (upstream setMap() itself is not emitted)
  map_ptr_ = &map;
  residual_block_id_ = 0;  // reset.
  dense_indices_ = 0;
  residual_block_id_ = 0;
  error_computation_valid_ = false;
  parameter_block_infos_.reserve(100);
  residual_block_ids_.reserve(500);
  new_parameter_block_ids_.reserve(200);
}

// 0x180079D90  upstream-modified (heavily):
//   * returns void; CHECK(parameters != nullptr) instead of DEBUG_CHECK(error_interface_ptr);
//   * landmarks are General3DParameterBlock (upstream HomogeneousPointParameterBlock);
//   * H_/b0_ are NOT resized and nothing is evaluated here.  Only the bookkeeping is done:
//     num_residuals_ and num_new_dimensions_ grow by the new minimal dimension, counters for new
//     dense/landmark blocks, the residual id/spec/parameter-id list are stored for
//     linearizeResidualBlocks();
//   * the "also increase the rest" loop over the landmark part is gone (re-done in
//     linearizeResidualBlocks()).
void MarginalizationError::addResidualBlock(
    ceres::ResidualBlockId residual_block_id, bool keep)
{
  error_computation_valid_ = false;  // flag that the error computation is invalid

  // get the parameter blocks
  const Map::ParameterBlockCollection* parameters =
      map_ptr_->parametersPtr(residual_block_id);
#line 120
  CHECK(parameters != nullptr) << "Residual block not found";

  ParameterBlockInfo info;
  std::vector<uint64_t> parameter_block_ids;
  parameter_block_ids.reserve(parameters->size());

  // insert into parameter block ordering book-keeping
  for (size_t i = 0; i < parameters->size(); ++i)
  {
    const Map::ParameterBlockSpec& parameter_block_spec = (*parameters)[i];

    // does it already exist as a parameter block connected?
    std::unordered_map<uint64_t, size_t>::iterator it =
        parameter_block_id_to_parameter_block_info_idx_.find(parameter_block_spec.first);
    if (it != parameter_block_id_to_parameter_block_info_idx_.end())
    {
      parameter_block_ids.push_back(it->first);
      continue;
    }

    // not found. add it.
    // let's see, if it is actually a landmark, because then it will go into the sparse part
    bool is_landmark = false;
    if (std::dynamic_pointer_cast<General3DParameterBlock>(
        parameter_block_spec.second) != 0)
    {
      is_landmark = true;
    }

    int additional_size = 0;
    if (!parameter_block_spec.second->fixed())
      additional_size = parameter_block_spec.second->minimalDimension();
    size_t dense_size = 0;

    if (dense_indices_ > 0)
      dense_size =
          parameter_block_infos_.at(dense_indices_ - 1).ordering_idx
          + parameter_block_infos_.at(dense_indices_ - 1).minimal_dimension;

    base_t::set_num_residuals(base_t::num_residuals() + additional_size);
    num_new_dimensions_ += additional_size;

    // update book-keeping
    if (!is_landmark)
    {
      info = ParameterBlockInfo(parameter_block_spec.first,
                                parameter_block_spec.second, dense_size,
                                is_landmark);
      parameter_block_infos_.insert(
          parameter_block_infos_.begin() + dense_indices_, info);

      parameter_block_id_to_parameter_block_info_idx_.insert(
          std::pair<uint64_t, size_t>(parameter_block_spec.first,
                                      dense_indices_));

      //  update base_t book-keeping
      base_t::mutable_parameter_block_sizes()->insert(
          base_t::mutable_parameter_block_sizes()->begin() + dense_indices_,
          info.dimension);

      dense_indices_++;  // remember we increased the dense part of the problem
      num_new_dense_blocks_++;
    }
    else
    {
      // just add at the end
      info = ParameterBlockInfo(
          parameter_block_spec.first,
          parameter_block_spec.second,
          parameter_block_infos_.back().ordering_idx
              + parameter_block_infos_.back().minimal_dimension,
          is_landmark);
      parameter_block_infos_.push_back(info);
      parameter_block_id_to_parameter_block_info_idx_.insert(
          std::pair<uint64_t, size_t>(parameter_block_spec.first,
                                      parameter_block_infos_.size() - 1));

      //  update base_t book-keeping
      base_t::mutable_parameter_block_sizes()->push_back(info.dimension);
      num_new_landmark_blocks_++;
    }
    new_parameter_block_ids_.push_back(parameter_block_spec.first);
    parameter_block_ids.push_back(parameter_block_spec.first);
  }

  residual_block_ids_.push_back(residual_block_id);
  residual_block_specs_[residual_block_id] =
      map_ptr_->residualBlockIdToResidualBlockSpecMap().find(residual_block_id)->second;
  residual_block_parameter_ids_[residual_block_id] = std::move(parameter_block_ids);

  // finally, we also have to delete the nonlinear residual block from the map:
  if (!keep)
  {
    map_ptr_->removeResidualBlock(residual_block_id);
  }
}

// 0x18007B1F0  upstream-modified: no DEBUG_CHECK(map_ptr_), range-for.
void MarginalizationError::getParameterBlockPtrs(
    std::vector<std::shared_ptr<ceres_backend::ParameterBlock> >& parameter_block_ptrs)
{
  for (const ParameterBlockInfo& info : parameter_block_infos_)
  {
    parameter_block_ptrs.push_back(info.parameter_block_ptr);
  }
}

// 0x18007B280  pimax-new.  The deferred part of upstream addResidualBlock():
//   1. re-assign ordering_idx (and the id->index map) of all landmark blocks, which now start
//      after the (possibly grown) dense part;
//   2. rebuild H_ / b0_ at the new size: old dense block stays at the top-left, the old landmark
//      part is moved behind the new dense blocks, rows/cols of the new blocks are zero;
//   3. for every booked residual: evaluate the minimal Jacobians at the linearisation points,
//      apply the ceres loss-function correction (upstream code) and accumulate J^T J and -J^T r;
//   4. clear the booking containers and counters.
// Index containers: info_i is read with operator[] (no bounds check in the binary), info_j and
// the Jacobian vector with .at().
void MarginalizationError::linearizeResidualBlocks()
{
  const size_t orig_size = H_.cols();

  // 1. re-number the landmark part
  size_t ordering_idx = 0;
  if (dense_indices_)
    ordering_idx = parameter_block_infos_[dense_indices_ - 1].minimal_dimension
        + parameter_block_infos_[dense_indices_ - 1].ordering_idx;
  for (size_t i = dense_indices_; i < parameter_block_infos_.size(); ++i)
  {
    parameter_block_infos_.at(i).ordering_idx = ordering_idx;
    parameter_block_id_to_parameter_block_info_idx_[
        parameter_block_infos_.at(i).parameter_block_ptr->id()] = i;
    ordering_idx += parameter_block_infos_.at(i).minimal_dimension;
  }

  // size of the dense part before the blocks booked since the last call
  size_t dense_size = 0;
  if (dense_indices_ > num_new_dense_blocks_)
    dense_size =
        parameter_block_infos_.at(dense_indices_ - num_new_dense_blocks_ - 1).ordering_idx
        + parameter_block_infos_.at(dense_indices_ - num_new_dense_blocks_ - 1).minimal_dimension;

  // 2. rebuild the linear system
  // lhs
  Eigen::MatrixXd H00 = H_.topLeftCorner(dense_size, dense_size);
  Eigen::MatrixXd H01 = H_.topRightCorner(dense_size, orig_size - dense_size);
  Eigen::MatrixXd H10 = H_.bottomLeftCorner(orig_size - dense_size, dense_size);
  Eigen::MatrixXd H11 = H_.bottomRightCorner(orig_size - dense_size,
                                             orig_size - dense_size);
  // rhs
  Eigen::VectorXd b0 = b0_.head(dense_size);
  Eigen::VectorXd b1 = b0_.tail(orig_size - dense_size);

  size_t new_dense_size = 0;
  if (dense_indices_)
    new_dense_size = parameter_block_infos_.at(dense_indices_ - 1).minimal_dimension
        + parameter_block_infos_.at(dense_indices_ - 1).ordering_idx;

  H_.setZero(orig_size + num_new_dimensions_, orig_size + num_new_dimensions_);
  b0_.setZero(orig_size + num_new_dimensions_);

  H_.topLeftCorner(dense_size, dense_size) = H00;
  H_.block(0, new_dense_size, dense_size, orig_size - dense_size) = H01;
  H_.block(new_dense_size, 0, orig_size - dense_size, dense_size) = H10;
  H_.block(new_dense_size, new_dense_size,
           orig_size - dense_size, orig_size - dense_size) = H11;
  b0_.head(dense_size) = b0;
  b0_.segment(new_dense_size, orig_size - dense_size) = b1;

  // 3. linearise every booked residual block
  for (std::vector<ceres::ResidualBlockId>::iterator it = residual_block_ids_.begin();
       it != residual_block_ids_.end(); ++it)
  {
    Map::ResidualBlockSpec& residual_block_spec = residual_block_specs_[*it];
    ceres::LossFunction* lossFunction = residual_block_spec.loss_function_ptr;
    const std::vector<uint64_t>& parameter_block_ids = residual_block_parameter_ids_[*it];
    const unsigned int num_parameter_blocks =
        static_cast<unsigned int>(parameter_block_ids.size());

    double** jacobians_minimal_raw = new double*[num_parameter_blocks];
    std::vector<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
        Eigen::aligned_allocator<
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > >
        jacobians_minimal_eigen(num_parameter_blocks);
    Eigen::VectorXd residuals_eigen(
        residual_block_spec.error_interface_ptr->residualDim());

    for (size_t i = 0; i < num_parameter_blocks; ++i)
    {
      ParameterBlockInfo parameterBlockInfo = parameter_block_infos_[
          parameter_block_id_to_parameter_block_info_idx_[parameter_block_ids[i]]];
      jacobians_minimal_eigen[i].resize(
          residual_block_spec.error_interface_ptr->residualDim(),
          parameterBlockInfo.minimal_dimension);
      jacobians_minimal_raw[i] = jacobians_minimal_eigen[i].data();
    }

    double** parameters_raw = new double*[num_parameter_blocks];
    for (int i = 0; i < num_parameter_blocks; ++i)
    {
      ParameterBlockInfo parameterBlockInfo = parameter_block_infos_[
          parameter_block_id_to_parameter_block_info_idx_[parameter_block_ids[i]]];
      parameters_raw[i] = parameterBlockInfo.linearization_point.get();  // first estimate Jacobian!!
    }

    // evaluate residual block and get the minimal Jacobians.
    residual_block_spec.error_interface_ptr->EvaluateWithMinimalJacobians(
        parameters_raw, residuals_eigen.data(), nullptr, jacobians_minimal_raw);

    // correct for loss function if applicable
    if (lossFunction)
    {
      // following ceres in internal/ceres/corrector.cc
      const double sq_norm = residuals_eigen.transpose() * residuals_eigen;
      double rho[3];
      lossFunction->Evaluate(sq_norm, rho);
      const double sqrt_rho1 = sqrt(rho[1]);
      double residual_scaling;
      double alpha_sq_norm;
      if ((sq_norm == 0.0) || (rho[2] <= 0.0))
      {
        residual_scaling = sqrt_rho1;
        alpha_sq_norm = 0.0;
      }
      else
      {
        // Calculate the smaller of the two solutions to the equation
        //
        // 0.5 *  alpha^2 - alpha - rho'' / rho' *  z'z = 0.
        //
        // Start by calculating the discriminant D.
        const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];

        // Since both rho[1] and rho[2] are guaranteed to be positive at
        // this point, we know that D > 1.0.

        const double alpha = 1.0 - sqrt(D);

        // Calculate the constants needed by the correction routines.
        residual_scaling = sqrt_rho1 / (1 - alpha);
        alpha_sq_norm = alpha / sq_norm;
      }

      // correct Jacobians (Equation 11 in BANS)
      for (size_t i = 0; i < num_parameter_blocks; ++i)
      {
        jacobians_minimal_eigen[i] = sqrt_rho1
            * (jacobians_minimal_eigen[i]
                - alpha_sq_norm * residuals_eigen
                    * (residuals_eigen.transpose() * jacobians_minimal_eigen[i]));
      }

      // correct residuals (caution: must be after "correct Jacobians"):
      residuals_eigen *= residual_scaling;
    }

    // add blocks to lhs and rhs
    for (size_t i = 0; i < num_parameter_blocks; ++i)
    {
      ParameterBlockInfo parameterBlockInfo_i = parameter_block_infos_[
          parameter_block_id_to_parameter_block_info_idx_[parameter_block_ids[i]]];

      if (parameterBlockInfo_i.minimal_dimension == 0)
      {
        continue;
      }

      // Insert Hessian and rhs in diagonal.
      H_.block(parameterBlockInfo_i.ordering_idx, parameterBlockInfo_i.ordering_idx,
               parameterBlockInfo_i.minimal_dimension,
               parameterBlockInfo_i.minimal_dimension) +=
          jacobians_minimal_eigen.at(i).transpose().eval()
          * jacobians_minimal_eigen.at(i);
      b0_.segment(parameterBlockInfo_i.ordering_idx,
                  parameterBlockInfo_i.minimal_dimension) -=
          jacobians_minimal_eigen.at(i).transpose().eval() * residuals_eigen;

      for (size_t j = 0; j < i; ++j)
      {
        // Now the parts not in the diagonal
        ParameterBlockInfo parameterBlockInfo_j = parameter_block_infos_.at(
            parameter_block_id_to_parameter_block_info_idx_[parameter_block_ids[j]]);

        if (parameterBlockInfo_j.minimal_dimension == 0)
        {
          continue;
        }

        // upper triangular:
        H_.block(parameterBlockInfo_i.ordering_idx,
                 parameterBlockInfo_j.ordering_idx,
                 parameterBlockInfo_i.minimal_dimension,
                 parameterBlockInfo_j.minimal_dimension) +=
            jacobians_minimal_eigen.at(i).transpose().eval()
            * jacobians_minimal_eigen.at(j);
        // lower triangular:
        H_.block(parameterBlockInfo_j.ordering_idx,
                 parameterBlockInfo_i.ordering_idx,
                 parameterBlockInfo_j.minimal_dimension,
                 parameterBlockInfo_i.minimal_dimension) +=
            jacobians_minimal_eigen.at(j).transpose().eval()
            * jacobians_minimal_eigen.at(i);
      }
    }

    // cleanup temporarily allocated stuff
    delete[] parameters_raw;
    delete[] jacobians_minimal_raw;
  }

  // 4. reset the booking
  residual_block_ids_.clear();
  new_parameter_block_ids_.clear();
  residual_block_specs_.clear();
  residual_block_parameter_ids_.clear();
  num_new_landmark_blocks_ = 0;   // +0x224 and +0x228 are cleared by one 8-byte store
  num_new_dimensions_ = 0;
  num_new_dense_blocks_ = 0;
}

// 0x180080320  upstream-modified:
//   * linearizeResidualBlocks() is called first (even if the id list is empty);
//   * no keep_parameter_blocks argument: parameter_block_ptrs is an std::unordered_map<uint64_t,
//     bool> filled with false;
//   * std::stable_sort instead of std::sort (the binary uses _Optimistic_temporary_buffer +
//     insertion sort for <= 32 elements: MSVC stable_sort);
//   * an unconnected id returns false silently (no LOG(ERROR), no DEBUG_CHECK);
//   * the final LOG(FATAL) "unmarginalizeLandmark ... not implemented" branch and check() are gone.
//   Landmark (3x3 block Schur, sdim = 3) and dense parts are upstream-identical.
bool MarginalizationError::marginalizeOut(
    const std::vector<uint64_t>& parameter_block_ids)
{
  linearizeResidualBlocks();

  if (parameter_block_ids.size() == 0)
  {
    return false;
  }

  // copy so we can manipulate
  std::vector<uint64_t> parameter_block_ids_copy = parameter_block_ids;
  std::unordered_map<uint64_t, bool> parameter_block_ptrs;
  for (uint64_t parameter_block_id : parameter_block_ids_copy)
  {
    parameter_block_ptrs.insert(std::pair<uint64_t, bool>(parameter_block_id, false));
  }

  /* figure out which blocks need to be marginalized out */
  std::vector<std::pair<int, int> >
      marginalization_start_idx_and_length_pairs_landmarks;
  std::vector<std::pair<int, int> >
      marginalization_start_idx_and_length_pairs_dense;
  size_t marginalization_parameters_landmarks = 0;
  size_t marginalization_parameters_dense = 0;

  // make sure no duplications...
  std::stable_sort(parameter_block_ids_copy.begin(), parameter_block_ids_copy.end());
  for (size_t i = 1; i < parameter_block_ids_copy.size(); ++i)
  {
    if (parameter_block_ids_copy[i] == parameter_block_ids_copy[i - 1])
    {
      parameter_block_ids_copy.erase(parameter_block_ids_copy.begin() + i);
      --i;
    }
  }
  for (size_t i = 0; i < parameter_block_ids_copy.size(); ++i)
  {
    std::unordered_map<uint64_t, size_t>::iterator it =
        parameter_block_id_to_parameter_block_info_idx_.find(
          parameter_block_ids_copy[i]);

    // sanity check - are we trying to marginalize stuff that is not connected to this error term?
    if (it == parameter_block_id_to_parameter_block_info_idx_.end())
    {
      return false;
    }

    // distinguish dense and landmark (sparse) part for more efficient pseudo-inversion later on
    size_t start_idx = parameter_block_infos_.at(it->second).ordering_idx;
    size_t min_dim = parameter_block_infos_.at(it->second).minimal_dimension;
    if (parameter_block_infos_.at(it->second).is_landmark)
    {
      marginalization_start_idx_and_length_pairs_landmarks.push_back(
          std::pair<int, int>(start_idx, min_dim));
      marginalization_parameters_landmarks += min_dim;
    }
    else
    {
      marginalization_start_idx_and_length_pairs_dense.push_back(
          std::pair<int, int>(start_idx, min_dim));
      marginalization_parameters_dense += min_dim;
    }
  }

  // make sure the marginalization pairs are ordered
  std::stable_sort(marginalization_start_idx_and_length_pairs_landmarks.begin(),
                   marginalization_start_idx_and_length_pairs_landmarks.end(),
                   [](std::pair<int,int> left, std::pair<int,int> right){
                     return left.first < right.first;
                   });
  std::stable_sort(marginalization_start_idx_and_length_pairs_dense.begin(),
                   marginalization_start_idx_and_length_pairs_dense.end(),
                   [](std::pair<int,int> left, std::pair<int,int> right) {
                     return left.first < right.first;
                   });

  // Unify contiguous marginalization requests. I.e. if some of the parameters
  // are ordered right next to each other -> combine them.
  for (size_t m = 1;
       m < marginalization_start_idx_and_length_pairs_landmarks.size(); ++m)
  {
    if (marginalization_start_idx_and_length_pairs_landmarks.at(m - 1).first
        + marginalization_start_idx_and_length_pairs_landmarks.at(m - 1).second
        == marginalization_start_idx_and_length_pairs_landmarks.at(m).first)
    {
      marginalization_start_idx_and_length_pairs_landmarks.at(m - 1).second +=
          marginalization_start_idx_and_length_pairs_landmarks.at(m).second;
      marginalization_start_idx_and_length_pairs_landmarks.erase(
          marginalization_start_idx_and_length_pairs_landmarks.begin() + m);
      --m;
    }
  }
  for (size_t m = 1;
       m < marginalization_start_idx_and_length_pairs_dense.size(); ++m)
  {
    if (marginalization_start_idx_and_length_pairs_dense.at(m - 1).first
        + marginalization_start_idx_and_length_pairs_dense.at(m - 1).second
        == marginalization_start_idx_and_length_pairs_dense.at(m).first)
    {
      marginalization_start_idx_and_length_pairs_dense.at(m - 1).second +=
          marginalization_start_idx_and_length_pairs_dense.at(m).second;
      marginalization_start_idx_and_length_pairs_dense.erase(
          marginalization_start_idx_and_length_pairs_dense.begin() + m);
      --m;
    }
  }

  error_computation_valid_ = false;  // flag that the error computation is invalid

  // include in the fix rhs part deviations from linearization point of the parameter blocks to be marginalized
  // corrected: this is not necessary, will cancel itself

  /* landmark part (if existing) */
  if (marginalization_start_idx_and_length_pairs_landmarks.size() > 0)
  {
    // preconditioner
    Eigen::VectorXd p = (H_.diagonal().array() > 1.0e-9).select(
          H_.diagonal().cwiseSqrt(),1.0e-3);
    Eigen::VectorXd p_inv = p.cwiseInverse();

    // scale H and b
    H_ = p_inv.asDiagonal() * H_ * p_inv.asDiagonal();
    b0_ = p_inv.asDiagonal() * b0_;

    // U: Part to be kept, V: Marginalized part, W: Split.
    Eigen::MatrixXd U(H_.rows() - marginalization_parameters_landmarks,
                      H_.rows() - marginalization_parameters_landmarks);
    Eigen::MatrixXd V(marginalization_parameters_landmarks,
                      marginalization_parameters_landmarks);
    Eigen::MatrixXd W(H_.rows() - marginalization_parameters_landmarks,
                      marginalization_parameters_landmarks);
    // b_a kept, b_b marginalized.
    Eigen::VectorXd b_a(H_.rows() - marginalization_parameters_landmarks);
    Eigen::VectorXd b_b(marginalization_parameters_landmarks);

    // split preconditioner
    Eigen::VectorXd p_a(H_.rows() - marginalization_parameters_landmarks);
    Eigen::VectorXd p_b(marginalization_parameters_landmarks);
    splitVector(marginalization_start_idx_and_length_pairs_landmarks,
                p, p_a, p_b);  // output

    // split lhs
    splitSymmetricMatrix(marginalization_start_idx_and_length_pairs_landmarks,
                         H_, U, W, V);  // output

    // split rhs
    splitVector(marginalization_start_idx_and_length_pairs_landmarks,
                b0_, b_a, b_b);  // output


    // invert the marginalization block
    static const int sdim =
        ceres_backend::General3DParameterBlock::c_minimal_dimension;  // == 3
    b0_.resize(b_a.rows());
    b0_ = b_a;
    H_.resize(U.rows(), U.cols());
    H_ = U;
    const size_t numBlocks = V.cols() / sdim;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>
        delta_H(numBlocks);
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>>
        delta_b(numBlocks);
    Eigen::MatrixXd M1(W.rows(), W.cols());
    size_t idx = 0;
    // Calculate all the schur related things.
    for (size_t i = 0; static_cast<int>(i) < V.cols(); i += sdim)
    {
      Eigen::Matrix<double, sdim, sdim> V_inv_sqrt;
      Eigen::Matrix<double, sdim, sdim> V1 = V.block(i, i, sdim, sdim);
      MarginalizationError::pseudoInverseSymmSqrt(V1, V_inv_sqrt);
      Eigen::MatrixXd M = W.block(0, i, W.rows(), sdim) * V_inv_sqrt;
      Eigen::MatrixXd M1 = W.block(0, i, W.rows(), sdim)
          * V_inv_sqrt * V_inv_sqrt.transpose();
      // accumulate
      delta_H.at(idx).resize(U.rows(), U.cols());
      delta_b.at(idx).resize(b_a.rows());
      if (i == 0)
      {
        delta_H.at(idx) = M * M.transpose();
        delta_b.at(idx) = M1 * b_b.segment<sdim>(i);
      }
      else
      {
        delta_H.at(idx) = delta_H.at(idx - 1) + M * M.transpose();
        delta_b.at(idx) = delta_b.at(idx - 1) + M1 * b_b.segment<sdim>(i);
      }
      ++idx;
    }
    // Schur
    b0_ -= delta_b.at(idx - 1);
    H_ -= delta_H.at(idx - 1);

    // unscale
    H_ = p_a.asDiagonal() * H_ * p_a.asDiagonal();
    b0_ = p_a.asDiagonal() * b0_;
  }

  /* dense part (if existing) */
  if (marginalization_start_idx_and_length_pairs_dense.size() > 0)
  {
    // preconditioner
    Eigen::VectorXd p = (H_.diagonal().array() > 1.0e-9).select(
          H_.diagonal().cwiseSqrt(),1.0e-3);
    Eigen::VectorXd p_inv = p.cwiseInverse();

    // scale H and b
    H_ = p_inv.asDiagonal() * H_ * p_inv.asDiagonal();
    b0_ = p_inv.asDiagonal() * b0_;

    Eigen::MatrixXd U(H_.rows() - marginalization_parameters_dense,
                      H_.rows() - marginalization_parameters_dense);
    Eigen::MatrixXd V(marginalization_parameters_dense,
                      marginalization_parameters_dense);
    Eigen::MatrixXd W(H_.rows() - marginalization_parameters_dense,
                      marginalization_parameters_dense);
    Eigen::VectorXd b_a(H_.rows() - marginalization_parameters_dense);
    Eigen::VectorXd b_b(marginalization_parameters_dense);

    // split preconditioner
    Eigen::VectorXd p_a(H_.rows() - marginalization_parameters_dense);
    Eigen::VectorXd p_b(marginalization_parameters_dense);
    splitVector(marginalization_start_idx_and_length_pairs_dense,
                p, p_a, p_b);  // output

    // split lhs
    splitSymmetricMatrix(marginalization_start_idx_and_length_pairs_dense,
                         H_, U, W, V);  // output

    // split rhs
    splitVector(marginalization_start_idx_and_length_pairs_dense, b0_,
                b_a, b_b);  // output

    // invert the marginalization block
    Eigen::MatrixXd V_inverse_sqrt(V.rows(), V.cols());
    Eigen::MatrixXd V1 = 0.5 * (V + V.transpose());
    pseudoInverseSymmSqrt(V1, V_inverse_sqrt);

    // Schur
    Eigen::MatrixXd M = W * V_inverse_sqrt;
    // rhs
    b0_.resize(b_a.rows());
    b0_ = (b_a - M * V_inverse_sqrt.transpose() * b_b);
    // lhs
    H_.resize(U.rows(), U.cols());

    H_ = (U - M * M.transpose());

    // unscale
    H_ = p_a.asDiagonal() * H_ * p_a.asDiagonal();
    b0_ = p_a.asDiagonal() * b0_;
  }

  // also adapt the ceres-internal size information
  base_t::set_num_residuals(base_t::num_residuals() -
                            marginalization_parameters_dense -
                            marginalization_parameters_landmarks);

  // delete all the book-keeping
  for (size_t i = 0; i < parameter_block_ids_copy.size(); ++i)
  {
    size_t idx = parameter_block_id_to_parameter_block_info_idx_.find(
        parameter_block_ids_copy[i])->second;
    int margSize = parameter_block_infos_.at(idx).minimal_dimension;
    parameter_block_infos_.erase(parameter_block_infos_.begin() + idx);

    for (size_t j = idx; j < parameter_block_infos_.size(); ++j)
    {
      parameter_block_infos_.at(j).ordering_idx -= margSize;
      parameter_block_id_to_parameter_block_info_idx_.at(
          parameter_block_infos_.at(j).parameter_block_id) -= 1;
    }

    parameter_block_id_to_parameter_block_info_idx_.erase(
          parameter_block_ids_copy[i]);

    // also adapt the ceres-internal book-keepin
    base_t::mutable_parameter_block_sizes()->erase(
        mutable_parameter_block_sizes()->begin() + idx);
  }

  // assume everything got dense
  // this is a conservative assumption, but true in particular when marginalizing
  // poses w/o landmarks
  dense_indices_ = parameter_block_infos_.size();
  for (size_t i = 0; i < parameter_block_infos_.size(); ++i)
  {
    if (parameter_block_infos_[i].is_landmark)
    {
      parameter_block_infos_[i].is_landmark = false;
    }
  }

  // check if the removal is safe
  for (size_t i = 0; i < parameter_block_ids_copy.size(); ++i)
  {
    Map::ResidualBlockCollection residuals = map_ptr_->residuals(
        parameter_block_ids_copy[i]);
    if (residuals.size() != 0
        && parameter_block_ptrs.at(parameter_block_ids_copy[i]) == false)
    {
      map_ptr_->printParameterBlockInfo(parameter_block_ids_copy[i]);
    }
  }
  for (size_t i = 0; i < parameter_block_ids_copy.size(); ++i)
  {
    // upstream: if (keep) LOG(FATAL) << "unmarginalizeLandmark(...) not implemented."; -- removed
    if (!parameter_block_ptrs.at(parameter_block_ids_copy[i]))
    {
      map_ptr_->removeParameterBlock(parameter_block_ids_copy[i]);
    }
  }

  return true;
}

// 0x180088D10  upstream-identical.
// This must be called before optimization after adding residual blocks and/or
// marginalizing, since it performs all the lhs and rhs computations on from a
// given _H and _b.
void MarginalizationError::updateErrorComputation()
{
  if (error_computation_valid_)
  {
    return;  // already done.
  }

  // now we also know the error dimension:
  base_t::set_num_residuals(H_.cols());

  // preconditioner
  Eigen::VectorXd p = (H_.diagonal().array() > 1.0e-9).select(
        H_.diagonal().cwiseSqrt(),1.0e-3);
  Eigen::VectorXd p_inv = p.cwiseInverse();

  // H_lambda_lambda^star has to be positive semidefinit. Due to numeric issues
  // this migth not be the case => set eigenvalues below threshold to zero.
  // lhs SVD: _H = J^T*J = _U*S*_U^T
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(
      0.5  * p_inv.asDiagonal() * (H_ + H_.transpose())  * p_inv.asDiagonal() );

  static const double epsilon = std::numeric_limits<double>::epsilon();
  double tolerance = epsilon * H_.cols() * saes.eigenvalues().array().maxCoeff();
  S_ = Eigen::VectorXd(
      (saes.eigenvalues().array() > tolerance).select(
          saes.eigenvalues().array(), 0));
  S_pinv_ = Eigen::VectorXd(
      (saes.eigenvalues().array() > tolerance).select(
          saes.eigenvalues().array().inverse(), 0));

  S_sqrt_ = S_.cwiseSqrt();
  S_pinv_sqrt_ = S_pinv_.cwiseSqrt();

  // assign Jacobian
  J_ = (p.asDiagonal() * saes.eigenvectors()
        * (S_sqrt_.asDiagonal())).transpose();

  // constant error (residual) _e0 := (-pinv(J^T) * _b):
  Eigen::MatrixXd J_pinv_T =
      (S_pinv_sqrt_.asDiagonal()) * saes.eigenvectors().transpose()
      * p_inv.asDiagonal();
  e0_ = (-J_pinv_T * b0_);

  // reconstruct. TODO: check if this really improves quality --- doesn't seem so...
  //H_ = J_.transpose() * J_;
  //b0_ = -J_.transpose() * e0_;
  error_computation_valid_ = true;
}

// 0x18007AA20  upstream-identical.
// Computes the linearized deviation from the references (linearization points)
bool MarginalizationError::computeDeltaChi(double const* const* parameters,
                                           Eigen::VectorXd& DeltaChi) const
{
  DeltaChi.resize(H_.rows());
  for (size_t i = 0; i < parameter_block_infos_.size(); ++i)
  {
    // stack Delta_Chi vector
    if (!parameter_block_infos_[i].parameter_block_ptr->fixed())
    {
      Eigen::VectorXd Delta_Chi_i(parameter_block_infos_[i].minimal_dimension);
      parameter_block_infos_[i].parameter_block_ptr->
          minus(parameter_block_infos_[i].linearization_point.get(),
                parameters[i],
                Delta_Chi_i.data());
      DeltaChi.segment(parameter_block_infos_[i].ordering_idx,
                       parameter_block_infos_[i].minimal_dimension) = Delta_Chi_i;
    }
  }
  return true;
}

// 0x180078510  (chunk c04 range)  upstream-modified: dispatch on ErrorInterface flag.
// This evaluates the error term and additionally computes the Jacobians.
bool MarginalizationError::Evaluate(double const* const* parameters,
                                    double* residuals,
                                    double** jacobians) const
{
  if (use_minimal_jacobians_)
  {
    return EvaluateLocal(parameters, residuals, jacobians);
  }
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
}

// 0x180079000  (chunk c04 range)  pimax-new.
// Residual as upstream; Jacobian i (if requested) = Jmin_i * J_lift(x_lin) * J_plus(x_i),
// a rows x minimal_dimension row-major matrix written to jacobians[i].
bool MarginalizationError::EvaluateLocal(double const* const* parameters,
                                         double* residuals,
                                         double** jacobians) const
{
  Eigen::VectorXd Delta_Chi;
  computeDeltaChi(parameters, Delta_Chi);

  for (size_t i = 0; i < parameter_block_infos_.size(); ++i)
  {
    const ParameterBlockInfo& info = parameter_block_infos_.at(i);
    if (info.minimal_dimension == 0)
    {
      continue;
    }
    if (jacobians != NULL && jacobians[i] != NULL)
    {
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                               Eigen::RowMajor> >
          J_i(jacobians[i], e0_.rows(), info.minimal_dimension);
      J_i = J_.block(0, info.ordering_idx, e0_.rows(), info.minimal_dimension);

      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
          J_lift(info.parameter_block_ptr->minimalDimension(),
                 info.parameter_block_ptr->dimension());
      info.parameter_block_ptr->liftJacobian(
          info.linearization_point.get(), J_lift.data());

      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
          J_plus(info.parameter_block_ptr->dimension(),
                 info.parameter_block_ptr->minimalDimension());
      info.parameter_block_ptr->plusJacobian(parameters[i], J_plus.data());

      J_i = J_i * J_lift * J_plus;
    }
  }

  // finally the error (residual) e = (-pinv(J^T) * _b + _J*Delta_Chi):
  Eigen::Map<Eigen::VectorXd> e(residuals, e0_.rows());
  e = e0_ + J_ * Delta_Chi;

  return true;
}

// 0x180078540  (chunk c04 range)  upstream-modified:
//   * .at(i) access, blocks with minimal_dimension == 0 are skipped entirely,
//   * Jmin_i is a Block expression shared by both branches (upstream: RowMajor copy in the
//     `jacobians` branch), no DEBUG_CHECK.
// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool MarginalizationError::EvaluateWithMinimalJacobians(
    double const* const* parameters, double* residuals, double** jacobians,
    double** jacobians_minimal) const
{
  Eigen::VectorXd Delta_Chi;
  computeDeltaChi(parameters, Delta_Chi);

  for (size_t i = 0; i < parameter_block_infos_.size(); ++i)
  {
    const ParameterBlockInfo& info = parameter_block_infos_.at(i);
    if (info.minimal_dimension == 0)
    {
      continue;
    }
    const Eigen::Block<const Eigen::MatrixXd> Jmin_i =
        J_.block(0, info.ordering_idx, e0_.rows(), info.minimal_dimension);

    // decompose the jacobians: minimal ones are easy
    if (jacobians_minimal != NULL)
    {
      if (jacobians_minimal[i] != NULL)
      {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                 Eigen::RowMajor> >
            Jmin_i_mapped(jacobians_minimal[i], e0_.rows(), info.minimal_dimension);
        Jmin_i_mapped = Jmin_i;
      }
    }

    // hallucinate the non-minimal Jacobians
    if (jacobians != NULL)
    {
      if (jacobians[i] != NULL)
      {
        Eigen::Map<
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                          Eigen::RowMajor> >
            J_i(jacobians[i], e0_.rows(), info.dimension);

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
            J_lift(info.parameter_block_ptr->minimalDimension(),
                   info.parameter_block_ptr->dimension());
        info.parameter_block_ptr->liftJacobian(
            info.linearization_point.get(), J_lift.data());

        J_i = Jmin_i * J_lift;
      }
    }
  }

  // finally the error (residual) e = (-pinv(J^T) * _b + _J*Delta_Chi):
  Eigen::Map<Eigen::VectorXd> e(residuals, e0_.rows());
  e = e0_ + J_ * Delta_Chi;

  return true;
}

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
