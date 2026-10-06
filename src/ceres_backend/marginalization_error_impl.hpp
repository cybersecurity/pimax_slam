// pimax_slam.pi.dll -- src/ceres_backend/marginalization_error_impl.hpp  (drafts c05 + c04)
//
// Port of svo_ceres_backend/include/svo/ceres_backend/marginalization_error_impl.hpp.
// The instantiations live in the marginalization_error.obj part of the image that lies in the
// chunk-c04 address range:
//   splitSymmetricMatrix<MatrixXd,MatrixXd,MatrixXd,MatrixXd>   0x1800732A0
//   splitVector<VectorXd,VectorXd,VectorXd>                     0x1800740F0
//   pseudoInverseSymmSqrt<Matrix3d>                             0x18006CA60
//   pseudoInverseSymmSqrt<MatrixXd>                             0x18006BF60
// pseudoInverseSymmSqrt<MatrixXd> is NOT upstream-identical (c04 correction, callees of 0x18006BF60:
// LLT compute 0x1800653D0 and L^{-T} 0x1800771C0 besides the SAES path).
// Other bodies are upstream-identical EXCEPT that every DEBUG_CHECK was removed (none of the upstream
// DEBUG_CHECK message strings exist in the binary; with asserts on they would have been emitted).
// Checked against the binary: splitVector copies the pair vector and push_back(pair((int)size,0));
// pseudoInverseSymmSqrt<Matrix3d> uses tolerance = epsilon * 3.0 * maxCoeff.
// blockPinverse / blockPinverseSqrt / pseudoInverseSymm are not instantiated in the binary.
#pragma once

#include "ceres_backend/marginalization_error.hpp"

namespace pimax {
namespace totem {
namespace ceres_backend {

// Split for Schur complement op.
template<typename Derived_A, typename Derived_U, typename Derived_W,
    typename Derived_V>
void MarginalizationError::splitSymmetricMatrix(
    const std::vector<std::pair<int, int> >& marginalization_start_idx_and_length_pairs,
    const Eigen::MatrixBase<Derived_A>& A,  // input
    const Eigen::MatrixBase<Derived_U>& U,  // output
    const Eigen::MatrixBase<Derived_W>& W,  // output
    const Eigen::MatrixBase<Derived_V>& V)  // output
{
  // sanity check
  const int size = A.cols();

  std::vector<std::pair<int, int> > marginalization_start_idx_and_length_pairs2 =
      marginalization_start_idx_and_length_pairs;
  marginalization_start_idx_and_length_pairs2.push_back(
      std::pair<int, int>(size, 0));

  const size_t length = marginalization_start_idx_and_length_pairs2.size();

  int last_idx_row = 0;
  int start_a_i = 0;
  int start_b_i = 0;
  for (size_t i = 0; i < length; ++i)
  {
    int last_idx_col = 0;
    int start_a_j = 0;
    int start_b_j = 0;
    int this_idx_row = marginalization_start_idx_and_length_pairs2[i].first;
    const int size_b_i = marginalization_start_idx_and_length_pairs2[i].second;  // kept
    const int size_a_i = this_idx_row - last_idx_row;  // kept
    for (size_t j = 0; j < length; ++j)
    {
      int this_idx_col = marginalization_start_idx_and_length_pairs2[j].first;
      const int size_a_j = this_idx_col - last_idx_col;  // kept
      const int size_b_j = marginalization_start_idx_and_length_pairs2[j].second;  // kept

      // the kept part - only access and copy if non-zero
      if (size_a_j > 0 && size_a_i > 0)
      {
        const_cast<Eigen::MatrixBase<Derived_U>&>(U).block(start_a_i, start_a_j,
                                                           size_a_i, size_a_j) =
            A.block(last_idx_row, last_idx_col, size_a_i, size_a_j);
      }

      // now the mixed part
      if (size_b_j > 0 && size_a_i > 0)
      {
        const_cast<Eigen::MatrixBase<Derived_W>&>(W).block(start_a_i, start_b_j,
                                                           size_a_i, size_b_j) =
            A.block(last_idx_row, this_idx_col, size_a_i, size_b_j);
      }

      // and finally the marginalized part
      if (size_b_j > 0 && size_b_i > 0)
      {
        const_cast<Eigen::MatrixBase<Derived_V>&>(V).block(start_b_i, start_b_j,
                                                           size_b_i, size_b_j) =
            A.block(this_idx_row, this_idx_col, size_b_i, size_b_j);
      }

      last_idx_col = this_idx_col + size_b_j;  // remember
      start_a_j += size_a_j;
      start_b_j += size_b_j;
    }
    last_idx_row = this_idx_row + size_b_i;  // remember
    start_a_i += size_a_i;
    start_b_i += size_b_i;
  }
}

// Split for Schur complement op.
template<typename Derived_b, typename Derived_b_a, typename Derived_b_b>
void MarginalizationError::splitVector(
    const std::vector<std::pair<int, int> >& marginalization_start_idx_and_length_pairs,
    const Eigen::MatrixBase<Derived_b>& b,  // input
    const Eigen::MatrixBase<Derived_b_a>& b_a,  // output
    const Eigen::MatrixBase<Derived_b_b>& b_b)  // output
{
  const int size = b.rows();

  std::vector<std::pair<int, int> > marginalization_start_idx_and_length_pairs2 =
      marginalization_start_idx_and_length_pairs;
  marginalization_start_idx_and_length_pairs2.push_back(
      std::pair<int, int>(size, 0));

  const size_t length = marginalization_start_idx_and_length_pairs2.size();

  int last_idx_row = 0;
  int start_a_i = 0;
  int start_b_i = 0;
  for (size_t i = 0; i < length; ++i)
  {
    int this_idx_row = marginalization_start_idx_and_length_pairs2[i].first;
    const int size_b_i = marginalization_start_idx_and_length_pairs2[i].second;  // marginalized
    const int size_a_i = this_idx_row - last_idx_row;  // kept

    // the kept part - only access and copy if non-zero
    if (size_a_i > 0)
    {
      const_cast<Eigen::MatrixBase<Derived_b_a>&>(b_a).segment(start_a_i,
                                                               size_a_i) = b
          .segment(last_idx_row, size_a_i);
    }

    // and finally the marginalized part
    if (size_b_i > 0)
    {
      const_cast<Eigen::MatrixBase<Derived_b_b>&>(b_b).segment(start_b_i,
                                                               size_b_i) = b
          .segment(this_idx_row, size_b_i);
    }

    last_idx_row = this_idx_row + size_b_i;  // remember
    start_a_i += size_a_i;
    start_b_i += size_b_i;
  }
}

// Pseudo inversion of a symmetric matrix.
template<typename Derived>
bool MarginalizationError::pseudoInverseSymm(
    const Eigen::MatrixBase<Derived>& a, const Eigen::MatrixBase<Derived>& result,
    double epsilon, int* rank)
{
  Eigen::SelfAdjointEigenSolver<Derived> saes(a);

  typename Derived::Scalar tolerance = epsilon * a.cols()
      * saes.eigenvalues().array().maxCoeff();

  const_cast<Eigen::MatrixBase<Derived>&>(result) = (saes.eigenvectors())
      * Eigen::VectorXd(
          (saes.eigenvalues().array() > tolerance).select(
              saes.eigenvalues().array().inverse(), 0)).asDiagonal()
      * (saes.eigenvectors().transpose());

  if (rank)
  {
    *rank = 0;
    for (int i = 0; i < a.rows(); ++i)
    {
      if (saes.eigenvalues()[i] > tolerance)
      {
        (*rank)++;
      }
    }
  }

  return true;
}

// Pseudo inversion and square root (Cholesky decomposition) of a symmetric matrix.
// 0x18006CA60  <Eigen::Matrix3d>  (landmark blocks, sdim = 3)  -- upstream-identical path
// 0x18006BF60  <Eigen::MatrixXd>  (dense part)                 -- Pimax: LLT fast path first
// Only the dynamic-size instantiation contains the LLT code; modelled as a compile-time branch
// (TODO(verify): could also be a separately named Pimax function used only by the dense part).
template<typename Derived>
bool MarginalizationError::pseudoInverseSymmSqrt(
    const Eigen::MatrixBase<Derived>& a, const Eigen::MatrixBase<Derived>& result,
    double epsilon, int* rank)
{
  // DEBUG_CHECK(a.rows() == a.cols()) compiled out

  if (Derived::RowsAtCompileTime == Eigen::Dynamic)
  {
    // Pimax: try a Cholesky first; result = L^{-T}  (result * result^T = a^{-1})
    Eigen::LLT<typename Derived::PlainObject> llt(a);                       // compute 0x1800653D0
    if (llt.info() == Eigen::Success)
    {
      const_cast<Eigen::MatrixBase<Derived>&>(result) =
          llt.matrixL().solve(Derived::Identity(a.rows(), a.cols())).transpose();  // 0x1800771C0
      if (rank)
      {
        *rank = a.rows();
      }
      return true;
    }
  }

  Eigen::SelfAdjointEigenSolver<typename Derived::PlainObject> saes(a);

  typename Derived::Scalar tolerance = epsilon * a.cols()
      * saes.eigenvalues().array().maxCoeff();

  const_cast<Eigen::MatrixBase<Derived>&>(result) = (saes.eigenvectors())
      * Eigen::VectorXd(
          Eigen::VectorXd(
              (saes.eigenvalues().array() > tolerance).select(
                  saes.eigenvalues().array().inverse(), 0)).array().sqrt())
          .asDiagonal();

  if (rank)
  {
    *rank = 0;
    for (int i = 0; i < a.rows(); ++i)
    {
      if (saes.eigenvalues()[i] > tolerance)
      {
        (*rank)++;
      }
    }
  }

  return true;
}

template<typename Derived, int blockDim>
void MarginalizationError::blockPinverse(
    const Eigen::MatrixBase<Derived>& M_in,
    const Eigen::MatrixBase<Derived>& M_out, double epsilon)
{
  const_cast<Eigen::MatrixBase<Derived>&>(M_out).resize(M_in.rows(),
                                                        M_in.rows());
  const_cast<Eigen::MatrixBase<Derived>&>(M_out).setZero();
  for (int i = 0; i < M_in.cols(); i += blockDim)
  {
    Eigen::Matrix<double, blockDim, blockDim> inv;
    const Eigen::Matrix<double, blockDim, blockDim> in = M_in
        .template block<blockDim, blockDim>(i, i);
    pseudoInverseSymm(in, inv, epsilon);
    const_cast<Eigen::MatrixBase<Derived>&>(M_out)
        .template block<blockDim, blockDim>(i, i) = inv;
  }
}

template<typename Derived, int blockDim>
void MarginalizationError::blockPinverseSqrt(
    const Eigen::MatrixBase<Derived>& M_in,
    const Eigen::MatrixBase<Derived>& M_out, double epsilon)
{
  const_cast<Eigen::MatrixBase<Derived>&>(M_out).resize(M_in.rows(),
                                                        M_in.rows());
  const_cast<Eigen::MatrixBase<Derived>&>(M_out).setZero();
  for (int i = 0; i < M_in.cols(); i += blockDim)
  {
    Eigen::Matrix<double, blockDim, blockDim> inv;
    const Eigen::Matrix<double, blockDim, blockDim> in = M_in
        .template block<blockDim, blockDim>(i, i);
    pseudoInverseSymmSqrt(in, inv, epsilon);
    const_cast<Eigen::MatrixBase<Derived>&>(M_out)
        .template block<blockDim, blockDim>(i, i) = inv;
  }
}

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
