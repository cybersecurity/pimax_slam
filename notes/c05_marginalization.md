# c05_marginalization — [0x180079D90, 0x18009A530)

Drafts: `draft/c05_marginalization/` —
`ceres_backend/{marginalization_error.h, marginalization_error_impl.h, marginalization_error.cpp,
outlier_rejection.h, outlier_rejection.cpp, pose_error.h, pose_error.cpp,
pose_local_parameterization.h, pose_local_parameterization.cpp, pose_parameter_block.h,
pose_parameter_block.cpp, speed_and_bias_error.h, speed_and_bias_error.cpp,
speed_and_bias_parameter_block.h, speed_and_bias_parameter_block.cpp, error_interface.h,
parameter_block.h}`, `common/{frame.h, frame.cpp, feature_wrapper.h, point_c05_part.cpp}`.

## 1. Object-file boundaries (from code usage, vtables and callers; no `__FILE__` except where noted)

| range | object | evidence |
|---|---|---|
| ..0x18008A080 (starts in c04 range, at the latest at 0x1800732A0) | `ceres_backend/marginalization_error.cpp` | `__FILE__` ".../marginalization_error.cpp" (CHECK line 120 in 0x180079D90); vtables 0x1803B05C0/E0; ctor 0x1800773E0, dtor 0x180077C70, Evaluate 0x180078510, EvaluateWithMinimalJacobians 0x180078540, EvaluateLocal 0x180079000, splitSymmetricMatrix 0x1800732A0, splitVector 0x1800740F0, pseudoInverseSymmSqrt<Matrix3d> 0x18006CA60 / <MatrixXd> 0x18006BF60 all lie in the **c04 range** — reproduced in my draft for completeness, coordinator please dedupe |
| 0x18008A080..0x18008AD40 | `ceres_backend/outlier_rejection.cpp` (name by link order + caller's "Outlier rejection: removed %lu edgelets and %lu corners." log, TODO(verify)) | 0x18008A7C0 called from backend interface 0x180013680 just before that LOGD |
| 0x18008AD40..0x18008C980 | `ceres_backend/pose_error.cpp` | PoseError vtables 0x1803B1318/30, LLT<6x6> |
| 0x18008C980..0x18008D960 | `ceres_backend/pose_local_parameterization.cpp` | PoseLocalParameterization vtables 0x1803ADF58/90 |
| 0x18008D960..0x18008DB80 | `ceres_backend/pose_parameter_block.cpp` | vtable 0x1803B13E0 |
| 0x18008DB80..0x18008FE60 | `ceres_backend/speed_and_bias_error.cpp` | vtables 0x1803B1480/98, LLT<9x9> |
| 0x18008FE60..0x180090180 | `ceres_backend/speed_and_bias_parameter_block.cpp` | vtable 0x1803B14D8 |
| 0x180090180..0x180096E90 | `common/frame.cpp` | Frame vtable 0x1803B1588, "Frame %d has no obs!" |
| 0x180096E90..0x180097EC0 | std::basic_filebuf/basic_ofstream<char> COMDATs (vtables 0x1803B1640/0x1803B16C0); owner ambiguous (no frame/point function uses them; first users are 0x180125040 ff.) | — |
| 0x180097F70..(c06) | `common/point.cpp` **starts here**, not at 0x18009A530: point.obj COMDATs (KeypointIdentifier vector, unordered_map<int,...>, 3x3 Eigen solver used only by Point::optimize 0x18009B240 / 0x180158050), then KeypointIdentifier ctor **0x180099F40** and Point ctor **0x180099F80** (project, drafted in `common/point_c05_part.cpp` for c06 to merge) | |

Note on in-object order: MSVC did not emit functions in source order (e.g. computeDeltaChi sits between
addResidualBlock and getParameterBlockPtrs; Evaluate* precede addResidualBlock).

## 2. Summary of the Pimax changes

* **MarginalizationError** (upstream OKVIS/SVO-pro) was restructured into a *deferred* linearisation:
  `addResidualBlock()` (now `void`) only books parameter blocks (ParameterBlockInfo, ordering,
  ceres `parameter_block_sizes_`/`num_residuals_`), counts new dense/landmark blocks and their minimal
  dimension, and stores the residual id, a copy of its `Map::ResidualBlockSpec` and its parameter-id list;
  then removes it from the Map. `marginalizeOut()` first calls the new `linearizeResidualBlocks()`
  (0x18007B280) which renumbers the landmark part, rebuilds H_/b0_ at the new size (old dense block
  top-left, old landmark block shifted behind the new dense blocks, new rows/cols zero), evaluates all
  booked residuals at their linearisation points (minimal Jacobians only, loss-function correction =
  upstream code) and accumulates J^T J / -J^T r, then clears the booking. The Schur parts and
  updateErrorComputation are upstream-identical. Landmarks are `General3DParameterBlock` (sdim 3).
  `std::stable_sort` replaces `std::sort`; `keep_parameter_blocks` is gone; the id->index map is an
  `std::unordered_map`; all DEBUG_CHECK/check()/LOG(ERROR)/LOG(FATAL) are gone.
  New CostFunction virtual `EvaluateLocal()` (slot 2) returns Jacobians in local coordinates
  `Jmin*J_lift(x_lin)*J_plus(x)`; `Evaluate()` dispatches on the new ErrorInterface bool.
* **ErrorInterface** got a `bool` at +8 (`use_minimal_jacobians_`, name TODO). The folded
  `Evaluate()` of ImuError/PoseError/SpeedAndBiasError (0x18003AE00) passes `jacobians` as
  `jacobians_minimal` when it is set. No writer of `true` was found.
* **PoseError** is a position + yaw prior: residual = [(p_meas-p)*s00, 0, 0, 2*dq.z*s55]; information_
  and covariance_ members removed (sizeof 416); quaternion of the estimate normalised.
* **SpeedAndBiasError** weights each 3-block with a single diagonal entry of sqrt_info; J = -sqrt_info.
* **PoseLocalParameterization**: minus/plusJacobian/liftJacobian normalise their quaternions.
* **OutlierRejection::removeOutliers** no longer removes anything; it counts inliers.
* **Frame** layout differs a lot (float features, extra fields, image-statistics flags); see §4.

## 3. Function table (every function in range)

`project` rows are reconstructed in the drafts; `upstream status` relative to rpg_svo_pro_open.

| address | proposed qualified name | signature | kind | upstream status | summary |
|---|---|---|---|---|---|
| 0x180079D90 | pimax::totem::ceres_backend::MarginalizationError::addResidualBlock | void(ceres::ResidualBlockId, bool keep) | project | modified: void, deferred linearisation, General3D landmarks, CHECK(parameters!=nullptr) l.120 | book param blocks/ordering/sizes, store residual id/spec/param ids, remove from Map |
| 0x18007A840 | lib:std::vector<ParameterBlockInfo>::_Allocate (80-byte elems) |  | lib:std::vector | - |  |
| 0x18007A8B0 | lib:std::unordered_map<uint64_t,size_t>::at (ICF-folded with <uint64_t,bool>) |  | lib:std::unordered_map | - | "invalid unordered_map<K, T> key" |
| 0x18007A980 | lib:std::vector<ParameterBlockInfo>::at |  | lib:std::vector | - |  |
| 0x18007A9D0 | lib:std::vector<Eigen::MatrixXd,aligned_allocator>::at |  | lib:std::vector | - |  |
| 0x18007AA20 | pimax::totem::ceres_backend::MarginalizationError::computeDeltaChi | bool(double const* const*, Eigen::VectorXd&) const | project | identical | stack minus(lin_pt, x_i) into DeltaChi |
| 0x18007AD00 | lib:std::allocator<ParameterBlockInfo>::deallocate |  | lib:std | - |  |
| 0x18007AD50 | lib:Eigen::Diagonal<MatrixXd> ctor (assert) |  | lib:Eigen | - |  |
| 0x18007AD90 | lib:Eigen::SelfAdjointEigenSolver<MatrixXd>::eigenvectors() (m_isInitialized/m_eigenvectorsOk asserts) |  | lib:Eigen | - |  |
| 0x18007ADF0 | lib:std::unordered_map<uint64_t,size_t>::erase(const key&) |  | lib:std::unordered_map | - |  |
| 0x18007AF30 | lib:std::vector<int32_t>::erase(const_iterator) |  | lib:std::vector | - | ceres parameter_block_sizes_ |
| 0x18007AF80 | lib:std::vector<ParameterBlockInfo>::erase(const_iterator) |  | lib:std::vector | - |  |
| 0x18007B0E0 | lib:Eigen Block<...> (column tail) ctor + asserts |  | lib:Eigen | - |  |
| 0x18007B1F0 | pimax::totem::ceres_backend::MarginalizationError::getParameterBlockPtrs | void(std::vector<std::shared_ptr<ParameterBlock>>&) | project | modified: no DEBUG_CHECK, range-for |  |
| 0x18007B280 | pimax::totem::ceres_backend::MarginalizationError::linearizeResidualBlocks (name TODO) | void() | project | new | renumber landmark part, rebuild H_/b0_, evaluate+loss-correct booked residuals, accumulate, clear booking |
| 0x180080320 | pimax::totem::ceres_backend::MarginalizationError::marginalizeOut | bool(const std::vector<uint64_t>&) | project | modified: calls linearizeResidualBlocks first, no keep arg, stable_sort, no LOG/DEBUG_CHECK/LOG(FATAL)/check() | landmark (3x3 block Schur) + dense Schur, bookkeeping, remove param blocks from Map |
| 0x180087010 | pimax::totem::ceres_backend::MarginalizationError::residualDim | size_t() const (ErrorInterface slot 1) | project | identical | return num_residuals() |
| 0x180087020 | lib:Eigen resize_if_allowed assert (dst.rows()==rows && cols) |  | lib:Eigen | - |  |
| 0x180087050 | lib:Eigen call_dense_assignment (diag*M*diag lazy product) |  | lib:Eigen | - | called from c04 0x1800658E0 |
| 0x180087150 | lib:Eigen call_dense_assignment (diag*M*diag variant) |  | lib:Eigen | - | called from c04 0x180065E90 |
| 0x180087290 | lib:Eigen product/rank-update kernel (6 args) |  | lib:Eigen | - | called from c04 0x18006D1A0 |
| 0x180087B70 | lib:Eigen general_matrix_matrix_product (gemm, blocking heuristic) |  | lib:Eigen | - | called from c04 0x180064E00 |
| 0x180088370 | lib:Eigen selfadjoint rank-2 update kernel |  | lib:Eigen | - | called from c04 0x180075470 |
| 0x180088700 | lib:Eigen dense assignment loop (lazy product) |  | lib:Eigen | - |  |
| 0x180088870 | lib:Eigen dense assignment loop (lazy product) |  | lib:Eigen | - |  |
| 0x180088A50 | lib:Eigen Matrix3d lower-triangle / scale (SelfAdjointEigenSolver<Matrix3d>::computeDirect) |  | lib:Eigen | - |  |
| 0x180088AE0 | lib:Eigen dense assignment loop |  | lib:Eigen | - |  |
| 0x180088BD0 | lib:Eigen dense assignment loop |  | lib:Eigen | - |  |
| 0x180088CD0 | lib:std::vector<ParameterBlockInfo>::size |  | lib:std::vector | - |  |
| 0x180088D00 | pimax::totem::ceres_backend::MarginalizationError::typeInfo | ErrorType() const | project | identical | returns 3 |
| 0x180088D10 | pimax::totem::ceres_backend::MarginalizationError::updateErrorComputation | void() | project | identical | eigen-decomposition, J_, e0_ |
| 0x18008A080 | lib:Eigen::Quaternionf from Matrix3f (quaternionbase_assign_substitution) |  | lib:Eigen | - | outlier_rejection.obj |
| 0x18008A270 | lib:minkindr QuatTransformationTemplate<double>::cast<float>() |  | lib:minkindr | - |  |
| 0x18008A460 | lib:minkindr RotationQuaternionTemplate<double>::cast<float>() |  | lib:minkindr | - |  |
| 0x18008A610 | lib:minkindr RotationQuaternionTemplate<float>(const Eigen::Quaternionf&) (EPS norm CHECKs, line 73) |  | lib:minkindr | - |  |
| 0x18008A7C0 | pimax::totem::OutlierRejection::removeOutliers | void(Frame&, size_t&, size_t&, std::vector<int>&, int& n_inliers) const | project | modified (rewritten): only counts inliers (err<=2.5px, 0<z<=10) | called from backend interface 0x180013680 |
| 0x18008ACB0 | pimax::totem::OutlierRejection::depthInFrame (inline helper, name TODO) | float(const TransformationF&, const Eigen::Vector3f&) | project | new | z of T*p (float/double mix) |
| 0x18008AD40 | lib:Eigen::LLT<Matrix<double,6,6>>::compute |  | lib:Eigen | - | pose_error.obj |
| 0x18008B030 | lib:Eigen l1 norm (LLT<6x6> m_l1_norm) |  | lib:Eigen | - |  |
| 0x18008B0D0 | lib:Eigen LLT helper (6x6 max abs col sum) |  | lib:Eigen | - | also used by 0x18012FAF0 |
| 0x18008B170 | lib:Eigen llt_inplace blocked helper (6x6) |  | lib:Eigen | - |  |
| 0x18008B320 | lib:Eigen llt_inplace<Lower>::unblocked (6x6) |  | lib:Eigen | - |  |
| 0x18008B930 | lib:Eigen Map<Matrix<double,6,1>> ctor asserts |  | lib:Eigen | - | used by many |
| 0x18008B9E0 | lib:ceres::SizedCostFunction<6,7>::SizedCostFunction |  | lib:ceres | - |  |
| 0x18008BAC0 | pimax::totem::ceres_backend::PoseError::PoseError | (const Transformation&, const Matrix6d&) | project | identical |  |
| 0x18008BBD0 | pimax::totem::ceres_backend::PoseError::`vector deleting dtor' thunk (ErrorInterface, this-0x28) |  | project | identical |  |
| 0x18008BBE0 | pimax::totem::ceres_backend::PoseError::`scalar deleting dtor' |  | project | identical |  |
| 0x18008BC30 | pimax::totem::ceres_backend::PoseError::EvaluateWithMinimalJacobians | bool(...) const | project | modified: position*s00 + yaw*s55 residual, row-5 rotation Jacobian, q normalised |  |
| 0x18008C600 | lib:Eigen triangular assign unroller helper (6x6) |  | lib:Eigen | - |  |
| 0x18008C620 | lib:Eigen triangular assign zero helper (6x6) |  | lib:Eigen | - |  |
| 0x18008C640 | lib:Eigen triangular_assignment (L^T upper) 6x6 |  | lib:Eigen | - |  |
| 0x18008C820 | pimax::totem::ceres_backend::PoseError::setInformation | void(const Matrix6d&) | project | modified: no information_/covariance_ | LLT -> L^T |
| 0x18008C970 | pimax::totem::ceres_backend::PoseError::typeInfo | ErrorType() const | project | identical | returns 4 |
| 0x18008C980 | pimax::totem::ceres_backend::PoseLocalParameterization::ComputeJacobian (== PoseParameterBlock::plusJacobian, ICF) | bool(const double*, double*) const | project | identical | thunk to plusJacobian |
| 0x18008C990 | pimax::totem::ceres_backend::PoseLocalParameterization::ComputeLiftJacobian | bool(const double*, double*) const | project | modified (inlined liftJacobian, q_inv normalised) |  |
| 0x18008CC20 | pimax::totem::ceres_backend::PoseLocalParameterization::Minus (== PoseParameterBlock::minus, ICF) |  | project | identical | thunk |
| 0x18008CC40 | pimax::totem::ceres_backend::PoseLocalParameterization::Plus (== PoseParameterBlock::plus, ICF) |  | project | identical | thunk |
| 0x18008CC60 | pimax::totem::ceres_backend::PoseLocalParameterization::liftJacobian (static) | bool(const double*, double*) | project | modified: q_inv.normalized() |  |
| 0x18008CEE0 | pimax::totem::ceres_backend::PoseLocalParameterization::minus (static) | bool(const double*, const double*, double*) | project | modified: q_diff.normalized() |  |
| 0x18008D270 | pimax::totem::ceres_backend::PoseLocalParameterization::plus (static) | bool(const double*, const double*, double*) | project | identical (Pimax minkindr semantics) |  |
| 0x18008D570 | pimax::totem::ceres_backend::PoseLocalParameterization::plusJacobian (static) | bool(const double*, double*) | project | modified: q normalised |  |
| 0x18008D960 | pimax::totem::ceres_backend::PoseParameterBlock::PoseParameterBlock | (const Transformation&, uint64_t) | project | identical |  |
| 0x18008D9C0 | pimax::totem::ceres_backend::PoseParameterBlock::`scalar deleting dtor' |  | project | identical |  |
| 0x18008DA00 | pimax::totem::ceres_backend::PoseParameterBlock::dimension (ICF: also pimax::totem::PoseLocalParameterization slot 4) | size_t() const | project | identical | returns 7 |
| 0x18008DA10 | pimax::totem::ceres_backend::PoseParameterBlock::estimate | Transformation() const | project | identical |  |
| 0x18008DAF0 | pimax::totem::ceres_backend::PoseParameterBlock::liftJacobian | void(const double*, double*) const | project | identical | thunk |
| 0x18008DB00 | pimax::totem::ceres_backend::PoseParameterBlock::setEstimate | void(const Transformation&) | project | identical |  |
| 0x18008DB40 | pimax::totem::ceres_backend::PoseParameterBlock::typeInfo | std::string() const | project | identical | "PoseParameterBlock" |
| 0x18008DB80 | lib:Eigen::LLT<Matrix<double,9,9>>::compute |  | lib:Eigen | - | speed_and_bias_error.obj |
| 0x18008DFB0 | lib:Eigen l1 norm (LLT<9x9>) |  | lib:Eigen | - |  |
| 0x18008E050 | lib:Eigen gemv helper (LLT 9x9 blocked) |  | lib:Eigen | - |  |
| 0x18008E0C0 | lib:Eigen scalar*segment assignment loop (SpeedAndBiasError residual) |  | lib:Eigen | - |  |
| 0x18008E1E0 | lib:Eigen llt_inplace<Lower>::unblocked (9x9) |  | lib:Eigen | - |  |
| 0x18008E6B0 | lib:Eigen Map/Block<1x9> ctor asserts |  | lib:Eigen | - |  |
| 0x18008E760 | lib:Eigen Map/Block<9x1> ctor asserts |  | lib:Eigen | - |  |
| 0x18008E810 | lib:ceres::SizedCostFunction<9,9>::SizedCostFunction |  | lib:ceres | - |  |
| 0x18008E8F0 | pimax::totem::ceres_backend::SpeedAndBiasError::SpeedAndBiasError | (const SpeedAndBias&, const Matrix9d&) | project | identical |  |
| 0x18008E980 | pimax::totem::ceres_backend::SpeedAndBiasError::SpeedAndBiasError | (const SpeedAndBias&, double, double, double) | project | identical (no DEBUG_CHECK_NE) |  |
| 0x18008EC08 | pimax::totem::ceres_backend::SpeedAndBiasError dtor thunk (this-0x28) |  | project | identical |  |
| 0x18008EC20 | pimax::totem::ceres_backend::SpeedAndBiasError::`scalar deleting dtor' |  | project | identical |  |
| 0x18008EC70 | pimax::totem::ceres_backend::SpeedAndBiasError::EvaluateWithMinimalJacobians | bool(...) const | project | modified: diagonal-block weighting, J=-sqrt_info |  |
| 0x18008F450 | lib:Eigen triangular assign helper (9x9) |  | lib:Eigen | - |  |
| 0x18008F470 | lib:Eigen triangular assign zero helper (9x9) |  | lib:Eigen | - |  |
| 0x18008F490 | pimax::totem::ceres_backend::SpeedAndBiasError::residualDim / SpeedAndBiasParameterBlock::dimension,minimalDimension (ICF) | size_t() const | project | identical | returns 9 |
| 0x18008F4A0 | lib:Eigen triangular_assignment (L^T) 9x9 entry |  | lib:Eigen | - |  |
| 0x18008F510 | lib:Eigen triangular assign unroller (9x9) part 1 |  | lib:Eigen | - |  |
| 0x18008F6E0 | lib:Eigen triangular assign unroller (9x9) part 2 |  | lib:Eigen | - |  |
| 0x18008F8F0 | lib:Eigen triangular assign unroller (9x9) part 3 |  | lib:Eigen | - |  |
| 0x18008FB10 | pimax::totem::ceres_backend::SpeedAndBiasError::setInformation | void(const Matrix9d&) | project | identical |  |
| 0x18008FE50 | pimax::totem::ceres_backend::SpeedAndBiasError::typeInfo | ErrorType() const | project | identical | returns 2 |
| 0x18008FE60 | pimax::totem::ceres_backend::SpeedAndBiasParameterBlock::SpeedAndBiasParameterBlock | (const SpeedAndBias&, uint64_t) | project | identical |  |
| 0x18008FEB0 | pimax::totem::ceres_backend::SpeedAndBiasParameterBlock::`scalar deleting dtor' |  | project | identical |  |
| 0x18008FEF0 | pimax::totem::ceres_backend::SpeedAndBiasParameterBlock::plusJacobian / liftJacobian (ICF) | void(const double*, double*) const | project | identical | 9x9 identity |
| 0x180090050 | pimax::totem::ceres_backend::SpeedAndBiasParameterBlock::minus |  | project | identical |  |
| 0x1800900B0 | pimax::totem::ceres_backend::SpeedAndBiasParameterBlock::plus |  | project | identical |  |
| 0x180090110 | pimax::totem::ceres_backend::SpeedAndBiasParameterBlock::setEstimate |  | project | identical |  |
| 0x180090140 | pimax::totem::ceres_backend::SpeedAndBiasParameterBlock::typeInfo | std::string() const | project | identical | "SpeedAndBiasParameterBlock" |
| 0x180090180 | lib:std::vector<cv::Mat>::_Insert_range / assign |  | lib:std::vector | - | frame.obj COMDAT |
| 0x180090430 | lib:std::_Uninitialized_move (24-byte elems) |  | lib:std | - |  |
| 0x1800904B0 | lib:std::string::_Reallocate_grow_by (append) |  | lib:std::string | - |  |
| 0x1800905D0 | lib:std::_Destroy_range<cv::Mat> |  | lib:std | - |  |
| 0x180090610 | lib:std::_Destroy_range thunk |  | lib:std | - |  |
| 0x180090630 | lib:std::_Destroy_range<cv::Mat> |  | lib:std | - |  |
| 0x180090670 | lib:std::_Uninitialized_copy<cv::Mat> |  | lib:std | - |  |
| 0x1800906D0 | lib:std::allocator<cv::Mat>::deallocate |  | lib:std | - |  |
| 0x180090720 | lib:Eigen::Ref<const Matrix2Xd> from float Keypoints cast (internal object) |  | lib:Eigen | - |  |
| 0x1800908C0 | lib:std::_Destroy_range<SeedRef> |  | lib:std | - |  |
| 0x180090940 | lib:std::_Med3/_Guess_median<float> (nth_element) |  | lib:std | - |  |
| 0x180090AD0 | lib:std::_Partition_by_median_guess_unchecked<float> |  | lib:std | - |  |
| 0x180090DB0 | lib:std::vector<SeedRef>::_Resize_reallocate |  | lib:std::vector | - |  |
| 0x180090F90 | lib:std::vector<cv::Mat>::_Resize_reallocate |  | lib:std::vector | - |  |
| 0x1800910D0 | lib:std::vector<PointPtr>::_Resize_reallocate |  | lib:std::vector | - |  |
| 0x180091280 | lib:std::vector<FeatureType>::_Resize_reallocate |  | lib:std::vector | - |  |
| 0x1800913E0 | lib:Eigen::internal::conservative_aligned_realloc<4-byte> (realloc) |  | lib:Eigen | - |  |
| 0x180091430 | lib:std::_Hash::emplace (unordered_set/map, 8-byte key) |  | lib:std::unordered_map | - | "unordered_map/set too long" |
| 0x180091700 | lib:vk::getMedian<float> | float(std::vector<float>&) | lib:vikit | - | nth_element median (index (int)floor(size/2)) |
| 0x180091880 | lib:Eigen setConstant<int> scalar loop |  | lib:Eigen | - |  |
| 0x1800918B0 | lib:Eigen setConstant<float> scalar loop |  | lib:Eigen | - |  |
| 0x180091930 | lib:Eigen float copy scalar loop |  | lib:Eigen | - |  |
| 0x1800919B0 | lib:Eigen Block<Matrix<float,4,Dynamic>,4,1> ctor asserts |  | lib:Eigen | - |  |
| 0x180091A60 | lib:Eigen Block<Matrix<float,2,Dynamic>,2,1> ctor asserts |  | lib:Eigen | - |  |
| 0x180091B10 | lib:std::deque<T,aligned_allocator> ctor (allocates _Container_proxy) |  | lib:std::deque | - | FrameBundle+0x18 |
| 0x180091B90 | lib:std::vector<double>(initializer_list) / copy |  | lib:std::vector | - | Frame+0x1B0 {1.5,3,6,12} |
| 0x180091C80 | lib:std::vector<FramePtr> copy ctor |  | lib:std::vector | - |  |
| 0x180091DA0 | pimax::totem::Frame::Frame | (const CameraPtr&, const cv::Mat&, int64_t, size_t, int, int, int, const FrameExtraOptions&, uint16_t) | project | modified: 5 extra args, initFrame inlined w/o CHECKs/colour branch |  |
| 0x180092420 | pimax::totem::Frame::Frame | (int, int64_t, const CameraPtr&, const Transformation&) | project | identical |  |
| 0x1800928D0 | pimax::totem::FrameBundle::FrameBundle | (const std::vector<FramePtr>&) | project | modified: parallel_for_ lambda blurs img_pyr_[0] in place, stores mean/dark/bright |  |
| 0x180092BF0 | lib:EH unwind helper (free member +0x28) |  | lib:crt | - |  |
| 0x180092C00 | lib:EH unwind helper |  | lib:crt | - |  |
| 0x180092C20 | lib:EH unwind helper |  | lib:crt | - |  |
| 0x180092C50 | lib:std::vector<SeedRef>::_Tidy |  | lib:std::vector | - |  |
| 0x180092CE0 | lib:std::vector<PointPtr>::_Tidy thunk |  | lib:std::vector | - |  |
| 0x180092CF0 | lib:std::vector<cv::Mat>::_Tidy thunk |  | lib:std::vector | - |  |
| 0x180092D00 | lib:EH unwind helper |  | lib:crt | - |  |
| 0x180092D60 | pimax::totem::Frame::~Frame (body) |  | project | modified: releases pyramid + shrink_to_fit |  |
| 0x180093020 | lib:cv::ParallelLoopBodyLambdaWrapper::~ParallelLoopBodyLambdaWrapper |  | lib:opencv | - |  |
| 0x180093060 | lib:Eigen conservativeResize copy helper (4xN float) |  | lib:Eigen | - |  |
| 0x1800932D0 | lib:minkindr QuatTransformationTemplate<float>::transform(Vector3f) |  | lib:minkindr | - |  |
| 0x1800933E0 | pimax::totem::FrameBundle::FrameBundle lambda(const cv::Range&)::operator() | void(const cv::Range&) const | project | new | bundle id, 3x3 blur, mean, dark/bright flags |
| 0x1800935C0 | lib:cv::ParallelLoopBodyLambdaWrapper::operator() (std::function call) |  | lib:opencv | - |  |
| 0x1800935E0 | pimax::totem::Frame::`scalar deleting dtor' |  | project | identical | vtable 0x1803B1588 slot 0 |
| 0x180093630 | lib:cv::ParallelLoopBodyLambdaWrapper::`scalar deleting dtor' |  | lib:opencv | - |  |
| 0x1800936A0 | lib:std::vector<cv::Mat>::_Change_array |  | lib:std::vector | - |  |
| 0x180093780 | lib:std::_Func_impl_no_alloc<lambda>::_Copy |  | lib:std::function | - |  |
| 0x1800937A0 | lib:std::_Func_impl_no_alloc<lambda>::_Delete_this |  | lib:std::function | - |  |
| 0x1800937B0 | lib:std::_Destroy_range<SeedRef> thunk |  | lib:std | - |  |
| 0x1800937D0 | lib:std::_Func_impl_no_alloc<lambda>::_Do_call |  | lib:std::function | - | -> 0x1800933E0 |
| 0x1800937E0 | lib:std::_Hash::_Forced_rehash |  | lib:std::unordered_map | - | "invalid hash bucket count" |
| 0x1800939B0 | lib:std::_Func_impl::_Get (returns this+8) / ICF-folded _LocaleUpdate::GetLocaleT |  | lib:std::function | - |  |
| 0x1800939C0 | lib:std::vector<cv::Mat>::shrink_to_fit (_Reallocate) |  | lib:std::vector | - |  |
| 0x180093A70 | lib:std::_Func_impl_no_alloc<lambda>::_Target_type |  | lib:std::function | - |  |
| 0x180093A80 | lib:std::vector<cv::Mat>::_Tidy |  | lib:std::vector | - |  |
| 0x180093B40 | lib:std::allocator<cv::Mat>::allocate |  | lib:std | - |  |
| 0x180093BB0 | pimax::totem::frame_utils::computeNormalizedBearingVectors | void(const Keypoints&, const Camera&, Bearings*, Bearings*) | project | modified: float, extra un-normalised output |  |
| 0x1800942B0 | pimax::totem::frame_utils::createImgPyramid | void(const cv::Mat&, int, ImgPyr&) | project | modified: cv::pyrDown, no CHECKs |  |
| 0x180094460 | pimax::totem::Frame::deleteLandmark | void(const size_t&) | project | modified: also track_id = -1 |  |
| 0x180094540 | pimax::totem::Frame::getErrorMultiplier | double() const | project | identical |  |
| 0x180094550 | pimax::totem::Frame::getEmptyFeatureWrapper | FeatureWrapper() | project | identical |  |
| 0x180094570 | pimax::totem::Frame::getAngleError | double(double) const | project | identical |  |
| 0x180094580 | pimax::totem::Frame::getFeatureWrapper | FeatureWrapper(size_t) | project | modified: no CHECK, extra f_raw Ref |  |
| 0x180094880 | pimax::totem::Frame::getMask | const cv::Mat&() const | project | identical | cam_->mask (+0x38) |
| 0x180094890 | pimax::totem::frame_utils::getSceneDepth | bool(const FramePtr&) | project | modified (rewritten) | LOGW "Frame %d has no obs!\n" |
| 0x180094EC0 | pimax::totem::Frame::isSaturatedPatch (name TODO) | bool(const Eigen::Vector2i&) const | project | new | 5x5 window >80% pixels >220 |
| 0x180094FB0 | pimax::totem::Frame::isVisible | bool(const Eigen::Vector3d&, Eigen::Vector2d*) const | project | modified: z>=0 and hard-coded 16..624 x 16..464 bounds |  |
| 0x180095150 | pimax::totem::FrameBundle::numFeatures | size_t() const | project | identical |  |
| 0x180095180 | pimax::totem::FrameBundle::numTrackIds (name TODO) | size_t() const | project | new | count track_id > -1 |
| 0x180095240 | pimax::totem::FrameBundle::numLandmarksInBA | size_t() const | project | identical |  |
| 0x1800952D0 | pimax::totem::FrameBundle::numTrackedLandmarks | size_t() const | project | modified: predicate (t&0xF9)==0 && t!=2 |  |
| 0x180095360 | pimax::totem::FrameBundle::numTrackedFeatures | size_t() const | project | modified: landmark || cornerEdgeletSeed |  |
| 0x1800953F0 | pimax::totem::FrameBundle::numLandmarks | size_t() const | project | modified: counts first num_features_ slots |  |
| 0x180095450 | pimax::totem::Frame::w2c (name TODO) | Eigen::Vector2d(const Eigen::Vector3d&, Eigen::Vector2d*) const | project | new | project world point |
| 0x1800955C0 | pimax::totem::Frame::f2c (name TODO) | Eigen::Vector2d(const Eigen::Vector3d&, Eigen::Vector2d*) const | project | new | project camera point |
| 0x180095730 | pimax::totem::Frame::resizeFeatureStorage | void(size_t) | project | modified: n_new from cols, f_vec_raw_, no in_ba_graph_vec_, no error log |  |
| 0x180095ED0 | lib:minkindr QuatTransformationTemplate<float>::transform (variant) |  | lib:minkindr | - | used by 0x1800AE820 |
| 0x180096000 | lib:Eigen::PlainObjectBase<Matrix<float,2,Dynamic>>::conservativeResize |  | lib:Eigen | - |  |
| 0x180096260 | lib:Eigen::PlainObjectBase<Matrix<float,3,Dynamic>>::conservativeResize |  | lib:Eigen | - |  |
| 0x1800964C0 | lib:Eigen::PlainObjectBase<Matrix<float,4,Dynamic>>::conservativeResize |  | lib:Eigen | - |  |
| 0x180096730 | pimax::totem::Frame::setKeyPoints | void() | project | modified: principal point, track-id test, fixed quadrant tests |  |
| 0x180096E80 | pimax::totem::Frame::setKeyframe | void() | project | identical |  |
| 0x180096E90 | lib:std::basic_filebuf<char> / basic_ofstream<char> (vtable 0x1803B1640 / 0x1803B16C0) member |  | lib:std::fstream | - | ambiguous owner (frame.obj tail / point.obj head) |
| 0x180096EB0 | lib:std::basic_filebuf<char> / basic_ofstream<char> (vtable 0x1803B1640 / 0x1803B16C0) member |  | lib:std::fstream | - | ambiguous owner (frame.obj tail / point.obj head) |
| 0x180096ED0 | lib:std::basic_filebuf<char> / basic_ofstream<char> (vtable 0x1803B1640 / 0x1803B16C0) member |  | lib:std::fstream | - | ambiguous owner (frame.obj tail / point.obj head) |
| 0x180097070 | lib:std::basic_filebuf<char> / basic_ofstream<char> (vtable 0x1803B1640 / 0x1803B16C0) member |  | lib:std::fstream | - | ambiguous owner (frame.obj tail / point.obj head) |
| 0x180097160 | lib:std::basic_filebuf<char> / basic_ofstream<char> (vtable 0x1803B1640 / 0x1803B16C0) member |  | lib:std::fstream | - | ambiguous owner (frame.obj tail / point.obj head) |
| 0x1800971D0 | lib:std::basic_filebuf<char> / basic_ofstream<char> (vtable 0x1803B1640 / 0x1803B16C0) member |  | lib:std::fstream | - | ambiguous owner (frame.obj tail / point.obj head) |
| 0x180097480 | lib:std::basic_filebuf<char> / basic_ofstream<char> (vtable 0x1803B1640 / 0x1803B16C0) member |  | lib:std::fstream | - | ambiguous owner (frame.obj tail / point.obj head) |
| 0x1800975F0 | lib:std::basic_filebuf<char> / basic_ofstream<char> (vtable 0x1803B1640 / 0x1803B16C0) member |  | lib:std::fstream | - | ambiguous owner (frame.obj tail / point.obj head) |
| 0x1800976D0 | lib:std::basic_filebuf<char> / basic_ofstream<char> (vtable 0x1803B1640 / 0x1803B16C0) member |  | lib:std::fstream | - | ambiguous owner (frame.obj tail / point.obj head) |
| 0x1800977A0 | lib:std::basic_filebuf<char> / basic_ofstream<char> (vtable 0x1803B1640 / 0x1803B16C0) member |  | lib:std::fstream | - | ambiguous owner (frame.obj tail / point.obj head) |
| 0x180097880 | lib:std::basic_filebuf<char> / basic_ofstream<char> (vtable 0x1803B1640 / 0x1803B16C0) member |  | lib:std::fstream | - | ambiguous owner (frame.obj tail / point.obj head) |
| 0x180097960 | lib:std::basic_filebuf<char> / basic_ofstream<char> (vtable 0x1803B1640 / 0x1803B16C0) member |  | lib:std::fstream | - | ambiguous owner (frame.obj tail / point.obj head) |
| 0x1800979B0 | lib:std::basic_filebuf<char> / basic_ofstream<char> (vtable 0x1803B1640 / 0x1803B16C0) member |  | lib:std::fstream | - | ambiguous owner (frame.obj tail / point.obj head) |
| 0x180097A00 | lib:std::basic_filebuf<char>::`scalar deleting dtor' |  | lib:std::fstream | - |  |
| 0x180097A40 | lib:std::basic_filebuf<char> / basic_ofstream<char> (vtable 0x1803B1640 / 0x1803B16C0) member |  | lib:std::fstream | - | ambiguous owner (frame.obj tail / point.obj head) |
| 0x180097A7C | lib:std::basic_filebuf<char> / basic_ofstream<char> (vtable 0x1803B1640 / 0x1803B16C0) member |  | lib:std::fstream | - | ambiguous owner (frame.obj tail / point.obj head) |
| 0x180097A90 | lib:std::use_facet<std::codecvt<char,char,_Mbstatet>> |  | lib:std | - |  |
| 0x180097B60 | lib:std::ios_base / basic_ios init helper |  | lib:std | - |  |
| 0x180097C30 | lib:std::bad_cast ctor |  | lib:crt | - | "bad cast" |
| 0x180097C60 | lib:std::basic_filebuf<char>::~basic_filebuf |  | lib:std::fstream | - |  |
| 0x180097CD0 | lib:std::basic_filebuf<char> / basic_ofstream<char> (vtable 0x1803B1640 / 0x1803B16C0) member |  | lib:std::fstream | - | ambiguous owner (frame.obj tail / point.obj head) |
| 0x180097D40 | lib:std::basic_filebuf<char> / basic_ofstream<char> (vtable 0x1803B1640 / 0x1803B16C0) member |  | lib:std::fstream | - | ambiguous owner (frame.obj tail / point.obj head) |
| 0x180097D80 | lib:std::basic_filebuf<char> / basic_ofstream<char> (vtable 0x1803B1640 / 0x1803B16C0) member |  | lib:std::fstream | - | ambiguous owner (frame.obj tail / point.obj head) |
| 0x180097E70 | lib:std::basic_filebuf<char>::_Reset_back |  | lib:std::fstream | - |  |
| 0x180097EA0 | lib:Concurrency::cancel_current_task / std::_Throw_bad_cast |  | lib:crt | - | "bad cast" |
| 0x180097EC0 | lib:std::basic_filebuf<char> / basic_ofstream<char> (vtable 0x1803B1640 / 0x1803B16C0) member |  | lib:std::fstream | - | ambiguous owner (frame.obj tail / point.obj head) |
| 0x180097F70 | lib:std::_Destroy_range<KeypointIdentifier> (weak_ptr release) |  | lib:std | - | point.obj COMDAT |
| 0x180097FF0 | lib:std::vector<KeypointIdentifier>::_Emplace_reallocate |  | lib:std | - | point.obj COMDAT |
| 0x180098170 | lib:std::_Uninitialized_move<KeypointIdentifier> |  | lib:std | - | point.obj COMDAT |
| 0x1800981E0 | lib:Eigen ColPivHouseholderQR/FullPivLU<Matrix3d>::solve (Point::optimize) |  | lib:Eigen | - | point.obj COMDAT |
| 0x1800985A0 | lib:Eigen 3x3 decomposition compute (Point::optimize) |  | lib:Eigen | - | point.obj COMDAT |
| 0x180098860 | lib:std::unordered_map<int,KeypointIdentifier>::emplace (Point::addObservation) |  | lib:std | - | point.obj COMDAT |
| 0x180098B50 | lib:Eigen 3x3 decomposition helper (pivot search) |  | lib:Eigen | - | point.obj COMDAT |
| 0x180098CF0 | lib:Eigen 3x3 decomposition helper (Householder apply) |  | lib:Eigen | - | point.obj COMDAT |
| 0x180098EA0 | lib:Eigen 3x3 decomposition core loop |  | lib:Eigen | - | point.obj COMDAT |
| 0x180099E90 | lib:Eigen Block<...,1,2> ctor asserts |  | lib:Eigen | - | point.obj COMDAT |
| 0x180099F40 | pimax::totem::KeypointIdentifier::KeypointIdentifier | (const FramePtr&, size_t) | project | modified: + bundle id | point.obj (belongs with c06) |
| 0x180099F80 | pimax::totem::Point::Point | (const Position&) | project | modified: Pimax layout, float pos, atomic id | point.obj (belongs with c06) |
| 0x18009A070 | lib:std::unordered_map tidy thunk |  | lib:std | - | point.obj COMDAT |
| 0x18009A080 | lib:std::vector<KeypointIdentifier>::_Tidy |  | lib:std | - | point.obj COMDAT |
| 0x18009A110 | lib:std::shared_ptr<T>::~shared_ptr |  | lib:std | - | point.obj COMDAT |
| 0x18009A160 | lib:std::set<int>/map tidy (+0x60) |  | lib:std | - | point.obj COMDAT |
| 0x18009A1D0 | lib:std::vector<KeypointIdentifier>::_Change_array |  | lib:std | - | point.obj COMDAT |
| 0x18009A290 | lib:std::_Destroy_range<KeypointIdentifier> thunk |  | lib:std | - | point.obj COMDAT |
| 0x18009A2B0 | lib:std::_Hash::_Forced_rehash |  | lib:std | - | point.obj COMDAT |
| 0x18009A480 | lib:std::vector<KeypointIdentifier>::_Reallocate |  | lib:std | - | point.obj COMDAT |
| 0x18009A510 | lib:std::vector<void*>::_Xlen ("vector too long") |  | lib:std | - | point.obj COMDAT |

### 3b. marginalization_error.obj functions located in the c04 range (reconstructed in my draft as well)

| address | name | status |
|---|---|---|
| 0x1800732A0 | `MarginalizationError::splitSymmetricMatrix<MatrixXd,MatrixXd,MatrixXd,MatrixXd>` | identical (no DEBUG_CHECK) |
| 0x1800740F0 | `MarginalizationError::splitVector<VectorXd,VectorXd,VectorXd>` | identical (no DEBUG_CHECK) |
| 0x18006CA60 | `MarginalizationError::pseudoInverseSymmSqrt<Matrix3d>` | identical |
| 0x18006BF60 | `MarginalizationError::pseudoInverseSymmSqrt<MatrixXd>` | identical |
| 0x1800773E0 | `MarginalizationError::MarginalizationError(Map&)` | modified: reserve(100/500/200) |
| 0x180077740 | `MarginalizationError::ParameterBlockInfo::ParameterBlockInfo(id, ptr, ordering, is_landmark)` | identical |
| 0x180077C70 | `MarginalizationError::~MarginalizationError` (body) | identical (members) |
| 0x1800784A8 / 0x1800784C0 | dtor thunk (ErrorInterface) / scalar deleting dtor | — |
| 0x180078510 | `MarginalizationError::Evaluate` | modified (flag dispatch) |
| 0x180078540 | `MarginalizationError::EvaluateWithMinimalJacobians` | modified (.at, skip min_dim==0, shared Jmin block) |
| 0x180079000 | `MarginalizationError::EvaluateLocal` (name TODO) | new |
| 0x18003AE00 | folded `Evaluate` of ImuError / PoseError / SpeedAndBiasError | modified (flag dispatch) |

## 4. Types

### 4.1 MarginalizationError — sizeof 0x240 (malloc(0x240) in estimator 0x180027AE0), EIGEN_MAKE_ALIGNED_OPERATOR_NEW
vtables: 0x1803B05C0 (CostFunction: 0 dtor 0x1800784C0, 1 Evaluate 0x180078510, 2 EvaluateLocal 0x180079000),
0x1803B05E0 (ErrorInterface: 0 thunk 0x1800784A8, 1 residualDim 0x180087010, 2 parameterBlocks 0x180014980,
3 parameterBlockDim 0x180014950, 4 EvaluateWithMinimalJacobians 0x180078540, 5 typeInfo 0x180088D00).

| offset | type | name | evidence |
|---|---|---|---|
| 0x000 | vptr | ceres::CostFunction | ctor (sure) |
| 0x008 | std::vector<int32_t> | parameter_block_sizes_ | add/marginalize (sure) |
| 0x020 | int | num_residuals_ | add (+=), marginalize (-=), update (=H_.cols()) (sure) |
| 0x028 | vptr | ErrorInterface | ctor (sure) |
| 0x030 | bool | ErrorInterface::use_minimal_jacobians_ (name TODO) | ctor =0, Evaluate (sure) |
| 0x038 | Map* | map_ptr_ | (sure) |
| 0x040 | ceres::ResidualBlockId | residual_block_id_ | ctor =0 only |
| 0x048 | Eigen::MatrixXd | unknown (pimax-new) | ctor/dtor only |
| 0x060 | Eigen::VectorXd | unknown | ctor/dtor only |
| 0x070 | Eigen::MatrixXd | unknown | ctor/dtor only |
| 0x088 | Eigen::VectorXd | unknown | ctor/dtor only |
| 0x098, 0x0A0 | 2 × 8-byte POD | unknown, not initialised | gap |
| 0x0A8 | MatrixXd | H_ | (sure) |
| 0x0C0 | VectorXd | b0_ | (sure) |
| 0x0D0 | VectorXd | e0_ | (sure) |
| 0x0E0 | MatrixXd | J_ | (sure) |
| 0x0F8 | MatrixXd | U_ | dtor only |
| 0x110/0x120/0x130/0x140 | VectorXd | S_, S_sqrt_, S_pinv_, S_pinv_sqrt_ | update (sure) |
| 0x150/0x160 | VectorXd | p_, p_inv_ | dtor only |
| 0x170 | volatile bool | error_computation_valid_ | (sure) |
| 0x178 | std::vector<ParameterBlockInfo> (80 B) | parameter_block_infos_ | (sure) |
| 0x190 | std::unordered_map<uint64_t,size_t> (64 B) | parameter_block_id_to_parameter_block_info_idx_ (upstream std::map) | FNV-1a, mask 7/maxidx 8 init (sure) |
| 0x1D0 | std::vector<uint64_t> | new_parameter_block_ids_ (name TODO) | push_back in add, clear in linearize (sure) |
| 0x1E8 | std::map<ResidualBlockId, Map::ResidualBlockSpec> (node 0x48) | residual_block_specs_ (name TODO) | (sure) |
| 0x1F8 | std::vector<ResidualBlockId> | residual_block_ids_ (name TODO) | (sure) |
| 0x210 | std::map<ResidualBlockId, std::vector<uint64_t>> (node 0x40) | residual_block_parameter_ids_ (name TODO) | (sure) |
| 0x220 | uint32_t | num_new_dense_blocks_ (name TODO) | (sure) |
| 0x224 | uint32_t | num_new_landmark_blocks_ (write-only) | (sure) |
| 0x228 | uint32_t | num_new_dimensions_ | (sure) |
| 0x230 | size_t | dense_indices_ | (sure) |
| 0x238 | size_t | unknown, =0 in ctor | |

ParameterBlockInfo (80 B): id +0, shared_ptr<ParameterBlock> +8, ordering_idx +0x18, dimension +0x20,
minimal_dimension +0x28, local_dimension +0x30, shared_ptr<double> linearization_point +0x38,
is_landmark +0x48 — identical to upstream.

### 4.2 ErrorInterface (shared)
vptr +0, `bool use_minimal_jacobians_` +8 (pimax-new, name TODO), 16 bytes. Enum ErrorType values confirmed:
kSpeedAndBiasError 2, kMarginalizationError 3, kPoseError 4.

### 4.3 ParameterBlock (shared) — upstream layout, confirmed
vptr +0, id_ +8, fixed_ +0x10, local_parameterization_ptr_ +0x18, data from +0x20. Slots: 0 dtor, 1/2
parameters(), 3 dimension, 4 minimalDimension, 5 plus, 6 plusJacobian, 7 minus, 8 liftJacobian,
9 setLocalParameterizationPtr (0x18002C7A0), 10 localParameterizationPtr (0x18002A6C0), 11 typeInfo,
12 setEstimate (derived), 13 estimate() (SpeedAndBias only). id() is read inline as `*(ptr+8)`.

### 4.4 PoseError — sizeof 0x1A0 (make_shared 0x1B0 in 0x180022050 / 0x18002C7B0)
| offset | type | name |
|---|---|---|
| 0x00 | SizedCostFunction<6,7> | base (vtable 0x1803B1318) |
| 0x28 | ErrorInterface | (vtable 0x1803B1330) |
| 0x40 | Transformation (q xyzw, p) | measurement_ |
| 0x80 | Matrix<double,6,6> | square_root_information_ |
Upstream information_ and covariance_ are gone.

### 4.5 PoseLocalParameterization — 24 bytes (Map+0x520)
vptr LocalParameterization +0, vptr LocalParamizationAdditionalInterfaces +8, bool +0x10 (pimax-new, set by
Map(bool), see c02). Vtables in the header comment of the draft.

### 4.6 PoseParameterBlock — sizeof 0x58 (make_shared 0x68); SpeedAndBiasParameterBlock — sizeof 0x68 (make_shared 0x78)
Upstream-identical. SpeedAndBiasParameterBlock vtable has slot 13 = estimate() (folded with parameters()).

### 4.7 SpeedAndBiasError — sizeof 0x590 (make_shared 0x5A0 in 0x180026100)
SizedCostFunction<9,9> +0, ErrorInterface +0x28, measurement_ (Matrix<double,9,1>) +0x38,
information_ +0x80, square_root_information_ +0x308 (both 9x9). Upstream layout.

### 4.8 OutlierRejection (fields unknown — `this` is never read by the only function)

### 4.9 Frame — sizeof 0x340 (make_shared<Frame> 0x350 in 0x1800FB650), vtable 0x1803B1588
| offset | type | name | evidence |
|---|---|---|---|
| 0x000 | vptr | | |
| 0x008 | 8 B | unknown, never written by the ctors | |
| 0x010 | int | id_ = frame_counter_++ (dword_18047DB38) | ctor1 (sure) |
| 0x014 | int | cam_index_ (ctor arg: camera index i) TODO name | ctor1 |
| 0x018 | int | per-camera int ctor arg TODO | ctor1 |
| 0x01C | int | per-camera int ctor arg TODO | ctor1 |
| 0x020 | int (BundleId) | bundle_id_ | FrameBundle lambda, KeypointIdentifier (sure) |
| 0x024 | int | nframe_index_ = -1 | ctors |
| 0x028 | CameraPtr | cam_ | (sure) |
| 0x040 | Transformation | T_f_w_ | (sure) |
| 0x080 | std::vector<cv::Mat> | img_pyr_ | (sure) |
| 0x098 | std::vector<pair<int, Vector3f>, aligned_allocator> | key_pts_ (5 elems, 16 B) | (sure) |
| 0x0B0 | bool | is_keyframe_ | setKeyframe (sure) |
| 0x0B1 | bool | unknown (=false) | ctor |
| 0x0B4 | int | last_published_ts_ (uninit) | gap |
| 0x0C0 | 32 B aligned Eigen object (no init) | R_imu_world_? TODO type | ctor alignment assert |
| 0x0E0 | double | mean_intensity_ (TODO name) | FrameBundle lambda, getSceneDepth (sure) |
| 0x0E8/0x0E9 | bool | is_too_dark_ (<25) / is_too_bright_ (>220) (TODO names) | lambda (sure) |
| 0x0EA | bool | unknown (=false) | ctor |
| 0x0F0 | int64_t | timestamp_ | ctors (sure) |
| 0x100 | Transformation | T_body_cam_ | ctor (identity) / caller copies |
| 0x140 | Transformation | T_cam_body_ | ctor / caller 0x1800FB650 |
| 0x180 | 2 bytes | ctor arg from FrameProcessor+0x110 TODO | ctor1 |
| 0x188 | {std::vector<int>; 8 B} | copy of FrameProcessor+0xC08 TODO | ctor1 |
| 0x1A8 | float | min_depth_ = 0.5f | getSceneDepth (sure) |
| 0x1AC | float | median_depth_ = 2.0f | getSceneDepth (sure) |
| 0x1B0 | std::vector<double> | {1.5, 3.0, 6.0, 12.0} TODO name | ctors |
| 0x1C8 | cv::Mat | original_color_image_? TODO | ctor/dtor |
| 0x228 | size_t | num_features_ | (sure) |
| 0x230 | Matrix<float,2,Dynamic> | px_vec_ | (sure) |
| 0x240 | Matrix<float,3,Dynamic> | f_vec_ | (sure) |
| 0x250 | Matrix<float,3,Dynamic> | f_vec_raw_ (pimax-new, un-normalised bearings) TODO name | resize, FeatureWrapper |
| 0x260 | VectorXf | score_vec_ (init -1) | (sure) |
| 0x270 | VectorXi | level_vec_ | (sure) |
| 0x280 | Matrix<float,2,Dynamic> | grad_vec_ | (sure) |
| 0x290 | std::vector<FeatureType> | type_vec_ (init kCorner=7) | (sure) |
| 0x2A8 | std::vector<PointPtr> | landmark_vec_ | (sure) |
| 0x2C0 | VectorXi | track_id_vec_ (init -1) | (sure) |
| 0x2D0 | std::vector<SeedRef> (24 B) | seed_ref_vec_ | (sure) |
| 0x2E8 | Matrix<float,4,Dynamic> | invmu_sigma2_a_b_vec_ | (sure) |
| 0x300 | Transformation | accumulated_w_T_correction_ | ctor |
No in_ba_graph_vec_, seed_mu_range_, is_stable_, imu_vel_w_/biases.

SeedRef (24 B): FramePtr keyframe +0, int seed_id = -1 +0x10, int (pimax-new, =0) +0x14.
FeatureWrapper (144 B): see `common/feature_wrapper.h` (extra `f_raw` Ref at +0x38).
Point (partial, from ctor 0x180099F80): id_ +0 (atomic counter dword_18047DB60), Vector3f pos_ +4,
unordered_map<int, KeypointIdentifier> +0x10, 8 B +0x50, int +0x58, std::set<int> +0x60, 8 B +0x70,
int +0x78, int (-1) +0x7C, bool in_ba_graph_ +0x80. KeypointIdentifier (32 B): weak_ptr<Frame> +0,
frame_id +0x10, bundle_id +0x14 (pimax-new), keypoint_index_ +0x18.

### 4.10 FrameBundle — sizeof 0x100
frames_ +0; std::deque<?, aligned_allocator> +0x18 (40 B, proxy malloc'd); bool +0x40; Transformation
(identity) +0x50; three Vector3d (zero) +0x90/+0xA8/+0xC0; 8 B +0xD8; int +0xE0; 2 bools +0xE4; 8 B +0xE8;
8 B +0xF0; bool +0xF8; bundle_id_ +0xFC (counter dword_18047DB48). Names TODO.

## 5. External interfaces

| address | meaning |
|---|---|
| 0x18001D310 | `const Map::ParameterBlockCollection* Map::parametersPtr(ResidualBlockId) const` (c02 name) |
| Map+0x448 | `residual_block_id_to_residual_block_spec_map_` (unordered_map, read inline via `residualBlockIdToResidualBlockSpecMap().find(id)->second`) |
| 0x18001E4C0 | `bool Map::removeResidualBlock(ResidualBlockId)` |
| 0x18001EA00 | `Map::ResidualBlockCollection Map::residuals(uint64_t) const` |
| 0x18001D940 | `void Map::printParameterBlockInfo(uint64_t) const` |
| 0x18001E1D0 | `bool Map::removeParameterBlock(uint64_t)` |
| 0x180013040 | minkindr `Transformation::inverse()` (c01) |
| 0x1800089C0 | minkindr `RotationQuaternion(const Eigen::Quaterniond&)` with EPS CHECKs (c01) |
| 0x180009B80 | minkindr `Transformation::operator*` (normalises) (c01) |
| 0x180014A20 | minkindr quaternion rotate (c01) |
| 0x180029DF0 | minkindr `RotationQuaternion::getRotationMatrix()` (c02/c03) |
| 0x1800426F0 | `Eigen::Quaterniond::normalized()` (c04) |
| 0x1800453F0 / 0x1800455A0 | `quaternionOplusMatrix` / `quaternionPlusMatrix` (c04, comma-initialiser versions) |
| 0x18003A1F0 | Eigen CommaInitializer<Matrix4d>::operator, (c04) |
| 0x18000F6A0 | LOGW (logger 0x18046A000) |
| 0x180354E80 / 0x18035AC70 / 0x180355280 | glog LogMessageFatal ctor / stream() / dtor (CHECK) |
| 0x1801B77B0 / 0x1801B77D0 | ceres::CostFunction ctor / dtor |
| camera vtable | slot1 backProject3(Ref<const Matrix2Xd>, Matrix3Xd*, vector<bool>*), slot3 project3(Ref<const Vector3d>, Vector2d*, Matrix<2,3>*)->ProjectionResult, slot5 getAngleError, slot6 getIntrinsicParameters()->VectorXd, slot8 errorMultiplier(); Camera+0x38 = mask |
| OpenCV | cv::pyrDown, cv::blur, cv::mean, cv::noArray, cv::parallel_for_ |
| globals | dword_18047DB38 Frame::frame_counter_, dword_18047DB48 FrameBundle bundle counter (function static), dword_18047DB60 PointIdProvider::last_id_ (atomic) |

Callers into this chunk: estimator 0x180027AE0 (MarginalizationError ctor/add/marginalize/update/
getParameterBlockPtrs; logs "marginalizeOut previous H_ size = " etc. live there), 0x180026100 /
0x180027470 / 0x180022050 / 0x18002C7B0 (error/param-block ctors), 0x180013680 (removeOutliers),
frontend 0x1800FB650 (Frame / FrameBundle ctors), 0x180147110 (computeNormalizedBearingVectors,
resizeFeatureStorage), 0x1800B2F80 / 0x1800B4180 (setKeyframe, getSceneDepth).

## 6. Constants

* marginalization: preconditioner threshold 1.0e-9, fallback 1.0e-3; pseudo-inverse epsilon
  2.220446049250313e-16 (DBL_EPSILON); dense V1 = 0.5*(V+V^T); reserves 100/500/200.
* outlier counting: 2.5 px (level-scaled), depth (0, 10.0f].
* PoseLocalParameterization: NearlyOne 1-1e-10, exp small-angle threshold 1.0e-12 (inlined minkindr),
  1/48 = 0.02083333333333333, 8/3, 2/3, pi, 2pi.
* Frame: key_pts_ 5, min_depth_ 0.5f, median_depth_ 2.0f, vector {1.5, 3.0, 6.0, 12.0},
  type default kCorner (7), score default -1.0f, track id -1; isVisible bounds 16 < u < 624, 16 < v < 464;
  saturation: pixel > 220, ratio 4/5, window 5x5; FrameBundle blur 3x3, dark < 25.0, bright > 220.0;
  getSceneDepth: mean_intensity_ < 15.0 -> false, FLT_MAX initial min.

## 7. Quirks worth preserving

* MarginalizationError: the landmark branch of addResidualBlock uses `parameter_block_infos_.back()`
  even when the vector is empty (UB, upstream). `linearizeResidualBlocks()` reads info_i with
  operator[] but info_j / jacobians with `.at()`. `num_new_landmark_blocks_` is never read.
  `marginalizeOut` returns false silently for unconnected ids **after** having linearised.
* PoseError ignores sqrt_info entries other than (0,0) and (5,5).
* FrameBundle ctor blurs `img_pyr_[0]` **in place** (shallow cv::Mat copy).
* Frame::isVisible uses hard-coded 640x480 bounds regardless of the camera.
* setKeyPoints keeps the more negative product in quadrants 2 and 4.
* FrameBundle::numTrackedLandmarks predicate `(t & 0xF9) == 0 && t != 2` (types 0, 4, 6).

## 8. Open questions / TODO(verify)

* Names of all pimax-new members/functions (linearizeResidualBlocks, EvaluateLocal,
  use_minimal_jacobians_, booking containers, Frame extra fields, FrameBundle fields, w2c/f2c,
  isSaturatedPatch, numTrackIds, depthInFrame) and the four unused Eigen members at
  MarginalizationError+0x48..0xA8.
* The Pimax minkindr copy (thirdparty/minkindr): exp() threshold 1e-12, unconditional normalisation in
  operator*, EPS CHECKs only in the Eigen-quaternion ctor — affects plus()/T_WS construction codegen.
* FeatureType enum values (numTrackedLandmarks predicate does not match upstream helpers).
* Element type of the FrameBundle deque; types/meaning of Frame +0x008, +0x0C0, +0x180, +0x188, +0x1B0, +0x1C8.
* OutlierRejection file/class name.

## 9. Line counts

See reply (wc -l of draft files).
