# c04_imu_error — [0x1800427F0, 0x180079D90)  (219 functions, 224 890 bytes)

Drafts: `draft/c04_imu_error/`
- `ceres_backend/imu_error.h`, `ceres_backend/imu_error.cpp` — ImuError layout, `propagation`, `redoPreintegration`
- helper functions come from c03: `draft/c03_estimator/common/so3_gamma.h`, `common/sophus/so3ex_base.h` (names below follow c03)
- `vio_common/matrix_operations.h` — `quaternionPlusMatrix` / `quaternionOplusMatrix`
- `ceres_backend/local_parameterization_additional_interfaces.cpp` — `verify()`
- `ceres_backend/marginalization_error.h` (partial), `ceres_backend/marginalization_error.cpp` (c04 part:
  ctor, ParameterBlockInfo ctor, Evaluate, EvaluateWithMinimalJacobians, EvaluateLocal (Pimax),
  splitSymmetricMatrix, splitVector, pseudoInverseSymmSqrt)

## Object-file boundaries inside the chunk

| range | object | evidence |
|---|---|---|
| 0x1800427F0 – 0x180050EE3 | tail of `ceres_backend/imu_error.cpp` (object starts in c03; ImuError vtables 0x1803AF4B0/4C8 point to 0x18003AD54..0x18003AE40) | `__FILE__` imu_error.cpp in 0x1800427F0; typeInfo 0x180050EE0 is ImuError vtable slot 5; all templates up to 0x180050610 are reached from ImuError code |
| 0x180050EF0 – 0x180053DA8 | `ceres_backend/local_parameterization_additional_interfaces.cpp` (alphabetical project order: imu_error < local_param... < marginalization_error) | `verify` 0x180052D90 is slot 3 of every `LocalParamizationAdditionalInterfaces` sub-vtable; 0x180050EF0..0x1800528F0 are only reachable from it (plus ICF sharers) |
| 0x180053DB0 – (beyond 0x180079D90) | head of `ceres_backend/marginalization_error.cpp` | everything from here is reached from MarginalizationError code (0x18007B280 / 0x180080320 / 0x180088D10 / 0x180079D90 in c05) or is a MarginalizationError vtable slot; ctor 0x1800773E0 |

MSVC emitted each object's template instantiations *before* the object's own non-inline functions
(e.g. verify() comes after its Eigen kernels; MarginalizationError's virtuals 0x1800784A8..0x180079000
come after ~150 KB of Eigen/STL code).  Some instantiations here are only called from far-away objects
(0x180162E90, 0x180165AA0, ceres 0x1801B…, 0x18014C520) — identical-COMDAT-folding / first-use placement;
they are still library code.

## Function table

Kinds: `project` = to be reconstructed (drafted); `lib:*` = template/library instantiation, not reconstructed.
| address | size | proposed qualified name | signature | kind | upstream status | summary |
|---|---|---|---|---|---|---|
| 0x1800427F0 | 11257 | ImuError::propagation | int (const ImuMeasurements&, const ImuParameters&, Transformation& T_WS, SpeedAndBias&, const double& t_start, const double& t_end, const Eigen::Vector3d& g_W, covariance_t* cov=nullptr, jacobian_t* =nullptr) | project | modified: non-static, gravity arg, closed-form SO3/Gamma integration, no covariance/jacobian output, no delay_imu_cam, LOGE instead of assert | IMU propagation of T_WS/v (timestamps from arg deque, samples from this->imu_measurements_) |
| 0x1800453F0 | 424 | quaternionOplusMatrix | Eigen::Matrix4d (const Eigen::Quaterniond&) | project | modified: comma-initializer form | inline vio_common helper (out-of-line copy; also used by PoseLocalParameterization) |
| 0x1800455A0 | 424 | quaternionPlusMatrix | Eigen::Matrix4d (const Eigen::Quaterniond&) | project | modified: comma-initializer form | inline vio_common helper |
| 0x180045750 | 36037 | ImuError::redoPreintegration | int (const SpeedAndBias&, const Eigen::Vector3d* g_S0) const | project | modified: heavily (Gamma integration, 15x15 F and J, LLT information, no lock, T_WS arg dropped) | re-preintegrate t0_..t1_, P_delta_, information_, sqrt info, bias Jacobians |
| 0x18004E420 | 6 | ImuError::residualDim | size_t () const | project | identical | return 15 |
| 0x18004E430 | 40 | Eigen::DenseBase<Matrix<double,15,2>>::resize | void (Index,Index) | lib:Eigen::DenseBase::resize (fixed 15x2, assert only) | - | "rows == this->rows()..." assert |
| 0x18004E460 | 56 | Eigen::PlainObjectBase<Matrix15d>::operator=(Transpose<TriangularView<const Matrix15d,Lower>>) |  | lib:Eigen call_assignment (TriangularView transpose -> Matrix15d) | - | square_root_information_ = llt.matrixL().transpose() |
| 0x18004E4A0 | 1179 | Eigen::internal::general_matrix_matrix_product<long long,double,ColMajor,false,double,ColMajor,false,ColMajor,1>::run |  | lib:Eigen GEMM driver (Col*Col->Col) | - | blocked GEMM (pack 0x18000B840/0x18003AA40, kernel 0x180009E50) |
| 0x18004E940 | 3890 | Eigen::internal::general_matrix_vector_product<long long,double,const_blas_data_mapper<double,long long,RowMajor>,RowMajor,false,double,const_blas_data_mapper<double,long long,ColMajor>,false,0>::run |  | lib:Eigen GEMV kernel (row-major lhs) | - | n8 = stride*8>32000 ? 0 : rows-7 heuristic (Eigen >= 3.3.5) |
| 0x18004F880 | 279 | Eigen::internal::quat_product<Architecture::SSE,Quaterniond,Quaterniond,double>::run |  | lib:Eigen quaternion product (SSE) | - | q1*q2 (also used by ImuError ctor callers, PlatMap code) |
| 0x18004F9A0 | 507 | Eigen::internal::triangular_assignment_loop<...,Lower\|transpose,15x15>::run |  | lib:Eigen triangular assignment (15x15, zero opposite part) | - | used by 0x18004E460 |
| 0x18004FBA0 | 2663 | Eigen::internal::triangular_solve_matrix<double,long long,OnTheLeft,Lower,false,ColMajor,ColMajor,1>::run |  | lib:Eigen TRSM | - | LLT::solveInPlace (L part); also TriangularView::solveInPlace 0x180073010 |
| 0x180050610 | 2250 | Eigen::internal::triangular_solve_matrix<double,long long,OnTheLeft,Upper,false,RowMajor,ColMajor,1>::run |  | lib:Eigen TRSM (L^T) | - | LLT::solveInPlace (L^T part) |
| 0x180050EE0 | 3 | ImuError::typeInfo | ErrorType () const | project | identical | return ErrorType::kIMUError (5) |
| 0x180050EF0 | 1103 | Eigen::internal::generic_product_impl<Matrix<double,-1,-1,RowMajor>,Matrix<double,-1,-1,RowMajor>,DenseShape,DenseShape,GemmProduct>::evalTo<MatrixXd> |  | lib:Eigen product dispatch (lazy if r+c+d<20, else GEMM) | - | J_lift * J_plus in verify() |
| 0x180051340 | 652 | Eigen::internal::redux_impl<scalar_sum_op,(VectorXd-VectorXd).squaredNorm>::run |  | lib:Eigen redux squaredNorm(a-b) | - | (delta_x2-delta_x).norm() |
| 0x1800515D0 | 652 | Eigen::internal::redux_impl<scalar_sum_op,(MatrixRM-MatrixRM).squaredNorm>::run |  | lib:Eigen redux squaredNorm | - | (J_plus-J_plus_num_diff).norm() |
| 0x180051860 | 652 | Eigen::internal::redux_impl<scalar_sum_op,(Product-MatrixXd).squaredNorm>::run |  | lib:Eigen redux squaredNorm | - | ((J_lift*J_plus)-identity).norm() |
| 0x180051AF0 | 630 | Eigen::internal::gemv_dense_selector<OnTheRight,RowMajor,true>::run |  | lib:Eigen GEMV dispatch (row-major) | - | copies rhs to aligned temp if needed, calls 0x18004E940 |
| 0x180051D70 | 997 | Eigen::internal::gemv_dense_selector<OnTheRight,ColMajor,true>::run |  | lib:Eigen GEMV dispatch (col-major) | - | calls 0x18001F0F0 |
| 0x180052160 | 146 | Eigen::internal::unaligned_dense_assignment_loop (dst *= scalar, dynamic vector) |  | lib:Eigen dense assignment kernel | - | tail loop of VectorXd *= s (13 callers incl. ceres) |
| 0x180052200 | 1633 | Eigen::internal::generic_product_impl<MatrixRM,MatrixRM,DenseShape,DenseShape,GemmProduct>::scaleAndAddTo<MatrixXd> |  | lib:Eigen product scaleAndAddTo | - | GEMV special cases then GEMM 0x1800528F0 |
| 0x180052870 | 115 | Eigen::PlainObjectBase<VectorXd>::resize(Index) |  | lib:Eigen DenseStorage resize (dynamic double vector) | - | 52 callers across the DLL (ICF) |
| 0x1800528F0 | 1176 | Eigen::internal::general_matrix_matrix_product<long long,double,RowMajor,false,double,RowMajor,false,ColMajor,1>::run |  | lib:Eigen GEMM driver (Row*Row->Col) | - | also used by ceres 0x1801B84C0 |
| 0x180052D90 | 4120 | LocalParamizationAdditionalInterfaces::verify | bool (const double* x_raw, double purturbation_magnitude=1e-6) const | project | identical | numeric check of Plus/Minus/ComputeJacobian/ComputeLiftJacobian (vtable slot 3 of all Pimax local parameterizations) |
| 0x180053DB0 | 3332 | Eigen::MatrixBase<Block<MatrixXd>>::applyHouseholderOnTheLeft<VectorBlock<...>> | void (const Essential&, const double& tau, double* workspace) | lib:Eigen Householder | - | used by HouseholderSequence::applyThisOnTheLeft / evalTo |
| 0x180054AC0 | 3597 | Eigen::MatrixBase<Block<MatrixXd>>::applyHouseholderOnTheLeft<...> (ColPivHouseholderQR variant) |  | lib:Eigen Householder | - | used by ColPivHouseholderQR::computeInPlace |
| 0x1800558D0 | 1333 | Eigen::HouseholderSequence<MatrixXd,VectorXd,1>::applyThisOnTheLeft<MatrixXd,VectorXd> | (Dest&, Workspace&, bool inputIsIdentity) | lib:Eigen HouseholderSequence | - | HouseholderSequence.h:0xE7; block path via 0x180055E10 |
| 0x180055E10 | 2395 | Eigen::internal::apply_block_householder_on_the_left<...> |  | lib:Eigen BlockHouseholder | - | T = make_block_householder_triangular_factor, two GEMM/TRMM products |
| 0x180056770 | 54 | Eigen::internal::conditional_aligned_new_auto<int,true> | int* (size_t n) | lib:Eigen aligned alloc (4-byte elems) | - |  |
| 0x1800567B0 | 54 | Eigen::internal::conditional_aligned_new_auto<long long,true> | long long* (size_t n) | lib:Eigen aligned alloc (8-byte elems) | - |  |
| 0x1800567F0 | 13 | Eigen::internal::conditional_aligned_new_auto<char/bool,true> | void* (size_t n) | lib:Eigen aligned alloc (1-byte elems) | - |  |
| 0x180056800 | 988 | Eigen::internal::computeProductBlockingSizes<double,double,1,long long> | void (Index& k, Index& m, Index& n, Index num_threads) | lib:Eigen blocking heuristic | - | thread-safe static cache sizes via cpuid (0x180018C90); 35 callers (ICF) |
| 0x180056BE0 | 213 | Eigen::internal::redux_impl<scalar_max_op,cwiseAbs(Block)>::run |  | lib:Eigen redux max \|x\| | - | called by ceres 0x18026C4D0 only (ICF-shared) |
| 0x180056CC0 | 1212 | Eigen::MatrixBase<VectorBlock<...>>::makeHouseholder<VectorBlock<...>> | void (Essential&, double& tau, double& beta) const | lib:Eigen Householder | - | sqrt, squaredNorm tail; used by ColPivQR and tridiagonalization |
| 0x180057180 | 2828 | Eigen::internal::make_block_householder_triangular_factor<MatrixXd,Block,VectorBlock> |  | lib:Eigen BlockHouseholder | - | BlockHouseholder.h:0x36 |
| 0x180057C90 | 835 | Eigen::internal::generic_product_impl<...,OuterProduct>::subTo (outer_product_selector_run) |  | lib:Eigen outer product (dst -= u*v^T) | - | householder update |
| 0x180057FE0 | 835 | Eigen::internal::generic_product_impl<...,OuterProduct>::subTo (ColPivQR variant) |  | lib:Eigen outer product | - |  |
| 0x180058330 | 321 | Eigen::internal::redux_impl<scalar_max_op,...>::run (maxCoeff, small vector) | double (const Xpr&) | lib:Eigen redux maxCoeff | - | 16 callers (eigenvalue max) |
| 0x180058480 | 291 | Eigen::internal::triangular_product_impl<Mode,true,TriangularView<...>,false,MatrixXd,false>::run |  | lib:Eigen TRMM wrapper | - | blocking 0x180056800 + TRMM 0x18005EFB0 |
| 0x1800585B0 | 584 | Eigen::internal::redux_impl<scalar_sum_op,conj_prod(Block,Block)>::run (dot) |  | lib:Eigen dot product | - |  |
| 0x180058800 | 600 | Eigen::internal::redux_impl<scalar_sum_op,conj_prod(...)>::run (dot, ColPivQR) |  | lib:Eigen dot product | - |  |
| 0x180058A60 | 600 | Eigen::internal::redux_impl<scalar_sum_op,conj_prod(...)>::run (dot, TRMV) |  | lib:Eigen dot product | - |  |
| 0x180058CC0 | 451 | Eigen::internal::redux_impl<scalar_sum_op,abs2(VectorBlock)>::run (squaredNorm) |  | lib:Eigen squaredNorm | - |  |
| 0x180058E90 | 492 | Eigen::internal::redux_impl<scalar_max_op,...>::run (maxCoeff, vectorized) |  | lib:Eigen redux max | - |  |
| 0x180059080 | 283 | Eigen::internal::triangular_product_impl<...>::run (TRMM wrapper, variant 2) |  | lib:Eigen TRMM wrapper | - | -> 0x18005DC80 |
| 0x1800591A0 | 283 | Eigen::internal::triangular_product_impl<...>::run (TRMM wrapper, variant 3) |  | lib:Eigen TRMM wrapper | - | -> 0x18005E600 |
| 0x1800592C0 | 354 | Eigen::internal::triangular_product_impl<...>::run (TRMM wrapper, variant 4) |  | lib:Eigen TRMM wrapper | - | -> 0x18005F910 |
| 0x180059430 | 381 | Eigen::internal::trmv_selector<Mode,RowMajor>::run |  | lib:Eigen TRMV dispatch | - | -> 0x180060240 |
| 0x1800595B0 | 448 | Eigen::internal::gemv_dense_selector<OnTheRight,RowMajor,true>::run (Householder tmp) |  | lib:Eigen GEMV dispatch | - | -> 0x18004E940 |
| 0x180059770 | 454 | Eigen::internal::gemv_dense_selector<OnTheRight,RowMajor,true>::run (ColPivQR) |  | lib:Eigen GEMV dispatch | - | -> 0x18004E940 |
| 0x180059940 | 274 | Eigen::internal::dense_assignment_loop (dst = src / scalar) |  | lib:Eigen dense assignment kernel | - | essential = tail / (c0 - beta) |
| 0x180059A60 | 131 | Eigen::internal::dense_assignment_loop (dst = Constant) |  | lib:Eigen setConstant/setZero kernel | - | 92 callers |
| 0x180059AF0 | 229 | Eigen::internal::dense_assignment_loop (dst -= src * scalar) |  | lib:Eigen dense assignment kernel | - | 34 callers |
| 0x180059BE0 | 205 | Eigen::internal::dense_assignment_loop (dst += src * scalar) |  | lib:Eigen dense assignment kernel | - |  |
| 0x180059CB0 | 178 | Eigen::internal::dense_assignment_loop<swap_assign_op> |  | lib:Eigen swap kernel | - | column/row swap |
| 0x180059D70 | 393 | Eigen::internal::dense_assignment_loop (dst += src, vectorized) |  | lib:Eigen dense assignment kernel | - | 21 callers |
| 0x180059F00 | 274 | Eigen::internal::dense_assignment_loop (dst = src * scalar) |  | lib:Eigen dense assignment kernel | - |  |
| 0x18005A020 | 185 | Eigen::internal::dense_assignment_loop (VectorXd -= VectorXd) |  | lib:Eigen dense assignment kernel | - | marginalizeOut |
| 0x18005A0E0 | 237 | Eigen::internal::dense_assignment_loop (dst = src, unaligned) |  | lib:Eigen dense copy kernel | - |  |
| 0x18005A1D0 | 408 | Eigen::internal::triangular_product_impl<Mode,true,Lhs,false,Rhs,true>::run (TRMV) |  | lib:Eigen TRMV wrapper | - | TriangularMatrixVector.h:0xBD |
| 0x18005A370 | 301 | Eigen::internal::triangular_solver_selector<...,OnTheLeft,...,Dynamic>::run |  | lib:Eigen TRSM wrapper | - | SolveTriangular.h:0xAA; -> 0x180060790 |
| 0x18005A4A0 | 135 | Eigen::internal::gemm_blocking_space<ColMajor,double,double,Dynamic,Dynamic,Dynamic,1,false>::gemm_blocking_space |  | lib:Eigen blocking ctor | - |  |
| 0x18005A530 | 486 | Eigen::internal::local_nested_eval_wrapper<(scalar*Block),Dynamic,true>::ctor |  | lib:Eigen aligned temp eval | - |  |
| 0x18005A720 | 486 | Eigen::internal::local_nested_eval_wrapper<...>::ctor (variant) |  | lib:Eigen aligned temp eval | - |  |
| 0x18005A910 | 485 | Eigen::internal::local_nested_eval_wrapper<...>::ctor (variant) |  | lib:Eigen aligned temp eval | - |  |
| 0x18005AB00 | 498 | Eigen::internal::local_nested_eval_wrapper<...>::ctor (variant) |  | lib:Eigen aligned temp eval | - |  |
| 0x18005AD00 | 84 | Eigen::ColPivHouseholderQR<MatrixXd>::~ColPivHouseholderQR |  | lib:Eigen dtor (7 buffers) | - |  |
| 0x18005AD60 | 3798 | Eigen::ColPivHouseholderQR<MatrixXd>::computeInPlace |  | lib:Eigen ColPivHouseholderQR | - | ColPivHouseholderQR.h:0x1E7; caller 0x180162E90 only (ICF/first-use) |
| 0x18005BC40 | 115 | Eigen::PlainObjectBase<Matrix<int,-1,1>>::resize |  | lib:Eigen resize (int vector, permutation indices) | - | 30 callers |
| 0x18005BCC0 | 115 | Eigen::PlainObjectBase<Matrix<long long,-1,1>>::resize |  | lib:Eigen resize (Index vector, transpositions) | - |  |
| 0x18005BD40 | 115 | Eigen::PlainObjectBase<Matrix<1-byte,-1,1>>::resize |  | lib:Eigen resize (byte vector) | - |  |
| 0x18005BDC0 | 4076 | Eigen::internal::general_matrix_vector_product<...,RowMajor,...>::run (variant) |  | lib:Eigen GEMV kernel | - | used by TRMV 0x180060240 |
| 0x18005CDB0 | 3791 | Eigen::internal::general_matrix_vector_product<...,ColMajor,...>::run (variant) |  | lib:Eigen GEMV kernel (col-major, 128-col blocks) | - | used by triangular_solve_vector 0x1800611C0 |
| 0x18005DC80 | 2427 | Eigen::internal::product_triangular_matrix_matrix<double,long long,Mode,true,ColMajor,...>::run |  | lib:Eigen TRMM kernel driver | - | memset + gebp; BlasUtil "incr==1" |
| 0x18005E600 | 2470 | Eigen::internal::product_triangular_matrix_matrix<double,long long,Mode,true,RowMajor,...>::run |  | lib:Eigen TRMM kernel driver | - |  |
| 0x18005EFB0 | 2395 | Eigen::internal::product_triangular_matrix_matrix<...> (variant 3) |  | lib:Eigen TRMM kernel driver | - |  |
| 0x18005F910 | 2344 | Eigen::internal::product_triangular_matrix_matrix<...> (variant 4) |  | lib:Eigen TRMM kernel driver | - |  |
| 0x180060240 | 1346 | Eigen::internal::triangular_matrix_vector_product<long long,Mode,double,false,double,false,RowMajor,0>::run |  | lib:Eigen TRMV kernel | - | dot 0x180058A60 + GEMV 0x18005BDC0 |
| 0x180060790 | 2596 | Eigen::internal::triangular_solve_matrix<double,long long,OnTheLeft,Mode,false,ColMajor,ColMajor,1>::run (variant) |  | lib:Eigen TRSM | - |  |
| 0x1800611C0 | 1004 | Eigen::internal::triangular_solve_vector<double,double,long long,OnTheLeft,Mode,false,ColMajor>::run |  | lib:Eigen TRSV | - | caller 0x18014C520 only |
| 0x1800615B0 | 325 | Eigen::SelfAdjointEigenSolver<MatrixXd>::SelfAdjointEigenSolver<(expr)> | (const EigenBase<expr>&, int) | lib:Eigen SAES ctor from expression | - | updateErrorComputation: saes(0.5*p_inv.asDiagonal()*(H_+H_^T)*p_inv.asDiagonal()) |
| 0x180061700 | 173 | std::_Optimistic_temporary_buffer<uint64_t>::_Optimistic_temporary_buffer | (ptrdiff_t n) | lib:std stable_sort buffer | - | 512-element inline buffer, operator new(nothrow) |
| 0x1800617B0 | 86 | Eigen::Product<Lhs,Rhs,0>::Product |  | lib:Eigen expression ctor | - | "invalid matrix product" assert |
| 0x180061810 | 68 | Eigen::Product<Lhs,Rhs,0>::Product (variant) |  | lib:Eigen expression ctor | - |  |
| 0x180061860 | 231 | std::_Buffered_inplace_merge_divide_and_conquer2<std::pair<int,int>*, cmp(first)> |  | lib:std stable_sort<pair<int,int>> (compare .first only) | - | marginalizeOut: Pimax sorts the (start,len) pairs with a .first-only comparator (stable) |
| 0x180061950 | 451 | std::_Buffered_inplace_merge_divide_and_conquer2<uint64_t*, less<>> |  | lib:std stable_sort<uint64_t> | - | marginalizeOut: stable_sort(parameter_block_ids_copy) (upstream std::sort) |
| 0x180061B20 | 207 | std::_Buffered_inplace_merge_unchecked<std::pair<int,int>*, cmp(first)> |  | lib:std stable_sort<pair<int,int>> (compare .first only) | - | marginalizeOut: Pimax sorts the (start,len) pairs with a .first-only comparator (stable) |
| 0x180061BF0 | 244 | std::_Buffered_inplace_merge_unchecked<uint64_t*, less<>> |  | lib:std stable_sort<uint64_t> | - | marginalizeOut: stable_sort(parameter_block_ids_copy) (upstream std::sort) |
| 0x180061CF0 | 890 | std::_Buffered_inplace_merge_divide_and_conquer<std::pair<int,int>*, cmp(first)> |  | lib:std stable_sort<pair<int,int>> (compare .first only) | - | marginalizeOut: Pimax sorts the (start,len) pairs with a .first-only comparator (stable) |
| 0x180062070 | 711 | std::_Buffered_inplace_merge_divide_and_conquer<uint64_t*, less<>> |  | lib:std stable_sort<uint64_t> | - | marginalizeOut: stable_sort(parameter_block_ids_copy) (upstream std::sort) |
| 0x180062340 | 632 | std::_Uninitialized_chunked_merge_unchecked2/_Stable_sort part<std::pair<int,int>*, cmp(first)> |  | lib:std stable_sort<pair<int,int>> (compare .first only) | - | marginalizeOut: Pimax sorts the (start,len) pairs with a .first-only comparator (stable) |
| 0x1800625C0 | 556 | std::_Uninitialized_chunked_merge_unchecked2<uint64_t*, less<>> |  | lib:std stable_sort<uint64_t> | - | marginalizeOut: stable_sort(parameter_block_ids_copy) (upstream std::sort) |
| 0x1800627F0 | 614 | std::_Buffered_rotate_unchecked<std::pair<int,int>*, cmp(first)> |  | lib:std stable_sort<pair<int,int>> (compare .first only) | - | marginalizeOut: Pimax sorts the (start,len) pairs with a .first-only comparator (stable) |
| 0x180062A60 | 333 | std::_Chunked_merge_unchecked<std::pair<int,int>*, cmp(first)> |  | lib:std stable_sort<pair<int,int>> (compare .first only) | - | marginalizeOut: Pimax sorts the (start,len) pairs with a .first-only comparator (stable) |
| 0x180062BB0 | 202 | std::_Chunked_merge_unchecked<uint64_t*, less<>> |  | lib:std stable_sort<uint64_t> | - | marginalizeOut: stable_sort(parameter_block_ids_copy) (upstream std::sort) |
| 0x180062C80 | 343 | std::vector<MarginalizationError::ParameterBlockInfo>::_Emplace_reallocate<const ParameterBlockInfo&> |  | lib:std vector | - |  |
| 0x180062DE0 | 243 | std::vector<int>::_Emplace_reallocate<const int&> |  | lib:std vector (4-byte elems) | - | 41 callers (ICF) |
| 0x180062EE0 | 338 | std::vector<std::pair<int,int>>::_Emplace_reallocate<std::pair<int,int>> |  | lib:std vector | - | push_back in split*/marginalizeOut |
| 0x180063040 | 171 | std::_Tree<map<uint64_t,ResidualBlockSpec-like>>::_Erase |  | lib:std map erase | - | MarginalizationError +488 (value holds a shared_ptr) |
| 0x1800630F0 | 191 | std::_Tree<map<uint64_t,std::vector<...>>>::_Erase |  | lib:std map erase | - | MarginalizationError +528 |
| 0x1800631B0 | 180 | std::_Insertion_sort_unchecked<std::pair<int,int>*, cmp(first)> |  | lib:std stable_sort<pair<int,int>> (compare .first only) | - | marginalizeOut: Pimax sorts the (start,len) pairs with a .first-only comparator (stable) |
| 0x180063270 | 186 | std::_Insertion_sort_unchecked<uint64_t*, less<>> |  | lib:std stable_sort<uint64_t> | - | marginalizeOut: stable_sort(parameter_block_ids_copy) (upstream std::sort) |
| 0x180063330 | 92 | std::_Rotate_one / _Move_backward helper<std::pair<int,int>*, cmp(first)> |  | lib:std stable_sort<pair<int,int>> (compare .first only) | - | marginalizeOut: Pimax sorts the (start,len) pairs with a .first-only comparator (stable) |
| 0x180063390 | 258 | std::_Stable_sort_unchecked<std::pair<int,int>*, cmp(first)> |  | lib:std stable_sort<pair<int,int>> (compare .first only) | - | marginalizeOut: Pimax sorts the (start,len) pairs with a .first-only comparator (stable) |
| 0x1800634A0 | 258 | std::_Stable_sort_unchecked<uint64_t*, less<>> |  | lib:std stable_sort<uint64_t> | - | marginalizeOut: stable_sort(parameter_block_ids_copy) (upstream std::sort) |
| 0x1800635B0 | 642 | std::_Hash<_Umap_traits<uint64_t,...>>::_Forced_rehash |  | lib:std unordered_map rehash | - | "unordered_map/set too long"; caller 0x18007B280 |
| 0x180063840 | 191 | std::_Uninitialized_move<ParameterBlockInfo*> |  | lib:std | - | vector realloc |
| 0x180063900 | 1949 | Eigen::internal::call_assignment<MatrixXd,(product expression)> (marginalizeOut) |  | lib:Eigen dense assignment with product | - | -> 0x180072830 |
| 0x1800640A0 | 2965 | Eigen::MatrixBase<Block<...>>::applyHouseholderOnTheLeft<...> (evalTo variant) |  | lib:Eigen Householder | - | used by HouseholderSequence::evalTo |
| 0x180064C40 | 443 | Eigen::internal::apply_rotation_in_the_plane<Map<VectorXd,0,InnerStride>,...,double> |  | lib:Eigen Jacobi rotation | - | Jacobi.h:0x1CB |
| 0x180064E00 | 937 | Eigen::internal::llt_inplace<double,Lower>::blocked<MatrixXd> |  | lib:Eigen LLT blocked | - | LLT.h:0x155 |
| 0x1800651B0 | 58 | Eigen::internal::checkTransposeAliasing_impl<...>::run |  | lib:Eigen assert helper | - | Transpose.h:0x1B6 |
| 0x1800651F0 | 469 | Eigen::SelfAdjointEigenSolver<Matrix3d>::compute<Matrix3d> | (const EigenBase<Matrix3d>&, int options) | lib:Eigen SAES (3x3, iterative path) | - | used by pseudoInverseSymmSqrt<Matrix3d> |
| 0x1800653D0 | 1290 | Eigen::LLT<MatrixXd,Lower>::compute<MatrixXd> |  | lib:Eigen LLT compute | - | LLT.h:0x1B4; l1-norm 0x18006B510 |
| 0x1800658E0 | 1451 | Eigen::SelfAdjointEigenSolver<MatrixXd>::compute<MatrixXd> |  | lib:Eigen SAES compute | - | SAES.h:0x1AB/0x1AE |
| 0x180065E90 | 1701 | Eigen::SelfAdjointEigenSolver<MatrixXd>::compute<(expression)> |  | lib:Eigen SAES compute (expression input) | - |  |
| 0x180066540 | 1308 | Eigen::internal::computeFromTridiagonal_impl<Matrix3d> | (diag, subdiag, maxIterations=30, computeEigenvectors, eivec) | lib:Eigen SAES | - |  |
| 0x180066A60 | 1841 | Eigen::internal::computeFromTridiagonal_impl<MatrixXd> |  | lib:Eigen SAES | - | QR steps 0x180074E80, sort via minCoeff + swap |
| 0x1800671A0 | 533 | std::vector<ParameterBlockInfo>::emplace(const_iterator, const ParameterBlockInfo&) |  | lib:std vector insert | - | caller 0x180079D90 |
| 0x1800673C0 | 662 | std::_Hash<...>::_Forced_rehash (variant) |  | lib:std unordered rehash | - | caller 0x180079D90 |
| 0x180067660 | 774 | std::_Hash<...>::_Forced_rehash (variant, other bucket vector) |  | lib:std unordered rehash | - | caller 0x180080320 |
| 0x180067970 | 1338 | Eigen::internal::generic_product_impl<Product<Map<MatrixRM>,MatrixRM>,MatrixRM,...,GemmProduct>::evalTo |  | lib:Eigen product evalTo | - | EvaluateLocal: (J_i*J_lift)*J_plus |
| 0x180067EB0 | 2211 | Eigen::internal::generic_product_impl<Block<const MatrixXd>,MatrixRM,...,GemmProduct>::evalTo |  | lib:Eigen product evalTo | - | EvaluateWithMinimalJacobians: Jmin_i*J_lift |
| 0x180068760 | 1141 | Eigen::internal::generic_product_impl<Map<MatrixRM>,MatrixRM,...,GemmProduct>::evalTo |  | lib:Eigen product evalTo | - |  |
| 0x180068BE0 | 2123 | Eigen::internal::generic_product_impl<...>::evalTo (marginalizeOut variant) |  | lib:Eigen product evalTo | - |  |
| 0x180069430 | 2210 | Eigen::internal::generic_product_impl<...>::evalTo (addResidualBlock variant) |  | lib:Eigen product evalTo | - |  |
| 0x180069CE0 | 2164 | Eigen::internal::generic_product_impl<...>::evalTo (marginalizeOut variant) |  | lib:Eigen product evalTo | - |  |
| 0x18006A560 | 4007 | Eigen::HouseholderSequence<MatrixXd,VectorXd,1>::evalTo<MatrixXd,VectorXd> |  | lib:Eigen HouseholderSequence | - | builds Q for tridiagonalization |
| 0x18006B510 | 213 | Eigen::internal::redux_impl<scalar_sum_op,cwiseAbs(...)>::run (lpNorm<1>) |  | lib:Eigen redux | - | LLT l1 norm |
| 0x18006B5F0 | 835 | Eigen::internal::generic_product_impl<...,OuterProduct>::subTo (evalTo variant) |  | lib:Eigen outer product | - |  |
| 0x18006B940 | 1556 | Eigen::internal::generic_product_impl<...,GemvProduct>::scaleAndAddTo / assignment (addResidualBlock) |  | lib:Eigen GEMV product | - |  |
| 0x18006BF60 | 2811 | MarginalizationError::pseudoInverseSymmSqrt<Eigen::MatrixXd> | static bool (const MatrixBase<MatrixXd>& a, const MatrixBase<MatrixXd>& result, double epsilon, int* rank) | project | modified: LLT fast path (result = L^{-T}, rank = rows) before the SAES fallback | project template (dense marginalization) |
| 0x18006CA60 | 1090 | MarginalizationError::pseudoInverseSymmSqrt<Eigen::Matrix3d> | static bool (const MatrixBase<Matrix3d>&, const MatrixBase<Matrix3d>&, double epsilon, int* rank) | project | identical | project template (landmark blocks, sdim = 3) |
| 0x18006CEB0 | 492 | Eigen::internal::redux_impl<scalar_max_op,VectorXd>::run (maxCoeff) |  | lib:Eigen redux | - | saes.eigenvalues().maxCoeff() |
| 0x18006D0A0 | 137 | Eigen::internal::generic_product_impl<...,GemvProduct>::scaleAndAddTo (variant) |  | lib:Eigen GEMV product | - |  |
| 0x18006D130 | 112 | Eigen::internal::generic_product_impl<Block,Transpose<Block>,...,GemvProduct>::scaleAndAddTo (llt unblocked) |  | lib:Eigen GEMV product | - |  |
| 0x18006D1A0 | 542 | Eigen::internal::selfadjoint_product_impl<...,SelfAdjointShape,...>::run |  | lib:Eigen SYMV dispatch | - | SelfadjointMatrixVector.h:0xBA; -> 0x180087290 |
| 0x18006D3C0 | 122 | Eigen::internal::generic_product_impl<...,GemvProduct>::scaleAndAddTo (llt unblocked variant) |  | lib:Eigen GEMV product | - |  |
| 0x18006D440 | 584 | Eigen::internal::redux_impl<scalar_sum_op,conj_prod(...)>::run (dot, tridiagonalization) |  | lib:Eigen dot | - |  |
| 0x18006D690 | 569 | Eigen::internal::redux_impl<scalar_sum_op,conj_prod(...)>::run (dot) |  | lib:Eigen dot | - |  |
| 0x18006D8D0 | 600 | Eigen::internal::redux_impl<scalar_max_op,cwiseAbs(MatrixXd)>::run |  | lib:Eigen redux max\|x\| | - | SAES scaling |
| 0x18006DB30 | 97 | Eigen::internal::generic_product_impl<...,GemvProduct>::scaleAndAddTo (updateErrorComputation) |  | lib:Eigen GEMV product | - |  |
| 0x18006DBA0 | 631 | Eigen::internal::tridiagonalization_inplace_selector<Matrix3d,3,false>::run |  | lib:Eigen 3x3 tridiagonalization | - | comma initializer + sqrt |
| 0x18006DE20 | 1187 | Eigen::internal::tridiagonalization_inplace_selector<MatrixXd,Dynamic,false>::run |  | lib:Eigen tridiagonalization | - |  |
| 0x18006E2D0 | 975 | Eigen::internal::gemv_dense_selector<OnTheRight,ColMajor,true>::run (variant) |  | lib:Eigen GEMV dispatch | - |  |
| 0x18006E6A0 | 88 | Eigen::internal::generic_product_impl<...,GemvProduct>::scaleAndAddTo (variant) |  | lib:Eigen GEMV product | - |  |
| 0x18006E700 | 1019 | Eigen::internal::gemv_dense_selector<OnTheRight,ColMajor,true>::run (variant) |  | lib:Eigen GEMV dispatch | - |  |
| 0x18006EB00 | 1003 | Eigen::internal::gemv_dense_selector<OnTheRight,ColMajor,true>::run (variant) |  | lib:Eigen GEMV dispatch | - |  |
| 0x18006EEF0 | 997 | Eigen::internal::gemv_dense_selector<OnTheRight,ColMajor,true>::run (variant) |  | lib:Eigen GEMV dispatch | - |  |
| 0x18006F2E0 | 598 | Eigen::internal::gemv_dense_selector<OnTheRight,RowMajor,true>::run (variant) |  | lib:Eigen GEMV dispatch | - | -> 0x18004E940 |
| 0x18006F540 | 997 | Eigen::internal::gemv_dense_selector<OnTheRight,ColMajor,true>::run (variant) |  | lib:Eigen GEMV dispatch | - |  |
| 0x18006F930 | 165 | Eigen::internal::dense_assignment_loop (copy/assign kernel, variant) |  | lib:Eigen dense assignment kernel | - |  |
| 0x18006F9E0 | 237 | Eigen::internal::dense_assignment_loop (copy/assign kernel, variant) |  | lib:Eigen dense assignment kernel | - |  |
| 0x18006FAD0 | 180 | Eigen::internal::dense_assignment_loop (copy/assign kernel, variant) |  | lib:Eigen dense assignment kernel | - |  |
| 0x18006FB90 | 307 | Eigen::internal::dense_assignment_loop (copy/assign kernel, variant) |  | lib:Eigen dense assignment kernel | - |  |
| 0x18006FCD0 | 177 | Eigen::internal::dense_assignment_loop (copy/assign kernel, variant) |  | lib:Eigen dense assignment kernel | - |  |
| 0x18006FD90 | 166 | Eigen::internal::dense_assignment_loop (dst = src.array().sqrt()) |  | lib:Eigen dense assignment kernel | - |  |
| 0x18006FE40 | 205 | Eigen::internal::dense_assignment_loop (copy/assign kernel, variant) |  | lib:Eigen dense assignment kernel | - |  |
| 0x18006FF10 | 282 | Eigen::internal::evaluator<CwiseBinaryOp<...Block...>> ctor |  | lib:Eigen evaluator ctor | - |  |
| 0x180070030 | 277 | Eigen::internal::evaluator<CwiseBinaryOp<...Block...>> ctor (variant) |  | lib:Eigen evaluator ctor | - |  |
| 0x180070150 | 3713 | Eigen::internal::generic_product_impl<Product<...>,MatrixRM,...,GemmProduct>::scaleAndAddTo (nested) |  | lib:Eigen product scaleAndAddTo | - |  |
| 0x180070FE0 | 1591 | Eigen::internal::generic_product_impl<...,GemmProduct>::scaleAndAddTo (variant) |  | lib:Eigen product scaleAndAddTo | - | GEMV special cases + GEMM driver |
| 0x180071620 | 1627 | Eigen::internal::generic_product_impl<...,GemmProduct>::scaleAndAddTo (variant) |  | lib:Eigen product scaleAndAddTo | - | GEMV special cases + GEMM driver |
| 0x180071C80 | 1444 | Eigen::internal::generic_product_impl<...,GemmProduct>::scaleAndAddTo (variant) |  | lib:Eigen product scaleAndAddTo | - | GEMV special cases + GEMM driver |
| 0x180072230 | 1526 | Eigen::internal::generic_product_impl<...,GemmProduct>::scaleAndAddTo (variant) |  | lib:Eigen product scaleAndAddTo | - | GEMV special cases + GEMM driver |
| 0x180072830 | 1553 | Eigen::internal::generic_product_impl<...,GemmProduct>::scaleAndAddTo (variant) |  | lib:Eigen product scaleAndAddTo | - | GEMV special cases + GEMM driver |
| 0x180072E50 | 203 | Eigen::Select<...>::Select / evaluator |  | lib:Eigen select expression | - | (H_.diagonal().array() > 1e-9).select(...) |
| 0x180072F20 | 240 | Eigen::Select<...>::Select (variant) |  | lib:Eigen select expression | - |  |
| 0x180073010 | 295 | Eigen::TriangularViewImpl<const MatrixXd,Lower,Dense>::solveInPlace<OnTheLeft,MatrixXd> |  | lib:Eigen TRSM wrapper | - | SolveTriangular.h:0xAA |
| 0x180073140 | 344 | Eigen::internal::triangular_solver_selector<...,OnTheRight,...>::run (llt blocked) |  | lib:Eigen TRSM wrapper | - | -> 0x180014B50 |
| 0x1800732A0 | 3656 | MarginalizationError::splitSymmetricMatrix<MatrixXd,MatrixXd,MatrixXd,MatrixXd> | static void (const std::vector<std::pair<int,int>>&, A, U, W, V) | project | identical | project template (DEBUG_CHECKs compiled out) |
| 0x1800740F0 | 1503 | MarginalizationError::splitVector<VectorXd,VectorXd,VectorXd> | static void (const std::vector<std::pair<int,int>>&, b, b_a, b_b) | project | identical | project template |
| 0x1800746D0 | 1953 | Eigen::internal::call_assignment<MatrixXd,(product expression)> (marginalizeOut, variant) |  | lib:Eigen dense assignment with product | - | -> 0x180072830 |
| 0x180074E80 | 1511 | Eigen::internal::tridiagonal_qr_step<ColMajor,double,double,long long> |  | lib:Eigen SAES QR step | - |  |
| 0x180075470 | 4000 | Eigen::internal::tridiagonalization_inplace<MatrixXd,VectorXd> |  | lib:Eigen tridiagonalization | - | Tridiagonalization.h:0x163/0x164 |
| 0x180076410 | 129 | Eigen::internal::tridiagonalization_inplace<MatrixXd,VectorXd,VectorXd,VectorXd> |  | lib:Eigen tridiagonalization wrapper | - | Tridiagonalization.h:0x1B1 |
| 0x1800764A0 | 1691 | Eigen::internal::llt_inplace<double,Lower>::unblocked<MatrixXd> |  | lib:Eigen LLT unblocked | - | LLT.h:0x13D |
| 0x180076B40 | 1447 | Eigen::internal::llt_inplace<double,Lower>::unblocked<Block<MatrixXd>> |  | lib:Eigen LLT unblocked | - |  |
| 0x1800770F0 | 34 | Eigen::internal::evaluator<...> ctor (small) |  | lib:Eigen evaluator ctor | - |  |
| 0x180077120 | 69 | Eigen::CwiseNullaryOp<scalar_constant_op<double>,MatrixXd>::CwiseNullaryOp |  | lib:Eigen Constant ctor | - |  |
| 0x180077170 | 70 | Eigen::MapBase<Map<...>>::MapBase(ptr,rows,cols) |  | lib:Eigen Map ctor | - |  |
| 0x1800771C0 | 493 | Eigen::internal::evaluator<Solve<TriangularView<const MatrixXd,Lower>,Identity>>::evaluator |  | lib:Eigen solve evaluator | - | L^{-1} for pseudoInverseSymmSqrt<MatrixXd> |
| 0x1800773B0 | 44 | Eigen::internal::evaluator<...> ctor (small, variant) |  | lib:Eigen evaluator ctor | - |  |
| 0x1800773E0 | 724 | MarginalizationError::MarginalizationError | (Map& map) | project | modified: unordered_map, extra members, reserve(100/500/200) | ctor (sizeof 0x240) |
| 0x1800776C0 | 126 | MarginalizationError::ParameterBlockInfo::ParameterBlockInfo(const ParameterBlockInfo&) |  | lib:implicit copy ctor | - | compiler-generated |
| 0x180077740 | 457 | MarginalizationError::ParameterBlockInfo::ParameterBlockInfo | (uint64_t, std::shared_ptr<ParameterBlock>, size_t orderingIdx, bool is_landmark) | project | identical | header-inline struct ctor |
| 0x180077910 | 54 | Eigen::SelfAdjointEigenSolver<MatrixXd>::~SelfAdjointEigenSolver |  | lib:Eigen dtor (4 buffers) | - | also EH unwind |
| 0x180077950 | 20 | EH unwind helper: operator delete(obj+8) |  | lib:EH cleanup | - |  |
| 0x180077970 | 164 | std::unordered_map<uint64_t,size_t>::~unordered_map (_Hash::~_Hash) |  | lib:std | - | dtor of +400 |
| 0x180077A20 | 19 | std::_Optimistic_temporary_buffer<uint64_t>::~_Optimistic_temporary_buffer |  | lib:std | - | free if capacity > 512 |
| 0x180077A40 | 15 | EH unwind helper: conditional delete (owns flag at +16) |  | lib:std/EH | - |  |
| 0x180077A50 | 11 | EH unwind helper: free(obj+16) (Eigen dtor piece) |  | lib:EH cleanup | - |  |
| 0x180077A60 | 11 | EH unwind helper: free(obj+64) |  | lib:EH cleanup | - |  |
| 0x180077A70 | 11 | EH unwind helper: free(obj+8) |  | lib:EH cleanup | - |  |
| 0x180077A80 | 92 | std::list<...> node chain delete (unordered_map unwind) |  | lib:std | - |  |
| 0x180077AE0 | 42 | std::map<uint64_t,ResidualBlockSpec-like>::~map |  | lib:std | - |  |
| 0x180077B10 | 42 | std::map<uint64_t,std::vector<...>>::~map |  | lib:std | - |  |
| 0x180077B40 | 5 | (thunk) std::unordered_map<uint64_t,size_t>::~unordered_map |  | lib:std | - | -> 0x180077970 |
| 0x180077B50 | 178 | std::vector<ParameterBlockInfo>::_Tidy (~vector) |  | lib:std | - |  |
| 0x180077C10 | 94 | std::vector<Eigen::VectorXd,Eigen::aligned_allocator<VectorXd>>::~vector |  | lib:std | - | delta_b in marginalizeOut |
| 0x180077C70 | 532 | MarginalizationError::~MarginalizationError (body) |  | project | identical (implicit) | member destruction + ~CostFunction 0x1801B77D0 |
| 0x180077E90 | 137 | MarginalizationError::ParameterBlockInfo::~ParameterBlockInfo |  | lib:implicit dtor | - | releases two shared_ptrs |
| 0x180077F20 | 259 | MarginalizationError::ParameterBlockInfo::operator=(ParameterBlockInfo&&) |  | lib:implicit move-assign | - |  |
| 0x180078030 | 257 | std::map<uint64_t,ResidualBlockSpec-like>::operator[] (try_emplace) |  | lib:std map | - | node 0x48 |
| 0x180078140 | 253 | std::map<uint64_t,std::vector<...>>::operator[] (try_emplace) |  | lib:std map | - | node 0x40 |
| 0x180078240 | 144 | Eigen::CwiseBinaryOp<...>::CwiseBinaryOp (updateErrorComputation expr) |  | lib:Eigen expression ctor | - |  |
| 0x1800782D0 | 165 | Eigen::CwiseBinaryOp<...>::CwiseBinaryOp (marginalizeOut expr) |  | lib:Eigen expression ctor | - |  |
| 0x180078380 | 296 | Eigen::internal::evaluator<TriangularView/CwiseBinaryOp ...> ctor (SAES) |  | lib:Eigen expression ctor | - |  |
| 0x1800784A8 | 9 | [thunk]:MarginalizationError::`vector deleting destructor` (this-40) |  | project | - | ErrorInterface-side adjustor thunk |
| 0x1800784C0 | 79 | MarginalizationError::`scalar deleting destructor` | void* (unsigned flags) | project | - | calls 0x180077C70, free() |
| 0x180078510 | 46 | MarginalizationError::Evaluate | bool (double const* const*, double*, double**) const | project | modified: dispatch to EvaluateLocal when ErrorInterface flag set | ceres entry point |
| 0x180078540 | 2743 | MarginalizationError::EvaluateWithMinimalJacobians | bool (double const* const*, double*, double**, double**) const | project | modified: skips minimal_dimension==0, uses J_ block directly | residual e0_+J_*DeltaChi and lifted Jacobians |
| 0x180079000 | 2967 | MarginalizationError::EvaluateLocal (name from c05) | bool (double const* const*, double*, double**) const | project | new | minimal-size Jacobians J_blk*J_lift(lin)*J_plus(x) |
| 0x180079BA0 | 221 | std::vector<ParameterBlockInfo>::_Change_array |  | lib:std vector | - |  |
| 0x180079C80 | 9 | EH unwind helper: operator delete(obj+16) |  | lib:EH cleanup | - |  |
| 0x180079C90 | 49 | std::_Destroy_range<ParameterBlockInfo> |  | lib:std | - |  |
| 0x180079CD0 | 47 | std::_Ref_count_resource<double*,std::default_delete<double[]>>::_Get_deleter |  | lib:std shared_ptr | - | vtable slot 3 |
| 0x180079D00 | 131 | std::vector<ParameterBlockInfo>::_Reallocate_exactly (reserve) |  | lib:std vector | - |  |

## Types

### ImuMeasurement (32 bytes) — element of `ImuMeasurements = std::deque<ImuMeasurement, Eigen::aligned_allocator<...>>`
| off | type | name | evidence |
|---|---|---|---|
| 0 | double | timestamp_ | deque copy 0x180036F50; `**(double**)map[...]` reads in propagation/redo |
| 8 | Eigen::Vector3f | angular_velocity_ | floats at +8/+12/+16, cvtss2sd |
| 20 | Eigen::Vector3f | linear_acceleration_ | floats at +20/+24/+28 |
MSVC deque: proxy +0, map +8, mapsize +16, myoff +24, mysize +32; block size 1 (element > 8 bytes). Sure.

### ImuParameters (128 bytes, embedded at ImuError+56; defaults from ImuError ctor 0x1800398A0)
| off | type | name | default | evidence |
|---|---|---|---|---|
| 0 | double | a_max | 150.0 | propagation `imu_params[0]` vs acc |
| 8 | double | g_max | 35.0 | propagation `imu_params[1]` vs gyro |
| 16 | double | sigma_g_c | (none) | redo `this+72` |
| 24 | double | sigma_bg | (none) | upstream order |
| 32 | double | sigma_a_c | (none) | redo `this+88` |
| 40 | double | sigma_ba | (none) | upstream order |
| 48 | double | sigma_gw_c | (none) | redo `this+104` |
| 56 | double | sigma_aw_c | (none) | redo `this+112` |
| 64 | double | g | 9.80667 (0x40239D03D9A95422) | ctor |
| 72 | Vector3d | a0 | Zero | ctor zeroes 3 doubles |
| 96 | Vector3d? | ??? | Zero | ctor zeroes 3 doubles — TODO(verify) name |
| 120 | double | rate? | 1000.0 (0x408F400000000000) | ctor — no `delay_imu_cam` in Pimax |

### ImuError (sizeof 0x18B0 = 6320; make_shared allocates 0x18C0; `_Ref_count_obj2<ImuError>` vtable 0x1803AEFB0)
Bases: `ceres::SizedCostFunction<15,7,9,7,9,3>` (vtable 0x1803AF4B0: [dtor 0x18003AD60, Evaluate 0x18003AE00])
and `ErrorInterface` at +40 (vtable 0x1803AF4C8: [dtor-thunk 0x18003AD54, residualDim 0x18004E420,
parameterBlocks 0x180014980, parameterBlockDim 0x180014950, EvaluateWithMinimalJacobians 0x18003AE40, typeInfo 0x180050EE0]).
| off | type | name | evidence / sure? |
|---|---|---|---|
| 0 | vptr | CostFunction | sure |
| 8 | std::vector<int32> | parameter_block_sizes_ = {7,9,7,9,3} | ctor (xmmword_1803AF9F0 + 3) sure |
| 32 | int | num_residuals_ = 15 | sure |
| 40 | vptr | ErrorInterface | sure |
| 48 | bool | ErrorInterface flag (`use_minimal_jacobians_` (name from c03/c05)) = false | written in base-ctor phase; read by MarginalizationError::Evaluate — sure it is ErrorInterface's |
| 56 | ImuParameters | imu_parameters_ | sure |
| 184 | ImuMeasurements | imu_measurements_ | sure |
| 224 | double | t0_ | sure (no delay subtraction) |
| 232 | double | t1_ | sure |
| 240 | std::mutex (80) | preintegration_mutex_ | `_Mtx_init_in_situ(this+240, 2)` sure |
| 320 | Quaterniond | Delta_q_ | sure |
| 352 | Matrix3d | C_integral_ (= -J(6,12)) | redo epilogue, sure |
| 424 | Matrix3d | C_doubleintegral_ (= -J(0,12)) | sure |
| 496 | Vector3d | acc_integral_ (Δv) | sure |
| 520 | Vector3d | acc_doubleintegral_ (Δp) | sure |
| 544 | Matrix3d | cross_ | not touched by Pimax redo; read by EvaluateWithMinimalJacobians? (c03) |
| 616 | Matrix3d | dalpha_db_g_ (= -C_end*J(3,9)) | sure |
| 688 | Matrix3d | dv_db_g_ (= J(6,9)) | sure |
| 760 | Matrix3d | dp_db_g_ (= J(0,9)) | sure |
| 832 | Matrix<15,15> | P_delta_ | sure |
| 2632 | Matrix<9,1> | speed_and_biases_ref_ | sure |
| 2704 | bool | redo_ (= true) | sure |
| 2708 | int | redoCounter_ | sure (`++` after redo in 0x18003AE40) |
| 2712 | Matrix<15,15> | information_ | sure (not 16-aligned: 1800-byte matrices need no alignment) |
| 4512 | Matrix<15,15> | square_root_information_ | sure |

### LocalParamizationAdditionalInterfaces
vtable (as secondary base of every Pimax local parameterization): [dtor, Minus (+8), ComputeLiftJacobian (+16), verify (+24 = 0x180052D90)].
ceres::LocalParameterization slots used by verify: +8 Plus, +16 ComputeJacobian, +32 GlobalSize, +40 LocalSize.

### ParameterBlock (virtual slots used here; class owned elsewhere)
+16 `parameters()` (non-const overload; MSVC puts the const overload first at +8), +24 `dimension()`,
+32 `minimalDimension()`, +48 `plusJacobian(x0, J)`, +64 `liftJacobian(x0, J)`, +80 `localParameterizationPtr()`.
Data: `fixed_` bool at ParameterBlock+16.

### MarginalizationError (sizeof 0x240 = 576; `malloc(0x240)` + ctor 0x1800773E0 in Estimator 0x180027AE0)
CostFunction vtable 0x1803B05C0 = [scalar-deleting dtor 0x1800784C0, Evaluate 0x180078510, **EvaluateLocal 0x180079000 (Pimax-added virtual)**];
ErrorInterface vtable 0x1803B05E0 = [thunk 0x1800784A8, residualDim 0x180087010, 0x180014980, 0x180014950, EvaluateWithMinimalJacobians 0x180078540, typeInfo 0x180088D00].
| off | type | name | evidence |
|---|---|---|---|
| 0..39 | ceres::CostFunction | | ctor calls 0x1801B77B0; dtor 0x1801B77D0 |
| 40 | vptr | ErrorInterface | |
| 48 | bool | ErrorInterface flag | Evaluate dispatch |
| 56 | Map* | map_ptr_ | ctor |
| 64 | ResidualBlockId | residual_block_id_ | ctor |
| 72 | MatrixXd | unknown_mat_48_ (Pimax) | ctor zero / dtor free (names as in c05) |
| 96 | VectorXd | unknown_vec_60_ (Pimax) | |
| 112 | MatrixXd | unknown_mat_70_ (Pimax) | |
| 136 | VectorXd | unknown_vec_88_ (Pimax) | |
| 152 / 160 | double ×2 | unknown_98_ / unknown_a0_ (not initialised) | gap |
| 168 | MatrixXd | H_ | c05 (from marginalizeOut/updateErrorComputation) |
| 192 | VectorXd | b0_ | c05 |
| 208 | VectorXd | e0_ | Evaluate* (rows at +216) — sure |
| 224 | MatrixXd | J_ | Evaluate* (+232 rows, +240 cols) — sure |
| 248 | MatrixXd | U_ | |
| 272..352 | 6 × VectorXd | S_, S_sqrt_, S_pinv_, S_pinv_sqrt_, p_, p_inv_ | upstream order |
| 368 | volatile bool | error_computation_valid_ | ctor (=false) |
| 376 | vector<ParameterBlockInfo> (80 B elems) | parameter_block_infos_ | Evaluate* — sure; reserve(100) |
| 400 | unordered_map<uint64_t,size_t> (64 B) | parameter_block_id_to_parameter_block_info_idx_ | max_load_factor 1.0f, 8 buckets, mask 7 — upstream used std::map |
| 464 | vector<uint64_t> | new_parameter_block_ids_ (c05 name) | reserve(200) |
| 488 | map<ResidualBlockId, Map::ResidualBlockSpec> | residual_block_specs_ (c05) | node 0x48 (value = id, loss ptr, shared_ptr<ErrorInterface>), _Erase 0x180063040, operator[] 0x180078030 |
| 504 | vector<ResidualBlockId> | residual_block_ids_ (c05) | reserve(500) |
| 528 | map<ResidualBlockId, vector<uint64_t>> | residual_block_parameter_ids_ (c05) | node 0x40, _Erase 0x1800630F0, operator[] 0x180078140 |
| 544 / 548 / 552 | uint32 ×3 = 0 | num_new_dense_blocks_ / num_new_landmark_blocks_ / num_new_dimensions_ (c05) | default member init (qword + dword stores) |
| 560 | size_t | dense_indices_ | ctor body (=0) |
| 568 | size_t | unknown_238_ = 0 | default member init |

ParameterBlockInfo (80 B): +0 parameter_block_id, +8 shared_ptr<ParameterBlock>, +24 ordering_idx, +32 dimension,
+40 minimal_dimension, +48 local_dimension, +56 shared_ptr<double> linearization_point, +72 is_landmark — upstream-identical.

## External interfaces (calls out of the chunk)

| address | meaning (inferred) | used by |
|---|---|---|
| 0x18000C2C0 | `LOGE(fmt,...)` on logger 0x18046A000 | propagation, redo |
| 0x180354E50 / 0x18035AC70 / 0x180006290 / 0x1803551B0 | glog `LogMessage(file,line,sev)` / `.stream()` / `operator<<(const char*)` / `~LogMessage` | propagation (LOG(WARNING) lines 488, 495) |
| 0x18002E0E0 | `Sophus::SO3d(const Eigen::Quaterniond&)` (SO3exBase::normalize, "Quaternion ({}) should not be close to zero!" so3ex_base.h:125, threshold 1e-10) | propagation, redo |
| 0x180042060 | `Sophus::SO3d::exp(const Vector3d&, const double& eps=1e-5)` (Taylor series below eps) | propagation, redo |
| 0x18002E710 | `SO3d::operator*` (quaternion product + normalize check) | propagation, redo |
| 0x180029DF0 | `Eigen::Quaterniond::toRotationMatrix` / `SO3d::matrix()` | everywhere |
| 0x180040AD0 | `gamma1(phi, eps)` = Jl(φ) (c03 name) | propagation, redo |
| 0x180041C40 | `gamma1Right(phi, eps)` = Jl(−φ) | redo |
| 0x180040FE0 / 0x180041600 | `gamma2` (=2Γ₂) / `gamma3` (=6Γ₃) (phi, eps) | propagation, redo |
| 0x18002EF20 / 0x1800319C0 / 0x1800344E0 | `dGammaTV2/3/4(phi, d_omega, eps)` (c03 "family A") | propagation, redo |
| 0x180035AD0 / 0x180030480 / 0x180032F70 | `dGammaV1/2/3(phi, v, eps)` = ∂(n!Γₙ v)/∂φ (c03 "family B") | redo |
| 0x180042410 | `Sophus::SO3d::hat(Vector3d)` (comma init) | redo |
| 0x1800426F0 | `Eigen::Quaterniond::normalized()` | propagation |
| 0x1800089C0 | `kindr::minimal::RotationQuaternion(const Eigen::Quaterniond&)` (norm CHECK) | propagation |
| 0x18003A0F0 | `ImuMeasurements::operator[](i)` (out-of-line) | propagation |
| 0x180037510 | 9×9 `Block<9,3>*Matrix3d*Block<9,3>^T` evaluation into temp | redo |
| 0x180037E50 | `F*P*F^T` (15×15 GEMM 0x18001EC50) | redo |
| 0x180037140 / 0x180038B50 / 0x18002E0A0 / 0x180042690 | `LLT<Matrix15d>::compute` / `solveInPlace` / ctor / `matrixL()` | redo |
| 0x180008420 / 0x180022AF0 / 0x1800392A0 / 0x1800391F0 / 0x180039350 | Eigen `Constant` ctors 3×1 / 3×3 / 15×15 / 3×15 / 9×9 | redo, propagation |
| 0x180008720 / 0x1800087D0 / 0x1800391A0 / 0x1800396C0 | Eigen Block/diagonal ctors (assert-only) | |
| 0x1800385B0 / 0x1800074E0 / 0x180006E50 / 0x180008630 | copy kernel / fill / transpose-aliasing assert / alignment assert | |
| 0x18007AA20 | `MarginalizationError::computeDeltaChi(parameters, DeltaChi)` (c05) | Evaluate* |
| 0x1801B77B0 / 0x1801B77D0 | `ceres::CostFunction::CostFunction` / `~CostFunction` | ctor / dtor |
| 0x18000F840 / 0x18001B1E0 / 0x18007A840 | unordered_map bucket vector assign / `vector<8B>::reserve` / `vector<PBI>` allocate | ctor |
| 0x18001EBD0 / 0x1800178F0 / 0x18001F0F0 / 0x18001EC50 | `MatrixXd::resize` / `aligned_malloc` / col-major GEMV / GEMM | many |
| 0x180087290 / 0x180087B70 / 0x180014B50 / 0x180088A50 / 0x180088AE0 / 0x180088BD0 | SYMV kernel / LLT rank update / TRSM right / SAES helpers | Eigen templates |

Callers into the chunk: 0x180026100 (Estimator, c02/c03) → propagation; 0x18003AE40 (ImuError::EvaluateWithMinimalJacobians)
→ redoPreintegration(sb0, &g_S0 = C_WS_0ᵀ·(−parameters[4])), quaternionPlus/OplusMatrix; 0x180027AE0 (Estimator) →
MarginalizationError ctor; c05 (0x180079D90 addResidualBlock?, 0x18007B280, 0x180080320 marginalizeOut, 0x180088D10
updateErrorComputation) → ParameterBlockInfo ctor, vector/map/hash ops, stable_sorts, split*, pseudoInverseSymmSqrt, SAES ctor.

Globals: 0x18046A000 (Pimax logger), .rdata eps constants 0x1803AF4F8 (1e-5), 0x1803AF500 (1e-3), 0x1803AF508 (0.02),
0x1803AF510 (0.06), 0x1803AF518 (0.1); 0x1803AF290 (1e-12), 0x1803AF298 (1e-6) (verify); 0x1803AF9F0 {7,9,7,9}.

## Constants / defaults
- ImuParameters defaults: a_max 150, g_max 35, g 9.80667, a0 0, ??? 0, rate 1000.
- propagation/redo: `0.5`, `2/3` (0x3FE5555555555555), `3.0`, `dt*dt*0.5`, `-(dt*0.5)`; small-angle thresholds above.
- Noise: Q_g = σ_g_c²/dt·I, Q_a = σ_a_c²/dt·I; bias RW σ_gw_c²·dt, σ_aw_c²·dt on P(9..11), P(12..14).
- verify: dx = 1e-9 (central diff /2e-9), plus/minus tol 1e-12, Jacobian tol 1e-6, default perturbation 1e-6.
- MarginalizationError: reserve 100 / 500 / 200; unordered_map max_load_factor 1.0, 8 initial buckets.

## Quirks worth preserving
1. `propagation` reads timestamps from its argument but gyro/acc from `this->imu_measurements_`.
2. `propagation`: saturation warnings only when `covariance != nullptr` (never at the only call site), tested on the
   *bias-corrected* samples; sigmas no longer inflated; no covariance/Jacobian output at all; no delay_imu_cam.
3. Both integrators add gravity twice in a sense: the per-step accelerations are gravity-compensated in the local
   frame (`a_k += Cᵀ g_S`) *and* Δp/Δv get `-½dt² g_S` / `-dt g_S`; final T_WS also subtracts `½Δt² g_W`. Reproduce as is.
4. `redoPreintegration` takes no lock (upstream did), does not reset C_integral_/C_doubleintegral_/cross_/d*_db_g_
   (they are overwritten from J afterwards; cross_ is never written).
5. `information_` = LLT-solve of identity (not `.inverse()`); both symmetrised as upstream.
6. NaN semantics checked in asm: `if (dt <= 0.0) continue;` (NaN dt proceeds), `front().timestamp_ < t_end` error
   test (NaN passes), `nexttime == t_end` break.
7. `MarginalizationError::Evaluate` returns minimal-size Jacobians (rows × minimal_dimension) when the ErrorInterface
   flag is set (EvaluateLocal); both Evaluate paths skip blocks with minimal_dimension == 0.
8. `pseudoInverseSymmSqrt<MatrixXd>` returns L^{-T} (non-symmetric square root) and rank = rows whenever the LLT
   succeeds; SAES fallback only for non-PD input.  The 3×3 instantiation keeps the upstream SAES path.
9. (for c05) marginalizeOut uses `std::stable_sort` — on `std::pair<int,int>` with a `.first`-only comparator and on
   the `uint64_t` id copy (upstream `std::sort`).

## Eigen version evidence
Row-major GEMV uses `lhs.stride()*sizeof(Scalar)>32000 ? 0 : rows-7` (Eigen ≥ 3.3.5); `setRandom` = `-1 + 2*rand()/RAND_MAX`;
the `s*(A*B) → (s*A)*B` evaluator rule exists in all 3.3+/3.4, so products followed by a scale are written `(A*B)*s` in
the drafts to reproduce the observed "product temp, then scale" order.  Assert line numbers seen: LLT.h 0x89/0xC1/0x13D/0x155/0x1B4/0x206,
SelfAdjointEigenSolver.h 0x117/0x118/0x12E/0x1AB/0x1AE, Tridiagonalization.h 0x163/0x164/0x1B1, Transpose.h 0x1B6,
PlainObjectBase.h 0x115/0x130, HouseholderSequence.h 0xE7, BlockHouseholder.h 0x36, ColPivHouseholderQR.h 0x1E7, Jacobi.h 0x1CB.

## Confidence / open questions
- propagation: high (all paths read; FP order follows the asm; `C*(v*s)` vs `C*v*s` chosen from the observed multiply order).
- redoPreintegration: medium-high. Structure, constants, blocks of F/J and member updates are certain; open: (a) whether
  the acc-noise 9×9 term is a named local evaluated before `F P Fᵀ` (as drafted) or an Eigen temporary ordering effect —
  numerically the drafted order matches; (b) exact Eigen spelling of a few scaled products (`(C*X)*s`); (c) helper names.
- Helper semantics/names follow c03 (`so3_gamma.h`); the drafts call them by those names.
- `ImuParameters` fields at +96..+119 and +120 (`rate`?) need confirmation from the parameter loader chunk.
- MarginalizationError: names of Pimax members +112..+192, +464..+568 and of the 3rd CostFunction virtual (`EvaluateLocal`)
  and of the ErrorInterface flag are guesses; c05 should rename consistently.
- `pseudoInverseSymmSqrt`: whether the LLT path is a compile-time branch in one template or a separate Pimax function.

## Reconciliation with other chunks' drafts (for the coordinator)
1. **c03 `imu_error_c03.hpp`** declares `static int propagation(...)` without a gravity argument — wrong: 0x1800427F0 is a
   non-static member (reads `this->imu_measurements_` at +184/+208) with signature
   `(imu_measurements, imu_params, T_WS, speed_and_biases, t_start, t_end, const Eigen::Vector3d& g_W, covariance_t*, jacobian_t*)`
   (call site 0x180026743: rcx = ImuError, 10 arguments).  Use `draft/c04_imu_error/ceres_backend/imu_error.h`.
2. **c03** declares `redoPreintegration(const SpeedAndBias&, const Eigen::Vector3d& g_S)` — the binary null-checks the
   second argument (`if (g_S0)` at two places) → it is a pointer: `const Eigen::Vector3d* g_S0`.
3. **c05 `marginalization_error_impl.h`** marks `pseudoInverseSymmSqrt<MatrixXd>` (0x18006BF60) as upstream-identical —
   it is not: it first tries `Eigen::LLT` and, on success, returns `L^{-T}` (via `matrixL().solve(Identity)`, 0x1800771C0)
   with `*rank = a.rows()`; only on LLT failure does it run the upstream SAES code.  The Matrix3d instantiation
   (0x18006CA60) is upstream-identical.  See `draft/c04_imu_error/ceres_backend/marginalization_error.cpp`.
4. c05 already drafted the c04-range MarginalizationError functions (ctor, ParameterBlockInfo ctor, Evaluate,
   EvaluateWithMinimalJacobians, EvaluateLocal) — they agree with the c04 drafts (both use `.at(i)`: the loop calls
   `std::_Xout_of_range` 0x180010410 "invalid vector subscript"); keep one copy.
