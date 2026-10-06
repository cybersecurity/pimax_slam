# Chunk c03_estimator: 0x180027AE0 to 0x1800427F0

**Source objects.** The range covers the tail of `estimator.obj` and, from about 0x18002E0A0 on, the start of `imu_error.obj`.

- **estimator.obj tail.** The `__FILE__` strings are `src\ceres_backend\estimator.cpp` (lines 1168..1586) and `src\ceres_backend/estimator_types.hpp` (lines 187, 203).
- **Where imu_error.obj starts.** Everything from about 0x18002E0A0 on is only referenced from imu_error code (0x18003AE40, 0x1800427F0, 0x180045750), and the ImuError ctor, vtable functions and EvaluateWithMinimalJacobians lie at 0x1800398A0..0x18003AE40. These have no `__FILE__` string, so `imu_error.obj` starts inside this chunk: at the latest at 0x1800398A0, most likely at 0x18002E0A0. The first `imu_error.cpp` string (0x1800427F0) is only the first function that logs.
- **estimator.obj head.** It starts in chunk c02 at about 0x180020440: ctors, addCameraBundle, addImu, addStates, addLandmark, addGroundPlaneError, addVelocityPrior (see `notes`/`draft/c02_ceres_map`).

**Drafts.** Everything is under `draft/c03_estimator/`:

| file | contents |
|---|---|
| `ceres_backend/estimator.h` | Estimator class, merged layout with c02, `States`, `MarginalizationTiming`, `GroundPlaneConstraint` |
| `ceres_backend/estimator.cpp` | all Estimator functions of this chunk |
| `ceres_backend/estimator_types.h` | `MapPoint` (Pimax layout), `MapPoint::getTriangulationParallax`, `ErrorType` + `kGroundPlaneError` |
| `ceres_backend/parameter_block.h` | base class; two inline virtuals emitted here |
| `ceres_backend/gravity_parameter_block.h` | pimax-new |
| `ceres_backend/general_3d_parameter_block.h` | pimax-new |
| `ceres_backend/ground_plane_error.h` | pimax-new |
| `ceres_backend/homogeneous_point_local_parameterization.h` | upstream bodies moved into the header |
| `ceres_backend/imu_error_c03.{hpp,cpp}` | ImuError ctor / Evaluate / EvaluateWithMinimalJacobians / deltaQ (belong to imu_error.cpp; merge with chunk c04) |
| `common/sophus/so3ex_base.h`, `common/so3_gamma.h` | Sophus-variant and SO(3) "Gamma" helpers used by imu_error (see section 6) |

**Summary of the estimator part.** Upstream `svo::Estimator` became `pimax::totem::Estimator`, heavily modified:

- `applyMarginalizationStrategy`:
  - the frame lists are now members;
  - there is no keep vector, no extrinsics handling and no fixed-landmark handling;
  - VLOG(21) diagnostics are added;
  - a `get_T_WS` call whose result is unused remains.
- `optimize`:
  - adds a 25 ms solver time limit;
  - landmark gating sets low-parallax / over-observed points constant;
  - adds the ground-plane residual before the solve;
  - after the solve, Pimax-specific outlier handling updates the svo::Point / Frame bookkeeping.
- **New in Pimax:**
  - `GroundPlaneConstraint` and `GroundPlaneError` (relative plane-normal factor between two poses);
  - `GravityParameterBlock` (gravity becomes a state, 5th parameter block of ImuError);
  - `General3DParameterBlock` (landmarks are Euclidean 3-D blocks, no longer homogeneous);
  - `MapPoint::getTriangulationParallax`;
  - reset / resetMap.
- **Removed:**
  - `setOptimizationTimeLimit` / `CeresIterationCallback` (no vtable in the binary);
  - fixed landmarks;
  - `needPoseFixation`;
  - temporal extrinsics.

---

## 1. Function table

### 1a. Project code: estimator.obj

| address | proposed qualified name | signature | kind | upstream status | summary |
|---|---|---|---|---|---|
| 0x180027AE0 | `pimax::totem::Estimator::applyMarginalizationStrategy` | `bool (size_t num_keyframes, size_t num_imu_frames, MarginalizationTiming* timing)` | project | modified (heavy, see header comment in estimator.cpp) | sliding-window marginalization (OKVIS style), member frame lists, VLOG 1168..1194 |
| 0x180029610 | `Estimator::reset` | `void ()` | project | new | clear states (and `states_ = States()`), landmarks, ground plane, gp residual ids, prior |
| 0x1800298A0 | `Estimator::resetGroundPlaneConstraint` | `void ()` | project | new | `ground_plane_ = GroundPlaneConstraint()` (called by frontend 0x1800F5170, 0x18011B420) |
| 0x180029B80 | `Estimator::getPoseEstimate` | `std::pair<Transformation,bool> (BackendId) const` | project | modified | `#ifndef NDEBUG` branch with `dynamic_pointer_cast` but **without** the CHECK |
| 0x180029F30 | `Estimator::getSpeedAndBias` | `bool (BackendId, SpeedAndBias&) const` | project | identical | changeIdType(ImuStates) + getSpeedAndBiasEstimate |
| 0x180029F90 | `Estimator::getSpeedAndBiasEstimate` | `std::pair<SpeedAndBias,bool> (BackendId) const` | project | modified | as getPoseEstimate; `estimate()` virtual (vtable +0x68) |
| 0x18002A160 | `MapPoint::getTriangulationParallax` | `double () const` | project (inline, estimator_types.hpp) | new | max angle between first observation's ray and the others (float math, acosf); LOG(ERROR) lines 187/203 |
| 0x18002A4F0 | `Estimator::get_T_WS` | `bool (BackendId, Transformation&) const` | project | identical | |
| 0x18002A6D0 | `Estimator::optimize` | `void (size_t num_iter, bool verbose, bool /*unused*/)` | project | modified (heavy) | ceres options, landmark gating, ground-plane residual, solve, landmark update/outlier bookkeeping, LOG lines 1454..1586 |
| 0x18002B960 | `Frame::pos()` (out-of-line inline, returns `Eigen::Vector3f`) | `Eigen::Vector3f () const` | project (inline of the Frame header) | modified? | `T_f_w_.inverse().getPosition().cast<float>()`; only caller 0x18002A160. TODO(verify) whether Frame::pos() itself returns float |
| 0x18002B9D0 | `Estimator::printStates` | `void (BackendId, std::ostream&) const` | project | modified | temporal-extrinsics branch removed |
| 0x18002BFB0 | `Estimator::registerFixedFrame` | `void (uint64_t)` | project (inline, out-of-line copy) | modified | `fixed_frame_parameter_ids_.insert(id)`; the CHECK of checkAndAddToSet is removed |
| 0x18002C080 | `Estimator::removeGroundPlaneErrors` | `void ()` | project | new | removeResidualBlock for each id in `ground_plane_residual_ids_`, clear |
| 0x18002C100 | `Estimator::removeObservation` | `bool (ceres::ResidualBlockId)` | project | modified | `landmarks_map_.at(parameterBlockIdOfResidual(id,1)).observations.erase(id)`; returns removeResidualBlock() |
| 0x18002C350 | `Estimator::resetMap` | `void ()` | project | new | `map_ptr_ = make_shared<Map>()`, counter 0, gp reset, `marginalization_residual_id_ = 0`, fixed set cleared |
| 0x18002C530 | `Estimator::setGroundPlaneConstraint` | `void (const GroundPlaneConstraint&)` | project | new | copy, validate (valid, ids>=0, ids differ, norms>1e-12, finite), normalize, sigma clamped to [0.05,0.25], else reset |
| 0x18002C7B0 | `Estimator::setOldestFrameFixed` | `void ()` | project | identical | PoseError with info diag(1e14,1e14,1e14,0,0,1e14) on `states_.ids[0]`. 2nd register arg is garbage (called with a leftover value from 0x180013680) |
| 0x18002CC70 | `Estimator::setPoseEstimateAndZeroVelocity` (name TODO) | `void (const BackendId&, const Transformation&)` | project | new | PoseParameterBlock::setEstimate(T); speed&bias velocity part zeroed if the block exists |
| 0x18002CF00 | `Estimator::updateAllActivePoints` | `void () const` | project | modified | `point->pos_ = hom.head<3>().cast<float>()` for points with `obs_.size() >= 2` (no isLandmarkFixed) |

### 1b. Project code: parameter blocks and errors (header-inline, emitted in estimator.obj)

| address | proposed qualified name | signature | kind | upstream status | summary |
|---|---|---|---|---|---|
| 0x18002A560 | `GravityParameterBlock::liftJacobian` | `void (const double*, double*) const` | project | new | `J(2x3,RowMajor) = TangentBasis(x).transpose()` |
| 0x18002B800 | `GravityParameterBlock::plusJacobian` | `void (const double*, double*) const` | project | new | `J(3x2,RowMajor) = TangentBasis(x)` |
| 0x18002C510 | `GravityParameterBlock::setEstimate` | `void (const Eigen::Vector3d&)` | project | new | estimate_ (+0x20) = g |
| 0x18002CEC0 | `GravityParameterBlock::typeInfo` | `std::string () const` | project | new | "GravityParameterBlock" |
| 0x18002A6C0 | `ParameterBlock::localParameterizationPtr` | `const ceres::LocalParameterization* () const` | project (inline) | identical | return +0x18 (also ICF-used by ceres BlockSparseMatrix vtable) |
| 0x18002C7A0 | `ParameterBlock::setLocalParameterizationPtr` | `void (const ceres::LocalParameterization*)` | project (inline) | identical | +0x18 = p |
| 0x18002B7F0 | `*ParameterBlock::parameters()` / `parameters() const` / `General3DParameterBlock::estimate()` / `SpeedAndBiasParameterBlock::estimate()` (ICF) | `double* ()` | project (inline) | identical | return this+0x20 |
| 0x18002CFB0 | `General3DParameterBlock::General3DParameterBlock` | `(const Eigen::Vector3d&, uint64_t id, bool initialized)` | project | new (3-D copy of HomogeneousPointParameterBlock) | |
| 0x18002CFF0 | `General3DParameterBlock::~General3DParameterBlock` (scalar deleting) | | project (implicit) | new | |
| 0x18002D030 | `General3DParameterBlock::plusJacobian` = `liftJacobian` (ICF) | `void (const double*, double*) const` | project | new | 3x3 identity |
| 0x18002D060 | `General3DParameterBlock::minus` | | project | new | delta = x1 - x0 |
| 0x18002D090 | `General3DParameterBlock::plus` | | project | new | x0 + delta |
| 0x18002D0C0 | `General3DParameterBlock::setEstimate` | `void (const Eigen::Vector3d&)` | project | new | |
| 0x18002D0D0 | `General3DParameterBlock::typeInfo` | | project | new | "General3DParameterBlock" |
| 0x18002D110 | `GroundPlaneError::GroundPlaneError` | `(const Vector3d& n0, const Vector3d& n1, double sigma)` | project | new | normals normalized if norm>1e-12 and finite, else (0,0,1); weight = 1/max(|sigma|,1e-6) |
| 0x18002D338 | `GroundPlaneError::~GroundPlaneError` thunk (this-0x28) | | project (implicit) | new | |
| 0x18002D350 | `GroundPlaneError::Evaluate` | `bool (double const* const*, double*, double**) const` | project | new | flag +0x30 selects minimal/full Jacobians; direct call |
| 0x18002D390 | `GroundPlaneError::EvaluateWithMinimalJacobians` | `bool (double const* const*, double*, double**, double**) const` | project | new | r = w(R1 n1 - R0 n0); J_min(3x6) = [0 \| ±w [R n]x]; J = J_min * J_lift |
| 0x18002DEF0 | `GroundPlaneError::typeInfo` | `ErrorType () const` | project | new | returns 7 (`kGroundPlaneError`) |
| 0x18002DF00 | `HomogeneousPointLocalParameterization::ComputeJacobian` | | project | identical (inline in Pimax) | 4x3 [I;0] |
| 0x18002DF40 | `HomogeneousPointLocalParameterization::ComputeLiftJacobian` | | project | identical | 3x4 [I 0] |
| 0x18002DF90 | `HomogeneousPointLocalParameterization::Minus` | | project | identical | DEBUG_CHECK compiled out |
| 0x18002DFC0 | `HomogeneousPointLocalParameterization::Plus` | | project | identical | |

### 1c. Project code: imu_error.obj part (drafted in imu_error_c03.*)

| address | proposed qualified name | signature | kind | upstream status | summary |
|---|---|---|---|---|---|
| 0x1800398A0 | `ImuError::ImuError` | `(const ImuMeasurements&, const ImuParameters&, const double& t_0, const double& t_1, const SpeedAndBias& speed_and_biases_ref)` | project | modified: `SizedCostFunction<15,7,9,7,9,3>`, extra speed&bias reference, no lock, no delay_imu_cam, no DEBUG_CHECK | copies measurements (deque assign 0x180036F50), parameters, t0, t1, sab ref |
| 0x18003AD60 | `ImuError::~ImuError` (scalar deleting) | | project (implicit) | identical | mutex (+240), deque (+184), ~CostFunction, aligned free (0x18B0) |
| 0x18003AD54 | `ImuError::~ImuError` thunk (this-40) | | project (implicit) | identical | |
| 0x18003AE00 | `ImuError::Evaluate` (ICF-folded with `PoseError::Evaluate`, `SpeedAndBiasError::Evaluate`) | `bool (double const* const*, double*, double**) const` | project | modified | `use_minimal_jacobians_` (+48) selects (p,r,nullptr,J) vs (p,r,J,nullptr); virtual call |
| 0x18003AE40 | `ImuError::EvaluateWithMinimalJacobians` | `bool (double const* const*, double*, double**, double**) const` | project | modified (see quirks) | 15-dim IMU residual incl. gravity block |
| 0x180041ED0 | `ceres_backend::deltaQ` (sinc inlined) | `Eigen::Quaterniond (const Eigen::Vector3d&)` | project (header inline, out of line) | identical | |

### 1e. Project code: SO(3) / Sophus-variant helpers (imu_error.obj; drafted in common/sophus/so3ex_base.h and common/so3_gamma.h, file and function names are guesses)

| address | proposed qualified name | signature | kind | upstream status | summary |
|---|---|---|---|---|---|
| 0x18002E0E0 | `Sophus::SO3<double,0>::SO3(const Quaterniond&)` + inlined `SO3exBase::normalize` | ctor | project (vendored Sophus variant) | modified: "{}"-style ensure, so3ex_base.h:125 | normalise; ensure "Quaternion ({}) should not be close to zero!" if norm < 1e-10 |
| 0x18002E710 | `Sophus::SO3exBase<SO3d>::operator*(const SO3exBase&) const` | `SO3d (const SO3d&)` | project (vendored) | identical | quaternion product then normalising ctor |
| 0x180042060 | `Sophus::SO3<double>::exp(const Tangent&, const double& eps)` (name TODO) | static SO3d | project (vendored, Pimax) | new | θ<eps: alternating series (inlined 0x1800409E0); else sin(θ/2)/θ, cos(θ/2); normalise. Callers 0x1800427F0, 0x180045750 (eps 1e-5) |
| 0x180042530 | `Sophus::SO3exBase<SO3d>::log() const` | `Vector3d ()` | project (vendored) | newer-Sophus logAndTheta (atan2 wrap) | small branch `2/w - (2/3)n²/w³`; ensure "Quaternion ({}) should be normalized!" line 109. Caller 0x18003AE40 |
| 0x180042410 | `Sophus::SO3<double>::hat(const Vector3d&)` | static Matrix3d | project (vendored) | identical | comma initializer |
| 0x1800423F0 | `Sophus::SO3exBase::matrix() const` | `Matrix3d ()` | project (vendored) | identical | -> toRotationMatrix 0x180029DF0 (callers 0x18009B240, 0x1800A0AB0) |
| 0x1800409E0 | `alternatingSeries(const double& x, int n, int p, double x2 = NaN, const double& tol = DBL_EPSILON)` (name TODO) | `double` | project | new | Σ (-1)^j x^(p+2j)/(n+2j)!, stops at first |term| <= tol |
| 0x180040AD0 | `gamma1(phi, eps)` = Jl(φ) | `Matrix3d (const Vector3d&, const double&)` | project | new | eps 1e-5 |
| 0x180040FE0 | `gamma2(phi, eps)` = 2Γ₂ | same | project | new | eps 1e-3 |
| 0x180041600 | `gamma3(phi, eps)` = 6Γ₃ | same | project | new | eps 0.02 |
| 0x180041C40 | `gamma1Right(phi, eps)` = Jl(-φ) | same | project | new | caller 0x180045750 |
| 0x18002EF20 | `dGammaTV2(phi, v, eps)` (family A, n=2) | `Matrix3d (const Vector3d&, const Vector3d&, const double&)` | project | new | eps 0.02; both imu callers |
| 0x1800319C0 | `dGammaTV3` (family A, n=3) | same | project | new | eps 0.06 |
| 0x1800344E0 | `dGammaTV4` (family A, n=4) | same | project | new | eps 0.1 |
| 0x180035AD0 | `dGammaV1` (family B, n=1) = ∂(Jl·v)/∂φ | same | project | new | eps 0.001; caller 0x180045750 |
| 0x180030480 | `dGammaV2` (family B, n=2) | same | project | new | eps 0.02 |
| 0x180032F70 | `dGammaV3` (family B, n=3) | same | project | new | eps 0.06 |
| 0x180037490 | `Sophus::ensureFailed<Transpose<const Vector4d>>` | `[[noreturn]] void (func, file, line, desc, args...)` | project (vendored) | modified | printf "mlog ensure failed in function '%s', file '%s', line %d.\n", cout << FormatString, abort |
| 0x18002EDE0 | `Sophus::details::FormatString<...>` | `std::string (const char*, T&&)` | project (vendored) | identical shape | stringstream + FormatStream |
| 0x18002E970 | `Sophus::details::FormatStream<Transpose<const Vector4d>>` | `void (std::stringstream&, const char*, T&&)` | project (vendored) | modified: "{}", "{{", "{.N}" | else "\nFormat-Warning: There are 1 args unused." |


### 1d. Library / template instantiations (not reconstructed)

| address | identification |
|---|---|
| 0x180029930 | `std::allocator<4-byte T>::deallocate` (ICF, used by vector<bool>/vector<int> storage, many callers) |
| 0x180029980 | `std::vector<bool>::erase(const_iterator)` (States::removeState) |
| 0x180029DF0 | `Eigen::QuaternionBase<Quaterniond>::toRotationMatrix` (callers in ceres_map, imu_error, frontend) |
| 0x18002BE30 | `std::vector<bool>::push_back` -> `_Insert_x` 0x1800253B0 + bit fill (States::addState, 0x180185E40) |
| 0x18002C2F0 | `std::shared_ptr<T>::reset()` (MarginalizationError; ICF with frontend use 0x1800FA6B0) |
| 0x18002CB90 | `vk::Timer::start()` (out-of-line inline; QueryPerformanceCounter steady clock) |
| 0x18002CC00 | `vk::Timer::stop()` -> seconds (`duration*1e-9`) |
| 0x18002DDA0 | `Eigen::QuaternionBase<Map<const Quaterniond>>::normalized()` (GroundPlaneError) |
| 0x18002E0A0 | `Eigen::LLT<Matrix<double,15,15>>::LLT(const Matrix&)` (copy 0x708 bytes + compute 0x180037140); caller redoPreintegration 0x180045750 |
| 0x180036F50 | `std::deque<ImuMeasurement, aligned_allocator>::_Assign_range` (ImuError ctor) |
| 0x180041CA0 | `std::deque<ImuMeasurement>::_Growmap` |
| 0x180041E40 | `std::deque<ImuMeasurement>::_Tidy` |
| 0x1800376C0 / 0x1800377F0 / 0x180037590 | Eigen `Matrix15d * Block<15,6>` / `<15,9>` / `Matrix<15,3>` products -> gemm 0x18004E4A0 |
| 0x180037920 | Eigen `Matrix<15,3> * MatrixXd` -> `Matrix<double,15,Dynamic>` |
| 0x180038560 | Eigen gemv `Matrix15d * Vector15` -> 0x18001F0F0 |
| 0x1800385B0 | Eigen dense-assign copy kernel |
| 0x180039400 / 0x1800394B0 / 0x180039560 / 0x180039610 | Eigen `CwiseNullaryOp<scalar_constant_op>` ctors (15x1 / 15x3 / 15x6 / 15x9) |
| 0x18003A0B0 | Eigen `DenseStorage<double,Dynamic>::swap` (MatrixXd move-assign) |
| 0x18002E1C0 | `std::operator<<(std::ostream&, char)` |
| 0x18002E380 | `Eigen::operator<<(ostream&, DenseBase<Transpose<const Vector4d>>)` (default IOFormat) |
| 0x180038100 | `Eigen::internal::print_matrix<Matrix<double,1,4>>` |
| 0x1800423D0 | `std::fixed(std::ios_base&)` |
| 0x180042690 | `Eigen::LLT<Matrix15d>::matrixL()/matrixU()` accessor (LLT.h:0x89 assert) |
| 0x180037140 | `Eigen::LLT<Matrix15d,Lower>::compute` |
| 0x180038CD0 | `Eigen::internal::llt_inplace<double,Lower>::unblocked` (15x15) |
| 0x180038B50 | `Eigen::LLT<Matrix15d>::solveInPlace` |
| 0x180037510 / 0x180038640 | Eigen 9x9 product evalTo / unrolled small product kernel |
| 0x180037E50 | Eigen 15x15 GEMM evalTo -> 0x18001EC50 |
| 0x180037F70 | Eigen 15x15 product with Identity operand -> gemm 0x18004E4A0 |
| 0x180038080 | Eigen gemm_functor dispatch -> 0x18001EC50 |
| 0x1800384F0 | Eigen gemv wrapper (stride 15) -> 0x18001F0F0 |
| 0x180038900 | Eigen `generic_product_impl<...Dynamic...>::scaleAndAddTo` |
| 0x180039870 | Eigen `gemm_blocking_space<...,15,15,15,1,true>` ctor |
| 0x1800391A0 / 0x1800391F0 / 0x1800392A0 / 0x180039350 / 0x1800396C0 / 0x180039740 | Eigen expression / CwiseNullaryOp / Block ctors (`v == T(Value)` asserts) |
| 0x1800397F0 | Eigen nested product/sum evaluator copy |
| 0x180039830 | minkindr `QuatTransformationTemplate<double>(const Position&, const Eigen::Quaterniond&)` (callers 0x180110490, 0x18018D770) |
| 0x18003A080 | `std::deque<ImuMeasurement>::~deque` (static dtor via atexit thunks) |
| 0x18003A0F0 | `std::deque<ImuMeasurement>::operator[]` |
| 0x18003A120 / 0x18003A1F0 | `Eigen::CommaInitializer<Matrix3d>` / 4-column CommaInitializer `operator,` |
| 0x18003A2C0 | Eigen `gemm_pack_lhs` |
| 0x18003A6D0 / 0x18003AA40 | Eigen `gemm_pack_rhs` (ColMajor / RowMajor, PanelMode) |
| 0x1800426F0 | `Eigen::QuaternionBase::normalized` |

---

## 2. Types

### 2a. `pimax::totem::Estimator` (embedded in CeresBackendInterface at +0xB0)

The full declaration is in `draft/c03_estimator/ceres_backend/estimator.h`. "c02" means taken from chunk c02 (ctor/dtor); "sure" means accessed with a clear meaning in c03.

| offset | type | name | evidence |
|---|---|---|---|
| 0x000 | `std::map<uint64_t,uint64_t>` | unknown_map_0_ | c02 |
| 0x010 | `std::map<uint64_t, vector<...>>` | unknown_map_10_ | c02 |
| 0x020 | bool | is_reinit_ | c02 |
| 0x028 | `Matrix<double,9,1>` | reinit_speed_bias_ | c02 |
| 0x070 | Transformation | reinit_T_WS_ | c02 |
| 0x0B0 | double | reinit_timestamp_start_ | c02 |
| 0x0B8 | uint64_t | unknown_b8_ | c02 |
| 0x0C0 | `std::vector<BackendId>` | **marginalize_pose_frames_** | sure (0x180027AE0: cleared, filled, searched) |
| 0x0D8 | `std::vector<BackendId>` | **marginalize_all_but_pose_frames_** | sure |
| 0x0F0 | `std::vector<BackendId>` | **all_linearized_frames_** | sure (`.at(0)` = current_kf_id) |
| 0x108 | `PointMap` (head, size at +0x110) | landmarks_map_ | sure |
| 0x118 | CameraBundlePtr | camera_rig_ | c02 |
| 0x128 | `std::vector<BackendId>` | constant_extrinsics_ids_ | sure (printStates) |
| 0x140 | States (`ids` +0x140, `is_keyframe` +0x158 (_Mysize +0x170), `timestamps` +0x178) | states_ | sure |
| 0x190 | `shared_ptr<ceres_backend::Map>` | map_ptr_ | sure |
| 0x1A0 | ExtrinsicsEstimationParametersVec | extrinsics_estimation_parameters_ | c02 |
| 0x1B8 | ImuParameters (0x80) | imu_parameters_ | c02 |
| 0x238 / 0x248 / 0x258 | `shared_ptr<ceres::LossFunction>` | cauchy / huber / ground_plane loss | c02 |
| 0x268 | GroundPlaneConstraint (0x48) | ground_plane_ | sure (0x18002C530, 0x1800298A0) |
| 0x2B0 | `std::vector<ceres::ResidualBlockId>` | ground_plane_residual_ids_ | sure (0x18002C080) |
| 0x2C8 | `shared_ptr<MarginalizationError>` | marginalization_error_ptr_ | sure |
| 0x2D8 | `ceres::ResidualBlockId` | marginalization_residual_id_ | sure |
| 0x2E0 | `std::set<uint64_t>` (size +0x2E8) | fixed_frame_parameter_ids_ | sure (registerFixedFrame, deRegister inline, hasFixedPose) |
| 0x2F0 | uint64_t | gravity_parameter_block_id_ (= -2) | c02 |
| 0x2F8 | int | **optimize_count_** (name TODO) | sure (optimize: `<=10` then `++`, resetMap = 0) |

**Differences from upstream:**
- No `ceres_callback_`, no `fixed_landmark_parameter_ids_`, no `estimate_temporal_extrinsics_`, no `min_num_3d_points_for_fixation_`.
- The `imu_parameters_` vector became a single struct.
- New members: the three frame lists, ground plane, gravity id and counter.

### 2b. `GroundPlaneConstraint` (pimax-new, 0x48 bytes)

| off | type | name | evidence |
|---|---|---|---|
| 0x00 | bool | valid | sure |
| 0x04 | int | bundle_id_0 | sure (`createNFrameId` in addGroundPlaneError) |
| 0x08 | int | bundle_id_1 | sure |
| 0x10 | Vector3d | normal_0 (default (0,0,1)) | sure |
| 0x28 | Vector3d | normal_1 (default (0,0,1)) | sure |
| 0x40 | double | sigma (default 0.15, clamp [0.05,0.25]) | sure |

The producer is 0x1800F5170, which logs "ground-plane relative normal candidates %lu fit %lu overlap %.3f delta %.3f sigma %.3f". It sends {1, prev_bundle, cur_bundle, prev_normal, cur_normal, clamp(0.5*sqrt(..), 0.08, 0.18)}.

### 2c. `MapPoint` (PointMap value, 0x80 bytes; node 0xB0, value at node+0x30)

| off | type | name | evidence |
|---|---|---|---|
| 0x00 | Vector4d | hom_coordinates | sure (optimize writes head<3> and [3]=1.0) |
| 0x20 | PointPtr | point | sure |
| 0x30 | `std::unordered_map<uint64_t residual_id, KeypointIdentifier>` | observations | sure (FNV-1a 8-byte hash lookup by residual id; list node: key +0x10, frame weak_ptr +0x18/+0x20, frame_id +0x28, keypoint_index_ +0x30) |
| 0x70 | bool | fixed_position: Pimax uses it as "set constant for this solve" | sure |
| 0x78 | double | parallax (ctor 10.0, c02) | sure (written by optimize) |

Upstream used `std::map<KeypointIdentifier,uint64_t>`.

### 2d. svo `Point` fields used here (owner: frontend chunk; offsets are sure, names guessed)

| off | type | name (guess) | use |
|---|---|---|---|
| 0x00 | int | id_ | |
| 0x04 | Vector3f | pos_ | updateAllActivePoints writes `hom.head<3>().cast<float>()` |
| 0x10 | `unordered_map<int, FrameWeakPtr>` (size +0x20) | obs_ | `obs_.size() < 2` gating; `removeObservation(frame_id)` 0x18009BBE0 |
| 0x50 | int | n_failed_reproj_ | ++ on BA outlier |
| 0x54 | int | n_succeeded_reproj_ | -- on BA outlier |
| 0x60 | `std::set<int>` (size +0x68) | ba_bundle_ids_ | insert frame->bundle_id_; `> 20` gating; cleared |
| 0x70 | int | ba_inlier_count_ | ++ if chi2 < 1.0; ratio gating |
| 0x74 | int | ba_total_count_ | ++ per kept observation |
| 0x78 | int | ba_obs_frames_ (?) | `< 2` gating (first loop). TODO(verify) meaning |
| 0x80 | bool | in_ba_graph_ | false when removed |

### 2e. `Frame` fields used here (owner: frontend chunk)

| off | type | name | use |
|---|---|---|---|
| 0x10 | int | id_ | Point::removeObservation key |
| 0x20 | int | bundle_id_ | `createNFrameId(bundle_id_)` = `(int64)bundle_id << 16`; inserted in Point::ba_bundle_ids_ |
| 0x40 | Transformation | T_f_w_ | pos() = `T_f_w_.inverse().getPosition()` (0x180013040) |
| 0xB0 | bool | is_keyframe_ (?) | on BA outlier: point->removeObservation(frame id) only if set. TODO(verify) |
| 0x2A8 | `std::vector<PointPtr>` | landmark_vec_ | reset to nullptr on outlier |
| 0x2C0 | `Eigen::VectorXi` (size +0x2C8) | track_id_vec_ | `== -1`: drop observation; set to -1 on outlier |

### 2f. Parameter blocks and errors (pimax-new)

**Common layout.** All parameter blocks start with ParameterBlock: +0 vptr, +8 id_, +0x10 fixed_, +0x18 local_parameterization_ptr_.

**Virtual slot order:**

| slots | functions |
|---|---|
| 0 | dtor |
| 1, 2 | parameters, parameters const |
| 3, 4 | dimension, minimalDimension |
| 5, 6 | plus, plusJacobian |
| 7, 8 | minus, liftJacobian |
| 9, 10 | setLocalParameterizationPtr, localParameterizationPtr |
| 11 | typeInfo |
| 12 | setEstimate |
| 13 | estimate (where virtual) |

**GravityParameterBlock** (vtable 0x1803AEBB0, 13 slots)
- +0x20 Vector3d estimate_.
- dim 3, minimal 2.
- plus/minus via GravityLocalParameterization (S² of radius |g|, TangentBasis 0x18001ACE0).
- No virtual estimate().

**General3DParameterBlock** (vtable 0x1803AF330, 14 slots)
- +0x20 Vector3d estimate_, +0x38 bool initialized_; sizeof 0x40.
- dim 3/3, identity plus/minus/Jacobians.
- `estimate()` returns a const ref.

**GroundPlaneError** (vtables 0x1803AF3E0 / 0x1803AF3F8), sizeof 0x70
- Base `SizedCostFunction<3,7,7>`, plus `ErrorInterface` at +0x28.
- +0x30 bool `use_minimal_jacobians_`: a Pimax addition to ErrorInterface; ImuError and ReprojectionError have it at +0x30 too.
- +0x38 normal_0_, +0x50 normal_1_, +0x68 weight_.

**ErrorType.** `kGroundPlaneError = 7` is appended to the upstream enum.

### 2g. ImuError (sizeof 0x18B0)

This layout is from the ImuError part (drafted in imu_error_c03.hpp). The deltas from upstream are:

| offset | member | note |
|---|---|---|
| +0 | `SizedCostFunction<15,7,9,7,9,3>` | |
| +40 | ErrorInterface vptr | |
| +48 | `use_minimal_jacobians_` | |
| +56 | ImuParameters (128 B) | |
| +184 | `std::deque<ImuMeasurement>` | 32-byte elements: double + 6 floats |
| +224 / +232 | t0_ / t1_ | |
| +240 | `std::mutex` | never locked in this code |
| +320 | Delta_q_ | |
| +352 | C_integral_ | |
| +424 | C_doubleintegral_ | |
| +496 | acc_integral_ | |
| +520 | acc_doubleintegral_ | |
| +544 | cross_ | |
| +616 | dalpha_db_g_ | |
| +688 | dv_db_g_ | |
| +760 | dp_db_g_ | |
| +832 | P_delta_ (15x15) | |
| +2632 | speed_and_biases_ref_ | |
| +2704 | redo_ (true) | |
| +2708 | redoCounter_ | |
| +2712 | information_ (15x15) | |
| +4512 | square_root_information_ (15x15) | |

`Sophus::SO3<double,0>` (derived from `SO3exBase<SO3<double,0>>`) has one member, `Eigen::Quaterniond unit_quaternion_` at +0 (32 bytes, coefficients x,y,z,w).

---

## 3. External interfaces (calls out of the chunk)

| address | inferred signature / meaning |
|---|---|
| 0x180014730 | `std::chrono::high_resolution_clock::now()` (hidden return pointer); vk::Timer ctor; also two unused calls around marginalizeOut |
| 0x18001D360 | `Map::parameterBlockExists(uint64_t)` |
| 0x18001D4A0 | `Map::parameterBlockPtr(uint64_t)` -> `shared_ptr<ParameterBlock>` |
| 0x18001D3A0 | `Map::parameterBlockIdOfResidual(ResidualBlockId, size_t idx)` (CHECK "Invalid residual parameter index", ceres_map.cpp:588) |
| 0x18001EA00 | `Map::residuals(uint64_t)` -> ResidualBlockCollection (32-byte ResidualBlockSpec) |
| 0x18001EA40 | `Map::residuals(uint64_t, ResidualBlockCollection&)` (clears first) |
| 0x18001E4C0 | `Map::removeResidualBlock(ResidualBlockId)` -> bool |
| 0x18001E1D0 | `Map::removeParameterBlock(uint64_t)` |
| 0x1800202C0 / 0x180020380 | `Map::setParameterBlockConstant / setParameterBlockVariable(uint64_t)` |
| 0x18001B840 | `Map::addResidualBlock(shared_ptr<CostFunction>, LossFunction*, vector<shared_ptr<ParameterBlock>>&)` |
| 0x18001C370 | `Map::addResidualBlock(cost, loss, x0, x1..x9 = nullptr)` |
| 0x18001A950 -> 0x18001A980 | `Map::<unknown>(vector<vector<uint64_t>>& unused, bool)`: toggles the pending new residual/parameter block lists against Map+0x3E8. Name TODO (draft calls it `applyPendingBlockStates`) |
| 0x180018E90 | `Map::Map(bool flag = false)`; make_shared size 0x590 |
| Map+0x000 / +0x1F0 / +0x3F8 / +0x488 | `options` / `summary` / `problem_` / `id_to_residual_block_multimap_` (`count(id)` read directly by optimize, so the member must be accessible) |
| 0x1801C06B0 | `ceres::Solve(const Solver::Options&, Problem*, Solver::Summary*)` |
| 0x1801BC0D0 | `ceres::Solver::Summary::FullReport()` |
| 0x1800773E0 | `MarginalizationError::MarginalizationError(Map&)` (malloc 0x240 = sizeof 576, EIGEN aligned new) |
| 0x180079D90 | `MarginalizationError::addResidualBlock(ResidualBlockId, bool keep = false)` |
| 0x180080320 | `MarginalizationError::marginalizeOut(const std::vector<uint64_t>&)`: **one argument** (no keep vector) |
| 0x180088D10 | `MarginalizationError::updateErrorComputation()` |
| 0x18007B1F0 | `MarginalizationError::getParameterBlockPtrs(vector<shared_ptr<ParameterBlock>>&)` |
| MargErr+0x20 / +0xB0 / +0xB8 / +0x178 / +0x190 | `num_residuals_` / `H_` (rows +0xB8) / `parameter_block_infos_` (80-byte elements, +0x178..0x180) / `parameter_block_id_2_parameter_block_info_idx_` (unordered_map, isInMarginalizationTerm inlined) |
| 0x18008BAC0 | `PoseError::PoseError(const Transformation&, const Matrix<double,6,6>&)` (make_shared 0x1B0) |
| 0x18008DA10 | `PoseParameterBlock::estimate() const` -> Transformation (non-virtual) |
| PoseParameterBlock vtable +0x60 | `setEstimate(const Transformation&)` (0x18008DB00) |
| SpeedAndBiasParameterBlock vtable +0x60 / +0x68 | `setEstimate` (0x180090110) / `estimate()` (0x18002B7F0) |
| ReprojectionError vtable +0x28 (0x180016B50) | returns `this+0x50`, a Vector2d written by EvaluateWithMinimalJacobians (0x18000C490) = last weighted residual. Name TODO (draft: `lastResidual()`) |
| 0x18008CC60 | `PoseLocalParameterization::liftJacobian(const double*, double*)` |
| 0x18001ACE0 | `TangentBasis(const Vector3d&)` -> MatrixXd 3x2 (c02 name; fork used `GravityLocalParameterization::tangentBasis`) |
| 0x180016490 | `skewSymmetric(const Vector3d&)` |
| 0x180027A20 | `isFinite(const Vector3d&)` (c02) |
| 0x180013040 | `Transformation::inverse()` (minkindr) |
| 0x180020CE0 / 0x180021190 | `operator<<(ostream&, Transformation)` / Eigen `operator<<` for `Matrix<9,1>.transpose()` |
| 0x18009BBE0 | `Point::removeObservation(int frame_id)` (erase from obs_) |
| 0x180025930 | `Estimator::addGroundPlaneError()` (c02) |
| 0x1800245A0 | `States& States::operator=(States&&)` |
| 0x180024750 | `std::map<std::string,double>::operator[]` |
| 0x180024F60 / 0x1800248E0 / 0x180024FE0 / 0x180021F50 | PointMap erase(it) -> next / tree iterator ++ / set extract node / `std::find` over set |
| 0x180354E20 / 0x180354E50 / 0x18035AC70 / 0x1803551B0 | glog `LogMessage(file,line)` / `LogMessage(file,line,severity)` / `stream()` / `~LogMessage` |
| 0x18048EA8C | `FLAGS_v` (glog) |
| 0x18046A000 / 0x18000C2C0 | Logger instance / LOGE |
| 0x18047DA40 | `MarginalizationTiming::names_` |
| callers into this chunk | 0x180013680 (CeresBackendInterface optimization loop): applyMarginalizationStrategy(n, imu_frames+1, &timing), setOldestFrameFixed() if fixed set empty, optimize(n, verbose=+0x50 byte, +0xA0 byte), setPoseEstimateAndZeroVelocity when +0xA0 set, get_T_WS / getSpeedAndBias, updateAllActivePoints. 0x180012B20 (reset): reset() + resetMap(). Frontend 0x1800F5170 / 0x18011B420: setGroundPlaneConstraint / resetGroundPlaneConstraint |

| 0x180022AF0 / 0x180008720 / 0x1800087D0 | Eigen constant-nullary 3x3 / MapBase / Block ctors (SO3 helpers, GroundPlaneError) |
| 0x180016B90 | printf (ensure handler) |

---

## 4. Constants

| where | value | meaning |
|---|---|---|
| optimize | `linear_solver_type = DENSE_SCHUR (3)` (+0xD0) | |
| optimize | `trust_region_strategy_type = DOGLEG (1)` (+0x58) | |
| optimize | `max_num_iterations = num_iter` (+0x68) | |
| optimize | `max_solver_time_in_seconds = 0.025` (+0x70, 0x3F9999999999999A) | |
| optimize | `logging_type` (+0x174) | SILENT when not verbose |
| optimize | `minimizer_progress_to_stdout` (+0x178) | |
| optimize gating | `optimize_count_ <= 10` | no gating for the first 11 calls |
| optimize gating | parallax < 0.017453292519943295 (1°, 0x1803AF2A0); point obs < 2; Point+0x78 < 2 | set constant |
| optimize gating | with >= 50 landmarks: observations > 5; inlier ratio `n_in/(n_tot+1) > 0.9` (0x1803AF2C0); > 20 bundles | set constant |
| optimize update | `|‖p_new‖ - ‖p_old‖| > 1.0` (0x1803ADDD0) | drop landmark if any observing frame's pose exists |
| optimize update | chi2 > 2.5 (0x1803AF2C8) | outlier |
| optimize update | chi2 < 1.0 | inlier count |
| optimize update | survivors/residuals < 0.2f (0x1803ADDC0) | drop landmark |
| setOldestFrameFixed | information diag 1e14 at (0,0),(1,1),(2,2),(5,5) | |
| setGroundPlaneConstraint | norm > 1e-12 (0x1803AF290); sigma clamp [0.05 (0x1803AF2A8), 0.25 (0x1803AF2B0)] | |
| GroundPlaneConstraint | defaults (0,0,1), (0,0,1), sigma 0.15 (0x3FC3333333333333), ids -1 | |
| GroundPlaneError | weight = 1/max(\|sigma\|, 1e-6 (0x1803AF298)) | normals default (0,0,1) |
| MarginalizationTiming names | "0_mag_pre_iterate", "1_mag_collection_non_pose_terms", "2_marg_collect_poses", "3_actual_marginalization", "4_marg_update_errors", "5_finish" | |
| ImuError | redo threshold \|Δb_g\|·dt > 1e-4 (0x1803ADDA0); deltaQ series 1/6, 1/120, 1/5040, threshold 1e-6 | |
| ImuParameters defaults | a_max 150, g_max 35, g 9.80667, rate 1000 | |

| SO3 helpers | see section 6 |

---

## 5. Quirks and bugs to preserve

1. **applyMarginalizationStrategy:**
   - The pose of the newest kept keyframe is fetched for every surplus keyframe. Only "get_T_WS failed\n" (LOGE) is logged; the pose is unused.
   - `deRegisterFixedFrame` erases the result of `std::find` without checking it; the CHECK was removed.
   - The first landmark pass reads residual parameter 0 of every residual, whatever its type.
   - The old marginalization residual is removed after the frame loop. If that removal fails the function returns false and keeps the cleared frame lists.
   - No keep vector is used, so `marginalizeOut` gets only the id list.
2. **optimize:**
   - The 4th argument is ignored.
   - `options.max_solver_time_in_seconds` is overwritten with 0.025 on every call.
   - The gating loop compares `(int)observations.size()` and `(int)set.size()` as signed ints.
   - The ratio `ba_inlier / (ba_total + 1)` is computed in double.
   - The survivor ratio is computed in **float** (`(float)count / (float)n`), then compared as double with 0.2.
   - The squared residual (chi2) is computed even when the frame could not be locked.
   - When a lock race occurs (expired() false, then lock() fails), `frame->...` is dereferenced as null. This matches the binary.
   - The landmark-jump test makes `n_obs` lock()/unlock() round-trips.
3. **getPoseEstimate / getSpeedAndBiasEstimate** dereference a failed `dynamic_pointer_cast` without checking it.
4. **removeObservation** returns `removeResidualBlock`'s result, not `true`.
5. **getTriangulationParallax:**
   - All of its math is in float.
   - The loop starts at the 2nd observation (`std::next(begin)`).
   - It returns 0.0 when there are fewer than 2 observations or the reference frame is gone.
   - The maximum keeps the new value on ties and NaN (`std::max(parallax, max)`).
6. **setGroundPlaneConstraint** divides by norms computed before the validity test; it does not call normalize().
7. **ImuError::EvaluateWithMinimalJacobians:**
   - It never locks the mutex.
   - The rotation residual is the SO(3) log, but the Jacobians are still the 2·vec() forms. They are inconsistent; keep them as they are.
8. **Evaluate of ImuError / PoseError / SpeedAndBiasError** is one ICF-folded function. The `+0x30` flag makes ceres receive the minimal Jacobians.

9. SO3 family-A Jacobians are mathematically inexact; reproduce them literally (section 6).

---

## 6. SO(3) / Sophus-variant helpers (imu_error support code inside this range)

Source of the formulas: see the header comment of `draft/c03_estimator/common/so3_gamma.h`. Notation: θ=|φ|, u=φ/θ, K=hat(u).

- **Family B** (0x180035AD0, 0x180030480, 0x180032F70) is the true derivative ∂(n!·Γₙ(φ)·v)/∂φ; checked against finite differences.
- **Family A** (0x18002EF20, 0x1800319C0, 0x1800344E0) is not an exact derivative of n!Γₙ(φ)ᵀv. It is probably a Pimax derivation error and is reproduced literally from the asm.
- **Small-angle branches** use the alternating tgamma series 0x1800409E0 (tolerance DBL_EPSILON, default x2 = NaN at 0x1803AF8E8). They agree with the closed forms to 1e-9 at θ = 0.08 and 0.3.
- **eps per caller:** 1e-5 (0x1803AF4F8), 0.001, 0.02, 0.06, 0.1 (0x1803AF500..518).
- **Sophus constants:** epsilon 1e-10 (0x1803AF870); log uses n² < 1e-20 (0x1803AF860).
- **Literals checked with `rd`:** 2/3, -1, 2, 3, 4, 5, 6, 8, 12, 15, 24.
- **Two `__FILE__` spellings** exist for so3ex_base.h: `src\common\sophus\so3ex_base.h` (used here) and `src\common/sophus/so3ex_base.h` (0x18010A400, another TU).
- **Ensure lines** are normalize 125 and log 109; the draft forces them with `#line`.
- **Family A per-n formulas.** Family A writes the identity first, then J = e2·hat(v) + (e4K + e3K²)(u·v)I + e1(u vᵀ − 2(u·v)I + v uᵀ). The per-n e1..e4 are in the header.
- **NaN.** All branch tests are `θ < eps`, so NaN takes the closed-form branch.
- **dGammaV1 series order.** It evaluates the series in a different order: S(3,0), S(4,1), S(2,0), S(5,2).
- **Formatter.** It parses precision digits from spec[1] even without a '.'.


---

## 7. Open questions

- The real names of:
  - `optimize_count_`;
  - `setPoseEstimateAndZeroVelocity`;
  - Map 0x18001A950;
  - the ReprojectionError +0x50 accessor;
  - Point +0x60..+0x78;
  - Frame +0xB0.
- Whether Frame::pos() returns Vector3f (0x18002B960).
- The exact boundary between estimator.obj and imu_error.obj (0x18002E0A0 vs 0x1800398A0).
- GravityParameterBlock: its ctor and size (c02 make_shared) and whether `estimate()` exists.
