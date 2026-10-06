# c01_backend_interface — 0x180010440 … 0x18001B4D0

Source: `src/ceres_backend/ceres_backend_interface.cpp` (`pimax::totem::CeresBackendInterface`), plus header-inline
and template code first emitted by that object. The tail of the range (from 0x180018E90 on) is probably the
beginning of `ceres_map.obj`: `Map::Map(bool)`, the Ceres Options/Summary ctors it inlines, and the
local-parameterization virtuals referenced by its vtables. `ceres_map.cpp`'s own `__FILE__` first shows up at
0x18001B4D0 (`Map::addParameterBlock`).

Drafts in `draft/c01_backend_interface/ceres_backend/`:
- `ceres_backend_interface.h`: class, the two options structs, full field layout.
- `ceres_backend_interface.cpp`: every interface method in the range. The ctor and dtor are also included,
  marked as outside the range. **glog `__LINE__`s match the binary (281, 410, 494–500, 509, 613);** blank
  padding holds them in place, so do not reflow the file.
- `estimator_inline_c01.hpp`: `Estimator::addObservation` (0x180010C20), `isPointInEstimator` (0x180013420),
  inline `setKeyframe`, and pimax `createNFrameId` (no CHECK).
- `ceres_map_c01.cpp`: `Map::Map(bool)` (0x180018E90), `Map::applyFlag` (0x18001A980) and `Map::setFlag`
  (0x18001A950). These belong in `ceres_map.cpp`.
- `inline_bodies_c01.hpp`: `GravityLocalParameterization` with `TangentBasis`, the Pose/HomogeneousPoint
  GlobalSize/LocalSize bodies, ReprojectionError/ErrorInterface accessors, `crossMx`, and
  FrameBundle::set_T_W_B / Frame::set_T_w_imu. To be merged by the owners of those headers.

## Overall
Upstream `svo::CeresBackendInterface` heavily rewritten:
- No thread, no condition variable, no `optimizationLoop()`. FrameProcessor calls
  `loadMapFromBundleAdjustment` → `bundleAdjustment` → `optimize` synchronously.
- No `AbstractBundleAdjustment` base and **no vtable**. `type_` (= 2, kCeres) became a plain member at +976.
- Removed: MotionDetector, publisher, last_state_, timers_, lock_to_fixed_landmarks_/global landmark version
  logic, fixed-landmark sorting, getLatestSpeedBiasPose, setReinitStartValues, getAllActiveKeyframes,
  startThread/quitThread.
- Added: second frame window `active_frames_` (non-KF bundles); `obs_count_map_` (track id → count) limiting
  observations per bundle to 80/100 (KF path: 150 landmarks); reprojection-consistency gate `|w2c(lm) −
  f2c(f)| > 4 px` when |v| < 0.02 m/s; velocity priors (σ = 0.001) when stationary or |v| < 0.005; a
  stepwise world correction (`correction_steps_` of 1 mm); adaptive marginalization window and iteration count;
  pose pinning of the backend to the frontend while stationary.

## Function table

Legend: kind `project` = reconstructed; `lib:*` = not reconstructed. "ICF" = identical-COMDAT folded body shared
with unrelated functions.

| address | proposed name | signature | kind | upstream | summary |
|---|---|---|---|---|---|
| 180010440 | `CeresBackendInterface::addLandmarksAndObservationsToBackend` | `void(const FramePtr&, double speed /*xmm2*/)` | project | modified: new speed param; level>1 / track −1 / obs-count (<2, or >2 with Point+120<2) / reprojection gates; re-enabled parallax gate; per-bundle `obs_count_map_` (skip if >1); break when map size >150; no fixed-landmark handling; LOG(WARNING) line 410; VLOG(6) lines 494–500 | Adds new landmarks + observations of a keyframe |
| 180010C20 | `Estimator::addObservation` (estimator_impl.hpp inline) | `ceres::ResidualBlockId(const FramePtr&, size_t kp, const BackendId& nframe_id)` | project | modified: see estimator_inline_c01.hpp (measurement = w2c(landmark), cam index stored, no findSlot / fixed / temporal extrinsics) | |
| 180011170 | `CeresBackendInterface::addStatesAndInertialMeasurementsToBackend` | `bool(const FrameBundlePtr&, bool imu_stationary, const Eigen::Vector3d& gravity_prior, bool flag)` | project | modified: IMU taken from FrameBundle+24, no waitTill/getMeasurements; new Estimator::addStates signature; Vector3f result → FrameBundle+216; LOG(ERROR) line 509 | |
| 180011350 | — | `void*(size_t)` | lib:Eigen `aligned_malloc` (malloc + 16-B assert + bad_alloc) | - | |
| 1800113B0 | — | | lib:std `_Allocate<16>` (1-byte elements, `allocator<char>`) | - | |
| 180011410 | — | | lib:std allocator::allocate, 40-B elements (`map<int,int>` node) | - | |
| 180011490 | — | | lib:std allocator::allocate, 32-B elements | - | |
| 180011500 | — | | lib:std allocator::allocate, 16-B elements (shared_ptr) | - | |
| 180011570 | — | | lib:std `string::assign(const char*, size_t)` | - | |
| 1800116D0 | — | | lib:std `vector<FramePtr>::at` (FrameBundle::at out-of-line) | - | |
| 180011700 | `CeresBackendInterface::bundleAdjustment` | `void(const FrameBundlePtr&, int /*unused*/, uint8_t tracking_mode)` | project | modified (heavily): no stop_thread/mutex/motion detector; velocity priors; KF vs non-KF paths with obs budget 80 (100 for mode 1/2); `active_keyframes_`/`active_frames_` push; VLOG(10) line 281; skip_optimization flags | |
| 180012AB0 | — | | lib:std `map<int,int>::clear` | - | |
| 180012B20 | `CeresBackendInterface::clearBackend` (name unknown) | `void()` | project | new | Estimator resets, clears windows and map, flags, counter |
| 180012B90 | — | | lib:std `string::compare` | - | |
| 180012C00 | `ReprojectionError::covariance()` | `const Matrix2d&() const` | project (inline, ICF-free) | identical | returns this+192 |
| 180012C10 | — | | lib:std `allocator<char>::deallocate` | - | |
| 180012C50 / 180012CA0 | — | | lib:std deallocate (32-B / 16-B elements) | - | |
| 180012CF0 | — | | lib:std `deque<FramePtr>::erase(first,last)` | - | |
| 180013020 | ReprojectionError vslot 6 | `const T&() const` | project (inline) | new | returns this+96. TODO(verify) member |
| 180013030 | `ReprojectionError::information()` | | project (inline) | identical | returns this+128 |
| 180013040 | — | | lib:minkindr `QuatTransformation::inverse()` (normalizes result) | - | |
| 1800132A0 | — | | lib:minkindr/Eigen `RotationQuaternion::inverse()` | - | |
| 180013420 | `Estimator::isPointInEstimator` | `bool(int) const` | project (inline) | identical | landmarks_map_.find(createLandmarkId(id)) |
| 180013480 | `CeresBackendInterface::loadMapFromBundleAdjustment` | `void(const FrameBundlePtr& new, const FrameBundlePtr& last, const MapPtr&, bool& have_motion_prior, const Vector3d& gravity_prior, bool flag)` | project | modified: stationary flag from FrameBundle+228; motion prior only if `!imu_init_pending_`; speed not rotated; map/frame update moved to optimize(); LOGE instead of LOG(ERROR) | |
| 180013680 | `CeresBackendInterface::optimize` | `void(const FrameBundlePtr&, int frame_count, uint8_t tracking_mode)` | project | modified: body of upstream optimizationLoop + map update; see below | |
| 180014670 | `ReprojectionError::measurement()` | | project (inline) | identical | this+64 |
| 180014680 | — | | lib:Eigen `Quaterniond::normalize()` | - | |
| 1800146D0 | — | | lib:minkindr `RotationQuaternion::normalize()` (returns *this) | - | |
| 180014730 | — | | lib:std `chrono::steady_clock::now` | - | |
| 1800147A0 | — | | lib:std `basic_stringbuf::overflow` | - | |
| 180014950 | `ErrorInterface::parameterBlockDim` (this−32 thunk body, ICF) | `size_t(size_t) const` | project (inline) | identical | parameter_block_sizes().at(i) |
| 180014980 | `ErrorInterface::parameterBlocks` (ICF) | `size_t() const` | project (inline) | identical | parameter_block_sizes().size() |
| 180014990 | — | | lib:std `basic_stringbuf::pbackfail` | - | |
| 1800149F0 | `CeresBackendInterface::reset` | `void()` | project | modified: only `LOGI("Backend: Reset\n")` | |
| 180014A10 | `residualDim()` = 2 / `GravityLocalParameterization::LocalSize()` = 2 | | project (inline, ICF with boost `codecvt_null<wchar_t>::do_encoding`) | - | return 2 |
| 180014A20 | — | | lib:Eigen `Quaterniond * Vector3d` (rotate) | - | |
| 180014B50 | — | | lib:Eigen triangular solve / blocked kernel (BlasUtil incr==1) | - | |
| 180015620 / 180015780 | — | | lib:std `basic_stringbuf::seekoff` / `seekpos` | - | |
| 180015870 | `CeresBackendInterface::setCorrectionInWorld` | `void(const Transformation&)` | project | modified: steps = ceil(|t|/0.001); stores t/steps; accumulates the FULL correction into every active keyframe | |
| 180015A30 | — | | lib:minkindr `QuatTransformation::setIdentity` (called by FrameProcessor on its own member) | - | |
| 180015A80 | `CeresBackendInterface::setImu` | `void(const std::shared_ptr<ImuProcessor>&)` | project | modified: const ref; adds g0 (gyro bias); no rate/delay assignment | |
| 180015C30 | `ReprojectionError::setInformation` | | project (inline) | identical | info, inverse → covariance, LLT → sqrt info |
| 180015DB0 | `ReprojectionError::setMeasurement` | | project (inline) | identical | |
| 180015DC0 | `CeresBackendInterface::setPerformanceMonitor` | `void(const std::string&)` | project | identical (loop by reference) | |
| 1800162B0 | `FrameBundle::set_T_W_B` | `void(const Transformation&)` | project (frame.h inline) | modified (normalize) | |
| 180016490 | `crossMx` (skew matrix) | `Matrix3d(const MatrixBase&)` | project template (kinematics hdr) | - | used by error terms 0x18000C490/0x18000E0A0/0x18002D390/0x18003AE40 |
| 180016530 | — | | lib:std `basic_stringbuf::str()` | - | |
| 1800165E0 | — | | lib:CRT `_Throw_bad_alloc` (named cancel_current_task) | - | |
| 180016600 | — | | lib:Eigen `Quaterniond::toRotationMatrix` | - | |
| 180016740 | `typeInfo()` = kReprojectionError (1) (ICF "return true") | | project (inline, ICF with codecvt/ceres) | identical | |
| 180016750 | — | | lib:std `basic_stringbuf::underflow` | - | |
| 1800167C0 | `CeresBackendInterface::updateActiveKeyframes` | `void()` | project | modified: erase(remove_if) of frames of `backend_.marginalized_nframe_ids_.front()` from both windows | |
| 180016B50 | ReprojectionError vslot 5 | | project (inline) | new | returns this+80. TODO(verify) |
| 180016B60 | — | | lib:CRT `exception::what` | - | |
| 180016B80 | — | | lib:CRT `__local_stdio_printf_options` | - | |
| 180016B90 | — | | lib:CRT `printf` (inline UCRT) | - | |
| 180016BF0 | — | | lib:CRT `snprintf`-style (`__stdio_common_vsprintf`, options|2) | - | |
| 180016C50 | — | | lib:std `operator<<(ostream&, const string&)` | - | |
| 180016C70 | — | | lib:std `_Destroy_range<ParameterBlockSpec>` (24 B, shared_ptr at +8) | - | |
| 180016CF0 | — | | lib:std `_Destroy_range<ResidualBlockSpec>` (32 B) | - | |
| 180016D70 | — | | lib:std `vector<uint64_t>::_Emplace_reallocate` | - | |
| 180016F10 | — | | lib:std `vector<shared_ptr<T>>::_Emplace_reallocate` | - | |
| 180017080 | — | | lib:std `vector<ResidualBlockSpec>::_Emplace_reallocate` | - | |
| 180017280 | — | | lib:std `vector<8-byte>::_Emplace_reallocate` | - | |
| 180017380 | — | | lib:std `vector<ParameterBlockSpec>::_Emplace_reallocate` | - | |
| 180017510 / 1800175A0 | — | | lib:std unordered_map node-list free (`_Free_non_head`) | - | |
| 180017630 | — | | lib:std `_Insert_string` (ostream) | - | |
| 1800177F0 / 180017870 | — | | lib:std `_Uninitialized_move` (24-B / 32-B specs) | - | |
| 1800178F0 | — | | lib:Eigen `conditional_aligned_new_auto<double>` | - | |
| 180017980 / 180017C30 / 180017F50 / 1800181E0 | — | | lib:std `unordered_map/multimap<uint64_t,...>` emplace/try_emplace/insert (FNV-1a) used by Map | - | |
| 180018500 | — | | lib:std `endl` | - | |
| 180018540 | — | | lib:Eigen `evaluateProductBlockingSizesHeuristic` | - | |
| 180018920 | — | | lib:std unordered_(multi)map find | - | |
| 1800189F0 / 180018A80 / 180018B10 | — | | lib:Eigen CwiseNullaryOp / MapBase ctors | - | |
| 180018BA0 | — | | lib:std `vector<ParameterBlockSpec>` copy ctor | - | |
| 180018C90 | — | | lib:Eigen `queryCacheSizes` (cpuid) | - | |
| 180018E90 | `ceres_backend::Map::Map(bool flag)` | | project (really ceres_map.cpp) | modified: flag, disable_all_safety_checks=true, gravity parameterization, two new-id vectors | |
| 180019230 | — | | lib:ceres `Solver::Options::Options()` (implicit, inline) | - | |
| 1800195D0 | — | | lib:ceres `Solver::Summary::Summary()` (implicit, inline) | - | |
| 180019800 … 18001A0A0 | — | | lib: small destructors/EH helpers (DenseStorage free, vector/list/unordered_map/shared_ptr member dtors, `unique_ptr<ceres::Problem>`, thunks 180019E30/40/50/60/ED0, 18001A0A0 → ~LocalParameterization) | - | |
| 18001A0B0 | — | | lib:ceres `Solver::Summary::~Summary` | - | |
| 18001A260 | — | | lib:std shared_ptr member release (+24) | - | |
| 18001A2B0 | — | | lib:ceres `Solver::Options::~Options` | - | |
| 18001A538 / 18001A544 | — | | compiler thunks (this−8) → deleting dtors | - | |
| 18001A550 | `HomogeneousPointLocalParameterization` / `GravityLocalParameterization` deleting dtor (ICF) | | project (inline) | - | EIGEN aligned delete |
| 18001A5A0 | — | | lib:ceres `LocalParameterization` deleting dtor | - | |
| 18001A5E0 | `PoseLocalParameterization` deleting dtor | | project (inline) | - | |
| 18001A620 | `GravityLocalParameterization::ComputeJacobian` | | project (pimax-new hdr) | new | J(3×2,row-major) = TangentBasis(x) |
| 18001A780 | `GravityLocalParameterization::ComputeLiftJacobian` | | project | new | J(2×3) = TangentBasis(x)ᵀ |
| 18001A8E0 | `GravityLocalParameterization::GlobalSize` | | project | new | flag_ ? 0 : 3 |
| 18001A8F0 | `HomogeneousPointLocalParameterization::GlobalSize` | | project | modified | flag_ ? 0 : 4 |
| 18001A900 | `PoseLocalParameterization::GlobalSize` | | project | modified | flag_ ? 0 : 7 |
| 18001A910 | `HomogeneousPoint…::LocalSize` = 3 (ICF; also General3DParameterBlock / GroundPlaneError / InitGravityLocalParameter) | | project | identical | |
| 18001A920 | `PoseLocalParameterization::LocalSize` = 6 (ICF; also PoseParameterBlock::minimalDimension, BEBLID) | | project | identical | |
| 18001A930 | `GravityLocalParameterization::Minus` | | project | new | → static minus 0x18001CF70 |
| 18001A950 | `Map::setFlag(const vector&, bool)` | | project | new | → applyFlag |
| 18001A960 | `GravityLocalParameterization::Plus` | | project | new | → static plus 0x18001D5D0 |
| 18001A980 | `Map::applyFlag(bool)` | | project | new | propagate flag to parameterizations and error terms |
| 18001ACE0 | `TangentBasis(Vector3d&)` | `MatrixXd` | project (VINS-Mono) | VINS verbatim | |
| 18001AF50 / 18001AFE0 / 18001B0A0 | — | | lib:std vector `_Change_array` (8-B / 24-B / 16-B) | - | |
| 18001B140 / 18001B1B0 | — | | lib:std `_Ref_count_resource<ceres::Problem*, default_delete>` _Destroy / _Get_deleter | - | |
| 18001B170 / 18001B190 | — | | lib:std `_Destroy_range` wrappers | - | |
| 18001B1E0 / 18001B250 / 18001B2E0 | — | | lib:std `vector::_Reallocate_exactly` (reserve) for uint64 / ParameterBlockSpec / shared_ptr | - | |
| 18001B390 | — | | lib:std `unordered_map::_Desired_grow_bucket_count` | - | |
| 18001B440 | — | | lib:std `vector<ParameterBlockSpec>::_Tidy` | - | |

Outside the range but reconstructed in the .cpp for completeness: ctor **0x180008CD0** and dtor **0x180009950**.
Factory **0x180158F70** (`interface/ceres_backend_factory.cpp`) builds the object with hard-coded options; the
`make_shared` control block is `_Ref_count_obj2<CeresBackendInterface>` (vtable 0x1803B7A80).

### optimize() 0x180013680 in detail
1. Return if both windows are empty. Then `++num_optimizations_`, `t0 = system_clock::now()`, `vk::Timer`, and
   `MarginalizationTiming`.
2. If `marginalize`: `nkf = num_keyframes − (frame_count==2 ? 2 : 4)`, or `8` when `tracking_mode==1`.
   Then `applyMarginalizationStrategy(nkf, num_imu_frames+1, &timing)` (LOG(ERROR) line 613 on failure) and
   `updateActiveKeyframes()`.
3. Performance-monitor logs "marginalization" plus the MarginalizationTiming names, and LOGD
   `vi-estimator marginalization %f ms`.
4. `if (!hasFixedPose()) setOldestFrameFixed()`, then log "fixation".
5. If `skip_optimization_once_ || imu_init_pending_`, clear the skip flag. Otherwise
   `backend_.optimize(n, verbose, imu_stationary)` with n = 5 for the first 30 calls, then `num_iterations`
   (−2 if frame_count > 8). LOGD ceres_time.
6. Log tot_time (s) and writeToFile, then LOGI `vi-estimator total time %.2f ms` (system_clock based).
7. Not stationary: for the first frame only, `get_T_WS` (normalized) and `getSpeedAndBias` (LOGE on failure).
   Write the IMU biases to ImuProcessor (+208 acc, +232 gyro). Then `FrameBundle::setIMUState(v, bg, ba)`;
   if |ba| ≥ 1.0 the biases are zeroed instead. If successful, `at(0)->set_T_w_imu`.
   Stationary: `backend_.set_T_WS(id(bundle), at(0)->T_world_imu())`.
8. Pose update of `active_keyframes_` then `active_frames_`. `get_T_WS` is called only when the bundle id
   changes; `last_bundle_id` is shared by both loops and starts at 0. A failed get_T_WS skips the keyframe in
   loop 1, but loop 2 only LOGEs and still applies the stale pose.
9. `updateAllActivePoints()`, `num_outliers_removed_ = 0`, outlier rejection per frame (int count
   accumulated), LOGD. Deleted points are no longer removed from the backend.

### bundleAdjustment() 0x180011700 in detail
- `max = (mode==1||mode==2) ? 100 : 80`, `speed = |FrameBundle.imu_vel_w_|`.
- Velocity priors:
  - Stationary: zero velocity, σ 0.001. Success sets `skip_optimization_once_ = true`, which inverts the
    upstream behaviour.
  - Otherwise, if speed < 0.005: prior at the bundle's own velocity. Return value ignored.
- Copy `obs_count_map_` to `last`, then clear it. `nframe_id` comes from `at(0)->bundleId()`.
- Keyframe bundle: `setKeyframe(nframe_id)`. Each frame is pushed to `active_keyframes_`. Then either:
  - "continue-only" mode, when `last.size() > max && (stationary || (at(0)->near_map_kf_ && numLandmarks() >
    max))`: observations only for points that are in the estimator, absent from the current map and present in
    `last`; or
  - otherwise, `addLandmarksAndObservationsToBackend(frame, speed)`.
- Non-keyframe bundle: push to `active_frames_` and collect candidates. Candidates pass:
  - level ≤ 1 and a valid track id;
  - when (stationary || near_map_kf && numLandmarks > max), Point+120 ≥ 2 (no null check);
  - the 4 px gate;
  - obs > 1 and isPointInEstimator.

  Candidates are split into those in `last` and the rest, the `last` ones first. For each: skip if already in
  the current map and in the estimator; else addObservation, ++count, insert; stop when count > max.
- VLOG(10) line 281, the "Too few visual measurements" check (LOGW), `last_added_nframe_images_`, and
  `last_added_frame_stamp_ns_`.

## Types

### CeresBackendInterface (sizeof 1264 = 0x4F0; no vtable)
| off | type | name | evidence |
|---|---|---|---|
| 0 | CeresBackendInterfaceOptions (64) | options_ | ctor copies 4×16 B (sure) |
| 64 | CeresBackendOptions (56) | optimizer_options_ | ctor 3×16+8 (sure) |
| 120 | std::deque<FramePtr> (40) | active_keyframes_ | KF push, optimize loop 1, setCorrection (sure) |
| 160 | bool | imu_motion_detector_stationary_ | = FrameBundle+228 (sure) |
| 164 | int | num_outliers_removed_ | optimize (sure; name guess) |
| 176 | Estimator (768) | backend_ | ctor 0x180023430 / dtor 0x1800239A0 (sure) |
| 944 | shared_ptr<ImuProcessor> | imu_handler_ | setImu (sure) |
| 960 | size_t | no_motion_counter_ | zeroed, never used (guess) |
| 968 | unique_ptr<OutlierRejection> | outlier_rejection_ | ctor malloc(8)+threshold, dtor free (sure) |
| 976 | int | type_ | 0 then 2 in ctor (sure) |
| 980 | int (BundleId) | last_added_nframe_imu_ | -1 (sure) |
| 984 | int | last_added_nframe_images_ | -1 (sure) |
| 992 | int64 | last_added_frame_stamp_ns_ | (sure) |
| 1000 | bool | skip_optimization_once_ | (sure) |
| 1008 | std::deque<FramePtr> | active_frames_ | non-KF push, optimize loop 2 (sure; name guess) |
| 1048 | std::map<int,int> | obs_count_map_ | node 40 B, key +28 value +32 (sure; name guess) |
| 1064 | std::mutex (80) | w_T_correction_mut_ | Mtx_init/lock (sure) |
| 1152 | Transformation (56, 16-aligned) | w_T_correction_to_apply_ | (sure) |
| 1216 | bool | is_w_T_valid_ | (sure) |
| 1220 | int | num_optimizations_ | ++ in optimize, >30 check (sure) |
| 1224 | shared_ptr<vk::PerformanceMonitor> | g_permon_backend_ | (sure) |
| 1240 | bool | unknown_flag_1240_ | only cleared (ctor/clearBackend) |
| 1241 | bool | imu_init_pending_ | FrameProcessor 0x1800FB287 copies its +0xD80 ("imu_initial true" clears it) |
| 1244 | int | correction_steps_ | =1 (sure) |
| 1248 | int | correction_step_idx_ | =0 (sure) |
| 1256 | double | correction_step_size_ | 0.001 (sure) |

CeresBackendInterfaceOptions and CeresBackendOptions layouts/values: see header (factory constants verified:
2, 2°=0x3FA1DF46A2529D39, 0x100, 5, 1, 2.75, 5, 1 / -1.0, 5, 1, 0x100, 8, 1, 0x101, 10).

### Other project types seen (offsets used by this chunk)
- **Frame** (has a vtable at +0; make_shared size 0x340):
  - +16 `id_` (frame_counter_++); +20 `cam_id_` (ctor arg = index in bundle; pimax); +32 `bundle_id_`;
    +36 `nframe_index_`; +40 `cam_`.
  - +64 `T_f_w_`; +234 bool `near_map_kf_` (set when |Δt|<0.05 && angle<0.3 in 0x1800B2F80; name guess);
    +240 `timestamp_` (ns); +256 `T_body_cam_`.
  - +552 `num_features_`; +576/+584 `f_vec_` (Matrix<float,3,Dynamic>: data, cols); +624/+632 `level_vec_`
    (VectorXi); +680 `landmark_vec_` (vector<PointPtr>); +704/+712 `track_id_vec_` (VectorXi);
    +768 `accumulated_w_T_correction_`.
  - Methods: `w2c` 0x180095450 and `f2c` 0x1800955C0 (both take an optional out-pointer, null here);
    KeypointIdentifier ctor 0x180099F40; getTriangulationParallax 0x18009AA50 (a Point method).
- **FrameBundle**:
  - +0 `frames_`; +24 IMU measurements (passed to addStates); +144 `imu_vel_w_`; +168 `imu_gyr_bias_`;
    +192 `imu_acc_bias_`.
  - +216 Vector3f (addStates output); +228 bool `imu_stationary_`; +248 bool `is_keyframe_`; +252 `bundle_id_`.
  - `numLandmarksInBA` 0x180095240.
  - `getMinTimestampNanoseconds` = `frames_[0]->timestamp_` with no CHECK; `getMinTimestampSeconds` = ns·1e-9;
    `Frame::getTimestampSec` = ns/1e9.
- **Point**: +0 `id_`; +4 `pos_` (Vector3f!); +24/+32 `obs_` (list; size at +32); +120 int (`n_succeeded_reproj_`?
  TODO(verify)); +128 bool `in_ba_graph_`.
- **KeypointIdentifier** (32 B): weak_ptr<Frame> +0, int frame_id +16, int bundle_id +20 (pimax), size_t kp +24.
- **Estimator** (offsets relative to backend_ = interface+176):
  - +184 `min_num_3d_points_for_fixation_`; +192 `marginalized_nframe_ids_` (vector<BackendId>; name guess).
  - +264 `landmarks_map_` (std::map<uint64,MapPoint>, size at +272); +280 `camera_rig_` (raw .get());
    +296 `constant_extrinsics_ids_`.
  - +320 `states_` (ids +320, is_keyframe vector<bool> +344 with size +368, timestamps +376).
  - +400 `map_ptr_` (shared_ptr<Map>); +584 `cauchy_loss_function_ptr_`; +744 fixed-frame-ids set size
    (⇒ hasFixedPose).
  - MapPoint: observations at MapPoint+56, an `unordered_map<uint64_t residual, KeypointIdentifier>`
    (landmarks_map_ node: key +32, value +40, observations +96).
- **ReprojectionError**:
  - +0 CostFunction; +40 ErrorInterface vptr; +48 ErrorInterface::flag_; +56 int64 `cam_index_` (pimax).
  - +64 measurement_; +80/+96 pimax extras; +128 information_; +160 sqrt-info; +192 covariance_.
  - make_shared block 0x100.
- **ImuProcessor**: imu_calib_ at +0 (upstream ImuCalibration layout); imu_init_ at +112;
  +208 acc bias; +232 gyro bias (no mutex in the getters/setters).
- **ImuParameters** (128 B): 9 doubles (a_max … g), a0 +72, g0 +96 (new), rate +120 = 1000.0 default
  (no delay_imu_cam).
- **ceres_backend::Map**: layout in ceres_map_c01.cpp. LocalParameterizations are 24 B
  (`{vptr, vptr_additional, bool flag_}`).

## External interfaces (called from c01)
| address | meaning |
|---|---|
| 0x180023430 / 0x1800239A0 | Estimator ctor / dtor (from interface ctor/dtor) |
| 0x180025810 | Estimator::addCameraBundle(camera_bundle) (ctor) |
| 0x180025BE0 | Estimator::addImu(const ImuParameters&) |
| 0x180025C80 | Estimator::addLandmark(const PointPtr&) → bool |
| 0x180026100 | Estimator::addStates(FrameBundlePtr (by value), imu meas (FB+24), const double& t, Vector3f& out, bool imu_init_pending, bool stationary, const Vector3d& gravity_prior, bool flag) → bool |
| 0x180027470 | Estimator::addVelocityPrior(BackendId, const Vector3d& v, double sigma, const Vector3d& acc_bias, const Vector3d& gyr_bias) → bool |
| 0x180027AE0 | Estimator::applyMarginalizationStrategy(size_t nkf, size_t nimu, MarginalizationTiming*) → bool |
| 0x180029610 | Estimator::resetStates() (name guess) |
| 0x180029F30 | Estimator::getSpeedAndBias(BackendId, SpeedAndBias&) → bool |
| 0x18002A4F0 | Estimator::get_T_WS(BackendId, Transformation&) → bool |
| 0x18002A6D0 | Estimator::optimize(int iters, bool verbose, bool imu_stationary) |
| 0x18002C350 | Estimator::resetMap() (make_shared<Map>(false) + state reset; name guess) |
| 0x18002C7B0 | Estimator::setOldestFrameFixed() |
| 0x18002CC70 | Estimator::set_T_WS(const BackendId&, const Transformation&) |
| 0x18002CF00 | Estimator::updateAllActivePoints() |
| 0x18001C370 | Map::addResidualBlock(shared_ptr<CostFunction>, LossFunction*, x0..x9 shared_ptr<ParameterBlock>) |
| 0x18001D4A0 | Map::parameterBlockPtr(uint64) |
| 0x18001CF70 / 0x18001D5D0 | GravityLocalParameterization static minus / plus |
| 0x180008F50 | ReprojectionError ctor(CameraConstPtr, const Vector2d&, const Matrix2d&) |
| 0x1801B41A0 | NCamera::getCameraShared(size_t) |
| 0x180095450 / 0x1800955C0 | Frame::w2c(const Vector3d&, Vector2d* = nullptr) / Frame::f2c(const Vector3d&, …) |
| 0x180095240 | FrameBundle::numLandmarksInBA() |
| 0x180099F40 | KeypointIdentifier(const FramePtr&, size_t) |
| 0x18009AA50 | Point::getTriangulationParallax() |
| 0x18008A7C0 | OutlierRejection::removeOutliers(Frame&, size_t& edges, size_t& corners, vector<int>& deleted, int& n) |
| 0x1801B4970 / 4EF0 / 5080 / 55F0 / 58E0 | vk::PerformanceMonitor ctor / addLog / init(name, dir) / log(name, double) / writeToFile |
| 0x180009B80, 0x180009C80, 0x180009B30, 0x180008930, 0x180013040 | minkindr Transformation `*`, quaternion `*`, assign, setIdentity (default ctor), inverse |
| 0x1800066A0, 0x180006A40, 0x180006BE0, 0x1800075A0, 0x18000FE80, 0x18000FCA0, 0x180010100, 0x180010200, 0x1800068A0, 0x180009340, 0x1800067A0, 0x180007120 | std map/deque/vector/unordered_map helpers (c00 range) |
| 0x180354E20 / 0x180354E50 / 0x18035AC70 / 0x1803551B0 / 0x180006290 | glog LogMessage(file,line) / (file,line,sev) / stream() / ~LogMessage; ostream<<const char* |
| 0x18000C120 / 0x18000F500 / 0x18000F6A0 / 0x18000C2C0 | LOGD / LOGI / LOGW / LOGE |

Globals: `0x18046A000` logger; `env_8` @0x18048EA8C = `FLAGS_v`; `Buf1` @0x18047DA40 =
`MarginalizationTiming::names_` (vector<string>); `xmmword_18047ED98` = the global Vector3d that FrameProcessor
passes as gravity_prior; constants: 0.02 @0x1803ADDB8, 4.0 @0x1803ADDE0, 0.001 @0x1803ADDA8,
0.005 @0x1803ADDB0, 1e9 @0x1803ADDF8, 1e-9 @0x1803ADD98, 1e7 @0x1803ADDF0, 1000.0 @0x1803ADDE8,
1.0 @0x1803ADDD0, Identity2 columns @0x1803ADE30/0x1803ADE80.

## Constants / config
- Observation budgets: 80, or 100 for tracking_mode 1/2; KF landmark addition stops once obs_count_map_.size() > 150.
- Reprojection gate 4.0 px, applied only when speed < 0.02 m/s. Velocity prior σ = 0.001, below 0.005 m/s or when stationary.
- Marginalization window `num_keyframes−2` (frame_count==2) / `−4`, or 8 in mode 1, plus `num_imu_frames+1`.
- Iterations: 5 for the first 30 optimizations; then `num_iterations` (factory 5), −2 if frame_count > 8.
- Acc-bias plausibility: |ba| < 1.0, otherwise both biases are zeroed in the FrameBundle state.
- Correction step 0.001 m per step.
- Map: Problem::Options all DO_NOT_TAKE_OWNERSHIP except manifold; disable_all_safety_checks = true.

## Quirks to preserve
1. obs_count_map_ values start at `Frame::cam_id_` (0/1), not 1. The "skip if > 1" test therefore lets a
   landmark first seen by cam 0 be observed 2 more times, but one first seen by cam 1 only once more.
2. addObservation uses the projected landmark, not the measured keypoint, as the measurement.
3. Missing null checks: on `landmark_vec_[kp]` in addLandmarks, and in the non-KF candidate gate when the
   Point+120 test applies.
4. loadMapFromBundleAdjustment uses an uninitialized SpeedAndBias if getSpeedAndBias fails.
5. Stationary prior success sets `skip_optimization_once_ = true`.
6. setCorrectionInWorld accumulates the full correction into keyframes, but stores only the per-step one.
7. optimize loop 2 applies a stale T_WS after a failed get_T_WS. `last_bundle_id` starts at 0 and carries over
   between the two loops.
8. LocalParameterization::GlobalSize() returns 0 once Map::applyFlag(true) has run (before every Solve).
9. VLOG "Skipped because less than … observations" and "not corner" always print 0.

## Open questions
- Names: clearBackend, near_map_kf_, cam_id_, unknown_flag_1240_, the Map flag (`applyFlag`/`setFlag`),
  ReprojectionError vslots 5/6 (+80/+96), Point+120.
- `tracking_mode` (FrameProcessor+464, uint8) values 1/2 meaning; `frame_count` (FrameProcessor+3112).
- MarginalizationTiming ctor (estimator.hpp) must iterate `names_` by const reference (no string copies); also
  `named_timing_[k]`/`emplace(k,0.0)` both fit.
- Whether `T_WS.getRotation().normalize()` vs `toImplementation().normalize()` is used per call site. The binary
  calls both the minkindr (0x1800146D0) and the Eigen (0x180014680) out-of-line normalize; this is attributed
  to the inliner.
- Exact source order of functions other than the glog-anchored ones.
