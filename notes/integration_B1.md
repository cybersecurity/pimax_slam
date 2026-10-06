# Integration B1: src/ceres_backend/*.cpp

## Status

The 14 `.cpp` files in src/ceres_backend all build.

- **Compile checks:** each file passes `tools/cc.sh` and `CC_OBJ=1 tools/cc.sh` (objects in `build-win/cc/src/ceres_backend/`). Every `src/ceres_backend/*.hpp` also passes `tools/cc.sh --header` after the B1 header edits.
- **Symbol closure:** after `llvm-nm --undefined-only` over the 14 objects, every undefined `pimax::`, `vk::`, `kindr` or `Sophus` symbol is defined either by another backend object or by A1's objects already built in `build-win/cc/src/common`, `src/direct` and `third_party/vikit`.
- **No phase-B symbols are needed.** The backend objects reference nothing from B2, B3 or B4.

The A1 symbols they use are:
- `pimax::g_logger`
- `KeypointIdentifier::KeypointIdentifier(const FramePtr&, size_t)`
- `Frame::w2c`, `Frame::f2c`
- `Point::getTriangulationParallax`, `Point::removeObservation(int)`
- `FrameBundle::numLandmarksInBA`
- `vk::cameras::NCamera::getCameraShared`, `vk::cameras::NCamera::get_T_C_B`
- `vk::PerformanceMonitor` ctor/dtor, `addLog`, `init`, `log`, `writeToFile`

Files and the drafts they came from (link order = `.CRT$XCU` order: interface, map, error_interface, estimator, imu_error, local_param…, marginalization, outlier_rejection, pose_error, pose_LP, pose_PB, sab_error, sab_PB):

| file | drafts |
|---|---|
| ceres_backend_interface.cpp | c01 ceres_backend_interface.cpp + c00 ceres_backend_interface_ctor.cpp (ctor/dtor) |
| ceres_map.cpp | c02 ceres_map.cpp + c01 ceres_map_c01.cpp (Map(bool), setFlag, applyFlag) |
| error_interface.cpp | c00 error_interface.cpp (`kErrorToStr`) |
| estimator.cpp | c02 estimator_c02.cpp + c03 estimator.cpp + c00 estimator_globals.cpp (`MarginalizationTiming::names_`) |
| imu_error.cpp | c03 imu_error_c03.cpp (ctor, Evaluate, EvaluateWithMinimalJacobians) + c04 imu_error.cpp (propagation, redoPreintegration) |
| local_parameterization_additional_interfaces.cpp | c04 (verify 0x180052D90) |
| marginalization_error.cpp | c05 marginalization_error.cpp (+ c04 duplicates resolved); templates stay in `marginalization_error_impl.hpp` |
| outlier_rejection.cpp, pose_error.cpp, pose_local_parameterization.cpp, pose_parameter_block.cpp, speed_and_bias_error.cpp, speed_and_bias_parameter_block.cpp | c05 |

Other drafts:
- c01's `inline_bodies_c01.hpp` and `estimator_inline_c01.hpp` were already merged into the A2 headers (`gravity_local_parameterization.hpp`, `local_parameterization_additional_interfaces.hpp`, `estimator_impl.hpp`, `matrix_operations.hpp`) and are not used.
- c00's `other_objects_globals.inc` is not backend code.

## Globals defined
| address | global | file |
|---|---|---|
| 0x18047D9F0 (init 0x180001030) | `ceres_backend::kErrorToStr` (unordered_map, 8 entries incl. GroundPlaneError) | error_interface.cpp |
| 0x18047DA40 (init 0x180001460) | `MarginalizationTiming::names_` (6 strings) | estimator.cpp |

## Test switch
In `Estimator::optimize` (0x18002A6D0, estimator.cpp ~l.951):

```
#ifdef PIMAX_SLAM_TEST_DETERMINISTIC
  map_ptr_->options.max_solver_time_in_seconds = 1e9;
#else
  map_ptr_->options.max_solver_time_in_seconds = 0.025;
#endif
```

- **Normal build:** unchanged; preprocessing confirms 0.025.
- **glog line numbers:** every later glog statement has its own `#line`, so the extra lines shift nothing.
- **For the coordinator:** CMakeLists.txt does not define this option yet. Add `option(PIMAX_SLAM_TEST_DETERMINISTIC ...)` and, when it is ON, `target_compile_definitions(... PIMAX_SLAM_TEST_DETERMINISTIC)`. The random_device half of the switch is in StereoTriangulation::compute (B3).

## Header changes made by B1 (all in src/ceres_backend)
1. **`ceres_map.hpp`:** `Map::setFlag(const std::vector<std::vector<uint64_t>>&, bool)`, changed from `vector<uint64_t>`. This settles A2 §3's open question. In the binary, Estimator::optimize zeroes a 24-byte local, passes it to 0x18001A950, and destroys it by looping over 24-byte elements with `~vector` (0x180019E70), so it is a vector of vectors. The definition in ceres_map.cpp was changed to match.
2. **`marginalization_error.hpp`:** forward-declared `class pimax::totem::Estimator;` and added `friend class ::pimax::totem::Estimator;`. The VLOG(21) lines 1184/1185/1193/1194 in `applyMarginalizationStrategy` read the protected `H_` and `parameter_block_infos_`. Access specifiers do not change the generated code.
3. **`estimator.hpp`:** added the upstream inline `size_t numLandmarks() const { return landmarks_map_.size(); }`, used by `bundleAdjustment` 0x180011700 (reads Estimator+0x110).

No header outside src/ceres_backend was touched, and `notes/integration_requests.md` has no B1 requests.

## ceres_backend_interface.cpp (object #1, TU1): drafts c00 + c01

Function order: ctor and dtor first, then the c01 order. Every glog line number is pinned with
`#line` (281, 410, 494–500, 509, 613). The draft's blank-line padding was removed and replaced by
those directives.

| address | function | draft | status / where |
|---|---|---|---|
| 0x180008CD0 | `CeresBackendInterface::CeresBackendInterface(options, optimizer_options, camera_bundle)` | c00 = c01 (identical) | modified: no MotionDetector, no extrinsics, no time limit. The other members use the header's in-class initialisers |
| 0x180009950 | `CeresBackendInterface::~CeresBackendInterface` | c00 = c01 | modified: empty body, no quitThread |
| 0x180010440 | `addLandmarksAndObservationsToBackend(const FramePtr&, double speed)` | c01 | modified |
| 0x180011170 | `addStatesAndInertialMeasurementsToBackend` | c01 | modified (writes `FrameBundle::gravity_`) |
| 0x180011700 | `bundleAdjustment(const FrameBundlePtr&, int, uint8_t)` | c01 | modified (heavily) |
| 0x180012B20 | `clearBackend()` (name TODO) | c01 | new |
| 0x180013480 | `loadMapFromBundleAdjustment` | c01 | modified |
| 0x180013680 | `optimize(const FrameBundlePtr&, int, uint8_t)` | c01 | modified |
| 0x1800149F0 | `reset()` | c01 | modified (LOGI only) |
| 0x180015870 | `setCorrectionInWorld` | c01 | modified |
| 0x180015A80 | `setImu` | c01 | modified |
| 0x180015DC0 | `setPerformanceMonitor` | c01 | identical |
| 0x1800167C0 | `updateActiveKeyframes` | c01 | modified |
| 0x180010C20 / 0x180013420 / (inline) | `Estimator::addObservation` / `isPointInEstimator` / `setKeyframe` | c01 | header-inline, `estimator_impl.hpp` (A2) |
| 0x180008F50, 0x18000BE8C, 0x18000C020, 0x18000C460, 0x18000C490, 0x18000E0A0, 0x180015C30, 0x180015DB0, 0x180014670, 0x180016B50, 0x180013020, 0x180013030, 0x180012C00, 0x180016740 | ReprojectionError: ctor, dtors, Evaluate, EvaluateWithMinimalJacobians, EvaluateMinimal, setInformation, setMeasurement, accessors, typeInfo | c00 / c01 | header-only, `reprojection_error*.hpp` (A2) |
| 0x180014950 / 0x180014980 / 0x180014A10 | parameterBlockDim / parameterBlocks / residualDim=2 (ICF) | c01 | header-inline |
| 0x180016490 | `skewSymmetric(Ref<const Vector3d>)` (c01 "crossMx") | c01 | header-inline, `matrix_operations.hpp` |
| 0x1800162B0 | `FrameBundle::set_T_W_B` | c01 | header-inline, `common/frame.h` (A1) |
| 0x18000C120, 0x18000C2C0, 0x18000F310, 0x18000F500, 0x18000F6A0 | Logger Debug / Error / TimeString / Info / Warn (COMDATs placed in object #1) | c00 | header-inline, `common/logger.h` (A1) |

The c01 draft names were adapted to the reconciled headers:
- FrameBundle +0xE4 `imu_stationary_` became `is_static_`; +0xD8 `state_vec3f_` became `gravity_`.
- Frame +0xEA `near_map_kf_` became `is_redundant_kf_`; +0x14 `cam_id_` became `cam_index_`.
- Point +0x78 is `n_consecutive_obs_`. c01 called it "Point+120 n_succeeded_reproj_?"; n_succeeded_reproj_ is at +0x54.
- `backend_.resetStates()` became `reset()` (0x180029610). `backend_.set_T_WS` became `setPoseEstimateAndZeroVelocity` (0x18002CC70).
- `backend_.marginalized_nframe_ids_` became `marginalize_pose_frames_`. Checked: 0x1800167C0 reads this+368 = Estimator+0xC0.
- `backend_.numLandmarks()` is the upstream inline, which the estimator fork added to estimator.hpp.
- The ImuProcessor bias setters and getters are gone (Pimax has no bias mutex). The code writes and reads `imu_handler_->acc_bias_` (+208) and `omega_bias_` (+232) directly.
- `ImuParameters::g0` (c01) is `unknown_60` (estimator_types.hpp, +0x60). c01's evidence suggests the name `g0` (prior gyro bias); this is still TODO(verify).
- Checked in the binary: the marginalization window is `int` (low dword of `num_keyframes`) and is sign-extended (`movsxd`) into `applyMarginalizationStrategy`, as drafted.

## ceres_map.cpp (object #2, TU2): drafts c01 + c02

| address | function | draft | status |
|---|---|---|---|
| 0x180018E90 | `Map::Map(bool)` | c01 | modified (see below) |
| 0x18001A950 | `Map::setFlag(const std::vector<std::vector<uint64_t>>&, bool)` | c01 | new |
| 0x18001A980 | `Map::applyFlag(bool)` | c01 | new |
| 0x18001B4D0 | `addParameterBlock` | c02 | modified |
| 0x18001B840 | `addResidualBlock(cost, loss, vector&)` | c02 | modified |
| 0x18001C370 | `addResidualBlock(cost, loss, x0..x9)` | c02 | modified |
| 0x18001CD70 | `errorInterfacePtr(id) const` | c02 | identical |
| 0x18001CEC0 | `isParameterBlockConstant` | c02 | modified |
| 0x18001D310 | `parametersPtr` | c02 | new |
| 0x18001D360 | `parameterBlockExists` | c02 | identical |
| 0x18001D3A0 | `parameterBlockIdOfResidual` | c02 | new |
| 0x18001D4A0 | `parameterBlockPtr(id)` | c02 | modified |
| 0x18001D940 | `printParameterBlockInfo` | c02 | identical |
| 0x18001E1D0 | `removeParameterBlock(id)` | c02 | modified |
| 0x18001E4C0 | `removeResidualBlock` | c02 | modified |
| 0x18001EA00 / 0x18001EA40 | `residuals(id)` / `residuals(id, out)` | c02 | modified / new |
| 0x1800202C0 / 0x180020380 | `setParameterBlockConstant` / `setParameterBlockVariable` | c02 | modified |
| (inline only) | `parameterBlockPtr(id) const` | c02 | `.cpp`, upstream-identical |
| 0x18001CF70 / 0x18001D5D0 / 0x18001ACE0 | `GravityLocalParameterization::minus` / `plus`, `TangentBasis` | c02 / c01 | header-inline, `gravity_local_parameterization.hpp` |
| 0x18001A620, 0x18001A780, 0x18001A8E0, 0x18001A930, 0x18001A960, 0x18001A550 | GravityLocalParameterization ComputeJacobian / ComputeLiftJacobian / GlobalSize / Minus / Plus / dtor | c01 | header-inline |
| 0x18001A8F0, 0x18001A910, 0x18001A900, 0x18001A920, 0x18001A5E0 | HomogeneousPoint / Pose LocalParameterization GlobalSize / LocalSize / dtor | c01 | header-inline |

Conflict resolutions and checks against the binary:
- **c01 `flag_`:** now `use_minimal_jacobians_` on Map +0x3E8, ErrorInterface +8 and LocalParamizationAdditionalInterfaces +8 (A2 §2.1).
- **`new_residual_block_ids_`:** now `std::vector<ceres::ResidualBlockId>` (A2 §2.2). c01's applyFlag loop uses that type, so the `reinterpret_cast` is gone. The binary hashes the 8-byte key with FNV-1a in both cases, so the generated code is identical.
- **`Map::Map(bool)` (0x180018E90):** Problem::Options on the stack:
  - the cost, loss and local-parameterization ownerships are `DO_NOT_TAKE_OWNERSHIP`;
  - `manifold_ownership` keeps its default of 1;
  - word +0x10 = 0x100, i.e. `disable_all_safety_checks = true`.

  The control block is `_Ref_count_resource<ceres::Problem*, default_delete<Problem>>` and is allocated only after a null test. This is the `shared_ptr(unique_ptr&&)` path, so the source is written `problem_ = std::unique_ptr<ceres::Problem>(new ceres::Problem(opts));`. The draft's `problem_.reset(new ...)` would produce `_Ref_count<Problem>`. The flag stores come after the Problem, in the order pose, gravity, homogeneous point, as drafted.
- **applyFlag (0x18001A980):** checked against the pseudocode, including the order of the two loops in both branches, the double call of `localParameterizationPtr()` (vslot 10) around the `dynamic_cast`, and clearing both vectors at the end.
- **`new_parameter_block_ids_` (+0x568):** grepping all.asm finds no writer apart from the ctor, the dtor and applyFlag's clear. It is kept, always empty.

## error_interface.cpp (object #3, TU3): draft c00
Defines `kErrorToStr` at 0x18047D9F0 (initializer 0x180001030). The draft is used as is, with a `.hpp` include. The definition gets external linkage through the header's `extern const` declaration.

## Small c05 objects

| address | function | file | status |
|---|---|---|---|
| 0x18008A7C0 | `OutlierRejection::removeOutliers` | outlier_rejection.cpp | modified (only counts inliers) |
| 0x18008ACB0 | `depthInFrame(const TransformationF&, const Vector3f&)` | outlier_rejection.hpp (free inline, A2) | new |
| 0x18008BAC0 | `PoseError::PoseError(T, info)` | pose_error.cpp | identical |
| 0x18008BBD0 / 0x18008BBE0 | PoseError dtor thunk / deleting dtor | implicit (`= default`) | identical |
| 0x18003AE00 (ICF) | `PoseError::Evaluate` | pose_error.cpp | modified (flag dispatch) |
| 0x18008BC30 | `PoseError::EvaluateWithMinimalJacobians` | pose_error.cpp | modified |
| 0x18008C820 | `PoseError::setInformation` | pose_error.cpp | modified |
| 0x18008C970 | `PoseError::typeInfo` | pose_error.hpp | identical |
| 0x18008C980 / 0x18008C990 / 0x18008CC20 / 0x18008CC40 | PoseLocalParameterization `ComputeJacobian` / `ComputeLiftJacobian` / `Minus` / `Plus` | pose_local_parameterization.cpp | identical / modified / identical / identical |
| 0x18008CC60 / 0x18008CEE0 / 0x18008D270 / 0x18008D570 | static `liftJacobian` / `minus` / `plus` / `plusJacobian` | pose_local_parameterization.cpp | modified (normalisations; `plus` uses `quaternionExp` + `normalize()`) |
| 0x18001A900 / 0x18001A920 / 0x18001A5E0 | PoseLocalParameterization GlobalSize / LocalSize / dtor | pose_local_parameterization.hpp | modified / identical |
| 0x18008D960 / 0x18008D9C0 | PoseParameterBlock ctor / dtor | pose_parameter_block.cpp | identical |
| 0x18008DA00, 0x18008DAF0, 0x18008DB40 | dimension / liftJacobian / typeInfo | pose_parameter_block.hpp | identical |
| 0x18008DA10 / 0x18008DB00 | `estimate()` / `setEstimate` | pose_parameter_block.cpp | identical |
| 0x18008E8F0 / 0x18008E980 | SpeedAndBiasError ctors | speed_and_bias_error.cpp | identical |
| 0x18008EC08 / 0x18008EC20 | SpeedAndBiasError dtors | implicit | identical |
| 0x18003AE00 (ICF) | `SpeedAndBiasError::Evaluate` | speed_and_bias_error.cpp | modified (flag dispatch) |
| 0x18008EC70 | `SpeedAndBiasError::EvaluateWithMinimalJacobians` | speed_and_bias_error.cpp | modified |
| 0x18008FB10 | `SpeedAndBiasError::setInformation` | speed_and_bias_error.cpp | identical |
| 0x18008F490 / 0x18008FE50 | residualDim / typeInfo | speed_and_bias_error.hpp | identical |
| 0x18008FE60 | SpeedAndBiasParameterBlock ctor | speed_and_bias_parameter_block.cpp | identical |
| 0x18008FEB0, 0x18008FEF0, 0x180090050, 0x1800900B0, 0x180090110, 0x180090140 | SpeedAndBiasParameterBlock dtor / plusJacobian=liftJacobian / minus / plus / setEstimate / typeInfo | speed_and_bias_parameter_block.hpp | identical |

Changes from the c05 drafts:
- **`PoseLocalParameterization::plus` (0x18008D270):** the draft's `Quaternion::exp` (kindr) is now A1's `quaternionExp()` plus `q.normalize()`. Checked in the pseudocode:
  - the threshold is `theta >= 1e-12`;
  - the small-angle value is `0.5 + theta²·0.0208333`;
  - the product is normalised by `if (n > 0) q /= sqrt(n)`.
- **`PoseParameterBlock::setEstimate`:** reads `T_WS.getRotation().toImplementation()`, because `Quaternion` is now `Eigen::Quaterniond` and getRotation() returns the kindr type.
- **`depthInFrame`:** a free function (A2 §2.8). c05 had it as a static member.
- **Default ctors:** the PoseParameterBlock and SpeedAndBiasParameterBlock default ctors are kept (declared in the headers), although the binary does not emit them because nothing references them.

## estimator.cpp (object #4, TU4): drafts c00 + c02 + c03

glog line numbers are pinned with `#line`. The c03 draft had none, so one was added in front of every glog statement:
- 1168, 1176, 1184, 1185, 1193, 1194 (applyMarginalizationStrategy)
- 1454, 1506, 1509 (optimize)
- 1581, 1586 (optimize summary / printStates)

Preprocessing confirms these values. The 187/203 lines of `MapPoint::getTriangulationParallax` are in `estimator_types.hpp` (A2).

| address | function | draft | status |
|---|---|---|---|
| 0x18047DA40 (init 0x180001460) | `MarginalizationTiming::names_` | c00 | identical |
| 0x180022EF0 | `Estimator::Estimator(shared_ptr<Map>)` | c02 | modified: map_ptr moved in, Huber(0.5), extra Huber(1.5) (ground plane) |
| 0x180023430 | `Estimator::Estimator()` | c02 | modified: reserve(10) on the 3 States vectors |
| 0x1800239A0 | `Estimator::~Estimator` | c02 | identical (empty body) |
| 0x180025810 | `addCameraBundle` | c02 | modified |
| 0x180025930 | `addGroundPlaneError` | c02 | new |
| 0x180025BE0 | `addImu` | c02 | modified |
| 0x180025C80 | `addLandmark` | c02 | modified (General3DParameterBlock, Trivial) |
| 0x180026060 | `States::addState` | c02 | header-inline (estimator.hpp), modified (no DEBUG_CHECK) |
| 0x180026100 | `addStates` | c02 | modified (heavily) |
| 0x180027470 | `addVelocityPrior` | c02 | modified (1e6 bias information) |
| 0x180027A20 | `isFinite(Vector3d)` | c02 | header-inline (estimator_types.hpp), new |
| 0x180024D50 | `Frame::T_world_imu` | c02 | header-inline (common/frame.h), identical |
| 0x180027AE0 | `applyMarginalizationStrategy` | c03 | modified (heavily) |
| 0x180029610 | `reset` | c03 | new |
| 0x1800298A0 | `resetGroundPlaneConstraint` | c03 | new |
| 0x180029B80 | `getPoseEstimate` | c03 | modified (no CHECK) |
| 0x180029F30 | `getSpeedAndBias` | c03 | identical |
| 0x180029F90 | `getSpeedAndBiasEstimate` | c03 | modified |
| 0x18002A160 | `MapPoint::getTriangulationParallax` | c03 | header-inline (estimator_types.hpp, #line 187/203), new |
| 0x18002A4F0 | `get_T_WS` | c03 | identical |
| 0x18002A6D0 | `optimize(size_t, bool, bool)` | c03 | modified (heavily); + test switch |
| 0x18002B960 | `Frame::pos()` | c03 | header-inline (common/frame.h), modified (Vector3f) |
| 0x18002B9D0 | `printStates` | c03 | modified |
| 0x18002BFB0 | `registerFixedFrame` | c03 | header-inline (estimator.hpp), modified (no CHECK) |
| 0x18002C080 | `removeGroundPlaneErrors` | c03 | new |
| 0x18002C100 | `removeObservation` | c03 | modified |
| 0x18002C350 | `resetMap` | c03 | new |
| 0x18002C530 | `setGroundPlaneConstraint` | c03 | new |
| 0x18002C7B0 | `setOldestFrameFixed` | c03 | identical |
| 0x18002CC70 | `setPoseEstimateAndZeroVelocity` | c03 | new |
| 0x18002CF00 | `updateAllActivePoints` | c03 | modified |

Header-inline functions first emitted in estimator.obj (c03 §1b), all in A2 headers with their addresses:
- **gravity_parameter_block.hpp:** 0x18002A560, 0x18002B800, 0x18002C510, 0x18002CEC0, 0x180024C80
- **parameter_block.hpp:** 0x18002A6C0, 0x18002C7A0, 0x18002B7F0
- **general_3d_parameter_block.hpp:** 0x18002CFB0, 0x18002CFF0, 0x18002D030, 0x18002D060, 0x18002D090, 0x18002D0C0, 0x18002D0D0
- **ground_plane_error.hpp:** 0x18002D110, 0x18002D338, 0x18002D350, 0x18002D390, 0x18002DEF0, 0x180024CC0
- **homogeneous_point_local_parameterization.hpp:** 0x18002DF00, 0x18002DF40, 0x18002DF90, 0x18002DFC0

Everything else in the c02 (from 0x180020440) and c03 estimator tables is library code.

Conflicts and how they were resolved:
- **No overlapping functions:** c02 covers 0x180020440..0x180027AE0 and c03 covers the rest, so neither draft is dropped.
- **Map flag call:** c03's `applyPendingBlockStates(vector<vector<uint64_t>>&, true)` is `Map::setFlag` with a vector-of-vectors argument (header change 1).
- **ReprojectionError chi2 accessor:** c03's `lastResidual()` is A2's `weightedError()`. The binary calls vtable +0x28 (slot 5, 0x180016B50), which returns the value at +0x50.
- **Renamed members:**
  - Point +0x78: c03's `ba_obs_frames_` is now `n_consecutive_obs_` (A1).
  - FrameBundle: c02's `velocity_` / `bias_gyr_` / `bias_acc_` are now `imu_vel_w_` / `imu_gyr_bias_` / `imu_acc_bias_`.
- **Includes:** `frontend/frame.h` and `frontend/point.h` became `common/*`; backend `.h` includes became `.hpp`.
- **Checked against the binary:**
  - `optimize`:
    - store order of the options (offsets 208, 88, 104, 112);
    - gating offsets on the point and MapPoint;
    - the parallax is stored only in the second loop;
    - the frame is locked and the jump tested in the binary's order, and the frame is released before the landmark is removed;
    - the chi2 branches at 2.5 and 1.0;
    - the survivor ratio is computed in float and compared with 0.2;
    - `FullReport` and `printStates` run only when `verbose` is set.
  - Start of `applyMarginalizationStrategy`:
    - the inlined reset;
    - the early return;
    - `get_T_WS` + LOGE;
    - the old residual is removed before `resetPrior`;
    - the id in "removing block with id" is printed in decimal.
  - `reset` (0x180029610): checked as well.

## imu_error.cpp (object #5) and local_parameterization_additional_interfaces.cpp (object #6): drafts c03 + c04

| address | function | draft | status / where |
|---|---|---|---|
| 0x1800398A0 | `ImuError::ImuError(meas, params, t0, t1, speed_and_biases_ref)` | c03 | imu_error.cpp, modified |
| 0x18003AD60 / 0x18003AD54 | ImuError deleting dtor / thunk | c03 | implicit (`= default`) |
| 0x18003AE00 | `ImuError::Evaluate` (ICF with PoseError / SpeedAndBiasError) | c03 | imu_error.cpp |
| 0x18003AE40 | `ImuError::EvaluateWithMinimalJacobians` | c03 | imu_error.cpp; redo call adapted |
| 0x180041ED0 | `deltaQ` (+ `sinc`) | c03 | header-inline (imu_error.hpp) |
| 0x1800427F0 | `ImuError::propagation` (non-static, 10 args) | c04 | imu_error.cpp; `#line 488` / `#line 495` added |
| 0x1800453F0 / 0x1800455A0 | `quaternionOplusMatrix` / `quaternionPlusMatrix` | c04 | header-inline (matrix_operations.hpp) |
| 0x180045750 | `ImuError::redoPreintegration(const SpeedAndBias&, const Vector3d* g_S0) const` | c04 | imu_error.cpp |
| 0x18004E420 / 0x180050EE0 | `residualDim` (15) / `typeInfo` (5) | c04 | header-inline (imu_error.hpp) |
| 0x180052D90 | `LocalParamizationAdditionalInterfaces::verify` | c04 | local_parameterization_additional_interfaces.cpp, identical |
| 0x18002E0E0, 0x18002E710, 0x180042060, 0x180042530, 0x180042410, 0x1800423F0, 0x1800409E0 | Sophus SO3 ctor, operator*, exp(omega,eps), log, hat, matrix, alternatingSeries | c03 | A1 `common/sophus/so3ex_base.h` |
| 0x180040AD0, 0x180040FE0, 0x180041600, 0x180041C40, 0x18002EF20, 0x1800319C0, 0x1800344E0, 0x180035AD0, 0x180030480, 0x180032F70 | gamma1/2/3, gamma1Right, dGammaTV2/3/4, dGammaV1/2/3 | c03 | A1 `common/so3_gamma.h` |
| 0x180037490, 0x18002EDE0, 0x18002E970 | Sophus defaultEnsure / FormatString / FormatStream | c03 | A1 `common/sophus/common.hpp` |

Conflicts and how they were resolved:
1. **`propagation`:** c03 made it static with no gravity argument. c04's non-static member with `const Vector3d& g_W` is kept: at the call site 0x180026743, rcx holds the ImuError (as A2 decided).
2. **`redoPreintegration` 2nd argument:** a pointer, null-checked twice (c04). EvaluateWithMinimalJacobians now builds `const Vector3d g_S0 = C_S0_W * (-g_W);` and passes `&g_S0`, matching the binary (address of a temporary).
3. **Tangent basis:** c03's `GravityLocalParameterization::tangentBasis` is now the free `TangentBasis` (0x18001ACE0, A2).
4. **glog line numbers:** 488 and 495 were only in c04's comments. They are now `#line` directives; the binary passes 488 and 495 to 0x180354E50.
5. **Layout:** c03 and c04 agree, so `imu_error.hpp` is used unchanged. c04's local ImuMeasurement and ImuParameters copies were dropped.
6. **Where `verify()` lives:** its range 0x180050EF0..0x180053DA8 lies between imu_error.obj's last function (typeInfo 0x180050EE0) and marginalization_error.obj's first (0x180053DB0). That matches the link order, so it belongs to local_parameterization_additional_interfaces.obj. The evidence is in the file comment.

## marginalization_error.cpp (object #7): drafts c04 + c05

| address | function | draft | status / where |
|---|---|---|---|
| 0x1800732A0 | `splitSymmetricMatrix<MatrixXd x4>` | c04 = c05 | marginalization_error_impl.hpp (instantiated by marginalizeOut) |
| 0x1800740F0 | `splitVector<VectorXd x3>` | c04 = c05 | _impl.hpp |
| 0x18006CA60 | `pseudoInverseSymmSqrt<Matrix3d>` | c04 = c05 | _impl.hpp (upstream path) |
| 0x18006BF60 | `pseudoInverseSymmSqrt<MatrixXd>` | c04 (LLT fast path) | _impl.hpp; c05's "identical" rejected (A2 §2.5) |
| 0x1800773E0 | `MarginalizationError(Map&)` | c05 (c04 equivalent: setMap inlined) | .cpp |
| 0x180077740 | `ParameterBlockInfo(id, ptr, ordering, is_landmark)` | c04 = c05 | header-inline (marginalization_error.hpp) |
| 0x180077C70 / 0x1800784C0 / 0x1800784A8 | dtor body / deleting dtor / thunk | c04/c05 | implicit (`= default`) |
| 0x180078510 | `Evaluate` | c05 (c04 identical) | .cpp |
| 0x180078540 | `EvaluateWithMinimalJacobians` | c05 (c04 identical) | .cpp |
| 0x180079000 | `EvaluateLocal` (name TODO) | c05 (c04 identical) | .cpp (CostFunction vslot 2, as in the binary) |
| 0x180079D90 | `addResidualBlock` | c05 | .cpp; `#line 120` added (binary: `lea r8d,[r15+78h]` at 0x180079DE1) |
| 0x18007AA20 | `computeDeltaChi` | c05 | .cpp |
| 0x18007B1F0 | `getParameterBlockPtrs` | c05 | .cpp |
| 0x18007B280 | `linearizeResidualBlocks` (name TODO) | c05 | .cpp |
| 0x180080320 | `marginalizeOut` | c05 | .cpp |
| 0x180087010 / 0x180088D00 | `residualDim` / `typeInfo` (3) | c05 | header-inline |
| 0x180088D10 | `updateErrorComputation` | c05 | .cpp |

c04 and c05 both drafted the ctor, ParameterBlockInfo, Evaluate, EvaluateWithMinimalJacobians and EvaluateLocal. The two versions mean the same; they differ only in `nullptr`/`NULL`, formatting, and `setMap()` versus inlined assignments. c05's text is kept. `addResidualBlock` was re-checked against the pseudocode, including these upstream-UB quirks:
- `parameter_block_infos_.back()` on an empty vector;
- the spec-map `find()->second` without an end check.

The template bodies were removed from both .cpp drafts. llvm-nm shows that all four instantiations are emitted through their uses.

## Remaining TODO(verify) (backend)
- **Names not proven by the binary:**
  - Map: `use_minimal_jacobians_`, `applyFlag`, `setFlag` and their argument element type, `parametersPtr`, `parameterBlockIdOfResidual`, `new_*_ids_`.
  - CeresBackendInterface: `clearBackend`, `unknown_flag_1240_`, `num_outliers_removed_`.
  - Estimator: `setPoseEstimateAndZeroVelocity`, `optimize_count_`, the ground-plane members, `gravity_parameter_block_id_`.
  - `ImuParameters::unknown_60`: c01 calls it `g0`, the prior gyro bias written from `ImuProcessor::omega_bias_` in `setImu`.
  - MarginalizationError: `EvaluateLocal`, `linearizeResidualBlocks`, the booking containers at +0x1D0..+0x228, the unused Eigen members at +0x48..+0xA0, and `unknown_238_`.
- **Estimator:**
  - parameter names of `addStates`;
  - the exact form of the speed/bias fill in `addVelocityPrior`;
  - the unused `get_T_WS` in `applyMarginalizationStrategy`;
  - the two unused `high_resolution_clock::now()` calls around `marginalizeOut`;
  - evaluation order in `timing->add(std::string(..), timer.stop())`: the binary builds the string first.
- **MarginalizationTiming** (estimator.hpp): `optimize` 0x180013680 builds the map in its ctor with an inlined lower_bound and insert per name. This matches both `emplace` and `operator[]`, so either form fits the code.
- **ImuError:**
  - c03's open point: the order in which the Jacobian pointers are re-tested in EvaluateWithMinimalJacobians;
  - c04's open points in redoPreintegration: whether the 9x9 acc-noise term is a named local, and the spelling of the scaled products.
- **pseudoInverseSymmSqrt<MatrixXd>:** the LLT fast path is modelled as a compile-time branch inside the template. It could also be a separately named Pimax function.
- **Default ctors:** the PoseParameterBlock / SpeedAndBiasParameterBlock default ctors are defined but not emitted in the binary (nothing references them).
- **Evaluation order:** c02 quirk 11 / A2 §4: some glog `<<` chains evaluate operands before the LogMessage ctor (C++14, `/std:c++14`); cc.sh uses `-std:c++17`. The coordinator decides.

## For the coordinator
- Add the CMake option `PIMAX_SLAM_TEST_DETERMINISTIC` (see "Test switch").
- No unresolved externals are expected from other phase-B groups. The backend objects need only A1 objects (common/point.obj, common/frame.obj, common/logger.obj, third_party/vikit ncamera.obj and performance_monitor.obj) plus the libraries.
- B3's interface/ceres_backend_factory.cpp should construct `CeresBackendInterface(options, optimizer_options, camera_bundle)` with the header's option structs. The constructor is defined in ceres_backend_interface.cpp.
