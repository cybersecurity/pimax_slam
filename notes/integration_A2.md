# Integration A2 — headers of src/ceres_backend, src/frontend, src/loop_closing, src/interface

Status: every header below compiles on its own (`tools/cc.sh --header`) and all of them together
with A1's headers in one TU. Every class with a known size has `static_assert(sizeof/offsetof)`
(friend `...LayoutCheck` struct or `static void layout_check()` for private members).
No `.cpp` files were written. Marker: `notes/A2_headers_ready`.

## 0. Conventions decided here

* **ceres_backend headers are `.hpp`** (upstream svo_ceres_backend style). Evidence: the
  `__FILE__` of the two `LOG(ERROR)` in `MapPoint::getTriangulationParallax` is
  `...\src\ceres_backend/estimator_types.hpp` (lines 187/203, reproduced with `#line`).
  Phase B: replace `#include "ceres_backend/xxx.h"` by `.hpp` everywhere.
  frontend / loop_closing / interface headers are `.h` (upstream svo style).
* `upstream svo/vio_common` helpers used only by the backend (`skewSymmetric(Ref<const Vector3d>)`
  0x180016490, `quaternionPlusMatrix` 0x1800455A0, `quaternionOplusMatrix` 0x1800453F0) are in
  `ceres_backend/matrix_operations.hpp` (namespace pimax::totem). Replace
  `vio_common/matrix.hpp` / `vio_common/matrix_operations.h(pp)` includes by it.
* `TangentBasis(const Eigen::Vector3d&)` 0x18001ACE0 is inline in
  `ceres_backend/gravity_local_parameterization.hpp`, **namespace pimax::totem** (used by the backend
  and by the frontend's `InitGravityLocalParameter`). c03's `GravityLocalParameterization::tangentBasis`
  → `TangentBasis`. (ImuInitializer::TangentBasis 0x1801569A0 is a separate member function.)
* All data members of FrameProcessorBase, CeresBackendInterface, LoopClosing, SLAMManager are public
  (the binary's users poke at them across classes; access does not change codegen).
* MSVC pads the vfptr slot to 16 bytes in 16-aligned polymorphic classes: the "+8 members" of the
  drafts (`FrameProcessorBase::m_8_/unknown_8_`, `LoopClosing::unk_8_`) are that padding and are gone.

## 1. Files, classes, and which draft `.cpp` files implement them

### src/ceres_backend (namespace pimax::totem::ceres_backend unless noted)
| header | contents | implementing drafts (phase B) |
|---|---|---|
| error_interface.hpp | ErrorType (+kGroundPlaneError=7), `extern kErrorToStr` (**unordered_map**), ErrorInterface (16 B, `bool use_minimal_jacobians_` +8) | c00 error_interface.cpp (kErrorToStr) |
| parameter_block.hpp | ParameterBlock (0x20) | header-only |
| local_parameterization_additional_interfaces.hpp | LocalParamizationAdditionalInterfaces (16 B: vptr + `bool use_minimal_jacobians_`) | c04 local_parameterization_additional_interfaces.cpp (verify 0x180052D90) |
| homogeneous_point_local_parameterization.hpp | inline, GlobalSize `flag ? 0 : 4` | header-only (c03 draft) |
| pose_local_parameterization.hpp | decl only; GlobalSize `flag ? 0 : 7`; **no** aligned new (dtor 0x18001A5E0 = sized delete) | c05 pose_local_parameterization.cpp |
| gravity_local_parameterization.hpp | TangentBasis + GravityLocalParameterization, all inline (aligned new) | header-only (c01 inline_bodies_c01.hpp, c02 gravity_local_parameterization.h — drop) |
| general_3d_parameter_block.hpp / gravity_parameter_block.hpp | inline (0x40 / 0x38) | header-only (c03) |
| pose_parameter_block.hpp / speed_and_bias_parameter_block.hpp | 0x58 / 0x68 | c05 pose_parameter_block.cpp, speed_and_bias_parameter_block.cpp |
| pose_error.hpp / speed_and_bias_error.hpp | 0x1A0 / 0x590 | c05 pose_error.cpp, speed_and_bias_error.cpp |
| ground_plane_error.hpp | inline (0x70) | header-only (c03) |
| reprojection_error_base.hpp, reprojection_error.hpp, reprojection_error_impl.hpp | header-only (0xF0), **new member `int64_t cam_index_` +0x38** (written by addObservation 0x180010C20, not by the ctor); ctor gets default `information = Identity()` (c16: make_shared(cam, obs) in 0x18017EB00 builds Identity inside make_shared) | header-only (c00) |
| imu_error.hpp | ImuError (0x18B0), deltaQ/sinc inline | c03 imu_error_c03.cpp (ctor, Evaluate, EvaluateWithMinimalJacobians), c04 imu_error.cpp (propagation, redoPreintegration); SO3 helpers = A1 common/so3_gamma.h + common/sophus |
| marginalization_error.hpp + _impl.hpp | MarginalizationError (0x240); templates split*/pseudoInverse* in _impl.hpp | c05 marginalization_error.cpp (c04's marginalization_error.cpp duplicates ctor/ParameterBlockInfo/Evaluate*/EvaluateLocal — keep ONE copy; **delete the template bodies from both .cpp drafts**, they are in _impl.hpp now) |
| ceres_map.hpp | Map (0x580) | c02 ceres_map.cpp + c01 ceres_map_c01.cpp (Map(bool), applyFlag, setFlag) |
| estimator_types.hpp (pimax::totem) | ExtrinsicsEstimationParameters(Vec), ImuParameters (0x80), IdType, BackendId + factories, isFinite, MapPoint (0x80) + getTriangulationParallax (inline, #line 187/203), PointMap, SpeedAndBias | header-only (c02 estimator_types_c02.h, c03 estimator_types.h) |
| estimator.hpp + estimator_impl.hpp (pimax::totem) | States (addState inline), MarginalizationTiming, GroundPlaneConstraint (0x48), Estimator (0x300); impl: addObservation 0x180010C20, isPointInEstimator 0x180013420, setKeyframe | c02 estimator_c02.cpp + c03 estimator.cpp + c00 estimator_globals.cpp (names_) |
| outlier_rejection.hpp (pimax::totem) | OutlierRejection (8 B), **free inline `depthInFrame(TransformationF, Vector3f)`** (0x18008ACB0; c05 had a static member, c09 a free function) | c05 outlier_rejection.cpp |
| ceres_backend_interface.hpp (pimax::totem) | BundleAdjustmentType, CeresBackendInterfaceOptions (0x40), CeresBackendOptions (0x38), CeresBackendInterface (0x4F0) | c01 ceres_backend_interface.cpp + c00 ceres_backend_interface_ctor.cpp |
| matrix_operations.hpp (pimax::totem) | skewSymmetric, quaternionPlus/OplusMatrix | header-only |

### src/frontend (namespace pimax::totem)
| header | contents | implementing drafts |
|---|---|---|
| global.h | `PerformanceMonitorPtr`, `extern g_permon`, SVO_START/STOP_TIMER, SVO_LOG | c00 frame_processor_base_globals.cpp (definition) |
| frame_processor_base.h | Stage/TrackingQuality/UpdateResult (+ extern name maps), KeyframeCriterion, BaseOptions (256), ImuParams (96), **PoseState (448)**, RunningStats, ReprojectResult (72), FrameProcessorBase (4032), free helpers collectTrashPoints / computePoseDifference / computePolygonArea / slerpByTime | c07 frame_processor_base_ctor.cpp (+ part_templates.cpp: eulerToRotation; its Sophus part → A1, its KeyFrameGraph save/load → now in loop_closing/platmap.h), c08 frame_processor_base_dtor.cpp + frame_processor_base_ground.cpp, c09 frame_processor_base.cpp + frame_processor_base_imu_init.cpp, c10 frame_processor_base.cpp, c00 frame_processor_base_globals.cpp |
| frame_processor.h | FrameProcessor (4080) | c07 frame_processor.cpp |
| imu_processor.h | ImuCalibration (112), ImuInitialization (96) (A1 left them to me), IMUTemporalStatus + `extern imu_temporal_status_names_`, ImuProcessor (0x2A8) | c10 imu_processor.cpp (ctor/dtor/addImuMeasurement), c11 imu_processor.cpp, c00 imu_processor_globals.cpp |
| initialization.h | InitializerType, InitializationOptions (64), InitResult, AbstractInitialization (288), StereoInit (320), makeInitializer | c11 initialization.cpp |
| map.h | Map (0xB8) | c11 map.cpp |
| pose_optimizer.h | PoseOptimizer (0x4E0) + pose_optimizer_utils | c11 pose_optimizer.cpp + c12 pose_optimizer.cpp (vikit solver template → A1 third_party/vikit) |
| reprojector.h | ReprojectorOptions (96), Reprojector (0xF0) + reprojector_utils | c12 reprojector.cpp |
| stereo_triangulation.h | StereoTriangulationOptions (32), StereoTriangulation (0x90) | c12 stereo_triangulation_c12.cpp (ctor/triangulate/computeStd) + c13 stereo_triangulation.cpp (compute) |
| visual_imu_alignment.h | ImageFrame (448), ImuInitializer (136/144) | c13 visual_imu_alignment.cpp |
| integration_base.h | IntegrationBase (10576) — fully inline | header-only (c07 ctor, c08 rightJacobian/skewSymmetric, c09 push_back/midPointIntegration/evaluate, c10 propagate/repropagate — all merged) |
| imu_factor.h | `extern Eigen::Vector3d G` (0x18047ED98), IMUFactor (0x50), Evaluate inline | header-only (c08); **define `Eigen::Vector3d G;` in frame_processor_base.cpp** |
| pose_local_parameterization.h | VINS PoseLocalParameterization, InitGravityLocalParameter | header-only (c08) |
| utility.h | Utility: deltaQ, skewSymmetric, R2ypr(R, bool radians=false) 0x1800F3260, ypr2R(ypr) 0x18014DB90, ypr2R(ypr, bool) 0x18017EE10, g2R 0x1801572E0, positify, Qleft, Qright | header-only (c08 + c13 + c15 merged) |

### src/loop_closing (namespace pimax::totem; BEBLID/PairHash global)
| header | contents | implementing drafts |
|---|---|---|
| serialization_helpers.h | the ONE copy of the free boost serialize/save/load for Eigen::Matrix, cv::Point2f/3f, cv::Mat, DBoW2::BowVector, Transformation (+ SPLIT_FREE macros) | header-only (c07 serialization_part.cpp, c09/c10 serialization_helpers.h, c15 platmap.h — all drop theirs) |
| platmap.h | KeyFrame (0x300, serialize member), MapIndex, `extern kPlatMapVersion`, PlatMap (0xE0), KeyFrameGraph save/load | c15 platmap.cpp (bodies; per c15 all PlatMap code lies in loop_closing.obj — TODO(verify) file split) |
| bow.h | OrbVocabulary + bow functions | c15 bow.cpp |
| map_alignment.h | MapAlignmentOptions, MapAlignmentSE3 (0xA0) | c16 map_alignment.cpp |
| beblid.h | ABWLParams, BEBLID (72, global ns) | c14 beblid.cpp |
| pair_hash.h | PairHash | header-only |
| loop_closing.h | LCScaleRetMethod, GlobalMapType, LoopClosingMode, `extern kStrToScaleRetMap/kStrToGlobalMapType`, LoopClosureOptions (656), ReLocCorrectionInfo (112), TagIndex (120), LoopClosing (0xC10, inline setRecoveryMode), commonLandMarkCheck, recovery_kf | c15 loop_closing_ctor.cpp (ctor/dtor), c16 loop_closing.cpp, c15 geometric_verification.cpp, c00 loop_closing_globals.cpp (make kPlatMapVersion non-static) |

### src/interface
| header | contents | implementing drafts |
|---|---|---|
| ceres_backend_factory.h | `ceres_backend_factory::makeBackend` | c13 interface/ceres_backend_factory.cpp (VLOG must stay on line 13) |
| svo_factory.h | factory::load*Options, getImuProcessor, getLoopClosingModule, setInitialPose, makeFrameProcessor, FibonacciSphere, SplitPath | c14 svo_factory.cpp |
| slam_manager.h | ImageBundle (80), OrientationSample (48), SLAMManager (0xB20) | c14 slam_manager.cpp, headset_api.cpp |
| include/pimax_slam.h | unchanged (identical to the c14 draft) | |

## 2. Reconciliation decisions (with evidence)

### Backend
1. **ErrorInterface flag** name `use_minimal_jacobians_` (c03/c04/c05 majority; c00 `use_minimal_jacobian_`,
   c01 `flag_`). The same name is used for the bool of LocalParamizationAdditionalInterfaces (+8, c01:
   `dynamic_cast<LocalParamizationAdditionalInterfaces*>` + store at +8 in 0x18001A980) and for the Map
   bool +0x3E8 (c02 `flag_3e8_`, c01 `flag_`). c05's `PoseLocalParameterization::flag_10_` is that base
   member (not a derived one). GlobalSize() of the three Pimax parameterizations returns 0 while it is set
   (asm 0x18001A8E0/8F0/900: `cmp [rcx+10h],0; cmovnz`).
2. **Map::new_residual_block_ids_** is `std::vector<ceres::ResidualBlockId>` (c02: push of the
   addResidualBlock return value); c01's applyFlag draft used uint64_t + reinterpret_cast — adapt.
3. **Estimator layout** = c03's merged table + c01: +0xB8 is `size_t min_num_3d_points_for_fixation_`
   (written by the CeresBackendInterface ctor; c02/c03 `unknown_b8_`); +0xC0/D8/F0 are c03's
   `marginalize_pose_frames_ / marginalize_all_but_pose_frames_ / all_linearized_frames_`
   (c01 `marginalized_nframe_ids_`, c02 `unknown_vec_*`); +0x2F8 `optimize_count_` (c02 `unknown_2f8_`).
   Function names: c03 (owner) `reset()` 0x180029610 (c01 `resetStates`),
   `setPoseEstimateAndZeroVelocity` 0x18002CC70 (c01 `set_T_WS`), `optimize(size_t, bool, bool unused)`.
   Observations: `unordered_map<uint64_t residual id, KeypointIdentifier>`.
4. **ImuError** (c04 corrections applied): `propagation` is a non-static member with `const Vector3d& g_W`
   (10 args, rcx = ImuError at 0x180026743); `redoPreintegration(const SpeedAndBias&, const Vector3d* g_S0)`
   (null checks). Layout c03 = c04.
5. **MarginalizationError::pseudoInverseSymmSqrt<MatrixXd>** is NOT upstream-identical (c04 vs c05):
   checked the callees of 0x18006BF60 (LLT 0x1800653D0, L^{-T} 0x1800771C0, SAES) vs 0x18006CA60 (SAES only).
   c04's compile-time-branch version is in marginalization_error_impl.hpp.
6. **CeresBackendInterface**: c01 layout; +0x418 is `std::map<int,int> obs_count_map_` (c01: node 40 B,
   value +32) not c00's `std::set<int>`; `type_` is `BundleAdjustmentType` (=kCeres in the ctor body).
   CeresBackendOptions::num_keyframes is **size_t** (factory store `48 c7 45 e7 08` = qword; c01 had
   int because of a dword read); num_iterations/num_threads are int (dword stores).
7. **PoseLocalParameterization** has no EIGEN_MAKE_ALIGNED_OPERATOR_NEW (deleting dtor 0x18001A5E0 calls
   sized delete 0x18) — as upstream; all other backend classes free() (checked: General3D 0x18002CFF0,
   Gravity 0x180024C80, PosePB 0x18008D9C0, SabPB 0x18008FEB0, GroundPlaneError 0x180024CC0,
   PoseError 0x18008BBE0, SabError 0x18008EC20, HomogeneousPoint/Gravity LP 0x18001A550).
8. OutlierRejection: sizeof 8 (malloc(8)), `depthInFrame` is a free inline function (used by
   initializeImu through the out-of-line copy 0x18008ACB0).

### Frontend
9. **FrameProcessorBase** (4032 B): every offset re-derived from the ctor 0x1800DFDF0 (all initialisations
   visible); conflicts resolved:
   * +464 `uint8_t loc_mode_` = ctor's 10th argument (`const uint8_t&`; the ctor stores it and calls
     `loadPriorPosition(&T_prior_ (+3216), &prior_position_loaded_ (+3200))` when it is 0 — c07 had
     `&T_newimu_lastimu_prior_, &have_motion_prior_`, wrong). The 9th argument is unused; FrameProcessor
     forwards c14's `std::string device_sn` there → `const std::string&` (c07: `int`).
   * +536 is c14's **ImuParams** (96 B, ctor 0x1800E1ED0; c07 "S96", c10 init_gyr_bias_/init_acc_bias_
     = wBias/aBias).
   * +632..+688 are c10's watchdog counters (c07 read the zero stores as vector/pointers; c09's
     lost_count_a_/b_ = dark_t_/bright_t_; +684 `p_diff_` is c09's big-position-jump flag).
   * +784 **PoseState** (448 B, c14 layout; c07's "S552" read the zeroed Vector3d's as vector<double>);
     +1232..+1335 (104 B) are never initialised/destroyed → `uint8_t unknown_1232_[104]` TODO(verify).
   * +1680 `vector<vector<PointPtr>> trash_points_` (c10; c07 vector<vector<FramePtr>>);
     +1704/+1728 `vector<set<int>>` (c07/c10; c08 thought shared_ptr vectors).
   * +2784 `ImuMeasurements imu_window_` (32-B elements: checkImuMotion inserts the input
     ImuMeasurements range; c08's 24-B ImuSample3f was wrong).
   * +2912 `T_world_correction_` (c08/c10; c09 T_map_world_), built with the checking kindr
     RotationQuaternion ctor (0x1800089C0) as the ctor does.
   * +3408 `shared_ptr<CeresBackendInterface> bundle_adjustment_`, +3424 `BundleAdjustmentType` (c09/c10;
     c07's "prior_map_ with mode at +976" was the backend and its type_).
   * +3464 `map<double, FrameBundlePtr> frame_bundle_map_` (c09: insert(make_pair(t_cur, ...));
     c10 `kf_bundles_` with int64 key), +3520 `map<double, Quaternionf>` with std::allocator (ctor
     allocates the head node with operator new, not malloc).
   * +3576 `Vector3f imu_init_pos_` (qword+dword zero store); c10's `ground_height_` +3584 is its z.
   * +3608 `LmkPositionMap lmk_points_` (A1 plane/plane.h type), +3672/+3696 `vector<Plane>`,
     ground-plane block with c08's names/types (c10's gp_* were partly wrong types).
   * +3984 `vk::Timer` (24 B), +4008/+4016 doubles.
   * BaseOptions defaults are the inlined default ctor of loadBaseOptions 0x18015D6E0 (max_n_kfs 200,
     quality_max_fts_drop 150, relocalization_max_trials 100, use_imu true, grid_size 20 ...).
   Names where drafts differ (final ← alternatives): num_tracked_last_ ← c09 num_landmarks_last_;
   output_mutex_/output_ ← c14 state mutex/state_; plane_mutex_, plane_valid_, plane_imu_pos_,
   plane_center_ ← c14 ground_*; loc_mutex_/loc_state_ ← c09 mutex_1448_/flag_1528_;
   saved_gyro_bias_*; frame_flag_ (+3112) ← c07 input_value_, c10 exposure_level_;
   redundant_kf_count_ (+3116) ← c10 frame_flag234_cnt_; prior_position_loaded_ (+3200) ← c09
   have_prior_pose_; imu_not_initialized_ (+3456) ← c09 imu_initial_; imu_rotation_buffer_ (+3520)
   ← c10 attitude_history_; reloc_timer_ (+3984) ← c09 imu_init_timer_; t1_voImuInit_ (+4008) ← c09
   imu_init_time_sec_; reloc_enabled_ (+3936) ← c09 reloc_disabled_; set_reset_ (+2980) ← c09
   set_start_, c14 need_reset_; last_overlap_kfs_ (+3056) ← c07 m_3056_.
   Methods: `estimateGroundPlane(const FrameBundle&)` and `updateGroundPlane(const Frame&)` (c08: raw
   pointer passed; c09 declared FrameBundlePtr); `optimizeStructureConsecutive` 0x1801188C0 ← c07
   `optimizeStructureInPriorMap`; free helpers `slerpByTime` 0x180115430 ← c10 `interpolateQuaternion`,
   `computePolygonArea` 0x1800FE140 ← c08 `polygonArea`.
   `addImageBundle` exposures/gains are `std::vector<uint32_t>` (SLAMManager passes its ImageBundle
   vectors directly; c09 had `std::vector<int>`) — TODO(verify).
   `setRecovery(bool)` is declared only (the binary inlines it; needs complete LoopClosing).
10. **PoseState** field names: c09 where it was the writer (T_world_imu, T_map_world, status,
    backend_static = copy of CeresBackendInterface+160), c14 otherwise.
11. **ImuProcessor**: c10 layout (it has the ctor), c11 methods, c14 call names
    (`limitMeasurementsSize` = c14 "trimMeasurements").
12. **IntegrationBase**: +0 `Eigen::Vector3d G{0,0,9.80667}` (c07 ctor) instead of c09's
    reserved0_/reserved1_/G_NORM (same bytes; c09's initializeImu writes them → write `G` components);
    0x180110290 is `skewSymmetric` (c08) = c09's `hat`; step_V at +5664 (16-aligned).
13. **ImuInitializer** (c13, member functions; c07 ctor 0x1800E1FD0) replaces c09's `ImuInitParams` +
    free `VisualIMUAlignment(params, ...)`. **ImageFrame**: c07's empty user ctor (only the map, the
    shared_ptr and the 4 Transformations are initialised; verified 0x1800E1CC0), `shared_ptr
    pre_integration` at +120 / `is_key_frame` +136 (c07/c09; c13's raw pointer + +128 bool was wrong);
    Pimax member names T_c0_body (+144), T_world_cam, T_i, T_w_body (+336), V_w, Bg.
14. **ReprojectorOptions**: projectMapInFrame reads Reprojector+32 as max_n_kfs (c10), so max_n_kfs is at
    +32 and the Pimax extra size_t at +24 (`pimax_unknown_24`). c12 put the extra field at +56, c14 at +32.
    **c14's svo_factory.cpp must set `pimax_unknown_24 = 20; max_n_kfs = 19;`** (its `max_n_kfs = 20` /
    `unknown_32 = 19`).
15. **StereoTriangulation**: c12 names (`triangulate(..., Eigen::Vector3d& x3D) const`, member
    `computeStd(values, mean)`), c13's compute must call those (it used triangulatePoint/computeStdDev).
16. **Utility**: 0x1800DDCF0 (initializeImu "diff yaw") computes Rz*Rx*Ry — it is c07's
    `eulerToRotation(euler, is_rad)`, NOT `Utility::ypr2R(.., bool)` (0x18017EE10, Rz*Ry*Rx, loop closing).
    c09's call must use eulerToRotation, otherwise the COMDATs merge. R2ypr has `bool radians = false`.
17. **PoseOptimizer::run(frame_bundle, thresh, bool remove_outliers)** (c12), `initTracing` inline.
18. **InitializationOptions** defaults upstream; factory values in the comment.

### Loop closing / interface
19. **LoopClosing**: c16 supersedes c15 (c16 lists its corrections). Frontend call-site names map to
    c16: c09 `addFrameBundle` 0x180185E40 → `addFrameToPR`; c09 `computePoseDifference` 0x180186F30 →
    `computePoseDiff`; c10 `reLocalize(ReLocCorrectionInfo*)` 0x18018B600 → `getReLocCorrection`; c10
    `addReLocFrames` 0x180186A50 → `reLocalize(bundle, T_w_odom)`; `setRecoveryMode` is an inline member
    (+1928/+1929). c15 member names → c16 (`kf_list_loop_` → `kf_list_`, `enable_save_map_` →
    `save_map_enabled_`, ...). c14's ctor call `(options, ncam, device_sn, loc_mode)` matches
    `(options, cams, const std::string& map_tag, LoopClosingMode)`.
20. **KeyFrame**: c15 names (c07's m_0_/T_128_/v_352_ ... → map_id_/T_w_c_/bow_keypoints_ ...).
21. **Serialization**: one copy (serialization_helpers.h + platmap.h). Eigen::Matrix uses a single
    `serialize` (c10: the save path also has the no-op resize assertion); cv::Mat/BowVector/Transformation
    save from c07 (owner of 0x1800DB.../0x1800DC...), Transformation load from c09.
22. **SLAMManager**: c14 layout; ImuMeasurement from common/imu_calibration.h (field names with trailing
    `_`), PoseState from frontend (renamed fields, see §2.10), `unique_ptr<DequeHolder>` (A1's name for
    c14's Unknown2840), ThreeDof types from A1 (`ImuSample` is {acc, gyr, t}; c14's `Config` = the
    tracker's last ImuSample).
23. svo_factory.h: ImuParams now lives in frontend/frame_processor_base.h.

## 3. Remaining TODO(verify) (main ones)
* Names of all Pimax members marked TODO in the headers (FrameProcessorBase unknown_1232_[104],
  unknown_1640_, mutex_1536_, ofs_1976_/2240_/2504_, unknown_2768_..2776_; ReprojectionError cam_index_;
  Map/LP flag name; ImuParameters::unknown_60; MarginalizationError unknown_*; LoopClosureOptions unk_*;
  ReprojectorOptions pimax_unknown_24 and which of +16/+24 is cell_size).
* `addImageBundle` exposure/gain element type (uint32_t vs int).
* Header file names/paths without __FILE__ evidence (matrix_operations.hpp, utility.h,
  visual_imu_alignment.h, platmap.h, serialization_helpers.h, outlier_rejection.hpp, global.h).
* Whether `Map::setFlag` takes `vector<uint64_t>` or `vector<vector<uint64_t>>` (unused argument).

## 4. For the coordinator / other agents
* c02 quirk 11: some `<<` chains evaluate the right operand before the LogMessage ctor — C++14
  (unsequenced) evaluation order, i.e. MSVC `/std:c++14`; cc.sh uses `-std:c++17`. Decide for the build.
* Globals that phase B must define: `kErrorToStr` (error_interface.cpp), `MarginalizationTiming::names_`
  (estimator.cpp), `g_permon`, `kStageName`, `kTrackingQualityName`, `kUpdateResultName`, `G`
  (frame_processor_base.cpp), `imu_temporal_status_names_` (imu_processor.cpp), `kStrToScaleRetMap`,
  `kStrToGlobalMapType`, `kPlatMapVersion` (loop_closing.cpp; c00 had it `static`).
* No requests to A1 are outstanding (ImuMeasurement request answered; ImuCalibration/ImuInitialization
  taken into frontend/imu_processor.h).
