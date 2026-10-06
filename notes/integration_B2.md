# Integration B2 — frontend/frame_processor_base*.cpp, frontend/frame_processor.cpp

Status: all four .cpp files compile with `tools/cc.sh` and with `CC_OBJ=1`; both owned headers
compile standalone (`--header`).  Symbol closure checked with `llvm-nm` against every object in
`build-win/cc` (A1's common/direct/plane/tracker/vikit rebuilt for the check): **no unresolved
pimax::/vk::/kindr/Sophus symbol** (details in §5).

| file | lines | content |
|---|---|---|
| `src/frontend/frame_processor_base.cpp` | 1961 | globals, helpers, ctor, dtor, input (c09), motion prior, priors file, setters, resets, relocalisation, watchdog, tracking/structure (c10, with `#line` 2417/2568/2619/2668/2714/2770/2774) |
| `src/frontend/frame_processor_base_ground.cpp` | 853 | mesh / plane / ground-plane members (c08) |
| `src/frontend/frame_processor_base_imu_init.cpp` | 808 | `initializeImu` (c09) + `eulerToRotation` (c07) |
| `src/frontend/frame_processor.cpp` | 550 | FrameProcessor (c07) |
| `src/frontend/frame_processor_base.h` | 675 | A2's header + B2 edits (§3) |
| `src/frontend/frame_processor.h` | 82 | A2's header, unchanged |

The original is ONE object `frame_processor_base.obj` (0x1800B57D0..0x180129D60) plus
`frame_processor.obj` (0x1800B1480..0x1800B57D0).  The split into three .cpp files is for size
only (none of the glog `__LINE__`s are in the ground / imu-init parts).

## 1. Every draft project function and where it went

### frame_processor.cpp (c07 frame_processor.cpp)
| address | function | status |
|---|---|---|
| 0x1800B2460 | FrameProcessor::FrameProcessor(…, const std::string& device_sn, const uint8_t& loc_mode) | upstream-modified |
| 0x1800B29B0 | FrameProcessor scalar deleting dtor | implicit (`~FrameProcessor() = default` in header) |
| 0x1800B2F80 | FrameProcessor::makeKeyframe(int) | upstream-modified (rewritten) |
| 0x1800B4180 | FrameProcessor::processFirstFrame | upstream-modified |
| 0x1800B4290 | FrameProcessor::processFrame | upstream-modified (heavily) |
| 0x1800B4970 | FrameProcessor::processFrameBundle | upstream-identical |
| 0x1800B49A0 | FrameProcessor::removeOutliersByFundamentalMat (name ours) | pimax-new |
| 0x1800B5760 | FrameProcessor::resetAll | upstream-modified |

### frame_processor_base.cpp
| address | function | from | status |
|---|---|---|---|
| 0x18047DDA0 | `g_permon` | c00 | |
| 0x18047DDB0 / DDF0 / DE30 | `kStageName` / `kTrackingQualityName` / `kUpdateResultName` (std::hash, not EnumClassHash) | c00 | |
| 0x18047ED98 | `Eigen::Vector3d G` (declared in imu_factor.h) | A2 assignment | |
| 0x1800CB4B0 | `collectTrashPoints<T>` (function template, see §2.6) | **new in B2** (nobody had drafted it; c07 table: "lib:std set insert range") | pimax-new |
| 0x1800DFDF0 | FrameProcessorBase ctor | c07 | upstream-modified |
| 0x1800E46A0 | ~FrameProcessorBase (scalar deleting 0x1800EBFB0 implicit) | c08 | upstream-modified |
| 0x1800FB650 | addImageBundle | c09 | upstream-modified |
| 0x1800FA6B0 | addFrameBundle | c09 | upstream-modified |
| (inlined) | setRecovery | upstream | upstream-identical |
| 0x18010A690 | getMotionPrior (vslot 6) | c09 | upstream-modified |
| 0x1800FE320 | checkImuMotion | c09 | pimax-new |
| 0x180115F00 | loadPriorPosition | c09 | pimax-new |
| 0x180125040 | savePriorPosition | c10 | pimax-new |
| 0x1800FDEE0 | computePoseDifference (free) | c09 | pimax-new |
| 0x1800FE140 | computePolygonArea (free) | c09 | pimax-new |
| 0x180115430 | slerpByTime (free; c10 "interpolateQuaternion") | c09 | pimax-new |
| 0x180127550 | setFirstFrames (vslot 1) | c10 | upstream-modified |
| 0x1801274A0 | setBundleAdjuster | c10 | upstream-modified |
| 0x1801277B0 | setInitialPose | c10 | upstream-modified |
| 0x1801280A0 | setRotationIncrementPrior | c10 | upstream-modified |
| 0x180128210 | setRotationPrior | c10 | upstream-modified |
| 0x180128260 | setTrackingQuality (vslot 5) | c10 | upstream-modified |
| 0x180118740 | optimizePose | c10 | upstream-modified |
| 0x18011B370 | resetAll (vslot 3, header-inline) | c10 | upstream-identical |
| 0x18011B380 | resetBackend (vslot 4) | c10 | upstream-modified |
| 0x18011B420 | resetVisionFrontendCommon | c10 | upstream-modified |
| 0x18011B8F0 | resetVisionFrontendCommonWhenSetStart | c10 | pimax-new |
| 0x18011AA90 | reLocalize | c10 | pimax-new |
| 0x18011BCA0 | checkTrackingHealth (name guessed) | c10 | pimax-new |
| 0x180119260 (+ lambda 0x1800E61D0) | projectMapInFrame | c10 (lambda verified by c08) | upstream-modified |
| 0x180118D80 | optimizeStructure | c10 | upstream-modified |
| 0x1801188C0 | optimizeStructureConsecutive (c07 "optimizeStructureInPriorMap") | c10 | pimax-new |
| 0x180128FA0 | upgradeSeedsToFeatures | c10 | upstream-modified |

### frame_processor_base_ground.cpp (c08 frame_processor_base_ground.cpp)
0x1800F19D0 `median` (file-static), 0x1800F2070 updateGroundPlane, 0x1800F3430 refinePlanes,
0x1800F4190 refitWallPlane, 0x1800EC2B0 mergeSimilarPlanes, 0x1800F64A0 selectGroundPlane,
0x1800F5170 estimateGroundPlane, 0x1800F4BD0 shouldRemoveKeyframe (all pimax-new), plus the
inlined depth helper (0x1800F2520..0x1800F258D, now `cameraDepth`).

### frame_processor_base_imu_init.cpp
0x180110490 initializeImu (c09, pimax-new), 0x1800DDCF0 eulerToRotation<double> (c07, pimax-new).

### Header-only / elsewhere (accounted for, not in my files)
* In frame_processor_base.h (A2): RunningStats::mean 0x1801181B0, ImuParams ctor 0x1800E1ED0,
  PoseState ctor 0x1800E2B10, BaseOptions dtor 0x1800E4630 (implicit).
* B3 headers: IntegrationBase (0x1800E20B0, 0x180104140, 0x180108AE0, 0x180109180, 0x180110290,
  0x18011A810, 0x18011AFE0, 0x1800F1400), IMUFactor (0x1800E1950, Evaluate 0x1800ECFD0, deleting
  dtor 0x1800EC000), VINS PoseLocalParameterization / InitGravityLocalParameter (0x1800F1DD0,
  0x1800ECF10, 0x1800F1A60, dtor 0x1800EC0A0), Utility (R2ypr 0x1800F3260, Qleft/Qright
  0x1800B8A50/0x1800B8C00, skewSymmetric 0x1800DDA80, deltaQ 0x1800CBD00), ImageFrame
  (0x1800E1CC0/0x1800E1AA0, dtor 0x1800E4E10), ImuInitializer ctor 0x1800E1FD0 / dtor 0x1800E4E90.
* A1: Frame::imuPos 0x180110400, Frame::numLandmarks 0x180118580, FrameBundle::get_T_W_B
  0x18010ADA0, setIMUState 0x180127700, Frame::set_T_w_imu 0x1801283F0, SO3::exp 0x18010A400,
  Sophus FormatStream/FormatString/defaultEnsure (0x1800B7BA0..0x1800B8900, 0x1800CBC40),
  Plane copy ctor 0x1800E2910 / dtor 0x1800E5230.
* B4 headers: KeyFrameGraph save/load 0x1800B8DB0/0x1800B9270/0x1800B9520/0x1800B99E0,
  cv::Mat/BowVector/Transformation serialize 0x1800DB230..0x1800DCFE0, KeyFrame::serialize
  0x1800DB460/0x1800DBD40/0x1800DC3C0/0x1800DCD10, KeyFrame ctor 0x1800E26D0.  (c07's
  serialization_part.cpp is fully covered by loop_closing/serialization_helpers.h + platmap.h;
  c07's vio/imu_integration_ctors.cpp by B3's integration_base.h / imu_factor.h.)

## 2. Conflicts resolved / decisions

1. **Renames to A1/A2 names** (all drafts): see the header comments of each .cpp.  Most important:
   imu_initial_ → `imu_not_initialized_`; c09 reloc_disabled_ → `reloc_enabled_` (+3936, same byte
   value, no logic inversion: ctor sets 1, the set-start reset writes 1, reLocalize clears it, LC
   frames are only added while it is 0); ba_mode_/slam_mode_/skip_prior_position_ → `loc_mode_`;
   set_start_ → `set_reset_`; T_map_world_ → `T_world_correction_`; lost_count_a_/b_ →
   `dark_t_`/`bright_t_`; big_position_jump_ → `p_diff_`; exposure_level_/input_value_ →
   `frame_flag_`; frame_flag234_cnt_ → `redundant_kf_count_`; kf_bundles_ → `frame_bundle_map_`;
   attitude_history_ → `imu_rotation_buffer_`; imu_init_timer_/imu_init_time_sec_ →
   `reloc_timer_`/`t1_voImuInit_`; c10 gp_* ground fields → c08 ground_* fields; init_*_bias_ →
   `imu_params_.wBias/aBias`; ImuProcessor gyro_bias_ → `omega_bias_`; FrameBundle T_80_ →
   `T_W_B_init_`, vec_216_ → `gravity_`, flag_64_ → `is_relocalized_`, flag_248_ → `is_keyframe_`,
   force_stereo_triangulation_ → `low_feature_kf_`, is_stationary_ → `is_static_`; Frame img_mean_
   → `mean_intensity_`, T_imu_cam_/T_cam_imu_ → `T_body_cam_`/`T_cam_body_`; Point
   n_consecutive_tracked_ → `n_consecutive_obs_`.
2. **FrameBundle counting functions by address** (A1 table): processFrame/makeKeyframe/
   addFrameBundle/checkTrackingHealth call 0x180095180 = `numLandmarks()`; c10's
   `numTrackedLandmarks() > numLandmarks()*0.8` is 0x1800952D0 vs 0x1800953F0 =
   `numTrackedEdgelets() > numTrackedLandmarks()*0.8` (verified in 0x18011BCA0).  c10's
   Frame::numTrackedIds = Frame::numLandmarks (0x180118580).
3. **processFrame / makeKeyframe "map mode"** (c07 map_mode_ +3424, map_mode_flag_ +3456,
   prior_map_ +3408): these are `bundle_adjustment_type_ == kCeres`, `imu_not_initialized_` and
   `bundle_adjustment_` (deque at +120 = `CeresBackendInterface::active_keyframes_`); verified in
   0x1800B4290 / 0x1800B2F80.  c07's `checkOverlap`/`keyframe_ids_`/`computeSceneDepth`/
   `getKeyframes` → B3's `Map::allKeyPointsVisible`/`sorted_keyframe_ids_`, A1's
   `frame_utils::getSceneDepth`, `DepthFilter::GetFramesWithoutSeeds`.
4. **ctor**: `map_` is built in the mem-initializer list from a `std::unique_ptr<Map>(new Map)`
   temporary (`_Ref_count_resource<Map*,default_delete>` in 0x1800DFDF0, between the +2980 and
   +3000 initialisations; a header default initializer is impossible with the forward-declared
   Map).  c07's `loadPriorPosition(&T_newimu_lastimu_prior_, &have_motion_prior_)` corrected to
   `(&T_prior_, &prior_position_loaded_)` (+3216/+3200, A2 §2.9).  n_cells reads the height
   (+12) before the width (+8) in the binary → written `height/gs * (width/gs)`.
5. **CeresBackendInterface API** (B1 header): c09 updateFrameBundleState 0x180013680 →
   `optimize(bundle, frame_flag_, loc_mode_)`; unknown_164_ → `num_outliers_removed_`;
   backend +160 → `imu_motion_detector_stationary_`; +1241 imu_initial_ → `imu_init_pending_`;
   +96 window_size_ → `optimizer_options_.num_imu_frames` (verified `*(ba+96)` in 0x180110490);
   getType() → `type_`; the inlined reset is `reset()` (0x1800149F0, only the LOGI) followed by a
   separate call `clearBackend()` (0x180012B20) — both called explicitly (resetBackend and dtor).
6. **collectTrashPoints 0x1800CB4B0**: undrafted; reconstructed from the pseudocode (set of all
   points when `is_static`, empty set otherwise; returned by move).  Written as a function
   template because its COMDAT sits among the "??$" templates of the object.  Removed the
   non-template declaration from frame_processor_base.h.  TODO(verify) name/template-ness.
7. **eulerToRotation 0x1800DDCF0** (c07) is used by initializeImu instead of c09's
   `Utility::ypr2R(.., false)` (A2 §2.16); defined as a template in the imu-init file.
8. **ImuInitializer** (c13) replaces c09's ImuInitParams/free VisualIMUAlignment:
   `initializer.G = (0, 0, g)`, RIC/TIC assign, `initializer.VisualIMUAlignment(...)`,
   `T_x = initializer.T_w0_ * T_body_cam_ * T0`, `new IMUFactor(pre_integration, initializer.G)`
   (3rd ctor arg is `&ImuInitializer` = its G, verified at 0x180110490 call of 0x1800E1950).
   Bgs is an aligned_allocator vector (allocator 0x1800FD8E0) — B3 changed
   `visual_imu_alignment.h` (`GyroBiasVector`) on request.
9. **IntegrationBase noise overwrite** (c09 reserved0_/reserved1_/G_NORM) → `G.x()/G.y()/G.z()`.
10. **LoopClosing names** (c16/B4): computePoseDifference → `computePoseDiff`, addFrameBundle →
    `addFrameToPR`, c10 `reLocalize(info*)` → `getReLocCorrection`, c10 addReLocFrames →
    `reLocalize(bundle, T)`, `enable_reloc_` (+953) → `options_.use_plat_map`,
    `plat_map_->keyframes_.size()` ((lc+24)->+96) → `plat_map_->kf_map_.size()`, reloc_kf_id_
    (+16) → `map_id_`, info.T_correction/kf_id → `w_T_new_old_`/`map_id_`.
11. **Loop-closure correction in addFrameBundle**: the binary is upstream's
    `hasCorrectionInfo()/consumeOldestCorrection()` on `deque<LoopCorrectionInfo>` (element read
    at +16, one element per deque block).  loop_closing.h still has `std::deque<int64_t>` and no
    helpers → request to B4 (integration_requests.md).  Until then `lc_shim::` helpers in
    frame_processor_base.cpp (marker `B2_SHIM_LC_CORRECTION`) forward to the members if they
    exist (detection idiom), else fall back to the raw deque.  **Coordinator: once B4 adds the
    members, replace the two `lc_shim::` calls by `lc_->hasCorrectionInfo()` /
    `lc_->consumeOldestCorrection(&w_T_correction)` and delete the shim.**
12. **getMotionPrior**: c09's inlined 1e-12 exp → A1's `quaternionExp()`.
13. **Checking kindr ctors** written explicitly (`kindr::minimal::RotationQuaternionTemplate<double>(q)`
    for 0x1800089C0 and `(R)` for 0x1800DEA10) since `Quaternion` is Eigen::Quaterniond.
14. **Mesher** (A1): c08 `populate3dMesh(..., Mesh3D*)` → public `populate3dMeshTimeHorizon(...,
    Mesh2D*)` with `Mesh2D mesh(3)`; `polygonArea` → `computePolygonArea`;
    `g_wall_distance_tolerance` → `g_distance_tolerance_polygon_plane_association`.
    c08's file-static `depthInFrame(const Transformation&, Vector3f)` renamed `cameraDepth` (it is
    a different, double-precision inline computation from ceres_backend's float
    `depthInFrame(TransformationF, Vector3f)` 0x18008ACB0 used by initializeImu).
15. **computePolygonArea**: c09 read `polygon_[i].pt` (cv::Point3f) — vertices are A1's
    `PolygonVertex{int lmk_id; Vector3f pos}` (same +4/+8 bytes) → `.pos.x()/.y()`.
16. **addImageBundle Frame ctor**: n_pyr_levels = `options_.img_align_max_level` (+120 of this,
    passed unchanged — c09's `frame_pyramid_levels`), grid = `grid_occupancy_`, `grid_size`.

## 3. Changes to the owned headers
* `frame_processor_base.h`: `ReprojectResult` fields renamed after the summed
  Reprojector::Statistics fields (`n_seed_matches` +24, `n_lm_matches` +32, `ave_lm_obs` +48;
  c10 n_stat136/n_stat128/ave_stat120, c07 stat24/stat32/stat48_ratio); added upstream inline
  getters `getLastFrames()` / `getNCamera()` (used by c14's SLAMManager); removed the
  `collectTrashPoints` declaration (now a template in the .cpp).  Layout unchanged.
* `frame_processor.h`: unchanged.

## 4. Remaining TODO(verify)
* Names: checkTrackingHealth, removeOutliersByFundamentalMat, collectTrashPoints (and whether it
  is a template), eulerToRotation, quaternionAngle (inlined, maybe a header helper),
  is_destructing_, ReprojectResult fields; `addImageBundle` exposure/gain element type (uint32_t).
* Spelling of the unique_ptr temporaries (reprojectors_, pose_optimizer_, depth_filter_,
  stereo_triangulation_, map_) and of initPerformanceMonitor (inlined in the ctor).
* processFrame static-bundle path: explicit `getRotation().normalize()` vs. set_T_w_imu.
* G's definition site (placed in frame_processor_base.cpp per A2).
* COMDAT/ordering effects of the 3-file split (none functional).

## 5. Symbol closure / externals expected from other groups
Undefined project symbols of my four objects (90 after removing self-references) all resolve
against current objects: B1 ceres_backend_interface.obj (loadMapFromBundleAdjustment,
bundleAdjustment, optimize, reset, clearBackend, setCorrectionInWorld, setPerformanceMonitor),
estimator.obj (setGroundPlaneConstraint, resetGroundPlaneConstraint),
pose_local_parameterization.obj, local_parameterization_additional_interfaces.obj; B3 map.obj,
pose_optimizer.obj, reprojector.obj, stereo_triangulation.obj, initialization.obj
(makeInitializer), visual_imu_alignment.obj (ImuInitializer::VisualIMUAlignment); B4
loop_closing.obj (addFrameToPR, computePoseDiff, getReLocCorrection, reLocalize,
resetLoopClosing, resetReLocalize); A1 frame.obj, point.obj, logger.obj, depth_filter.obj,
feature_detection_utils.obj, mesher.obj, mesh.obj, histogram.obj, vikit ncamera.obj /
performance_monitor.obj.  (B1/B3/B4 objects were whatever was in build-win/cc at check time.)
Other groups' objects reference from mine: FrameProcessor ctor, addImageBundle,
setBundleAdjuster, setRotationPrior, setRotationIncrementPrior (slam_manager/svo_factory) — all
defined.  No duplicate strong definitions of my globals/functions found.

## 6. For the coordinator
* `B2_SHIM_LC_CORRECTION` (§2.11) pending B4.
* c00: TU30 included pangolin/handler/handler.h (two atexit-only statics) and is where the 84
  boost::serialization singletons for PlatMap/KeyFrame/... and the KeyFrameGraph save/load
  templates were first instantiated (both archive families) — i.e. the original
  frame_processor_base.cpp (through a loop-closing header) instantiated PlatMap serialization,
  e.g. via BOOST_CLASS_EXPORT or an inline save/load in platmap.h.  Nothing in my code calls it;
  only placement differs unless B4's platmap.h reproduces that.
* frame_processor.cpp (TU29) likewise had pangolin statics and an unreferenced static std::string
  (0x18046A170) — not reproduced.
