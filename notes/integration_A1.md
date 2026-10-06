# Integration A1 — core headers, common/, direct/, sensor_fusion/, plane/, tracker/, vendored libs

(Complete. Parts B and C were done by two helper forks of A1 and are merged below.)

## API for A2 (names that differ from the chunk drafts)

### Types / aliases (`common/types.h`, `common/transformation.h`)
* `FloatType = float`; `Keypoint/BearingVector/Position/GradientVector/SeedState` and the dynamic
  matrices are float. `Scores = VectorXf`, `Levels = VectorXi`, `TrackIds = VectorXi`.
* `FeatureType` has the **upstream values** (all compiled predicates agree). The only odd
  predicate, `(t & 0xF9) == 0 && t != 2`, is `isTrackedEdgeletType()` (used by
  `FrameBundle::numTrackedEdgelets`).
* `Transformation = kindr::minimal::QuatTransformationTemplate<double>`,
  `TransformationF = ...<float>`.
* **`Quaternion = Eigen::Quaterniond`** (not kindr). Evidence: Frame +0xC0 /
  AbstractInitialization +176/+208 are left uninitialised, quaternion inversion divides by |q|²
  (Eigen) in setRotationIncrementPrior/optimizePose (c10 quirk 15), `Quaternion(C_imu_world)` in
  getInitialAttitude has no isValidRotationMatrix CHECK. Where the binary uses the *checking*
  kindr ctor (0x1800089C0, e.g. c09 `T_80_.getRotation() = Quaternion(q)`), write
  `kindr::minimal::RotationQuaternionTemplate<double>(q)` explicitly.
* **`quaternionExp(const Eigen::Vector3d&)`** (transformation.h): the 1e-12-threshold
  rotation-vector→quaternion map inlined in PoseLocalParameterization::plus 0x18008D270,
  getMotionPrior 0x18010A690 and getRelativeRotationPrior 0x18012AB60 (no CHECK, result not
  normalised; callers multiply and then `normalize()`). It is **not** minkindr's exp:
  `Transformation::exp` (PoseOptimizer::update 0x18013F830) is the unmodified minkindr one
  (eps^(1/4) threshold, CHECK_NEAR line 59). Replace the drafts' `Quaternion::exp(...)` by
  `quaternionExp(...)` and keep the explicit renormalisation.
* `EnumClassHash` is in types.h (used by kStageName etc.).

### Frame (`common/frame.h`, sizeof 0x340)
| offset | final name | draft names |
|---|---|---|
| +0x14 | `cam_index_` | c01 `cam_id_`, c11 `bundle_id_` (wrong: +20 is the camera index; c11's Map::getClosestNKeyframesWithOverlap comparator compares `cam_index_`) |
| +0x18 / +0x1C | `exposure_time_` / `gain_` | c05 `extra_int_18_/1c_` |
| +0x20 | `bundle_id_` | |
| +0x0B0 / +0x0B1 | `is_keyframe_` / `flag_b1_` | |
| +0x0C0 | `R_imu_world_` (Eigen::Quaterniond) | |
| +0x0E0 / +0xE8 / +0xE9 | `mean_intensity_` / `is_too_dark_` / `is_too_bright_` | c11/c13 `img_mean_intensity_`, `img_too_dark_`; c10 `img_mean_`; c12 `is_dark_/is_bright_` |
| +0x0EA | `is_redundant_kf_` | c01 `near_map_kf_`, c10 "flag234" |
| +0x100 / +0x140 | `T_body_cam_` / `T_cam_body_` (accessors `T_imu_cam()` / `T_cam_imu()`) | c11 `T_cam_imu_`, c09 `T_imu_cam_` |
| +0x180 | `grid_cell_size_` (uint16) | c05 `flags_180_` |
| +0x188 | `grid_occupancy_` (std::vector<bool>) | c05 `extra_options_` |
| +0x1A8 / +0x1AC | `min_depth_` / `median_depth_` | c06 `depth_min_/depth_median_` |
| +0x1B0 | `level_reproj_thresh_` (vector<double> {1.5,3,6,12}) | c05 `level_scales_` |
| +0x1C8 | `feature_mask_` (cv::Mat) | c05 `original_color_image_` |
| +0x250 | `f_vec_raw_` (Bearings, 3xN float) | c12 `uv_vec_` (2xN — wrong), c11 `score_vec_` (wrong) |
| +0x2F8 | `seed_mu_range_` (float) | |
| +0x300 | `accumulated_w_T_correction_` | |

Frame functions: main ctor `Frame(cam, img, ts_ns, n_pyr_levels, int cam_index, int exposure_time,
int gain, const std::vector<bool>& grid_occupancy, uint16_t grid_cell_size)`;
`set_T_cam_imu(const Transformation& T_cam_imu, const Transformation& T_imu_cam)` (2 args, Pimax);
`set_T_w_imu` (renormalises); `numLandmarks()` counts **track ids > -1** (0x180118580);
`numLandmarksInBA()`, `numTrackedEdgelets()`, `numTrackedFeatures()`, `numTrackedLandmarks()`
(inline, see FrameBundle); `deleteLandmark(const size_t&)` (c12 "removeLandmark");
`isSaturatedPatch(const Vector2i&)` (c06 "isSaturated"); `w2c` 0x180095450 / `f2c` 0x1800955C0;
`getAngleError` = **0x180094540** (vslot 8), `getErrorMultiplier` = **0x180094570** (vslot 5)
— c05 had them swapped; `pos()` / `imuPos()` return **Vector3f**; `getSeedDepth`,
`getSeedPosInFrame`; `getTimestampSec()` divides by 1e9; the IMU Jacobians
(`jacobian_xyz2uv_imu`, `jacobian_xyz2img_imu`, `jacobian_xyz2f_imu`) are static inline
members in frame.h (c11's frame_jacobians.h is merged there).

### FrameBundle (`common/frame.h`, sizeof 0x100)
| offset | final name | draft names |
|---|---|---|
| +0x18 | `imu_measurements_` (ImuMeasurements) | |
| +0x40 | `is_relocalized_` | c09 `flag_64_`, c05 `is_keyframe_` |
| +0x50 | `T_W_B_init_` (Transformation) | c09 `T_80_`, c05 `T_W_B_` |
| +0x90/+0xA8/+0xC0 | `imu_vel_w_` / `imu_gyr_bias_` / `imu_acc_bias_` | |
| +0xD8 | `gravity_` (Vector3f) | c09 `vec_216_` |
| +0xE4 | `is_static_` | c01 `imu_stationary_`, c07 `is_stationary_` |
| +0xE5 | `low_feature_kf_` | c10 `force_stereo_triangulation_` |
| +0xE8 | `num_tracked_` (size_t) | |
| +0xF0 | `last_timestamp_sec_` | |
| +0xF8 | `is_keyframe_` | |
| +0xFC | `bundle_id_` | |

Counting functions, named by the decorated-name order inside frame.obj
(numFeatures < numLandmarks < numLandmarksInBA < numTrackedEdgelets < numTrackedFeatures <
numTrackedLandmarks):

| address | final name | semantics | draft names |
|---|---|---|---|
| 0x180095150 | `numFeatures()` | Σ num_features_ | |
| 0x180095180 | `numLandmarks()` | Σ track_id > -1 | c09/c07 numLandmarks, **c10 numTrackedIds**, c05 numTrackIds |
| 0x180095240 | `numLandmarksInBA()` | landmark && in_ba_graph_ | |
| 0x1800952D0 | `numTrackedEdgelets()` | landmark && type∈{0,4,6} | **c10/c05 numTrackedLandmarks** |
| 0x180095360 | `numTrackedFeatures()` | landmark \|\| corner/edgelet seed | |
| 0x1800953F0 | `numTrackedLandmarks()` | landmark != nullptr | **c10/c05 numLandmarks** |

Inline: `at()`, `size()`, `getMinTimestampNanoseconds()` (no CHECK),
`getMinTimestampSeconds()` (×1e-9), `get_T_W_B()`, `set_T_W_B()`, `setIMUState()` (writes the bundle
members), `setKeyframe()`, `isKeyframe()`.

### Point (`common/point.h`, sizeof 0x88)
`id_` +0, `pos_` (Vector3f) +4, `obs_` (unordered_map<int, KeypointIdentifier>) +0x10,
`n_failed_reproj_` +0x50, `n_succeeded_reproj_` +0x54, `last_structure_optim_` +0x58,
`ba_bundle_ids_` (std::set<int>) +0x60, `ba_inlier_count_` +0x70, `ba_total_count_` +0x74,
**`n_consecutive_obs_` +0x78** (c03 `ba_obs_frames_`, c10 `n_consecutive_tracked_`),
**`last_obs_bundle_id_` +0x7C**, `in_ba_graph_` +0x80. Ctor `Point(const Position&)`.
KeypointIdentifier: `frame` +0, `frame_id` +16, `bundle_id` +20, `keypoint_index_` +24.

### Misc
* `SeedRef` is in `common/feature_wrapper.h` (24 B; no extra member at +20 — that is padding).
* `FeatureWrapper::f_raw` (+0x38) is `Eigen::Ref<BearingVector>` (c12 "uv").
* `ImuMeasurement`/`ImuMeasurements` in `common/imu_calibration.h` (per A2 request).
* Sophus: `common/sophus/common.hpp` + `common/sophus/so3ex_base.h` (`so3ex.h` includes it).
  The ensure handler is `Sophus::defaultEnsure` (c03 called the 1-arg instance `ensureFailed`).
  `SO3d::exp(omega)` (0x18010A400, line 303) and `SO3d::exp(omega, eps)` (0x180042060) are
  overloads.
* Logger: `pimax::g_logger`, `LOGD/LOGI/LOGW/LOGE`; `Logger::Init(dir)` is defined in
  `common/logger.cpp` (binary: COMDAT in the headset TU, i.e. originally header-inline).
* vikit: `vk::cameras::CameraGeometryBase` (Pimax, no CHECKs), `NCamera(T_C_B, T_B_C, cams, label)`
  with `get_T_C_B()`, `get_T_B_C()`, `getCameraShared()`; `EquidistantDistortion(k1..k4, p1, p2)`
  (Fisheye62); `PinholeProjection::backProject3` has the Pimax principal-point shortcut.

## Where the draft functions went

### src/common
| file | content | from |
|---|---|---|
| `types.h`, `transformation.h`, `camera_fwd.h`, `camera.h` | aliases, FeatureType + predicates, `quaternionExp` | upstream svo_common + all chunks' assumed-type headers (c11_external.h, c12_assumed_types.h, c13_external.h) |
| `logger.h` / `logger.cpp` | Logger (inline Debug 0x18000C120, Error 0x18000C2C0, Info 0x18000F500, Warn 0x18000F6A0, TimeString 0x18000F310), `kLogTag` / `g_logger` (0x18046A000), `Logger::Init` 0x1801692E0 | c00 draft + c14 `common/logger_init.cpp` |
| `imu_calibration.h` | ImuMeasurement / ImuMeasurements | c09/c10/c11 (+ A2 request) |
| `feature_wrapper.h` | SeedRef (ctor 0x180181040), FeatureWrapper | c05 + c13 |
| `point.h` / `point.cpp` | PointIdProvider::last_id_ 0x18047DB60, KeypointIdentifier ctor 0x180099F40, Point ctor 0x180099F80, addObservation 0x18009A530, getCloseViewObs 0x18009A780, getTriangulationParallax 0x18009AA50, jacobian_xyz2f 0x18009ADE0 / jacobian_xyz2uv 0x18009B040 (inline), optimize 0x18009B240, removeObservation 0x18009BBE0, updateHessianGradientUnitPlane 0x18009BE10 / UnitSphere 0x18009C130 | c05 `point_c05_part.cpp` + c06 `point.cpp` (c06's `inline` duplicate of the KeypointIdentifier ctor dropped: one out-of-line definition, inlined by the compiler into addObservation as in the binary) |
| `seed.h`, `occupancy_grid_2d.h` | seed helpers (getSigma2FromDepthSigma COMDAT 0x1800A0A60), OccupandyGrid2D (ctor 0x1800AA2F0, dtor 0x1800AA440, reset 0x1800AAB80) | c06 |
| `frame.h` / `frame.cpp` | everything of frame.obj 0x180090180..0x180096E90 (see c05 table) + inline copies emitted elsewhere: imuPos 0x180110400, get_T_W_B 0x18010ADA0, numLandmarks 0x180118580 (c09); set_T_w_imu 0x1801283F0, setIMUState 0x180127700 (c10); Jacobians 0x18013BC70 / 0x18013B8F0 / 0x18013B300 (c11 frame_jacobians.h); getSeedDepth 0x1801420C0 (c12); set_T_W_B 0x1800162B0 (c01); FrameBundle::at 0x1800116D0 (std::vector::at) | c05 frame.h/.cpp + c09/c10/c11/c12/c13/c01 |
| `sophus/common.hpp` | FormatStream (0x18002E970 / 0x1800B7BA0 / 0x1800B8020 / 0x1800B8490), FormatString (0x18002EDE0 / 0x1800B8900), defaultEnsure (0x180037490 / 0x1800CBC40), SOPHUS_ENSURE, Constants | c07 (variadic version) + c03 |
| `sophus/so3ex_base.h` (+ `so3ex.h`) | alternatingSeries 0x1800409E0, SO3(q) 0x18002E0E0, operator* 0x18002E710, exp(omega,eps) 0x180042060, exp(omega) 0x18010A400, log 0x180042530, hat 0x180042410, matrix 0x1800423F0 | c03 + c09 |
| `so3_gamma.h` | gamma1/2/3, gamma1Right, dGammaTV2/3/4, dGammaV1/2/3 | c03 |

### src/direct
| file | content | from |
|---|---|---|
| `feature_detection_types.h` | Corner (20 B), DetectorType, DetectorOptions (80 B) | c06 |
| `feature_detection.h/.cpp` | AbstractDetector ctor 0x1800AA150, FastGradDetector::detect 0x1800AA6E0 | c06 |
| `feature_detection_utils.h/.cpp` | makeDetector 0x1800AD840, fillFeatures 0x1800ACB30, fastDetector 0x1800AC5E0, edgeletDetector_V2 0x1800AC200 (+ lambdas 0x1800ABEB0 / 0x1800ABB50), getAngleAtPixelUsingHistogram 0x1800AD4A0, smoothOrientationHistogram 0x1800ADA80 | c06 |
| `feature_alignment.h/.cpp` | align1D 0x1800A63E0, align2D 0x1800A84F0 | c06 |
| `patch_score.h`, `patch_utils.h` | ZMSSD<4> (ctor 0x1800ADF30), createPatchFromPatchWithBorder 0x1800AE5A0 | upstream svo_direct (c06/c07: identical); patch_utils debug helpers dropped |
| `patch_warp.h/.cpp` | getBestSearchLevel 0x1800B05B0, getWarpMatrixAffine 0x1800B0600, warpAffine 0x1800B0C10 | c07 |
| `matcher.h/.cpp` | depthFromTriangulation 0x1800AE610 (+lambda 0x1800AE150), findEpipolarMatchDirect 0x1800AE820, findLocalMatch 0x1800AF4B0, findMatchDirect 0x1800AF5E0, scanEpipolarUnitPlane 0x1800AFD70, updateZMSSD 0x1800B0270 | c06 `matcher_c06.cpp` (head) + c07 `matcher.cpp` (tail), header = c07 layout + c06/c13 corrections |
| `depth_filter.h/.cpp` | DepthFilter ctors 0x18009F280 / 0x18009F650, dtor 0x18009F940, GetFramesWithoutSeeds 0x18009FC10, addKeyframe 0x18009FE90, compareCornerScore 0x1800A0520, computeTau 0x1800A0530, detectFastCorners 0x1800A0750, initializeSeeds 0x1800A0AB0, isRotationMatrix 0x1800A2100, reset 0x1800A2300, stopThread 0x1800A2730, updateFilterVogiatzis 0x1800A2810, updateSeed 0x1800A2AC0, updateSeeds 0x1800A3270, updateSeedsLoop 0x1800A3630; Job dtor 0x18009FAF0 implicit | c06 |

## Conflicts resolved (common/, direct/)

1. **Frame +0x08**: c11/c13 declared `enable_shared_from_this` (+0..+16); impossible with the
   vtable at +0 and `id_` at +0x10. It is MSVC's 16-byte vfptr slot of a 16-aligned polymorphic
   class (same as AbstractInitialization / MiniLeastSquaresSolver, c11/c12) — no member.
2. **Frame +0x14**: c11 "bundle_id_" vs c05/c06/c01 camera index → `cam_index_` (ctor argument
   `i`, used as `frames_` index in initializeSeeds; bundle id is +0x20, set by the FrameBundle
   lambda and copied into KeypointIdentifier+0x14).
3. **Frame +0x180 / +0x188**: c05 "flags_180_ / FrameExtraOptions{vector<int>; 8 B}" vs c06
   "uint16 grid cell size / std::vector<bool>" → c06 (MSVC `vector<bool>` = word vector + bit
   count = 32 B; c09 passes `&grid_occupancy_` and `options_.frame_grid_param`).
4. **Frame +0x1C8**: c05 "original_color_image_?" vs c06 `feature_mask_` (clone of the camera mask,
   circles drawn, released) → c06.
5. **Frame +0x250**: c12 "2xN uv_vec_", c11 "score_vec_ (VectorXd)" vs c05/c13 "3xN f_vec_raw_" →
   3xN (resizeFeatureStorage resizes it with the 3xN conservativeResize 0x180096260; FeatureWrapper
   +0x38 has a 12-byte stride, c13).
6. **Frame +0x2F8**: c05 said no seed_mu_range_; c06 reads a float there → `seed_mu_range_`.
7. **Frame::getAngleError / getErrorMultiplier**: c05 table had 0x180094540 = getErrorMultiplier;
   disassembly: 0x180094540 jumps through vtable +0x40 (slot 8 getAngleError, caller updateSeed),
   0x180094570 through +0x28 (slot 5 errorMultiplier, caller PoseOptimizer::run) → swapped.
   Also consistent with the decorated-name order getAngleError < getEmptyFeatureWrapper <
   getErrorMultiplier.
8. **FrameBundle counting functions**: renamed by the decorated-name order of frame.obj (see
   table in "API for A2"); semantics unchanged.
9. **FrameBundle +0x40** (c05 is_keyframe_, c09 flag_64_, c10 is_relocalized_) → `is_relocalized_`
   (written true by reLocalize, c10); is_keyframe_ is +0xF8 (c09/c01). +0xE5 (c07
   low_feature_kf_ set in processFrame, c10 force_stereo_triangulation_ read in
   checkTrackingHealth) → `low_feature_kf_` (the writer decides).
10. **SeedRef +0x14 "extra int = 0"** (c05) → padding (the 2-arg ctor 0x180181040 does not write it;
    value-initialisation zero-fills).
11. **Point +0x50..+0x80**: c06 placeholders vs c03/c09/c10/c12 names → c03 for the BA fields,
    c09 for +0x78/+0x7C; +0x58 = upstream `last_structure_optim_` (only the ctor writes it).
12. **Matcher**: c07 sizeof 336 / defaults 0.7, 2.0 vs c06/c13 352 / 0.5, 2.5 → c06/c13 (malloc(0x160),
    explicit stores in the inlined ctor 0x18009F280: +32 = 0x3FE0000000000000, +48 =
    0x4004000000000000, +320..+343 zeroed → default member initialisers on f_cur_ and the extra
    Vector3f). +332 is a Vector3f (c06: copy of the un-normalised f_cur) — c12's "uv_cur_ Vector2f"
    rejected. The depth out-parameters are float (c06), the T-less findEpipolarMatchDirect overload
    does not exist (c06), `createPatchFromPatchWithBorder` is `patch_utils::` (upstream; c07 had
    `matcher_utils::`).
13. **vk::Timer::stop() int truncation** (c16): rejected, `cvtsi2sd xmm0, rax` is a 64-bit
    conversion (0x18002CC00); Timer is upstream.
14. **"Pimax minkindr exp threshold 1e-12"** (c05/c09/c11): the 1e-12 exp is not minkindr's (see
    third_party/minkindr/PIMAX_PATCH.md §4) → `quaternionExp()` helper; minkindr exp unchanged.
15. **ImuCalibration / ImuInitialization**: left to A2 (frontend/imu_processor.h) on request.
16. **Logger::Init placement**: kept in logger.cpp per coordinator instruction although the binary
    emits it as a COMDAT inside the headset TU (header-inline in the original).

## Vendored libraries
* `third_party/minkindr/PIMAX_PATCH.md`: operator* and inverse() renormalise; glog line numbers
  forced (59, 73, 102, 488/489/493/494); exp/operator*(RotationQuaternion)/cast unchanged.
* `third_party/vikit/PIMAX_PATCH.md`: cameras (c18 + c14 Fisheye62), solver (c12), timer/sample/
  performance_monitor/robust_cost (c18), getMedian without assert, unused files removed.

---

# Part B — sensor_fusion / plane / tracker (merged from the A1 helper)

## Integration A1 (fork) — sensor_fusion / plane / tracker

All files compile with `tools/cc.sh` (every header also standalone with `--header`); every known
layout is pinned by `static_assert`s. Every `project` row of the c17 function table and the four
Histogram functions of c16 (0x180197940 / 0x180197BB0 / 0x180197C20 / 0x180197CC0) occur exactly
once in `src/` with their address comment (checked by grepping all addresses).

## Where the drafts went

| src file | lines | from | content |
|---|---|---|---|
| sensor_fusion/deque.h | 138 | c17 | `pimax::common::Deque<T>` (0x1801A3ED0..0x1801A4280) |
| sensor_fusion/deque_holder.h | ~50 | c17 (fixed) | `pimax::DequeHolder` (= c14 `Unknown2840`), dtor 0x1801A3E00 |
| sensor_fusion/vector.h, rotation.h | 122 / 68 | c17 | Cardboard `Vector`, `Dot`/`Cross` (0x1801A5D10/D70/DA0), `Rotation` (0x1801A5A10, 0x1801A5B20) |
| sensor_fusion/filters.h/.cpp | 73 / 114 | c17 | Lowpass/Mean/Median filters |
| sensor_fusion/gyroscope_bias_estimator.h/.cpp | ~125 / 211 | c17 | `pimax::ThreeDof::GyroscopeBiasEstimator` (threshold 1000.0f) |
| sensor_fusion/imu_filter.h/.cpp | 91 / 161 | c17 | float Madgwick `ImuFilter` (world frame 3 = Pimax) |
| sensor_fusion/three_dof_tracker.h/.cpp | 129 / 165 | c17 | `ThreeDof::State` (80 B), `ImuSample` (32 B), `ThreeDofTracker` (0x2A0) |
| plane/plane.h | 144 | c17 (+ c08 names) | `Plane` (232 B), `PolygonVertex`, `LandmarkId(s)`, `LmkPositionMap`, `LmkPixelMap` |
| plane/histogram.h/.cpp | 103 / 246 | c16 + c17 merged | `Histogram` (0x1B0): ctor/dtor/operator= (c16), calculateHistogram/findPeaks/getLocalMaximum1D (c17) |
| plane/mesh.h/.cpp | 125 / 149 | c17 | Kimera `Mesh<>` without colours, explicit instantiations |
| plane/mesher.h/.cpp | 247 / 1232 | c17 | `Mesher` (0x4C8), mesher globals, `isPointInPolygon`, `polygonsOverlap`, `mergePolygons` |
| tracker/feature_tracking_types.h | 160 | upstream svo_tracker (new) | `FeatureTrackerOptions` (72 B), `FeatureRef`, `FeatureTrack`, `FeatureTracks` |
| tracker/feature_tracker.h/.cpp | 75 / 45 | c17 | `FeatureTracker` (152 B): ctor 0x1801A7DF0, reset 0x1801A8030, resetTerminatedTracks 0x1801A80F0 |

(c08's `plane/plane_types.h` is superseded by `plane/plane.h` + `plane/mesher.h`; c08's only
plane-related *functions* are FrameProcessorBase members — mergeSimilarPlanes 0x1800EC2B0,
updateGroundPlane 0x1800F2070, refinePlanes 0x1800F3430, refitWallPlane 0x1800F4190,
estimateGroundPlane 0x1800F5170, selectGroundPlane 0x1800F64A0 — and c09's
`computePolygonArea(const Plane&)` 0x1800FE140 lives in frame_processor_base.obj: all A2's.)
`IntegrationBase` (c09 draft `sensor_fusion/integration_base.h`) is NOT in sensor_fusion: A2 owns
it as `src/frontend/integration_base.h`.

## Conflicts resolved

1. **Histogram** (c16 vs c17 header): merged into one class; member names from c17
   (`unknown_f0_`, `unknown_150_`; c16 had `mat_240_`/`mat_336_`), `ranges_` typed
   `const float**` (c16 allocates `new const float*[n]`; c17 only reads it). No function was
   drafted twice, so no binary arbitration was needed.
2. **Plane** (c08 `plane_types.h` vs c17 `plane.h`): c17's version kept (it proves +48 =
   `unordered_map<int, Vector3f>` via 0x18019A5D0, +112 = float centroid, the inline ctor and
   `geometricEqual` 0x1801A13D0); c08's names reused. c08 placeholders `unknown_48`,
   `unknown_112`, `unknown_208` are `lmk_ids_map_`, `centroid_`, `is_valid_`.
3. **Mesher globals**: c17 names kept. c08's `g_wall_distance_tolerance` (0x18046A2B8) is
   `g_distance_tolerance_polygon_plane_association`; c00's `g_plane_lmk_ids` (0x18047EF68) is
   `g_new_plane_ids` (defined in mesher.cpp = TU51, as c00 says). The others coincide
   (`g_normal_tolerance_plane_plane`, `g_distance_tolerance_plane_plane`,
   `g_plane_distance_update_tolerance`, `g_max_plane_id`, `g_min_*`, `g_max_triangle_side`).
4. **c08's partial Mesher** called 0x18019DB90 `populate3dMesh` (public); it is
   `populate3dMeshTimeHorizon` (public), `populate3dMesh` 0x18019D000 is private (c17).
5. **ThreeDof::ImuSample layout**: c14 `{double t; acc; gyr}` vs c17 `{acc +0; gyr +12; t +24}`.
   Verified in the binary: SLAMManager::UpdateThreeDof 0x18016B210 builds the sample on the stack
   as {ImuMeasurement+20 (acc), ImuMeasurement+8 (gyr), timestamp} → **c17 is right**.
6. **ThreeDof::State** is 80 bytes (Quaternionf at +48 forces 16-byte alignment, t at +64; the
   tracker has state_ at +128 and last_imu_ at +208); c14's `static_assert(sizeof == 72)` is
   wrong (the copy in UpdateThreeDof moves 72 data bytes).
7. **DequeHolder** layout: the c17 draft put the first Deque at +208 (string ends at +208,
   no padding). Dtor 0x1801A3E00 frees the Deques at +216/+272/+424 and the vector at +400:
   an 8-byte unknown member `pod_208_` was added; offsets now pinned. sizeof is ≥ 456; c14
   records 0x200 for `Unknown2840` (trailing members unknown, TODO(verify)).
8. **FeatureTracker** draft included `svo/...` paths; `tracker/feature_tracking_types.h` was
   created from upstream (upstream-identical layout, 72-byte options; c17 §2.6) since no chunk
   drafted it.

## Names A2 / the coordinator must use

* SLAMManager (c14 `slam_manager.{h,cpp}`):
  - `std::unique_ptr<ThreeDof::ThreeDofTracker> three_dof_` (+2832) — include
    `sensor_fusion/three_dof_tracker.h`; delete c14's local `State`/`ImuSample`/`Config` decls.
  - c14 `ThreeDof::Config` / `config_` = **`ThreeDof::ImuSample` / `last_imu_`** (+208).
  - `ImuSample` brace order is **`{acc, gyr, t}`** (c14 wrote `{m.timestamp, acc, gyr}`).
  - `SetBias(const Vector3f& bg, const Vector3f& ba)` 0x1801A7A50 (c17 notes mention a
    "SetImuExtrinsics" name; the c14 draft already uses SetBias).
  - `Propagate(const ImuSample& last_imu, const std::vector<ImuSample>&, const State& in, State* out)`.
  - `std::unique_ptr<pimax::DequeHolder>` (+2840) instead of `Unknown2840`
    (`sensor_fusion/deque_holder.h`).
* FrameProcessorBase: `std::shared_ptr<Mesher> mesher_` (+3592, `plane/mesher.h`),
  `std::vector<Plane>` members (`plane/plane.h`); public Mesher API:
  `populate3dMeshTimeHorizon(...)`, `clusterPlanesFromMesh(...)`, members `mesh_3d`,
  `mesh_output`, `z_hist_`, `R_w_c_` (+0x450), `t_w_c_` (+0x474), `lmk_px_` (+0x480),
  `is_first_frame_`; free functions `polygonsOverlap`, `mergePolygons`, `isPointInPolygon`;
  `Mesh2D`/`Mesh3D` from `plane/mesh.h` (Mesh2D(3) is built by 0x1800F2070).
  `computePolygonArea` 0x1800FE140 is NOT declared here (A2's frame_processor_base).
* AbstractInitialization: `FeatureTracker(const FeatureTrackerOptions&, const DetectorOptions&,
  const CameraBundlePtr&)`, `reset()`; `FeatureTrackerOptions` in
  `tracker/feature_tracking_types.h` (factory loadTrackerOptions 0x18015E170).

## Line numbers forced with `#line` (verified by preprocessing)

* mesher.cpp glog CHECKs: 796, 798 (updatePlanesLmkIdsFromMesh), 982, 983, 999, 1010, 1011
  (segmentPlanesInMesh); the two multi-line CHECK_EQs were joined onto one line.
* sensor_fusion/deque.h `assert`s (live `_wassert`): push_back 111, push_front 124, pop_front 138,
  pop_back 152, operator[] 168, from_back 178, operator[] const 189, from_back const 199.

## Remaining TODO(verify)

* Owning objects / out-of-line placement: `DequeHolder::~DequeHolder` and the `Rotation` /
  `Vector` functions are header-inline here, but the binary emits them inside the sensor_fusion
  region (0x1801A3E00, 0x1801A5A10..0x1801A5DA0) — they may have been defined in a
  sensor_fusion .cpp (c00 counts 13 objects for TU52..64).
* All TODO(verify)s of the c17 notes §7 (globals' names, Histogram path/extra Mats, LmkPixelMap
  value type, ThreeDofTracker member names).
* DequeHolder trailing size (0x200 per c14?) and Deque capacities (ctor never emitted).

---

# Part C — DBoW2 / QuickLZ / fast / portable_archive (merged from the A1 helper)

## Integration A1 (vendor part) — DBoW2, QuickLZ, fast, portable archive

All files compile with tools/cc.sh (DBoW2/src/*.cpp, fast/src/*.cpp, quicklz.c,
src/common/portable_archive/*.cpp; headers with --header). A test TU
(build-win/a1_scratch/t_dbow.cpp, CC_OBJ=1) instantiates `OrbVocabulary` (ctor(string), load,
fromStream, transform×2, score) and pins sizeof(OrbVocabulary) == 80, sizeof(Node) == 152,
Node::descriptor +48, Node::word_id +144.

## third_party/DBoW2 (details: third_party/DBoW2/PIMAX_PATCH.md)
* `TemplatedVocabulary::load(const std::string&)` replaced by the Pimax/DBoW3 version
  (0x180176910): binary signature 88877711233 → `fromStream`, else cv::FileStorage.
* New public non-virtual `void TemplatedVocabulary::fromStream(std::istream& str)` (0x180175D80),
  checked against the decompilation (read order k, L, scoring, weighting; nnodes==0 early return;
  QuickLZ chunks of 10000 bytes).
* New `DBoW2::DescManip::fromStream(cv::Mat&, std::istream&)` (`DBoW2/DescManip.h`,
  src/DescManip.cpp; 0x1801B6D90).
* Removed: TemplatedDatabase.h, QueryResults.h/.cpp (not in the image). `DBoW2/DBoW2.h` now
  includes TemplatedVocabulary/BowVector/FeatureVector/FORB only and defines only the global
  `typedef ... OrbVocabulary`.
* src/*.cpp include `"DBoW2/xxx.h"` (only third_party/DBoW2/include is on the include path).

## third_party/quicklz
Unchanged; byte-identical to DBoW3's QuickLZ 1.5.0 with QLZ_COMPRESSION_LEVEL 1 /
QLZ_STREAMING_BUFFER 0 (state 0x9008 B as memset in the binary). 0x1801B70D0 is
`qlz_size_compressed` (c18 mislabelled it qlz_size_decompressed).

## third_party/fast (details: third_party/fast/PIMAX_PATCH.md)
* src/faster_corner_10_sse.cpp = c17 draft (MSVC wrapper: w<22 → plain detector, else h<7 →
  return, else plain detector; disassembly of 0x1801AFA30 checked — c17's header comment had the
  condition garbled, fixed).
* Removed headers corner_9.h, corner_10.h, faster_corner_utilities.h (SSE2-only / FAST-9).

## src/common/portable_archive
Boost 1.74 `libs/serialization/example` portable_binary archive (c06 copy):
portable_binary_archive.hpp, portable_binary_iarchive.hpp/.cpp, portable_binary_oarchive.hpp/.cpp.
* c06 vs c15 drafts were identical except `portable_binary_iarchive::init()`: c15 kept the
  example's `BOOST_ARCHIVE_VERSION() < input_library_version → unsupported_version` throw; the
  binary (0x18009CFC0) has the signature check (archive_exception code 3 = invalid_signature)
  and the 2-byte library-version load with the `library_version_type` range assert only, no
  call to BOOST_ARCHIVE_VERSION and no unsupported_version (code 4) throw → c06 version kept
  (check commented out). Quirk: archives of any library version are accepted.
* `portable_binary_iarchive_exception::what()` assert(false) stays at
  portable_binary_iarchive.hpp line 57 (header identical to the boost example).
* load_impl 0x18009D3B0, load_override(class_name_type&) 0x18009D4F0, save_impl 0x18009DA50:
  identical to the example.

## For A2 (loop_closing)
* Include `<DBoW2/DBoW2.h>` (or TemplatedVocabulary.h + FORB.h). `OrbVocabulary` exists as a
  global typedef; c15's `pimax::totem::OrbVocabulary` alias of the same type is compatible.
* Vocabulary API used by the binary: `OrbVocabulary(const std::string& filename)` (0x180177110,
  calls the Pimax `load(filename)`), `load(const std::string&)`, `fromStream(std::istream&)`,
  `transform(const std::vector<cv::Mat>&, BowVector&)` / `(…, BowVector&, FeatureVector&, int
  levelsup)`, `score(const BowVector&, const BowVector&)`, `getParentNode`, the transform with
  `NodeId*`/levelsup used by getNodeID.  Exceptions: load throws `std::runtime_error` (binary
  file cannot be opened / bad binary signature) or `std::string` (FileStorage path).
* Portable archive includes: `"common/portable_archive/portable_binary_iarchive.hpp"`,
  `"common/portable_archive/portable_binary_oarchive.hpp"` (classes at global scope:
  `portable_binary_iarchive`, `portable_binary_oarchive`, `portable_binary_iarchive_exception`).

## Open TODO(verify)
* Original class/file name of the descriptor reader (DBoW3 `DescManip` kept).
* Exact source form of the fast SSE2 wrapper.
* Whether the PlatMap save path inlines `portable_binary_oarchive::init` (c06 found no
  out-of-line copy) — nothing to do in source.

---

# Open issues / for the coordinator
* Names marked TODO(verify) in the headers (Frame +0x18/+0x1C/+0xB1/+0xEA/+0x1B0/+0x1C8/+0x250,
  FrameBundle +0x40/+0x50/+0xD8/+0xE5/+0xE8, Point +0x58/+0x78/+0x7C, Matcher +280/+288/+332,
  FrameBundle::numTrackedEdgelets, Frame::w2c/f2c, quaternionExp).
* `Quaternion = Eigen::Quaterniond` is a reconciliation decision (evidence in transformation.h);
  A2's code must use `kindr::minimal::RotationQuaternionTemplate<double>` explicitly where the
  binary calls the checking kindr ctor.
* Logger::Init is out of line in logger.cpp (binary: COMDAT in the headset TU).
* `__FILE__` strings will differ from the original `E:\code_codex\...` paths; only `__LINE__`s were
  forced (minkindr, Sophus, vikit solver, mesher.cpp, sensor_fusion/deque.h).
* All my .cpp files compile to objects (CC_OBJ=1) and every pimax/vk/Sophus symbol they reference
  is defined within them (checked with llvm-nm); not linked against A2's objects yet.
