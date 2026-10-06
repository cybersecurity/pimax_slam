# Integration B4 — src/loop_closing/*.cpp, src/interface/*.cpp (except ceres_backend_factory)

Status: every file below compiles with `tools/cc.sh` (syntax) and with `CC_OBJ=1` (objects in
`build-win/cc/src/{loop_closing,interface}/`).  Symbol closure checked with `llvm-nm --undefined-only`
against my objects + every other object present in `build-win/cc` (A1's common/direct/sensor_fusion/
portable_archive/vikit/DBoW2/quicklz objects were built for that with `CC_OBJ=1`): **all
pimax/vk/kindr/DBoW2/quicklz symbols resolve except 5 that B2 must provide** (list below).
Every `project` row of the c14/c15/c16 function tables carries its `// 0x1800XXXXX` comment in `src/`
(table at the end).  The 10 `Headset*` exports are defined with C linkage in headset_api.obj and match
`src/pimax_slam.def` exactly; `HeadsetGetVersion()` returns `"Pimax_SLAM_2.0.0.1"`.

## 1. Files

| file | lines | from drafts | content |
|---|---|---|---|
| src/loop_closing/loop_closing.cpp | 2165 | c00 loop_closing_globals.cpp, c15 loop_closing_ctor.cpp, c16 loop_closing.cpp | globals `kStrToScaleRetMap`, `kStrToGlobalMapType`, `kPlatMapVersion` (now non-static, extern in platmap.h; definition order = .CRT$XCU #155/#156/#157); LoopClosing ctor 0x18017F730 / dtor 0x180181AE0 (implicit LoopClosureOptions copy ctor 0x180180CD0); all c16 functions 0x180185E40..0x180196B60 in address order incl. `recovery_kf` 0x18018C070 and `runReLocalization` 0x18018D770 (complete) |
| src/loop_closing/platmap.cpp | 417 | c15 platmap.cpp | KeyFrame ctor 0x18017F4C0, PlatMap UpdateMap/BuildMapIndex/load/loadIndex/load_bin/save/save_bin(dead)/saveIndex/Map2Vec/Vec2Map |
| src/loop_closing/bow.cpp | 255 | c15 bow.cpp | changeStructure, compareBOWs, createBOW, extractBoWFeaturesFromImage, extractFeaturesFromSVOKeypoints, getNodeID (DBoW2 vocabulary instantiated here, as in the binary) |
| src/loop_closing/geometric_verification.cpp | 43 | c15 | commonLandMarkCheck 0x180178E40 |
| src/loop_closing/map_alignment.cpp | 21 | c16 | MapAlignmentSE3 ctor 0x1801975B0 |
| src/loop_closing/beblid.cpp (+ beblid.p256.hpp, beblid.p512.hpp: table fragments included inside the ctor body, not standalone headers) | 276 | c14 beblid.cpp | BEBLID ctor/create/compute/getDefaultName, rectifyABWL, computeABWLResponse, file-local `ParallelLambdaWrapper`; the two table files are verbatim copies of opencv_contrib-4.5.3 (c14: byte-identical to 0x1803B9220/0x1803BC220) |
| src/interface/headset_api.cpp | 207 | c14 | the 10 exports, `g_system` (0x18047EDF0), `g_pimax_slam_version` (off_18046A1D8), put-image counter |
| src/interface/slam_manager.cpp | 798 | c14 | SLAMManager (all 16 functions), SetThreadName, Sinc |
| src/interface/svo_factory.cpp | 534 | c14 | factory option loaders, getImuProcessor, getLoopClosingModule, setInitialPose, FibonacciSphere, SplitPath, makeFrameProcessor (device_calibration.xml) |
| src/interface/slam_manager.h | (A2) | | only a comment added (ImageBundle implicit dtor 0x180167260) |

Not turned into files:
* **c07 loop_closing/serialization_part.cpp**: everything in it (cv::Mat / BowVector / Transformation
  save+load, KeyFrame::serialize) is already in `loop_closing/serialization_helpers.h` and
  `loop_closing/platmap.h` (A2), bodies identical → nothing left to add.
* c15 `loop_closing/utility.h` (Utility::ypr2R(ypr, bool) 0x18017EE10) → A2's `frontend/utility.h`.
* c15 `loop_closing_c15.h`, c16 `loop_closing.h`/`map_alignment.h`, c14 headers → A2's headers.
* c14 `common/logger_init.cpp` (Logger::Init 0x1801692E0 + sort lambda 0x180167950) → A1's common/logger.cpp.
* c14 `thirdparty/vikit/equidistant_distortion.h` → A1's third_party/vikit.
* c16 `plane/histogram.*` → A1's src/plane/histogram.* (0x180197940/BB0/C20/CC0).

## 2. Conflicts resolved / integration changes (with evidence)

1. **KeyFrame cam id = `Frame::cam_index_`** (Frame+0x14).  c16 wrote `frame->bundle_id_` (c11's name for
   +20) in addFrameToPR, reLocalize and svoFrameToKeyframe; A1 established +0x14 = `cam_index_`
   (bundle id is +0x20).  KeyFrame ctor args are (FrameBundle::getBundleId() (+0xFC), Frame+0x14,
   Frame::id_, map_id_) — consistent with the KeyFrame field name `cam_id_`.
2. **LoopClosing ctor: `beblid_(BEBLID::create(256, 0.75f))`** instead of c15's
   `std::make_shared<BEBLID>(256, 0.75f)`: the ctor calls 0x180170570 with (ret, 256, 0.75) by value,
   and 0x180170570 is the out-of-line `BEBLID::create` (operator new(0x58) + ctor 0x18016F2F0).
3. **FeatureWrapper argument order in runReLocalization** (c16 TODO): verified on 0x18017F430 (stores
   arg4 at +0x20 = `f`, arg5 at +0x38 = `f_raw`) and the call site: `f` = un-normalized bearing,
   `f_raw` = normalized.  Kept; TODO removed.
4. **Reloc depth is `FloatType`** (float): `Matcher::findEpipolarMatchDirect(..., FloatType& depth)` (A1)
   and the binary keeps it in a float (v451); c16 had `double`.  1/d, 1/(1.2d), 1/(0.8d) are still
   computed in double as in the binary.
5. **R2ypr / ypr2R**: c16's free forward declarations replaced by A2's `Utility::R2ypr(R, false)`
   (0x1800F3260) and `Utility::ypr2R(ypr, false)` (0x18017EE10) from frontend/utility.h.
6. **LoopClosing member names**: c15's `K_list_/D_list_/enable_save_map_` → A2's `K_/D_/save_map_enabled_`.
7. **`kPlatMapVersion`** (c00: `static`) is defined non-static in loop_closing.cpp (declared
   `extern const std::string` in platmap.h).  Checked with llvm-nm: external `B` symbol in
   loop_closing.obj, `U` in platmap.obj.
8. **VLOG line numbers**: the 13 `VLOG(40)` sites of loop_closing.cpp get the binary's `__LINE__`
   (checked in the pseudocode: 1351, 658, 694, 707, 723, 1952, 1970, 1981, 1992, 2029, 2035, 2066,
   2638) through `#line N` + a restoring `#line` after each statement; verified by preprocessing
   (these are the only `__LINE__` uses of the TU).
9. **BEBLID `ParallelLambdaWrapper`**: c14 included a non-existent `common/parallel_lambda_wrapper.h`.
   vtables.json shows a separate RTTI vtable `ParallelLambdaWrapper` (global ns, 0x1803B8E38) whose
   slots are the same functions as `cv::ParallelLoopBodyLambdaWrapper` (0x180093630 / 0x1800935C0,
   ICF-folded) → defined as a file-local class in beblid.cpp with the same body as OpenCV's wrapper.
10. **ReprojectorOptions factory values** (A2 §2.14): `pimax_unknown_24 = 20; max_n_kfs = 19`.
11. **DetectorOptions factory** (A1 names): +40 `disable_edgelets = false`, +48 `threshold_edgelet =
    200.0`, +56/+60 `sampling_level = level = 0` (one qword store); **BaseOptions** +248 `grid_size = 20`.
12. **getImuProcessor biases**: ImuProcessor +208 = `acc_bias_` ← aBias, +232 = `omega_bias_` ← wBias
    (c10 layout) — c14's "possible aBias/wBias swap" TODO is resolved: no swap.
13. **getLoopClosingModule**: `uint8_t loc_mode` → `static_cast<LoopClosingMode>(loc_mode)` for A2's
    ctor `(options, cams, const std::string& map_tag, LoopClosingMode)` (same codegen).
14. **setInitialPose**: no `setInitialImuPose()` in A2's FrameProcessorBase → `vo.T_world_imuinit = T`
    (binary 0x1801621A0: checking RotationQuaternion ctor 0x1800089C0 then member copy to +400).
15. **makeFrameProcessor**: `vk::TransformationVector` (vikit's namespace), kindr ctor
    `Transformation(Eigen::Quaterniond, Vector3d)` = 0x180022CE0 (calls 0x1800089C0) — c14 wrote a
    kindr `Quaternion(...)` wrapper which with A1's `Quaternion = Eigen::Quaterniond` is the same call.
16. **SLAMManager → frontend names (A2)**: `getNCamera()` → `cams_`; `imu_processor_` (FP+520) →
    `imu_handler_`; `state_mutex_/state_` → `output_mutex_/output_`; `ground_mutex_/ground_valid_/
    ground_point0_/ground_point1_` → `plane_mutex_/plane_valid_/plane_imu_pos_/plane_center_`;
    `loc_state_mutex_` → `loc_mutex_`; `need_reset_` → `set_reset_`; `getLastFrames()` → `last_frames_`;
    PoseState `T_odom_imu/T_world_odom/tracking_flag/reset` → `T_world_imu/T_map_world/status/
    backend_static`; ImuMeasurement fields with trailing `_`; `getMeasurements(t, imus, true)` →
    `getMeasurementsContainingEdges(t, imus, true)` (0x18012A950); `trimMeasurements()` →
    `limitMeasurementsSize()` (0x18012B090); backend `quitThread()` → `CeresBackendInterface::reset()`
    (0x1800149F0 only logs "Backend: Reset").
17. **SLAMManager ↔ ThreeDof (A1)**: c14's local State/ImuSample/Config removed; `UpdateThreeDof` copies
    the tracker's `last_imu_` (+208, c14 "config_") and builds `ImuSample{acc, gyr, t}`;
    `std::unique_ptr<DequeHolder>` instead of `Unknown2840`.
18. **HeadsetGetVersion** loads a data pointer (off_18046A1D8) → `const char* g_pimax_slam_version`
    with external linkage (an internal-linkage pointer would be constant-folded away).
19. **PairHash**: bow.cpp uses the single global `PairHash` of loop_closing/pair_hash.h (c15 had a
    private copy; the binary has one functor 0x180170E50).

## 3. Unresolved externals expected from other groups

After building every object that exists in build-win/cc (A1 + whatever B1/B2/B3 had built), only these
are undefined (all **B2**, frontend/frame_processor_base.cpp / frame_processor.cpp):

| symbol | used by |
|---|---|
| `FrameProcessor::FrameProcessor(const BaseOptions&, const DepthFilterOptions&, const DetectorOptions&, const InitializationOptions&, const StereoTriangulationOptions&, const ReprojectorOptions&, const FeatureTrackerOptions&, const CameraBundlePtr&, const std::string&, const uint8_t&)` 0x1800B2460 | svo_factory.cpp makeFrameProcessor |
| `FrameProcessorBase::addImageBundle(std::vector<cv::Mat>&, const std::vector<uint32_t>&, const std::vector<uint32_t>&, const uint64_t&, const ImuMeasurements&, const int&, const Eigen::Quaternionf&, bool, double)` 0x1800FB650 | slam_manager.cpp ProcessLoop |
| `FrameProcessorBase::setBundleAdjuster(const std::shared_ptr<CeresBackendInterface>&)` 0x1801274A0 | SLAMManager ctor |
| `FrameProcessorBase::setRotationPrior(const Eigen::Quaterniond&)` 0x180128210 | SetImuPrior |
| `FrameProcessorBase::setRotationIncrementPrior(const Eigen::Quaterniond&)` 0x1801280A0 | SetImuPrior |

(plus, implicitly through `make_shared<FrameProcessor>`, FrameProcessor's vtable entries.)
Already provided by objects of other groups at check time: B1 `CeresBackendInterface::reset/setImu`,
`ceres_backend::Map` (ctor, addParameterBlock, addResidualBlock, setParameterBlockConstant(id)),
`PoseLocalParameterization` virtuals, `PoseParameterBlock(T, id)` / `estimate()`,
`LocalParamizationAdditionalInterfaces::verify`; B3 `ImuProcessor` (ctor/dtor, addImuMeasurement,
getMeasurementsContainingEdges, getRelativeRotationPrior, getInitialAttitude, waitTill,
limitMeasurementsSize), `ceres_backend_factory::makeBackend`.  A1: Frame/frame_utils, Logger::Init,
Matcher, DepthFilter::stopThread, ThreeDofTracker, GyroscopeBiasEstimator, vikit cameras, DBoW2,
DescManip, quicklz, portable archives.

## 4. Remaining TODO(verify)

* File split / names (no behavioural effect): PlatMap + KeyFrame bodies lie inside loop_closing.obj in
  the image (platmap.cpp here); commonLandMarkCheck 0x180178E40 also lies in that range
  (geometric_verification.cpp here); BEBLID TU name; headset_api.cpp / slam_manager.cpp /
  svo_factory.cpp names; `g_pimax_slam_version`, `kPlatMapVersion`, `ParallelLambdaWrapper` home.
* `PlatMap::save_bin` body (dead-stripped; only its oserializer singleton is evidence).
* All names marked TODO(verify) in A2's headers that these sources use (LoopClosureOptions `unk_*`,
  LoopClosing `i272_`/`vec_1256_`/`deque_1280_`, KeyFrame `T_unk_192_`/`mat_256_`, ReprojectorOptions
  `pimax_unknown_24` and `cell_size`, DetectorOptions names, HeadsetGroundState point meaning).
* DequeHolder (+2840) stays empty (only reset to null in the ctor, never created).
* Pangolin: c00 found two Pangolin `StaticHandler`/`StaticHandlerScroll` atexit statics in TU40,
  TU41 (headset API) and TU42 (BEBLID/DBoW2 area): the original TUs include Pangolin's handler.h.
  Not reproduced (no Pangolin in the build; no code uses them).
* Logger::Init is out of line in common/logger.cpp (binary: COMDAT in the SLAMManager TU) — A1 decision.

## 5. Quirks deliberately kept (see the drafts' notes for the full lists)

LoopClosing: threads started before `plat_map_` exists; `.bk` vs `-bk` backup check; `stopAllThread`
dereferences `plat_map_` unconditionally; getReLocCorrection's averaging loop never advances; no BoW
score threshold in runReLocalization; leaked HuberLoss; corrections accumulate over candidates;
DRAW mode blocks in `waitKey(0)`; triangulate returns true for points behind cam1; saveTagIndex drops
`map_version_` and writes keys with trailing spaces; resetReLocalize's swapped LOGI labels; `%d` with
size_t.  PlatMap: load rethrows / load_bin swallows, UB read of `kf_list_[0]` after a failed load,
UpdateMap reads `kfs[0]` without empty check, Map2Vec ignores its argument.  bow: appending
changeStructure / extractBoWFeaturesFromImage, divergent svo vector lengths.  Interface: all c14
quirks (skip first 5 image calls, SetDeviceEnable/SetTrackingMode stop the vision thread for good,
"%lu" with doubles, wrong log texts, aBias/wBias only into the tracker biases, ...).

## 6. Function accounting (every `project` row of c14/c15/c16; "home" = where the definition and its
## address comment live)

### c14_interface_api

| address | function | upstream status | home |
|---|---|---|---|
| 0x180159E10 | pimax::totem::LoopClosureOptions::LoopClosureOptions | modified: +Pimax fields 256..655 | src/interface/svo_factory.cpp |
| 0x18015A4F0 | pimax::totem::LoopClosureOptions::~LoopClosureOptions | - | src/interface/svo_factory.cpp |
| 0x18015A8A0 | pimax::totem::ImuParams::operator= | - | src/interface/svo_factory.cpp |
| 0x18015AEC0 | pimax::totem::factory::FibonacciSphere | new | src/interface/svo_factory.cpp |
| 0x18015B0E0 | pimax::totem::factory::SplitPath | new | src/interface/svo_factory.cpp |
| 0x18015B690 | vk::cameras::CameraGeometry<PinholeProjection<EquidistantDistortion>>::backProje | modified: x==y==0 shortcut, undistort(x,y,fx,fy) | third_party/vikit/include/vikit/cameras/equidistant_distortion.h |
| 0x18015B760 | vk::cameras::EquidistantDistortion::distort | modified: Fisheye62 tangential, Horner | third_party/vikit/include/vikit/cameras/equidistant_distortion.h |
| 0x18015B960 | vk::cameras::EquidistantDistortion::jacobian | modified: tangential chain rule | third_party/vikit/include/vikit/cameras/equidistant_distortion.h |
| 0x18015BD50 | vk::cameras::EquidistantDistortion::undistort | rewritten: Gauss-Newton, 10 its, -1000 on failure, <4 px skip | third_party/vikit/include/vikit/cameras/equidistant_distortion.h |
| 0x18015CA90 | CameraGeometry<...>::getDistortionParameters | modified: 6 params | third_party/vikit/include/vikit/cameras/equidistant_distortion.h |
| 0x18015CBD0 | pimax::totem::factory::getImuProcessor | modified (getImuHandler): literal calibration, biases from XML | src/interface/svo_factory.cpp |
| 0x18015CEE0 | pimax::totem::factory::getLoopClosingModule | modified | src/interface/svo_factory.cpp |
| 0x18015D6E0 | pimax::totem::factory::loadBaseOptions | modified: literals | src/interface/svo_factory.cpp |
| 0x18015DA90 | pimax::totem::factory::loadDepthFilterOptions | modified: literals, float thresholds | src/interface/svo_factory.cpp |
| 0x18015DAD0 | pimax::totem::factory::loadDetectorOptions | modified: literals | src/interface/svo_factory.cpp |
| 0x18015DB40 | pimax::totem::factory::loadInitializationOptions | modified: literals | src/interface/svo_factory.cpp |
| 0x18015DB90 | pimax::totem::factory::loadLoopClosureOptions | modified | src/interface/svo_factory.cpp |
| 0x18015E0B0 | pimax::totem::factory::loadReprojectorOptions | modified: literals | src/interface/svo_factory.cpp |
| 0x18015E130 | pimax::totem::factory::loadStereoOptions | modified: literals | src/interface/svo_factory.cpp |
| 0x18015E170 | pimax::totem::factory::loadTrackerOptions | identical values | src/interface/svo_factory.cpp |
| 0x18015E250 | pimax::totem::factory::makeFrameProcessor | new (device_calibration.xml) + makeStereo | src/interface/slam_manager.cpp, src/interface/svo_factory.cpp |
| 0x180161900 | CameraGeometry<...>::printParameters | modified: "Distortion: Fisheye62(" 6 params | third_party/vikit/include/vikit/cameras/equidistant_distortion.h |
| 0x1801621A0 | pimax::totem::factory::setInitialPose | modified: literal identity | src/interface/svo_factory.cpp |
| 0x180162340 | HeadsetGetGroundState | new | src/interface/headset_api.cpp |
| 0x180162360 | HeadsetGetLocModeState | new | src/interface/headset_api.cpp |
| 0x180162380 | HeadsetGetVersion | new | src/interface/headset_api.cpp |
| 0x180162390 | HeadsetInitialImpl | new | src/interface/headset_api.cpp |
| 0x180162510 | HeadsetLeftImageNumber | new | src/interface/headset_api.cpp |
| 0x180162530 | HeadsetPutCameraImage | new | src/interface/headset_api.cpp |
| 0x180162D70 | HeadsetPutHmdImuData | new | src/interface/headset_api.cpp |
| 0x180162DF0 | HeadsetReleaseImpl | new | src/interface/headset_api.cpp |
| 0x180162E30 | HeadsetSetDeviceEnable | new | src/interface/headset_api.cpp |
| 0x180162E60 | HeadsetSetTrackingMode | new | src/interface/headset_api.cpp |
| 0x1801662D0 | pimax::ThreeDof::State::State | new | src/interface/slam_manager.cpp |
| 0x1801663F0 | pimax::totem::SLAMManager::SLAMManager | modified (SvoInterface ctor) | src/interface/slam_manager.cpp |
| 0x180167260 | pimax::totem::ImageBundle::~ImageBundle | - | src/interface/slam_manager.h |
| 0x180167320 | pimax::totem::SLAMManager::~SLAMManager | modified | src/interface/slam_manager.cpp |
| 0x180167790 | pimax::totem::PoseState::operator= | - | src/interface/slam_manager.h |
| 0x180167950 | Logger::Init sort lambda (last_write_time(a) < last_write_time(b)) | new | src/common/logger.cpp |
| 0x180167C60 | pimax::totem::SLAMManager::GetGroundState | new | src/interface/slam_manager.cpp |
| 0x180167DB0 | pimax::totem::SLAMManager::PopImageBundle | new | src/interface/slam_manager.cpp |
| 0x1801680D0 | pimax::totem::SLAMManager::PutImages | new | src/interface/slam_manager.cpp |
| 0x180168640 | pimax::totem::SLAMManager::PutImu | new (SvoInterface::imuCallback heritage) | src/interface/slam_manager.cpp |
| 0x1801691D0 | pimax::totem::SLAMManager::SetDeviceEnable | new | src/interface/slam_manager.cpp |
| 0x1801692E0 | pimax::Logger::Init | new (6DOF_ variant of LedObjectPoseEstimator) | src/common/logger.cpp |
| 0x18016A220 | pimax::totem::SLAMManager::SetTrackingMode | new | src/interface/slam_manager.cpp |
| 0x18016A280 | pimax::totem::SLAMManager::Stop | new | src/interface/slam_manager.cpp |
| 0x18016A3B0 | pimax::totem::SLAMManager::ClearImageQueue | new | src/interface/slam_manager.cpp |
| 0x18016A460 | pimax::totem::SLAMManager::ProcessLoop | modified (SvoInterface::stereoLoop/Callback) | src/interface/slam_manager.cpp |
| 0x18016B210 | pimax::totem::SLAMManager::UpdateThreeDof | new | src/interface/slam_manager.cpp |
| 0x18016B860 | pimax::totem::SLAMManager::GetLocModeState | new | src/interface/slam_manager.cpp |
| 0x18016B940 | pimax::totem::SLAMManager::PredictPose | new | src/interface/slam_manager.cpp |
| 0x18016D1D0 | pimax::totem::SLAMManager::Propagate | new (VINS mid-point variant) | src/interface/slam_manager.cpp |
| 0x18016EC80 | pimax::totem::SLAMManager::SetImuPrior | modified (SvoInterface::setImuPrior) | src/interface/slam_manager.cpp |
| 0x18016EED0 | pimax::totem::(anon)::SetThreadName | new | src/interface/slam_manager.cpp |
| 0x18016F2F0 | BEBLID::BEBLID | modified (BEBLID_Impl ctor) | src/loop_closing/beblid.cpp |
| 0x18016F540 | BEBLID::compute lambda body (operator()) | identical (upstream lambda) | src/loop_closing/beblid.cpp |
| 0x18016FC40 | BEBLID::`vector deleting dtor' | - | src/loop_closing/beblid.h |
| 0x18016FE50 | BEBLID::compute | modified | src/loop_closing/beblid.cpp |
| 0x1801700F0 | computeABWLResponse (static) | modified: cols indexing | src/loop_closing/beblid.cpp |
| 0x180170570 | BEBLID::create | modified | src/loop_closing/beblid.cpp |
| 0x180170610 | BEBLID::descriptorSize | identical | src/loop_closing/beblid.h |
| 0x18017063C | BEBLID::empty vtordisp thunk | new (returns false) | src/loop_closing/beblid.h |
| 0x180170660 | BEBLID::getDefaultName | new | src/loop_closing/beblid.cpp |
| 0x180170840 | rectifyABWL (static) | modified: double m02 | src/loop_closing/beblid.cpp |
| 0x180170E50 | PairHash::operator() (16-byte key) | new | src/loop_closing/pair_hash.h |

### c15_platmap

| address | function | upstream status | home |
|---|---|---|---|
| 0x180174300 | pimax::totem::changeStructure | modified: reserve+push_back(row(i)) instead of resize+assign (appends) | src/loop_closing/bow.cpp |
| 0x180174480 | pimax::totem::compareBOWs | identical | src/loop_closing/bow.cpp |
| 0x1801747E0 | pimax::totem::createBOW | identical (getNodeID inlined, levelup 0) | src/loop_closing/bow.cpp |
| 0x180174B70 | pimax::totem::extractBoWFeaturesFromImage | modified: static ORB(500,1.2,4,31,0,2,HARRIS,31,20); response-sorted m | src/loop_closing/bow.cpp |
| 0x180175160 | pimax::totem::extractFeaturesFromSVOKeypoints | modified heavily (see notes) | src/loop_closing/bow.cpp |
| 0x180176330 | pimax::totem::getNodeID | identical | src/loop_closing/bow.cpp |
| 0x180178E40 | pimax::totem::commonLandMarkCheck | modified: empty ids1 -> false, no isnan term | src/loop_closing/geometric_verification.cpp |
| 0x18017E8E0 | std::make_shared<pimax::totem::PlatMap>() | new | src/loop_closing/platmap.h |
| 0x18017EE10 | pimax::totem::Utility::ypr2R<Eigen::Vector3d> | modified (VINS ypr2R + is_radian flag) | src/frontend/utility.h, src/loop_closing/loop_closing.cpp |
| 0x18017F4C0 | pimax::totem::KeyFrame::KeyFrame | new (upstream KeyFrame(int)) | src/loop_closing/platmap.cpp |
| 0x18017F730 | pimax::totem::LoopClosing::LoopClosing | modified (see notes) | src/loop_closing/loop_closing.cpp |
| 0x180180CD0 | pimax::totem::LoopClosureOptions::LoopClosureOptions(const&) | modified (extra fields) | src/loop_closing/loop_closing.cpp |
| 0x1801819D0 | LcStruct112::~LcStruct112 (implicit) | - | src/loop_closing/loop_closing.h |
| 0x180181AE0 | pimax::totem::LoopClosing::~LoopClosing | modified: calls stopAllThread() | src/loop_closing/loop_closing.cpp |
| 0x1801826F0 | ReLocalize sort comparator lambda (in 0x18018b600) | new | src/loop_closing/loop_closing.cpp |
| 0x180182910 | LoopClosing scalar deleting dtor | - | src/loop_closing/loop_closing.cpp |
| 0x180182960 | pimax::totem::PlatMap::~PlatMap (scalar deleting form) | new | src/loop_closing/platmap.cpp |
| 0x180182B70 | pimax::totem::PlatMap::UpdateMap | new | src/loop_closing/platmap.cpp |
| 0x180182D40 | pimax::totem::PlatMap::BuildMapIndex | new | src/loop_closing/platmap.cpp |
| 0x180182F80 | pimax::totem::PlatMap::load | new | src/loop_closing/platmap.cpp |
| 0x180183320 | pimax::totem::PlatMap::loadIndex | new | src/loop_closing/platmap.cpp |
| 0x180183AA0 | pimax::totem::PlatMap::load_bin | new | src/loop_closing/platmap.cpp |
| 0x180183EC0 | pimax::totem::PlatMap::save | new | src/loop_closing/platmap.cpp |
| 0x180184590 | pimax::totem::PlatMap::saveIndex | new | src/loop_closing/platmap.cpp |
| 0x180184F30 | pimax::totem::PlatMap::Map2Vec | new | src/loop_closing/platmap.cpp |
| 0x180185110 | pimax::totem::PlatMap::Vec2Map | new | src/loop_closing/platmap.cpp |

### c16_loop_closing

| address | function | upstream status | home |
|---|---|---|---|
| 0x180185E40 | LoopClosing::addFrameToPR | modified: 2 cams, pose-cell key map, quality gates (num_features_>=opt | src/loop_closing/loop_closing.cpp |
| 0x180186A50 | LoopClosing::reLocalize | new | src/loop_closing/loop_closing.cpp |
| 0x180186CB0 | drawText` (file static) | new | src/loop_closing/loop_closing.cpp |
| 0x180186F30 | LoopClosing::computePoseDiff | new | src/loop_closing/loop_closing.cpp |
| 0x1801872E0 | LoopClosing::clearReLocFrames | new | src/loop_closing/loop_closing.cpp |
| 0x180187370 | LoopClosing::backProject | new | src/loop_closing/loop_closing.cpp |
| 0x180187830 | LoopClosing::extractAndConvert | modified (see draft comment) | src/loop_closing/loop_closing.cpp |
| 0x180187C90 | LoopClosing::svoFrameToKeyframe | modified | src/loop_closing/loop_closing.cpp |
| 0x180187E30 | LoopClosing::getPoseKey | new | src/loop_closing/loop_closing.cpp |
| 0x180188830 | LoopClosing::loopClosingThread | new | src/loop_closing/loop_closing.cpp |
| 0x180188900 | LoopClosing::load | new | src/loop_closing/loop_closing.cpp |
| 0x180188CE0 | LoopClosing::loadIndex | new | src/loop_closing/loop_closing.cpp |
| 0x1801890E0 | LoopClosing::loadSavePlatMapThread | new | src/loop_closing/loop_closing.cpp |
| 0x1801891B0 | LoopClosing::loadTagIndex | new | src/loop_closing/loop_closing.cpp |
| 0x1801899C0 | LoopClosing::bundleAdjustKfList | new | src/loop_closing/loop_closing.cpp |
| 0x18018B3D0 | LoopClosing::reLocalizeThread | new | src/loop_closing/loop_closing.cpp |
| 0x18018B600 | LoopClosing::getReLocCorrection | new | src/loop_closing/loop_closing.cpp |
| 0x18018C070 | recovery_kf | new | src/loop_closing/loop_closing.cpp |
| 0x18018C3A0 | LoopClosing::resetLoopClosing | new | src/loop_closing/loop_closing.cpp |
| 0x18018C4E0 | LoopClosing::resetReLocalize | new | src/loop_closing/loop_closing.cpp |
| 0x18018C850 | LoopClosing::runPROnLatestKeyframe | modified: database building only (LC-1/2/3 VLOGs l.658/694/707/723), d | src/loop_closing/loop_closing.cpp |
| 0x18018D770 | LoopClosing::runReLocalization | new (26 KB, complete) | src/loop_closing/loop_closing.cpp |
| 0x180193E00 | LoopClosing::save | new | src/loop_closing/loop_closing.cpp |
| 0x180194020 | LoopClosing::saveIndex | new | src/loop_closing/loop_closing.cpp |
| 0x180194200 | LoopClosing::saveTagIndex | new | src/loop_closing/loop_closing.cpp |
| 0x180195AB0 | ceres_backend::Map::setParameterBlockConstant(std::shared_ptr<ParameterBlock>) | - | src/loop_closing/loop_closing.cpp |
| 0x180195B50 | ceres_backend::Map::solve() | - | src/loop_closing/loop_closing.cpp |
| 0x180195B70 | LoopClosing::startAllThread | new | src/loop_closing/loop_closing.cpp |
| 0x180195E60 | LoopClosing::stopAllThread | new | src/loop_closing/loop_closing.cpp |
| 0x1801963D0 | LoopClosing::triangulate | new | src/loop_closing/loop_closing.cpp |
| 0x180196B60 | LoopClosing::updateSVOPointsDescriptors | modified: KeyFramePtr instead of index, null check, reduced extractor  | src/loop_closing/loop_closing.cpp |
| 0x1801975B0 | MapAlignmentSE3::MapAlignmentSE3 | identical (opengv members removed) | src/loop_closing/map_alignment.cpp |
| 0x180197940 | Histogram::Histogram(int, const vector<int>&, cv::Mat, int, const vector<int>&,  | Kimera-modified | src/plane/histogram.cpp |
| 0x180197BB0 | Histogram::Histogram() | Kimera | src/plane/histogram.cpp |
| 0x180197C20 | Histogram::~Histogram() | Kimera-modified (scalar delete of ranges_[i]) | src/plane/histogram.cpp |
| 0x180197CC0 | Histogram::operator=(const Histogram&) | Kimera | src/plane/histogram.cpp |

Notes on the table: implicit members are listed with the header that defines them implicitly
(ImageBundle dtor → slam_manager.h; PoseState::operator= 0x180167790 → frontend/frame_processor_base.h
(A2), ThreeDof::State ctor 0x1801662D0 → sensor_fusion/three_dof_tracker.h (A1), both only mentioned at
their use sites in slam_manager.cpp; LoopClosureOptions ctor/dtor/copy → loop_closing.h default member
initialisers; BEBLID dtors/thunks/descriptorSize/empty → beblid.h).  `Map::setParameterBlockConstant
(shared_ptr)` 0x180195AB0 and `Map::solve()` 0x180195B50 are inline in B1's ceres_map.hpp; their
out-of-line copies are emitted in loop_closing.obj (comment at the use site).
