# Integration B3 — remaining frontend sources + backend factory

Status: all 8 `.cpp` files compile with `tools/cc.sh` and with `CC_OBJ=1` (objects in
`build-win/cc/src/frontend/*.obj`, `build-win/cc/src/interface/ceres_backend_factory.obj`); the
deterministic variant of stereo_triangulation.cpp compiles too
(`build-win/b3_scratch/st_det.cpp`). All owned headers still compile standalone (`--header`).
Every forced glog `__LINE__` was checked by preprocessing (`build-win/b3_scratch/pp.sh`):
imu_processor 109/119/342, initialization 113/270, map 32/87, pose_optimizer 62, reprojector 121,
ceres_backend_factory 13. Symbol closure: `build-win/b3_scratch/closure.sh`.

## 1. What went where

| src file | lines | drafts | content |
|---|---|---|---|
| frontend/imu_processor.cpp | 336 | c00 imu_processor_globals.cpp, c10 imu_processor.cpp, c11 imu_processor.cpp | `imu_temporal_status_names_` (initializer 0x1800028D0), ctor 0x180129D60, dtor 0x180129EC0, addImuMeasurement 0x180129F40, getClosestMeasurement (inlined only), getInitialAttitude 0x18012A2B0, getMeasurements 0x18012A700, getMeasurementsContainingEdges 0x18012A950, getRelativeRotationPrior 0x18012AB60, limitMeasurementsSize 0x18012B090 (3000-sample trim), waitTill 0x18012B140 |
| frontend/initialization.cpp | 145 | c11 | AbstractInitialization ctor 0x18012B320 / dtor 0x18012B860 / reset 0x18012C030, StereoInit ctor 0x18012B4E0 / addFrameBundle 0x18012BB60, initialization_utils::makeInitializer 0x18012BF30 |
| frontend/map.cpp | 243 | c11 | Map ctor 0x18012CE00, dtor 0x18012CEB0, addKeyframe 0x18012D680, getClosestNKeyframesWithOverlap 0x18012D900, getOverlapKeyframes 0x18012E1B0, allKeyPointsVisible 0x18012E930, removeKeyframe 0x18012E9F0, removeOldestKeyframe 0x18012EC70, reset 0x18012ED20, safeDeletePoint 0x18012EDE0 |
| frontend/pose_optimizer.cpp | 646 | c11 (part 1) + c12 (part 2) | ctor 0x180132D90, getDefaultSolverOptions 0x18013B2B0, applyPrior 0x1801331C0, evaluateError (inline wrapper, no address), evaluateErrorImpl 0x18013A840, pose_optimizer_utils::calculate{FeatureResidualUnitPlane 0x180139AE0, FeatureResidualImagePlane 0x180138430, FeatureResidualBearingVectorDiff 0x180136EA0, EdgeletResidualUnitPlane 0x1801364D0, EdgeletResidualImagePlane 0x180135AF0, EdgeletResidualBearingVectorDiff 0x1801338F0}, setRotationPrior 0x18013F580, run 0x18013ED70, removeOutliers 0x18013D8B0, update 0x18013F830 |
| frontend/reprojector.cpp | 635 | c12 | Reprojector ctor 0x1801412F0, reprojectFrames 0x180143380, reprojector_utils::{IC_Angle 0x1801415B0, computeUmax 0x180141B00, getCandidate 0x180141C40, checkPatchExposure 0x180142190, matchCandidate 0x1801422D0, matchCandidates 0x180142A60 (static umax 0x18047ED70, TLS rot_hist[30]), filterCandidatesByGrid 0x180143DB0}, file-local depthInFrameT / computeThreeMinima (inlined) |
| frontend/stereo_triangulation.cpp | 392 | c12 stereo_triangulation_c12.cpp + c13 stereo_triangulation.cpp | ctor 0x180145320, triangulate 0x180145470, computeStd 0x180146360 (c12); compute 0x180147110 (c13) with the `PIMAX_SLAM_TEST_DETERMINISTIC` switch |
| frontend/visual_imu_alignment.cpp | 341 | c13 | ImuInitializer::solveGyroscopeBias 0x180158050, TangentBasis 0x1801569A0, RefineGravity 0x1801518F0, LinearAlignment 0x18014DFE0, computeStatesInGravityFrame 0x180156C10, VisualIMUAlignment 0x180157240, computeStd 0x180158C90 |
| interface/ceres_backend_factory.cpp | 65 | c13 | ceres_backend_factory::makeBackend 0x180158F70 (VLOG(1) on line 13) |

Header-only project functions in this range, already placed by A1/A2 and only *used* here:
Frame::jacobian_xyz2f_imu 0x18013B300 / jacobian_xyz2img_imu 0x18013B8F0 / jacobian_xyz2uv_imu
0x18013BC70 (common/frame.h), Frame::getSeedDepth 0x1801420C0 (common/frame.h; its COMDAT lands in
reprojector.obj), MiniLeastSquaresSolver<6,...>::optimizeGaussNewton 0x18013C500 /
optimizeLevenbergMarquardt 0x18013CD60 (third_party/vikit solver .hpp, instantiated by
pose_optimizer.cpp), Utility::ypr2R<Vector3d> 0x18014DB90 and Utility::g2R 0x1801572E0
(frontend/utility.h, instantiated by visual_imu_alignment.cpp).
integration_base.h / imu_factor.h / utility.h / pose_local_parameterization.h: A2 made every
function inline; nothing was left out of line (`Eigen::Vector3d G` is B2's, frame_processor_base.cpp).
Every `project` row of the c11/c12/c13 tables plus c10's three ImuProcessor rows is present
exactly once (checked by grepping every address).

## 2. Conflicts resolved / renames applied

1. **Frame member names** (A1 table): c11/c13 `img_mean_intensity_`, `img_too_dark_` →
   `mean_intensity_`, `is_too_dark_`; c12 `is_dark_/is_bright_` → `is_too_dark_/is_too_bright_`;
   c12 `Frame::removeLandmark` → `deleteLandmark` (0x180094460).
2. **Map comparator field**: c11's nth_element lambda compared `bundle_id_` at Frame +20; that is
   `cam_index_` (A1). Re-checked in `_Med3_unchecked` 0x18012C5C0 (`*(DWORD*)(frame+20)`).
3. **StereoTriangulation::triangulate inputs**: c12 read `ref_ftr.uv` / `matcher.uv_cur_` as
   Vector2f; A1/c13: FeatureWrapper +56 is `f_raw` (Ref into the 3xN `f_vec_raw_`) and Matcher +332 is
   `f_cur_unnormalized_` (Vector3f). The code now reads components 0 and 1 of those (same
   addresses, same values).
4. **c13 → c12 helper names**: `triangulatePoint(..., Vector3d*)` → `triangulate(..., Vector3d&)`,
   `computeStdDev` → `computeStd` (A2 §15). The value `triangulate` returns is the depth `x3D(2)`
   (checked at the end of 0x180145470: `return v30` = z), which compute() buffers as its "error"
   for the 2-sigma gate (c13's "returns error" was a naming issue only).
5. **quaternionExp**: c11's file-local copy dropped; `pimax::totem::quaternionExp`
   (common/transformation.h, A1) is used by getRelativeRotationPrior.
6. **ImuInitializer::computeStatesInGravityFrame**: c13 wrote
   `Transformation(Quaternion(Eigen::Quaterniond(R0)), 0)`. With `Quaternion = Eigen::Quaterniond`
   the CHECKing kindr ctor 0x1800089C0 (present in the callee list of 0x180156C10) must be spelled
   explicitly: `Transformation(kindr::minimal::RotationQuaternionTemplate<double>(Eigen::Quaterniond(R0)), Zero)`.
   ImageFrame member names → A2 (`T_c0_body`, `T_w_body`, `V_w`, `Bg`, shared_ptr pre_integration).
7. **Bgs type** (B2 request): `GyroBiasVector = std::vector<Vector3d, aligned_allocator<Vector3d>>`
   for VisualIMUAlignment / solveGyroscopeBias / computeStatesInGravityFrame (header + .cpp).
8. **pose_optimizer.cpp line 62** (c12 note): the merge puts c11's functions first, so the VLOG(5)
   in run() is forced with `#line 62`; all other glog lines of B3 files are forced the same way.
   (Function order in an object is not source order — the objects look sorted by decorated
   name — so no source ordering constraint remains.)
9. **checkPatchExposure** flag logic: c12's notes say "if neither cur nor ref is dark/bright →
   true", the draft code returns true if *either* frame is normal. Re-checked 0x180142190: the
   code is right (return 1 if cur normal; return 1 if ref normal; only both-abnormal pairs run the
   5x5 test).
10. **StereoInit::addFrameBundle** re-checked against 0x18012BB60 (pairs (0,1), (0,2), (1,3), (2,3);
    VLOG prints at(0)->T_world_cam() and at(1)->T_cam_world(); at(2) quirk; return 0/3).
11. **waitTill** re-checked (0x18012B140): Timer ctor + start, 64-bit ns difference (cvtsi2sd rax),
    VLOG after Sleep(1).
12. **c12 depthInFrame helper** (template, double literals) kept file-local as `depthInFrameT`
    (renamed so it can never clash with A2's float `pimax::totem::depthInFrame` in
    ceres_backend/outlier_rejection.hpp; both give identical values — the binary's operand orders
    differ only by commutation of two-term sums).
13. `Frame::getSeedDepth` definition in c12's reprojector.cpp dropped (inline in frame.h).
14. `vikit/solver/implementation/mini_least_squares_solver.hpp` is included by the A1 header itself
    (no include guard in the .hpp — do not include it a second time).

## 3. Test switch

`StereoTriangulation::compute`: `#ifdef PIMAX_SLAM_TEST_DETERMINISTIC` both std::shuffle calls use
`std::mt19937(5489u)` (matches test/make_deterministic_orig.py, which patches std::_Random_device
to return 5489); otherwise `std::mt19937(std::random_device()())` as in the binary. The build
system must pass `/DPIMAX_SLAM_TEST_DETERMINISTIC` for the test configuration (coordinator / CMake).

## 4. Remaining TODO(verify)

* Names: `ImuProcessor::limitMeasurementsSize` (c14 "trimMeasurements"), `Map::allKeyPointsVisible`
  (in-object order hints at a name between getOverlapKeyframes and removeKeyframe),
  `reprojector_utils::checkPatchExposure / filterCandidatesByGrid / computeUmax`,
  `StereoTriangulation::triangulate / computeStd / mask_`, `ImuInitializer` (class + file name),
  `computeStatesInGravityFrame`, `ImuInitializer::computeStd`, `ceres_backend_factory` namespace.
* map.cpp: the inlined quaternion-angle helper (also inlined in the frame processor) and the
  normalised inverse-rotate spelling in getClosestNKeyframesWithOverlap.
* imu_processor.cpp: spelling of the fixed helper axis p = (0,0,1) in getInitialAttitude.
* stereo_triangulation.cpp: element types of the four unused local vectors; explicit
  renormalisation of T_f1f0; one vs. two std::random_device objects.
* visual_imu_alignment.cpp: local names (tmp_A_t, pos_x/y/z).
* pose_optimizer.cpp: project2 helper spelling (Pimax vikit).

## 5. Unresolved externals (expected from other groups)

`llvm-nm --undefined-only` on the 8 B3 objects, filtered to pimax/vk/kindr/Sophus symbols and
minus everything defined by the B3 objects + A1's objects (common/, direct/, tracker/,
third_party/vikit/src): only

* `pimax::totem::CeresBackendInterface::CeresBackendInterface(const CeresBackendInterfaceOptions&,
  const CeresBackendOptions&, const std::shared_ptr<vk::cameras::NCamera>&)` (0x180008CD0)
* `pimax::totem::CeresBackendInterface::~CeresBackendInterface()` (0x180009950)

both from **B1** (ceres_backend/ceres_backend_interface.cpp; its current object already defines
both with matching decorated names). Everything else (Frame/FrameBundle/Point/Matcher/DepthFilter
utils/detectors/FeatureTracker/NCamera/Robust cost/Logger) resolves to A1.

Symbols B3 defines that other groups use: ImuProcessor (SLAMManager/factory, B4),
Map / PoseOptimizer / Reprojector / StereoTriangulation / makeInitializer / ImuInitializer
(FrameProcessorBase/FrameProcessor, B2), makeBackend (headset init, B4),
`imu_temporal_status_names_`.
