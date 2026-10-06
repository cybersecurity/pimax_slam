# c14_interface_api — [0x180159E10, 0x180172460): public C API, headset system, factory, BEBLID

## Summary

The range holds four translation units (string-pool boundaries `a_position/a_color/...` at
0x1803B7AB0 / 0x1803B82A8 / 0x1803B83E0 mark the first three):

| TU | code | content | draft file |
|---|---|---|---|
| #1 "factory" (follows `interface/ceres_backend_factory.cpp`) | 0x180159E10–0x18016233F | svo_ros `svo_factory.cpp` descendants with all ROS params turned into literals (`load*Options`, `getImuProcessor`, `getLoopClosingModule`, `setInitialPose`), the 14 KB `device_calibration.xml` parser (`makeFrameProcessor`, tinyxml2, device detection, Fisheye62 cameras + FOV masks, rig extrinsics), Fibonacci sphere, `_splitpath` helper, and the Pimax-modified vikit `EquidistantDistortion` ("Fisheye62") / `CameraGeometry` instantiations | `draft/c14_interface_api/interface/svo_factory.{h,cpp}`, `draft/c14_interface_api/thirdparty/vikit/equidistant_distortion.h` |
| #2 C API | 0x180162340–0x180162E8F | the 10 `Headset*` exports | `interface/headset_api.cpp`, `include/pimax_slam.h`, `pimax_slam.def` |
| #3 headset system | 0x180162E90–0x18016F0EF | `SLAMManager` (0xB20 bytes; name from the thread description "SLAMManager_Thread", TODO(verify)) — SvoInterface heritage: factory wiring, image queue + processing thread, `setImuPrior`, IMU ingest with IMU-rate 6DoF prediction (`Propagate` = VINS-style mid-point integration variant, `PredictPose` = smoothing / jump hiding / dead angular-velocity fit), 3DoF fallback, `Logger::Init` (6DOF_ log rotation), `SetThreadName` | `interface/slam_manager.{h,cpp}`, `common/logger_init.cpp` |
| #4 BEBLID | 0x18016F180–0x180170C6F (+ lib tail to 0x180172460) | project-local copy of opencv_contrib BEBLID (tables byte-identical to upstream), followed by library instantiations used by the loop-closing TU (vector/unordered_map/FileStorage helpers) and a 16-byte-key hash functor | `loop_closing/beblid.{h,cpp}`, `loop_closing/pair_hash.h` |

Line counts (draft): headset_api.cpp 203, slam_manager.h 274, slam_manager.cpp 788,
svo_factory.h 80, svo_factory.cpp 520, thirdparty/vikit/equidistant_distortion.h 226,
common/logger_init.cpp 60, beblid.h 67, beblid.cpp 253, pair_hash.h 25, include/pimax_slam.h 159,
pimax_slam.def 21 (≈2700 lines).

## Exported C API (see `draft/c14_interface_api/include/pimax_slam.h`)

Global `std::unique_ptr<pimax::totem::SLAMManager> g_system` at **0x18047EDF0** ("Block" in IDA).
`SLAMManager` has `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` → `malloc(0xB20)` + Eigen alignment assert / `free`.

| ordinal | name | RVA | signature | returns |
|---|---|---|---|---|
| 323 | HeadsetGetGroundState | 0x162340 | `int (HeadsetGroundState* /*28 B*/)` | 1 written, 0 kDof3 mode (zeroed), -1 no system/disabled (zeroed) |
| 324 | HeadsetGetLocModeState | 0x162360 | `int (uint8_t*)` | 1/0/-1 as above; value = FrameProcessor +1528 |
| 325 | HeadsetGetVersion | 0x162380 | `const char* ()` | "Pimax_SLAM_2.0.0.1" via data pointer off_18046A1D8 |
| 326 | HeadsetInitialImpl | 0x162390 | `int (const char* calib, const char* out_dir, const char* voc, const char* unused, uint8_t loc_mode)` | -1 if a system exists or `std::ifstream(calib)` is not good ("calibration_directory not exist", LOGE); else `g_system.reset(new SLAMManager(...))`, 0 |
| 327 | HeadsetLeftImageNumber | 0x162510 | `unsigned ()` | image_queue_.size() (unlocked), 0 without system |
| 328 | HeadsetPutCameraImage | 0x162530 | `int (const HeadsetImage*, x4)` | -1 for the first 5 calls (static counter 0x18047EDF8), -1 on empty/non-640x480 image ("cameraN empty image", LOGW), else 0 |
| 329 | HeadsetPutHmdImuData | 0x162D70 | `int (const HeadsetImuData*, HeadsetPose*)` | SLAMManager::PutImu result: 1 pose written, 0 dropped, -1 disabled/no system |
| 330 | HeadsetReleaseImpl | 0x162DF0 | `int ()` | 0 |
| 331 | HeadsetSetDeviceEnable | 0x162E30 | `int (bool)` | 0 / -1 |
| 332 | HeadsetSetTrackingMode | 0x162E60 | `int (uint8_t)` (1 = kDof6, else kDof3) | 0 / -1 |

Export directory: name "pimax_slam.pi.dll", timestamp 0xFFFFFFFF, ordinal base 1, **3414 names**
(322 boost::serialization `?...` symbols, the 10 Headset* functions at ordinals 323..332 / hints
0x142..0x14B, and GLEW `__GLEW_*`/`__glew*`/`glew*`/`wglew*` — 2328 `__glew*` alone). The .def gives
the Headset* functions explicit ordinals; pi_server.exe resolves 9 of them by name
(HeadsetLeftImageNumber is not used by pi_server; its strings mention "slam voc or bin file not
found", "pimax_database.bin", "suggest GroundPose:", "slam warning flags:").

Records (all verified in the disassembly):
* `HeadsetImage` (40 B): +0 u64 timestamp (only the 4th header's is used), +8 u32 shutter_speed_ns
  (only camera 3 checked: < 100000 → LOGW "-------- image shutter_speed_ns is %u, too short!",
  frame still used), +12 u32 "gain" (TODO(verify) meaning; forwarded as 2nd vector), +16 never read,
  +20 width (cols), +24 height (rows), +28 stride (int, sign-extended), +32 data (CV_8UC1).
  NOTE: LedObjectPoseEstimator reads cols/rows/step at +16/+20/+24 — this DLL is shifted by 4.
* `HeadsetImuData` (40 B): +0 u64 ns, +8 float acc[3], +20 float gyr[3], +32 u64 copied, unused.
* `HeadsetPose` (104 B, same layout as LedObjectPoseEstimator ControllerPose): +0 q (x,y,z,w),
  +16 position, +28 angular_velocity, +40 velocity, +52 angular_acceleration (0), +64
  linear_acceleration, +80 u64 timestamp ns, +88 u8 tracking_state, +89 u8 tracking_flag,
  +92 float confidence, +96 bool is_6dof.
* `HeadsetGroundState` (28 B): +0 bool valid, +4 float[3], +16 float[3] (FrameProcessor
  +1416/+1420/+1432; frontend sets valid only if |z0 - z1| <= 3).

## Function table

| address | proposed qualified name | signature | kind | upstream status | one-line summary |
|---|---|---|---|---|---|
| 0x180159E10 | `pimax::totem::LoopClosureOptions::LoopClosureOptions` | LoopClosureOptions() | project | modified: +Pimax fields 256..655 | ctor (default member initialisers incl. "platMap_orborb_K8L4.bin/.yaml", "tag.yaml") |
| 0x18015A160 | `cv::MatConstIterator::MatConstIterator(const Mat*)` | - | lib:opencv mat.inl.hpp | - | used by Mat_<float> comma initializer |
| 0x18015A240 | `cv::_InputOutputArray::_InputOutputArray(Mat&)` | - | lib:opencv | - | flags 0x03010000 |
| 0x18015A260 | `Eigen::CommaInitializer<Vector3d>::finished/~CommaInitializer` | - | lib:Eigen | - | "Too few coefficients" assert 3x1 |
| 0x18015A2A0 | `Eigen::CommaInitializer<Matrix3d>::~CommaInitializer` | - | lib:Eigen | - | assert 3x3 |
| 0x18015A2D0 | `std::vector<Eigen::VectorXd>::_Tidy` | - | lib:std | - | frees 16-byte elements |
| 0x18015A360 | `std::vector<Eigen::MatrixXd>::_Tidy` | - | lib:std | - | frees 24-byte elements |
| 0x18015A420 | `vk::cameras::CameraGeometryBase::~CameraGeometryBase (body)` | - | lib:vikit (upstream-identical) | identical | ~mask_ (cv::Mat +56), ~label_ (+16) |
| 0x18015A490 | `std::vector<int>::_Tidy` | - | lib:std | - | klt_patch_sizes of FeatureTrackerOptions |
| 0x18015A4F0 | `pimax::totem::LoopClosureOptions::~LoopClosureOptions` | - | project (implicit) | - | destroys 9 std::string members |
| 0x18015A810 | `std::string::operator=(std::string&&)` | - | lib:std | - | move assign |
| 0x18015A8A0 | `pimax::totem::ImuParams::operator=` | ImuParams& (const ImuParams&) | project (implicit) | - | 96-byte copy (fp +536) |
| 0x18015A930 | `Eigen::CommaInitializer<Vector3d>::CommaInitializer` | - | lib:Eigen | - |  |
| 0x18015A960 | `Eigen::Transform<double,3,Isometry> operator* wrapper` | - | lib:Eigen | - | calls 0x1801239E0 |
| 0x18015A980 | `cv::MatConstIterator::operator++` | - | lib:opencv | - |  |
| 0x18015A9D0 | `Eigen::CommaInitializer<Vector3f>::operator,` | - | lib:Eigen | - |  |
| 0x18015AAA0 | `Eigen::CommaInitializer<Matrix2d>::operator,` | - | lib:Eigen | - |  |
| 0x18015AB70 | `Eigen::CommaInitializer<Vector3d>::operator,` | - | lib:Eigen | - | also used by Matrix3d? (3-row) |
| 0x18015AC40 | `std::basic_istringstream<char>::~basic_istringstream (vbase)` | - | lib:std | - |  |
| 0x18015ACA0 | `std::basic_istringstream<char> vector deleting dtor thunk` | - | lib:std | - | vtable 0x1803B7B18 |
| 0x18015ACB0 | `vk::cameras::CameraGeometry<PinholeProjection<EquidistantDistortion>>::`scalar deleting dtor'` | - | lib:vikit | identical | vtable slot 0 |
| 0x18015AD00 | `std::_Ref_count_obj2<CameraGeometry<...>>::`deleting dtor'` | - | lib:std | - |  |
| 0x18015AD30 | `std::_Ref_count_obj2<pimax::totem::FrameProcessor>::`deleting dtor'` | - | lib:std | - |  |
| 0x18015AD60 | `std::_Ref_count_obj2<pimax::totem::ImuProcessor>::`deleting dtor'` | - | lib:std | - |  |
| 0x18015AD90 | `std::_Ref_count_obj2<pimax::totem::LoopClosing>::`deleting dtor'` | - | lib:std | - |  |
| 0x18015ADC0 | `std::_Ref_count_obj2<pimax::totem::MapAlignmentSE3>::`deleting dtor'` | - | lib:std | - | IDA unknown_libname_107 |
| 0x18015ADF0 | `std::_Ref_count_obj2<vk::cameras::NCamera>::`deleting dtor'` | - | lib:std | - |  |
| 0x18015AE20 | `std::basic_istringstream<char>::`vector deleting dtor'` | - | lib:std | - |  |
| 0x18015AEC0 | `pimax::totem::factory::FibonacciSphere` | std::vector<Eigen::Vector3d> (double radius, int n) | project | new | golden-angle sphere sampling (used with 10.0, 100000 for FOV masks) |
| 0x18015B0E0 | `pimax::totem::factory::SplitPath` | void (const std::string&, std::string& dir, std::string& file) | project | new | _splitpath -> drive+dir / fname+ext |
| 0x18015B430 | `std::_Ref_count_obj2<ImuProcessor>::_Destroy` | - | lib:std | - | -> ~ImuProcessor (0x180129EC0) |
| 0x18015B440 | `std::_Ref_count_obj2<MapAlignmentSE3>::_Destroy` | - | lib:std | - | ~MapAlignmentSE3 inlined (3 Eigen buffers) |
| 0x18015B470 | `std::_Ref_count_obj2<vk::cameras::NCamera>::_Destroy` | - | lib:std | - | ~NCamera inlined (label, vectors) |
| 0x18015B520 | `std::vector<Eigen::Vector3d>::_Reallocate_exactly (reserve)` | - | lib:std | - |  |
| 0x18015B5C0 | `std::_Uninitialized_move<Transformation*>` | - | lib:std | - | 64-byte elements |
| 0x18015B620 | `Eigen::aligned_allocator<Transformation>::allocate` | - | lib:Eigen | - | malloc(64n) + alignment assert |
| 0x18015B690 | `vk::cameras::CameraGeometry<PinholeProjection<EquidistantDistortion>>::backProject3` | bool (const Ref<const Vector2d>&, Vector3d*) const | project (vikit template) | modified: x==y==0 shortcut, undistort(x,y,fx,fy) | see thirdparty/vikit/equidistant_distortion.h |
| 0x18015B750 | `CameraGeometry<...>::errorMultiplier` | double () const | lib:vikit | identical | fabs(fx_) |
| 0x18015B760 | `vk::cameras::EquidistantDistortion::distort` | Eigen::Vector2d (const Eigen::Vector2d&) const | project (vikit) | modified: Fisheye62 tangential, Horner |  |
| 0x18015B960 | `vk::cameras::EquidistantDistortion::jacobian` | Eigen::Matrix2d (const Eigen::Vector2d&) const | project (vikit) | modified: tangential chain rule |  |
| 0x18015BD50 | `vk::cameras::EquidistantDistortion::undistort` | void (double& x, double& y, double fx, double fy) const | project (vikit) | rewritten: Gauss-Newton, 10 its, -1000 on failure, <4 px skip |  |
| 0x18015CA20 | `CameraGeometry<...>::getAngleError` | double (double) const | lib:vikit | identical | atan(e/2fx)+atan(e/2fy) |
| 0x18015CA90 | `CameraGeometry<...>::getDistortionParameters` | Eigen::VectorXd () const | project (vikit) | modified: 6 params |  |
| 0x18015CBD0 | `pimax::totem::factory::getImuProcessor` | std::shared_ptr<ImuProcessor> (ImuParams) | project | modified (getImuHandler): literal calibration, biases from XML |  |
| 0x18015CE00 | `CameraGeometry<...>::getIntrinsicParameters` | Eigen::VectorXd () const | lib:vikit | identical | (fx, fy, cx, cy) |
| 0x18015CEE0 | `pimax::totem::factory::getLoopClosingModule` | std::shared_ptr<LoopClosing> (const char*, const char*, const NCamera::Ptr&, std::string, uint8_t) | project | modified | voc file check "Could not open voc file %s", MapAlignmentSE3{8, 40.0} |
| 0x18015D410 | `Eigen::Transform<double,3,Isometry>::inverse(TransformTraits)` | - | lib:Eigen | - | Projective/Isometry/general (0x180161F70 = 3x3 inverse) |
| 0x18015D6C0 | `Eigen::Transform::linear() block accessor` | - | lib:Eigen | - |  |
| 0x18015D6E0 | `pimax::totem::factory::loadBaseOptions` | BaseOptions (bool forward_default, std::string trace_dir) | project | modified: literals | "FORWARD"/"DOWNLOOKING" string built but unused |
| 0x18015DA90 | `pimax::totem::factory::loadDepthFilterOptions` | DepthFilterOptions () | project | modified: literals, float thresholds |  |
| 0x18015DAD0 | `pimax::totem::factory::loadDetectorOptions` | DetectorOptions () | project | modified: literals |  |
| 0x18015DB40 | `pimax::totem::factory::loadInitializationOptions` | InitializationOptions () | project | modified: literals |  |
| 0x18015DB90 | `pimax::totem::factory::loadLoopClosureOptions` | LoopClosureOptions (const char* voc, const char* out_dir) | project | modified | "voc_path %s", "voc_name %s", "input voc_name is not correct!" |
| 0x18015E0B0 | `pimax::totem::factory::loadReprojectorOptions` | ReprojectorOptions () | project | modified: literals |  |
| 0x18015E130 | `pimax::totem::factory::loadStereoOptions` | StereoTriangulationOptions () | project | modified: literals |  |
| 0x18015E170 | `pimax::totem::factory::loadTrackerOptions` | FeatureTrackerOptions () | project | identical values |  |
| 0x18015E250 | `pimax::totem::factory::makeFrameProcessor` | std::shared_ptr<FrameProcessor> (std::string, std::string, double*, std::string*, int*, const uint8_t&) | project | new (device_calibration.xml) + makeStereo | 14 KB tinyxml2 parser |
| 0x1801618F0 | `vk::cameras::NCamera::numCameras` | size_t () const | lib:vikit inline | identical |  |
| 0x180161900 | `CameraGeometry<...>::printParameters` | void (std::ostream&, const std::string&) const | project (vikit) | modified: "Distortion: Fisheye62(" 6 params |  |
| 0x180161BA0 | `CameraGeometry<...>::project3` | const ProjectionResult (const Ref<const Vector3d>&, Vector2d*, Matrix<double,2,3>*) const | lib:vikit | identical |  |
| 0x180161C40 | `PinholeProjection<EquidistantDistortion>::project3` | void (...) const | lib:vikit | identical | uses Pimax distort/jacobian |
| 0x180161F40 | `Eigen::internal::Ref stride helper` | - | lib:Eigen | - |  |
| 0x180161F70 | `Eigen::internal::compute_inverse<Matrix3d>::run` | - | lib:Eigen | - |  |
| 0x1801621A0 | `pimax::totem::factory::setInitialPose` | void (FrameProcessor&) | project | modified: literal identity | fp +400 = Transformation(identity) |
| 0x1801622B0 | `Eigen::Transform::translation() block accessor` | - | lib:Eigen | - |  |
| 0x1801622E0 | `std::_Uninitialized_copy<cv::Mat>` | - | lib:std | - |  |
| 0x180162340 | `HeadsetGetGroundState` | int (HeadsetGroundState*) | project | new | export |
| 0x180162360 | `HeadsetGetLocModeState` | int (uint8_t*) | project | new | export |
| 0x180162380 | `HeadsetGetVersion` | const char* () | project | new | export, "Pimax_SLAM_2.0.0.1" |
| 0x180162390 | `HeadsetInitialImpl` | int (const char*, const char*, const char*, const char*, uint8_t) | project | new | export |
| 0x180162510 | `HeadsetLeftImageNumber` | unsigned (void) | project | new | export (image_queue_ size, unlocked) |
| 0x180162530 | `HeadsetPutCameraImage` | int (const HeadsetImage* x4) | project | new | export, 640x480 check, skip first 5 calls |
| 0x180162D70 | `HeadsetPutHmdImuData` | int (const HeadsetImuData*, HeadsetPose*) | project | new | export |
| 0x180162DF0 | `HeadsetReleaseImpl` | int () | project | new | export |
| 0x180162E30 | `HeadsetSetDeviceEnable` | int (bool) | project | new | export |
| 0x180162E60 | `HeadsetSetTrackingMode` | int (uint8_t) | project | new | export |
| 0x180162E90 | `Eigen::ColPivHouseholderQR<MatrixXd>::ColPivHouseholderQR(const EigenBase&)` | - | lib:Eigen | - | PredictPose (dead code) |
| 0x180163150 | `Eigen::AngleAxis<double>::operator=(const QuaternionBase&)` | - | lib:Eigen | - |  |
| 0x1801632A0 | `std::filesystem::path::path(const std::string&)` | - | lib:std | - | narrow->wide |
| 0x180163320 | `std::operator<<(ostream&, _Timeobj) (std::put_time)` | - | lib:std | - |  |
| 0x180163590 | `std::_Allocate<16> (deque block / big alloc)` | - | lib:std | - |  |
| 0x1801635F0 | `std::vector<cv::Mat>::assign(first,last)` | - | lib:std | - |  |
| 0x180163850 | `std::vector<std::string>::_Emplace_reallocate(const string&)` | - | lib:std | - | log file list |
| 0x1801639C0 | `std::vector<ThreeDof::ImuSample>::_Emplace_reallocate` | - | lib:std | - |  |
| 0x180163BD0 | `std::vector<ImuMeasurement>::_Emplace_reallocate` | - | lib:std | - |  |
| 0x180163DF0 | `std::_Insertion_sort_unchecked<string*, last_write_time cmp>` | - | lib:std | - | Logger::Init sort |
| 0x1801643B0 | `std::_Med3_unchecked<string*, cmp>` | - | lib:std | - |  |
| 0x180164470 | `std::_Partition_by_median_guess_unchecked<string*, cmp>` | - | lib:std | - |  |
| 0x180165090 | `std::_Pop_heap_hole_by_index<string*, cmp>` | - | lib:std | - |  |
| 0x180165390 | `std::_Push_heap_by_index<string*, cmp>` | - | lib:std | - |  |
| 0x180165620 | `std::_Sort_unchecked<string*, cmp>` | - | lib:std | - | recursive introsort |
| 0x1801658B0 | `std::condition_variable::wait_until<steady_clock, pred>` | - | lib:std | - | pred = PopImageBundle lambda |
| 0x180165AA0 | `Eigen::ColPivHouseholderQR<MatrixXd>::_solve_impl` | - | lib:Eigen | - |  |
| 0x180166050 | `Eigen::QuaternionBase<Quaterniond>::slerp` | - | lib:Eigen | - |  |
| 0x1801661E0 | `std::wstring::wstring(size_t, wchar_t)` | - | lib:std | - | SetThreadName |
| 0x1801662D0 | `pimax::ThreeDof::State::State` | State () | project (sensor_fusion struct) | new | 72-byte float state, t = -1 |
| 0x1801663F0 | `pimax::totem::SLAMManager::SLAMManager` | (const char*, const char*, const char*, const char*, uint8_t) | project | modified (SvoInterface ctor) | "Headset tracking version %s" |
| 0x180167010 | `EH helper: ~vector<cv::Mat> via pointer` | - | lib:std (unwind) | - |  |
| 0x180167020 | `thunk -> 0x18016B690` | - | lib:std | - |  |
| 0x180167030 | `std::deque<OrientationSample>::~deque (proxy free)` | - | lib:std | - | unwind |
| 0x180167060 | `std::deque<ImageBundle>::~deque/_Tidy` | - | lib:std | - |  |
| 0x180167160 | `std::filesystem::directory_iterator impl release` | - | lib:std | - |  |
| 0x1801671B0 | `thunk -> 0x180167060` | - | lib:std | - |  |
| 0x1801671D0 | `std::unique_ptr<Unknown2840>::~unique_ptr` | - | lib:std | - | calls 0x1801A3E00 |
| 0x180167200 | `std::vector<ImuMeasurement>::_Tidy` | - | lib:std | - |  |
| 0x180167260 | `pimax::totem::ImageBundle::~ImageBundle` | - | project (implicit) | - | 3 vectors |
| 0x180167320 | `pimax::totem::SLAMManager::~SLAMManager` | () | project | modified | Stop(), g_logger.Close(), members |
| 0x1801675F0 | `std::thread::~thread` | - | lib:std | - | terminate if joinable |
| 0x180167610 | `Eigen dense assignment (column copy)` | - | lib:Eigen | - | QR solve helper |
| 0x180167790 | `pimax::totem::PoseState::operator=` | PoseState& (const PoseState&) | project (implicit) | - | 448-byte copy |
| 0x180167950 | `Logger::Init sort lambda (last_write_time(a) < last_write_time(b))` | bool (const string&, const string&) | project | new |  |
| 0x180167AF0 | `std::default_delete<ThreeDof::ThreeDofTracker>::operator()` | - | lib:std | - | ~ThreeDofTracker inlined |
| 0x180167BE0 | `std::string `deleting dtor'-style destroy helper` | - | lib:std | - |  |
| 0x180167C60 | `pimax::totem::SLAMManager::GetGroundState` | int (HeadsetGroundState*) | project | new |  |
| 0x180167DB0 | `pimax::totem::SLAMManager::PopImageBundle` | bool (vector<Mat>&, vector<u32>&, vector<u32>&, uint64_t&, int&) | project | new | "image leave size %d" |
| 0x180168070 | `Eigen::Quaterniond::Identity()` | - | lib:Eigen | - |  |
| 0x1801680D0 | `pimax::totem::SLAMManager::PutImages` | void (const vector<Mat>&, vector<u32>, vector<u32>, uint64_t) | project | new |  |
| 0x180168640 | `pimax::totem::SLAMManager::PutImu` | int (HeadsetImuData, HeadsetPose*) | project | new (SvoInterface::imuCallback heritage) | rate detection, 6DoF/3DoF output |
| 0x1801691D0 | `pimax::totem::SLAMManager::SetDeviceEnable` | void (const bool&) | project | new |  |
| 0x1801692E0 | `pimax::Logger::Init` | void (const std::string& dir) | project | new (6DOF_ variant of LedObjectPoseEstimator) | "%Y_%m_%d_%H_%M_%S", keep 5 |
| 0x18016A220 | `pimax::totem::SLAMManager::SetTrackingMode` | void (const uint8_t&) | project | new |  |
| 0x18016A280 | `pimax::totem::SLAMManager::Stop` | void () | project | new | "quit backend thread", "thread_ join" |
| 0x18016A3B0 | `pimax::totem::SLAMManager::ClearImageQueue` | void () | project | new |  |
| 0x18016A460 | `pimax::totem::SLAMManager::ProcessLoop` | void () | project | modified (SvoInterface::stereoLoop/Callback) | "SLAMManager_Thread", "Could not align gravity!" |
| 0x18016B210 | `pimax::totem::SLAMManager::UpdateThreeDof` | void (ThreeDof::State&, const ImuMeasurement&) | project | new |  |
| 0x18016B3F0 | `std::deque<OrientationSample>::_Growmap` | - | lib:std | - |  |
| 0x18016B5D0 | `std::deque<OrientationSample>::clear (_Tidy)` | - | lib:std | - |  |
| 0x18016B690 | `std::wstring/_Tidy_deallocate (fs::path dtor)` | - | lib:std | - | IDA unknown_libname_108 |
| 0x18016B700 | `std::_Uninitialized_move<ThreeDof::ImuSample>` | - | lib:std | - |  |
| 0x18016B770 | `std::_Uninitialized_move<ImuMeasurement>` | - | lib:std | - |  |
| 0x18016B7E0 | `std::vector<ImuMeasurement>::erase(first,last)` | - | lib:std | - |  |
| 0x18016B860 | `pimax::totem::SLAMManager::GetLocModeState` | int (uint8_t*) | project | new |  |
| 0x18016B940 | `pimax::totem::SLAMManager::PredictPose` | HeadsetPose (PoseState&, vector<ImuMeasurement>&, bool&, PoseState&) | project | new | 6 KB |
| 0x18016D100 | `Eigen::Quaternionf::normalized()` | - | lib:Eigen | - |  |
| 0x18016D1D0 | `pimax::totem::SLAMManager::Propagate` | void (const vector<ImuMeasurement>&, PoseState&, const double&, const double&) | project | new (VINS mid-point variant) | "front t %lu > t_end %lu" |
| 0x18016EAD0 | `Eigen::internal::Assignment<MatrixXd, Solve<ColPivHouseholderQR>>::run` | - | lib:Eigen | - |  |
| 0x18016EB90 | `Eigen::internal::quat_product<float> (SSE)` | - | lib:Eigen | - |  |
| 0x18016EC80 | `pimax::totem::SLAMManager::SetImuPrior` | bool (const int64_t&) | project | modified (SvoInterface::setImuPrior) | "Set initial orientation from accelerometer measurements." |
| 0x18016EED0 | `pimax::totem::(anon)::SetThreadName` | void (const std::string&) | project | new | SetThreadDescription via GetProcAddress |
| 0x18016F0F0 | `std::filesystem::path::string()` | - | lib:std | - | wide->narrow |
| 0x18016F180 | `std::vector<ABWLParams>::_Resize_reallocate` | - | lib:std | - |  |
| 0x18016F2F0 | `BEBLID::BEBLID` | (int n_bits, float scale_factor) | project | modified (BEBLID_Impl ctor) |  |
| 0x18016F480 | `thunk -> 0x1800E4150 (vector<Vector3d>/vector dtor)` | - | lib:std | - |  |
| 0x18016F490 | `std::vector<ABWLParams>::assign(first,last)` | - | lib:std | - |  |
| 0x18016F540 | `BEBLID::compute lambda body (operator())` | void (const cv::Range&) const | project | identical (upstream lambda) |  |
| 0x18016FBF4 | `BEBLID vbase deleting dtor thunk` | - | lib:compiler | - |  |
| 0x18016FC00 | `cv::Feature2D vbase deleting dtor thunk` | - | lib:compiler | - |  |
| 0x18016FC10 | `std::_Ref_count_obj2<BEBLID>::`deleting dtor'` | - | lib:std | - |  |
| 0x18016FC40 | `BEBLID::`vector deleting dtor'` | - | project (implicit) | - |  |
| 0x18016FCA0 | `cv::Feature2D::`vector deleting dtor' (inline instance)` | - | lib:opencv | - |  |
| 0x18016FCF0 | `std::vector<ABWLParams>::_Clear_and_reserve_geometric` | - | lib:std | - |  |
| 0x18016FDD0 | `std::_Func_impl_no_alloc<lambda,void,const Range&>::_Copy` | - | lib:std | - |  |
| 0x18016FE00 | `std::_Func_impl_no_alloc<...>::_Delete_this` | - | lib:std | - | shared by 3 vtables (folded) |
| 0x18016FE10 | `std::_Ref_count_obj2<BEBLID>::_Destroy` | - | lib:std | - |  |
| 0x18016FE30 | `std::_Func_impl_no_alloc<lambda>::_Do_call` | - | lib:std | - | -> 0x18016F540 |
| 0x18016FE40 | `std::_Func_impl_no_alloc<lambda>::_Target_type` | - | lib:std | - |  |
| 0x18016FE50 | `BEBLID::compute` | void (InputArray, vector<KeyPoint>&, OutputArray) | project | modified | no checks, ParallelLambdaWrapper |
| 0x1801700F0 | `computeABWLResponse (static)` | float (const ABWLParams&, const cv::Mat&) | project | modified: cols indexing |  |
| 0x180170570 | `BEBLID::create` | std::shared_ptr<BEBLID> (int, float) | project | modified | make_shared |
| 0x180170610 | `BEBLID::descriptorSize` | int () const | project | identical |  |
| 0x18017063C | `BEBLID::empty vtordisp thunk` | bool () const | project | new (returns false) | target folded with boost codecvt_null::do_always_noconv |
| 0x180170648 | `cv::Feature2D::empty vtordisp thunk` | - | lib:compiler | - |  |
| 0x180170654 | `BEBLID::getDefaultName vtordisp thunk` | - | lib:compiler | - |  |
| 0x180170660 | `BEBLID::getDefaultName` | cv::String () const | project | new | "BEBLID" + to_string(n) |
| 0x18017080C | `cv::Feature2D::getDefaultName vtordisp thunk` | - | lib:compiler | - |  |
| 0x180170818 | `cv::Feature2D::read vtordisp thunk` | - | lib:compiler | - |  |
| 0x180170824 | `cv::Feature2D::read vtordisp thunk (BEBLID)` | - | lib:compiler | - |  |
| 0x180170840 | `rectifyABWL (static)` | void (const vector<ABWLParams>&, vector<ABWLParams>&, const KeyPoint&, float, const Size&) | project | modified: double m02 |  |
| 0x180170B80 | `std::vector<ABWLParams>::resize` | - | lib:std | - |  |
| 0x180170C50 | `cv::Feature2D::write vtordisp thunk` | - | lib:compiler | - |  |
| 0x180170C5C | `cv::Feature2D::write vtordisp thunk (BEBLID)` | - | lib:compiler | - |  |
| 0x180170C70 | `cv::operator<<(FileStorage&, const int&)` | - | lib:opencv persistence.hpp | - | "No element name has been given" |
| 0x180170D20 | `std::operator+(std::string&&, const std::string&)` | - | lib:std | - |  |
| 0x180170D80 | `std::operator+(std::string&&, const char*)` | - | lib:std | - |  |
| 0x180170DE0 | `std::operator+(const char*, const std::string&)` | - | lib:std | - |  |
| 0x180170E50 | `PairHash::operator() (16-byte key)` | size_t (const Key16&) const | project | new | FNV-1a halves + hash_combine; loop_closing unordered_map |
| 0x180170F40 | `std::vector<T152>::_Assign_counted_range / assign` | - | lib:std | - | 152-byte element (vector at +16, cv::Mat at +48); loop_closing |
| 0x180171150 | `std::vector<T>::_Assign_range` | - | lib:std | - | loop_closing |
| 0x180171350 | `std::vector<T>::_Insert_range / assign` | - | lib:std | - | loop_closing |
| 0x1801714F0 | `std::_Copy_unchecked<T152*>` | - | lib:std | - |  |
| 0x180171580 | `std::_Destroy_range<std::vector<int/float>>` | - | lib:std | - |  |
| 0x180171610 | `std::vector<T12>::_Emplace_reallocate` | - | lib:std | - | 12-byte element |
| 0x1801717C0 | `std::vector<cv::KeyPoint>::_Emplace_reallocate` | - | lib:std | - |  |
| 0x180171920 | `std::vector<cv::Mat>::_Emplace_reallocate(const Mat&)` | - | lib:std | - |  |
| 0x180171A80 | `std::vector<T152>::_Emplace_reallocate` | - | lib:std | - |  |
| 0x180171C50 | `std::vector<std::string>::_Emplace_reallocate(string&&)` | - | lib:std | - |  |
| 0x180171DE0 | `std::vector<cv::Mat>::_Emplace_reallocate(Mat&&)` | - | lib:std | - |  |
| 0x180171F40 | `std::_Tree<...>::_Erase (recursive node destroy)` | - | lib:std | - |  |
| 0x180172000 | `std::list<pair<Key16, vector<T8>>>::_Tidy (unordered_map nodes)` | - | lib:std | - |  |
| 0x1801720A0 | `std::vector<T152>::_Resize_reallocate` | - | lib:std | - |  |
| 0x1801721F0 | `std::vector<std::vector<T4>>::_Resize_reallocate` | - | lib:std | - |  |
| 0x180172310 | `std::vector<std::vector<T4>>::_Resize_reallocate(n, const value&)` | - | lib:std | - |  |

## Types

### SLAMManager (0xB20 = 2848 bytes, no vtable) — `interface/slam_manager.h`
Layout table is in the header comment (all offsets verified from ctor 0x1801663F0, dtor
0x180167320 and the users). Sure: everything except names. Highlights:

| offset | type | name | evidence |
|---|---|---|---|
| +0 | bool | smooth_initialized_ | PredictPose, PutImu |
| +16 | PoseState | smooth_state_ | PredictPose copy target |
| +464 / +480 / +512 | bool / Quaterniond / Quaterniond | q_filter_initialized_ / q_offset_ / q_last_ | ctor identity (0,0,0,1) |
| +544 / +552 / +576 | bool / Vector3d / Vector3d | pos_filter_initialized_ / pos_offset_ / pos_last_ | ctor zero |
| +600 | bool | angular_velocity_fit_ | only written 0 in ctor → dead code in PredictPose |
| +608 | std::deque<OrientationSample(48 B)> | q_history_ | |
| +648 / +656 / +664 / +672 | int / double / double / size_t | imu_rate_mode_ / imu_time_offset_ (0.007) / first_imu_t_ (-1) / imu_count_ | PutImu |
| +680..+1039 | Vector3d×2, Quaterniond×2, Matrix3d×2, Vector3d×4 | Propagate scratch (un_gyr_, un_acc_, delta_q_, result_delta_q_, delta_R0_, delta_R1_, gyr_0_, acc_0_, gyr_1_, acc_1_) | 0x18016D1D0; not initialised by ctor |
| +1040/+1056/+1072/+1088 | shared_ptr | frame_processor_ / ncam_ / imu_processor_ / backend_ | ctor |
| +1104 / +1120 | std::thread | process_thread_ / unused_thread_ | ctor, dtor check |
| +1136 | std::deque<ImageBundle(80 B)> | image_queue_ | PutImages / PopImageBundle |
| +1176 | std::vector<ImuMeasurement> | imu_vec_ | PutImu (cleared at end of ctor) |
| +1200 | HeadsetPose | last_6dof_pose_ | PutImu |
| +1304 | bool = true | no_6dof_pose_yet_ | PutImu |
| +1312 | Quaternionf | q_3dof_to_6dof_ | PutImu |
| +1328 / +1408 | mutex / condition_variable | image_mutex_ / image_cv_ | |
| +1480 / +1488 / +1568 / +1648 | bool / mutex×3 | quit_ / quit_mutex_ / wait_mutex_ / enable_mutex_ | |
| +1728 / +1729 | bool = 1 / u8 = 1 | enabled_ / tracking_mode_ | |
| +1736 / +1744 / +1776 / +1780 | double / string / int = -1 / int | cam_imu_delta_ / device_sn_ / device_type_ / frame_count_ | parser outputs |
| +1792 / +2240 | PoseState | last_low_pose_ / predicted_state_ | |
| +2688 | ImuMeasurement | last_imu_ | |
| +2720 / +2800 / +2816 / +2824 | mutex / Quaternionf / bool / double | ground_mutex_ / ground_q_ / ground_valid_ / ground_t_ | 3DoF → frontend |
| +2832 | unique_ptr<ThreeDof::ThreeDofTracker> (0x2A0, ctor 0x1801A6AC0) | three_dof_ | |
| +2840 | unique_ptr<Unknown2840> (0x200, dtor 0x1801A3E00) | unknown_2840_ | only reset to null in ctor |

### ImuMeasurement (32 B) — Pimax float version of svo::ImuMeasurement
+0 double timestamp [s], +8 Vector3f angular_velocity, +20 Vector3f linear_acceleration
(upstream: double + 2× Vector3d). `ImuMeasurements = std::deque<ImuMeasurement, aligned_allocator>`
(malloc'd proxy/blocks in ProcessLoop).

### ThreeDof::ImuSample (32 B)
+0 double t, +8 Vector3f acc, +20 Vector3f gyr (note: acc first, unlike ImuMeasurement).

### ThreeDof::State (72 B, ctor 0x1801662D0)
+0 Vector3f, +12 Vector3f, +24 Vector3f, +36 Vector3f (p, v, ba, bg by analogy with
LedObjectPoseEstimator), +48 Quaternionf (0,0,0,1 = xmmword_1803B8950), +64 double t = -1.

### PoseState (448 B; ctor 0x1800E2B10, operator= 0x180167790; owned by the frontend chunk)
| off | type | meaning (evidence) |
|---|---|---|
| +0 | double | timestamp (s) |
| +16 | Isometry3d (Matrix4d storage, ctor identity) | T_odom_imu: position = (T144*T16).translation(); Propagate integrates it |
| +144 | Isometry3d (setIdentity 0x1800F1380) | T_world_odom: velocity / acceleration rotated by its linear() |
| +272 | Vector3d | velocity |
| +296 / +320 | Vector3d | gyro bias / acc bias (subtracted in Propagate) |
| +344 | Vector3d | angular velocity (last bias-free gyro) |
| +368 | Vector3d | linear acceleration R*a - g |
| +392 | Vector3d | unknown (zeroed on reset) |
| +416 | Vector3d | gravity |
| +440 | float | confidence / valid (== 1.0f → 6DoF output) |
| +444 / +445 / +446 | u8 / u8 (=4) / bool | tracking flag / tracking state / reset |

### ImageBundle (80 B)
+0 u64 timestamp ns, +8 vector<cv::Mat>, +32 vector<u32> exposures, +56 vector<u32> gains.

### OrientationSample (48 B): +0 double t, +16 Quaterniond q.

### ImuParams (96 B, FrameProcessor +536; ctor 0x1800E1ED0)
+0 wBias, +12 aBias, +24 ka, +36 kg, +48 na, +60 ng (Vector3f each, from `SFConfig/Stateinit`
attributes), +72 Vector3f not set by the parser, +88 double delta. The SLAMManager ctor forwards
wBias/aBias to 0x1801A7A50 (tracker +100/+112), named `ThreeDofTracker::SetBias(bg, ba)` by analogy
with LedObjectPoseEstimator — TODO(verify).

### LoopClosureOptions (656 B; ctor 0x180159E10, dtor 0x18015A4F0, loader 0x18015DB90)
| off | type | upstream name | ctor default | loader value |
|---|---|---|---|---|
| +0 | bool | runlc | — | true (false if voc file name != "voc_GEN_8X4.dbow") |
| +8 | string | voc_name | "" | "voc_GEN_8X4.dbow" |
| +40 | string | voc_path | "" | directory part of the vocabulary path (`_splitpath` drive+dir) |
| +72 / +80 | double | alpha / beta | — | 1.0 / 1.0 |
| +88 | int | ignored_past_frames | — | 15 |
| +96 | string | scale_ret_app | "" | "None" |
| +128 / +136 | double | bowthresh / gv_3d_inlier_thresh | — | 0.65 / 0.4 |
| +144 / +148 | int | min_num_3d / orb_dist_thresh | — | 10 / 48 |
| +152 | double | gv_2d_match_thresh | — | 0.1 |
| +160 / +161 | bool | use_opengv / enable_image_logging | — | false / false |
| +168 | string | image_log_base_path | "" | "/home/cc/tmp/img_dir/" |
| +200 | double | proximity_dist_ratio | 0.01 | 0.01 |
| +208 | double | proximity_offset | 0.3 | 0.2 |
| +216 | string | global_map_type | "" | "BuiltInPoseGraph" |
| +248 | double | force_correction_dist_thresh_meter | 0.1 | 0.01 |
| +256 | size_t | Pimax | 20 | |
| +264 / +272 / +280 | double | Pimax | 0.6 / 0.15 / 15.0 | |
| +288 / +296 | size_t | Pimax | 40 / 1500 | |
| +304 | int | Pimax | 6 | |
| +312 | double | Pimax | 0.5 | |
| +320 / +328 | size_t | Pimax | 3 / 18 | |
| +336 / +340 | int | Pimax | 20 / 1 | |
| +344 | float | Pimax | 2.0f | |
| +352 / +360 / +368 / +376 / +384 | double | Pimax | 0.99 / 0.1 / 0.75 / 0.04 / 1.0 | |
| +392 / +393 / +394 | bool | Pimax | 0 / 1 / 1 | |
| +400 | size_t | Pimax | 30 | |
| +408 | float | Pimax | 0.5f | |
| +412 | int | Pimax | 20 | |
| +416 | size_t | Pimax | 30 | |
| +424 | double | Pimax | 0.2 | |
| +432 | size_t | Pimax | 10 | |
| +440 | int | Pimax | 48 | |
| +448 / +456 | double | Pimax | 0.09 / 0.12 | |
| +464 / +468 | int | Pimax | 20 / 1 | |
| +472 | float | Pimax | 3.0f | |
| +480 | double | Pimax | 0.9 | |
| +488 | size_t | Pimax | 8 | |
| +496 | double | Pimax | 0.5 | |
| +504 | bool | Pimax | false | |
| +512 | string | map_path | "" | `<output_directory>/` |
| +544 / +552 | size_t | Pimax | 50 / 2 | |
| +560 / +592 / +624 | string | Pimax | "platMap_orborb_K8L4.bin" / "platMap_orborb_K8L4.yaml" / "tag.yaml" | |
The loop-closing chunk should name the Pimax fields (they are consumed by LoopClosing 0x18017F730).

### Options built by the factory (final values; byte offsets)
* **BaseOptions** (0x18015D6E0, ≥ 250 B): +0 max_n_kfs 200, +8 kfselect_criterion 1 (FORWARD),
  +16 kfselect_min_dist 0.12, +24 numkfs_upper 140, +32 numkfs_lower 60, +40 min_dist_metric 0.1,
  +48 min_angle 20.0, +56 min_num_frames_between_kfs 2, +64 min_disparity 40.0,
  +72 backend_max_time_sec 3.0, +80 init_map_scale 1.5, +88 init_use_att_and_depth false,
  +96 img_align_max_level 4, +104 min_level 2, +112 robustification false, +120 prior_lambda_rot
  0.5, +128 prior_lambda_trans 0.0, +136 img_align_max_num_features (int) 0, +140 use_distortion_jacobian
  false, +141 est_illumination_gain true, +142 est_illumination_offset true, +144 poseoptim_thresh
  2.0, +152 poseoptim_prior_lambda 0.5, +160 poseoptim_using_unit_sphere true,
  +164 structure_optimization_max_pts 20, +168 trace_dir = output_directory, +200 quality_min_fts
  50, +208 quality_max_fts_drop 100, +216 relocalization_max_trials 5, +224 use_imu true,
  +225 update_seeds_with_old_keyframes true, +226 use_async_reprojectors true, +227
  trace_statistics false, +232 backend_scale_stable_thresh 0.02, +240 global_map_lc_timeout_sec_ 2.0,
  +248 (Pimax int16?) 20. Ctor defaults that differ from upstream: max_n_kfs 200, quality_max_fts_drop
  150, relocalization_max_trials 100, use_imu true, kfselect_numkfs 110/80, min_dist_metric 0.5,
  min_angle 5.0, min_disparity -1.0.
* **DepthFilterOptions** (0x18015DA90, 56 B): +0 float 200, +4 float 500, +8 use_inverse_depth 1,
  +16 max_search_level 2, +24 verbose 0, +25 use_threaded 1, +26 update_3d_point 1, +27
  scan_epi_unit_sphere 1, +32 max_n_seeds_per_frame 256, +40 max_map_seeds_per_frame 200,
  +48 affine_est_offset 1, +49 affine_est_gain 0, +50 extra_map_points 0.
* **DetectorOptions** (0x18015DAD0, 80 B): +0 20, +8 int 2, +12 int 0, +16 int 8, (+20 not
  written), +24 20.0, +32 10.0, +40 byte 0, +48 200.0, +56 qword 0, +64 1, +72 50.0.
  TODO(verify) member names (Pimax layout differs from upstream).
* **InitializationOptions** (0x18015DB40, 64 B): +0 init_type 0 (re-set to 0 by the caller,
  upstream kStereo), +8 30.0, +16 0.5, +24 45, +32 2.0, +40 50, +48 70, +56 2.0.
* **ReprojectorOptions** (0x18015E0B0, 96 B): +0 100, +8 100, +16 100, +24 20, +32 (Pimax
  size_t) 19, +40 bool 1, +48 -1.0, +56 0, +64 float 200, +68 bool 1, +69 bool 1, +70 bool 0,
  +72 50, +80 20, +88 50.0.
* **StereoTriangulationOptions** (0x18015E130): 120, mean 0.3, min 1.0, max 0.05.
* **FeatureTrackerOptions** (0x18015E170): upstream defaults (4, 0, {16,16,16,8,8}, 30, 0.001,
  true, 50, true).
* **ImuCalibration** (getImuProcessor 0x18015CBD0): delay_imu_cam 0, max_imu_delta_t 0.01,
  gyro_noise_density 0.004, acc_noise_density 0.02, imu_integration_sigma 0,
  gyro_bias_random_walk_sigma 0.0004, acc_bias_random_walk_sigma 0.002, gravity_magnitude 9.80667,
  coriolis 0, saturation_accel_max 156.8, saturation_omega_max 35.0, imu_rate 1000.
  **ImuInitialization**: velocity/omega_bias/acc_bias 0, sigmas 2.0 / 0.01 / 0.1. After
  construction: ImuProcessor +208 = aBias (double), +232 = wBias — TODO(verify) whether this maps
  aBias to omega_bias (upstream order velocity, omega_bias, acc_bias).
* **MapAlignmentOptions**: ransac3d_min_pts 8, ransac3d_inlier_percent 40.0.

### vikit CameraGeometry<PinholeProjection<EquidistantDistortion>> (make_shared block 0x128)
vtable 0x1803B7C78. +0 vfptr, CameraGeometryBase fields (+8 width, +12 height, +16 label_ string,
+48 type_ (int, copied from projection cam_type_), +56 mask_ cv::Mat), +152 PinholeProjection
{+0 cam_type_ 0, +8 fx, +16 fy, +24 fx_inv, +32 fy_inv, +40 cx, +48 cy, +56 EquidistantDistortion
{k1..k4, p1, p2, 1e-8, M_PI, kRThresh 1e-8}}.

### BEBLID (72 B) — see `loop_closing/beblid.h`
+0 vfptr 0x1803B8A28 (Feature2D part), +8 vbptr, +16 vector<ABWLParams>, +40 float scale_factor_,
+44 cv::Size patch_size_ (32,32), +60 vtordisp, +64 cv::Algorithm vfptr 0x1803B8A70.

## External interfaces

Calls out of the chunk (address → inferred meaning):
* Logger (0x18046A000): LOGD 0x18000C120, LOGI 0x18000F500, LOGW 0x18000F6A0, LOGE 0x18000C2C0.
* ceres_backend_factory::makeBackend(const NCamera::Ptr&) **0x180158F70** → shared_ptr<CeresBackendInterface>;
  `CeresBackendInterface::setImu(const shared_ptr<ImuProcessor>&)` **0x180015A80**; 0x1800149F0 (logs
  "Backend: Reset\n" only; called from Stop() if backend_ — TODO(verify) name, maybe quitThread).
* FrameProcessor (frontend): `setBundleAdjuster` 0x1801274A0, `addImageBundle(images, exposures,
  gains, const uint64_t& ts, const ImuMeasurements&, const int& left, const Quaternionf& ground_q,
  bool ground_valid, double ground_t)` **0x1800FB650**, `setRotationPrior` 0x180128210,
  `setRotationIncrementPrior` 0x1801280A0, `PoseState` ctor 0x1800E2B10 / operator= 0x180167790,
  make_shared<FrameProcessor>(base, depth, detector, init, stereo, reprojector, tracker, ncam,
  std::string& device_sn, const uint8_t& loc_mode) 0x180159940 (c13 range), ImuParams ctor 0x1800E1ED0.
  Fields used: +280 ncam_ (shared_ptr), +312 last_frames_ (shared_ptr<FrameBundle>; frames_[0]->+240
  timestamp ns), +400 T_world_imuinit_ (Transformation), +464 loc_mode_ (u8), +504 depth_filter_
  (DepthFilter*, `stopThread` 0x1800A2730), +520 imu_processor_, +536 imu_params_ (ImuParams),
  +696 state mutex, +784 state_ (PoseState), +1336 ground mutex, +1416 ground_valid_, +1420/+1432
  ground points (Vector3f), +1448 loc mutex, +1528 loc state (u8), +1624 lc_ (shared_ptr<LoopClosing>),
  +2980 need_reset_ (bool; plays !hasStarted()), +3456 imu_not_initialized_ (bool; cleared on
  "imu_initial true").
* ImuProcessor (frontend, ctor 0x180129D60 (calib, init), size 0x2A8): `addImuMeasurement`
  0x180129F40, `trimMeasurements` (halves its deque at >= 3000) 0x18012B090, `waitTill(t, 0.033)`
  0x18012B140, `getMeasurements(t, ImuMeasurements&, true)` 0x18012A950, `getInitialAttitude(t, Quaterniond&)`
  0x18012A2B0, `getRelativeRotationPrior(t0, t1, false, Quaterniond&)` 0x18012AB60; fields +208/+232.
* LoopClosing ctor (options, ncam, const std::string& device_sn, uint8_t loc_mode) 0x18017F730
  (calls BEBLID::create(256, 0.75f)); MapAlignmentSE3 ctor 0x1801975B0; lc +1936 map_alignment_se3_.
* sensor_fusion: ThreeDofTracker ctor 0x1801A6AC0 (0x2A0 B: +0 shared_ptr<ThreeDof::ImuFilter>,
  +16 mutex, +96 ready_, +128 State, +208 Config(32 B), +240 vector, +280 GyroscopeBiasEstimator
  (dtor 0x1801A4540)), `SetBias(wBias, aBias)`-like 0x1801A7A50 (+100/+112), `Propagate(config, imus,
  state, State* out)` 0x1801A7540; Unknown2840 dtor 0x1801A3E00 (0x200 B).
* Eigen helpers in other objects: Isometry3d product 0x1801239E0, Quaterniond from matrix 0x1800B5D60
  / 0x180020AF0, quaternion product 0x18004F880, normalized 0x1800426F0, inverse 0x1800132A0,
  toRotationMatrix 0x180029DF0, kindr Transformation(q,t) 0x180022CE0 / inverse 0x180013040.
* vikit: CameraGeometryBase(width,height) 0x1801B3C00, setMask 0x1801B3F30; NCamera ctor
  (T_C_B vector, T_B_C vector, cameras, label) 0x180159A40 (Pimax: two transformation vectors).
* tinyxml2: XMLDocument(true, PRESERVE_WHITESPACE) 0x1801B0190, ~XMLDocument 0x1801B0550,
  LoadFile 0x1801B1CC0, FirstChildElement 0x1801B1480, Attribute 0x1801B10A0, NextSiblingElement
  0x1801B2190.
* ParallelLambdaWrapper (project class, vtable 0x1803B8E38, dtor 0x180093630, run 0x1800935C0).

Globals: g_logger 0x18046A000 (m_file at +40 → 0x18046A028, filebuf 0x18046A030, _Myfile
0x18046A0B0); version pointer 0x18046A1D8 → 0x1803B8300; g_system 0x18047EDF0; put-image counter
0x18047EDF8; imu overflow counter 0x18047EE1C; SetThreadDescription pointer 0x18047EE10 (+ guard
0x18047EE18); thread_local HeadsetPose in TLS (+784); BEBLID tables 0x1803B9220 (512) / 0x1803BC220
(256).

## Constants / config defaults
* HeadsetPutCameraImage: skip first 5 calls; image must be 480×640; shutter warning < 100000.
* PutImu: IMU rate detection after 10 samples: rate < 900 Hz → mode 2, time offset 0.0065 s;
  else mode 1, 0.0035 s (default before detection 0.007). Drop if |acc| < 0.001. imu_vec_ halved
  when > 3000 (warn every 10th time). Vision state stale if |t_imu - t_vis| > 4.0 s
  ("m.t - low_pose.t > 1s" message is wrong). 3DoF-mode pose: tracking_state 4, confidence 1.0.
* PredictPose: propagation horizon min(t_vis + 0.099, last IMU t); blend 0.9 / (1-0.9);
  position offset decay ×0.99 per sample; orientation offset slerp(Identity, 0.99); angular fit
  window 0.02 s, ≥ 4 and ≤ 32 samples, threshold 1e-9 (dead code).
* Propagate: skip steps with dt < 1e-6; sinc Taylor below |x| <= 1e-6 (float 1/6, 1/120, 1/5040).
* ProcessLoop: wait 2 ms; "super jump first 25 frames" for device_type 2 (Crystal Super) unless
  loc_mode; waitTill timeout 0.033 s; need ≥ 20 IMU samples; thread priority TIME_CRITICAL,
  affinity 0x3C, SetThreadExecutionState(ES_CONTINUOUS|ES_SYSTEM_REQUIRED), working set (-1,-1).
* Device detection (deviceUID prefix): "P90"/"P330" → 1 Crystal Light, "P40" → 2 Crystal Super,
  "P51" → 3 Dream Air, "P61" → 4 Dream Air SE, else unknown (device_type_ stays -1).
* FOV mask: FibonacciSphere(10.0, 100000) projected, cv::circle radius 3 filled, 255.
* Fisheye62 undistort: skip if (x·fx)²+(y·fy)² < 16; ≤ 10 Gauss-Newton steps, stop |step|² < 1e-16;
  failure → (-1000, -1000).
* Logger: `<dir>/6DOF_%Y_%m_%d_%H_%M_%S.txt`, keep 5 newest "6DOF_*" files.

## Quirks / bugs worth preserving
* SetDeviceEnable(false) sets the image thread's quit flag → the vision thread exits for good;
  SetDeviceEnable(true) only clears the flag. SetTrackingMode(kDof3) calls Stop() (joins the thread);
  switching back to kDof6 does not restart it.
* HeadsetInitialImpl's 4th argument is never used; HeadsetLeftImageNumber reads the deque size
  without locking; HeadsetPutCameraImage's skip counter is process-global and never reset.
* PutImu: "m.t - low_pose.t > 1s" printed for a 4 s threshold (and only when the state was
  updated); the drop message says "older than last frame" for IMU-vs-IMU ordering.
* PredictPose: the HeadsetPose tracking_state/flag come from the *input* low_pose even when the
  previous predicted state is used; erase of old IMU samples only when a new vision state arrived;
  `imus.back()` used without emptiness check on reset.
* Propagate: "%lu" with double arguments; the acceleration midpoint uses (R0+R1)·½(a0+a1); the
  sinc is single precision.
* ThreeDof state/config read from the tracker without its mutex (UpdateThreeDof).
* BEBLID::compute has no empty-image check and does not convert colour images.
* Fisheye62 undistort leaves points within 4 px of the principal point *distorted* (no-op).
* Factory: the parser requires exactly 4 `<Camera>` elements (no null checks); "Load more cameras
  not needed!" can never trigger; the vocabulary must be named exactly "voc_GEN_8X4.dbow".
* getImuProcessor may swap aBias/wBias semantics (TODO(verify) against ImuProcessor layout).

## Open questions
* Original names of SLAMManager, its file, the factory TU and several option fields; PoseState
  field names (frontend chunk); Unknown2840 type; ThreeDofTracker method names.
* DetectorOptions/ReprojectorOptions exact Pimax member lists (+20 of DetectorOptions unwritten).
* Meaning of HeadsetImage +12 ("gain") and of the two extra Fisheye62 constants (+48 1e-8, +56 π).
* Key type of PairHash (16-byte key of a loop-closing unordered_map).
