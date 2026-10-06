# Chunk c13_stereo_imuinit — 0x180147110 – 0x180159E10

Contents (three object files):

1. **frontend/stereo_triangulation.cpp** — `StereoTriangulation::compute` (0x180147110, 10.5 KB), heavily
   Pimax-modified, plus the Eigen/STL template code instantiated there (JacobiSVD accessors, ...).
2. **VINS-Mono visual-inertial alignment** (file name unknown, see "Open questions"):
   `ImuInitializer::{VisualIMUAlignment, solveGyroscopeBias, LinearAlignment (14.6 KB), RefineGravity
   (20.6 KB), TangentBasis, computeStatesInGravityFrame (new, "ImuInitialGw0"), computeStd (new)}`,
   `Utility::g2R`, `Utility::ypr2R<Vector3d>`, and the Eigen LDLT / GEMM / GEMV / triangular-solve
   instantiations they pull in.
3. **interface/ceres_backend_factory.cpp** — `ceres_backend_factory::makeBackend` (0x180158F70) and its
   `make_shared` control block; the tail of the range (0x1801590E0 – 0x180159DC0) is STL/OpenCV template
   code instantiated for the *next* object (callers 0x18015AEC0 / 0x18015E250, interface API code).

Drafts (`draft/c13_stereo_imuinit/`):

| file | content |
|---|---|
| `common/c13_external.h` | declarations of other chunks' types as c13 uses them (Frame fields, FeatureWrapper, Point, Matcher, IntegrationBase, ImageFrame, detector) with offsets |
| `frontend/stereo_triangulation.h/.cpp` | `StereoTriangulationOptions`, `StereoTriangulation`, `compute` (complete) |
| `common/utility.h` | VINS `Utility::R2ypr`, `ypr2R`, `g2R` |
| `frontend/visual_imu_alignment.h/.cpp` | `ImuInitializer` (all 7 functions, complete) |
| `interface/ceres_backend_factory.h/.cpp` | Pimax option-struct layouts, `makeBackend` (VLOG on line 13) |

Line counts: stereo_triangulation.cpp 276, visual_imu_alignment.cpp 330, ceres_backend_factory.cpp 53,
headers 538 (total 1197).

Confidence: high for the control flow and constants of every reconstructed function (all literals read with
`rd`, every float comparison checked in the asm for its NaN behaviour); high for the IMU-init math
(each Eigen block traced through the helper calls, operand order of the modified expressions verified);
medium for names (class/file of the IMU initializer, a few Frame/ImageFrame members) — marked TODO(verify).

## Function table

| address | proposed qualified name | signature | kind | upstream status | summary |
|---|---|---|---|---|---|
| 0x180147110 | `pimax::totem::StereoTriangulation::compute` | `void (const FramePtr& frame0, const FramePtr& frame1, bool is_init)` | project | modified (heavily, see Quirks) | detect new corners (masked), optional bright-image rejection, epipolar match + DLT triangulation + depth/parallax filtering, 2-sigma error gate, insert landmarks into both frames |
| 0x180149A10 | `Eigen::JacobiSVD<Matrix4d>::computeU` | `bool () const` (`m_computeFullU \|\| m_computeThinU`, +295/+296) | lib:Eigen SVDBase accessor | - | used by 0x180146440 (JacobiSVD compute, prev. chunk) |
| 0x180149A30 | `Eigen::JacobiSVD<Matrix4d>::computeV` | `bool () const` (+297/+298) | lib:Eigen | - | ditto |
| 0x180149A50 | `std::_Deallocate<16>` for 144-byte elements | `(alloc&, void*, size_t n)` | lib:STL (vector<FeatureWrapper> deallocate, used by unwind funclets) | - | |
| 0x180149AA0 | `Eigen::LDLT<MatrixXd>::_solve_impl<VectorXd,VectorXd>` | | lib:Eigen | - | P, L, D, L^T, P^T solves (asserts "a_index <= m_matrix.cols()...") |
| 0x18014A0B0 | `Eigen::internal::check_transpose_aliasing_run_time_selector` wrapper | | lib:Eigen | - | "aliasing detected during transposition" |
| 0x18014A0E0 | same, other instantiation | | lib:Eigen | - | |
| 0x18014A110 | `Eigen::LDLT<MatrixXd>::compute<MatrixXd>` | | lib:Eigen | - | "a.rows()==a.cols()", calls ldlt_inplace 0x18014C7A0 |
| 0x18014A5E0 | `std::vector<double>::push_back(const double&)` | | lib:STL | - | |
| 0x18014A610 | `generic_product_impl<Transpose<MatrixXd>,MatrixXd,DenseShape,DenseShape,GemmProduct>::evalTo` | | lib:Eigen | - | r_A = tmp_A^T * tmp_A in LinearAlignment (lazy path for small sizes, GEMM otherwise) |
| 0x18014AAB0 | Eigen redux (maxCoeff with index) | | lib:Eigen | - | used by ldlt_inplace pivoting |
| 0x18014AC60 | `std::next(std::_Tree_iterator<map<double,ImageFrame>>, n)` (advance) | | lib:STL | - | |
| 0x18014AD40 | Eigen `general_matrix_vector_product` call thunk | | lib:Eigen | - | |
| 0x18014ADA0 | Eigen dot-product redux (dynamic) | | lib:Eigen | - | |
| 0x18014AFF0 | Eigen `squaredNorm` redux (VectorXd) | | lib:Eigen | - | dg.norm() in RefineGravity |
| 0x18014B1A0 | Eigen gemv thunk (`MatrixXd * VectorXd` add-to) | | lib:Eigen | - | `g0 + lxly*dg` |
| 0x18014B200 | `Eigen::internal::quaternionbase_assign_impl<Product<Transpose<Matrix3d>,Matrix3d>>::run` | | lib:Eigen | - | `Quaterniond q_ij(R_i^T R_j)` |
| 0x18014B6F0 | `gemv_dense_selector<OnTheRight,RowMajor,true>::run` | | lib:Eigen | - | |
| 0x18014B8A0 | `gemv_dense_selector<OnTheRight,ColMajor,true>::run` | | lib:Eigen | - | r_b = tmp_A^T tmp_b |
| 0x18014BA20 | dense assignment loop tail (`VectorXd *= scalar`) | | lib:Eigen | - | |
| 0x18014BAE0 | dense assignment loop tail (`MatrixXd *= scalar`) | | lib:Eigen | - | |
| 0x18014BBA0 | `generic_product_impl<Transpose<MatrixXd>,MatrixXd,...,GemmProduct>::scaleAndAddTo` | | lib:Eigen | - | |
| 0x18014C440 | `SolverBase<LDLT<Matrix3d>>::solve` (expression ctor) | | lib:Eigen | - | "Solver is not initialized." |
| 0x18014C4A0 | `SolverBase<LDLT<MatrixXd>>::solve` (expression ctor) | | lib:Eigen | - | |
| 0x18014C520 | `TriangularViewImpl::solveInPlace` (triangular_solver_selector, matrix rhs) | | lib:Eigen | - | |
| 0x18014C660 | (misnamed `_kbhit_nolock` by FLIRT) `triangular_solver_selector<...,1>::run` (vector rhs, alloca) | | lib:Eigen | - | calls 0x180157CC0 |
| 0x18014C7A0 | `Eigen::internal::ldlt_inplace<Lower>::unblocked<MatrixXd,Transpositions,VectorXd>` | | lib:Eigen | - | ("mat.rows()==mat.cols()", 5 KB) |
| 0x18014DB90 | `pimax::totem::Utility::ypr2R<Eigen::Vector3d>` | `Matrix3d (const MatrixBase<Vector3d>& ypr)` | project (VINS header template) | identical | Rz*Ry*Rx |
| 0x18014DF10 | Eigen product-evaluator copy ctor | | lib:Eigen | - | |
| 0x18014DF50 | Eigen row-block-of-expression ctor | | lib:Eigen | - | |
| 0x18014DF80 | Eigen product-evaluator copy ctor | | lib:Eigen | - | |
| 0x18014DFB0 | `Eigen::LDLT<MatrixXd>::~LDLT` | | lib:Eigen | - | 3 frees |
| 0x18014DFE0 | `pimax::totem::ImuInitializer::LinearAlignment` | `bool (std::map<double,ImageFrame>&, Vector3d& g, VectorXd& x)` | project | modified | VINS linear alignment + static-motion test + Pimax acceptance thresholds + 2 INFO logs, calls RefineGravity |
| 0x1801518F0 | `pimax::totem::ImuInitializer::RefineGravity` | `void (std::map<double,ImageFrame>&, Vector3d& g, VectorXd& x)` | project | modified | up to 10 iterations, break when \|dg\| < 0.001 |
| 0x1801569A0 | `pimax::totem::ImuInitializer::TangentBasis` | `MatrixXd (Vector3d& g0)` | project | identical | |
| 0x180156C10 | `pimax::totem::ImuInitializer::computeStatesInGravityFrame` (name ours) | `void (std::map<double,ImageFrame>&, std::vector<Vector3d>& Bgs, Vector3d& g, VectorXd& x)` | project | new | T_w0 = (g2R(RIC0 g), 0); per frame T_w_body = T_w0*T_c0_body, V = T_w0*(RIC0 R x_k), Bg = Bgs[k]; "INFO=ImuInitialGw0" |
| 0x180157240 | `pimax::totem::ImuInitializer::VisualIMUAlignment` | `bool (std::map<double,ImageFrame>&, std::vector<Vector3d>& Bgs, Vector3d& g, VectorXd& x, Vector3d ba)` | project | modified | gyro bias → linear alignment → (on success) world states |
| 0x1801572E0 | `pimax::totem::Utility::g2R` | `Matrix3d (const Vector3d& g)` | project | identical | (R2ypr inlined, roll dead) |
| 0x1801575C0 | `Eigen::LDLT<MatrixXd>::LDLT(const EigenBase<MatrixXd>&)` | | lib:Eigen | - | |
| 0x1801576C0 | `Eigen::LDLT<MatrixXd>::transpositionsP()` | | lib:Eigen | - | "LDLT is not initialized." |
| 0x180157720 | `DenseBase::resize(3,2)` assert stub (Block<…,3,2>) | | lib:Eigen | - | |
| 0x180157750 | `PlainObjectBase::resize(3)` assert stub | | lib:Eigen | - | |
| 0x180157780 | `VectorXd::operator=(Solve<LDLT<MatrixXd>,VectorXd>)` | | lib:Eigen | - | |
| 0x180157820 | `general_matrix_matrix_product<...>::run` | | lib:Eigen | - | "incr==1" |
| 0x180157CC0 | `triangular_solve_vector<...>::run` | | lib:Eigen | - | |
| 0x180158050 | `pimax::totem::ImuInitializer::solveGyroscopeBias` | `void (std::map<double,ImageFrame>&, std::vector<Vector3d>& Bgs, Vector3d ba)` | project | modified | log-map residual, LDLT<Matrix3d>, "INFO=ImuInitial\|\|frame_num..." , repropagate(ba, Bgs[0]) |
| 0x180158C90 | `pimax::totem::ImuInitializer::computeStd` (name ours) | `double (const std::vector<double>&)` | project | new | population std-dev (accumulate / transform / inner_product) |
| 0x180158F30 | `std::_Ref_count_obj2<CeresBackendInterface>::~_Ref_count_obj2` (scalar deleting) | | lib:STL | - | |
| 0x180158F60 | `std::_Ref_count_obj2<CeresBackendInterface>::_Destroy` | | lib:STL | - | calls ~CeresBackendInterface 0x180009950 |
| 0x180158F70 | `pimax::totem::ceres_backend_factory::makeBackend` | `CeresBackendInterface::Ptr (const CameraBundlePtr&)` | project | modified | constants instead of ROS params; no motion detector; no startThread |
| 0x1801590E0 | `std::vector<T64>::_Emplace_reallocate` (64-byte, 7 qwords used) | | lib:STL | - | for next object (0x180159900) |
| 0x180159250 | `std::vector<T24>::_Emplace_reallocate(pos, a, b, c)` | | lib:STL | - | caller 0x18015AEC0 |
| 0x180159410 | `std::string::_Reallocate_grow_by` (insert lambda) | | lib:STL | - | many callers |
| 0x1801595C0 | `std::_UIntegral_to_buff<char, unsigned long long>` | | lib:STL (to_string) | - | |
| 0x180159600 | Eigen expression helper (Vector3d copy + block) | | lib:Eigen | - | caller 0x18015E250 |
| 0x180159640 | `std::copy(istream_iterator<int>, istream_iterator<int>, back_inserter(vector<int>))` | | lib:STL | - | |
| 0x180159710 | `cv::cv2eigen<double, …>(const cv::Mat&, Eigen::Matrix&)` | | lib:OpenCV inline (opencv2/core/eigen.hpp) | - | |
| 0x180159900 | `std::vector<T64>::push_back` | | lib:STL | - | |
| 0x180159940 | `std::make_shared<FrameProcessor>(…10 args)` | | lib:STL | - | block 0x1000, ctor 0x1800B2460 |
| 0x180159A40 | `std::make_shared<vk::cameras::NCamera>(int,int,int,const char*)` | | lib:STL | - | block 0x78, ctor 0x1801B4050 |
| 0x180159B90 | `cv::Scalar_<double>::Scalar_(double)` | | lib:OpenCV inline | - | |
| 0x180159BB0 | `std::basic_istringstream<char>::basic_istringstream(const std::string&)` | | lib:STL | - | |
| 0x180159C60 | `std::basic_stringbuf<char>::_Init` | | lib:STL | - | |
| 0x180159DC0 | `std::istream_iterator<int>::istream_iterator(istream&)` | | lib:STL | - | |

Other functions of these objects that are *outside* this range (for the coordinator): StereoTriangulation ctor
0x180145320, `StereoTriangulation::triangulatePoint` 0x180145470 (DLT, `this` unused, returns error or
-1/-2/-3/-4), `computeStdDev` 0x180146360 (`pow(x-mean, 2.0)` form), JacobiSVD<Matrix4d> 0x180146440;
ImuInitializer ctor 0x1800E1FD0 / dtor 0x1800E4E90.

## Types

### StereoTriangulationOptions (32 bytes) — sure

| off | type | name | evidence |
|---|---|---|---|
| 0 | size_t | triangulate_n_features | `numLandmarks() >= *this`, `n_desired = *this - numLandmarks()` |
| 8 | double | mean_depth_inv | 6th arg of findEpipolarMatchDirect |
| 16 | double | min_depth_inv | 7th arg |
| 24 | double | max_depth_inv | 8th arg |

### StereoTriangulation (0x90) — sure

| off | type | name | evidence |
|---|---|---|---|
| 0 | StereoTriangulationOptions | options_ | |
| 32 | DetectorPtr | feature_detector_ | `(*(this[4]))->vtbl[1]` = detect |
| 48 | cv::Mat | mask_ (Pimax) | `copyTo(this+48)`, `cv::circle(this+48, ...)` |

### ImuInitializer (name ours, 136 bytes, no vtable) — layout sure, names VINS

| off | type | name | evidence |
|---|---|---|---|
| 0 | Vector3d | G | ctor (0,0,9.80667); caller sets z from config; `G.norm()` in Linear/RefineGravity |
| 24 | std::vector<Matrix3d, aligned_allocator> | RIC | `*(this+24)` = RIC[0]; dtor uses plain free() |
| 48 | std::vector<Vector3d, aligned_allocator> | TIC | `*(this+48)` = TIC[0] |
| 80 | Transformation (16-aligned) | T_w0_ (ours) | ctor identity; written by 0x180156C10 |

### ImageFrame (value of `std::map<double, ImageFrame>`; owned by the 0x180110490 chunk)

Offsets relative to the tree node (pair at node+32, key double at node+32). With ImageFrame 16-aligned it
starts at node+48 and the VINS members fall exactly on the observed offsets:

| node off | ImageFrame off | type | name | used by c13 |
|---|---|---|---|---|
| +48 | 0 | std::map<int, vector<pair<int,Matrix<double,7,1>>>> | points (VINS) | - |
| +64 | 16 | double | t | - |
| +72 | 24 | Matrix3d | R | all |
| +144 | 96 | Vector3d | T | LinearAlignment, RefineGravity |
| +168 | 120 | IntegrationBase* | pre_integration | all |
| +176 | 128 | bool | is_key_frame (VINS) | - |
| +192 | 144 | Transformation | T_c0_body_ (ours) | read 0x180156C10 |
| +256 | 208 | Transformation | ? | - |
| +320 | 272 | Transformation | ? | - |
| +384 | 336 | Transformation | T_w_body_ (ours) | written 0x180156C10 |
| +448 | 400 | Vector3d | V_w_ (ours) | written 0x180156C10 |
| +472 | 424 | Vector3d | Bg_ (ours) | written 0x180156C10 |

### IntegrationBase (c08 owns) — offsets c13 relies on (agree with draft/c08_preint_ground)
jacobian +256 (block<3,3>(O_R,O_BG) = +1720/+1840/+1960), sum_dt +10416, delta_p +10424, delta_q +10448,
delta_v +10480; `repropagate(ba, bg)` 0x18011AFE0.

### Frame (central) — fields used here

| off | type | name | evidence / note |
|---|---|---|---|
| 40 | CameraPtr | cam_ | getMask = cam_+56 (0x180094880); `*cam()` passed to computeNormalizedBearingVectors |
| 64 | Transformation | T_f_w_ | `T_world_cam()` = inverse (0x180013040) |
| 128 | std::vector<cv::Mat> | img_pyr_ | detect arg; `[0]` rows/cols/data/step |
| 224 | double | img_mean_intensity_ (c11 name) | `> 100.0` |
| 232 | bool | img_too_dark_ (c11 name) | OR-ed with the above (TODO(verify) meaning in this context) |
| 552 | size_t | num_features_ | |
| 560 | Keypoints (2xN float) | px_vec_ | |
| 576 | Bearings (3xN float) | f_vec_ | |
| 592 | Bearings (3xN float) | **f_vec_raw_ (ours)** | 2nd output of computeNormalizedBearingVectors; FeatureWrapper+56 is a Block<…,3,1> of it with 12-byte stride. c11 guessed `VectorXd score_vec_` here — wrong |
| 608 | VectorXf | score_vec_ | FeatureWrapper+104 points here (4-byte stride) |
| 624 | VectorXi | level_vec_ | FeatureWrapper+112 |
| 640 | Gradients (2xN float) | grad_vec_ | FeatureWrapper+80 |
| 656 | std::vector<FeatureType> | type_vec_ | |
| 680 | std::vector<PointPtr> | landmark_vec_ | |
| 704 | VectorXi | track_id_vec_ | |
| 720 | std::vector<SeedRef> (24 B) | seed_ref_vec_ | FeatureWrapper+128; `.keyframe` tested in the mask loop |

### FeatureWrapper (144 bytes, built by 0x180094580) — sure
type& +0, px Block +8, f Block +32, f_raw Block +56 (Pimax), grad Block +80, score& +104, level& +112,
landmark& +120, seed_ref& +128, track_id& +136.

### Matcher (c07 owns) — additions from the inlined ctor in 0x180147110
Options defaults: align_max_iter 10, max_epi_length_optim 2.0, max_epi_search_steps 100, subpix_refinement,
epi_search_edgelet_filtering, scan_on_unit_sphere = true, **epi_search_edgelet_max_angle 0.5** (upstream 0.7),
verbose false, use_affine_warp_ true, affine_est_offset_ true, affine_est_gain_ false,
**max_patch_diff_ratio 2.5** (upstream 2.0). `f_cur_` (+320) zero-initialised; **extra Vector3f at +332**
(zero-initialised, read by triangulatePoint); **sizeof(Matcher) = 352** (vector<Matcher> stride), not 336.

### CeresBackendOptions (56 bytes) / CeresBackendInterfaceOptions (64 bytes)
Full layouts and factory values are in `interface/ceres_backend_factory.h` (verified byte-by-byte from the
instruction encodings at 0x180158FE8: +8/+12 are 32-bit stores). Differences from upstream:
`num_iterations`/`num_threads` are `int`, `max_fixed_lm_in_ceres_` removed; interface options lose
`refine_extrinsics`, `extrinsics_pos_sigma_meter`, `extrinsics_rot_sigma_rad`; the CeresBackendInterface
ctor takes no MotionDetectorOptions. `sizeof(CeresBackendInterface) = 0x4F0`.

## External interfaces

| address | inferred signature / meaning |
|---|---|
| 0x18000C2C0 / 0x18000F500 / 0x18000F6A0 | LOGE / LOGI / LOGW (global logger 0x18046A000) |
| 0x180118580 | `size_t Frame::numLandmarks() const` (counts track_id_vec_ > -1 over num_features_) |
| 0x180094880 | `const cv::Mat& Frame::getMask() const` = `cam_->mask` (+56) |
| 0x180093BB0 | `frame_utils::computeNormalizedBearingVectors(const Keypoints&, const Camera&, Bearings* f, Bearings* f_raw)` |
| 0x180095730 | `void Frame::resizeFeatureStorage(size_t)` |
| 0x180094580 | `FeatureWrapper Frame::getFeatureWrapper(size_t)` |
| 0x180144620 | `std::vector<FeatureType>::insert(pos, first, last)` |
| 0x1801448C0 | `std::shuffle` with `_Rng_from_urng<mt19937>` |
| 0x1800AE820 | `Matcher::MatchResult Matcher::findEpipolarMatchDirect(const Frame&, const Frame&, const Transformation& T_cur_ref, const FeatureWrapper&, double, double, double, float& depth)` |
| 0x180145470 | `double StereoTriangulation::triangulatePoint(frame0, frame1, T_f1f0, P0, P1, matcher, ref_ftr, Vector3d* xyz_f0)` |
| 0x180146360 | `double computeStdDev(const std::vector<double>&, double mean)` |
| 0x180099F80 | `Point::Point(const Vector3f& pos)` (make_shared block 0x98) |
| 0x18009A530 | `void Point::addObservation(const FramePtr&, size_t)` |
| 0x180013040 | `Transformation::inverse()` (used as Frame::T_world_cam) |
| 0x180009B80 | `Transformation operator*(const Transformation&) const` (normalises the result quaternion unconditionally) |
| 0x18008A270 | `Transformation::cast<float>()` |
| 0x180029DF0 | `RotationQuaternion::getRotationMatrix()` / `Quaterniond::toRotationMatrix()` |
| 0x180014A20 / 0x180009C80 / 0x180014680 | kindr rotate(v) / quaternion product / in-place normalise |
| 0x1800089C0 | `kindr RotationQuaternion(const Eigen::Quaterniond&)` (CHECK_NEAR, minkindr line 73) |
| 0x180020AF0 | `Eigen::Quaterniond(const Matrix3d&)` |
| 0x1800DD6E0 | `Eigen::Quaterniond::setFromTwoVectors` |
| 0x1800132A0 | `Eigen::Quaterniond::inverse()` |
| 0x18011AFE0 | `IntegrationBase::repropagate(const Vector3d& ba, const Vector3d& bg)` |
| 0x180008CD0 | `CeresBackendInterface::CeresBackendInterface(const CeresBackendInterfaceOptions&, const CeresBackendOptions&, const CameraBundlePtr&)` |
| 0x180009950 | `CeresBackendInterface::~CeresBackendInterface()` |
| 0x180354E20 / 0x18035AC70 / 0x1803551B0 | glog `LogMessage(file, line)` / `stream()` / `~LogMessage` |
| env_8 | glog `FLAGS_v` (VLOG_IS_ON) |

Callers into this chunk: `compute` from 0x18012BB60 (initialization.cpp, is_init = true, only when both
frames have img_mean_intensity_ > 15) and 0x1800B2F80 (frame processor, is_init = false);
`VisualIMUAlignment` from 0x180110490; `makeBackend` from 0x1801663F0 (headset init).

## Constants / config defaults

* compute: detector budget `400 - n` (is_init) / `256 - n`; brightness pre-check when
  `is_init && (img_too_dark_ || img_mean_intensity_ > 100.0)`: 5x5 window, pixel "bad" if `< 15 || > 130`,
  reject when `bad / (2*N) > 0.2`; mask circles radius 3 filled; Matcher `max_epi_search_steps = 500`,
  `subpix_refinement = true`; depth window `(0.05f, 10.0f]`; parallax `>= 0.5` deg; error gate
  `(e - mean)/std <= 2.0`.
* solveGyroscopeBias: small-angle threshold 0.001 rad (log map), `2.0 * vec` below it.
* LinearAlignment: `n_state = 3N + 4`, scale `x(n-1) * 0.01`, A,b `* 1000.0`; static if all position
  std-devs `< 0.001`; static: `|g|-|G| <= 0.1`; moving: `<= 0.4` and `0.75 <= s <= 1.25`.
* RefineGravity: `n_state = 3N + 3`, max 10 iterations, stop when `|dg| < 0.001`.
* ImuInitializer::G default (0, 0, 9.80667) (0x40239D03D9A95422).
* makeBackend: see ceres_backend_factory.h.

## Quirks / bugs worth preserving

* compute: new scores are never copied into frame0->score_vec_ (the detector's `new_scores` is dropped).
* compute: `T_f1f0` is renormalised again right after the (already normalising) transformation product.
* compute: if all accepted candidates have identical errors, `stddev == 0` → `0/0 = NaN` → every candidate
  is rejected (the gate is `<= 2.0`, compiled `jb`, so NaN fails). NaN depth/error/parallax *pass* their
  filters (reject-form comparisons).
* compute: a failed match counts towards the loop-exit check, a filtered successful match does not
  (`continue` skips the `n_succeded >= n_desired` test).
* compute: four unused local vectors (incl. `std::vector<Matcher>`) are constructed/destroyed.
  assigned in candidate order.
* solveGyroscopeBias: `q_ij.normalized()` is computed and stored but never used; residual uses the raw q_ij.
* solveGyroscopeBias: Bgs updated for `all_image_frame.size()` entries (caller must size Bgs accordingly).
* RefineGravity: A and b keep accumulating across iterations and are multiplied by 1000 every iteration
  (inherited from VINS-Mono).
* LinearAlignment: upstream's final `s < 0` rejection is gone; acceptance tests are reject-forms (NaN passes).
* LinearAlignment / RefineGravity: `/100.0` became `* 0.01` (Constant node holds 0.01) — not exactly
  equal to a division; keep the multiplication.
* computeStatesInGravityFrame: no `is_key_frame` filter (VINS has one); every frame gets a velocity.

## Open questions

* File/class name of the IMU initializer (only constraint: object sorts between
  `frontend/stereo_triangulation` and `interface/`); g2R might live in a separate utility.cpp.
* Names of ImageFrame's three Pimax transformations and the world-state members.
* Exact meaning of Frame +232 here (c11 calls it img_too_dark_).
* Matcher's extra Vector3f at +332 (c07 should add it; also sizeof 352).
* Namespace of makeBackend (`ceres_backend_factory` assumed).
