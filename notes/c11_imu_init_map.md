# Chunk `c11_imu_init_map` — 0x18012A2B0 – 0x18013C500 (89 functions)

Drafts: `draft/c11_imu_init_map/`
- `frontend/imu_processor.{h,cpp}`: `ImuProcessor` (svo `imu_handler.cpp`).
- `frontend/initialization.{h,cpp}`: `AbstractInitialization`, `StereoInit`, `makeInitializer`.
- `frontend/map.{h,cpp}`: `Map`.
- `frontend/pose_optimizer.{h,cpp}`: first half of `pose_optimizer.obj`. It contains:
  - the ctor, `getDefaultSolverOptions` and `applyPrior`;
  - `evaluateErrorImpl` and the six `pose_optimizer_utils::calculate*Residual*` helpers.
- `common/frame_jacobians.h`: out-of-line copies of the header-inline `Frame::jacobian_xyz2{uv,img,f}_imu`. Merge them into the central `frame.h`.
- `common/c11_external.h`: declarations of the types from other chunks that this code uses, with their binary offsets. It is a placeholder for the central headers.

## Summary

Object files in the range, in image order:

| range | object | content |
|---|---|---|
| 0x18012A2B0–0x18012B320 | `frontend/imu_processor.cpp` (tail) | 6 functions. The rest of the object (ctor, `addImuMeasurement`, …) lies before 0x18012A2B0. |
| 0x18012B320–0x18012C0B0 | `frontend/initialization.cpp` | Only `StereoInit` survives. |
| 0x18012C0B0–0x18012EFE0 | `frontend/map.cpp` | Includes the STL instantiations used by `getClosestNKeyframesWithOverlap`. |
| 0x18012EFE0–0x18013C500 | `frontend/pose_optimizer.cpp` (first half) | Starts with about 30 Eigen/kindr COMDATs (LDLT, IOFormat printing, products, `kindr::log`, `RotationQuaternion(w,x,y,z)`). |

The second half of `pose_optimizer.obj` continues past 0x18013C500. It holds `optimizeGaussNewton` ("Failure", "Error increased. Stop optimizing."), `run` (" corner outliers and "), `removeOutliers` ("removeOutliers has NULL"), `update` and `Transformation::exp`.

**Upstream fidelity**
- All four objects are close forks of rpg_svo_pro_open.
- The Pimax changes are systematic:
  - float storage for samples, landmarks, bearing vectors, key points and gradients (see Types);
  - glog warnings replaced by the custom logger;
  - the IMU–camera delay correction removed;
  - the `Map` keyframe container changed from `unordered_map` to `std::map`.
- Notable functional changes:
  - `StereoInit::addFrameBundle`: brightness gating, 4-camera pairs, and a quirky success test;
  - `Map::getOverlapKeyframes`: a second pose-proximity pass;
  - `Map::getClosestNKeyframesWithOverlap`: a completely new ranking;
  - `calculateEdgeletResidualBearingVectorDiff`: float normalisation and a division guard;
  - `getDefaultSolverOptions`: `max_iter = 6`.

**Confidence**
- High: control flow, constants and strings, and the layouts of `Map`, `AbstractInitialization`/`StereoInit` and `PoseOptimizer`.
- Medium:
  - the exact spelling of a few inlined helpers (quaternion exp, quaternion angle, the inverse-rotate in `getClosestN`);
  - the `ImuProcessor` layout below +256.
- The residual helpers were checked for structure, constants and float/double conversion points. Their Eigen expression order follows upstream.
- Not verified: the products inside the 6×6 H/g accumulations, beyond the fact that they are the upstream `noalias() +=` forms.

## Function table

kind: `project` = reconstructed; `lib:` = library/template/compiler-generated, not reconstructed.

| address | name | signature | kind | upstream | summary |
|---|---|---|---|---|---|
| 0x18012A2B0 | `ImuProcessor::getInitialAttitude` | `bool (double timestamp, Quaternion& R_imu_world) const` | project | modified | inlined getClosestMeasurement (no delay, LOGW); gravity frame with fixed helper axis p=(0,0,1); VLOG removed |
| 0x18012A700 | `ImuProcessor::getMeasurements` | `bool (double old_ts, double new_ts, bool delete_old, ImuMeasurements&)` | project | modified | LOGE instead of assert; no delay; "too old" check removed; `assign` instead of insert |
| 0x18012A950 | `ImuProcessor::getMeasurementsContainingEdges` | `bool (double ts, ImuMeasurements&, bool remove)` | project | modified | glog warnings @109/@119 kept; LOGW "need older imu data!"; assign |
| 0x18012AB60 | `ImuProcessor::getRelativeRotationPrior` | `bool (double, double, bool, Quaternion&)` | project | modified | integrates gyro (float) – bias with exp threshold 1e-12; normalises after every step |
| 0x18012B090 | `ImuProcessor::limitMeasurementsSize` (name TODO) | `void ()` | project | new | if size ≥ 3000 erase the older half |
| 0x18012B140 | `ImuProcessor::waitTill` | `bool (double ts, double timeout)` | project | modified | busy wait with Sleep(1), VLOG(20) @342, silent timeout from start |
| 0x18012B320 | `AbstractInitialization::AbstractInitialization` | `(const InitializationOptions&, const FeatureTrackerOptions&, const DetectorOptions&, const CameraBundlePtr&)` | project | identical | copies options, `new FeatureTracker` (aligned malloc 0x98) |
| 0x18012B4E0 | `StereoInit::StereoInit` | same | project | identical | makeDetector(cam 0); `new StereoTriangulation(opts{120,1/3,1,1/50}, detector_)` |
| 0x18012B740 | `std::vector<std::vector<FeatureTrack>>::_Tidy` | | lib:std | - | member of FeatureTracker |
| 0x18012B860 | `AbstractInitialization::~AbstractInitialization` | | project | identical | releases frames_ref_, deletes tracker_ |
| 0x18012B8E0 | `AbstractInitialization` scalar deleting dtor | | lib:compiler | - | vtbl[0]; aligned delete = free |
| 0x18012B930 | `FeatureTracker` scalar deleting dtor | | lib:compiler | - | implicit ~FeatureTracker (vectors of tracks/detectors) |
| 0x18012BAB0 | `StereoInit` scalar deleting dtor | | lib:compiler | - | ~detector_, ~stereo_ (unique_ptr dtor 0x1800B2880), base dtor |
| 0x18012BB60 | `StereoInit::addFrameBundle` | `InitResult (const FrameBundlePtr&)` | project | modified | see Quirks; LOGI messages; brightness > 15 gating |
| 0x18012BF30 | `initialization_utils::makeInitializer` | `UniquePtr (const InitializationOptions&, const FeatureTrackerOptions&, const DetectorOptions&, const CameraBundlePtr&)` | project | modified | only kStereo(=0); else LOG(FATAL) @270 |
| 0x18012C030 | `AbstractInitialization::reset` | `virtual void ()` | project | modified | no have_depth_prior_ |
| 0x18012C0B0 | `vector<pair<FramePtr,double>>::_Emplace_reallocate<const FramePtr&, double&>` | | lib:std | - | |
| 0x18012C240 | `vector<pair<FramePtr,double>>::_Emplace_reallocate<pair<FramePtr,double>>` | | lib:std | - | |
| 0x18012C3C0 | `std::_Insertion_sort_unchecked<pair*, getClosestN lambda>` | | lib:std | - | |
| 0x18012C5C0 | `std::_Med3_unchecked<…>` | | lib:std | - | |
| 0x18012C710 | `std::_Partition_by_median_guess_unchecked<…>` | | lib:std | - | |
| 0x18012CBC0 | `vector<pair<FramePtr,double>>::_Resize_reallocate` | | lib:std | - | resize(N) |
| 0x18012CCE0 | `std::nth_element<pair*, getClosestN lambda>` | | lib:std | - | lambda captures cur_frame by value |
| 0x18012CE00 | `Map::Map` | `()` | project | identical | |
| 0x18012CEB0 | `Map::~Map` | | project | modified | inlined reset() (+ last_removed_kf_.reset()), LOGI "Map destructed" |
| 0x18012D010 | `vector<pair<FramePtr,double>>::_Change_array` | | lib:std | - | |
| 0x18012D0D0 | `std::deque<int>::_Growmap` | | lib:std | - | also used by other objects |
| 0x18012D2B0 | `std::deque<int>::_Tidy` | | lib:std | - | |
| 0x18012D370 | `std::_Uninitialized_move<pair<FramePtr,double>*>` | | lib:std | - | |
| 0x18012D3E0 | `std::_Uninitialized_move<…>` (variant) | | lib:std | - | |
| 0x18012D450 | `std::_Hash<unordered_map<int,KeypointIdentifier>>::erase(first,last)` | | lib:std | - | from obs_.clear() |
| 0x18012D680 | `Map::addKeyframe` | `void (const FramePtr&)` | project | modified | VLOG(100) @87; always push_back id |
| 0x18012D900 | `Map::getClosestNKeyframesWithOverlap` | `void (const FramePtr&, const size_t& num, std::vector<FramePtr>*) const` | project | modified | new ranking (principal-point reprojection + 100·dist), same-bundle first |
| 0x18012E1B0 | `Map::getOverlapKeyframes` | `void (const FramePtr&, std::vector<pair<FramePtr,double>>*) const` | project | modified | reverse map order; extra pose-proximity pass |
| 0x18012E930 | `Map::allKeyPointsVisible` (name TODO) | `bool (const FramePtr& frame, const FramePtr& kf) const` | project | new | count of visible key points == 5 |
| 0x18012E9F0 | `Map::removeKeyframe` | `void (int)` | project | modified | LOG(WARNING) @32; uses track_id_vec_, clears landmark + track id |
| 0x18012EC70 | `Map::removeOldestKeyframe` | `void ()` | project | identical | |
| 0x18012ECE0 | `std::vector<FramePtr>::reserve` | | lib:std | - | |
| 0x18012ED20 | `Map::reset` | `void ()` | project | modified | + last_removed_kf_.reset() |
| 0x18012EDE0 | `Map::safeDeletePoint` | `void (PointPtr&)` | project | modified | null guard, LOGE, resets pt |
| 0x18012EFE0 | `Eigen::Ref<const Matrix<float,2,1>>` ctor | | lib:Eigen | - | used by removeOutliers (next chunk) |
| 0x18012F020 | `Eigen::Ref<const Matrix<float,3,1>>` ctor | | lib:Eigen | - | " |
| 0x18012F060 | `Eigen::operator<<(ostream&, DenseBase)` (IOFormat) #1 | | lib:Eigen | - | used by the solver's verbose printing |
| 0x18012F3A0 | `Eigen::operator<<(ostream&, DenseBase)` #2 | | lib:Eigen | - | |
| 0x18012F6E0 | `std::vector<float>::_Emplace_reallocate<double>` | | lib:std | - | evaluateErrorImpl emplace_back |
| 0x18012F7E0 | `Eigen::LDLT<Matrix6d>::_solve_impl` | | lib:Eigen | - | |
| 0x18012FA90 | `kindr::minimal::detail::arcSinXOverX<double>` | | lib:minkindr | - | |
| 0x18012FAF0 | `Eigen::LDLT<Matrix6d>::compute` | | lib:Eigen | - | |
| 0x18012FE00 | `kindr::minimal::detail::isLessThenEpsilons4thRoot<double>` | | lib:minkindr | - | static pow(eps, 0.25) |
| 0x18012FE90 | `Eigen` max-abs visitor (LDLT pivoting) | | lib:Eigen | - | |
| 0x180130030 | `Eigen::internal::print_matrix<…>` #1 | | lib:Eigen | - | |
| 0x180130420 | `Eigen::internal::print_matrix<…>` #2 | | lib:Eigen | - | |
| 0x180130890 | Eigen product/assign kernel | | lib:Eigen | - | callee of 0x180131380 |
| 0x180130AD0 | Eigen product/assign kernel | | lib:Eigen | - | " |
| 0x180130D10 | Eigen product/assign kernel | | lib:Eigen | - | " |
| 0x180130F70 | Eigen product/assign kernel | | lib:Eigen | - | " |
| 0x1801311D0 | Eigen assignment kernel | | lib:Eigen | - | used by LDLT |
| 0x180131380 | Eigen generic product evalTo (3x3) | | lib:Eigen | - | used by kindr log, 0x180163150 |
| 0x180131960 | `Eigen::internal::ldlt_inplace<Lower>::unblocked` | | lib:Eigen | - | |
| 0x180132960 | Eigen MapBase ctor (1x2) | | lib:Eigen | - | |
| 0x180132A10 | Eigen Block ctor | | lib:Eigen | - | |
| 0x180132AC0 | Eigen Block ctor | | lib:Eigen | - | |
| 0x180132B70 | `kindr::minimal::RotationQuaternionTemplate<double>(w,x,y,z)` | | lib:minkindr | - | CHECK_NEAR(squaredNorm,1,1e-4); file `thirdparty\minkindr\...\rotation-quaternion-inl.h` line 59 |
| 0x180132D90 | `PoseOptimizer::PoseOptimizer` | `(SolverOptions)` | project | identical | |
| 0x180132FB0 | `Eigen::CommaInitializer<Matrix<double,2,3>>::operator,` | | lib:Eigen | - | |
| 0x180133074 | ICF-folded `vcall{0}` thunk | | lib:compiler | - | IDA FLIRT name `??_9_Concurrent_queue_base_v4…` |
| 0x180133080 | `MiniLeastSquaresSolver<6,…>` scalar deleting dtor | | lib:compiler | - | |
| 0x1801330C0 | `MADScaleEstimator` scalar deleting dtor | | lib:compiler | - | |
| 0x1801330F0 | `PoseOptimizer` scalar deleting dtor | | lib:compiler | - | ~ofstream, ~frame_bundle_, free |
| 0x1801331A0 | `PoseOptimizer::applyPrior` vcall thunk `vcall{8}` | | lib:compiler | - | from `&Implementation::applyPrior` comparison |
| 0x1801331C0 | `PoseOptimizer::applyPrior` | `virtual void (const State&)` | project | identical | "applying rotation prior, I = " on std::cout if verbose |
| 0x1801338F0 | `pose_optimizer_utils::calculateEdgeletResidualBearingVectorDiff` | see header | project | modified | float f_est, 1e-8 guard |
| 0x180135AF0 | `pose_optimizer_utils::calculateEdgeletResidualImagePlane` | | project | modified | float inputs |
| 0x1801364D0 | `pose_optimizer_utils::calculateEdgeletResidualUnitPlane` | | project | modified | float inputs |
| 0x180136EA0 | `pose_optimizer_utils::calculateFeatureResidualBearingVectorDiff` | | project | modified | float inputs |
| 0x180138430 | `pose_optimizer_utils::calculateFeatureResidualImagePlane` | | project | modified | float inputs |
| 0x180139AE0 | `pose_optimizer_utils::calculateFeatureResidualUnitPlane` | | project | modified | float inputs |
| 0x18013A840 | `PoseOptimizer::evaluateErrorImpl` | `double (const Transformation&, HessianMatrix*, GradientVector*, std::vector<float>*)` | project | modified | track_id gate, no seeds, no CHECK_GE |
| 0x18013B260 | `Eigen::CommaInitializer<Matrix<double,6,1>>::finished` | | lib:Eigen | - | kindr log |
| 0x18013B2B0 | `PoseOptimizer::getDefaultSolverOptions` | `static SolverOptions ()` | project | modified | max_iter 6 |
| 0x18013B300 | `Frame::jacobian_xyz2f_imu` | `static void (const Transformation&, const Vector3d&, Matrix<double,3,6>&)` | project | modified | 1/(sqrt(n2)*n2) instead of 1/pow(n2,1.5) |
| 0x18013B8F0 | `Frame::jacobian_xyz2img_imu` | `static void (…, const Matrix<double,2,3>& J_cam, Matrix<double,2,6>&)` | project | identical | |
| 0x18013BC70 | `Frame::jacobian_xyz2uv_imu` | `static void (…, Matrix<double,2,6>&)` | project | identical | |
| 0x18013C150 | `kindr::minimal::QuatTransformationTemplate<double>::log` | | lib:minkindr | - | |
| 0x18013C4A0 | `Eigen::LDLT<Matrix6d>::matrixL` (m_isInitialized assert) | | lib:Eigen | - | |

## Types

### `ImuMeasurement` (32 bytes, deque block = 1)
| off | type | name | evidence |
|---|---|---|---|
| 0 | double | timestamp_ | all loops |
| 8 | Vector3f | angular_velocity_ | getRelativeRotationPrior (+8,+12,+16) |
| 20 | Vector3f | linear_acceleration_ | getInitialAttitude (+20,+28) |

`ImuMeasurements = std::deque<ImuMeasurement, Eigen::aligned_allocator<ImuMeasurement>>`. The proxy is allocated with malloc and has the Eigen alignment assert. The newest sample is at the front.

### `ImuProcessor` (no vtable; size unknown; `_Ref_count_obj2<ImuProcessor>` vtable 0x1803B8100)
| off | type | name | sure? |
|---|---|---|---|
| 0 | ImuCalibration | imu_calib_ (`max_imu_delta_t` @ +8) | +8 sure, rest guessed |
| 128 | std::mutex | bias_mut_ | guess (gap) |
| 208 | Vector3d | acc_bias_ | guess |
| 232 | Vector3d | omega_bias_ | sure (getRelativeRotationPrior) |
| 256 | std::mutex | measurements_mut_ | sure |
| 336 | ImuMeasurements | measurements_ (map +344, mapsize +352, off +360, size +368) | sure |

Upstream `options_` (IMUHandlerOptions) is not in front of `imu_calib_`. It is either gone or placed after the deque.

### `InitializationOptions` (64 bytes)
| off | type | name | evidence |
|---|---|---|---|
| 0 | InitializerType (int) | init_type | makeInitializer: `!= 0` → FATAL ⇒ kStereo == 0 |
| 8 | double | init_min_disparity | upstream order |
| 16 | double | init_disparity_pivot_ratio | " |
| 24 | size_t | init_min_features | read in addFrameBundle (obj+40) |
| 32 | double | init_min_features_factor | upstream order |
| 40 | size_t | init_min_tracked | " |
| 48 | size_t | init_min_inliers | " |
| 56 | double | reproj_error_thresh | " |

Upstream `expected_avg_depth` and `init_min_depth_error` are gone: the struct is copied as exactly 64 bytes.

### `AbstractInitialization` (280 bytes; vtable 0x1803B7068 = {deleting dtor 0x18012B8E0, _purecall, reset 0x18012C030})
| off | type | name | evidence |
|---|---|---|---|
| 0 | vfptr | – | the vfptr is padded to 16 by MSVC because of the Eigen members; nothing at +8 |
| 16 | InitializationOptions | options_ | ctor copy |
| 80 | unique_ptr<FeatureTracker> | tracker_ | ctor / reset |
| 88 | FrameBundlePtr | frames_ref_ | reset / addFrameBundle |
| 112 | Transformation | T_cur_from_ref_ | identity written by the kindr ctor |
| 176 | Eigen::Quaterniond | R_ref_world_ | alignment assert only, uninitialised |
| 208 | Eigen::Quaterniond | R_cur_world_ | " |
| 240 | Vector3d | t_ref_cur_ | gap |
| 264 | bool | have_rotation_prior_ | 16-bit store |
| 265 | bool | have_translation_prior_ | 16-bit store |
| 272 | double | depth_at_current_frame_ = 1.0 | ctor |

### `StereoInit` (320 bytes = malloc(0x140); vtable 0x1803B7088 = {0x18012BAB0, addFrameBundle 0x18012BB60, reset 0x18012C030})
| off | type | name |
|---|---|---|
| 288 | unique_ptr<StereoTriangulation> | stereo_ (object 0x90: options 32 + DetectorPtr + cv::Mat @+48) |
| 296 | shared_ptr<AbstractDetector> | detector_ |

### `Map` (0xB8 = 184 bytes, no vtable)
| off | type | name |
|---|---|---|
| 0 | std::map<int, FramePtr> | keyframes_ (node 0x38: key @+32, FramePtr @+40) |
| 16 | vector<PointPtr> | points_to_delete_ |
| 40 | std::mutex | points_to_delete_mutex_ |
| 120 | int | last_added_kf_id_ (= -1) |
| 128 | std::deque<int> | sorted_keyframe_ids_ |
| 168 | FramePtr | last_removed_kf_ |

Differences from upstream: `keyframes_` is an ordered `std::map`, not `unordered_map`. Several upstream members are absent from the image (see map.h).

### `PoseOptimizer` (0x4E0 bytes; vtable 0x1803B7328 = {deleting dtor 0x1801330F0, applyPrior 0x1801331C0}; base vtable 0x1803B7318 = {0x180133080})
- Base `MiniLeastSquaresSolver<6, Transformation, PoseOptimizer>`: upstream layout, listed in pose_optimizer.h.
  - The options are at +16; their layout is the upstream one.
- Derived members:

| off | type | name | init |
|---|---|---|---|
| 896 | Statistics | stats_ | 0, 0 |
| 912 | FrameBundlePtr | frame_bundle_ | null |
| 928 | double | prior_lambda_ | uninit |
| 936 | MADScaleEstimator | scale_estimator_ | vtable 0x1803B7300 |
| 944 | TukeyWeightFunction | robust_weight_ | ctor 0x1801B5C30 |
| 960 | double | measurement_sigma_ | 1.0 |
| 968 | ErrorType (int) | err_type_ | 0 (kUnitPlane) |
| 976 | double | focal_length_ | 1.0 |
| 984 | std::ofstream | ofs_reproj_errors_ | ctor 0x180097B60 |

### Offsets used from other chunks' types
- **Frame**:

  | offset | member |
  |---|---|
  | +16 | id_ |
  | +20 | bundle_id_ |
  | +40 | cam_ |
  | +64 | T_f_w_ (q @+64, t @+96) |
  | +152 | key_pts_ (`vector<pair<int, Vector3f>>`, 16-byte elements) |
  | +224 | double mean image intensity |
  | +232 / +233 | too-dark / too-bright flags |
  | +320 | T_cam_imu_ (Transformation) |
  | +552 | num_features_ |
  | +560 | px_vec_ (Matrix2Xf) |
  | +576 | f_vec_ (Matrix3Xf) |
  | +592 | score_vec_ ? |
  | +608 | 16 bytes unknown |
  | +624 | level_vec_ (VectorXi) |
  | +640 | grad_vec_ (Matrix2Xf) |
  | +656 | type_vec_ (vector<uint8 FeatureType>) |
  | +680 | landmark_vec_ |
  | +704 | track_id_vec_ (VectorXi) |

- **Point**:
  - +4: pos_ (Vector3f).
  - +16: obs_ = `std::unordered_map<int, KeypointIdentifier>`. In a node, the key is at +16, then the frame weak_ptr at +24, frame_id at +40 and keypoint_index_ at +48.
- **FrameBundle**: frames_ at +0.
- **Camera vtable**:

  | offset | function |
  |---|---|
  | +16 | backProject3 |
  | +24 | project3 |
  | +48 | getIntrinsicParameters (returns VectorXd by value) |

  These are 8 bytes later than upstream vikit: there is one extra virtual before backProject3.

## External interfaces (calls out of the chunk)

| address | meaning |
|---|---|
| 0x18000C2C0 / 0x18000F500 / 0x18000F6A0 | LOGE / LOGI / LOGW (logger @0x18046A000) |
| 0x180354E20 / 0x180354E50 / 0x180354E80 | glog `LogMessage(file,line)` / `(file,line,severity)` / `LogMessageFatal(file,line)` |
| 0x18035AC70 / 0x1803551B0 / 0x180355280 | `LogMessage::stream()` / `~LogMessage` / `~LogMessageFatal` |
| 0x180354DC0, 0x180354EB0, 0x180355010, 0x180355D70, 0x180357BF0 | glog CHECK_OP helpers (kindr CHECK_NEAR) |
| `env_8` | `FLAGS_v` (VLOG_IS_ON(n) == FLAGS_v >= n) |
| 0x180006290 | `operator<<(ostream&, const char*)` |
| 0x180018500 | `std::endl` |
| 0x180014730 | `vk::Timer::Timer()` (steady clock read) |
| 0x180008720 / 0x180008B90 / 0x180091A60 / 0x180099E90 / 0x1800087D0 / 0x180008630 / 0x180008670 / 0x180008880 / 0x180018A80 / 0x180018B10 / 0x180039740 / 0x18008B930 / 0x180022BA0 / 0x1800DE250 / 0x180007870 / 0x180059CB0 | Eigen Map/Block/Ref/constant ctors (lib) |
| 0x180014A20 | Eigen `Quaterniond::_transformVector` (q * v) |
| 0x180013040 | `Transformation::inverse()` |
| 0x1800132A0 | kindr inverse helper (used by 0x180013040) |
| 0x180009B80 | `Transformation * Transformation` |
| 0x180020AF0 | Eigen `quaternion_assign_impl<Matrix3d>` (Quaterniond from rotation matrix) |
| 0x180020CE0 | `operator<<(ostream&, const Transformation&)` |
| 0x180029DF0 | `RotationQuaternion::getRotationMatrix()` |
| 0x180042410 | `vk::skew(Vector3d)` |
| 0x1800074E0 | Eigen fill (Quaternion zero) |
| 0x180129870 | `ImuMeasurements::_Assign_range` (deque::assign) |
| 0x18012A0C0 | `ImuMeasurements::erase(first,last)` |
| 0x180041E40 | `ImuMeasurements::clear()` |
| 0x180041CA0 | `ImuMeasurements::_Growmap` |
| 0x1801A7DF0 | `FeatureTracker::FeatureTracker(tracker_opts, detector_opts, cams)` |
| 0x1801A8030 | `FeatureTracker::reset()` |
| 0x1801B41A0 | `CameraBundle::getCameraShared(size_t)` |
| 0x1800AD840 | `feature_detection_utils::makeDetector(det_opts, cam)` |
| 0x180145320 | `StereoTriangulation::StereoTriangulation(opts, detector)` |
| 0x180147110 | `StereoTriangulation::compute(const FramePtr&, const FramePtr&, bool)` (Pimax 3rd arg) |
| 0x1800B2880 | `unique_ptr<StereoTriangulation>::~unique_ptr` |
| 0x1800B2920 | `shared_ptr<T>::operator=(const shared_ptr&)` |
| 0x180118580 | `Frame::numLandmarks()` |
| 0x180094FB0 | `Frame::isVisible(const Vector3d&, Vector2d* px)`. Pimax hard-codes image bounds 16 < u < 624. |
| 0x180094460 | `Frame::deleteLandmark(const size_t&)` |
| 0x18009BBE0 | `Point::removeObservation(int frame_id)` |
| 0x180095150 | `FrameBundle::numFeatures()` |
| 0x180024FE0 | `std::map<int,FramePtr>::_Extract` (erase) |
| 0x18000FE80 | `std::map` `_Insert_node` |
| 0x1800BCC20 | `std::map` `_Erase_tree` |
| 0x180091430 | `unordered_set<int>::insert` |
| 0x18000F840 | `_Hash` init |
| 0x1800199A0 | `~unordered_set` |
| 0x1800E5FB0 | `map` iterator `operator--` |
| 0x18009A110 | release of the lambda-captured shared_ptr |
| 0x1800E5BE0 | shared_ptr move-assign |
| 0x180020AC0 | shared_ptr copy |
| 0x1800908C0 / 0x180092C50 / 0x180006820 / 0x18001A030 / 0x18001CAB0 / 0x18001B2E0 / 0x1800BC580 / 0x18000F970 / 0x180027A70 / 0x180024DA0 / 0x180025640 | STL vector helpers |
| 0x1801B5C30 | `TukeyWeightFunction::TukeyWeightFunction()` |
| 0x180097B60 / 0x180097CD0 | `std::ofstream` ctor / dtor |
| 0x18013EA10, 0x18013EBD0 | LDLT triangular solves (next chunk) |

Called from outside the chunk:

| caller | calls |
|---|---|
| 0x1800DFDF0 (frame-processor ctor) | `makeInitializer`, `new Map`, `getDefaultSolverOptions`, `new PoseOptimizer` |
| 0x18016EC80 | `getInitialAttitude`, `getRelativeRotationPrior` |
| 0x18016A460 | `getMeasurementsContainingEdges`, `waitTill` |
| 0x180168640 | `limitMeasurementsSize` |
| 0x1800B2F80, 0x1800B4180, 0x180110490, 0x180127550 | `addKeyframe`, `removeKeyframe`, `removeOldestKeyframe`, `allKeyPointsVisible` |
| 0x180119260 | `getClosestNKeyframesWithOverlap`, `safeDeletePoint` |
| 0x18011B420 / 0x18011B8F0 (`resetVisionFrontendCommon*`) | `Map::reset` |

## Constants / config defaults

| what | value | address |
|---|---|---|
| quaternion exp small-angle threshold | 1e-12 | 0x1803AF290 |
| 1/48 | 0.020833333333333332 | 0x1803B1378 |
| IMU buffer trim threshold | 3000 (erase from size/2) | imm |
| waitTill sleep | Sleep(1) ms; VLOG level 20 | imm |
| StereoTriangulationOptions | {120, 0.3333333333333333, 1.0, 0.02} | 0x1803B71B0.. |
| StereoInit brightness gate | img_mean_intensity_ > 15.0 | imm |
| getClosestN | ray depth 50.0 (0x1803B72E0 / 0x1803B2C20), score 100.0·dist (0x1803B6D30) | |
| getOverlapKeyframes 2nd pass | only if ≤ 20 hits; |Δp| < 0.5 (0x1803AF2B8), angle < 1.0 (0x1803ADDD0); angle → 0 if |1−|dot|| < 1e-6 (0x1803AF298) | |
| `allKeyPointsVisible` | == 5 | imm |
| getDefaultSolverOptions | strategy GN(0), mu_init 0.01f (0x3F847AE140000000), nu_init 2.0, max_iter **6**, max_trials 5, stop/verbose false, eps 1e-6 | 0x18013B2B0 |
| PoseOptimizer base | mu_ 0.01 (0x3F847AE147AE147B), nu_ 2.0 | ctor |
| edgelet sigma factor | 2.0 | evaluateErrorImpl |
| EdgeletBearingVectorDiff guard | 1e-8 (0x1803B6C58) | |

## Quirks / bugs worth preserving

1. **`StereoInit::addFrameBundle`**:
   - The success test reads `frames->at(2)` whenever camera 0 has ≤ `init_min_features` landmarks. For a 2-frame bundle this throws `std::out_of_range` (MSVC `_Xrange`) instead of returning kFailure.
   - The success test uses `>` (so "≤ min" fails), where upstream failed on `<`.
   - Pairs whose frames are dark (mean intensity ≤ 15) are not triangulated at all, but the landmark count is still checked.
   - The `%d` format is used with `size_t` arguments.
2. **Delay correction gone:** `getMeasurements*`, `getClosestMeasurement` and `waitTill` ignore `imu_calib_.delay_imu_cam`.
3. **`assign` instead of `insert`:** `getMeasurements` / `getMeasurementsContainingEdges` call `deque::assign`, so pre-existing content of the output deque is overwritten instead of kept behind.
4. **`getInitialAttitude` helper axis:** it uses the fixed helper axis (0,0,1). If gravity is parallel to z, y = 0 and the result degenerates (Eigen `normalize()` leaves a zero vector).
5. **`waitTill`:** it measures the timeout from the start of the wait and returns false silently.
6. **`Map::removeKeyframe`:** it dereferences `landmark_vec_[i]` whenever `track_id_vec_(i) > -1`, without a null check.
7. **`Map::getOverlapKeyframes`:** the visibility pass hoists the distance computation before the key-point loop (no visible effect). The second pass compares `|dist|` (always ≥ 0) and `|angle|`.
8. **`Map::getClosestNKeyframesWithOverlap`:**
   - The ProjectionResult of `project3` is ignored, so px may be stale or garbage for points that project outside the image.
   - The principal point goes through `float`.
   - The nth_element comparator prefers keyframes whose `bundle_id_` equals the current frame's.
9. **`Map::safeDeletePoint`:** it resets the caller's `PointPtr`.
10. **`PoseOptimizer::evaluateErrorImpl`:** `unwhitened_errors` uses `emplace_back(double)`, which narrows to float on store.
11. **`jacobian_xyz2f_imu`:** it uses `1/(sqrt(n2)*n2)`.
12. **`ImuProcessor::getRelativeRotationPrior`:** it renormalises the quaternion after every step; kindr only renormalised past a tolerance.

## Open questions

- The real names of three functions: `ImuProcessor::limitMeasurementsSize` (0x18012B090), `Map::allKeyPointsVisible` (0x18012E930), and the inlined quaternion exp/angle helpers.
- `getClosestNKeyframesWithOverlap` computes `q.inverse().normalized() * (50 f − t)`. This would be `kf->T_f_w_.inverseTransform(50 f)` only if Pimax's kindr `inverseRotate` normalises. The upstream kindr does not normalise, and Eigen's `operator*` does not either.
- The `ImuProcessor` layout between +16 and +232, and its ctor (outside the chunk).
- The full `InitializerType` enumerator list. Only kStereo == 0 is proven.
- Whether `Map::addKeyframe` still takes an unused `temporal_map` flag. The binary shows two arguments only.

## Line counts (drafts)
See the summary reply. Files:
- `frontend/imu_processor.{h,cpp}`
- `frontend/initialization.{h,cpp}`
- `frontend/map.{h,cpp}`
- `frontend/pose_optimizer.{h,cpp}`
- `common/frame_jacobians.h`
- `common/c11_external.h`
