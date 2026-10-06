# c09_frame_processor — 0x1800FA6B0 .. 0x180118740

## Summary

Despite the chunk name, this range holds the **tail of the FrameProcessorBase object**
(svo `frame_handler_base.cpp` heritage; the `__FILE__`-carrying functions of
`frame_processor_base.cpp` follow right after at 0x180118740+), plus header-only project code
first instantiated here (VINS `IntegrationBase`, Pimax Sophus `SO3::exp`, Frame/FrameBundle
inline helpers) and a large block of boost::serialization singletons / `load_object_data`
instantiations for the PlatMap/KeyFrame persistence types.

387 functions: 20 project functions (all reconstructed, no stubs), 367 library instantiations
(Eigen JacobiSVD/LU/QR internals, STL containers, boost serialization machinery, CRT).

| file | content |
|---|---|
| `draft/c09_frame_processor/frontend/frame_processor_base.h` | partial FrameProcessorBase declaration with verified member offsets, RunningStats, OutputState |
| `draft/c09_frame_processor/frontend/frame_processor_base.cpp` | addImageBundle, addFrameBundle, getMotionPrior, checkImuMotion, loadPriorPosition, computePoseDifference, computePolygonArea, slerpByTime |
| `draft/c09_frame_processor/frontend/frame_processor_base_imu_init.{h,cpp}` | initializeImu (20 KB), ImageFrame, ImuInitParams |
| `draft/c09_frame_processor/sensor_fusion/integration_base.h` | IntegrationBase (layout + push_back, midPointIntegration, evaluate, hat) |
| `draft/c09_frame_processor/common/sophus/so3ex_base.h` | Sophus::SO3<double>::exp |
| `draft/c09_frame_processor/common/frame_inline_helpers.h` | Frame::imuPos, FrameBundle::get_T_W_B, Frame::numLandmarks + Frame/FrameBundle/Point offsets |
| `draft/c09_frame_processor/loop_closing/serialization_helpers.h` | free boost serialize/load/save for Eigen::Matrix, QuatTransformation, cv::Point2f/3f |

Virtual slots of `FrameProcessor` vtable 0x1803B2A90 / `FrameProcessorBase` vtable 0x1803B2E70
(only slot 6 is in this chunk):

| slot | FrameProcessorBase | FrameProcessor | name |
|---|---|---|---|
| 0 | 0x1800EBFB0 | 0x1800B29B0 | dtor |
| 1 | 0x180127550 | 0x180127550 | setFirstFrames |
| 2 | _purecall | 0x1800B4970 | processFrameBundle |
| 3 | 0x18011B370 | 0x1800B5760 | resetAll |
| 4 | 0x18011B380 | 0x18011B380 | resetBackend ("FrameProcessorBase resetBackend") |
| 5 | 0x180128260 | 0x180128260 | setTrackingQuality? (TODO(verify)) |
| 6 | **0x18010A690** | **0x18010A690** | **getMotionPrior** |
| 7–9 | – | 0x1800B4180, 0x1800B4290, 0x1800B2F80 | FrameProcessor-only virtuals (other chunk) |

## Function table

| address | proposed qualified name | signature | kind | upstream status | summary |
|---|---|---|---|---|---|
| 0x1800FA6B0 | FrameProcessorBase::addFrameBundle | bool(const FrameBundlePtr&, const Eigen::Quaternionf& imu_rotation, bool has_imu_rotation, double imu_rotation_timestamp) | project | modified: IMU-rotation buffer, reset on kFailure/set_start_, backend loadMap with G, motion prior repeated after tracking, consecutive-observation counters, reLocalize/health watchdog, LC correction, VI init call, set_start_ reset, no relocalization/callbacks | SVO addFrameBundle (Pimax) |
| 0x1800FB650 | FrameProcessorBase::addImageBundle | bool(std::vector<cv::Mat>&, const std::vector<int>&, const std::vector<int>&, const uint64_t& ts, const ImuMeasurements&, const int& frame_flag, const Eigen::Quaternionf&, bool, double) | project | modified: CLAHE on dark stereo pairs, Pimax Frame ctor, IMU stationarity, C-API output state, ground plane, trajectory log, LC hand-off | builds FrameBundle, calls addFrameBundle, publishes pose |
| 0x1800FCF10 | Eigen::JacobiSVD<Matrix<float,2,3>>::allocate |  | lib:Eigen JacobiSVD | - | asserts JacobiSVD.h |
| 0x1800FD250 | Eigen::JacobiSVD<MatrixXf>::allocate |  | lib:Eigen JacobiSVD | - |  |
| 0x1800FD6C0 | Eigen::JacobiSVD<Matrix<double,2,3>>::allocate |  | lib:Eigen JacobiSVD | - |  |
| 0x1800FD8E0 | Eigen::aligned_allocator<Vector3d>::allocate |  | lib:Eigen aligned_allocator (24*n) | - |  |
| 0x1800FD950 | std::allocator<_Tree_node<pair<const double,FrameBundlePtr>>>::allocate |  | lib:std (56*n) | - |  |
| 0x1800FD9C0 | std::allocator<_Tree_node<pair<const double,ImageFrame>>>::allocate |  | lib:std (496*n, 32-aligned) | - |  |
| 0x1800FDA30 | std::allocator<T(12 bytes)>::allocate |  | lib:std (12*n) | - |  |
| 0x1800FDAB0 | Eigen JacobiSVD<2x3> matrixV/U copy helper |  | lib:Eigen JacobiSVD | - |  |
| 0x1800FDD60 | std::string::append(const char*, size_t) |  | lib:std | - |  |
| 0x1800FDDF0 | Eigen::internal::partial_lu_impl<double,0,int,15>::blocked_lu wrapper |  | lib:Eigen PartialPivLU<15x15> | - |  |
| 0x1800FDEE0 | computePoseDifference | void(double* trans_diff, double* rot_diff_deg, const Transformation& T_a, const Transformation& T_b) | project | new | float position distance + rotation angle (deg) via trace(Ra^T Rb); used by checkTrackingHealth 0x18011BCA0 |
| 0x1800FE140 | computePolygonArea | double(const Plane&) | project | new | shoelace area of Plane::polygon_ (x,y), |sum|*0.5, 0 if < 3 vertices; used by 0x1800F2070 |
| 0x1800FE320 | FrameProcessorBase::checkImuMotion | bool(const ImuMeasurements&) | project | new | 500-sample gyro window, std-dev*sqrt(0.001) < 8e-5 on all axes => static; true = moving / < 200 samples |
| 0x1800FE8F0 | std::unordered_map<...>::clear |  | lib:std | - |  |
| 0x1800FE9B0 | std::map<...>::clear |  | lib:std | - |  |
| 0x1800FEA20 | std::map<double,Eigen::Quaternionf>::clear |  | lib:std | - | imu_rotation_buffer_.clear() |
| 0x1800FEA90 | std::map<double,FrameBundlePtr>::clear |  | lib:std | - | frame_bundle_map_.clear() |
| 0x1800FEAD0 | std::map<double,ImageFrame>::clear |  | lib:std | - | all_image_frame_.clear() |
| 0x1800FEB10 | std::vector<Plane>::clear |  | lib:std (232-byte elements) | - |  |
| 0x1800FEB70 | std::vector<FrameBundlePtr>::clear |  | lib:std | - | bundle_buffer_.clear() |
| 0x1800FEBA0 | Eigen::JacobiSVD<Matrix<float,2,3>>::compute |  | lib:Eigen JacobiSVD | - |  |
| 0x1800FF740 | Eigen::JacobiSVD<MatrixXf>::compute |  | lib:Eigen JacobiSVD | - |  |
| 0x180100FB0 | Eigen::JacobiSVD<Matrix<double,2,3>>::compute |  | lib:Eigen JacobiSVD | - |  |
| 0x180101BB0 | Eigen::PartialPivLU<Matrix<double,15,15>>::compute |  | lib:Eigen PartialPivLU | - | caller 0x18011D5C0 |
| 0x180101D70 | Eigen JacobiSVD<2x3 float> QR preconditioner / 2x2 Jacobi helper |  | lib:Eigen JacobiSVD | - |  |
| 0x180102720 | Eigen::ColPivHouseholderQR<...>::computeInPlace |  | lib:Eigen ColPivHouseholderQR | - | caller 0x1800CB970 |
| 0x1801035F0 | Eigen JacobiSVD<2x3 double> QR preconditioner / 2x2 Jacobi helper |  | lib:Eigen JacobiSVD | - |  |
| 0x180103FB0 | Eigen::SVDBase::computeU |  | lib:Eigen | - |  |
| 0x180103FD0 | Eigen::SVDBase::computeU |  | lib:Eigen | - |  |
| 0x180103FF0 | Eigen::SVDBase::computeU |  | lib:Eigen | - |  |
| 0x180104010 | Eigen::SVDBase::computeV |  | lib:Eigen | - |  |
| 0x180104030 | Eigen::SVDBase::computeV |  | lib:Eigen | - |  |
| 0x180104050 | Eigen::SVDBase::computeV |  | lib:Eigen | - |  |
| 0x180104070 | Eigen::QuaternionBase<Quaterniond>::conjugate |  | lib:Eigen | - | caller 0x1800F5170 |
| 0x1801040E0 | boost::serialization factory/extended_type_info_typeid assert(false) |  | lib:boost (noreturn) | - |  |
| 0x180104140 | IntegrationBase::midPointIntegration | void(double, const V3&x4, const V3& dp, const Quaterniond& dq, const V3& dv, const V3& ba, const V3& bg, V3&, Quaterniond&, V3&, V3&, V3&, bool) | project | modified: Sophus SO3 exp, Jr-based F, -0.5 factors, block-wise jacobian/covariance update | VINS pre-integration step (17.7 KB) |
| 0x1801086B0 | std::current_exception |  | lib:crt | - |  |
| 0x1801086D0 | std::allocator<T(12 bytes)>::deallocate |  | lib:std | - |  |
| 0x180108720 | std::allocator<T(128 bytes)>::deallocate |  | lib:std (Isometry3d) | - |  |
| 0x180108770 | std::error_category::default_error_condition |  | lib:crt | - |  |
| 0x180108780 | iserializer<*,T>::destroy / class_info / tracking helpers (COMDAT-folded) |  | lib:boost iserializer vtable slot | - |  |
| 0x180108790 | iserializer<*,T>::destroy / class_info / tracking helpers (COMDAT-folded) |  | lib:boost iserializer vtable slot | - |  |
| 0x1801087A0 | iserializer<*,T>::destroy / class_info / tracking helpers (COMDAT-folded) |  | lib:boost iserializer vtable slot | - |  |
| 0x1801087B0 | iserializer<*,T>::destroy / class_info / tracking helpers (COMDAT-folded) |  | lib:boost iserializer vtable slot | - |  |
| 0x1801087C0 | iserializer<*,T>::destroy / class_info / tracking helpers (COMDAT-folded) |  | lib:boost iserializer vtable slot | - |  |
| 0x180108830 | iserializer<*,T>::destroy / class_info / tracking helpers (COMDAT-folded) |  | lib:boost iserializer vtable slot | - |  |
| 0x1801088A0 | iserializer<*,T>::destroy / class_info / tracking helpers (COMDAT-folded) |  | lib:boost iserializer vtable slot | - |  |
| 0x180108910 | iserializer<*,T>::destroy / class_info / tracking helpers (COMDAT-folded) |  | lib:boost iserializer vtable slot | - |  |
| 0x180108940 | iserializer<*,T>::destroy / class_info / tracking helpers (COMDAT-folded) |  | lib:boost iserializer vtable slot | - |  |
| 0x180108970 | iserializer<*,T>::destroy / class_info / tracking helpers (COMDAT-folded) |  | lib:boost iserializer vtable slot | - |  |
| 0x1801089A0 | iserializer<*,T>::destroy / class_info / tracking helpers (COMDAT-folded) |  | lib:boost iserializer vtable slot | - |  |
| 0x1801089D0 | iserializer<*,T>::destroy / class_info / tracking helpers (COMDAT-folded) |  | lib:boost iserializer vtable slot | - |  |
| 0x180108A00 | iserializer<*,T>::destroy / class_info / tracking helpers (COMDAT-folded) |  | lib:boost iserializer vtable slot | - |  |
| 0x180108A30 | iserializer<*,T>::destroy / class_info / tracking helpers (COMDAT-folded) |  | lib:boost iserializer vtable slot | - |  |
| 0x180108A60 | Eigen::Matrix3d::determinant |  | lib:Eigen | - | caller setInitialPose 0x1801277B0 |
| 0x180108AE0 | IntegrationBase::push_back | void(double dt, const V3& acc, const V3& gyr) | project | modified: dt < 1e-4 rejected with LOGW | VINS push_back |
| 0x180108BF0 | std::error_category::equivalent(const error_code&,int) |  | lib:crt | - |  |
| 0x180108C10 | std::error_category::equivalent(int, const error_condition&) |  | lib:crt | - |  |
| 0x180108C50 | std::deque<int>::erase(const_iterator) wrapper |  | lib:std | - | Map::keyframe_ids_ erase in initializeImu |
| 0x180108CA0 | std::vector<bool>::erase(first,last) |  | lib:std | - | caller 0x18011D130 |
| 0x180108E90 | Eigen::HouseholderSequence::essentialVector (float 3x3) |  | lib:Eigen | - |  |
| 0x180108F80 | Eigen::HouseholderSequence::essentialVector (MatrixXf) |  | lib:Eigen | - |  |
| 0x180109090 | Eigen::HouseholderSequence::essentialVector (double 3x3) |  | lib:Eigen | - |  |
| 0x180109180 | IntegrationBase::evaluate | Matrix<double,15,1>(Pi,Qi,Vi,Bai,Bgi,Pj,Qj,Vj,Baj,Bgj,const V3& G) | project | modified: gravity argument, discarded corrected_delta_q.normalized() | VINS residual; caller 0x1800ECFD0 |
| 0x18010A400 | Sophus::SO3<double,0>::exp | SO3(const Vector3d&) | project | modified (Pimax so3ex_base.h): cos before sin, mlog ensure line 303 | project header template |
| 0x18010A650 | boost::archive::detail::interface_iarchive::This adjust |  | lib:boost | - |  |
| 0x18010A670 | boost::archive::detail::interface_iarchive::This adjust |  | lib:boost | - |  |
| 0x18010A690 | FrameProcessorBase::getMotionPrior | void(bool) [vtable slot 6] | project | modified: Eigen-quaternion priors, IMU branch over FrameBundle deque newest->oldest with 1e-6 dt abort, exp inlined (1e-12 thr), size<=2 -> constant velocity | SVO getMotionPrior |
| 0x18010ADA0 | FrameBundle::get_T_W_B | Transformation() const | project | modified: no CHECK | frames_[0]->T_world_imu() (out-of-line inline) |
| 0x18010AE00 | std::future_category (thread-safe static init) |  | lib:crt/concrt | - |  |
| 0x18010AEB0 | singleton<iserializer<binary_iarchive,pimax::totem::KeyFrame>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AEC0 | singleton<iserializer<portable_binary_iarchive,pimax::totem::KeyFrame>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AED0 | singleton<oserializer<binary_oarchive,pimax::totem::KeyFrame>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AEE0 | singleton<oserializer<portable_binary_oarchive,pimax::totem::KeyFrame>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AEF0 | singleton<extended_type_info_typeid<std::pair<uint const,double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AF00 | singleton<extended_type_info_typeid<Eigen::Matrix<double,3,1,0,3,1>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AF10 | singleton<extended_type_info_typeid<cv::Point3_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AF20 | singleton<extended_type_info_typeid<cv::Point_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AF30 | singleton<extended_type_info_typeid<kindr::minimal::QuatTransformationTemplate<double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AF40 | singleton<extended_type_info_typeid<std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AF50 | singleton<extended_type_info_typeid<std::vector<int,std::allocator<int>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AF60 | singleton<extended_type_info_typeid<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AF70 | singleton<extended_type_info_typeid<std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AF80 | singleton<extended_type_info_typeid<std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AF90 | singleton<extended_type_info_typeid<std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>,std::allocator<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AFA0 | singleton<extended_type_info_typeid<std::vector<cv::Mat,std::allocator<cv::Mat>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AFB0 | singleton<extended_type_info_typeid<DBoW2::BowVector>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AFC0 | singleton<extended_type_info_typeid<pimax::totem::KeyFrame>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AFD0 | singleton<extended_type_info_typeid<cv::Mat>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AFE0 | singleton<extended_type_info_typeid<pimax::totem::PlatMap>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010AFF0 | singleton<iserializer<binary_iarchive,std::pair<uint const,double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B000 | singleton<iserializer<binary_iarchive,Eigen::Matrix<double,3,1,0,3,1>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B010 | singleton<iserializer<binary_iarchive,cv::Point3_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B020 | singleton<iserializer<binary_iarchive,cv::Point_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B030 | singleton<iserializer<binary_iarchive,kindr::minimal::QuatTransformationTemplate<double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B040 | singleton<iserializer<binary_iarchive,std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B050 | singleton<iserializer<binary_iarchive,std::vector<int,std::allocator<int>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B060 | singleton<iserializer<binary_iarchive,std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B070 | singleton<iserializer<binary_iarchive,std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B080 | singleton<iserializer<binary_iarchive,std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B090 | singleton<iserializer<binary_iarchive,std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>,std::allocator<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B0A0 | singleton<iserializer<binary_iarchive,std::vector<cv::Mat,std::allocator<cv::Mat>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B0B0 | singleton<iserializer<binary_iarchive,DBoW2::BowVector>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B0C0 | singleton<iserializer<binary_iarchive,cv::Mat>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B0D0 | singleton<iserializer<binary_iarchive,pimax::totem::PlatMap>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B0E0 | singleton<iserializer<portable_binary_iarchive,std::pair<uint const,double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B0F0 | singleton<iserializer<portable_binary_iarchive,Eigen::Matrix<double,3,1,0,3,1>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B100 | singleton<iserializer<portable_binary_iarchive,cv::Point3_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B110 | singleton<iserializer<portable_binary_iarchive,cv::Point_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B120 | singleton<iserializer<portable_binary_iarchive,kindr::minimal::QuatTransformationTemplate<double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B130 | singleton<iserializer<portable_binary_iarchive,std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B140 | singleton<iserializer<portable_binary_iarchive,std::vector<int,std::allocator<int>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B150 | singleton<iserializer<portable_binary_iarchive,std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B160 | singleton<iserializer<portable_binary_iarchive,std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B170 | singleton<iserializer<portable_binary_iarchive,std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B180 | singleton<iserializer<portable_binary_iarchive,std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>,std::allocator<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B190 | singleton<iserializer<portable_binary_iarchive,std::vector<cv::Mat,std::allocator<cv::Mat>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B1A0 | singleton<iserializer<portable_binary_iarchive,DBoW2::BowVector>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B1B0 | singleton<iserializer<portable_binary_iarchive,cv::Mat>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B1C0 | singleton<iserializer<portable_binary_iarchive,pimax::totem::PlatMap>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B1D0 | singleton<oserializer<binary_oarchive,std::pair<uint const,double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B1E0 | singleton<oserializer<binary_oarchive,Eigen::Matrix<double,3,1,0,3,1>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B1F0 | singleton<oserializer<binary_oarchive,cv::Point3_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B200 | singleton<oserializer<binary_oarchive,cv::Point_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B210 | singleton<oserializer<binary_oarchive,kindr::minimal::QuatTransformationTemplate<double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B220 | singleton<oserializer<binary_oarchive,std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B230 | singleton<oserializer<binary_oarchive,std::vector<int,std::allocator<int>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B240 | singleton<oserializer<binary_oarchive,std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B250 | singleton<oserializer<binary_oarchive,std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B260 | singleton<oserializer<binary_oarchive,std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B270 | singleton<oserializer<binary_oarchive,std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>,std::allocator<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B280 | singleton<oserializer<binary_oarchive,std::vector<cv::Mat,std::allocator<cv::Mat>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B290 | singleton<oserializer<binary_oarchive,DBoW2::BowVector>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B2A0 | singleton<oserializer<binary_oarchive,cv::Mat>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B2B0 | singleton<oserializer<binary_oarchive,pimax::totem::PlatMap>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B2C0 | singleton<oserializer<portable_binary_oarchive,std::pair<uint const,double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B2D0 | singleton<oserializer<portable_binary_oarchive,Eigen::Matrix<double,3,1,0,3,1>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B2E0 | singleton<oserializer<portable_binary_oarchive,cv::Point3_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B2F0 | singleton<oserializer<portable_binary_oarchive,cv::Point_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B300 | singleton<oserializer<portable_binary_oarchive,kindr::minimal::QuatTransformationTemplate<double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B310 | singleton<oserializer<portable_binary_oarchive,std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B320 | singleton<oserializer<portable_binary_oarchive,std::vector<int,std::allocator<int>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B330 | singleton<oserializer<portable_binary_oarchive,std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B340 | singleton<oserializer<portable_binary_oarchive,std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B350 | singleton<oserializer<portable_binary_oarchive,std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B360 | singleton<oserializer<portable_binary_oarchive,std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>,std::allocator<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B370 | singleton<oserializer<portable_binary_oarchive,std::vector<cv::Mat,std::allocator<cv::Mat>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B380 | singleton<oserializer<portable_binary_oarchive,DBoW2::BowVector>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B390 | singleton<oserializer<portable_binary_oarchive,cv::Mat>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B3A0 | singleton<oserializer<portable_binary_oarchive,pimax::totem::PlatMap>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B3B0 | singleton<pointer_iserializer<binary_iarchive,pimax::totem::KeyFrame>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B3C0 | singleton<pointer_iserializer<portable_binary_iarchive,pimax::totem::KeyFrame>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B3D0 | singleton<pointer_oserializer<binary_oarchive,pimax::totem::KeyFrame>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B3E0 | singleton<pointer_oserializer<portable_binary_oarchive,pimax::totem::KeyFrame>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B3F0 | boost::serialization::typeid_system::extended_type_info_typeid_0::get_key |  | lib:boost | - |  |
| 0x18010B410 | singleton<extended_type_info_typeid<std::pair<uint const,double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B520 | singleton<extended_type_info_typeid<Eigen::Matrix<double,3,1,0,3,1>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B630 | singleton<extended_type_info_typeid<cv::Point3_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B740 | singleton<extended_type_info_typeid<cv::Point_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B850 | singleton<extended_type_info_typeid<kindr::minimal::QuatTransformationTemplate<double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010B960 | singleton<extended_type_info_typeid<std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010BA70 | singleton<extended_type_info_typeid<std::vector<int,std::allocator<int>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010BB80 | singleton<extended_type_info_typeid<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010BC90 | singleton<extended_type_info_typeid<std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010BDA0 | singleton<extended_type_info_typeid<std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010BEB0 | singleton<extended_type_info_typeid<std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>,std::allocator<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010BFC0 | singleton<extended_type_info_typeid<std::vector<cv::Mat,std::allocator<cv::Mat>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010C0D0 | singleton<extended_type_info_typeid<DBoW2::BowVector>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010C1E0 | singleton<extended_type_info_typeid<pimax::totem::KeyFrame>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010C2F0 | singleton<extended_type_info_typeid<cv::Mat>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010C400 | singleton<extended_type_info_typeid<pimax::totem::PlatMap>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010C510 | singleton<iserializer<binary_iarchive,std::pair<uint const,double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010C5F0 | singleton<iserializer<binary_iarchive,Eigen::Matrix<double,3,1,0,3,1>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010C6D0 | singleton<iserializer<binary_iarchive,cv::Point3_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010C7B0 | singleton<iserializer<binary_iarchive,cv::Point_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010C890 | singleton<iserializer<binary_iarchive,kindr::minimal::QuatTransformationTemplate<double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010C970 | singleton<iserializer<binary_iarchive,std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010CA50 | singleton<iserializer<binary_iarchive,std::vector<int,std::allocator<int>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010CB30 | singleton<iserializer<binary_iarchive,std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010CC10 | singleton<iserializer<binary_iarchive,std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010CCF0 | singleton<iserializer<binary_iarchive,std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010CDD0 | singleton<iserializer<binary_iarchive,std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>,std::allocator<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010CEB0 | singleton<iserializer<binary_iarchive,std::vector<cv::Mat,std::allocator<cv::Mat>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010CF90 | singleton<iserializer<binary_iarchive,DBoW2::BowVector>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010D070 | singleton<iserializer<binary_iarchive,pimax::totem::KeyFrame>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010D150 | singleton<iserializer<binary_iarchive,cv::Mat>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010D230 | singleton<iserializer<binary_iarchive,pimax::totem::PlatMap>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010D310 | singleton<iserializer<portable_binary_iarchive,std::pair<uint const,double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010D3F0 | singleton<iserializer<portable_binary_iarchive,Eigen::Matrix<double,3,1,0,3,1>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010D4D0 | singleton<iserializer<portable_binary_iarchive,cv::Point3_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010D5B0 | singleton<iserializer<portable_binary_iarchive,cv::Point_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010D690 | singleton<iserializer<portable_binary_iarchive,kindr::minimal::QuatTransformationTemplate<double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010D770 | singleton<iserializer<portable_binary_iarchive,std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010D850 | singleton<iserializer<portable_binary_iarchive,std::vector<int,std::allocator<int>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010D930 | singleton<iserializer<portable_binary_iarchive,std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010DA10 | singleton<iserializer<portable_binary_iarchive,std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010DAF0 | singleton<iserializer<portable_binary_iarchive,std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010DBD0 | singleton<iserializer<portable_binary_iarchive,std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>,std::allocator<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010DCB0 | singleton<iserializer<portable_binary_iarchive,std::vector<cv::Mat,std::allocator<cv::Mat>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010DD90 | singleton<iserializer<portable_binary_iarchive,DBoW2::BowVector>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010DE70 | singleton<iserializer<portable_binary_iarchive,pimax::totem::KeyFrame>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010DF50 | singleton<iserializer<portable_binary_iarchive,cv::Mat>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010E030 | singleton<iserializer<portable_binary_iarchive,pimax::totem::PlatMap>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010E110 | singleton<oserializer<binary_oarchive,std::pair<uint const,double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010E1F0 | singleton<oserializer<binary_oarchive,Eigen::Matrix<double,3,1,0,3,1>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010E2D0 | singleton<oserializer<binary_oarchive,cv::Point3_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010E3B0 | singleton<oserializer<binary_oarchive,cv::Point_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010E490 | singleton<oserializer<binary_oarchive,kindr::minimal::QuatTransformationTemplate<double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010E570 | singleton<oserializer<binary_oarchive,std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010E650 | singleton<oserializer<binary_oarchive,std::vector<int,std::allocator<int>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010E730 | singleton<oserializer<binary_oarchive,std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010E810 | singleton<oserializer<binary_oarchive,std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010E8F0 | singleton<oserializer<binary_oarchive,std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010E9D0 | singleton<oserializer<binary_oarchive,std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>,std::allocator<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010EAB0 | singleton<oserializer<binary_oarchive,std::vector<cv::Mat,std::allocator<cv::Mat>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010EB90 | singleton<oserializer<binary_oarchive,DBoW2::BowVector>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010EC70 | singleton<oserializer<binary_oarchive,pimax::totem::KeyFrame>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010ED50 | singleton<oserializer<binary_oarchive,cv::Mat>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010EE30 | singleton<oserializer<binary_oarchive,pimax::totem::PlatMap>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010EF10 | singleton<oserializer<portable_binary_oarchive,std::pair<uint const,double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010EFF0 | singleton<oserializer<portable_binary_oarchive,Eigen::Matrix<double,3,1,0,3,1>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010F0D0 | singleton<oserializer<portable_binary_oarchive,cv::Point3_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010F1B0 | singleton<oserializer<portable_binary_oarchive,cv::Point_<float>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010F290 | singleton<oserializer<portable_binary_oarchive,kindr::minimal::QuatTransformationTemplate<double>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010F370 | singleton<oserializer<portable_binary_oarchive,std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010F450 | singleton<oserializer<portable_binary_oarchive,std::vector<int,std::allocator<int>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010F530 | singleton<oserializer<portable_binary_oarchive,std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010F610 | singleton<oserializer<portable_binary_oarchive,std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010F6F0 | singleton<oserializer<portable_binary_oarchive,std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010F7D0 | singleton<oserializer<portable_binary_oarchive,std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>,std::allocator<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010F8B0 | singleton<oserializer<portable_binary_oarchive,std::vector<cv::Mat,std::allocator<cv::Mat>>>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010F990 | singleton<oserializer<portable_binary_oarchive,DBoW2::BowVector>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010FA70 | singleton<oserializer<portable_binary_oarchive,pimax::totem::KeyFrame>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010FB50 | singleton<oserializer<portable_binary_oarchive,cv::Mat>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010FC30 | singleton<oserializer<portable_binary_oarchive,pimax::totem::PlatMap>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010FD10 | singleton<pointer_iserializer<binary_iarchive,pimax::totem::KeyFrame>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010FE40 | singleton<pointer_iserializer<portable_binary_iarchive,pimax::totem::KeyFrame>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x18010FF70 | singleton<pointer_oserializer<binary_oarchive,pimax::totem::KeyFrame>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x1801100A0 | singleton<pointer_oserializer<portable_binary_oarchive,pimax::totem::KeyFrame>>::get_const_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x1801101D0 | singleton<iserializer<binary_iarchive,pimax::totem::KeyFrame>>::get_mutable_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x180110200 | singleton<iserializer<portable_binary_iarchive,pimax::totem::KeyFrame>>::get_mutable_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x180110230 | singleton<oserializer<binary_oarchive,pimax::totem::KeyFrame>>::get_mutable_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x180110260 | singleton<oserializer<portable_binary_oarchive,pimax::totem::KeyFrame>>::get_mutable_instance(void) |  | lib:boost serialization singleton | - |  |
| 0x180110290 | IntegrationBase::hat | Matrix3d(const Vector3d&) | project | new | skew-symmetric matrix (element-wise); only caller rightJacobian 0x1800F1400 |
| 0x1801102F0 | boost pointer_iserializer<*,KeyFrame>::heap_allocation |  | lib:boost (malloc 0x300 => sizeof(KeyFrame)=768) | - |  |
| 0x180110340 | Eigen::ColPivHouseholderQR::householderQ/solve helper |  | lib:Eigen | - |  |
| 0x1801103A0 | Eigen::ColPivHouseholderQR::householderQ/solve helper |  | lib:Eigen | - |  |
| 0x180110400 | Frame::imuPos | Eigen::Vector3f() const | project | modified: returns float | T_world_imu().getPosition().cast<float>() |
| 0x180110490 | FrameProcessorBase::initializeImu | bool(const Eigen::Quaternionf&, bool, double) | project | new (VINS-Mono style VI initialisation) | 20 KB: image-frame window, visual BA, VisualIMUAlignment, VI refinement, gravity checks, yaw alignment, re-anchoring |
| 0x180115310 | std::vector<std::shared_ptr<T>>::emplace(pos, const&) |  | lib:std | - | caller 0x1800E61D0 |
| 0x180115430 | slerpByTime | Eigen::Quaternionf(const Quaternionf& q0, const Quaternionf& q1, double t0, double t1, double t) | project | new | q0.slerp((t-t0)/(t1-t0), q1); callers initializeImu, setInitialPose |
| 0x1801155B0 | Concurrency::details::_PPLTaskHandle<uchar,...>::invoke |  | lib:concrt | - | std::async task body thunk |
| 0x180115670 | kindr::minimal::isValidRotationMatrix | bool(const Matrix3d&, double) | lib:minkindr | - | caller 0x1800DEA10 |
| 0x1801159C0 | singleton<extended_type_info_typeid<std::pair<uint const,double>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x1801159D0 | singleton<extended_type_info_typeid<Eigen::Matrix<double,3,1,0,3,1>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x1801159E0 | singleton<extended_type_info_typeid<cv::Point3_<float>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x1801159F0 | singleton<extended_type_info_typeid<cv::Point_<float>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115A00 | singleton<extended_type_info_typeid<kindr::minimal::QuatTransformationTemplate<double>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115A10 | singleton<extended_type_info_typeid<std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115A20 | singleton<extended_type_info_typeid<std::vector<int,std::allocator<int>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115A30 | singleton<extended_type_info_typeid<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115A40 | singleton<extended_type_info_typeid<std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115A50 | singleton<extended_type_info_typeid<std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115A60 | singleton<extended_type_info_typeid<std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>,std::allocator<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115A70 | singleton<extended_type_info_typeid<std::vector<cv::Mat,std::allocator<cv::Mat>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115A80 | singleton<extended_type_info_typeid<DBoW2::BowVector>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115A90 | singleton<extended_type_info_typeid<pimax::totem::KeyFrame>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115AA0 | singleton<extended_type_info_typeid<cv::Mat>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115AB0 | singleton<extended_type_info_typeid<pimax::totem::PlatMap>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115AC0 | singleton<iserializer<binary_iarchive,std::pair<uint const,double>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115AD0 | singleton<iserializer<binary_iarchive,Eigen::Matrix<double,3,1,0,3,1>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115AE0 | singleton<iserializer<binary_iarchive,cv::Point3_<float>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115AF0 | singleton<iserializer<binary_iarchive,cv::Point_<float>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115B00 | singleton<iserializer<binary_iarchive,kindr::minimal::QuatTransformationTemplate<double>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115B10 | singleton<iserializer<binary_iarchive,std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115B20 | singleton<iserializer<binary_iarchive,std::vector<int,std::allocator<int>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115B30 | singleton<iserializer<binary_iarchive,std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115B40 | singleton<iserializer<binary_iarchive,std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115B50 | singleton<iserializer<binary_iarchive,std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115B60 | singleton<iserializer<binary_iarchive,std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>,std::allocator<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115B70 | singleton<iserializer<binary_iarchive,std::vector<cv::Mat,std::allocator<cv::Mat>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115B80 | singleton<iserializer<binary_iarchive,DBoW2::BowVector>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115B90 | singleton<iserializer<binary_iarchive,pimax::totem::KeyFrame>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115BA0 | singleton<iserializer<binary_iarchive,cv::Mat>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115BB0 | singleton<iserializer<binary_iarchive,pimax::totem::PlatMap>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115BC0 | singleton<iserializer<portable_binary_iarchive,std::pair<uint const,double>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115BD0 | singleton<iserializer<portable_binary_iarchive,Eigen::Matrix<double,3,1,0,3,1>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115BE0 | singleton<iserializer<portable_binary_iarchive,cv::Point3_<float>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115BF0 | singleton<iserializer<portable_binary_iarchive,cv::Point_<float>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115C00 | singleton<iserializer<portable_binary_iarchive,kindr::minimal::QuatTransformationTemplate<double>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115C10 | singleton<iserializer<portable_binary_iarchive,std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115C20 | singleton<iserializer<portable_binary_iarchive,std::vector<int,std::allocator<int>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115C30 | singleton<iserializer<portable_binary_iarchive,std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115C40 | singleton<iserializer<portable_binary_iarchive,std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115C50 | singleton<iserializer<portable_binary_iarchive,std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115C60 | singleton<iserializer<portable_binary_iarchive,std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>,std::allocator<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115C70 | singleton<iserializer<portable_binary_iarchive,std::vector<cv::Mat,std::allocator<cv::Mat>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115C80 | singleton<iserializer<portable_binary_iarchive,DBoW2::BowVector>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115C90 | singleton<iserializer<portable_binary_iarchive,pimax::totem::KeyFrame>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115CA0 | singleton<iserializer<portable_binary_iarchive,cv::Mat>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115CB0 | singleton<iserializer<portable_binary_iarchive,pimax::totem::PlatMap>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115CC0 | singleton<oserializer<binary_oarchive,std::pair<uint const,double>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115CD0 | singleton<oserializer<binary_oarchive,Eigen::Matrix<double,3,1,0,3,1>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115CE0 | singleton<oserializer<binary_oarchive,cv::Point3_<float>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115CF0 | singleton<oserializer<binary_oarchive,cv::Point_<float>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115D00 | singleton<oserializer<binary_oarchive,kindr::minimal::QuatTransformationTemplate<double>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115D10 | singleton<oserializer<binary_oarchive,std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115D20 | singleton<oserializer<binary_oarchive,std::vector<int,std::allocator<int>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115D30 | singleton<oserializer<binary_oarchive,std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115D40 | singleton<oserializer<binary_oarchive,std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115D50 | singleton<oserializer<binary_oarchive,std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115D60 | singleton<oserializer<binary_oarchive,std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>,std::allocator<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115D70 | singleton<oserializer<binary_oarchive,std::vector<cv::Mat,std::allocator<cv::Mat>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115D80 | singleton<oserializer<binary_oarchive,DBoW2::BowVector>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115D90 | singleton<oserializer<binary_oarchive,pimax::totem::KeyFrame>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115DA0 | singleton<oserializer<binary_oarchive,cv::Mat>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115DB0 | singleton<oserializer<binary_oarchive,pimax::totem::PlatMap>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115DC0 | singleton<oserializer<portable_binary_oarchive,std::pair<uint const,double>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115DD0 | singleton<oserializer<portable_binary_oarchive,Eigen::Matrix<double,3,1,0,3,1>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115DE0 | singleton<oserializer<portable_binary_oarchive,cv::Point3_<float>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115DF0 | singleton<oserializer<portable_binary_oarchive,cv::Point_<float>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115E00 | singleton<oserializer<portable_binary_oarchive,kindr::minimal::QuatTransformationTemplate<double>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115E10 | singleton<oserializer<portable_binary_oarchive,std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115E20 | singleton<oserializer<portable_binary_oarchive,std::vector<int,std::allocator<int>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115E30 | singleton<oserializer<portable_binary_oarchive,std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115E40 | singleton<oserializer<portable_binary_oarchive,std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115E50 | singleton<oserializer<portable_binary_oarchive,std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115E60 | singleton<oserializer<portable_binary_oarchive,std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>,std::allocator<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115E70 | singleton<oserializer<portable_binary_oarchive,std::vector<cv::Mat,std::allocator<cv::Mat>>>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115E80 | singleton<oserializer<portable_binary_oarchive,DBoW2::BowVector>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115E90 | singleton<oserializer<portable_binary_oarchive,pimax::totem::KeyFrame>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115EA0 | singleton<oserializer<portable_binary_oarchive,cv::Mat>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115EB0 | singleton<oserializer<portable_binary_oarchive,pimax::totem::PlatMap>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115EC0 | singleton<pointer_iserializer<binary_iarchive,pimax::totem::KeyFrame>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115ED0 | singleton<pointer_iserializer<portable_binary_iarchive,pimax::totem::KeyFrame>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115EE0 | singleton<pointer_oserializer<binary_oarchive,pimax::totem::KeyFrame>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115EF0 | singleton<pointer_oserializer<portable_binary_oarchive,pimax::totem::KeyFrame>>::is_destroyed(void) |  | lib:boost serialization singleton | - |  |
| 0x180115F00 | FrameProcessorBase::loadPriorPosition | void(Transformation* T_prior, bool* loaded) | project | new | reads 3 floats from <trace_dir>/pimax_prior_position.txt; called by ctor |
| 0x1801161D0 | iserializer<binary_iarchive,std::pair<uint const,double>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x180116280 | iserializer<binary_iarchive,Eigen::Matrix<double,3,1,0,3,1>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - | inlines boost::serialization::load(Eigen::Vector3d) (project free template) |
| 0x180116360 | iserializer<binary_iarchive,cv::Point3_<float>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - | inlines serialize(cv::Point3f) |
| 0x1801163F0 | iserializer<binary_iarchive,cv::Point_<float>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - | inlines serialize(cv::Point2f) |
| 0x180116460 | iserializer<binary_iarchive,kindr::minimal::QuatTransformationTemplate<double>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - | inlines load(kindr QuatTransformation): w,x,y,z + Vector3d |
| 0x1801165C0 | iserializer<binary_iarchive,std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x180116600 | iserializer<binary_iarchive,std::vector<int,std::allocator<int>>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x180116720 | iserializer<binary_iarchive,std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x180116780 | iserializer<binary_iarchive,std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x180116920 | iserializer<binary_iarchive,std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x180116B00 | iserializer<binary_iarchive,std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>,std::allocator<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x180116CA0 | iserializer<binary_iarchive,std::vector<cv::Mat,std::allocator<cv::Mat>>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x180116E40 | iserializer<binary_iarchive,DBoW2::BowVector>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - | calls BowVector serialize 0x1800DB230 |
| 0x180116E90 | iserializer<binary_iarchive,pimax::totem::KeyFrame>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - | calls KeyFrame::serialize 0x1800DB460 |
| 0x180116EE0 | iserializer<binary_iarchive,cv::Mat>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - | calls cv::Mat serialize 0x1800DB6F0 |
| 0x180116F30 | iserializer<binary_iarchive,pimax::totem::PlatMap>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - | calls 0x1800B8DB0(ar, PlatMap+104) (PlatMap::serialize body) |
| 0x180116F80 | iserializer<portable_binary_iarchive,std::pair<uint const,double>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x180117040 | iserializer<portable_binary_iarchive,Eigen::Matrix<double,3,1,0,3,1>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - | inlines load(Eigen::Vector3d) (element-wise) |
| 0x180117180 | iserializer<portable_binary_iarchive,cv::Point3_<float>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - | inlines serialize(cv::Point3f) |
| 0x180117210 | iserializer<portable_binary_iarchive,cv::Point_<float>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - | inlines serialize(cv::Point2f) |
| 0x180117280 | iserializer<portable_binary_iarchive,kindr::minimal::QuatTransformationTemplate<double>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - | inlines load(kindr QuatTransformation) |
| 0x1801173E0 | iserializer<portable_binary_iarchive,std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x180117420 | iserializer<portable_binary_iarchive,std::vector<int,std::allocator<int>>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x180117580 | iserializer<portable_binary_iarchive,std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x1801175D0 | iserializer<portable_binary_iarchive,std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x180117760 | iserializer<portable_binary_iarchive,std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x1801177B0 | iserializer<portable_binary_iarchive,std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>,std::allocator<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>>>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x180117940 | iserializer<portable_binary_iarchive,std::vector<cv::Mat,std::allocator<cv::Mat>>>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x180117AD0 | iserializer<portable_binary_iarchive,DBoW2::BowVector>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x180117B20 | iserializer<portable_binary_iarchive,pimax::totem::KeyFrame>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - | calls KeyFrame::serialize (portable) |
| 0x180117B70 | iserializer<portable_binary_iarchive,cv::Mat>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x180117BC0 | iserializer<portable_binary_iarchive,pimax::totem::PlatMap>::load_object_data(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - | calls 0x1800B9520(ar, PlatMap+104) |
| 0x180117C10 | pointer_iserializer<binary_iarchive,pimax::totem::KeyFrame>::load_object_ptr(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x180117CA0 | pointer_iserializer<portable_binary_iarchive,pimax::totem::KeyFrame>::load_object_ptr(basic_iarchive &,void *,uint) |  | lib:boost iserializer | - |  |
| 0x180117D30 | std::unique_lock<std::mutex>::lock |  | lib:std | - |  |
| 0x180117D80 | std::make_error_code(io_errc) |  | lib:crt | - |  |
| 0x180117DA0 | std::make_error_code(io_errc) |  | lib:crt | - |  |
| 0x180117DC0 | Eigen (R*R^T - I) abs max redux (3x3) |  | lib:Eigen (minkindr isValidRotationMatrix) | - |  |
| 0x180117F50 | Eigen::DenseBase<Matrix<double,15,15>>::maxCoeff |  | lib:Eigen | - | caller 0x1800ECFD0 |
| 0x1801181B0 | RunningStats::mean | double() const | project | new | sum / n (0 if n==0) |
| 0x1801181F0 | std::_Future_error_category::message |  | lib:crt | - |  |
| 0x1801182B0 | std::_Generic_error_category::message |  | lib:crt | - |  |
| 0x180118300 | Eigen::DenseBase<Matrix<double,15,15>>::minCoeff |  | lib:Eigen | - | caller 0x1800ECFD0 |
| 0x180118560 | std::_Future_error_category::name |  | lib:crt | - |  |
| 0x180118570 | std::_Generic_error_category::name |  | lib:crt | - |  |
| 0x180118580 | Frame::numLandmarks | size_t() const | project | modified: counts track_id_vec_ > -1 | inline svo_common helper |
| 0x180118600 | std::basic_filebuf<char>::open |  | lib:std | - |  |

Upstream status details for the project functions:

* **addImageBundle 0x1800FB650** – upstream-modified. Extra: CLAHE (`clahe_->apply`) on each stereo
  pair whose `cv::mean` of both images is < 25.0 (only when 4 images); timestamp check is signed and
  logs with LOGE; Pimax Frame ctor takes (cam, img, ts, `options_+96`, cam index, `exposure[i]`,
  `gain[i]`, `&grid_occupancy_`, `options_+248` u16); `set_T_cam_imu(T_C_B, T_B_C)`;
  `FrameBundle::imu_measurements_` copied, `last_timestamp_sec_` set, IMU stationarity
  (`checkImuMotion`) – while `imu_initial_`, a static device sets the gyro bias of the
  ImuProcessor to the window mean; after `addFrameBundle` it fills the C-API output block
  (`output_mutex_`), trajectory vectors (`trajectory_mutex_`), ground-plane / plane state,
  loop-closing hand-off.
* **addFrameBundle 0x1800FA6B0** – upstream-modified, see table. No `stage_==kPaused` early return,
  no `timer_`, no callbacks, no relocalization block; `getMotionPrior` only when `stage_==kTracking`;
  every new camera pose predicted from `last_frames_->at(0)` (not `at(i)`); the
  `have_motion_prior_` block is executed twice (before and after `processFrameBundle`).
* **getMotionPrior 0x18010A690** – see quirks (IMU branch practically never succeeds if the deque
  is newest-first).
* **IntegrationBase::push_back/evaluate** – VINS with the noted changes;
  **midPointIntegration** – rewritten (see header comment in integration_base.h).

## Types

### FrameProcessorBase (partial, offsets verified; ctor 0x1800DFDF0 in another chunk)

| offset | type | name | evidence |
|---|---|---|---|
| 0 | vptr | – | vtable 0x1803B2E70 |
| 16 | int64 | num_landmarks_last_ (TODO name) | addFrameBundle: `= last_frames_->numLandmarks()` / `= backend+164` |
| 24 | BaseOptions (≈250 B) | options_ | ctor field copy |
| 120 | int64 (opt+96) | options_.frame_pyramid_levels (TODO) | Frame ctor arg 4 |
| 144/152 | double | img_align_prior_lambda_rot / _trans | getMotionPrior |
| 176 | double | poseoptim_prior_lambda | getMotionPrior |
| 184 | bool | load_prior_position (TODO) | ctor → loadPriorPosition |
| 192 | std::string | trace_dir | loadPriorPosition, setPerformanceMonitor |
| 251 | bool | trace_statistics | all SVO_* blocks |
| 264 | double | global_map_lc_timeout_sec_ | recovery timing in addFrameBundle |
| 272 | uint16 | frame_grid_param (TODO) | Frame ctor last arg |
| 280 | CameraBundlePtr | cams_ | getCameraShared/get_T_C_B |
| 296/312/328 | FrameBundlePtr | new_frames_ / last_frames_ / last_last_frames_ | |
| 360 | cv::Ptr<cv::CLAHE> | clahe_ | `apply` = vtable +56 |
| 376 | Vector3d | t_lastimu_newimu_ | |
| 400 | Transformation | unknown (identity in ctor) | – |
| 464 | uint8 | ba_mode_ (TODO) | bundleAdjustment arg, `== 2` |
| 512 | AbstractInitialization* (unique_ptr?) | initializer_ | `reset()` slot 2 |
| 520 | ImuProcessor* | imu_handler_ | +16..+56 calib, +208 acc bias, +232 gyro bias |
| 644/648 | int | lost_count_a_/b_ (TODO) | output status 2/3 |
| 684 | bool | big_position_jump_ | addImageBundle |
| 696 | std::mutex | output_mutex_ | |
| 784 | OutputState | output_ (see below) | |
| 1336 | std::mutex | plane_mutex_ | |
| 1416/1420/1432 | bool / Vector3f / Vector3f | plane_valid_, plane_imu_pos_, plane_center_ | |
| 1448/1528 | std::mutex / bool | mutex_1448_, flag_1528_ (TODO) | |
| 1624 | shared_ptr<LoopClosing> | lc_ | |
| 1648/1656 | bool / Vector3d | saved_gyro_bias_valid_ / saved_gyro_bias_ (TODO) | initializeImu bias choice |
| 1752 | vector<Vector3d> | trajectory_positions_ | |
| 1776/1800 | vector<Isometry3d,aligned> | trajectory_poses_a_/b_ (keyframe / non-kf) | |
| 1824/1896 | condition_variable / mutex | trajectory_cv_ / trajectory_mutex_ | |
| 2784 | deque<ImuMeasurement,aligned> | imu_window_ | checkImuMotion |
| 2824/2848/2872 | RunningStats | gyro_stats_[3] | |
| 2896/2900 | float | 0.001f / gyro_std_threshold_ = 8e-5f | ctor |
| 2912 | Transformation | T_map_world_ | output, plane, LC |
| 2976 | Stage | stage_ | |
| 2980 | bool | set_start_ | set by C API |
| 2984 | shared_ptr<Map> | map_ | |
| 3000 | size_t | num_obs_last_ | |
| 3012 | UpdateResult | update_res_ | "dropout" log |
| 3016 | size_t | frame_counter_ | |
| 3032 | vector<vector<FramePtr>> | overlap_kfs_ | |
| 3080 | vector<bool> | grid_occupancy_ (TODO) | Frame ctor arg |
| 3112 | int | frame_flag_ | addImageBundle arg |
| 3120 | bool | have_rotation_prior_ | |
| 3136/3168 | Eigen::Quaterniond | R_imu_world_ / R_imulast_world_ | Eigen (not minkindr) arithmetic |
| 3200/3216 | bool / Transformation | have_prior_pose_ / T_prior_ | loadPriorPosition, addImageBundle |
| 3280/3288/3312 | bool / Vector3d / Vector3d | have_prior_bias_ / prior_acc_bias_ / prior_gyro_bias_ | |
| 3336/3344 | bool / Transformation | have_motion_prior_ / T_newimu_lastimu_prior_ | |
| 3408 | shared_ptr<CeresBackendInterface> | bundle_adjustment_ | |
| 3441/3448 | bool / double | loss_without_correction_ / last_good_tracking_time_sec_ | |
| 3456 | bool | imu_initial_ (true until VI init succeeded) | ctor = 1 |
| 3464 | map<double,FrameBundlePtr> | frame_bundle_map_ | node 0x38 |
| 3480 | map<double,ImageFrame> | all_image_frame_ | node 0x1F0 |
| 3496 | vector<FrameBundlePtr> | bundle_buffer_ | |
| 3520 | map<double,Quaternionf> | imu_rotation_buffer_ | node 0x40 |
| 3536/3544/3552/3560/3568 | int/size_t/size_t/int/double | keyframe_counter_ 0 / max_image_frames_ 20 / window_size_ 6 / keyframe_step_ 3 / keyframe_min_dist_ 0.02 | ctor |
| 3576 | Vector3f | imu_init_pos_ | |
| 3592 | shared_ptr<Mesher> | mesher_ | ctor |
| 3672 | vector<Plane> (232 B) | planes_ | |
| 3720/3724 | bool / int | ground_plane_enable_ (1) / ground_plane_update_count_ (<500) | |
| 3928/3936/3940 | bool/bool/int | reset_loop_closing_pending_ / reloc_disabled_ / reset_count_ (TODO names) | |
| 3984 | vk::Timer | imu_init_timer_ | start/stop |
| 4008/4016 | double | imu_init_time_sec_ / unknown (-1 in ctor) | |

### OutputState (FrameProcessorBase +784, C-API pose block)
| off (abs) | type | name |
|---|---|---|
| 784 | double | timestamp_sec (ts/1e9) |
| 800 | Isometry3d | T_world_imu (+896 translation also written by loadPriorPosition) |
| 928 | Isometry3d | T_map_world |
| 1056/1080/1104 | Vector3d | speed / gyro_bias / acc_bias (biases zeroed if |acc_bias| > 0.5) |
| 1200 | Vector3d | bundle_vec (FrameBundle +216 floats) TODO |
| 1224 | float | confidence (1.0 tracking) |
| 1228/1229/1230 | uint8 | status (0; 1/2/3 during IMU init), tracking_state (4; 0 if bundle+64), backend_static |

### RunningStats (24 B): `size_t n; double sum; double sum_sq;` – mean 0x1801181B0, add/remove/stddev inlined.

### IntegrationBase (sizeof 10576, make_shared 0x2960) – see table in integration_base.h header.
Pimax prepends 7 doubles {0, 0, G_NORM 9.80667, ACC_N 0.04, ACC_W 0.004, GYR_N 0.008,
GYR_W 0.0008} before VINS `dt` (+56); `step_V` is 16-aligned (+5664, 8 B padding); noise at +7824.

### ImageFrame (448 B) and ImuInitParams (136 B) – see frame_processor_base_imu_init.h.

### ImuMeasurement (32 B): `double timestamp_; Vector3f angular_velocity_ (+8); Vector3f linear_acceleration_ (+20)`;
`ImuMeasurements = std::deque<ImuMeasurement, Eigen::aligned_allocator<>>` (1 element per block).
Evidence: getMotionPrior uses +8 as gyro; initializeImu passes +20 as acc, +8 as gyr to IntegrationBase.

### Frame / FrameBundle / Point offsets – see common/frame_inline_helpers.h.
Notable: `Frame::id_` +16 vs `bundle_id_` +32 (addFrameBundle's per-landmark counters use +32);
FrameBundle sizeof 0x100; Frame sizeof 0x340; KeyFrame sizeof 0x300 (boost heap_allocation).

### Other classes touched (offsets only)
| class | offset | meaning |
|---|---|---|
| CeresBackendInterface | +96 | size_t window size (initializeImu, TODO name) |
| | +160 | bool is_static_ (copied from bundle+228 by loadMap) |
| | +164 | int (copied to FrameProcessorBase+16) |
| | +1241 | bool imu_initial_ |
| LoopClosing | +1568 | std::mutex lc_info_lock_ |
| | +1656..+1680 | deque of corrections (size at +1680; element value at block+16) |
| | +1928/+1929 | recovery bools (setRecoveryMode writes both) |
| Map | +0 | std::map<int,FramePtr> keyframes_ |
| | +128 | std::deque<int> keyframe_ids_ (TODO name) |
| ImuProcessor | +16/+24/+40/+48/+56 | gyro_noise, acc_noise, gyro_rw, acc_rw, gravity_magnitude (TODO names) |
| | +208/+232 | acc_bias_ / gyro_bias_ (Vector3d) |
| NCamera | +0 / +24 / +48 | vector<T_C_B> / vector<T_B_C> (64-B stride) / vector<CameraPtr> |
| Plane (232 B) | +112 Vector3f center_, +152 vector<{int id; cv::Point3f pt}> polygon_, +224 double area_ | |
| ceres_backend::PoseLocalParameterization | 0x18 B, 2 vptrs, bool +16 (set true in initializeImu) | |
| ceres_backend::ReprojectionError | 0xF0 B, SizedCostFunction<2,7,3,7>, bool +48 set true | |
| IMUFactor | 0x50 B, SizedCostFunction<15,7,3,3,3,7,3,3,3,3> | |

## External interfaces (called from c09, defined elsewhere)

| address | inferred signature / meaning |
|---|---|
| 0x18000C120 / 0x18000F500 / 0x18000F6A0 / 0x18000C2C0 | LOGD / LOGI / LOGW / LOGE (logger 0x18046A000) |
| 0x1801B55F0 / 56D0 / 57D0 / 58E0 | vk::PerformanceMonitor::log(name,double) / startTimer / stopTimer / writeToFile |
| 0x180009B80 / 0x180013040 / 0x180014A20 / 0x180009C80 | Transformation operator* / inverse / RotationQuaternion::rotate / RotationQuaternion operator* (normalizationHelper) |
| 0x1800089C0 | RotationQuaternion(const Eigen::Quaterniond&) (CHECK |q|²∈[0.9999,1.0001]) |
| 0x180029DF0 / 0x1800426F0 / 0x1800132A0 / 0x18004F880 | Quaterniond toRotationMatrix / normalized / inverse / product |
| 0x18008A270 / 0x1800932D0 | QuatTransformation::cast<float>() / QuatTransformation<float> * Vector3f |
| 0x1800DEA10 | RotationQuaternion(const RotationMatrix&) with CHECK(isValidRotationMatrix) |
| 0x180024D50 | Frame::T_world_imu() (out-of-line) |
| 0x1801283F0 | Frame::set_T_w_imu(const Transformation&) |
| 0x180096E80 | Frame::setKeyframe() |
| 0x180095180 / 0x180095360 | FrameBundle::numLandmarks() / numTrackedFeatures() |
| 0x180127700 | FrameBundle::setIMUState(vel, gyr_bias, acc_bias) |
| 0x180091DA0 / 0x1800928D0 | Frame ctor (Pimax) / FrameBundle(const std::vector<FramePtr>&) |
| 0x1801B41A0 / 41E0 / 41F0 | NCamera::getCameraShared(i) / get_T_B_C(i) / get_T_C_B(i) |
| 0x1801277B0 | FrameProcessorBase::setInitialPose(bundle, quatf, bool) ("C_imu_world is not orthogonal matrix.") |
| 0x18011AA90 | FrameProcessorBase::reLocalize() ("input reLocalize") |
| 0x18011BCA0 | FrameProcessorBase::checkTrackingHealth() (watchdog, "Reset b_m_lost_r ...") |
| 0x18011B8F0 | resetVisionFrontendCommonWhenSetStart() |
| 0x1800F5170 / 0x1800F2070 | estimateGroundPlane(bundle) / updateGroundPlane(const Frame&) |
| 0x180013480 | CeresBackendInterface::loadMapFromBundleAdjustment(new, last, map, have_motion_prior&, const Vector3d* G, bool) |
| 0x180011700 | CeresBackendInterface::bundleAdjustment(bundle, int frame_flag, uint8 mode) |
| 0x180013680 | CeresBackendInterface::updateFrameBundleState(bundle, int, uint8) ("Could not get speed/bias for frame bundle") TODO name |
| 0x180015870 | CeresBackendInterface::setCorrectionInWorld(T) |
| 0x180015DC0 | CeresBackendInterface::setPerformanceMonitor(trace_dir) |
| 0x18018C3A0 / 0x18018C4E0 | LoopClosing::resetLoopClosing() / resetReLocalize() |
| 0x180186F30 | LoopClosing::computePoseDifference(double*, double*, T_a, T_b) (TODO name) |
| 0x180185E40 | LoopClosing::addFrameBundle(bundle, T_map_world) ("Last thread still running") TODO name |
| 0x18012D680 / 0x18012E9F0 | Map::addKeyframe(const FramePtr&) / Map::removeKeyframe(int id) |
| 0x18011A810 / 0x18011AFE0 / 0x1800E20B0 / 0x1800F1400 | IntegrationBase::propagate / repropagate / ctor / rightJacobian |
| 0x180042410 | Utility::skewSymmetric(Vector3d) |
| 0x1800F3260 / 0x1800DDCF0 | Utility::R2ypr(R, bool radian) / Utility::ypr2R(ypr, bool radian) |
| 0x180157240 | VisualIMUAlignment(params, all_image_frame, Bgs, g, x, ba) |
| 0x180008F50 / 0x1800E1950 | ceres_backend::ReprojectionError ctor / IMUFactor ctor |
| 0x18008ACB0 | depthInFrame(T_f_w float, p_w) (z of transformed point) |
| 0x1800CBC40 | Sophus ensure-failed handler ("mlog ensure failed in function ...") |
| 0x1801B9900/9940/9970/9980/9990/99E0, 0x1801C06B0 | ceres::Problem ctor/dtor/AddParameterBlock(2)/(3)/AddResidualBlock(vector)/SetParameterBlockConstant, ceres::Solve |
| 0x180019230 / 0x18001A0B0 / 0x1800195D0 / 0x18001A2B0 | Solver::Options ctor/dtor, Solver::Summary ctor/dtor |
| 0x1800DB460 / 0x1800B8DB0 / 0x1800DB6F0 / 0x1800DB230 | KeyFrame::serialize, PlatMap serialize body (on PlatMap+104), cv::Mat / BowVector serialize |

Globals: `0x18046A000` logger; `0x18047DDA0` g_permon (vk::PerformanceMonitor*, shared_ptr object);
`0x18047ED98` `Eigen::Vector3d G` (VINS gravity, written by IMUFactor::Evaluate path 0x1800ED3BD,
passed by pointer to loadMapFromBundleAdjustment, read for the gravity checks);
`0x18047DE98`/`0x18047DE9C` function-static `float kStdScale = sqrt(0.001)` + init guard (checkImuMotion);
`0x18047DB48` FrameBundle id counter (FrameBundle ctor, other chunk).

## Constants / defaults (all checked with `rd`)

| where | value |
|---|---|
| addImageBundle CLAHE trigger | `cv::mean(img)[0] < 25.0` on both images of a pair, only if 4 images |
| addImageBundle | position jump `> 0.15` m; acc-bias sanity `|ba| > 0.5`; plane area `> 0.3`; plane height diff `> 3.0f`; LC trigger `trans_diff > 0.01`, `frame_flag_ < 5`; ground plane `numLandmarks() > 10`, `< 500` updates |
| addFrameBundle | IMU-rotation buffer ≤ 10; timestamps `*1e-9` |
| checkImuMotion | window ≤ 500, test needs ≥ 200, `kStdScale = sqrt(0.001)`, threshold `gyro_std_threshold_` (8e-5f) |
| getMotionPrior | IMU branch needs > 2 samples; dt `< 1e-6` aborts; exp small-angle threshold 1e-12, 1/48 |
| IntegrationBase | push_back rejects dt `< 0.0001`; defaults 9.80667 / 0.04 / 0.004 / 0.008 / 0.0008; Jr eps 1e-8 |
| SO3::exp | eps² 1e-20, Taylor 1/48, 1/3840, 1/8, 1/384, ensure 1e-10 |
| initializeImu | min dist 0.02 m or every 3rd frame, bundle tracks ≥ 120, buffer ≤ 50, ≤ 20 image frames, window = max(6, backend+2); landmark depth (0, 10]; HuberLoss(0.5); Ceres DENSE_SCHUR, DOGLEG, 5 iterations, 1 thread, no stdout; sqrt-info `1/(1<<level)`; gravity: horizontal norm > 0.45 rejected, angle to (0,0,9.80667) > 3.5° rejected; prior position used if norm < 100; 3 newest backend bundles made keyframes |
| loadPriorPosition | file `<trace_dir>/pimax_prior_position.txt`, 3 floats |
| computePoseDifference | clamp [-1, 1], `*180.0/M_PI` |

## Quirks / bugs kept

* `getMotionPrior` IMU branch iterates `m_idx` from `size-1` down to 2 and requires
  `ts[m] - ts[m-1] >= 1e-6`; with SVO's newest-first deque this is negative, so the branch logs
  "IMU timestamps need to be strictly increasing." and returns without a prior. The pair (1,0) is
  never used. Size ≤ 2 → constant-velocity branch (only if a prior lambda > 0).
* `IntegrationBase::evaluate`: `corrected_delta_q.normalized();` result discarded (the unnormalised
  quaternion is used).
* `initializeImu` overwrites the IntegrationBase noise parameters *after* construction, so the
  18×18 noise matrix still uses the defaults (0.04/0.008/0.004/0.0008).
* `initializeImu` phase I loop runs exactly once (`i = size-1`) and reads `window_bundles[i-1]`.
* `initializeImu` bundle loop of phase G checks `updated_points.count(point->id())` but inserts with the
  track id; keyframe/last-frame loops check the track id.
* `initializeImu` slerp uses `lower_bound(t)` without checking for `end()`.
* `addImageBundle` takes 4 `at()` copies of the frames (throws for < 4 cameras); a
  `std::unique_lock` on `trajectory_mutex_` is taken and immediately released; `(T_map_world*T_world_imu)`
  translation/rotation are computed and discarded; "feature_track_show" timer started, never stopped.
* `addFrameBundle` copies predicted pose from `last_frames_->at(0)` to every camera; duplicated
  motion-prior block; `new_frames_->at(0)` evaluated for nothing after `reLocalize()`.
* `checkImuMotion` treats "< 200 samples" as moving.
* `loadPriorPosition` also writes the position into `output_.T_world_imu.translation()`.
* `Frame::numLandmarks()` counts valid track ids (not non-null landmarks).
* timestamps: `getTimestampSec()` divides by 1e9, `getMinTimestampSeconds()` multiplies by 1e-9 —
  both forms occur and are kept as in the binary.

## Open questions / TODO(verify)

* Names of many Pimax members (see TODO in the layout table) and of the backend / LC / Map helpers
  listed above; type of `exposure_times`/`gains` (int vector assumed) in addImageBundle.
* Exact Eigen expression spelling in midPointIntegration where only operand order of commutative
  ops differs (e.g. `0.5*(w0+w1)` vs `(w0+w1)/2`) — numerically identical.
* Whether `R_imu_world_`/`R_imulast_world_` are `Eigen::Quaterniond` (strongly suggested: no
  normalization helper, Eigen inverse 0x1800132A0) — affects the shared header.
* Exact form of the Q-block products (with/without `.noalias()`); results identical.
