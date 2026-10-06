# c08_preint_ground: 0x1800E46A0 to 0x1800FA6B0

Drafts are in `draft/c08_preint_ground/`:
- `frontend/frame_processor_base_dtor.cpp`: `FrameProcessorBase::~FrameProcessorBase` (0x1800E46A0).
- `frontend/frame_processor_base_ground.cpp`: the mesh, plane and ground-plane members of FrameProcessorBase (0x1800EC2B0, 0x1800F2070, 0x1800F3430, 0x1800F4190, 0x1800F4BD0, 0x1800F5170, 0x1800F64A0) and the file-static `median` (0x1800F19D0).
- `frontend/frame_processor_base_c08.h`: the c08 view of `FrameProcessorBase`. It lists the members the dtor and the ground code touch, with their offsets.
- `frontend/imu_factor.h`: `IMUFactor` (header-only). Contains `Evaluate` (0x1800ECFD0, 17322 bytes), which is reconstructed in full.
- `frontend/integration_base.h`: the full `IntegrationBase` layout. Defines `rightJacobian` (0x1800F1400) and `skewSymmetric` (0x180110290). The other methods are only declared.
- `frontend/pose_local_parameterization.h`: `PoseLocalParameterization` (Plus 0x1800F1DD0, ComputeJacobian 0x1800ECF10) and `InitGravityLocalParameter` (Plus 0x1800F1A60).
- `frontend/utility.h`: the VINS `Utility` helpers. Defines `R2ypr` (0x1800F3260) and declares the `deltaQ`, `skewSymmetric`, `Qleft` and `Qright` templates, plus `TangentBasis`.
- `plane/plane_types.h`: `Plane` (232 bytes), `PolygonVertex`, the landmark maps, a partial `Mesher`, and the Kimera mesher globals.

Line counts: 1792 lines of draft in total. The ground-plane cpp is 848 lines, `imu_factor.h` 231, `integration_base.h` 159, `pose_local_parameterization.h` 125, `utility.h` 114, `plane_types.h` 123, the FrameProcessorBase header 143, and the dtor 49.

## Scope and object-file boundaries
**There is no object boundary inside this range.** All of it lies in `frontend/frame_processor_base.obj`:
- The FrameProcessorBase ctor (0x1800DFDF0) comes just before the range. The dtor (0x1800E46A0) is the first function in the range.
- `addFrameBundle` ("New Frame Bundle received: %d", 0x1800FA6B0) starts right after the range. c10 places the start of imu_processor.obj at 0x180129D60.
- Everything between them is either FrameProcessorBase code or a COMDAT whose first reference comes from FrameProcessorBase code:
  - The VINS header-only classes `IntegrationBase`, `IMUFactor`, `PoseLocalParameterization` and `InitGravityLocalParameter`. The VINS-like IMU init 0x180110490 creates them.
  - The `std::async`/PPL machinery used by `projectMapInFrame`, including its lambda 0x1800E61D0.
  - Eigen float GEMM kernels.
  - The boost serializer singletons for the PlatMap types. frame_processor_base.cpp includes the loop-closing headers.
  - The `_Ref_count*` control blocks for Frame, FrameBundle, Point, Mesher, IntegrationBase, KeyFrame and Map.

Address order does not follow source order. The ctor, the dtor and the COMDATs of their member types come first. The ground-plane members follow in a cluster (0x1800EC2B0 to 0x1800F64A0), then std::async plumbing.

Expected contents, checked:
- "FrameProcessorBase resetBackend" at 0x1800E46A0 is the **destructor**. The virtual `resetBackend()` (slot 4, 0x18011B380, c10) is statically bound and inlined there.
- The preintegration function 0x1800ECFD0 is `IMUFactor::Evaluate`, not IntegrationBase code. `IntegrationBase::evaluate` is the out-of-line 0x180109180 (c09).
- Ground plane: "ground-plane relative normal candidates ..." is in 0x1800F5170, and "ground-plane height id ..." with five sibling strings is in 0x1800F64A0.

## Cross-chunk naming
- **Shared names.** `updateGroundPlane` (0x1800F2070) and `estimateGroundPlane` (0x1800F5170) use c09's names, and `shouldRemoveKeyframe` (0x1800F4BD0) uses c07's. In both 0x1800F2070 and 0x1800F5170 the binary passes the raw object pointer, so their parameters are `const Frame&` and `const FrameBundle&`. c09 declared `const FrameBundlePtr&` for 0x1800F5170, which is wrong.
- **Renamed function.** 0x1800F64A0 is named `selectGroundPlane()` here, to avoid a clash with c09's name.
- **Names taken from c10.** The members `imu_window_`, `gyr_stat_`, `T_world_correction_`, `T_prior_`, `bundle_adjustment_`, `bundle_adjustment_type_` and `mesher_` use c10's names.
- **c08 names with types.** The ground-plane members below use c08 names. c10 marks these as TODO(type), and the evidence here gives the types:
  - +3608 `LmkPositionMap lmk_points_` = `std::unordered_map<int, Eigen::Vector3f>`. c10 has `plane_map_` as `unordered_map<int,int>`, which is wrong.
  - +3672 `std::vector<Plane> planes_` (c10: `planes_a_`).
  - +3696 `other_planes_` (c10: `planes_b_`).
  - +3720 `mesh_enabled_` (c10: `gp_first_`).
  - +3724 `mesh_update_count_` (c10: `gp_count_`).
  - +3728 `cv::Rect2f mesh_roi_` (c10: `gp_roi_[4]`).
  - +3744 `ground_valid_` (c10: `gp_valid_`).
  - +3748 `ground_confirmations_` (c10: `gp_cnt_a_`).
  - +3752 `ground_miss_count_` (c10: `gp_cnt_b_`).
  - +3760 `Eigen::Vector3d ground_normal_` plus +3784 `double ground_distance_`. c10 has `Vector4d gp_plane_`; that layout works too.
  - +3792 `ground_sigma_` (c10: `gp_thresh_`).
  - +3800 `ground_area_` (c10: `gp_height_`, which is wrong: the field is the plane area).
  - +3808 `std::vector<cv::Point2f> ground_polygon_` (c10: `gp_hist_` as `vector<double>`, which is wrong).
  - +3832 `ground_rel_init_`, +3836 `ground_rel_bundle_id_`, +3840 `ground_rel_normal_b_`, +3864 `ground_rel_normal_w_`, +3888 `ground_rel_sigma_`, +3896 `ground_rel_ids_`.
- **Backend API names from c03.** The constraint uses `GroundPlaneConstraint`, `Estimator::setGroundPlaneConstraint` (0x18002C530), `Estimator::resetGroundPlaneConstraint` (0x1800298A0) and `isFinite` (0x180027A20).

## Function table
| address | proposed qualified name | signature | kind | upstream status | summary |
|---|---|---|---|---|---|
| 1800E46A0 | FrameProcessorBase::~FrameProcessorBase | () | project | modified: sets +1616, inlined resetBackend(), savePriorPosition(T_world_correction_*T_prior_), mesher_.reset() | Destructor; logs "FrameProcessorBase resetBackend\n" (from the inlined resetBackend) and writes pimax_prior_position.txt; then implicit member destruction. |
| 1800E4E10 | - |  | lib: implicit dtor of the +3480 map value (KfRecord: std::set + shared_ptr at +120) | - | Releases the shared_ptr at +128 (ctrl), erases the set tree (0x1800BCAE0), frees the head. |
| 1800E4E90 | - |  | lib: implicit dtor of a {Vector3d; MatrixXd; MatrixXd} struct (v778 in 0x180110490) | - | free() of two Eigen dynamic buffers (+48, +24). |
| 1800E4EE0 | KeyFrame::~KeyFrame (implicit) | () | lib: compiler-generated dtor of pimax::totem::KeyFrame (PlatMap) | - | cv::Mat +32/+256/+440, several vectors, BowVector/FeatureVector maps (0x180092CF0). Called by _Ref_count<KeyFrame>::_Destroy 0x1800F8620. |
| 1800E5230 | Plane::~Plane (implicit) | () | lib: compiler-generated dtor of the 232-byte Plane | - | Destroys +184, +152, +128 vectors, +48 unordered container (0x180077970), +24 vector<12-byte>. |
| 1800E5370 | - |  | lib: implicit dtor of a mutex+maps+string aggregate (owner 0x180108A30, c09) | - | std::string +192, vector<{ptr,..}> +168 (free each), map +128, vector<vector<sp>> +104, map +88, mutex +0. |
| 1800E5520 | cv::Subdiv2D::~Subdiv2D | () | lib: OpenCV inline (implicit) dtor | - | qedges (32-byte) then vtx (16-byte) vectors; used by the unwind path of 0x1800F2070. |
| 1800E55D0 | - |  | lib: std::unordered_map::clear through an owning pointer (unwind helper) | - |  |
| 1800E5690 | - |  | lib: Concurrency::details::_ContextCallback::_Reset thunk | - |  |
| 1800E56A0 | - |  | lib: Concurrency::details::_ExceptionHolder::~_ExceptionHolder | - |  |
| 1800E5720 | - |  | lib: free(*p) deleter | - |  |
| 1800E5740 | - |  | lib: delete of a heap block holding a std::function (_Tidy) | - |  |
| 1800E5790 | - |  | lib: std::_Associated_state body dtor (shared_ptr, mutex, condvar) | - |  |
| 1800E5800 | - |  | lib: std::vector<8-byte> storage free (member at +8) | - |  |
| 1800E5860 | - |  | lib: Concurrency::details::_TaskEventLogger::_LogWorkItemCompleted thunk | - |  |
| 1800E5880 | - |  | lib: Concurrency::details::_Task_impl_base::~_Task_impl_base | - |  |
| 1800E5970 | - |  | lib: std::shared_ptr weak release (_Decwref) + null | - |  |
| 1800E59B0 | - |  | lib: thunk __ExceptionPtrDestroy | - |  |
| 1800E59C0 | - |  | lib: Concurrency::details::_Task_impl<unsigned char> member dtor | - |  |
| 1800E5AA0 | - |  | lib: identity helper (returns 2nd arg; std::_Voidify_iter/placement new) | - |  |
| 1800E5AB0 | - |  | lib: std::deque<32-byte T>::operator= (copy assign) | - | Callers 0x1800FB650, 0x180110490. |
| 1800E5BE0 | - |  | lib: std::shared_ptr<T>::operator=(shared_ptr&&) | - |  |
| 1800E5C60 | - |  | lib: std::vector<4-byte>::operator=(vector&&) | - |  |
| 1800E5D00 | - |  | lib: std::vector<int>::operator=(const vector&) | - | Used by 0x1800F5170 (ground_rel_ids_ = ids). |
| 1800E5D80 | - |  | lib: copy-assign of {8-byte, std::vector<8-byte>} aggregate | - |  |
| 1800E5E20 | - |  | lib: std::map<double, KfRecord(448 bytes)>::operator[] (node 0x1F0) | - | Map at FrameProcessorBase+3480; key is double (c10 says int64 - TODO). |
| 1800E5F30 | - |  | lib: load of an atomic/int (returns *p) | - |  |
| 1800E5F40 | - |  | lib: std::map iterator: value of --end() (rbegin) | - |  |
| 1800E5FB0 | - |  | lib: std::_Tree_unchecked_const_iterator::operator++ | - |  |
| 1800E6040 | - |  | lib: PPL continuation scheduling (_ContextCallback capture/_CallInContext) | - |  |
| 1800E61D0 | FrameProcessorBase::projectMapInFrame()::<lambda_3d1ed943...>::operator() | void() const  [captures this, camera_idx] | project | modified (svo reprojection worker) | Async per-camera reprojection: overlap_kfs_.at(i).insert(begin, last_kf_frames_->at(i)); dedup via unordered_set; assign; sort; reprojectors_.at(i)->reprojectFrames(...). Source text belongs to projectMapInFrame (c10 draft reproduces it; verified against the binary here). |
| 1800E6650 | - |  | lib: Eigen::internal::gebp_kernel<float,float,...> (GEMM micro kernel) | - | Callers in c10 (0x180120960...). |
| 1800E7BE0 | - |  | lib: Eigen product helper for 15x15 * 15x3 (row-major Map) (caller 0x1800CF830) | - |  |
| 1800E7C90 | - |  | lib: Eigen product helper for 15x15 * 15x7 (caller 0x1800CF940) | - |  |
| 1800E7D40 | - |  | lib: Eigen product helper for 15x15 * 15x15 (caller 0x1800CFA50) | - |  |
| 1800E7DF0 | - |  | lib: Eigen::internal::gemm_pack_lhs<float,...> | - | PanelMode assert. |
| 1800E8290 | - |  | lib: Eigen::internal::gemm_pack_rhs<float,...> | - | PanelMode assert. |
| 1800E84A0 | - |  | lib: Eigen::internal::gemm_pack_rhs/lhs<float,...> (second variant) | - | PanelMode assert. |
| 1800E88D0 | - |  | lib: std::basic_ifstream<char>::~basic_ifstream (complete-object body) | - |  |
| 1800E8938 | - |  | lib: std::basic_ifstream<char> vbase deleting-dtor thunk | - |  |
| 1800E8950 | - |  | lib: std::_Associated_state<int> scalar deleting dtor | - |  |
| 1800E89C0 | - |  | lib: Concurrency::details::_CancellationTokenCallback<_lambda_be3e5d9dce35d2c8dbfa8485373731d5_> scalar deleting dtor | - |  |
| 1800E8A40 | - |  | lib: Concurrency::task<uchar>::_InitialTaskHandle<void,_lambda_3184c424febd4fe72fe42e9ad8c42595_,Concurrency::details::_TypeSelectorNoAsync> scalar deleting dtor | - |  |
| 1800E8A80 | - |  | lib: Concurrency::details::_PPLTaskHandle<uchar,Concurrency::task<uchar>::_InitialTaskHandle<void,_lambda_3184c424febd4fe72fe42e9ad8c42595_,Concurrency::details::_TypeSelectorNoAsync>,Concurrency::details: | - |  |
| 1800E8AC0 | - |  | lib: std::_Packaged_state<void (void)> scalar deleting dtor | - |  |
| 1800E8B00 | - |  | lib: std::_Ref_count_obj2<Concurrency::details::_Task_impl<uchar>> scalar deleting dtor | - |  |
| 1800E8B30 | - |  | lib: std::_Ref_count_obj2<Concurrency::details::_ExceptionHolder> scalar deleting dtor | - |  |
| 1800E8B60 | - |  | lib: std::_Ref_count_obj2<pimax::totem::Frame> scalar deleting dtor | - |  |
| 1800E8B90 | - |  | lib: std::_Ref_count_obj2<pimax::totem::FrameBundle> scalar deleting dtor | - |  |
| 1800E8BC0 | - |  | lib: std::_Ref_count_obj2<pimax::totem::IntegrationBase> scalar deleting dtor | - |  |
| 1800E8BF0 | - |  | lib: std::_Ref_count_obj2<pimax::totem::Mesher> scalar deleting dtor | - |  |
| 1800E8C20 | - |  | lib: std::_Ref_count_obj2<pimax::totem::Point> scalar deleting dtor | - |  |
| 1800E8C50 | - |  | lib: std::_Task_async_state<void> scalar deleting dtor | - |  |
| 1800E8D00 | - |  | lib: Concurrency::details::_Task_impl<uchar> scalar deleting dtor | - |  |
| 1800E8DB0 | - |  | lib: std::basic_ifstream<char> deleting dtor (virtual-base this adjust, -176) | - |  |
| 1800E8DF0 | - |  | lib: boost::serialization::extended_type_info_typeid<T> scalar deleting dtor (unnamed vtable) | - |  |
| 1800E8E40 | - |  | lib: boost::serialization::extended_type_info_typeid<T> scalar deleting dtor (unnamed vtable) | - |  |
| 1800E8E90 | - |  | lib: boost::serialization::extended_type_info_typeid<T> scalar deleting dtor (unnamed vtable) | - |  |
| 1800E8EE0 | - |  | lib: boost::serialization::extended_type_info_typeid<T> scalar deleting dtor (unnamed vtable) | - |  |
| 1800E8F30 | - |  | lib: boost::serialization::extended_type_info_typeid<T> scalar deleting dtor (unnamed vtable) | - |  |
| 1800E8F80 | - |  | lib: boost::serialization::extended_type_info_typeid<T> scalar deleting dtor (unnamed vtable) | - |  |
| 1800E8FD0 | - |  | lib: boost::serialization::extended_type_info_typeid<T> scalar deleting dtor (unnamed vtable) | - |  |
| 1800E9020 | - |  | lib: boost::serialization::extended_type_info_typeid<T> scalar deleting dtor (unnamed vtable) | - |  |
| 1800E9070 | - |  | lib: boost::serialization::extended_type_info_typeid<T> scalar deleting dtor (unnamed vtable) | - |  |
| 1800E90C0 | - |  | lib: boost::serialization::extended_type_info_typeid<T> scalar deleting dtor (unnamed vtable) | - |  |
| 1800E9110 | - |  | lib: boost::serialization::extended_type_info_typeid<T> scalar deleting dtor (unnamed vtable) | - |  |
| 1800E9160 | - |  | lib: boost::serialization::extended_type_info_typeid<T> scalar deleting dtor (unnamed vtable) | - |  |
| 1800E91B0 | - |  | lib: boost::serialization::extended_type_info_typeid<T> scalar deleting dtor (unnamed vtable) | - |  |
| 1800E9200 | - |  | lib: boost::serialization::extended_type_info_typeid<T> scalar deleting dtor (unnamed vtable) | - |  |
| 1800E9250 | - |  | lib: boost::serialization::extended_type_info_typeid<T> scalar deleting dtor (unnamed vtable) | - |  |
| 1800E92A0 | - |  | lib: boost::serialization::extended_type_info_typeid<T> scalar deleting dtor (unnamed vtable) | - |  |
| 1800E92F0 | - |  | lib: boost::archive::detail::iserializer<boost::archive::binary_iarchive,std::pair<uint const,double>> scalar deleting dtor | - |  |
| 1800E9330 | - |  | lib: boost::archive::detail::iserializer<boost::archive::binary_iarchive,Eigen::Matrix<double,3,1,0,3,1>> scalar deleting dtor | - |  |
| 1800E9370 | - |  | lib: boost::archive::detail::iserializer<boost::archive::binary_iarchive,cv::Point3_<float>> scalar deleting dtor | - |  |
| 1800E93B0 | - |  | lib: boost::archive::detail::iserializer<boost::archive::binary_iarchive,cv::Point_<float>> scalar deleting dtor | - |  |
| 1800E93F0 | - |  | lib: boost::archive::detail::iserializer<boost::archive::binary_iarchive,kindr::minimal::QuatTransformationTemplate<double>> scalar deleting dtor | - |  |
| 1800E9430 | - |  | lib: boost::archive::detail::iserializer<boost::archive::binary_iarchive,std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>> scalar deleting dtor | - |  |
| 1800E9470 | - |  | lib: boost::archive::detail::iserializer<boost::archive::binary_iarchive,std::vector<int,std::allocator<int>>> scalar deleting dtor | - |  |
| 1800E94B0 | - |  | lib: boost::archive::detail::iserializer<boost::archive::binary_iarchive,std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>> scalar deleting dtor | - |  |
| 1800E94F0 | - |  | lib: boost::archive::detail::iserializer<boost::archive::binary_iarchive,std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>> scalar deleting dtor | - |  |
| 1800E9530 | - |  | lib: boost::archive::detail::iserializer<boost::archive::binary_iarchive,std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>> scalar deleting dtor | - |  |
| 1800E9570 | - |  | lib: boost::archive::detail::iserializer<boost::archive::binary_iarchive,std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyF scalar deleting dtor | - |  |
| 1800E95B0 | - |  | lib: boost::archive::detail::iserializer<boost::archive::binary_iarchive,std::vector<cv::Mat,std::allocator<cv::Mat>>> scalar deleting dtor | - |  |
| 1800E95F0 | - |  | lib: boost::archive::detail::iserializer<boost::archive::binary_iarchive,DBoW2::BowVector> scalar deleting dtor | - |  |
| 1800E9630 | - |  | lib: boost::archive::detail::iserializer<boost::archive::binary_iarchive,pimax::totem::KeyFrame> scalar deleting dtor | - |  |
| 1800E9670 | - |  | lib: boost::archive::detail::iserializer<boost::archive::binary_iarchive,cv::Mat> scalar deleting dtor | - |  |
| 1800E96B0 | - |  | lib: boost::archive::detail::iserializer<boost::archive::binary_iarchive,pimax::totem::PlatMap> scalar deleting dtor | - |  |
| 1800E96F0 | - |  | lib: boost::archive::detail::iserializer<portable_binary_iarchive,std::pair<uint const,double>> scalar deleting dtor | - |  |
| 1800E9730 | - |  | lib: boost::archive::detail::iserializer<portable_binary_iarchive,Eigen::Matrix<double,3,1,0,3,1>> scalar deleting dtor | - |  |
| 1800E9770 | - |  | lib: boost::archive::detail::iserializer<portable_binary_iarchive,cv::Point3_<float>> scalar deleting dtor | - |  |
| 1800E97B0 | - |  | lib: boost::archive::detail::iserializer<portable_binary_iarchive,cv::Point_<float>> scalar deleting dtor | - |  |
| 1800E97F0 | - |  | lib: boost::archive::detail::iserializer<portable_binary_iarchive,kindr::minimal::QuatTransformationTemplate<double>> scalar deleting dtor | - |  |
| 1800E9830 | - |  | lib: boost::archive::detail::iserializer<portable_binary_iarchive,std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>>> scalar deleting dtor | - |  |
| 1800E9870 | - |  | lib: boost::archive::detail::iserializer<portable_binary_iarchive,std::vector<int,std::allocator<int>>> scalar deleting dtor | - |  |
| 1800E98B0 | - |  | lib: boost::archive::detail::iserializer<portable_binary_iarchive,std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>> scalar deleting dtor | - |  |
| 1800E98F0 | - |  | lib: boost::archive::detail::iserializer<portable_binary_iarchive,std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>> scalar deleting dtor | - |  |
| 1800E9930 | - |  | lib: boost::archive::detail::iserializer<portable_binary_iarchive,std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>> scalar deleting dtor | - |  |
| 1800E9970 | - |  | lib: boost::archive::detail::iserializer<portable_binary_iarchive,std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *> scalar deleting dtor | - |  |
| 1800E99B0 | - |  | lib: boost::archive::detail::iserializer<portable_binary_iarchive,std::vector<cv::Mat,std::allocator<cv::Mat>>> scalar deleting dtor | - |  |
| 1800E99F0 | - |  | lib: boost::archive::detail::iserializer<portable_binary_iarchive,DBoW2::BowVector> scalar deleting dtor | - |  |
| 1800E9A30 | - |  | lib: boost::archive::detail::iserializer<portable_binary_iarchive,pimax::totem::KeyFrame> scalar deleting dtor | - |  |
| 1800E9A70 | - |  | lib: boost::archive::detail::iserializer<portable_binary_iarchive,cv::Mat> scalar deleting dtor | - |  |
| 1800E9AB0 | - |  | lib: boost::archive::detail::iserializer<portable_binary_iarchive,pimax::totem::PlatMap> scalar deleting dtor | - |  |
| 1800E9AF0 | - |  | lib: boost::archive::detail::oserializer<boost::archive::binary_oarchive,std::pair<uint const,double>> scalar deleting dtor | - |  |
| 1800E9B30 | - |  | lib: boost::archive::detail::oserializer<boost::archive::binary_oarchive,Eigen::Matrix<double,3,1,0,3,1>> scalar deleting dtor | - |  |
| 1800E9B70 | - |  | lib: boost::archive::detail::oserializer<boost::archive::binary_oarchive,cv::Point3_<float>> scalar deleting dtor | - |  |
| 1800E9BB0 | - |  | lib: boost::archive::detail::oserializer<boost::archive::binary_oarchive,cv::Point_<float>> scalar deleting dtor | - |  |
| 1800E9BF0 | - |  | lib: boost::archive::detail::oserializer<boost::archive::binary_oarchive,kindr::minimal::QuatTransformationTemplate<double>> scalar deleting dtor | - |  |
| 1800E9C30 | - |  | lib: boost::archive::detail::oserializer<boost::archive::binary_oarchive,std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>> scalar deleting dtor | - |  |
| 1800E9C70 | - |  | lib: boost::archive::detail::oserializer<boost::archive::binary_oarchive,std::vector<int,std::allocator<int>>> scalar deleting dtor | - |  |
| 1800E9CB0 | - |  | lib: boost::archive::detail::oserializer<boost::archive::binary_oarchive,std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>> scalar deleting dtor | - |  |
| 1800E9CF0 | - |  | lib: boost::archive::detail::oserializer<boost::archive::binary_oarchive,std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>> scalar deleting dtor | - |  |
| 1800E9D30 | - |  | lib: boost::archive::detail::oserializer<boost::archive::binary_oarchive,std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>> scalar deleting dtor | - |  |
| 1800E9D70 | - |  | lib: boost::archive::detail::oserializer<boost::archive::binary_oarchive,std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyF scalar deleting dtor | - |  |
| 1800E9DB0 | - |  | lib: boost::archive::detail::oserializer<boost::archive::binary_oarchive,std::vector<cv::Mat,std::allocator<cv::Mat>>> scalar deleting dtor | - |  |
| 1800E9DF0 | - |  | lib: boost::archive::detail::oserializer<boost::archive::binary_oarchive,DBoW2::BowVector> scalar deleting dtor | - |  |
| 1800E9E30 | - |  | lib: boost::archive::detail::oserializer<boost::archive::binary_oarchive,pimax::totem::KeyFrame> scalar deleting dtor | - |  |
| 1800E9E70 | - |  | lib: boost::archive::detail::oserializer<boost::archive::binary_oarchive,cv::Mat> scalar deleting dtor | - |  |
| 1800E9EB0 | - |  | lib: boost::archive::detail::oserializer<boost::archive::binary_oarchive,pimax::totem::PlatMap> scalar deleting dtor | - |  |
| 1800E9EF0 | - |  | lib: boost::archive::detail::oserializer<portable_binary_oarchive,std::pair<uint const,double>> scalar deleting dtor | - |  |
| 1800E9F30 | - |  | lib: boost::archive::detail::oserializer<portable_binary_oarchive,Eigen::Matrix<double,3,1,0,3,1>> scalar deleting dtor | - |  |
| 1800E9F70 | - |  | lib: boost::archive::detail::oserializer<portable_binary_oarchive,cv::Point3_<float>> scalar deleting dtor | - |  |
| 1800E9FB0 | - |  | lib: boost::archive::detail::oserializer<portable_binary_oarchive,cv::Point_<float>> scalar deleting dtor | - |  |
| 1800E9FF0 | - |  | lib: boost::archive::detail::oserializer<portable_binary_oarchive,kindr::minimal::QuatTransformationTemplate<double>> scalar deleting dtor | - |  |
| 1800EA030 | - |  | lib: boost::archive::detail::oserializer<portable_binary_oarchive,std::map<uint,double,std::less<uint>,std::allocator<std::pair<uint const,double>>>> scalar deleting dtor | - |  |
| 1800EA070 | - |  | lib: boost::archive::detail::oserializer<portable_binary_oarchive,std::vector<int,std::allocator<int>>> scalar deleting dtor | - |  |
| 1800EA0B0 | - |  | lib: boost::archive::detail::oserializer<portable_binary_oarchive,std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *>>> scalar deleting dtor | - |  |
| 1800EA0F0 | - |  | lib: boost::archive::detail::oserializer<portable_binary_oarchive,std::vector<cv::Point3_<float>,std::allocator<cv::Point3_<float>>>> scalar deleting dtor | - |  |
| 1800EA130 | - |  | lib: boost::archive::detail::oserializer<portable_binary_oarchive,std::vector<cv::Point_<float>,std::allocator<cv::Point_<float>>>> scalar deleting dtor | - |  |
| 1800EA170 | - |  | lib: boost::archive::detail::oserializer<portable_binary_oarchive,std::vector<std::vector<pimax::totem::KeyFrame *,std::allocator<pimax::totem::KeyFrame *> scalar deleting dtor | - |  |
| 1800EA1B0 | - |  | lib: boost::archive::detail::oserializer<portable_binary_oarchive,std::vector<cv::Mat,std::allocator<cv::Mat>>> scalar deleting dtor | - |  |
| 1800EA1F0 | - |  | lib: boost::archive::detail::oserializer<portable_binary_oarchive,DBoW2::BowVector> scalar deleting dtor | - |  |
| 1800EA230 | - |  | lib: boost::archive::detail::oserializer<portable_binary_oarchive,pimax::totem::KeyFrame> scalar deleting dtor | - |  |
| 1800EA270 | - |  | lib: boost::archive::detail::oserializer<portable_binary_oarchive,cv::Mat> scalar deleting dtor | - |  |
| 1800EA2B0 | - |  | lib: boost::archive::detail::oserializer<portable_binary_oarchive,pimax::totem::PlatMap> scalar deleting dtor | - |  |
| 1800EA2F0 | - |  | lib: boost::archive::detail::pointer_iserializer<boost::archive::binary_iarchive,pimax::totem::KeyFrame> scalar deleting dtor | - |  |
| 1800EA340 | - |  | lib: boost::archive::detail::pointer_iserializer<portable_binary_iarchive,pimax::totem::KeyFrame> scalar deleting dtor | - |  |
| 1800EA390 | - |  | lib: boost::archive::detail::pointer_oserializer<boost::archive::binary_oarchive,pimax::totem::KeyFrame> scalar deleting dtor | - |  |
| 1800EA3E0 | - |  | lib: boost::archive::detail::pointer_oserializer<portable_binary_oarchive,pimax::totem::KeyFrame> scalar deleting dtor | - |  |
| 1800EA430 | - |  | lib: boost singleton_wrapper<extended_type_info_typeid<T>> scalar deleting dtor (sets m_is_destroyed) | - |  |
| 1800EA490 | - |  | lib: boost singleton_wrapper<extended_type_info_typeid<T>> scalar deleting dtor (sets m_is_destroyed) | - |  |
| 1800EA4F0 | - |  | lib: boost singleton_wrapper<extended_type_info_typeid<T>> scalar deleting dtor (sets m_is_destroyed) | - |  |
| 1800EA550 | - |  | lib: boost singleton_wrapper<extended_type_info_typeid<T>> scalar deleting dtor (sets m_is_destroyed) | - |  |
| 1800EA5B0 | - |  | lib: boost singleton_wrapper<extended_type_info_typeid<T>> scalar deleting dtor (sets m_is_destroyed) | - |  |
| 1800EA610 | - |  | lib: boost singleton_wrapper<extended_type_info_typeid<T>> scalar deleting dtor (sets m_is_destroyed) | - |  |
| 1800EA670 | - |  | lib: boost singleton_wrapper<extended_type_info_typeid<T>> scalar deleting dtor (sets m_is_destroyed) | - |  |
| 1800EA6D0 | - |  | lib: boost singleton_wrapper<extended_type_info_typeid<T>> scalar deleting dtor (sets m_is_destroyed) | - |  |
| 1800EA730 | - |  | lib: boost singleton_wrapper<extended_type_info_typeid<T>> scalar deleting dtor (sets m_is_destroyed) | - |  |
| 1800EA790 | - |  | lib: boost singleton_wrapper<extended_type_info_typeid<T>> scalar deleting dtor (sets m_is_destroyed) | - |  |
| 1800EA7F0 | - |  | lib: boost singleton_wrapper<extended_type_info_typeid<T>> scalar deleting dtor (sets m_is_destroyed) | - |  |
| 1800EA850 | - |  | lib: boost singleton_wrapper<extended_type_info_typeid<T>> scalar deleting dtor (sets m_is_destroyed) | - |  |
| 1800EA8B0 | - |  | lib: boost singleton_wrapper<extended_type_info_typeid<T>> scalar deleting dtor (sets m_is_destroyed) | - |  |
| 1800EA910 | - |  | lib: boost singleton_wrapper<extended_type_info_typeid<T>> scalar deleting dtor (sets m_is_destroyed) | - |  |
| 1800EA970 | - |  | lib: boost singleton_wrapper<extended_type_info_typeid<T>> scalar deleting dtor (sets m_is_destroyed) | - |  |
| 1800EA9D0 | - |  | lib: boost singleton_wrapper<extended_type_info_typeid<T>> scalar deleting dtor (sets m_is_destroyed) | - |  |
| 1800EAA30 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<boost::archive::binary_iarchive,std::pair<uint const,double>>> scalar deleting dtor | - |  |
| 1800EAA80 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<boost::archive::binary_iarchive,Eigen::Matrix<double,3,1,0,3,1>>> scalar deleting dtor | - |  |
| 1800EAAD0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<boost::archive::binary_iarchive,cv::Point3_<float>>> scalar deleting dtor | - |  |
| 1800EAB20 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<boost::archive::binary_iarchive,cv::Point_<float>>> scalar deleting dtor | - |  |
| 1800EAB70 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<boost::archive::binary_iarchive,kindr::minimal::QuatTransformation scalar deleting dtor | - |  |
| 1800EABC0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<boost::archive::binary_iarchive,std::map<uint,double,std::less<uin scalar deleting dtor | - |  |
| 1800EAC10 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<boost::archive::binary_iarchive,std::vector<int,std::allocator<int scalar deleting dtor | - |  |
| 1800EAC60 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<boost::archive::binary_iarchive,std::vector<pimax::totem::KeyFrame scalar deleting dtor | - |  |
| 1800EACB0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<boost::archive::binary_iarchive,std::vector<cv::Point3_<float>,std scalar deleting dtor | - |  |
| 1800EAD00 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<boost::archive::binary_iarchive,std::vector<cv::Point_<float>,std: scalar deleting dtor | - |  |
| 1800EAD50 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<boost::archive::binary_iarchive,std::vector<std::vector<pimax::tot scalar deleting dtor | - |  |
| 1800EADA0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<boost::archive::binary_iarchive,std::vector<cv::Mat,std::allocator scalar deleting dtor | - |  |
| 1800EADF0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<boost::archive::binary_iarchive,DBoW2::BowVector>> scalar deleting dtor | - |  |
| 1800EAE40 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<boost::archive::binary_iarchive,pimax::totem::KeyFrame>> scalar deleting dtor | - |  |
| 1800EAE90 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<boost::archive::binary_iarchive,cv::Mat>> scalar deleting dtor | - |  |
| 1800EAEE0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<boost::archive::binary_iarchive,pimax::totem::PlatMap>> scalar deleting dtor | - |  |
| 1800EAF30 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<portable_binary_iarchive,std::pair<uint const,double>>> scalar deleting dtor | - |  |
| 1800EAF80 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<portable_binary_iarchive,Eigen::Matrix<double,3,1,0,3,1>>> scalar deleting dtor | - |  |
| 1800EAFD0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<portable_binary_iarchive,cv::Point3_<float>>> scalar deleting dtor | - |  |
| 1800EB020 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<portable_binary_iarchive,cv::Point_<float>>> scalar deleting dtor | - |  |
| 1800EB070 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<portable_binary_iarchive,kindr::minimal::QuatTransformationTemplat scalar deleting dtor | - |  |
| 1800EB0C0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<portable_binary_iarchive,std::map<uint,double,std::less<uint>,std: scalar deleting dtor | - |  |
| 1800EB110 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<portable_binary_iarchive,std::vector<int,std::allocator<int>>>> scalar deleting dtor | - |  |
| 1800EB160 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<portable_binary_iarchive,std::vector<pimax::totem::KeyFrame *,std: scalar deleting dtor | - |  |
| 1800EB1B0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<portable_binary_iarchive,std::vector<cv::Point3_<float>,std::alloc scalar deleting dtor | - |  |
| 1800EB200 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<portable_binary_iarchive,std::vector<cv::Point_<float>,std::alloca scalar deleting dtor | - |  |
| 1800EB250 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<portable_binary_iarchive,std::vector<std::vector<pimax::totem::Key scalar deleting dtor | - |  |
| 1800EB2A0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<portable_binary_iarchive,std::vector<cv::Mat,std::allocator<cv::Ma scalar deleting dtor | - |  |
| 1800EB2F0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<portable_binary_iarchive,DBoW2::BowVector>> scalar deleting dtor | - |  |
| 1800EB340 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<portable_binary_iarchive,pimax::totem::KeyFrame>> scalar deleting dtor | - |  |
| 1800EB390 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<portable_binary_iarchive,cv::Mat>> scalar deleting dtor | - |  |
| 1800EB3E0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::iserializer<portable_binary_iarchive,pimax::totem::PlatMap>> scalar deleting dtor | - |  |
| 1800EB430 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<boost::archive::binary_oarchive,std::pair<uint const,double>>> scalar deleting dtor | - |  |
| 1800EB480 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<boost::archive::binary_oarchive,Eigen::Matrix<double,3,1,0,3,1>>> scalar deleting dtor | - |  |
| 1800EB4D0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<boost::archive::binary_oarchive,cv::Point3_<float>>> scalar deleting dtor | - |  |
| 1800EB520 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<boost::archive::binary_oarchive,cv::Point_<float>>> scalar deleting dtor | - |  |
| 1800EB570 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<boost::archive::binary_oarchive,kindr::minimal::QuatTransformation scalar deleting dtor | - |  |
| 1800EB5C0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<boost::archive::binary_oarchive,std::map<uint,double,std::less<uin scalar deleting dtor | - |  |
| 1800EB610 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<boost::archive::binary_oarchive,std::vector<int,std::allocator<int scalar deleting dtor | - |  |
| 1800EB660 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<boost::archive::binary_oarchive,std::vector<pimax::totem::KeyFrame scalar deleting dtor | - |  |
| 1800EB6B0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<boost::archive::binary_oarchive,std::vector<cv::Point3_<float>,std scalar deleting dtor | - |  |
| 1800EB700 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<boost::archive::binary_oarchive,std::vector<cv::Point_<float>,std: scalar deleting dtor | - |  |
| 1800EB750 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<boost::archive::binary_oarchive,std::vector<std::vector<pimax::tot scalar deleting dtor | - |  |
| 1800EB7A0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<boost::archive::binary_oarchive,std::vector<cv::Mat,std::allocator scalar deleting dtor | - |  |
| 1800EB7F0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<boost::archive::binary_oarchive,DBoW2::BowVector>> scalar deleting dtor | - |  |
| 1800EB840 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<boost::archive::binary_oarchive,pimax::totem::KeyFrame>> scalar deleting dtor | - |  |
| 1800EB890 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<boost::archive::binary_oarchive,cv::Mat>> scalar deleting dtor | - |  |
| 1800EB8E0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<boost::archive::binary_oarchive,pimax::totem::PlatMap>> scalar deleting dtor | - |  |
| 1800EB930 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<portable_binary_oarchive,std::pair<uint const,double>>> scalar deleting dtor | - |  |
| 1800EB980 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<portable_binary_oarchive,Eigen::Matrix<double,3,1,0,3,1>>> scalar deleting dtor | - |  |
| 1800EB9D0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<portable_binary_oarchive,cv::Point3_<float>>> scalar deleting dtor | - |  |
| 1800EBA20 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<portable_binary_oarchive,cv::Point_<float>>> scalar deleting dtor | - |  |
| 1800EBA70 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<portable_binary_oarchive,kindr::minimal::QuatTransformationTemplat scalar deleting dtor | - |  |
| 1800EBAC0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<portable_binary_oarchive,std::map<uint,double,std::less<uint>,std: scalar deleting dtor | - |  |
| 1800EBB10 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<portable_binary_oarchive,std::vector<int,std::allocator<int>>>> scalar deleting dtor | - |  |
| 1800EBB60 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<portable_binary_oarchive,std::vector<pimax::totem::KeyFrame *,std: scalar deleting dtor | - |  |
| 1800EBBB0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<portable_binary_oarchive,std::vector<cv::Point3_<float>,std::alloc scalar deleting dtor | - |  |
| 1800EBC00 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<portable_binary_oarchive,std::vector<cv::Point_<float>,std::alloca scalar deleting dtor | - |  |
| 1800EBC50 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<portable_binary_oarchive,std::vector<std::vector<pimax::totem::Key scalar deleting dtor | - |  |
| 1800EBCA0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<portable_binary_oarchive,std::vector<cv::Mat,std::allocator<cv::Ma scalar deleting dtor | - |  |
| 1800EBCF0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<portable_binary_oarchive,DBoW2::BowVector>> scalar deleting dtor | - |  |
| 1800EBD40 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<portable_binary_oarchive,pimax::totem::KeyFrame>> scalar deleting dtor | - |  |
| 1800EBD90 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<portable_binary_oarchive,cv::Mat>> scalar deleting dtor | - |  |
| 1800EBDE0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::oserializer<portable_binary_oarchive,pimax::totem::PlatMap>> scalar deleting dtor | - |  |
| 1800EBE30 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::pointer_iserializer<boost::archive::binary_iarchive,pimax::totem::KeyFrame>> scalar deleting dtor | - |  |
| 1800EBE90 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::pointer_iserializer<portable_binary_iarchive,pimax::totem::KeyFrame>> scalar deleting dtor | - |  |
| 1800EBEF0 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::pointer_oserializer<boost::archive::binary_oarchive,pimax::totem::KeyFrame>> scalar deleting dtor | - |  |
| 1800EBF50 | - |  | lib: boost::serialization::detail::singleton_wrapper<boost::archive::detail::pointer_oserializer<portable_binary_oarchive,pimax::totem::KeyFrame>> scalar deleting dtor | - |  |
| 1800EBFB0 | FrameProcessorBase::`scalar deleting destructor' | (unsigned) | project (compiler-generated) | - | Calls 0x1800E46A0, then free() (EIGEN_MAKE_ALIGNED_OPERATOR_NEW) unless flag 4. |
| 1800EC000 | IMUFactor::`scalar deleting destructor' | (unsigned) | project (compiler-generated) | - | Releases pre_integration (+48 ctrl), ceres::CostFunction dtor 0x1801B77D0, free(). |
| 1800EC0A0 | PoseLocalParameterization / InitGravityLocalParameter::`scalar deleting destructor' | (unsigned) | project (compiler-generated, ICF-shared) | - | ceres::LocalParameterization dtor 0x1801B8DC0, free(). |
| 1800EC0F0 | - |  | lib: Concurrency::details::_CancellationTokenRegistration deleting dtor | - |  |
| 1800EC150 | - |  | lib: deleting dtor shared (ICF) by vk::solver::TukeyWeightFunction and std::_Iostream_error_category2 | - |  |
| 1800EC180 | - |  | lib: Concurrency::details::_Threadpool_chore deleting dtor (_Release_chore) | - |  |
| 1800EC1C0 | - |  | lib: Concurrency::details::_RefCounter deleting dtor | - |  |
| 1800EC1F0 | - |  | lib: std::exception / std::system_error deleting dtor | - |  |
| 1800EC240 | - |  | lib: Concurrency::details::_TaskProcHandle deleting dtor | - |  |
| 1800EC270 | - |  | lib: Concurrency::details::_Task_impl_base deleting dtor | - |  |
| 1800EC2B0 | FrameProcessorBase::mergeSimilarPlanes | void(std::vector<Plane>& planes) | project | new (Kimera Plane::geometricEqual without CHECKs + hull merge) | Merges later same-cluster planes that are geometrically equal (normal tol 0.015 global, dynamic distance tol) and whose hulls overlap (0x18019E480): hull merge 0x18019CA00, unique lmk_ids_ append, erase. |
| 1800ECF10 | PoseLocalParameterization::ComputeJacobian | bool(const double* x, double* jacobian) const | project | identical (VINS-Mono) | 7x6 row-major: topRows<6>().setIdentity(), bottomRows<1>().setZero(). |
| 1800ECFD0 | IMUFactor::Evaluate | bool(double const* const* parameters, double* residuals, double** jacobians) const | project | modified: 9 parameter blocks (pose,V,Ba,Bg x2 + gravity), shared_ptr pre_integration, global G overwritten, discarded normalized() calls, LOGW | VINS-Mono IMUFactor::Evaluate split into per-block jacobians plus d/dG; residual = sqrt_info * pre_integration->evaluate(...); "numerical unstable in preintegration". |
| 1800F1380 | - |  | lib: Eigen Matrix4d = Identity() assignment (with alignment assert) | - | Caller 0x1800E2B10 (c07). |
| 1800F1400 | IntegrationBase::rightJacobian | Eigen::Matrix3d(const Eigen::Vector3d& phi, double eps) const | project | new (header-inline helper of the Pimax IntegrationBase) | SO(3) right Jacobian, small-angle series below eps; caller midPointIntegration 0x180104140 (eps 1e-8). |
| 1800F19D0 | median (file-static helper) | double(std::vector<double> values) | project | new | std::sort; NaN if empty; even size -> mean of the middle pair. |
| 1800F1A60 | InitGravityLocalParameter::Plus | bool(const double* x, const double* delta, double* x_plus_delta) const | project | new (VINS RefineGravity S^2 update) | g+ = (g + TangentBasis(g) * dg).normalized() * \|g\|. |
| 1800F1DD0 | PoseLocalParameterization::Plus | bool(const double* x, const double* delta, double* x_plus_delta) const | project | identical (VINS-Mono) | p += dp; q = (q * deltaQ(dtheta)).normalized(). |
| 1800F2070 | FrameProcessorBase::updateGroundPlane | void(const Frame& frame) | project | new | cv::Subdiv2D Delaunay over corner features (ROI, 0<depth<=10, below IMU), triangle->lmk-id mapping, Mesher populate/cluster, refinePlanes, mergeSimilarPlanes, plane areas, selectGroundPlane; disables itself once area > 1.5 and ground valid. |
| 1800F3260 | Utility::R2ypr | Eigen::Vector3d(const Eigen::Matrix3d& R, bool radians) | project | modified (VINS-Mono R2ypr + radians flag) | Yaw/pitch/roll; degrees unless the flag is set. Callers 0x180110490, 0x18018D770. |
| 1800F3430 | FrameProcessorBase::refinePlanes | void(std::vector<Plane>& planes) | project | new | Updates g_max_plane_id from g_new_plane_ids, re-estimates plane distances (robust mean of z / n.p), triggers wall re-fit, refreshes hull vertices and recomputes convex hulls. |
| 1800F4190 | FrameProcessorBase::refitWallPlane | bool(Plane& plane) | project | new | JacobiSVD line fit on centred x/y of <= 51 landmarks; sets normal x/y and distance even when the outlier test then fails. |
| 1800F4BD0 | FrameProcessorBase::shouldRemoveKeyframe | bool(const FramePtr& frame) | project | new | True if < 5 tracked features or >= 90% of them have < 4.1 deg parallax to every other observing frame at the same pyramid level. (name from c07) |
| 1800F5170 | FrameProcessorBase::estimateGroundPlane | void(const FrameBundle& frame_bundle) | project | new | Fits the ground normal from the <= 48 closest bundle points inside the ground polygon; sends a relative ground-plane constraint between consecutive bundles to the estimator; logs "ground-plane relative normal candidates ...". (name from c09) |
| 1800F64A0 | FrameProcessorBase::selectGroundPlane | void() | project | new | Scores horizontal planes (median/MAD heights, inlier ratio, camera height 0.2..3 m), keeps the best and filters the ground model; LOGD "ground-plane ..." x6. |
| 1800F7330 | - |  | lib: std::_Packaged_state<void()> run + delete (std::function call) | - |  |
| 1800F7390 | - |  | lib: Eigen::aligned_allocator<T(72 bytes)>::allocate (vector realloc) | - |  |
| 1800F7420 | - |  | lib: std::_Task_async_state<void> worker body (PPL) | - |  |
| 1800F74C0 | - |  | lib: Concurrency::details::_Threadpool_chore callback/release | - |  |
| 1800F7500 | - |  | lib: Concurrency::details::_Task_impl<unsigned char>::_CancelAndRunContinuations | - |  |
| 1800F76C0 | - |  | lib: Concurrency::details::_Task_impl_base::_CancelWithException | - |  |
| 1800F7880 | - |  | lib: std::vector<16-byte>::_Change_array | - |  |
| 1800F7910 | - |  | lib: std::vector<24-byte>::_Change_array | - |  |
| 1800F79D0 | - |  | lib: std::vector<12-byte>::_Change_array | - |  |
| 1800F7A80 | - |  | lib: std::vector<unique_ptr (0x1800E3EF0)>::_Change_array | - |  |
| 1800F7B30 | - |  | lib: std::vector<24-byte w/ dtor 0x1800BAB00>::_Change_array | - |  |
| 1800F7BF0 | - |  | lib: std::vector<std::vector<std::vector<sp>>>::_Change_array | - |  |
| 1800F7CD0 | - |  | lib: std::vector<int>::_Reallocate_exactly / reserve | - |  |
| 1800F7D90 | - |  | lib: std::vector<16-byte (shared_ptr)>::_Reallocate | - |  |
| 1800F7E70 | - |  | lib: std::_Associated_state::_Set_ready / notify (mutex+condvar) | - |  |
| 1800F7EC0 | - |  | lib: std::_Func_impl_no_alloc<lambda,...>::_Copy/_Move (const std::_Func_impl_no_alloc<_lambda_052e919cc0e5399df76dff3972c0cac1_,uchar,>::`vftable) | - |  |
| 1800F7F30 | - |  | lib: std::_Func_impl_no_alloc<lambda,...>::_Copy/_Move (const std::_Func_impl_no_alloc<_lambda_2fa3e3d11fb97352afa77a4a13bfb543_,void,>::`vftable') | - |  |
| 1800F7F50 | - |  | lib: std::_Func_impl_no_alloc<lambda,...>::_Copy/_Move (const std::_Func_impl_no_alloc<_lambda_3184c424febd4fe72fe42e9ad8c42595_,void,>::`vftable') | - |  |
| 1800F7F70 | - |  | lib: std::_Func_impl_no_alloc<lambda,...>::_Copy/_Move (const std::_Func_impl_no_alloc<_lambda_763529b0c7473cbc215a52d189ac9b18_,void,>::`vftable') | - |  |
| 1800F7F90 | - |  | lib: std::_Func_impl_no_alloc<lambda,...>::_Copy/_Move (const std::_Func_impl_no_alloc<_lambda_f25c37099038263181b5186a3fa41b37_,void,>::`vftable') | - |  |
| 1800F7FD0 | - |  | lib: std::_Func_impl_no_alloc<lambda,...>::_Copy/_Move (const std::_Func_impl_no_alloc<std::_Fake_no_copy_callable_adapter<_lambda_3d1ed943d4c6d2c) | - |  |
| 1800F7FF0 | - |  | lib: std::_Task_async_state<void> construction (std::async launch) | - | Caller 0x1800B5AD0. |
| 1800F8340 | - |  | lib: std::_Func_impl_no_alloc<...>::_Delete_this | - |  |
| 1800F83A0 | - |  | lib: std::_Func_impl_no_alloc<...>::_Delete_this | - |  |
| 1800F83B0 | - |  | lib: std::_Func_impl_no_alloc<...>::_Delete_this | - |  |
| 1800F8430 | - |  | lib: Concurrency::details::_Task_impl_base registration helper | - |  |
| 1800F8580 | - |  | lib: std::_Hash::_Desired_grow_bucket_count (ceilf(size / max_load_factor)) | - |  |
| 1800F8620 | - |  | lib: std::_Ref_count<KeyFrame>::_Destroy (delete KeyFrame) | - |  |
| 1800F8650 | - |  | lib: std::_Ref_count_obj2<_ExceptionHolder>::_Destroy | - |  |
| 1800F8660 | - |  | lib: std::_Ref_count_obj2<FrameBundle>::_Destroy (inlined ~FrameBundle) | - |  |
| 1800F86C0 | - |  | lib: std::_Ref_count_obj2<IntegrationBase>::_Destroy (frees gyr_buf, acc_buf, dt_buf at +10552/+10528/+10504 of the object) | - |  |
| 1800F8750 | - |  | lib: std::_Ref_count_obj2<Mesher>::_Destroy (inlined ~Mesher) | - |  |
| 1800F8790 | - |  | lib: std::_Ref_count_obj2<Point>::_Destroy (~Point 0x18009A160) | - |  |
| 1800F87A0 | - |  | lib: std::_Ref_count_resource<FrameBundle*, default_delete>::_Destroy | - |  |
| 1800F8810 | - |  | lib: std::_Ref_count_resource<Map*, default_delete>::_Destroy | - |  |
| 1800F8840 | - |  | lib: std::_Destroy_range / allocator destroy helper | - |  |
| 1800F8860 | - |  | lib: std::_Destroy_range / allocator destroy helper | - |  |
| 1800F88A0 | - |  | lib: std::_Destroy_range / allocator destroy helper | - |  |
| 1800F88E0 | - |  | lib: std::_Destroy_range / allocator destroy helper | - |  |
| 1800F8900 | - |  | lib: std::_Destroy_range / allocator destroy helper | - |  |
| 1800F8940 | - |  | lib: std::_Destroy_range / allocator destroy helper | - |  |
| 1800F8960 | - |  | lib: Concurrency::details::_RefCounter::_Destroy (delete this) | - |  |
| 1800F8980 | - |  | lib: std::_Func_impl_no_alloc<...>::_Do_call (const std::_Func_impl_no_alloc<_lambda_052e919cc0e5399df76dff3972c0cac1_,uchar,>) | - |  |
| 1800F89A0 | - |  | lib: std::_Func_impl_no_alloc<...>::_Do_call (const std::_Func_impl_no_alloc<_lambda_2fa3e3d11fb97352afa77a4a13bfb543_,void,>:) | - |  |
| 1800F89B0 | - |  | lib: std::_Func_impl_no_alloc<...>::_Do_call (const std::_Func_impl_no_alloc<_lambda_3184c424febd4fe72fe42e9ad8c42595_,void,>:) | - |  |
| 1800F89C0 | - |  | lib: std::_Func_impl_no_alloc<...>::_Do_call (const std::_Func_impl_no_alloc<_lambda_763529b0c7473cbc215a52d189ac9b18_,void,>:) | - |  |
| 1800F89D0 | - |  | lib: std::_Func_impl_no_alloc<...>::_Do_call (const std::_Func_impl_no_alloc<_lambda_f25c37099038263181b5186a3fa41b37_,void,>:) | - |  |
| 1800F89F0 | - |  | lib: std::_Func_impl_no_alloc<...>::_Do_call (const std::_Func_impl_no_alloc<std::_Fake_no_copy_callable_adapter<_lambda_3d1ed) | - | 0x1800F89F0 calls the projectMapInFrame lambda 0x1800E61D0. |
| 1800F8A00 | - |  | lib: std::_Associated_state<...>::_Do_notify | - |  |
| 1800F8A40 | - |  | lib: Concurrency::details::_CancellationTokenCallback<lambda>::_Exec | - |  |
| 1800F8B10 | - |  | lib: Concurrency::details::_Task_impl_base::_Cancel/_FinalizeAndRunContinuations | - |  |
| 1800F8BD0 | - |  | lib: std::_Hash<...>::_Forced_rehash ("invalid hash bucket count") | - |  |
| 1800F8D50 | - |  | lib: std::_Hash<...>::_Forced_rehash (second instantiation) | - |  |
| 1800F8F70 | - |  | lib: std::_Ref_count_resource<FrameBundle*,...>::_Get_deleter | - |  |
| 1800F8FA0 | - |  | lib: std::_Ref_count_resource<Map*,...>::_Get_deleter | - |  |
| 1800F8FD0 | - |  | lib: std::_Associated_state<int>::_Get_value | - |  |
| 1800F9100 | - |  | lib: std::_Task_async_state<void>::_Get_value (waits) | - |  |
| 1800F9240 | - |  | lib: boost detail serializer predicate returning (*(p+16) != 0) (ICF-shared) | - |  |
| 1800F9250 | - |  | lib: std::_Task_async_state<void> ctor lambda / PPL create_task | - |  |
| 1800F93C0 | - |  | lib: Concurrency / <future> runtime helper (task scheduling / wait / exception propagation) | - |  |
| 1800F9450 | - |  | lib: std::_Func_impl_no_alloc<lambda,...>::_Copy/_Move (const std::_Func_impl_no_alloc<_lambda_f25c37099038263181b5186a3fa41b37_,void,>::`vftable') | - |  |
| 1800F9490 | - |  | lib: std::_Func_impl_no_alloc<lambda,...>::_Copy/_Move (const std::_Func_impl_no_alloc<std::_Fake_no_copy_callable_adapter<_lambda_3d1ed943d4c6d2c) | - |  |
| 1800F94B0 | - |  | lib: Concurrency / <future> runtime helper (task scheduling / wait / exception propagation) | - |  |
| 1800F94E0 | - |  | lib: std::vector<std::pair<double,int>>::_Reallocate (reserve) | - | Caller 0x1800F5170. |
| 1800F9550 | - |  | lib: std::vector<PolygonVertex>::_Reallocate (reserve) | - | Callers 0x1800F3430, 0x18019CA00. |
| 1800F95D0 | - |  | lib: std::vector<Eigen::Vector3d>::_Reallocate (reserve) | - | Caller 0x1800F5170. |
| 1800F96A0 | - |  | lib: std::vector<12-byte>::_Reallocate (reserve) | - |  |
| 1800F9740 | - |  | lib: std::vector<8-byte>::_Reallocate (reserve) | - |  |
| 1800F97C0 | - |  | lib: std::vector<unique_ptr>::_Reallocate (reserve) | - |  |
| 1800F9830 | - |  | lib: std::vector<24-byte w/ dtor>::_Reallocate (reserve) | - |  |
| 1800F98C0 | - |  | lib: Concurrency / <future> runtime helper (task scheduling / wait / exception propagation) | - |  |
| 1800F9980 | - |  | lib: std::_Hash::_Check_rehash bucket computation (ceilf) | - |  |
| 1800F9A30 | - |  | lib: Concurrency / <future> runtime helper (task scheduling / wait / exception propagation) | - |  |
| 1800F9A70 | - |  | lib: Concurrency / <future> runtime helper (task scheduling / wait / exception propagation) | - |  |
| 1800F9AA0 | - |  | lib: Concurrency / <future> runtime helper (task scheduling / wait / exception propagation) | - |  |
| 1800F9BB0 | - |  | lib: Concurrency / <future> runtime helper (task scheduling / wait / exception propagation) | - |  |
| 1800F9C70 | - |  | lib: Concurrency / <future> runtime helper (task scheduling / wait / exception propagation) | - |  |
| 1800F9DB0 | - |  | lib: Concurrency / <future> runtime helper (task scheduling / wait / exception propagation) | - |  |
| 1800F9E20 | - |  | lib: Concurrency / <future> runtime helper (task scheduling / wait / exception propagation) | - |  |
| 1800F9F00 | - |  | lib: std::_Func_impl_no_alloc<...>::_Target_type | - |  |
| 1800F9F10 | - |  | lib: std::_Func_impl_no_alloc<...>::_Target_type | - |  |
| 1800F9F20 | - |  | lib: std::_Func_impl_no_alloc<...>::_Target_type | - |  |
| 1800F9F30 | - |  | lib: std::_Func_impl_no_alloc<...>::_Target_type | - |  |
| 1800F9F40 | - |  | lib: std::_Func_impl_no_alloc<...>::_Target_type | - |  |
| 1800F9F50 | - |  | lib: std::_Func_impl_no_alloc<...>::_Target_type | - |  |
| 1800F9F60 | - |  | lib: std::_Throw_Cpp_error (throws std::system_error(make_error_code(e))) | - |  |
| 1800F9FA0 | - |  | lib: std::_Uninitialized_value_construct_n of list heads (operator new(0x20)) | - |  |
| 1800FA020 | - |  | lib: std::_Uninitialized_move (memmove) for 4-byte elements | - |  |
| 1800FA060 | - |  | lib: std::_Uninitialized_copy of 128-byte elements | - |  |
| 1800FA0D0 | - |  | lib: std::_Uninitialized_copy of shared_ptr range | - |  |
| 1800FA130 | - |  | lib: std::unordered_map<int,Eigen::Vector3f>::erase(first,last) (used by clear()) | - | Callers 0x1800EC2B0, 0x1800FE8F0, 0x18019FA90. |
| 1800FA300 | - |  | lib: std::unordered_map<...>::erase(first,last) (second instantiation) | - |  |
| 1800FA4D0 | - |  | lib: Concurrency / <future> runtime helper (const std::_Associated_state<int>::`vftable'[1] ; const std::_Packaged_state<void (void)>:) | - |  |
| 1800FA560 | - |  | lib: Concurrency / <future> runtime helper (const std::_Task_async_state<void>::`vftable'[1]) | - |  |
| 1800FA580 | - |  | lib: Concurrency / <future> runtime helper (task scheduling / wait / exception propagation) | - |  |
| 1800FA5E0 | - |  | lib: Concurrency / <future> runtime helper (task scheduling / wait / exception propagation) | - |  |

## Types

### IntegrationBase (VINS-Mono, Pimax variant), sizeof 0x2950 = 10576
- **Allocation.** `make_shared<IntegrationBase>` in 0x180110490 allocates a 0x2960 block; the object starts at +16.
- **Ctor** 0x1800E20B0 (c07), signature `(acc_0, gyr_0, linearized_ba, linearized_bg)`.
- **Destroy.** `_Ref_count_obj2<IntegrationBase>::_Destroy` is 0x1800F86C0. It frees only the three buffers.
- **Methods.** All are header-inline COMDATs in frame_processor_base.obj:

| method | address | chunk |
|---|---|---|
| `rightJacobian` | 0x1800F1400 | c08 |
| `skewSymmetric` | 0x180110290 | c09 |
| `midPointIntegration` | 0x180104140 | c09 |
| `evaluate` | 0x180109180 | c09 |
| `propagate` | 0x18011A810 | c10 |
| `repropagate` | 0x18011AFE0 | c10 |

| offset | type | name | evidence |
|---|---|---|---|
| +0 | double | ? (0) | ctor (sure: written) |
| +8 | double | ? (0) | ctor |
| +16 | double | g_norm? = 9.80667 (0x40239D03D9A95422) | ctor; use unknown |
| +24 | double | acc_n = 0.04 | ctor: noise(0,0),(6,6) = acc_n^2 (sure) |
| +32 | double | acc_w = 0.004 | noise(12,12) = acc_w^2 (sure) |
| +40 | double | gyr_n = 0.008 | noise(3,3),(9,9) (sure) |
| +48 | double | gyr_w = 0.0008 | noise(15,15) (sure) |
| +56 | double | dt | propagate 0x18011A810 (sure) |
| +64 / +88 | Vector3d | acc_0 / gyr_0 | ctor args 1/2, propagate (sure) |
| +112 / +136 | Vector3d | acc_1 / gyr_1 | propagate (sure) |
| +160 / +184 | Vector3d (const) | linearized_acc / linearized_gyr | ctor (sure) |
| +208 / +232 | Vector3d | linearized_ba / linearized_bg | ctor args 3/4; Evaluate reads +232 (sure) |
| +256 | Matrix<double,15,15> | jacobian | Identity in ctor; Evaluate reads the dp_dba/dv_dba/dp_dbg/dq_dbg/dv_dbg blocks at +1336/+1384/+1696/+1720/+1744 (sure) |
| +2056 | Matrix<double,15,15> | covariance | Zero; Evaluate: LLT(covariance.inverse()) (sure) |
| +3856 | Matrix<double,15,15> | step_jacobian | layout |
| +5656 | Matrix<double,15,18> | step_V | layout |
| +7824 | Matrix<double,18,18> | noise | ctor (sure) |
| +10416 | double | sum_dt | Evaluate, propagate (sure) |
| +10424 | Vector3d | delta_p | ctor, propagate (sure) |
| +10448 | Quaterniond | delta_q | Evaluate (sure) |
| +10480 | Vector3d | delta_v | (sure) |
| +10504 / +10528 / +10552 | vector<double>, vector<Vector3d> x2 | dt_buf / acc_buf / gyr_buf | _Destroy 0x1800F86C0 (sure) |

### IMUFactor, sizeof 0x50
- **Base.** `ceres::SizedCostFunction<15, 7,3,3,3, 7,3,3,3, 3>`, block sizes from 0x1803B6DA0. The 9 parameter blocks are pose_i, Vi, Bai, Bgi, pose_j, Vj, Baj, Bgj, G.
- **vtable** 0x1803B2DF8: [0] dtor 0x1800EC000, [1] Evaluate 0x1800ECFD0.
- **Ctor** 0x1800E1950 (c07), signature `(shared_ptr<IntegrationBase>, const Vector3d&)`. It is allocated with Eigen's aligned new (malloc 0x50), hence `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`.

| offset | type | name | evidence |
|---|---|---|---|
| +0 | vptr | | |
| +8 | std::vector<int32> | parameter_block_sizes_ | ceres |
| +32 | int | num_residuals_ = 15 | ceres |
| +40 | std::shared_ptr<IntegrationBase> | pre_integration | Evaluate reads +40; the dtor releases +48 (sure) |
| +56 | Eigen::Vector3d | unknown (3rd ctor argument) | ctor; not read by Evaluate. Name TODO |

### PoseLocalParameterization (vtable 0x1803B2DA8) / InitGravityLocalParameter (vtable 0x1803B2E38)
- Both are 8 bytes (vptr only). They are allocated with Eigen aligned new (malloc 8). The scalar deleting dtor 0x1800EC0A0 is ICF-shared and calls free().
- PoseLocalParameterization: GlobalSize 7, LocalSize 6, the same as VINS.
- InitGravityLocalParameter: GlobalSize 3, LocalSize 2.

### Plane (Kimera `Plane`, Pimax), sizeof 232
- The dtor 0x1800E5230 is implicit.
- Changes from Kimera:
  - The gtsam::Symbol is replaced by an int id.
  - The normal is a `cv::Point3f` (Kimera uses `cv::Point3d`).
  - The polygon, all_ids, flags, age, ref distance and area fields are added.

| offset | type | name | evidence |
|---|---|---|---|
| +0 | int | id | "%d id" logs (sure) |
| +4 | cv::Point3f | normal_ | ddot in 0x1800EC2B0 / 0x1800F4190 (sure) |
| +16 | double | distance_ | (sure) |
| +24 | std::vector<12-byte> | ? | dtor only (type unsure) |
| +48 | unordered container (64 B) | ? | dtor 0x180077970 (unsure) |
| +112 | 16 bytes | ? | unused here |
| +128 | std::vector<int> | lmk_ids_ ("local_ids") | merge, wall fit, ground (sure) |
| +152 | std::vector<PolygonVertex> | polygon_ ("hull") | (sure) |
| +176 | int | cluster_id_ (1 wall, 2 horizontal) | (sure) |
| +184 | std::vector<int> | all_lmk_ids_ ("all_ids") | refinePlanes (sure) |
| +208 | uint8 | ? | |
| +209 | bool | hull_clockwise_ | convexHull clockwise flag, mergePolygons out-param (name unsure) |
| +210 | bool | needs_wall_refit_ | refinePlanes (name unsure) |
| +212 | int | age_ | `> 5` (name unsure) |
| +216 | double | distance_ref_ | set to distance_ when 0 (name unsure) |
| +224 | double | area_ | 0x1800FE140 result (sure) |

### PolygonVertex, 16 bytes
`{ int lmk_id; Eigen::Vector3f pos; }`. The pos member is used directly as an Eigen 3x1 block in `R * v.pos`.

### Landmark maps
- `LmkPositionMap = std::unordered_map<int, Eigen::Vector3f>`. Node 0x20: key at +16, value at +20. Used by FrameProcessorBase+3608, the mesher input, and locals.
- `LmkPixelMap = std::unordered_map<int, Eigen::Vector2d>`. Node 0x30, value 16-aligned at +32. Used by a local in 0x1800F2070 and Mesher+1152.

### Mesher (partial)
- make_shared block 0x4D8.
- +1104 `Matrix3f R_w_c_`, +1140 `Vector3f t_w_c_`, +1152 `LmkPixelMap`. 0x1800F2070 writes all three.

### FrameProcessorBase
- This chunk proves the members in `frontend/frame_processor_base_c08.h`. Main facts:
  - The +1616 byte is set by the dtor.
  - +2784 is a deque with an aligned allocator and 24-byte elements, followed by three RunningStats at +2824/+2848/+2872. resetBackend clears all of them.
  - +2912 is `T_world_correction_` and +3216 is `T_prior_`; the dtor saves `T_world_correction_ * T_prior_`.
  - +3408/+3424 hold the backend and its type.
  - +3592 to +3896 hold the mesh and ground state, listed with types above.
- The dtor proves the member-destruction order. The list is in the dtor draft comment.
- vtable 0x1803B2E70: the scalar deleting dtor 0x1800EBFB0 frees with free(), so the class has `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`.
- Ctor defaults for the c08 members (from 0x1800DFDF0):

| member | default |
|---|---|
| mesh_enabled_ | 1 |
| ground_normal_ | (0,0,1) |
| ground_sigma_ | 0.08 |
| ground_rel_bundle_id_ | -1 |
| ground_rel_normal_b_, ground_rel_normal_w_ | (0,0,1) |
| ground_rel_sigma_ | 0.15 |

### GroundPlaneConstraint (c03 type, 72 bytes)
- 0x1800F5170 builds it on the stack as `{valid=1, bundle_id_0=prev, bundle_id_1=cur, normal_0=prev n_b, normal_1=cur n_b, sigma}`.
- Field offsets: 0/4/8/16/40/64. Verified from the asm stores before 0x18002C530.

## External interfaces (called from c08 project code)
| address | meaning / inferred signature |
|---|---|
| 0x180109180 | `IntegrationBase::evaluate(Pi,Qi,Vi,Bai,Bgi,Pj,Qj,Vj,Baj,Bgj,G)` -> `Matrix<double,15,1>` (c09) |
| 0x180110290 | `IntegrationBase::skewSymmetric(const Vector3d&) const` (comma init; also defined in the c08 header) |
| 0x180042410 | `Utility::skewSymmetric<Vector3d>` |
| 0x1800CBD00 | `Utility::deltaQ<Product<Matrix3d, Vector3d-Vector3d>>` |
| 0x1800B8A50 / 0x1800B8C00 | `Utility::Qleft<Quaterniond>` / `Utility::Qright<Quaterniond>` (with 0x1800DDA80 = skewSymmetric of the vec block) |
| 0x18001ACE0 | `TangentBasis(Vector3d&)` -> `MatrixXd(3,2)` (VINS initial_alignment) |
| 0x1800426F0 / 0x1800132A0 / 0x18004F880 / 0x180029DF0 / 0x180104070 | Quaterniond normalized / inverse / product / toRotationMatrix / conjugate (Eigen) |
| 0x1801284D0, 0x1800DD270 | Quaternionf toRotationMatrix, FromTwoVectors(a,b) (Eigen float) |
| 0x1800149F0 + 0x180012B20 | inline body of `CeresBackendInterface::reset()` (LOGI "Backend: Reset\n"; reset of backend+176...) |
| 0x180041E40 | deque<…,aligned_allocator>::_Tidy (clear) |
| 0x180009B80 | `Transformation::operator*` (kindr, renormalising) |
| 0x180013040 | `Transformation::inverse()` |
| 0x180125040 | `FrameProcessorBase::savePriorPosition(const Transformation&)` (c10) |
| 0x180110400 | `Frame::imuPos()` -> `Eigen::Vector3f` |
| 0x180024D50 | `Frame::T_world_imu()` |
| 0x1800116D0 | `FrameBundle::at(size_t)` / `std::vector<FramePtr>::at` |
| 0x1801996D0 | `Mesh3D::Mesh3D(const size_t& polygon_dimension)` (Kimera Mesh) |
| 0x18019DB90 | `Mesher::populate3dMesh(triangles, tri_lmk_ids, lmk_positions, min_ratio, min_elongation, max_side, Mesh3D*)` -> bool |
| 0x1801A0AF0 | `Mesher::clusterPlanesFromMesh(vector<Plane>*, vector<Plane>*, const LmkPositionMap&)` |
| 0x18019E480 | `polygonsOverlap(const vector<PolygonVertex>&, vector<int>, const vector<PolygonVertex>&, vector<int>, const LmkPositionMap&)` -> bool (not a member: no `this`) |
| 0x18019CA00 | `mergePolygons(vector<PolygonVertex>* a, const vector<PolygonVertex>& b, const Vector3f& normal_a, bool* clockwise)` |
| 0x1800FE140 | `polygonArea(const Plane&)` -> double (shoelace; c09) |
| 0x18002C530 / 0x1800298A0 | `Estimator::setGroundPlaneConstraint` / `resetGroundPlaneConstraint` (on bundle_adjustment_+176) |
| 0x180027A20 | `isFinite(const Eigen::Vector3d&)` |
| 0x18000C120 / 0x18000F500 / 0x18000F6A0 | LOGD / LOGI / LOGW |
| OpenCV imports | `cv::Subdiv2D::{Subdiv2D(Rect), insert(vector<Point2f>), getTriangleList, findNearest}`, `cv::convexHull`, `cv::pointPolygonTest`, `cv::Mat::~Mat` |
| CRT | atan2, sin, cos, acos, acosf, sqrt, fmaxl, fminl, _dclass |

## Globals
| address | type | meaning / initial value |
|---|---|---|
| 0x18047ED98 | Eigen::Vector3d | VINS global `G`. Every IMUFactor::Evaluate overwrites it with parameters[8]. 0x180110490 reads it. |
| 0x18046A240 | double | 0.5: Kimera min_ratio_btw_largest_smallest_side (populate3dMesh arg) |
| 0x18046A248 | double | 0.5: Kimera min_elongation_ratio |
| 0x18046A250 | double | 1.0: max_triangle_side (Kimera 0.5). Passed as `* 0.5`. |
| 0x18046A2B8 | double | 0.15: refinePlanes height gate (`+ 0.15`); also used by the mesher (name guess `g_wall_distance_tolerance`) |
| 0x18046A2D8 | double | 0.015: normal tolerance plane-plane (Kimera 0.011) |
| 0x18046A2E8 | double | 0.1: distance tolerance plane-plane (Kimera 0.20) |
| 0x18046A2F0 | double | 0.15: plane distance update gate (`- 0.12`) |
| 0x18046A2F8 | int | -1: g_max_plane_id (max over g_new_plane_ids; also read by 0x1801A3750) |
| 0x18047EF68 | std::vector<int> | g_new_plane_ids. refinePlanes reads and clears it; the mesher fills it (0x1801A1095). |

These are plain mutable globals in `.data`; the Kimera gflags were replaced. The block 0x18046A240 to 0x18046A2FC holds the whole Kimera flag set: bools 01 01 01 01 at +0x258; ints 6, 2, 3 at +0x25C; 0.5, 0.1, 512, 15, -4, 4, 30, 3, 3.14, -4, 4, 0.025, 0.15, 0.015 x4, 0.2, 0.1, 0.15, -1. The mesher chunk should name them.

## Constants / config defaults
- **IMUFactor**
  - 1e8 and -1e8 are the jacobian sanity bounds.
  - deltaQ divides by {2.0, 2.0} and uses *0.5 for z.
- **IntegrationBase ctor (c07, for reference)**: acc_n 0.04, acc_w 0.004, gyr_n 0.008, gyr_w 0.0008, +16 = 9.80667. The noise blocks are the squares.
- **rightJacobian**: eps 1e-8 (passed by the caller); 1/6 = 0.16666666666666666.
- **updateGroundPlane**
  - The point must satisfy 0 < depth <= 10.0, be inside the ROI, be a kCorner (7), and lie below the IMU.
  - findNearest acceptance is d^2 < 1e-4f; the fallback search starts at FLT_MAX.
  - mesh_enabled_ is switched off when max plane area > 1.5 and the ground is valid.
- **refinePlanes**
  - age > 5 and |ref| < 0.001.
  - First accepted z must be within 10.0 of distance_; later ones within g(0.15) + 0.15 of the running mean.
  - At least 7 points are required.
  - Distance update gate: (g(0.15) - 0.12) * clamp(|avg - n.t_w_c|, 0.5, 1.0).
  - A hull vertex is refreshed only if within 0.4 of the plane.
- **refitWallPlane**
  - Uses 15 to 51 points, and requires singular value sv0 >= 0.3.
  - Outlier residual threshold 0.2; the fit fails when outliers reach half of the ids.
  - SVD flags 0x14 (ComputeFullU | ComputeFullV).
- **mergeSimilarPlanes**: distance tolerance = clamp(|n1.t_w_c - d1|, 0.5, 1.3) * 0.1; normal test > 1 - 0.015.
- **selectGroundPlane**
  - Plane filter: cluster 2, area >= 0.3, hull >= 3, |n| > 1e-6, n.z >= 0.98, at least 8 projections.
  - Statistics: MAD factor 1.4826; gate = clamp(3σ, 0.03, 0.1); inlier ratio >= 0.7; σ <= 0.05; at least 8 inliers.
  - Camera height must be in [0.2, 3.0]. Candidate gate = clamp(2σ, 0.05, 0.1). Score = sqrt(inliers) * area / (10σ + 1).
  - Filter update: accept if normal dot > 0.995 and the distance is within max(0.04, 3 max(σ_model, gate)). Up to k = 5 averaging steps; confirmations are capped at 1000.
  - The model is valid at >= 3 confirmations with >= 3 polygon points. It is reset after more than 5 misses. The initial best gate is 0.08.
- **estimateGroundPlane**
  - Point gate = clamp(2.5 σ_model, 0.06, 0.15). Points need obs >= 3 and pointPolygonTest >= -0.1.
  - Requires >= 8 candidates. Keeps the 48 closest and needs >= 12 fitted.
  - Eigenvalues: λ1 >= 0.01, λ2 >= 0.04, λ0/λ1 <= 0.08.
  - Angle checks: cos 15° (0.2617993877991494 rad) against the model; cos 2° (0.03490658503988659 rad) against the previous fit.
  - Fit σ = clamp(3 atan2(sqrt(max(λ0,0)), sqrt(max(λ1,1e-12))), 0.12, 0.25).
  - A constraint needs overlap >= 8 ids and ratio >= 0.6. Constraint σ = clamp(0.5 sqrt(σ² + σ_prev²), 0.08, 0.18).
- **shouldRemoveKeyframe**: initial max cos -1.1f; small-parallax limit 4.1°; returns true below 5 features or when the small fraction is >= 0.9.
- **R2ypr**: π = 3.141592653589793 and 180.0.

## Quirks / bugs to preserve
1. **Discarded normalized() calls.** `IMUFactor::Evaluate` calls `Qi.normalized()`, `Qj.normalized()` and `corrected_delta_q.normalized()` (twice) and throws the results away. All residuals and jacobians use the **un-normalised** quaternions read from the parameter blocks.
2. **Global G overwritten.** `IMUFactor::Evaluate` writes the gravity parameter block into the global `G` on every call, including during ceres line searches. The local copy is what it uses itself.
3. **Warning without newline.** "numerical unstable in preintegration" is printed via LOGW with no trailing newline. It is checked on `pre_integration->jacobian` and again on the weighted `jacobian_pose_i`.
4. **Destructor side effects.** The destructor saves the prior position file and resets the backend. `resetBackend` never releases `bundle_adjustment_`.
5. **mergeSimilarPlanes gate and dead branch.** It gates on `planes_.size()`, not on its argument (they are the same object). Because walls (cluster 1) are skipped as the first plane, the rotated-landmark branch for walls is dead code. Keep it.
6. **refitWallPlane writes before testing.** It writes the new normal and distance into the plane **before** the outlier test, so a failed refit leaves modified parameters. Only normal_.x and .y are written; z stays as before. sz is accumulated but unused.
7. **Unused centroid.** refinePlanes accumulates `centroid` for walls but never uses it. The Eigen float dot orders differ deliberately: `t.dot(n)` for horizontal planes, `n.dot(t)` for walls.
8. **NaN depth passes.** In updateGroundPlane a NaN depth passes the depth test, because the binary skips only on `depth <= 0 || depth > 10`. The depth formula is written exactly as compiled; see `depthInFrame`.
9. **Rotation direction differs.** refinePlanes rotates with `FromTwoVectors(ez, n)`, while mergeSimilarPlanes uses `FromTwoVectors(n, ez)` (verified from the argument registers).
10. **Second weak_ptr lock.** shouldRemoveKeyframe locks the observing frame's weak_ptr a second time to read its pose.
11. **NaN behaviour of min/max.** All min/max clamps are written as `std::min(C, x)` / `std::max(C, x)` with the constant first. This matches the NaN behaviour of the binary's comisd/cmov sequences. The one maxsd in selectGroundPlane is written `std::max(0.04, ...)`.
12. **fmaxl/fminl.** shouldRemoveKeyframe calls the CRT fmaxl/fminl. The draft uses std::fmax/std::fmin on doubles, which behave the same on MSVC. TODO(verify) the spelling.

## Open questions
- Names of IMUFactor +56, IntegrationBase +0/+8/+16, Plane +24/+48/+112/+208..+216, FrameBundle +252 (bundle id), and the exact original spelling of the depth expression in 0x1800F2070.
- Why the compiled depth formula factors out the 2s (likely `(R*p + t).z()` with MSVC strength reduction). The results are bit-identical either way.
- Whether `G` was assigned as `G = Gw` or through another statement. Behaviour is the same.
- 0x1800E61D0 (projectMapInFrame's async lambda) is project code in this range. c10's draft reproduces it, and I checked that version against the binary: insert at begin, unordered_set dedup, assign, sort, then `reprojectors_.at(i)->reprojectFrames(new_frames_->at(i), overlap_kfs_.at(i), trash_points_.at(i), imu_not_initialized_)`. It is not duplicated here.
