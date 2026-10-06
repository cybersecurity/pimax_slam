# c02_ceres_map — 0x18001B4D0 .. 0x180027AE0

Drafts: `draft/c02_ceres_map/ceres_backend/{ceres_map.h, ceres_map.cpp, gravity_local_parameterization.h,
estimator.h, estimator_c02.cpp, estimator_types_c02.h}`.

## Summary / object boundaries

* **0x18001B4D0 – 0x1800203FA: `ceres_map.obj`** (`pimax::totem::ceres_backend::Map`, upstream
  `svo_ceres_backend/src/map.cpp`) plus a few Eigen/std instantiations and two inline static helpers of
  `GravityLocalParameterization` (plus/minus). `Map::Map(bool)` (0x180018E90), `TangentBasis` (0x18001ACE0),
  `0x18001A980` and the Map-related vector/hash helpers lie *before* this chunk (c01).
* **0x180020440 – 0x180027AE0: start of `estimator.obj`**, not ceres_map. Evidence: 0x180020440 is the
  `insert(first,last)` used by the `kErrorToStr` dynamic initializer 0x180001030; all `_Ref_count_obj2<...>` of
  estimator types (Map, ImuError, PoseError, SpeedAndBias*, Gravity*, General3D*, GroundPlaneError); the
  Estimator ctors/dtor; in `.rdata` "numUsedImuMeasurements less 1\n" (used at 0x180026100) sits directly before
  the `estimator.cpp` `__FILE__` string, and "GroundPlaneError"/"5_finish" are between the ceres_map strings and
  it. Project functions there: `Estimator::Estimator(shared_ptr<Map>)`, `Estimator()`, `~Estimator`,
  `addCameraBundle`, `addGroundPlaneError`, `addImu`, `addLandmark`, `addStates`, `addVelocityPrior`,
  inline `States::addState`, `Frame::T_world_imu`, `isFinite(Vector3d)`. They are drafted in
  `estimator_c02.cpp` and must be merged with chunk c03's estimator.cpp. **The Estimator layout below must be
  reconciled with c03.**
* None of the Estimator functions in this chunk use glog; the only message is `LOGE`.

## Function table

| address | size | proposed qualified name | signature | kind | upstream status | summary |
|---|---|---|---|---|---|---|
| 0x18001B4D0 | 869 | `ceres_backend::Map::addParameterBlock` | `bool (const shared_ptr<ParameterBlock>&, int parameterization=Trivial, int group=-1)` | project | modified: Gravity=2/Trivial=3, const-ref param, no DEBUG_CHECK | VLOG(200) l.164, LOG(ERROR) l.169/201, AddParameterBlock with HP/Pose/Gravity/no local param |
| 0x18001B840 | 2853 | `ceres_backend::Map::addResidualBlock` | `ResidualBlockId (shared_ptr<CostFunction>, LossFunction*, vector<shared_ptr<ParameterBlock>>&)` | project | modified: reserve(), push to new_residual_block_ids_ (+0x550), no DEBUG_CHECK | ceres AddResidualBlock + 3 book-keeping maps; VLOG(200) dump via stringstream l.268 |
| 0x18001C370 | 1742 | `ceres_backend::Map::addResidualBlock` | `ResidualBlockId (shared_ptr<CostFunction>, LossFunction*, shared_ptr<ParameterBlock> x0..x9)` | project | modified: reserve(10), no DEBUG_CHECK | collects non-null x0..x9 and forwards |
| 0x18001CA40 | 112 | `std::allocator<8-byte T>::allocate` | `T* (size_t)` | lib:std::_Allocate (8-byte elements, 32-byte big-block alignment) | - | shared by many vectors |
| 0x18001CAB0 | 116 | `std::allocator<24-byte T>::allocate` | `T* (size_t)` | lib:std::_Allocate (24-byte elements) | - |  |
| 0x18001CB30 | 67 | `std::allocator<8-byte T>::deallocate` | `void (T*, size_t)` | lib:std::_Deallocate | - |  |
| 0x18001CB80 | 67 | `std::allocator<24-byte T>::deallocate` | `void (T*, size_t)` | lib:std::_Deallocate | - |  |
| 0x18001CBD0 | 18 | `std::dec` | `ios_base& (ios_base&)` | lib:std inline manipulator instance | - | flags = (flags & ~basefield) | dec |
| 0x18001CBF0 | 383 | `unordered_map<ResidualBlockId,Map::ResidualBlockSpec>::erase(const key&)` | `size_t (const key&)` | lib:std::_Hash::erase(key) | - | used by removeResidualBlock |
| 0x18001CD70 | 300 | `ceres_backend::Map::errorInterfacePtr` | `shared_ptr<const ErrorInterface> (ResidualBlockId) const` | project | identical | find in spec map, null if absent |
| 0x18001CEA0 | 18 | `std::hex` | `ios_base& (ios_base&)` | lib:std inline manipulator instance | - | used by operator<<(ostream,BackendId) |
| 0x18001CEC0 | 161 | `ceres_backend::Map::isParameterBlockConstant` | `bool (uint64_t)` | project | modified: one find(), CHECK_EQ vs problem_->IsParameterBlockConstant removed | returns block->fixed() |
| 0x18001CF70 | 914 | `ceres_backend::GravityLocalParameterization::minus` | `static bool (const double* x, const double* x_plus_delta, double* delta)` | project | new | delta = TangentBasis(x)^T (x_plus - x); header inline (GravityParameterBlock slot 7 thunk 0x18001A930) |
| 0x18001D310 | 67 | `ceres_backend::Map::parametersPtr` | `const ParameterBlockCollection* (ResidualBlockId) const` | project | new | null if residual unknown; used by MarginalizationError 0x180079D90 |
| 0x18001D360 | 54 | `ceres_backend::Map::parameterBlockExists` | `bool (uint64_t) const` | project | identical |  |
| 0x18001D3A0 | 250 | `ceres_backend::Map::parameterBlockIdOfResidual` | `uint64_t (ResidualBlockId, size_t parameter_index) const` | project | new | CHECK l.588 "Invalid residual parameter index "; callers 0x180027AE0, 0x18002C100 |
| 0x18001D4A0 | 295 | `ceres_backend::Map::parameterBlockPtr` | `shared_ptr<ParameterBlock> (uint64_t)` | project | modified: single find + CHECK(it != end) l.511 | "parameterBlock with id %x does not exist" |
| 0x18001D5D0 | 880 | `ceres_backend::GravityLocalParameterization::plus` | `static bool (const double* x, const double* delta, double* x_plus_delta)` | project | new | |x|*normalize(x + TangentBasis(x)*delta); header inline (slot 5 thunk 0x18001A960) |
| 0x18001D940 | 1190 | `ceres_backend::Map::printParameterBlockInfo` | `void (uint64_t) const` | project | identical (kErrorToStr is unordered_map) | LOG(INFO) l.94/100/104; only caller 0x180080320 |
| 0x18001DDF0 | 265 | `Eigen::internal::queryCacheSizes_intel_direct` | `void (int& l1, int& l2, int& l3, int max_std_funcs)` | lib:Eigen (Memory.h cpuid leaf 4) | - | called by 0x180018C90 (manage_caching_sizes) |
| 0x18001DF00 | 708 | `Eigen::internal::queryCacheSizes_intel_codes` | `void (int& l1, int& l2, int& l3)` | lib:Eigen (cpuid leaf 2 descriptor table) | - |  |
| 0x18001E1D0 | 740 | `ceres_backend::Map::removeParameterBlock` | `bool (uint64_t)` | project | modified: one find(), iterator reused for RemoveParameterBlock/erase(it) | VLOG(200) l.224 "Removing paramter block with ID " |
| 0x18001E4C0 | 1332 | `ceres_backend::Map::removeResidualBlock` | `bool (ResidualBlockId)` | project | modified: id from collection .first, duplicate-aware multimap erase, no DEBUG_CHECK | VLOG(200) l.382 |
| 0x18001EA00 | 62 | `ceres_backend::Map::residuals` | `ResidualBlockCollection (uint64_t) const` | project | modified: wraps the out-param overload |  |
| 0x18001EA40 | 397 | `ceres_backend::Map::residuals` | `void (uint64_t, ResidualBlockCollection&) const` | project | new | clear() + equal_range copy |
| 0x18001EBD0 | 127 | `Eigen::DenseStorage<double,Dynamic,Dynamic,Dynamic,0>::resize` | `void (Index size, Index rows, Index cols)` | lib:Eigen | - |  |
| 0x18001EC50 | 1179 | `Eigen::internal::general_matrix_matrix_product<Index,double,ColMajor,false,double,ColMajor,false,ColMajor,1>::run` | `void (...)` | lib:Eigen GEMM driver ("incr==1") | - |  |
| 0x18001F0F0 | 4546 | `Eigen::internal::gebp_kernel<double,double,Index,blas_data_mapper<..>,4,4>::operator()` | `void (...)` | lib:Eigen GEBP kernel | - | 4.5 KB |
| 0x1800202C0 | 186 | `ceres_backend::Map::setParameterBlockConstant` | `bool (uint64_t)` | project | modified: single find() | fixed_=true + problem_->SetParameterBlockConstant |
| 0x180020380 | 186 | `ceres_backend::Map::setParameterBlockVariable` | `bool (uint64_t)` | project | modified: single find() | fixed_=false + problem_->SetParameterBlockVariable  [last ceres_map.obj function] |
| 0x180020440 | 717 | `unordered_map<ErrorType,std::string>::insert(first,last)` | `void (const value_type*, const value_type*)` | lib:std::_Hash::insert range (kErrorToStr initializer 0x180001030) | - | start of estimator.obj (probable) |
| 0x180020710 | 97 | `std::pair<const ErrorType,std::string>::~pair (eh helper)` | `void (pair*)` | lib:std | - |  |
| 0x180020780 | 57 | `std::string::~basic_string (eh helper)` | `void (string*)` | lib:std | - |  |
| 0x1800207C0 | 109 | `std::string::_Tidy_deallocate (eh helper)` | `void (string*)` | lib:std | - |  |
| 0x180020830 | 97 | `std::string::~basic_string` | `void (string*)` | lib:std | - |  |
| 0x1800208A0 | 418 | `std::_Hash<..ErrorType..>::_Forced_rehash` | `void (size_t buckets)` | lib:std ("invalid hash bucket count") | - |  |
| 0x180020A50 | 111 | `std::shared_ptr<ceres::LossFunction>::shared_ptr(ceres::HuberLoss*)` | `ctor` | lib:std (_Ref_count<HuberLoss>) | - |  |
| 0x180020AC0 | 40 | `std::shared_ptr<ceres::CostFunction>::shared_ptr(const shared_ptr<Derived>&)` | `ctor` | lib:std converting copy ctor | - |  |
| 0x180020AF0 | 492 | `Eigen::internal::quaternionbase_assign_impl<Matrix3d,3,3>::run` | `void (Quaterniond&, const Matrix3d&)` | lib:Eigen (rotation matrix -> quaternion) | - |  |
| 0x180020CE0 | 358 | `kindr::minimal::operator<<(ostream&, const QuatTransformationTemplate<double>&)` | `ostream& (ostream&, const Transformation&)` | lib:minkindr (prints getTransformationMatrix()) | - |  |
| 0x180020E50 | 832 | `Eigen::operator<<(ostream&, const DenseBase<Matrix4d>&)` | `ostream& (ostream&, const Matrix4d&)` | lib:Eigen (default IOFormat) | - |  |
| 0x180021190 | 917 | `Eigen::operator<<(ostream&, const DenseBase<Matrix<double,9,1>>&)` | `ostream& (ostream&, const Matrix<double,9,1>&)` | lib:Eigen | - | used by printStates 0x18002B9D0 |
| 0x180021530 | 109 | `std::_Fnv1a_append_value<uint64_t>` | `size_t (size_t, const uint64_t&)` | lib:std hash (FNV-1a, 8 bytes) | - |  |
| 0x1800215A0 | 48 | `std::_Copy_memmove<uint64_t*>` | `T* (T*, T*, T*)` | lib:std | - |  |
| 0x1800215D0 | 133 | `std::_Destroy_range<std::string>` | `void (string*, string*)` | lib:std | - |  |
| 0x180021660 | 316 | `std::vector<uint64_t>::_Emplace_reallocate` | `T* (T* where, const T&)` | lib:std | - | (8-byte element, used for BackendId/double* vectors) |
| 0x1800217A0 | 176 | `std::_Tree<PointMap>::_Erase_tree` | `void (alloc&, node*)` | lib:std (node dtor = ~MapPoint, Eigen aligned free) | - |  |
| 0x180021850 | 85 | `std::_Tree<set<uint64_t>>::_Erase_tree` | `void (alloc&, node*)` | lib:std | - |  |
| 0x1800218B0 | 141 | `std::_Tree<map<K, vector<T,aligned_allocator>>>::_Erase_tree` | `void (alloc&, node*)` | lib:std | - | Estimator::unknown_map_10_ |
| 0x180021940 | 85 | `std::_Tree<map/set trivial>::_Erase_tree` | `void (alloc&, node*)` | lib:std | - |  |
| 0x1800219A0 | 512 | `std::_Tree<PointMap>::_Find_hint` | `_Tree_find_hint_result (const_iterator hint, const BackendId&)` | lib:std (emplace_hint) | - |  |
| 0x180021BA0 | 105 | `std::_Tree<PointMap>::_Freenode` | `void (node*)` | lib:std (~MapPoint + free) | - |  |
| 0x180021C10 | 358 | `std::string::_Reallocate_grow_by (push_back)` | `string& (size_t, lambda, char)` | lib:std | - |  |
| 0x180021D80 | 234 | `std::vector<uint32_t>::_Resize_reallocate (vector<bool> storage)` | `void (size_t, const uint32_t&)` | lib:std | - |  |
| 0x180021E70 | 213 | `std::vector<uint64_t>::_Resize_reallocate<_Value_init_tag>` | `void (size_t)` | lib:std | - | constant_extrinsics_ids_.resize |
| 0x180021F50 | 130 | `std::find<set<uint64_t>::const_iterator, uint64_t>` | `iterator (it, it, const uint64_t&)` | lib:std | - | checkAndAddToSet/DeleteFromSet |
| 0x180021FE0 | 111 | `std::make_shared<ceres_backend::Map>()` | `shared_ptr<Map> ()` | lib:std (new 0x590, Map::Map(false)) | - |  |
| 0x180022050 | 141 | `std::make_shared<ceres_backend::PoseError>(const Transformation&, const Matrix<double,6,6>&)` | `shared_ptr<PoseError>` | lib:std (new 0x1B0) | - |  |
| 0x1800220E0 | 1005 | `Eigen::internal::print_matrix<Matrix<double,9,1>>` | `ostream& (ostream&, const Matrix<double,9,1>&, const IOFormat&)` | lib:Eigen | - |  |
| 0x1800224D0 | 1129 | `Eigen::internal::print_matrix<Matrix4d>` | `ostream& (ostream&, const Matrix4d&, const IOFormat&)` | lib:Eigen | - |  |
| 0x180022940 | 425 | `Eigen::internal::redux_impl<scalar_sum_op, squaredNorm of VectorXd>::run` | `double (...)` | lib:Eigen | - |  |
| 0x180022AF0 | 166 | `Eigen Map<Matrix3d> ctor helper (variable_if_dynamic<3> asserts)` | `` | lib:Eigen | - |  |
| 0x180022BA0 | 166 | `Eigen CwiseNullaryOp<..,Matrix<double,6,6>> ctor (variable_if_dynamic<6> asserts)` | `` | lib:Eigen | - | Matrix<double,6,6>::Zero() |
| 0x180022C50 | 144 | `Eigen Matrix3d nullary/Map helper (variable_if_dynamic<3> asserts)` | `` | lib:Eigen | - |  |
| 0x180022CE0 | 52 | `kindr::minimal::QuatTransformationTemplate<double>::QuatTransformationTemplate(const Rotation&, const Position&)` | `ctor` | lib:minkindr | - | caller 0x18015E250 |
| 0x180022D20 | 181 | `std::basic_stringstream<char>::basic_stringstream()` | `ctor (vbase flag)` | lib:std | - |  |
| 0x180022DE0 | 113 | `std::unordered_map<uint64_t,KeypointIdentifier>::unordered_map()` | `ctor` | lib:std | - | MapPoint::observations |
| 0x180022E60 | 138 | `std::vector<BackendId>::vector(const vector&)` | `copy ctor` | lib:std | - |  |
| 0x180022EF0 | 1331 | `Estimator::Estimator` | `(shared_ptr<ceres_backend::Map> map_ptr)` | project | modified: moved map_ptr, Huber(0.5), extra Huber(1.5), Pimax members/defaults | see Types/Constants |
| 0x180023430 | 147 | `Estimator::Estimator` | `()` | project | modified: + reserve(10) of the 3 States vectors | Estimator(make_shared<Map>()) |
| 0x1800234D0 | 298 | `MapPoint::MapPoint(MapPoint&&)` | `implicit move ctor` | lib:compiler-generated (project type) | - | 128-byte MapPoint |
| 0x180023600 | 17 | `Eigen aligned buffer free helper (free(p+8))` | `void (T*)` | lib:Eigen/std (eh helper) | - |  |
| 0x180023620 | 20 | `std buffer free helper (j_j_free(p+8))` | `void (T*)` | lib:std (eh helper) | - |  |
| 0x180023640 | 97 | `std::unordered_map<uint64_t,KeypointIdentifier>::~unordered_map` | `dtor` | lib:std | - |  |
| 0x1800236B0 | 46 | `std::unique_ptr<LossFunction-like>::~unique_ptr (eh helper)` | `dtor` | lib:std | - |  |
| 0x1800236E0 | 13 | `std::vector<std::string>::~vector (eh helper)` | `dtor` | lib:std | - |  |
| 0x1800236F0 | 16 | `std::_Destroy_range<std::string> (eh helper)` | `` | lib:std | - |  |
| 0x180023700 | 130 | `std::list<pair<const uint64_t,KeypointIdentifier>>::_Tidy` | `void ()` | lib:std (weak_ptr release) | - |  |
| 0x180023790 | 39 | `std::map<BackendId,MapPoint>::~map (PointMap)` | `dtor` | lib:std | - |  |
| 0x1800237C0 | 42 | `std::map<K,vector<T,aligned_allocator>>::~map` | `dtor` | lib:std | - |  |
| 0x1800237F0 | 92 | `std::set<uint64_t>::~set` | `dtor` | lib:std | - |  |
| 0x180023850 | 20 | `std::unique_ptr<polymorphic>::~unique_ptr` | `dtor` | lib:std | - |  |
| 0x180023870 | 42 | `std::vector<T, Eigen::aligned_allocator<T>>::_Tidy` | `void ()` | lib:std/Eigen | - |  |
| 0x1800238A0 | 254 | `std::vector<std::vector<T>>::_Tidy` | `void ()` | lib:std | - |  |
| 0x1800239A0 | 1101 | `Estimator::~Estimator` | `()` | project | identical (empty body) | member destruction confirms the layout |
| 0x180023DF0 | 593 | `Eigen::IOFormat::~IOFormat` | `dtor` | lib:Eigen (6 std::string) | - |  |
| 0x180024050 | 977 | `ceres_backend::Map::~Map` | `implicit dtor` | lib:compiler-generated (project type) | - | confirms Map layout |
| 0x180024430 | 87 | `MapPoint::~MapPoint` | `implicit dtor` | lib:compiler-generated | - |  |
| 0x180024490 | 245 | `States::~States` | `implicit dtor` | lib:compiler-generated | - |  |
| 0x180024590 | 7 | `free_0 (CRT free thunk)` | `void (void*)` | lib:crt | - |  |
| 0x1800245A0 | 425 | `States::operator=(States&&)` | `implicit move assignment` | lib:compiler-generated | - | caller 0x180029610 |
| 0x180024750 | 288 | `std::map<std::string,double>::operator[] / try_emplace` | `double& (const std::string&)` | lib:std (MarginalizationTiming::named_timing_) | - |  |
| 0x180024870 | 97 | `kindr::minimal::QuatTransformationTemplate<double>::transform(const Vector3d&)` | `Vector3d (const Vector3d&) const` | lib:minkindr (R*p + t) | - |  |
| 0x1800248E0 | 102 | `std::_Tree_unchecked_const_iterator::operator++` | `` | lib:std | - |  |
| 0x180024950 | 109 | `std::_Vb_const_iterator::operator+= (vector<bool>)` | `` | lib:std | - |  |
| 0x1800249C0 | 196 | `Eigen::CommaInitializer<Vector4d>::operator,(const double&)` | `` | lib:Eigen | - | MapPoint ctor |
| 0x180024A90 | 43 | `std::_Ref_count_obj2<General3DParameterBlock>::_Delete_this` | `void ()` | lib:std | - |  |
| 0x180024AC0 | 43 | `std::_Ref_count_obj2<GravityParameterBlock>::_Delete_this` | `void ()` | lib:std | - |  |
| 0x180024AF0 | 43 | `std::_Ref_count_obj2<GroundPlaneError>::_Delete_this` | `void ()` | lib:std | - |  |
| 0x180024B20 | 43 | `std::_Ref_count_obj2<ImuError>::_Delete_this` | `void ()` | lib:std | - |  |
| 0x180024B50 | 43 | `std::_Ref_count_obj2<Map>::_Delete_this` | `void ()` | lib:std | - |  |
| 0x180024B80 | 43 | `std::_Ref_count_obj2<PoseError>::_Delete_this` | `void ()` | lib:std | - |  |
| 0x180024BB0 | 43 | `std::_Ref_count_obj2<PoseParameterBlock>::_Delete_this` | `void ()` | lib:std | - |  |
| 0x180024BE0 | 43 | `std::_Ref_count_obj2<SpeedAndBiasError>::_Delete_this` | `void ()` | lib:std | - |  |
| 0x180024C10 | 43 | `std::_Ref_count_obj2<SpeedAndBiasParameterBlock>::_Delete_this` | `void ()` | lib:std | - |  |
| 0x180024C40 | 52 | `ceres::HuberLoss / CauchyLoss scalar deleting dtor (COMDAT-folded)` | `void* (unsigned)` | lib:ceres (inline header dtor) | - |  |
| 0x180024C80 | 63 | `ceres_backend::GravityParameterBlock::`scalar deleting destructor'` | `void* (unsigned)` | lib:compiler-generated (EIGEN aligned delete) | - |  |
| 0x180024CC0 | 79 | `ceres_backend::GroundPlaneError::`scalar deleting destructor'` | `void* (unsigned)` | lib:compiler-generated | - |  |
| 0x180024D10 | 52 | `ceres::LossFunction scalar deleting dtor` | `void* (unsigned)` | lib:ceres | - |  |
| 0x180024D50 | 78 | `Frame::T_world_imu` | `Transformation () const` | project | identical (svo_common frame.h inline) | (T_imu_cam()*T_f_w_).inverse() |
| 0x180024DA0 | 136 | `std::vector<uint32_t>::_Change_array` | `void (T*, size_t, size_t)` | lib:std | - |  |
| 0x180024E30 | 191 | `std::vector<uint64_t>::_Clear_and_reserve_geometric` | `void (size_t)` | lib:std | - |  |
| 0x180024EF0 | 47 | `std::_Ref_count<ceres::CauchyLoss/HuberLoss>::_Destroy` | `void ()` | lib:std | - |  |
| 0x180024F20 | 9 | `std::_Ref_count_obj2<GroundPlaneError>::_Destroy` | `void ()` | lib:std | - |  |
| 0x180024F30 | 9 | `std::_Ref_count_obj2<Map>::_Destroy` | `void ()` | lib:std (-> Map::~Map) | - |  |
| 0x180024F40 | 21 | `std::_Ref_count_resource<T*,default_delete<T>>::_Destroy` | `void ()` | lib:std (COMDAT-folded) | - |  |
| 0x180024F60 | 128 | `std::_Tree<PointMap>::erase(iterator)` | `iterator (const_iterator)` | lib:std | - |  |
| 0x180024FE0 | 915 | `std::_Tree_val<..>::_Extract` | `node* (const_iterator)` | lib:std RB-tree erase/rebalance | - |  |
| 0x180025380 | 47 | `std::_Ref_count_resource<MarginalizationError*,default_delete>::_Get_deleter` | `void* (const type_info&)` | lib:std | - |  |
| 0x1800253B0 | 559 | `std::vector<bool>::_Insert_x` | `size_t (const_iterator, size_t)` | lib:std | - |  |
| 0x1800255E0 | 85 | `std::_Tree_val<..>::_Lrotate` | `void (node*)` | lib:std | - |  |
| 0x180025640 | 102 | `std::vector<uint32_t>::_Reallocate_exactly` | `void (size_t)` | lib:std (vector<bool>::reserve) | - |  |
| 0x1800256B0 | 107 | `std::vector<uint64_t>::_Reallocate_exactly` | `void (size_t)` | lib:std (reserve) | - |  |
| 0x180025720 | 89 | `std::_Tree_val<..>::_Rrotate` | `void (node*)` | lib:std | - |  |
| 0x180025780 | 107 | `std::vector<std::string>::_Tidy` | `void ()` | lib:std | - |  |
| 0x1800257F0 | 17 | `std::vector<bool>::_Xlen` | `[[noreturn]] void ()` | lib:std ("vector<bool> too long") | - |  |
| 0x180025810 | 277 | `Estimator::addCameraBundle` | `void (const CameraBundlePtr& camera_rig)` | project | modified: no extrinsics params / CHECKs / loop | camera_rig_ = rig; constant_extrinsics_ids_.resize(numCameras) |
| 0x180025930 | 688 | `Estimator::addGroundPlaneError` | `void ()` | project | new | GroundPlaneError between two NFrames with HuberLoss(1.5) |
| 0x180025BE0 | 154 | `Estimator::addImu` | `void (const ImuParameters&)` | project | modified: single IMU, plain copy | imu_parameters_ = p (0x80 bytes) |
| 0x180025C80 | 980 | `Estimator::addLandmark` | `bool (const PointPtr& landmark)` | project | modified: General3DParameterBlock/Trivial, float pos, no set_fixed | + landmarks_map_.emplace_hint, in_ba_graph_=true |
| 0x180026060 | 156 | `States::addState` | `void (BackendId id, bool keyframe, double timestamp)` | project | modified: DEBUG_CHECK removed | estimator.h inline |
| 0x180026100 | 4972 | `Estimator::addStates` | `bool (const FrameBundleConstPtr&, const ImuMeasurements&, const double& t, Vector3f& gravity_out, bool, bool, const Vector3d& gravity_init, bool)` | project | modified (heavily, see text) | pose/speed&bias/gravity/extrinsics blocks, priors, ImuError; LOGE "numUsedImuMeasurements less 1" |
| 0x180027470 | 1448 | `Estimator::addVelocityPrior` | `bool (BackendId, const Vector3d& v, double sigma, const Vector3d& bias_acc, const Vector3d& bias_gyr)` | project | modified: biases + 1e6 bias information | SpeedAndBiasError prior on the ImuStates block |
| 0x180027A20 | 69 | `isFinite` | `bool (const Eigen::Vector3d&)` | project | new | std::isfinite on 3 components (_dclass<=0) |
| 0x180027A70 | 112 | `std::allocator<uint32_t>::allocate` | `T* (size_t)` | lib:std | - |  |

Namespaces: `pimax::totem::ceres_backend` (RTTI names of ErrorInterface, Map, all blocks/errors) and, assumed,
`pimax::totem` for `Estimator`, `States`, `MapPoint`, `BackendId`.

## Detailed notes per project function

### Map (ceres_map.cpp) — glog line numbers reproduced with `#line`
| line | statement |
|---|---|
| 94 / 100 / 104 | `printParameterBlockInfo` LOG(INFO) x3 |
| 164 | `VLOG(200) << "Adding parameter block with parameterization " << p << " and id " << BackendId(id)` |
| 169 | `LOG(ERROR) << "Parameter block with id " << BackendId(id) << " exists already!"` |
| 201 | `LOG(ERROR) << "Unknown parameterization!"` |
| 224 | `VLOG(200) << "Removing paramter block with ID " << BackendId(id)` (typo "paramter" kept) |
| 268 | `VLOG(200) << s.str()` (stringstream built under `if (FLAGS_v >= 200)`) |
| 382 | `VLOG(200) << "Removing residual block with ID " << residual_block_id` (pointer value) |
| 511 | `CHECK(it != id_to_parameter_block_map_.end()) << "parameterBlock with id " << BackendId(id) << " does not exist"` |
| 588 | `CHECK(parameters != nullptr && parameter_index < parameters->size()) << "Invalid residual parameter index " << parameter_index` |

* `addParameterBlock` (0x18001B4D0): switch on parameterization 0/1/2 → `AddParameterBlock(values, size, &lp)` +
  `setLocalParameterizationPtr(&lp)` (vtable slot 9, +0x48); 3 → `AddParameterBlock(values, size)`; other →
  LOG(ERROR) + false. Arguments evaluated right-to-left (dimension() slot 3 before parameters() slot 2). The block
  is taken by **const reference**: the callee never releases it, callers (addStates/addLandmark) build a
  converted temporary and destroy it after the call (contrast: both `addResidualBlock` overloads release their
  by-value params in the callee).
* `addResidualBlock(vector)` (0x18001B840): `reserve(n)` on both temporaries, range-for building
  `parameter_blocks` (push_back) and `parameter_block_collection` (emplace_back(id, ptr)), ceres AddResidualBlock,
  VLOG block (const-ref range-for, no shared_ptr copies), `dynamic_pointer_cast<ErrorInterface>` (no
  DEBUG_CHECK), insert spec map, insert collection map (return 0 on failure), insert multimap per parameter,
  **`new_residual_block_ids_.push_back(return_id)` (Map+0x550)**.
* `removeResidualBlock` (0x18001E4C0): see draft; for each collection index i, counts occurrences k of
  `collection[i].first` and whether it occurred at j<i; if not seen before, walks `equal_range` and erases at most
  k entries whose spec id matches. Then `erase(it)` of the collection map and `erase(key)` of the spec map
  (0x18001CBF0). Upstream erased all matches per collection entry (equivalent unless a block appears twice).
* `removeParameterBlock` (0x18001E1D0): single `find`; iterator reused (`it->second->parameters()`, `erase(it)`).
* `setParameterBlockConstant/Variable`, `isParameterBlockConstant`: single `find` (upstream:
  `parameterBlockExists()` + `find()`); `isParameterBlockConstant` lost the `CHECK_EQ` against ceres.
* `parameterBlockPtr` (0x18001D4A0): `find` + CHECK on the iterator; returns `it->second`.
* `residuals(id)` / `residuals(id, out)`: out-param overload does `clear()` + equal_range copy (no early find).
  `printParameterBlockInfo` inlines `residuals(id)` (NRVO flag at rsp+0x38 = 1).
* `parametersPtr` / `parameterBlockIdOfResidual`: Pimax additions used by MarginalizationError (0x180079D90,
  own CHECK "Residual block not found") and by 0x180027AE0 / 0x18002C100.

### GravityLocalParameterization statics (header inline, emitted in ceres_map.obj)
* `plus` 0x18001D5D0: `g_plus = (g + B*dg).normalized() * g.norm()` with `MatrixXd B(3,2); B = TangentBasis(g)`.
* `minus` 0x18001CF70: `dg = B.transpose() * (g_plus - g)`.
* Thunks: GravityParameterBlock vtable slot 5 (`plus`) = 0x18001A960, slot 7 (`minus`) = 0x18001A930
  (they ignore `this`).

### Estimator (start of estimator.obj)
* `Estimator(shared_ptr<Map>)` 0x180022EF0: `map_ptr_(std::move(map_ptr))` (source shared_ptr is zeroed, then the
  by-value param is destroyed), CauchyLoss(1.0) {b=1,c=1}, HuberLoss(0.5) {a=0.5,b=0.25},
  HuberLoss(1.5) {a=1.5,b=2.25}, marginalization_residual_id_=0, all Pimax members default-initialised (see Types).
* `Estimator()` 0x180023430: delegating to `Estimator(std::make_shared<Map>())`, then `states_.ids.reserve(10)`,
  `states_.is_keyframe.reserve(10)` (1 word), `states_.timestamps.reserve(10)`.
* `addStates` 0x180026100 — parameters from the caller 0x180011170 (ceres_backend_interface.cpp:509 "Failed to add
  state. Will drop frames."): `frame_bundle` (FrameBundlePtr converted to a const temp), `imu_measurements =
  FrameBundle+0x18`, `timestamp = frame->timestamp_ns(+0xF0 of frame 0) * 1e-9`, `gravity_out` (Vector3f written
  back to FrameBundle+0xD8), `fix_pose_from_frame = backend_interface+0x4D9`, `skip_imu_propagation`,
  `gravity_init` (const Vector3d&), `states_from_frame`. Parameter names are mine (TODO(verify)). Flow:
  1. `fix_pose_from_frame`: T_WS = frames_.at(0)->T_world_imu() (inlined), `T_WS.getRotation().normalize()`,
     speed_and_bias = [FB+0x90 velocity, FB+0xA8 gyr bias, FB+0xC0 acc bias], last_timestamp =
     timestamps.back() if any, gravity_out = gravity_init. **No ImuError is created.**
  2. else: last_timestamp = timestamps.back() (no emptiness check), T_WS / speed&bias from the previous blocks,
     `imu_error = make_shared<ImuError>(meas, imu_parameters_, last_timestamp, timestamp, speed_and_bias)`,
     gravity = GravityParameterBlock(-2)->estimate(), gravity_out = gravity; if `skip_imu_propagation`
     velocity:=0 else `imu_error->propagation(meas, params, T_WS, sab, t0, t1, gravity, nullptr, nullptr)`;
     `< 1` → `LOGE("numUsedImuMeasurements less 1\n")`, return false. Then if `states_from_frame`: T_WS =
     frames_[0]->T_world_imu(), velocity=FB+0x90, acc bias=FB+0xC0, gyr bias=FB+0xA8 (that order).
  3. first state: GravityParameterBlock(gravity, -2) with `g=(0,0,imu.g)` unless `|gravity_init|>0`, added with
     `Map::Gravity` (result ignored).
  4. Pose block (Pose6d) → false on failure; if `fix_pose_from_frame` setParameterBlockConstant(pose);
     `states_.addState(id, states_.ids.empty(), timestamp)` (first state = keyframe).
  5. SpeedAndBias block (Trivial) → false on failure.
  6. size()==1: PoseError prior diag(1e8,1e8,1e8,0,0,1e8), registerFixedFrame(pose id), SpeedAndBiasError(sab,
     1.0, sigma_bg², sigma_ba²) on the ImuStates block, **then that block is set constant**. Else ImuError with
     5 blocks: last pose, last sab, pose, sab, gravity(-2).
  7. size()==1: for i < camera_rig_->getNumCameras(): PoseParameterBlock(get_T_C_B(i) **without inverse**,
     changeIdType(id, Extrinsics, i)) (Pose6d; false on failure) + setParameterBlockConstant; then
     `constant_extrinsics_ids_ = cur_bundle_extrinsics_ids`.
* `addLandmark` 0x180025C80: id = createLandmarkId(point->id_), `make_shared<General3DParameterBlock>(pos.cast<double>(),
  id, true)` added with Trivial (→ false), `landmarks_map_.emplace_hint(end(), id, MapPoint(point))`,
  `point->in_ba_graph_ = true` (Point+0x80). No set_fixed parameter.
* `addGroundPlaneError` 0x180025930: `removeGroundPlaneErrors()` (0x18002C080: removeResidualBlock for every id in
  +0x2B0, then clear), and if `ground_plane_.valid` and both NFrame blocks exist and are not constant, adds
  `GroundPlaneError(normal_0, normal_1, sigma)` with `ground_plane_loss_function_ptr_` (HuberLoss 1.5) on
  (pose(bundle_id_0), pose(bundle_id_1)); non-null ids are pushed to +0x2B0.
* `addVelocityPrior` 0x180027470: caller 0x180011700 ("IMU determined stationary, adding prior at time %f",
  "Failed to add a zero velocity prior!") passes (id, zero velocity, sigma, FB+0xC0, FB+0xA8). sab =
  [v, bias_gyr, bias_acc]; info = I; topLeft3x3 *= 1/sigma²; **bottomRightCorner<6,6> *= 1e6**.

## Types

### `ceres_backend::Map` — sizeof 0x580 (operator new 0x590 in make_shared, 0x180021FE0)
| offset | type | name | evidence |
|---|---|---|---|
| 0x000 | ceres::Solver::Options | options | ~Options (0x18001A0B0) on +0 in ~Map (sure) |
| 0x1F0 | ceres::Solver::Summary | summary | ~Summary (0x18001A2B0) on +0x1F0 (sure) |
| 0x3E8 | bool | ? (`flag_3e8_`) | compared/stored in 0x18001A980 (c01) |
| 0x3F0 | uint64_t | residual_counter_ | by elimination (not touched here) |
| 0x3F8 | std::shared_ptr<ceres::Problem> | problem_ | all ceres calls (sure) |
| 0x408 | unordered_map<uint64_t, shared_ptr<ParameterBlock>> | id_to_parameter_block_map_ | node: key+0x10, ptr+0x18, ctrl+0x20 (sure) |
| 0x448 | unordered_map<ResidualBlockId, ResidualBlockSpec> | residual_block_id_to_residual_block_spec_map_ | node: key+0x10, spec+0x18 (sure) |
| 0x488 | unordered_multimap<uint64_t, ResidualBlockSpec> | id_to_residual_block_multimap_ | (sure) |
| 0x4C8 | unordered_map<ResidualBlockId, ParameterBlockCollection> | residual_block_id_to_parameter_block_collection_map_ | vector at node+0x18 (sure) |
| 0x508 | HomogeneousPointLocalParameterization (24 B) | homogeneous_point_local_parameterization_ | ctor vtables, param 0 (sure) |
| 0x520 | PoseLocalParameterization (24 B) | pose_local_parameterization_ | param 1 (sure) |
| 0x538 | GravityLocalParameterization (24 B) | gravity_local_parameterization_ | param 2 (sure) |
| 0x550 | std::vector<ResidualBlockId> | new_residual_block_ids_ (name?) | push in addResidualBlock (sure) |
| 0x568 | std::vector<uint64_t> | new_parameter_block_ids_ (name?) | only ~Map / 0x18001A980 |

MSVC `unordered_map` = 0x40 bytes: +0 max_load_factor float, +8 list head, +0x10 size, +0x18 bucket vector,
+0x30 mask, +0x38 maxidx. All hashes are FNV-1a over the 8 key bytes.
Upstream differences: Gravity param + member, 2 vectors, +0x3E8 bool, local parameterizations have a bool at +16.

`ResidualBlockSpec` 32 B {id +0, loss +8, shared_ptr<ErrorInterface> +0x10}; `ParameterBlockSpec` = pair<uint64_t,
shared_ptr<ParameterBlock>> 24 B. Both identical to upstream.

### Local parameterizations (24 B each)
+0 vptr ceres::LocalParameterization, +8 vptr LocalParamizationAdditionalInterfaces, +0x10 bool (Map ctor flag;
also toggled through `dynamic_cast<LocalParamizationAdditionalInterfaces*>` +8 in 0x18001A980).

### `ceres_backend::ParameterBlock` (base; unchanged)
| offset | field |
|---|---|
| 0 | vptr |
| 8 | uint64_t id_ |
| 0x10 | bool fixed_ |
| 0x18 | const ceres::LocalParameterization* local_parameterization_ptr_ |

vtable (MSVC puts overloads in reverse order): 0 dtor, 1 `parameters() const`, 2 `parameters()`, 3 `dimension()`,
4 `minimalDimension()`, 5 `plus`, 6 `plusJacobian`, 7 `minus`, 8 `liftJacobian`, 9 `setLocalParameterizationPtr`,
10 `localParameterizationPtr`, 11 `typeInfo() -> std::string`, 12 `setEstimate(const T&)` (Pimax/subclass),
13 `estimate()` (SpeedAndBiasParameterBlock only, returns `const SpeedAndBias&` = this+0x20).
Sizes from make_shared: GravityParameterBlock 0x38 (estimate Vector3d at +0x20; dimension() = 3 (0x18001A910),
minimalDimension() = 2 (COMDAT-folded with boost codecvt_null::do_encoding 0x180014A10 `mov eax,2`);
parameters() = `lea rax,[rcx+20h]` 0x18002B7F0), PoseParameterBlock 0x58, SpeedAndBiasParameterBlock 0x68,
General3DParameterBlock 0x40.

### `ceres_backend::ErrorInterface`
vtable: 0 dtor, 1 residualDim, 2 parameterBlocks, 3 parameterBlockDim, 4 EvaluateWithMinimalJacobians,
5 `typeInfo() -> ErrorType` (+0x28, used here). Pimax: bool member at +8 (toggled by 0x18001A980).
Sizes: ImuError 0x18B0, PoseError 0x1A0, SpeedAndBiasError 0x590, GroundPlaneError 0x70 (SizedCostFunction<3,7,7>,
ErrorInterface subobject at +0x28).

### `kErrorToStr` — `std::unordered_map<ErrorType, std::string>` (upstream: std::map) at 0x18047D9F0
head 0x18047D9F8, buckets 0x18047DA08, mask 0x18047DA20. Initialised by 0x180001030 with the 7 upstream
names + `"GroundPlaneError"` (ErrorType 7, presumably). `.at()` throws "invalid unordered_map<K, T> key".

### `Estimator` (partial, size ≥ 0x2FC) — reconcile with c03
| offset | type | name | evidence |
|---|---|---|---|
| 0x000 | std::map<?,?> (16-byte trivially destructible value) | ? | ctor head node 0x30, dtor 0x180021940 |
| 0x010 | std::map<?, vector<T,aligned_allocator>> | ? | head node 0x40, dtor 0x1800218B0 |
| 0x020 | bool | is_reinit_ | zeroed |
| 0x028 | Matrix<double,9,1> | reinit_speed_bias_ | uninitialised |
| 0x070 | Transformation (64 B) | reinit_T_WS_ | identity |
| 0x0B0 | double | reinit_timestamp_start_ ? | uninitialised |
| 0x0B8 | 8 bytes | ? | uninitialised |
| 0x0C0/0x0D8/0x0F0 | std::vector<8-byte> x3 | ? | zeroed, freed in dtor |
| 0x108 | PointMap | landmarks_map_ | addLandmark (sure), node 0xB0 malloc'ed |
| 0x118 | CameraBundlePtr | camera_rig_ | addCameraBundle/addStates (sure) |
| 0x128 | std::vector<BackendId> | constant_extrinsics_ids_ | (sure) |
| 0x140 | States (0x50) | states_ | ids 0x140, is_keyframe 0x158, timestamps 0x178 (sure) |
| 0x190 | std::shared_ptr<Map> | map_ptr_ | (sure) |
| 0x1A0 | ExtrinsicsEstimationParametersVec | extrinsics_estimation_parameters_ | free()'d vector |
| 0x1B8 | ImuParameters (0x80) | imu_parameters_ | (sure) |
| 0x238 | shared_ptr<LossFunction> | cauchy_loss_function_ptr_ | CauchyLoss(1) |
| 0x248 | shared_ptr<LossFunction> | huber_loss_function_ptr_ | HuberLoss(0.5) |
| 0x258 | shared_ptr<LossFunction> | ground_plane_loss_function_ptr_ (name?) | HuberLoss(1.5), addGroundPlaneError (sure) |
| 0x268 | GroundPlaneConstraint (0x48) | ground_plane_ | (sure) |
| 0x2B0 | std::vector<ResidualBlockId> | ground_plane_residual_ids_ | (sure) |
| 0x2C8 | shared_ptr<MarginalizationError> | marginalization_error_ptr_ | dtor order |
| 0x2D8 | ResidualBlockId | marginalization_residual_id_ | = 0 |
| 0x2E0 | std::set<uint64_t> | fixed_frame_parameter_ids_ | registerFixedFrame 0x18002BFB0 inserts (no CHECK) |
| 0x2F0 | uint64_t | gravity_parameter_block_id_ (name?) | = 0xFFFFFFFFFFFFFFFE, used for the gravity block (sure) |
| 0x2F8 | int | ? | = 0 |

Not present vs upstream: `ceres_callback_` unique_ptr, `fixed_landmark_parameter_ids_` set, `imu_parameters_` vector
(now a single struct), `estimate_temporal_extrinsics_`, `min_num_3d_points_for_fixation_` (not default-set).

### `GroundPlaneConstraint` (name mine), 0x48 bytes at Estimator+0x268
+0 bool valid=false, +4 int bundle_id_0=-1, +8 int bundle_id_1=-1, +0x10 Vector3d normal_0=(0,0,1),
+0x28 Vector3d normal_1=(0,0,1), +0x40 double sigma=0.15. (Copied verbatim by a setter in c03; reset by
0x1800298A0.)

### `States` 0x50: ids vector +0, `std::vector<bool>` +0x18 (32 B), timestamps +0x38.
### `MapPoint` 0x80 / `PointMap` node 0xB0 — see `estimator_types_c02.h` (observations is an
`unordered_map<uint64_t, KeypointIdentifier-like 32 B with weak_ptr at +8>`, extra double 10.0 at +0x78).
### `ImuParameters` 0x80 — see Constants.
### FrameBundle (partial): +0 `std::vector<FramePtr> frames_`, +0x18 ImuMeasurements, +0x90 Vector3d velocity,
+0xA8 Vector3d gyro bias, +0xC0 Vector3d acc bias, +0xD8 Vector3f gravity (written by the caller from
`gravity_out`), +0xFC int bundle_id.
### Frame (partial): +0x40 `T_f_w_`, +0x100 `T_body_cam_` (T_imu_cam), +0xF0 int64 timestamp ns (caller).
### Point (partial): +0 int id_, +4 float pos_[3] (pos() returns float vector), +0x80 bool in_ba_graph_.
### NCamera: +0x18 `T_C_B_` vector (stride 64), +0x30 cameras vector (16-byte elements) → getNumCameras().

## External interfaces (called from this chunk)
| address | meaning |
|---|---|
| 0x180354E20 / 0x180354E50 / 0x180354E80 | google::LogMessage(file,line) / (file,line,severity) / LogMessageFatal(file,line) |
| 0x1803551B0 / 0x180355280 | ~LogMessage / ~LogMessageFatal |
| 0x18035AC70 | LogMessage::stream() |
| 0x18048EA8C | `FLAGS_v` (glog, `env_8`; VLOG_IS_ON = FLAGS_v >= n on MSVC) |
| 0x180006290 | operator<<(ostream&, const char*) |
| 0x180017630 | ostream insert (string data,size) = operator<<(ostream&, const std::string&) |
| 0x180018500 | std::endl<char> |
| 0x180016530 / 0x180009630 | std::stringbuf::str() / ~stringbuf |
| 0x180018920 | `_Hash::find` helper (returns node) shared by all uint64/pointer-keyed maps |
| 0x1800181E0 / 0x180017980 / 0x180017C30 / 0x180017F50 | emplace into id_to_parameter_block_map_ / spec map / collection map / multimap |
| 0x18001B1E0 / 0x18001B250 / 0x18001B2E0 | vector<double*>::reserve / vector<ParameterBlockSpec>::reserve / vector<shared_ptr<ParameterBlock>>::reserve |
| 0x180017280 / 0x180017380 / 0x180016F10 / 0x180016D70 / 0x180017080 | `_Emplace_reallocate` for vector<double*> / ParameterBlockCollection / vector<shared_ptr<PB>> / vector<uint64_t> / ResidualBlockCollection |
| 0x180016CF0 / 0x180018BA0 / 0x18001B440 | `_Destroy_range<ResidualBlockSpec>` / ParameterBlockCollection copy ctor / ~pair<id, collection> |
| 0x1801B9970 | ceres::Problem::AddParameterBlock(double*, int) |
| 0x1801B9980 | ceres::Problem::AddParameterBlock(double*, int, LocalParameterization*) |
| 0x1801B9990 | ceres::Problem::AddResidualBlock(CostFunction*, LossFunction*, const std::vector<double*>&) |
| 0x1801B99C0 / 0x1801B99D0 | Problem::RemoveParameterBlock(const double*) / RemoveResidualBlock(ResidualBlockId) |
| 0x1801B99E0 / 0x1801B99F0 | Problem::SetParameterBlockConstant(const double*) / SetParameterBlockVariable(double*) |
| 0x1801B9A00 / 0x1801B77D0 / 0x1801B8DC0 | ~LossFunction / ~CostFunction / ~LocalParameterization (vtable reset) |
| 0x1800178F0 | Eigen aligned_malloc |
| 0x18001ACE0 | `TangentBasis(const Vector3d&) -> MatrixXd(3,2)` (c01) |
| 0x180008720 / 0x180008630 / 0x1800074E0 | Eigen Block/segment helper, Matrix<6,6> nullary init, setConstant(n, v) |
| 0x180009B80 / 0x180013040 / 0x1800089C0 | minkindr Transformation operator* / inverse / RotationQuaternion(Quaterniond) ctor with norm check |
| 0x18000F970 / 0x180009790 | shared_ptr `_Decref` / ~shared_ptr |
| 0x180018E90 | Map::Map(bool) (c01) |
| 0x1800398A0 | ImuError::ImuError(meas, params, t0, t1, speed_and_bias) |
| 0x1800427F0 | ImuError::propagation(meas, params, T_WS&, sab&, t0, t1, gravity, cov*, jac*) — non-static member (rcx = ImuError) |
| 0x18008D960 / 0x18008DA10 | PoseParameterBlock(T, id) / PoseParameterBlock::estimate() (by value) |
| 0x18008FE60 | SpeedAndBiasParameterBlock(sab, id) |
| 0x18008E980 | SpeedAndBiasError(sab, double timeConstant=1.0 (0x1803ADDD0), var_bg, var_ba) |
| 0x18008E8F0 | SpeedAndBiasError(sab, const Matrix<double,9,9>& information) |
| 0x18008BAC0 | PoseError(T, const Matrix<double,6,6>& information) |
| 0x18002CFB0 | General3DParameterBlock(const Vector3d&, uint64_t id, bool) |
| 0x18002D110 | GroundPlaneError(const Vector3d& n0, const Vector3d& n1, double sigma) |
| 0x18002BFB0 | Estimator::registerFixedFrame(uint64_t) (set insert, no CHECK) |
| 0x18002C080 | Estimator::removeGroundPlaneErrors() |
| 0x18002BE30 | vector<bool>::push_back |
| 0x1801B41E0 | vk::cameras::NCamera::get_T_C_B(size_t) (out-of-line, no CHECK) |
| 0x18000C2C0 | LOGE (logger 0x18046A000) |
| 0x180019E70 | ~vector<BackendId> |
| 0x18000FE80 / 0x180011410 / 0x18000F840 | _Tree insert-at / tree head alloc / hash bucket vector assign |

Callers into this chunk (for the coordinator): Map::addParameterBlock ← 0x180025C80, 0x180026100, 0x18018D770;
addResidualBlock(vector) ← 0x180027AE0, 0x18018D770; addResidualBlock(x0..) ← 0x180010C20, 0x18002C7B0, ...;
parameterBlockPtr ← 0x180010C20, 0x180027AE0, 0x180029B80, 0x180029F90, 0x18002A6D0, ...; printParameterBlockInfo ←
0x180080320; parametersPtr ← 0x180079D90 (MarginalizationError); parameterBlockIdOfResidual ← 0x180027AE0,
0x18002C100; Estimator::addStates ← 0x180011170; addLandmark ← 0x180010440; addVelocityPrior ← 0x180011700;
addGroundPlaneError ← 0x18002A6D0; Estimator() ← 0x180008CD0; addCameraBundle ← 0x180008CD0; addImu ← 0x180015A80.

## Constants / config defaults
* Map: parameterization enum {HomogeneousPoint 0, Pose6d 1, Gravity 2, Trivial 3}; VLOG level 200.
* Estimator ctor: CauchyLoss(1.0) {b_=1.0, c_=1.0}; HuberLoss(0.5) {a_=0.5, b_=0.25}; HuberLoss(1.5) {a_=1.5,
  b_=2.25}; gravity block id = (uint64)-2; ground plane: valid=false, ids -1/-1, normals (0,0,1), sigma 0.15.
* ImuParameters defaults (Estimator+0x1B8): a_max 150.0, g_max **35.0**, g **9.80667** (0x40239D03D9A95422),
  a0 = 0, extra Vector3d(+0x60) = 0, rate **1000.0**; sigma_* left uninitialised.
* addStates: pose prior information diag 1e8 at (0,0),(1,1),(2,2),(5,5) (0x4197D78400000000); SpeedAndBiasError
  time constant 1.0; default gravity (0,0,imu.g).
* addVelocityPrior: bias information 1e6 (0x1803AF300), speed information 1/sigma².
* MapPoint extra field default 10.0; States reserve 10.

## Quirks / bugs to preserve
1. `addStates`: extrinsics blocks use `get_T_C_B(i)` **without** `.inverse()` (upstream inverts) and are always
   constant; loop bound is the camera count.
2. `addStates`: with `fix_pose_from_frame` on a non-first state, `imu_error` stays null and is still passed to
   `addResidualBlock` (ceres would then receive a null cost function).
3. `addStates`: `states_.timestamps.back()` is read without an emptiness check in the non-fixed path.
4. `addStates`: the gravity `addParameterBlock` result is ignored; the first speed&bias block is set constant.
5. `addVelocityPrior`: bias information 1e6 on bottomRightCorner (upstream: 0.0 on bottomLeftCorner).
6. ImuParameters: g = 9.80667 (not 9.80665), g_max 35, rate 1000, sigma defaults missing.
7. `parameterBlockIdOfResidual`: on CHECK failure the binary dereferences the null collection pointer before
   `~LogMessageFatal` → access violation instead of a FATAL log.
8. `removeResidualBlock`: duplicate-aware erase (see above); parameter id from `collection[i].first`.
9. Log typo "Removing paramter block with ID " (upstream typo, kept).
10. `createNFrameId` has no `CHECK_GE(bundle_id, 0)` in Pimax (every inline use is a bare shift).
11. Build: right operands of `<<` chains are evaluated before the LogMessage/stringstream insertions
    (typeInfo()/kErrorToStr.at() before `LOG(INFO)` ctor in 0x18001D940 and before writing "Adding residual
    block: " in 0x18001B840) → C++14 (unsequenced) evaluation, i.e. MSVC default `/std:c++14`, not `/std:c++17`.
    `static_pointer_cast` is used on rvalue shared_ptrs (moved, no refcount increment) in addStates.

## Open questions / TODO(verify)
* Real names of: `parametersPtr`, `parameterBlockIdOfResidual`, Map vectors at +0x550/+0x568 and bool +0x3E8,
  `addStates` bool parameters, `ground_plane_*` members, `gravity_parameter_block_id_`, Estimator +0x000/+0x010
  maps, +0x0C0/+0x0D8/+0x0F0 vectors, +0x2F8 int, MapPoint +0x78 double, ImuParameters +0x60 vector.
* `addVelocityPrior` speed&bias fill: three inline 24-byte copies; head/segment/tail form assumed.
* `MapPoint::observations` key/value types.
* Exact meaning of the Map(bool) flag (c01) and the General3DParameterBlock bool argument (`true` here).
* Whether `addCameraBundle`/`addImu` keep these names.

## Line counts
draft/c02_ceres_map/ceres_backend: ceres_map.cpp 499, ceres_map.h 212, estimator_c02.cpp ~380,
estimator.h 121, estimator_types_c02.h 112, gravity_local_parameterization.h 73; this file ~450.
Project functions reconstructed: 18 (16 Map + 2 GravityLocalParameterization statics) + 12 Estimator-side (135 functions in range, the
rest are std/Eigen/minkindr/ceres/compiler-generated instantiations).
