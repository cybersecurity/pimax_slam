# c00_globals — [0x180001000, 0x180010440)

Chunk contents:

* **0x180001000–0x180006060 `.text$di`**: 457 dynamic initializers (the complete `.CRT$XCU` table
  minus the CRT's own entry). Every one is listed below, in CRT-table order, with its object file.
* **0x180006060–0x180010440 `.text$mn`**: the first 128 functions of **object #1
  (ceres_backend/ceres_backend_interface.obj)**. That object is the first one linked, so most of
  them are COMDATs it was the first to use: STL/Eigen/minkindr helpers, the inline project logger, and
  the whole header-only `ReprojectionError` class. Real code of `ceres_backend_interface.cpp` starts at
  0x180008CD0 (the CeresBackendInterface ctor) and continues past 0x180010440 into the next chunk.

Deliverables: this file plus `draft/c00_globals/` (13 files, about 1100 lines):

| file | content |
|---|---|
| `common/logger.h`, `common/logger.cpp` | Logger (inline members), `g_logger`, `kLogTag` |
| `ceres_backend/error_interface.{hpp,cpp}` | `ErrorType` (+kGroundPlaneError), `kErrorToStr`, `ErrorInterface` (+flag at +8) |
| `ceres_backend/reprojection_error_base.hpp`, `reprojection_error.hpp`, `reprojection_error_impl.hpp` | full Pimax `ReprojectionError` (ctor, dtor, Evaluate, EvaluateWithMinimalJacobians, EvaluateMinimal, setInformation) |
| `ceres_backend/ceres_backend_interface_ctor.cpp` | `CeresBackendInterface` ctor/dtor, layout and option structs |
| `ceres_backend/estimator_globals.cpp` | `MarginalizationTiming::names_` |
| `frontend/frame_processor_base_globals.cpp` | `g_permon`, `kStageName`, `kTrackingQualityName`, `kUpdateResultName` |
| `frontend/imu_processor_globals.cpp` | `imu_temporal_status_names_` |
| `loop_closing/loop_closing_globals.cpp` | `kStrToScaleRetMap`, `kStrToGlobalMapType`, plat-map version "1.0.0" |
| `other_objects_globals.inc` | const-initialised globals of frame_processor / headset API / mesher, for the owning chunks to merge |

---------------------------------------------------------------------------------------------------

## 1. How the initializer list maps to object files (method, for the coordinator)

* The `.CRT$XCU` pointer table is at **0x1803A8FE8..0x1803A9E28** (`__xc_a` = 0x1803A8FE0). Entry #0
  (0x180006050) sits in `.CRT$XCC` (STL `locale0_implib` `_Fac_tidy_reg`, `init_seg(compiler)`). The
  last entry, #461 = `__scrt_initialize_thread_safe_statics`, comes after `__xc_z`. **The table order is
  exactly the object link order** (object-major). Inside one object it follows the source definition
  order. In the code, `.text$di` is also object-major, but **inside one object the code order differs
  from the table order**. COMDAT initializers (template static members such as boost singletons) are
  placed with the first object that instantiates them.
* **TU marker:** every object that includes `<Eigen/Core>` contains exactly one 15-byte initializer
  `movzx eax, word [X]; mov word [X+4], ax; ret`. This is Eigen 3.4.0 `IndexedViewHelper.h:181`
  `static const end_t end = Eigen::lastp1;` (namespace `Eigen::placeholders`, deprecated aliases). It is
  a dynamic copy of a 2-byte empty-struct `AddExpr`, and it **pins Eigen to 3.4.x**, because 3.3 has no
  `lastp1`. There are **182 markers**: 70 project, vikit, DBoW2 or early Ceres objects (table #1..#186),
  then 112 Ceres objects (#188..#299).
* A non-marker entry *e* belongs to one of the TUs bracketed by markers, `{a, a+1}`, in **both**
  orders (table order and `.text$di` code order). Intersecting the two brackets resolves most entries
  (column "TU"). An object that does **not** include Eigen has no marker, so it can only be seen
  through its own initializers. Such objects include tinyxml2, DBoW2, the
  portable_binary_archive sources, glog, gflags, boost, Pangolin and probably `common/logger.cpp`.
  Objects without any initializer are invisible here.
* `.bss` placement is object-major too, but a TU's own variables can sit before or after its marker,
  so `.bss` gives ±1-TU hints only. Those hints were used for the medium-confidence names below.

### 1.1 Condensed result (project and in-tree third party, link order)

H = high, M = medium, L = low/guess. "TUn" = n-th Eigen-including object.

| TU | table # | object | evidence | conf |
|---|---|---|---|---|
| 1 | 1 | ceres_backend/ceres_backend_interface.cpp | first `.text$mn` object (0x180006060..), CeresBackendInterface ctor 0x180008CD0, `__FILE__` refs from 0x180010440 | H |
| 2 | 2 | ceres_backend/ceres_map.cpp | `__FILE__` order, alphabetical | M |
| 3 | 3, 4 | ceres_backend/error_interface.cpp | `kErrorToStr` | H |
| 4 | 5, 6 | ceres_backend/estimator.cpp | `MarginalizationTiming::names_` | H |
| 5..18 | 7..20 | 14 more ceres_backend (+ maybe common) objects. In code order: General3DParameterBlock (0x18002D030), GroundPlaneError (0x18002D338..), HomogeneousPointLocalParameterization (0x18002DF00), so3ex/"Format-Warning"/"mlog ensure" area (0x18002E0E0..0x180037490), ImuError + imu_error.cpp (0x18003AD54..0x180050EE0), LocalParameterization slot thunks (0x180052D90), MarginalizationError (0x1800784A8..), PoseError (0x18008BBD0), PoseLocalParameterization (0x18008C980), PoseParameterBlock (0x18008D9C0), SpeedAndBiasError (0x18008EC08), SpeedAndBiasParameterBlock (0x18008FEB0) | marker only | L |
| 19 | 21 | common/frame.cpp | Frame ctors 0x180091DA0/0x180092420 use `.bss` 0x18047DB38 (±1) | M |
| 20 | 22 (+23, 24) | common/logger.cpp, **or** logger.cpp is a marker-less object between TU20 and TU21 | `kLogTag`/`g_logger` ∈ {TU20, TU21}. `basic_filebuf::_Stinit` (COMDAT bss 0x18047DB50) and the ofstream ctor COMDAT 0x180097B60 sit between frame.cpp and point.cpp code | M |
| 21 | 25 | common/point.cpp | Point code 0x18009A530..0x18009C8E0 | M |
| (none) | 26, 27 | common/portable_binary_iarchive.cpp / _oarchive.cpp (boost example sources, no Eigen) | `map<portable_binary_[io]archive>` singletons ∈ {21, 22}. Code 0x18009CCC4.. (`portable_binary_iarchive_exception`, `codecvt_null`) follows point.cpp | M |
| 22..28 | 28..34 | 7 objects: direct/* (depth_filter, feature_detection, matcher, …) and possibly common/seed-type files. `.bss` hints: TU23 ↔ 0x1800A2AC0 (DepthFilter area), TU24 ↔ 0x1800A63E0 (between DepthFilter and FastGradDetector). matcher.cpp `__FILE__` at 0x1800AF5E0 | L |
| 29 | 35..38 | frontend/frame_processor.cpp | includes Pangolin `handler.h` (StaticHandler pair) + unused static string. Code: pangolin::Handler vtable funcs 0x1800B1450, FrameProcessor 0x1800B29B0 | H |
| 30 | 39..129 | frontend/frame_processor_base.cpp | g_permon, kStageName…, and the 84 PlatMap boost-serialization singletons | H |
| 31 | 130, 131 | frontend/imu_processor.cpp | `imu_temporal_status_names_` | H |
| 32 | 132 | frontend/initialization.cpp | alphabetical, code order (0x18012B8E0) | M |
| 33 | 133 | frontend/map.cpp | `.bss` ↔ 0x18012FE00 (map code) | M |
| 34 | 134 | frontend/pose_optimizer.cpp | code order (0x180133080) | M |
| 35 | 135 | frontend/reprojector.cpp | `.bss` ↔ 0x180142A60 ("bin < 0 or bin > 30") | M |
| 36..39 | 136..139 | 4 objects: frontend/stereo_triangulation (0x180147110), frontend/<VINS IMU init, "INFO=ImuInitial…" 0x18014DFE0..0x180158050>, interface/ceres_backend_factory.cpp (0x180158F70), interface/<voc/calibration loader 0x180159E10..> | L |
| 40 | 140 (+141, 142) | interface/* including Pangolin handler.h | `.bss` ↔ 0x180162340..0x180162E60 | L |
| 41 | 145 (+143, 144, 146) | interface/<Headset C-API impl> (HeadsetInitialImpl 0x180162390, HeadsetReleaseImpl 0x180162DF0) | static `unique_ptr` of the tracker system. Includes Pangolin | M |
| 42 | 149 (+147, 148) | object including Pangolin (BEBLID/DBoW2 area, `.bss` ↔ 0x180174B70) → loop_closing/beblid.cpp? | L |
| 43..46 | 150..153 | loop_closing/* before loop_closing.cpp (DBoW2 vocabulary instantiations 0x180172D70..0x180177330, …). `.bss` TU46 ↔ 0x18017F730/0x18018B600 (±1) | L |
| 47 | 154..157 | loop_closing/loop_closing.cpp | kStrToScaleRetMap, kStrToGlobalMapType, "1.0.0" | H |
| 48..50 | 158..160 | loop_closing/{platmap…} (0x180193E00 "re-localization, save plat map success", 0x180195B70 "LoadSavePlatMap: Start thread.") | L |
| 51 | 161, 162 | plane/mesher.cpp | global `std::vector<int>` used by mesher code | M |
| 52..64 | 163..177 (+165, 166 Pangolin pair in TU53 or TU54) | 13 objects: sensor_fusion/* (Deque 0x1801A3ED0, GyroscopeBiasEstimator 0x1801A4690, ImuFilter 0x1801A5E20), vikit cameras (CameraGeometryBase 0x1801B3C50), vikit performance_monitor (0x1801B5080). tinyxml2 (0x1801B1CC0..) has no marker and no initializer | L |
| 65 | 178..180 | vikit_common/sample.cpp | `Sample::gen_real` (ranlux24), `gen_int` (mt19937) | H |
| 66 | 181 | probably vikit_solver/robust_cost.cpp (MAD/Tukey 0x1801B5C50) | L |
| (none) | 182 | DBoW2 ScoringObject.cpp (no Eigen) | `GeneralScoring::LOG_EPS = log(DBL_EPSILON)` ∈ {66, 67} | H (object), L (marker) |
| 67..70 | 183..186 | first Ceres objects / remaining DBoW2-vikit objects that include Eigen | L |
| 70/71 | 187 | Ceres `internal/ceres/miniglog/glog/logging.cc` | `std::set<LogSink*> log_sinks_global`, used by miniglog `MessageLogger::~MessageLogger` 0x1801B8DD0 | H |
| 71..182 | 188..299 | Ceres 2.1.0 objects (112 Eigen-including .cc files) | — | H (lib) |

Library objects after Ceres (no Eigen; table #300..#456, see the full table):
glog 0.5.0 `logging.cc` (#300..#381), `vlog_is_on.cc` (#382..#390), `utilities.cc` (#391..#397).
Then Pangolin objects (#398..#402: two `handler.h` pairs and a `std::map<std::string,…>`), ConcRT
`_Task_cv_mutex` (#403), and Pangolin pixel-format/video objects (#404..#408). Then boost 1.74
serialization `extended_type_info.cpp`, `extended_type_info_typeid.cpp`, `void_cast.cpp`,
`binary_iarchive.cpp`, `binary_oarchive.cpp` (#409..#413), boost filesystem `operations.cpp`
(#414, #415), gflags 2.2.2 `gflags.cc` (#416..#437), `gflags_reporting.cc` (#438..#451),
`gflags_completions.cc` (#452..#456), and STL `locale0_implib` (#0, code 0x180006050 = the last
`.text$di` contribution).

Key facts for the build:

* **Eigen 3.4.x** (`placeholders::end`, see above).
* **Ceres is built with MINIGLOG.** It has its own `logging.cc` with `log_sinks_global`, and its
  `MessageLogger` writes to `std::cerr` and calls `abort()` on FATAL (0x1801B8DD0, 342 callers, all
  inside Ceres). The project itself uses **real glog 0.5.0 + gflags 2.2.2**, which are linked after Ceres.
  `third_party/ceres-config` must add `internal/ceres/miniglog` to Ceres' include path, and Ceres must
  not see glog's headers.
* **The project defines no gflags of its own.** No project initializer registers a flag, so upstream
  SVO `DEFINE_*` (for example `FLAGS_extrinsics_sigma_rel_translation`) were removed.
* Pangolin `handler.h` is included by TU29 (frame_processor.cpp), TU30 (frame_processor_base.cpp),
  TU40, TU41 (headset API), TU42 and TU53/54. Each such object gets two atexit-only statics
  (`StaticHandler`, `StaticHandlerScroll`).
* All PlatMap / KeyFrame boost-serialization singletons are first instantiated in
  **frame_processor_base.cpp**: (i|o)serializer for portable_binary_[io]archive and boost
  binary_[io]archive of PlatMap, KeyFrame, vector<vector<KeyFrame*>>, vector<KeyFrame*>,
  Transformation, cv::Mat, cv::Point2f, cv::Point3f, std::vector<cv::Mat/Point2f/Point3f/int>,
  DBoW2::BowVector, Eigen::Vector3d, std::map<unsigned,double>, std::pair<const unsigned,double>,
  plus pointer_[io]serializer<KeyFrame>. That TU therefore calls PlatMap save/load with **both**
  archive families.
* Several upstream `std::map` globals became `std::unordered_map` with **`std::hash`** (FNV-1a over the
  key bytes). The upstream `EnumClassHash` is not used: kErrorToStr, kStageName,
  kTrackingQualityName, kUpdateResultName, imu_temporal_status_names_, kStrToScaleRetMap,
  kStrToGlobalMapType. Iteration order of these maps is hash order.

---------------------------------------------------------------------------------------------------

## 2. Function table — `.text$mn` part (0x180006060..0x180010440, 128 functions)

All belong to object #1 (ceres_backend_interface.obj). "project" rows are reconstructed in `draft/c00_globals`.

| address | proposed name | signature | kind | upstream | summary |
|---|---|---|---|---|---|
| 0x180006060 | Eigen::Ref<const Vector3d>::Ref(const Vector3d&) | Ref* (Ref*, const Vector3d*) | lib:Eigen | - | Ref construction with MapBase/stride asserts |
| 0x1800060A0 | Eigen::CommaInitializer<Matrix<double,3,6>>::CommaInitializer(xpr, const Matrix3d&) | | lib:Eigen | - | first 3x3 block of `J << C, …` |
| 0x180006290 | std::operator<<(std::ostream&, const char*) | | lib:std | - | 432 callers |
| 0x180006460 | CommaInitializer<Matrix<3,6>>::operator,(Product<-Matrix3d,Matrix3d>) | | lib:Eigen | - | second block `-C*skew(p)` |
| 0x1800066A0 | std::_Tree<set<int>>::_Copy_nodes | | lib:std | - | |
| 0x180006760 | std::_Deallocate<16> | | lib:std | - | |
| 0x1800067A0 | std::_Destroy_range<32-byte elem {shared_ptr,…}> | | lib:std | - | |
| 0x180006820 | std::_Destroy_range<std::shared_ptr<T>> | | lib:std | - | |
| 0x1800068A0 | std::vector<{shared_ptr<T>,int64,int,int}>::_Emplace_reallocate | | lib:std | - | caller 0x180011700 |
| 0x180006A40 | std::_Tree<…>::~_Tree (erase + free head) | | lib:std | - | |
| 0x180006AC0 | std::_Tree_val::_Erase_tree | | lib:std | - | |
| 0x180006B20 | std::_Tree<map<K,std::string>>::_Erase_tree | | lib:std | - | |
| 0x180006BE0 | std::set<int>::_Find_lower_bound | | lib:std | - | |
| 0x180006C30 | std::string::_Reallocate_grow_by (append) | | lib:std | - | |
| 0x180006DC0 | std::_Uninitialized_move<32-byte elem> | | lib:std | - | |
| 0x180006E50 | Eigen checkTransposeAliasing_impl::run | | lib:Eigen | - | |
| 0x180006E80 | Eigen::LLT<Matrix2d,Lower>::compute | | lib:Eigen | - | used by setInformation |
| 0x180007120 | std::unordered_map<uint64 key, {vector…}>::_Try_emplace | | lib:std | - | FNV over 8 bytes; caller 0x180010C20 |
| 0x180007450 | std::_Fill_unchecked<8-byte> | | lib:std | - | bucket vector fill |
| 0x1800074E0 | Eigen dense_assignment_loop (fill n doubles with constant) | | lib:Eigen | - | setZero/setConstant |
| 0x1800075A0 | std::set<int>::emplace | | lib:std | - | |
| 0x1800076A0 | Eigen redux: column cwiseAbs().sum() | | lib:Eigen | - | LLT |
| 0x180007870 | Eigen: block /= scalar | | lib:Eigen | - | LLT |
| 0x180007900 | Eigen llt_inplace helper (rank update) | | lib:Eigen | - | |
| 0x180007AB0 | Eigen::internal::llt_inplace<double,Lower>::unblocked | | lib:Eigen | - | |
| 0x1800080B0..0x180008930 (13 fns: 0x1800080B0, 160, 210, 2C0, 370, 420, 4D0, 580, 630, 670, 720, 7D0, 880, 930) | Eigen CwiseNullaryOp<scalar_constant_op>(rows,cols,val) / Block / Map ctors with dimension asserts; 0x180008630 = plain_array 16-byte alignment assert (DenseStorage.h:109) | | lib:Eigen | - | e.g. 0x180008160 = Constant(2,1), 0x180008210 = Constant(2,3), 0x180008370 = Constant(2,6), 0x180008720 = 3x1 block, 0x1800087D0 = 1x3 block, 0x180008670 = 1x6 row, 0x180008880 = 6x1 col |
| 0x1800089C0 | kindr::minimal::RotationQuaternionTemplate<double>::RotationQuaternionTemplate(const Eigen::Quaterniond&) | | lib:minkindr | - | CHECK squaredNorm within 1±eps (rotation-quaternion-inl.h) |
| 0x180008B90 | Eigen variable_if_dynamic / InnerStride assert | | lib:Eigen | - | |
| 0x180008BF0 | std::string::string(const std::string&) | | lib:std | - | 60 callers |
| **0x180008CD0** | pimax::totem::CeresBackendInterface::CeresBackendInterface | (const CeresBackendInterfaceOptions&, const CeresBackendOptions&, const CameraBundlePtr&) | project | modified: no MotionDetector, no extrinsics branch, no time limit, no FLAGS | ctor; see §4.3 |
| **0x180008F50** | ceres_backend::ReprojectionError::ReprojectionError | (CameraConstPtr, const Vector2d&, const Matrix2d&) | project | modified (3-D point block, extra members, no CHECK on camera) | ctor |
| 0x180009340 | implicit copy ctor of 32-byte {shared_ptr<T>, int64, int, int} | | lib (compiler-generated) | - | caller 0x180011700 |
| 0x180009380 | std::bad_alloc::bad_alloc(const bad_alloc&) | | lib:std | - | |
| 0x1800093C0 | throw std::bad_alloc | | lib:std | - | |
| 0x1800093F0 | std::bad_array_new_length copy ctor | | lib:std | - | |
| 0x180009430 | throw std::bad_array_new_length | | lib:std | - | |
| 0x180009460 | std::exception copy ctor | | lib:std | - | |
| 0x1800094A0 | Eigen::CommaInitializer::finished()/dtor assert | | lib:Eigen | - | |
| 0x1800094E0, 0x180009500, 0x180009520 | EH-funclet helpers (free buffer at +8) | | lib:std | - | callers in .text$x |
| 0x180009540 | std list/hash node guard dtor | | lib:std | - | |
| 0x180009590 | tree node guard dtor | | lib:std | - | |
| 0x1800095B0 | Eigen aligned_stack_memory_handler dtor | | lib:Eigen | - | |
| 0x1800095D0 | std::string::~string (_Tidy_deallocate) | | lib:std | - | 222 callers |
| 0x180009630 | std::stringbuf::_Tidy (~basic_stringbuf body) | | lib:std | - | |
| 0x1800096F0 | std::deque<shared_ptr<T>>::~deque | | lib:std | - | |
| 0x180009720 | std::lock_guard<std::mutex>::~lock_guard | | lib:std | - | |
| 0x180009730 | std::set<…>::~set | | lib:std | - | |
| 0x180009740 | std::map<…, std::string>::~map | | lib:std | - | |
| 0x180009770 | weak_ptr release (ctrl at +16) | | lib:std | - | |
| 0x180009790 | std::shared_ptr<T>::~shared_ptr | | lib:std | - | 174 callers |
| 0x1800097E0 | unique_ptr<Eigen-aligned T>::~unique_ptr (free) | | lib:std | - | |
| 0x1800097F0 | unique_ptr<T> dtor → T dtor 0x1801B4DD0 | | lib:std | - | |
| 0x180009820 | std::vector<4-byte>::~vector | | lib:std | - | |
| 0x180009880 | std::vector<16-byte>::~vector | | lib:std | - | |
| 0x1800098E0 | std::vector<32-byte elem with shared_ptr>::~vector | | lib:std | - | |
| **0x180009950** | pimax::totem::CeresBackendInterface::~CeresBackendInterface | (CeresBackendInterface*) | project | modified (no quitThread) | member dtors only |
| 0x180009A60 | std::_Ref_count_base::_Decwref | | lib:std | - | |
| 0x180009A80 | thunk → ceres::CostFunction::~CostFunction (0x1801B77D0) | | lib:ceres | - | |
| 0x180009A90 | std::basic_ostream::_Sentry_base::~_Sentry_base | | lib:std | - | |
| 0x180009AC0 | (IDA: Concurrency::agent::~agent) trivial dtor | | lib | - | |
| 0x180009AE0 | j__Mtx_destroy_in_situ (std::mutex::~mutex) | | lib:std | - | |
| 0x180009AF0 | std::basic_ostream::sentry::~sentry | | lib:std | - | |
| 0x180009B30 | kindr::minimal::QuatTransformationTemplate<double>::operator=(const&) | | lib:minkindr | - | |
| 0x180009B60 | Eigen::Quaterniond::operator= | | lib:Eigen | - | |
| 0x180009B80 | kindr::minimal::QuatTransformationTemplate<double>::operator*(const QuatTransformation&) | (Transformation* this, Transformation* ret, const Transformation* rhs) | lib:minkindr (**modified copy?**) | - | q·q', t+q.rotate(t'). The composed quaternion is then normalized again (`if (n>0) q/=sqrt(n)`); reference/minkindr's operator* does not do this. TODO(verify) the in-tree minkindr copy |
| 0x180009C80 | kindr::minimal::RotationQuaternionTemplate<double>::operator*(const RotationQuaternion&) | | lib:minkindr | - | Hamilton product + normalizationHelper (normalize if \|‖q‖²−1\|>1e-4) |
| 0x180009E50 | Eigen::internal::gebp_kernel<double,double,long,…,4,4>::operator() | | lib:Eigen | - | GEMM micro-kernel (6 KB), callers in project + Ceres |
| 0x18000B600, 0x18000B840 | Eigen gemm_pack_lhs<…,ColMajor> (PanelMode false/true) | | lib:Eigen | - | GeneralBlockPanelKernel.h:2110 |
| 0x18000BA50, 0x18000BC50 | Eigen gemm_pack_rhs<…,ColMajor> (PanelMode true/false) | | lib:Eigen | - | :2515 |
| 0x18000BE20 | std::basic_stringstream<char>::~basic_stringstream (complete) | | lib:std | - | |
| 0x18000BE80 | basic_stringstream vbase deleting-dtor thunk | | lib:std | - | |
| 0x18000BE8C | ReprojectionError dtor thunk (this −= 0x28) | | project (compiler thunk) | - | ErrorInterface-vtable slot 0 |
| 0x18000BEA0 | ceres::SizedCostFunction<…>::`scalar deleting dtor' (ICF-shared) | | lib:ceres | - | |
| 0x18000BEE0 | std::_Ref_count_obj2<ReprojectionError>::_Destroy | | lib:std | - | |
| 0x18000BF10 | std::_Ref_count_resource<T*,default_delete<T>>::_Destroy | | lib:std | - | |
| 0x18000BF40 | basic_stringbuf scalar deleting dtor | | lib:std | - | |
| 0x18000BF80 | basic_stringstream deleting dtor | | lib:std | - | |
| **0x18000C020** | ReprojectionError::~ReprojectionError (scalar deleting) | | project | identical | releases camera_geometry_, ~CostFunction, aligned free |
| 0x18000C0D0 | std::exception scalar deleting dtor | | lib:std | - | |
| **0x18000C120** | pimax::Logger::Debug | void (const char* fmt, ...) | project | sibling-modified (flush) | LOGD |
| **0x18000C2C0** | pimax::Logger::Error | void (const char* fmt, ...) | project | sibling-modified | LOGE |
| **0x18000C460** | ReprojectionError::Evaluate | bool (const double* const*, double*, double**) const | project | modified (flag dispatch) | |
| **0x18000C490** | ReprojectionError::EvaluateWithMinimalJacobians | bool (const double* const*, double*, double**, double**) const | project | modified (§4.2) | 7 KB |
| **0x18000E0A0** | ReprojectionError::EvaluateMinimal (name TODO) | bool (const double* const*, double*, double**) const | project | new | 4.7 KB, vtable slot 9 |
| **0x18000F310** | pimax::Logger::TimeString | std::string () | project | sibling-identical (non-static) | |
| **0x18000F500** | pimax::Logger::Info | void (const char* fmt, ...) | project | sibling-modified | LOGI |
| **0x18000F6A0** | pimax::Logger::Warn | void (const char* fmt, ...) | project | sibling-modified | LOGW |
| 0x18000F840 | std::vector<_List_iter>::assign(n, val) (unordered_map bucket init) | | lib:std | - | used by every global unordered_map init |
| 0x18000F970 | std::_Ref_count_base::_Decref | | lib:std | - | |
| 0x18000F9C0, 0x18000F9E0 | _Ref_count_obj2<…>::_Delete_this / dtor (ICF-shared) | | lib:std | - | |
| 0x18000F9F0 | _Ref_count_resource<vk::PerformanceMonitor*,default_delete>::_Destroy | | lib:std | - | |
| 0x18000FA20, 0x18000FA40 | vector tidy guards (EH) | | lib:std | - | |
| 0x18000FA60 | std::_Hash<…uint64 key…>::_Forced_rehash ("invalid hash bucket count") | | lib:std | - | |
| 0x18000FC50 | _Ref_count_resource::_Get_deleter | | lib:std | - | |
| 0x18000FC80 | empty `ret` (ICF) | | lib | - | |
| 0x18000FC90 | 4-byte "return 0/this" (ICF; boost singleton ctor, tinyxml2 virtuals …) | | lib | - | |
| 0x18000FCA0 | std::_Hash::_Rehash bucket-vector grow | | lib:std | - | |
| 0x18000FE80 | std::_Tree::_Insert_node (RB rebalance) | | lib:std | - | |
| 0x180010100 | std::vector<32-byte elem>::_Reallocate_exactly | | lib:std | - | |
| 0x1800101C0 | __scrt_throw_std_bad_alloc (IDA: cancel_current_task) | | lib:CRT | - | |
| 0x1800101E0, 0x180010390, 0x1800103B0, 0x1800103D0 | std::_Xlength_error("map/set too long" / "deque<T> too long" / "string too long" / "vector too long") | | lib:std | - | |
| 0x1800103F0, 0x180010410 | std::_Xout_of_range("invalid deque<T> subscript" / "invalid vector subscript") | | lib:std | - | |
| 0x180010200 | std::deque<shared_ptr<T>>::_Tidy | | lib:std | - | |
| 0x180010320 | std::_Uninitialized_move<shared_ptr<T>> | | lib:std | - | |
| 0x180010430 | nullsub (ICF-shared empty function) | | lib | - | |

`.text$di` part (0x180001000..0x180006060, 457 functions): every function is one row of the
initializer table in §8. Kinds: the 182 Eigen markers are `lib:Eigen (placeholders::end init)`; the 86
boost singleton inits are `lib:boost`; the Pangolin, glog, gflags and boost entries are libraries. The
project initializers are #4, #6, #23, #24, #36–#45, #131, #141–#148, #155–#157, #162, #165, #166.
They are compiler-generated from the global definitions in `draft/c00_globals`.

---------------------------------------------------------------------------------------------------

## 3. Logger (vs ../LedObjectPoseEstimator/src/common/logger.{h,cpp})

Same class and the same layout, `sizeof = 0x138`:

| offset | type | name | evidence |
|---|---|---|---|
| +0x00 | bool | m_enabled (=1) | `.data` 0x18046A000 const-init, tested first in every LOGx |
| +0x04 | int | m_level (=3, kWarn) | `cmp [rcx+4], 1/2/3/4; jg return` |
| +0x08 | std::string | m_tag ("S alg:") | copy-constructed from kLogTag in 0x180001810 |
| +0x28 | std::ofstream | m_file | ctor 0x180097B60, `_Myfile` checked at +0xB0 |
| +0x130 | bool | m_console (=0) | `cmp byte [rdi+130h]` → printf("%s %s %s", tag, level, line) |

Differences from the sibling:
1. **Inline header members.** Debug 0x18000C120, Info 0x18000F500, Warn 0x18000F6A0, Error 0x18000C2C0
   and TimeString 0x18000F310 are COMDATs placed in object #1. Each LOGx contains the whole
   `Write` body. logger.cpp only holds `kLogTag` and `g_logger`.
2. **`m_file.flush()` after every line** (`std::ostream::flush(this+0x28)`, called after the
   TimeString() temporary is destroyed). The sibling has no flush.
3. TimeString is a non-static member (`this` in rcx).
4. The tag is `"S alg:"`, where the sibling has `"C alg"`.
5. `Init()` is not in this chunk. It is called at 0x1801692E0 from the Headset init (0x1801663F0) and
   uses boost::filesystem (directory_iterator, remove) and "%Y_%m_%d_%H_%M_%S". `Close()` is inlined
   into the tracker-system destructor 0x180167320.

Constants: level names "[debug]" 0x1803AA338, "[info]", "[warn]", "[error]"; "%s %s %s" 0x1803AA340;
"%s" 0x1803AA330; "_" 0x1803AA260; buffers are 256 bytes (vsnprintf limit 0x100, snprintf limit 0xFF).

---------------------------------------------------------------------------------------------------

## 4. Types

### 4.1 ErrorInterface (Pimax)
`vptr` +0, **`bool` at +8** (that is, +0x30 in every concrete error). The ErrorInterface ctor clears it,
and every Evaluate tests it (see `error_interface.hpp`). Vtable slots match upstream: dtor, residualDim,
parameterBlocks, parameterBlockDim, EvaluateWithMinimalJacobians, typeInfo.
`ErrorType` is uint8 and gains `kGroundPlaneError = 7`.

### 4.2 ReprojectionError (header-only, sizeof 0xF0)
The layout table is in `draft/c00_globals/ceres_backend/reprojection_error.hpp` (all offsets sure).
Vtables: primary 0x1803AA288 (10 slots), ErrorInterface 0x1803AA2E0 (6 slots), and
`_Ref_count_obj2<ReprojectionError>` 0x1803ABC40.
Upstream status: **modified**.
* `SizedCostFunction<2,7,3,7>`: the point is a Euclidean 3-vector, not a homogeneous 4-vector. The
  math is `p_S = C_SW (p_W − t_WS)`, `p_C = C_CS (p_S − t_SC)`. There are no 4x4 matrices.
* The pose and extrinsics jacobians are comma-initialised 3x6 blocks `[C, −C·skew(p)]`, where
  `skewSymmetric(Ref<const Vector3d>)` is the COMDAT 0x180016490. `J1 = −Jh·(C_CS·C_SW)`. `J_lift`
  comes from `PoseLocalParameterization::liftJacobian` 0x18008CC60.
* EvaluateWithMinimalJacobians weights with the **scalar `square_root_information_(0,0)`**; the new
  EvaluateMinimal (slot 9) uses the full 2x2 matrix.
* The error and weighted error are cached in `mutable` members at +0x60/+0x50, with getters in vtable
  slots 6/5.
* project3 is not called when `p_C.z < 0`. The NaN-safe form is `if (p_C[2] < 0.0)`: `comisd 0,z; ja`,
  so NaN does project. Validity is `!(p_C.z < 0.2) && projection_ok`.
* `point_constant_` is ignored. There is no CHECK in setCameraGeometry.

### 4.3 CeresBackendInterface (sizeof 0x4F0, no vtable)
The layout and option structs are in `draft/c00_globals/ceres_backend/ceres_backend_interface_ctor.cpp`.
Upstream `CeresBackendInterface` derives from `AbstractBundleAdjustment`. In Pimax the class is flat:
`type_` is a member at +0x3D0 that is set to 2 (`kCeres`) in the body. The Estimator `backend_` sits at
+0xB0 and is 0x300 bytes. Two `std::deque<std::shared_ptr<…>>` sit at +0x78 and +0x3F0, a `std::set<int>`
at +0x418, a `std::mutex` at +0x428, and the identity Transformation `w_T_correction_to_apply_` at
+0x480.

### 4.4 Logger — §3.

### 4.5 Global objects (addresses)
| address | object | type |
|---|---|---|
| 0x18046A000 | pimax::g_logger | Logger (0x138) |
| 0x18046A138 | kLogTag | std::string "S alg:" |
| 0x18046A170 | (frame_processor.cpp) unused static string | std::string |
| 0x18046A200 | plat-map version | std::string "1.0.0" |
| 0x18047D9F0 | kErrorToStr | unordered_map<ErrorType,string> |
| 0x18047DA40 | MarginalizationTiming::names_ | vector<string> |
| 0x18047DDA0 | g_permon | shared_ptr<vk::PerformanceMonitor> |
| 0x18047DDB0 / DDF0 / DE30 | kStageName / kTrackingQualityName / kUpdateResultName | unordered_map<enum,string> |
| 0x18047ECE0 | imu_temporal_status_names_ | unordered_map<IMUTemporalStatus,string> |
| 0x18047EDF0 | headset API static tracker system | unique_ptr<…> |
| 0x18047EE90 / EEE0 | kStrToGlobalMapType / kStrToScaleRetMap | unordered_map<string,enum> |
| 0x18047EF68 | mesher global | std::vector<int> |
| 0x18047F178 | Ceres miniglog log_sinks_global | std::set<LogSink*> (the IDA name is `j`) |

---------------------------------------------------------------------------------------------------

## 5. External interfaces (called from this chunk; implemented elsewhere)

| address | meaning |
|---|---|
| 0x180011570 | std::string::assign(const char*, size_t) |
| 0x180011490 | vector allocate helper |
| 0x180014A20 | kindr RotationQuaternion::rotate(const Vector3d&) |
| 0x180016490 | `Eigen::Matrix3d skewSymmetric(const Eigen::Ref<const Eigen::Vector3d>&)` (vio_common/matrix.hpp, COMDAT) |
| 0x180016530 | std::stringbuf::str() |
| 0x180016600 | `Eigen::Map<const Eigen::Quaterniond>::toRotationMatrix()` (COMDAT) |
| 0x180016B80 / 0x180016B90 / 0x180016BF0 | `__local_stdio_printf_options` / printf / snprintf (inline CRT, COMDAT) |
| 0x18008CC60 | `static bool ceres_backend::PoseLocalParameterization::liftJacobian(const double* x, double* J /*6x7 RowMajor*/)` |
| 0x180023430 / 0x1800239A0 | Estimator::Estimator() / ~Estimator() |
| 0x180025810 | `Estimator::addCameraBundle(const CameraBundlePtr&)` (Pimax 1-arg) |
| Estimator +0xB8 | `min_num_3d_points_for_fixation_` (written by the CeresBackendInterface ctor) |
| camera vtbl +0x18 | `vk::cameras::CameraGeometryBase::project3(const Ref<const Vector3d>&, Vector2d*, Matrix<double,2,3>*) const` returning ProjectionResult (status 0 = KEYPOINT_VISIBLE) |
| 0x180020440, 0x1800D0090, 0x180129A60, 0x18017E530 | unordered_map range-insert for the global maps (ErrorType, 4-byte enum, IMUTemporalStatus, std::string keys) |
| 0x180020830 / 0x1800095D0 | ~pair<const Enum,std::string> / ~std::string (EH vector destructor iterator in initializers) |
| 0x1800207C0, 0x1800215D0 | ~list / _Destroy_range<string> (atexit dtors) |
| 0x180097B60 / 0x180097CD0 | std::ofstream ctor / dtor (placed in logger.obj) |
| 0x1801B77B0 / 0x1801B77D0 | ceres::CostFunction ctor / dtor |
| 0x1801692E0 | Logger::Init-equivalent (log file creation/rotation) |
| 0x180167320 | tracker-system destructor (calls quit-backend 0x18016A280 and g_logger.Close()) |

---------------------------------------------------------------------------------------------------

## 6. Constants / defaults

* Logger: m_enabled=1, m_level=3, m_console=0, tag "S alg:".
* ReprojectionError: behind-camera threshold 0.0, validity depth 0.2 (0x1803ADDC0), and jacobian sizes
  2x7/2x3/2x7 (2x6/2x3/2x6 minimal).
* CeresBackendInterface members: last_added_nframe_* = −1, w_T_correction = identity
  (q=(0,0,0,1)), a member at +0x4E8 = 0.001, a member at +0x4DC = 1, type_ = 2 (kCeres).
* Factory values (interface/ceres_backend_factory.cpp, for the reconciler):
  CeresBackendInterfaceOptions {2, 2/180·π, false, true, 5, true, 2.75, 5, true}.
  CeresBackendOptions {−1.0, 5, 1, false, true, 8, 1, true, true, 10}.
* Global maps: listed in §8 rows #4, #43–#45, #131, #155–#157.

## 7. Quirks / bugs to preserve

* ReprojectionError: when `p_C.z < 0` (or projection fails), `kp` is **uninitialised** and is still
  used for the residual (`measurement_ − kp`). The jacobians are zero because valid=false, but the
  residual is garbage stack memory. Keep `measurement_t kp;` uninitialised.
* ReprojectionError writes `mutable` members inside a `const` Evaluate. This is **not thread-safe**
  if Ceres evaluates residual blocks in parallel (num_threads=1 in the factory).
* With `use_minimal_jacobian_` set, Evaluate passes Ceres' ambient-size jacobian buffers (2x7) to code
  that writes minimal 2x6 row-major blocks. Rows end up packed differently than Ceres expects. It is
  harmless only if the flag stays false (nothing in this chunk sets it).
* `EvaluateWithMinimalJacobians` repeats the test `(jacobians || jacobians_minimal)` (kept). It uses
  only the (0,0) element of the 2x2 sqrt-information.
* minkindr in-tree copy: `QuatTransformation::operator*` renormalizes the result quaternion (§2,
  0x180009B80).
* Global maps are unordered, so any code that iterates them follows hash order.

---------------------------------------------------------------------------------------------------

## 8. Full `.CRT$XCU` table (object link order)

"TU" is filled only on Eigen-marker rows; other rows give the resolved object in the "object" column.
| # | init EA | TU | object (confidence) | global(s) |
|---|---|---|---|---|
| 0 | 180006050 | | lib:MSVC STL locale0_implib.obj (.CRT$XCC, init_seg(compiler)) | atexit(_Fac_tidy) -- static _Fac_tidy_reg_t |
| 1 | 180001000 | TU1 | ceres_backend/ceres_backend_interface.cpp (H) | Eigen 3.4 `placeholders::end = lastp1` (word 18047d9b0 -> 18047d9b4) |
| 2 | 180001010 | TU2 | ceres_backend/ceres_map.cpp (M) | Eigen 3.4 `placeholders::end = lastp1` (word 18047d9c0 -> 18047d9c4) |
| 3 | 180001020 | TU3 | ceres_backend/error_interface.cpp (H) | Eigen 3.4 `placeholders::end = lastp1` (word 18047da34 -> 18047da38) |
| 4 | 180001030 | | TU3 error_interface.cpp | kErrorToStr: std::unordered_map<ErrorType,std::string> {0 HomogeneousPointError,1 ReprojectionError,2 SpeedAndBiasError,3 MarginalizationError,4 PoseError,5 IMUError,6 RelativePoseError,7 GroundPlaneError} |
| 5 | 180001700 | TU4 | ceres_backend/estimator.cpp (H) | Eigen 3.4 `placeholders::end = lastp1` (word 18047da60 -> 18047da64) |
| 6 | 180001460 | | TU4 estimator.cpp | MarginalizationTiming::names_ (std::vector<std::string>, 6 entries "0_mag_pre_iterate".."5_finish") |
| 7 | 180001710 | TU5 | ceres_backend/* (14 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047da6c -> 18047da70) |
| 8 | 180001720 | TU6 | ceres_backend/* (14 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047da78 -> 18047da7c) |
| 9 | 180001730 | TU7 | ceres_backend/* (14 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047da88 -> 18047da8c) |
| 10 | 180001740 | TU8 | ceres_backend/* (14 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047da94 -> 18047da98) |
| 11 | 180001750 | TU9 | ceres_backend/* (14 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047daa0 -> 18047daa4) |
| 12 | 180001760 | TU10 | ceres_backend/* (14 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047dab0 -> 18047dab4) |
| 13 | 180001770 | TU11 | ceres_backend/* (14 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047dac0 -> 18047dac4) |
| 14 | 180001780 | TU12 | ceres_backend/* (14 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047dad0 -> 18047dad4) |
| 15 | 180001790 | TU13 | ceres_backend/* (14 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047dae0 -> 18047dae4) |
| 16 | 1800017a0 | TU14 | ceres_backend/* (14 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047daec -> 18047daf0) |
| 17 | 1800017b0 | TU15 | ceres_backend/* (14 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047dafc -> 18047db00) |
| 18 | 1800017c0 | TU16 | ceres_backend/* (14 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047db08 -> 18047db0c) |
| 19 | 1800017d0 | TU17 | ceres_backend/* (14 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047db14 -> 18047db18) |
| 20 | 1800017e0 | TU18 | ceres_backend/* (14 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047db24 -> 18047db28) |
| 21 | 1800017f0 | TU19 | common/frame.cpp (M) | Eigen 3.4 `placeholders::end = lastp1` (word 18047db30 -> 18047db34) |
| 22 | 180001800 | TU20 | common/logger.cpp (M; or logger.cpp is a marker-less obj between TU20/TU21) | Eigen 3.4 `placeholders::end = lastp1` (word 18047db40 -> 18047db44) |
| 23 | 180001850 | | TU20/21 common/logger.cpp | static const std::string kLogTag = "S alg:" (0x18046A138) |
| 24 | 180001810 | | TU20/21 common/logger.cpp | Logger g_logger(kLogTag) (0x18046A000; m_enabled=1,m_level=3 const-init in .data; ofstream ctor 0x180097B60) |
| 25 | 180001880 | TU21 | common/point.cpp (M) | Eigen 3.4 `placeholders::end = lastp1` (word 18047db68 -> 18047db6c) |
| 26 | 180001890 | | TU21/22 (portable_binary_iarchive.cpp?) | boost singleton<extra_detail::map<portable_binary_iarchive>>::m_instance |
| 27 | 1800018b0 | | TU21/22 (portable_binary_oarchive.cpp?) | boost singleton<extra_detail::map<portable_binary_oarchive>>::m_instance |
| 28 | 1800018d0 | TU22 | direct/* or common/* (7 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047dbb4 -> 18047dbb8) |
| 29 | 1800018e0 | TU23 | direct/* or common/* (7 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047dbc0 -> 18047dbc4) |
| 30 | 1800018f0 | TU24 | direct/* or common/* (7 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047dc38 -> 18047dc3c) |
| 31 | 180001900 | TU25 | direct/* or common/* (7 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047dd48 -> 18047dd4c) |
| 32 | 180001910 | TU26 | direct/* or common/* (7 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047dd54 -> 18047dd58) |
| 33 | 180001920 | TU27 | direct/* or common/* (7 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047dd60 -> 18047dd64) |
| 34 | 180001930 | TU28 | direct/* or common/* (7 TUs, see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047dd6c -> 18047dd70) |
| 35 | 180001950 | TU29 | frontend/frame_processor.cpp (H) | Eigen 3.4 `placeholders::end = lastp1` (word 18047dd7c -> 18047dd80) |
| 36 | 180001960 | | TU29 frame_processor.cpp | pangolin/handler/handler.h: static Handler StaticHandler (atexit only) |
| 37 | 180001970 | | TU29 frame_processor.cpp | pangolin/handler/handler.h: static HandlerScroll StaticHandlerScroll |
| 38 | 180001940 | | TU29 frame_processor.cpp | static std::string (0x18046A170, const-initialised empty, never referenced) -- atexit only |
| 39 | 180002890 | | TU30 frame_processor_base.cpp | pangolin StaticHandler |
| 40 | 1800028a0 | | TU30 frame_processor_base.cpp | pangolin StaticHandlerScroll |
| 41 | 180002400 | TU30 | frontend/frame_processor_base.cpp (H) | Eigen 3.4 `placeholders::end = lastp1` (word 18047dd98 -> 18047dd9c) |
| 42 | 1800028b0 | | TU30 frame_processor_base.cpp | PerformanceMonitorPtr g_permon (0x18047DDA0) -- atexit only |
| 43 | 180002410 | | TU30 frame_processor_base.cpp | kStageName: unordered_map<Stage,string>{0 Paused,1 Initializing,2 Tracking,3 Reloc} |
| 44 | 1800025a0 | | TU30 frame_processor_base.cpp | kTrackingQualityName: unordered_map<TrackingQuality,string>{0 Insufficient,1 Bad,2 Good} |
| 45 | 180002710 | | TU30 frame_processor_base.cpp | kUpdateResultName: unordered_map<UpdateResult,string>{0 Default,1 KF,2 Failure} |
| 46 | 180002360 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<portable_binary_oarchive,PlatMap>>::m_instance |
| 47 | 180001f60 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<portable_binary_iarchive,PlatMap>>::m_instance |
| 48 | 180001b60 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<bs::extended_type_info_typeid<PlatMap>>::m_instance |
| 49 | 180002160 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<ba::binary_oarchive,PlatMap>>::m_instance |
| 50 | 180001d60 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<ba::binary_iarchive,PlatMap>>::m_instance |
| 51 | 1800022c0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<portable_binary_oarchive,std::vector<std::vector<KeyFrame *>>>>::m_instance |
| 52 | 180001ec0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<portable_binary_iarchive,std::vector<std::vector<KeyFrame *>>>>::m_instance |
| 53 | 180001ac0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<bs::extended_type_info_typeid<std::vector<std::vector<KeyFrame *>>>>::m_instance |
| 54 | 1800020c0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<ba::binary_oarchive,std::vector<std::vector<KeyFrame *>>>>::m_instance |
| 55 | 180001cc0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<ba::binary_iarchive,std::vector<std::vector<KeyFrame *>>>>::m_instance |
| 56 | 180001e60 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<portable_binary_iarchive,std::vector<KeyFrame *>>>::m_instance |
| 57 | 180001a60 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<bs::extended_type_info_typeid<std::vector<KeyFrame *>>>::m_instance |
| 58 | 180002260 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<portable_binary_oarchive,std::vector<KeyFrame *>>>::m_instance |
| 59 | 180001c60 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<ba::binary_iarchive,std::vector<KeyFrame *>>>::m_instance |
| 60 | 180002060 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<ba::binary_oarchive,std::vector<KeyFrame *>>>::m_instance |
| 61 | 180001b20 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<bs::extended_type_info_typeid<KeyFrame>>::m_instance |
| 62 | 1800023a0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::pointer_iserializer<portable_binary_iarchive,KeyFrame>>::m_instance |
| 63 | 180001f20 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<portable_binary_iarchive,KeyFrame>>::m_instance |
| 64 | 1800023e0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::pointer_oserializer<portable_binary_oarchive,KeyFrame>>::m_instance |
| 65 | 180002320 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<portable_binary_oarchive,KeyFrame>>::m_instance |
| 66 | 180002380 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::pointer_iserializer<ba::binary_iarchive,KeyFrame>>::m_instance |
| 67 | 180001d20 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<ba::binary_iarchive,KeyFrame>>::m_instance |
| 68 | 1800023c0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::pointer_oserializer<ba::binary_oarchive,KeyFrame>>::m_instance |
| 69 | 180002120 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<ba::binary_oarchive,KeyFrame>>::m_instance |
| 70 | 180001e00 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<portable_binary_iarchive,kindr::minimal::QuatTransformationTemplate<double>>>::m_instance |
| 71 | 180001ea0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<portable_binary_iarchive,std::vector<cv::Point_<float>>>>::m_instance |
| 72 | 180001ee0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<portable_binary_iarchive,std::vector<cv::Mat>>>::m_instance |
| 73 | 180001f00 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<portable_binary_iarchive,DBoW2::BowVector>>::m_instance |
| 74 | 180001e80 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<portable_binary_iarchive,std::vector<cv::Point3_<float>>>>::m_instance |
| 75 | 180001a80 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<bs::extended_type_info_typeid<std::vector<cv::Point3_<float>>>>::m_instance |
| 76 | 180001b00 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<bs::extended_type_info_typeid<DBoW2::BowVector>>::m_instance |
| 77 | 180001ae0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<bs::extended_type_info_typeid<std::vector<cv::Mat>>>::m_instance |
| 78 | 180001aa0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<bs::extended_type_info_typeid<std::vector<cv::Point_<float>>>>::m_instance |
| 79 | 180001a00 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<bs::extended_type_info_typeid<kindr::minimal::QuatTransformationTemplate<double>>>::m_instance |
| 80 | 180001e40 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<portable_binary_iarchive,std::vector<int>>>::m_instance |
| 81 | 180002200 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<portable_binary_oarchive,kindr::minimal::QuatTransformationTemplate<double>>>::m_instance |
| 82 | 1800022a0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<portable_binary_oarchive,std::vector<cv::Point_<float>>>>::m_instance |
| 83 | 1800022e0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<portable_binary_oarchive,std::vector<cv::Mat>>>::m_instance |
| 84 | 180002300 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<portable_binary_oarchive,DBoW2::BowVector>>::m_instance |
| 85 | 180002280 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<portable_binary_oarchive,std::vector<cv::Point3_<float>>>>::m_instance |
| 86 | 180001a40 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<bs::extended_type_info_typeid<std::vector<int>>>::m_instance |
| 87 | 180002240 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<portable_binary_oarchive,std::vector<int>>>::m_instance |
| 88 | 180001c00 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<ba::binary_iarchive,kindr::minimal::QuatTransformationTemplate<double>>>::m_instance |
| 89 | 180001ca0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<ba::binary_iarchive,std::vector<cv::Point_<float>>>>::m_instance |
| 90 | 180001ce0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<ba::binary_iarchive,std::vector<cv::Mat>>>::m_instance |
| 91 | 180001d00 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<ba::binary_iarchive,DBoW2::BowVector>>::m_instance |
| 92 | 180001c80 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<ba::binary_iarchive,std::vector<cv::Point3_<float>>>>::m_instance |
| 93 | 180001c40 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<ba::binary_iarchive,std::vector<int>>>::m_instance |
| 94 | 180002000 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<ba::binary_oarchive,kindr::minimal::QuatTransformationTemplate<double>>>::m_instance |
| 95 | 1800020a0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<ba::binary_oarchive,std::vector<cv::Point_<float>>>>::m_instance |
| 96 | 1800020e0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<ba::binary_oarchive,std::vector<cv::Mat>>>::m_instance |
| 97 | 180002100 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<ba::binary_oarchive,DBoW2::BowVector>>::m_instance |
| 98 | 180002080 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<ba::binary_oarchive,std::vector<cv::Point3_<float>>>>::m_instance |
| 99 | 180002040 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<ba::binary_oarchive,std::vector<int>>>::m_instance |
| 100 | 180001da0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<portable_binary_iarchive,Eigen::Matrix<double,3,1,0,3,1>>>::m_instance |
| 101 | 180001e20 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<portable_binary_iarchive,std::map<unsigned int,double>>>::m_instance |
| 102 | 180001a20 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<bs::extended_type_info_typeid<std::map<unsigned int,double>>>::m_instance |
| 103 | 1800019a0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<bs::extended_type_info_typeid<Eigen::Matrix<double,3,1,0,3,1>>>::m_instance |
| 104 | 1800021a0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<portable_binary_oarchive,Eigen::Matrix<double,3,1,0,3,1>>>::m_instance |
| 105 | 180002220 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<portable_binary_oarchive,std::map<unsigned int,double>>>::m_instance |
| 106 | 180001ba0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<ba::binary_iarchive,Eigen::Matrix<double,3,1,0,3,1>>>::m_instance |
| 107 | 180001c20 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<ba::binary_iarchive,std::map<unsigned int,double>>>::m_instance |
| 108 | 180001fa0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<ba::binary_oarchive,Eigen::Matrix<double,3,1,0,3,1>>>::m_instance |
| 109 | 180002020 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<ba::binary_oarchive,std::map<unsigned int,double>>>::m_instance |
| 110 | 180001de0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<portable_binary_iarchive,cv::Point_<float>>>::m_instance |
| 111 | 180001f40 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<portable_binary_iarchive,cv::Mat>>::m_instance |
| 112 | 180001dc0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<portable_binary_iarchive,cv::Point3_<float>>>::m_instance |
| 113 | 1800019c0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<bs::extended_type_info_typeid<cv::Point3_<float>>>::m_instance |
| 114 | 180001b40 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<bs::extended_type_info_typeid<cv::Mat>>::m_instance |
| 115 | 1800019e0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<bs::extended_type_info_typeid<cv::Point_<float>>>::m_instance |
| 116 | 1800021e0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<portable_binary_oarchive,cv::Point_<float>>>::m_instance |
| 117 | 180002340 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<portable_binary_oarchive,cv::Mat>>::m_instance |
| 118 | 1800021c0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<portable_binary_oarchive,cv::Point3_<float>>>::m_instance |
| 119 | 180001be0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<ba::binary_iarchive,cv::Point_<float>>>::m_instance |
| 120 | 180001d40 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<ba::binary_iarchive,cv::Mat>>::m_instance |
| 121 | 180001bc0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<ba::binary_iarchive,cv::Point3_<float>>>::m_instance |
| 122 | 180001d80 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<portable_binary_iarchive,std::pair<unsigned int const,double>>>::m_instance |
| 123 | 180001980 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<bs::extended_type_info_typeid<std::pair<unsigned int const,double>>>::m_instance |
| 124 | 180001fe0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<ba::binary_oarchive,cv::Point_<float>>>::m_instance |
| 125 | 180002140 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<ba::binary_oarchive,cv::Mat>>::m_instance |
| 126 | 180001fc0 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<ba::binary_oarchive,cv::Point3_<float>>>::m_instance |
| 127 | 180002180 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<portable_binary_oarchive,std::pair<unsigned int const,double>>>::m_instance |
| 128 | 180001b80 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::iserializer<ba::binary_iarchive,std::pair<unsigned int const,double>>>::m_instance |
| 129 | 180001f80 | | TU30 frame_processor_base.cpp (COMDAT, first instantiation) | boost bs::singleton<ba::detail::oserializer<ba::binary_oarchive,std::pair<unsigned int const,double>>>::m_instance |
| 130 | 1800028c0 | TU31 | frontend/imu_processor.cpp (H) | Eigen 3.4 `placeholders::end = lastp1` (word 18047ecd4 -> 18047ecd8) |
| 131 | 1800028d0 | | TU31 imu_processor.cpp | imu_temporal_status_names_: unordered_map<IMUTemporalStatus,string>{0 Stationary,1 Moving,2 Unknown} |
| 132 | 180002a50 | TU32 | frontend/initialization.cpp (M) | Eigen 3.4 `placeholders::end = lastp1` (word 18047ed24 -> 18047ed28) |
| 133 | 180002a60 | TU33 | frontend/map.cpp (M) | Eigen 3.4 `placeholders::end = lastp1` (word 18047ed30 -> 18047ed34) |
| 134 | 180002a70 | TU34 | frontend/pose_optimizer.cpp (M) | Eigen 3.4 `placeholders::end = lastp1` (word 18047ed3c -> 18047ed40) |
| 135 | 180002a80 | TU35 | frontend/reprojector.cpp (M) | Eigen 3.4 `placeholders::end = lastp1` (word 18047ed58 -> 18047ed5c) |
| 136 | 180002a90 | TU36 | frontend/* or interface/* (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047ed64 -> 18047ed68) |
| 137 | 180002aa0 | TU37 | frontend/* or interface/* (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047ed90 -> 18047ed94) |
| 138 | 180002ab0 | TU38 | frontend/* or interface/* (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047edb8 -> 18047edbc) |
| 139 | 180002ac0 | TU39 | frontend/* or interface/* (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047edc8 -> 18047edcc) |
| 140 | 180002ad0 | TU40 | frontend/* or interface/* (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047edd8 -> 18047eddc) |
| 141 | 180002ae0 | | TU40 | pangolin StaticHandler |
| 142 | 180002af0 | | TU40 | pangolin StaticHandlerScroll |
| 143 | 180002b10 | | TU41 Headset API impl | pangolin StaticHandler |
| 144 | 180002b20 | | TU41 Headset API impl | pangolin StaticHandlerScroll |
| 145 | 180002b00 | TU41 | interface/<Headset*Impl API>.cpp (M-H) | Eigen 3.4 `placeholders::end = lastp1` (word 18047ede8 -> 18047edec) |
| 146 | 180002b30 | | TU41/42 Headset API impl | static std::unique_ptr<tracker-system> (0x18047EDF0, created in HeadsetInitialImpl 0x180162390, deleted in HeadsetReleaseImpl 0x180162DF0; dtor 0x180167320 closes g_logger file) -- atexit only |
| 147 | 180002b50 | | TU42 | pangolin StaticHandler |
| 148 | 180002b60 | | TU42 | pangolin StaticHandlerScroll |
| 149 | 180002b40 | TU42 | interface/* or loop_closing/* (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047ee04 -> 18047ee08) |
| 150 | 180002b70 | TU43 | interface/* or loop_closing/* (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047ee2c -> 18047ee30) |
| 151 | 180002b80 | TU44 | interface/* or loop_closing/* (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047ee68 -> 18047ee6c) |
| 152 | 180002b90 | TU45 | interface/* or loop_closing/* (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047ee74 -> 18047ee78) |
| 153 | 180002ba0 | TU46 | interface/* or loop_closing/* (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047ee84 -> 18047ee88) |
| 154 | 180002be0 | TU47 | loop_closing/loop_closing.cpp (H) | Eigen 3.4 `placeholders::end = lastp1` (word 18047eed8 -> 18047eedc) |
| 155 | 180002df0 | | TU47 loop_closing.cpp | kStrToScaleRetMap: unordered_map<string,LCScaleRetMethod>{CommonLM 0, MixedKP 1, None 2} |
| 156 | 180002bf0 | | TU47 loop_closing.cpp | kStrToGlobalMapType: unordered_map<string,GlobalMapType>{BuiltInPoseGraph 0, ExternalGlobalMap 1, None 2} |
| 157 | 180002bb0 | | TU47 loop_closing.cpp | static const std::string platmap version "1.0.0" (0x18046A200; compared in PlatMap load 0x180183320 "Wont load the map because of illegal map version.") |
| 158 | 180002fd0 | TU48 | loop_closing/* (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047ef34 -> 18047ef38) |
| 159 | 180002fe0 | TU49 | loop_closing/* (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047ef40 -> 18047ef44) |
| 160 | 180002ff0 | TU50 | loop_closing/* (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047ef50 -> 18047ef54) |
| 161 | 180003010 | TU51 | plane/mesher.cpp (M) | Eigen 3.4 `placeholders::end = lastp1` (word 18047ef84 -> 18047ef88) |
| 162 | 180003000 | | TU51 mesher.cpp | std::vector<int> global (0x18047EF68; const-init empty) used by mesher 0x1801A0AF0/0x1801A3750 and frame_processor_base 0x1800F3430 -- atexit only |
| 163 | 180003020 | TU52 | plane/sensor_fusion/vikit (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047ef90 -> 18047ef94) |
| 164 | 180003030 | TU53 | plane/sensor_fusion/vikit (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047efa0 -> 18047efa4) |
| 165 | 180003040 | | TU53/54 | pangolin StaticHandler |
| 166 | 180003050 | | TU53/54 | pangolin StaticHandlerScroll |
| 167 | 180003060 | TU54 | plane/sensor_fusion/vikit (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047efac -> 18047efb0) |
| 168 | 180003070 | TU55 | plane/sensor_fusion/vikit (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047efb8 -> 18047efbc) |
| 169 | 180003080 | TU56 | plane/sensor_fusion/vikit (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047efc8 -> 18047efcc) |
| 170 | 180003090 | TU57 | plane/sensor_fusion/vikit (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047efd4 -> 18047efd8) |
| 171 | 1800030a0 | TU58 | plane/sensor_fusion/vikit (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047efe0 -> 18047efe4) |
| 172 | 1800030b0 | TU59 | plane/sensor_fusion/vikit (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047efec -> 18047eff0) |
| 173 | 1800030c0 | TU60 | plane/sensor_fusion/vikit (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047eff8 -> 18047effc) |
| 174 | 1800030d0 | TU61 | plane/sensor_fusion/vikit (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047f01c -> 18047f020) |
| 175 | 1800030e0 | TU62 | plane/sensor_fusion/vikit (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047f028 -> 18047f02c) |
| 176 | 1800030f0 | TU63 | plane/sensor_fusion/vikit (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047f034 -> 18047f038) |
| 177 | 180003100 | TU64 | plane/sensor_fusion/vikit (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047f040 -> 18047f044) |
| 178 | 180003190 | TU65 | vikit_common/sample.cpp (H) | Eigen 3.4 `placeholders::end = lastp1` (word 18047f120 -> 18047f124) |
| 179 | 180003150 | | TU65 vikit sample.cpp | std::ranlux24 vk::Sample::gen_real (default seed 19780503) |
| 180 | 180003110 | | TU65 vikit sample.cpp | std::mt19937 vk::Sample::gen_int (default seed 5489) |
| 181 | 1800031a0 | TU66 | vikit/DBoW2/first Ceres objs (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047f12c -> 18047f130) |
| 182 | 1800031b0 | | lib:DBoW2 ScoringObject.cpp (marker-less or TU66/67) | const double GeneralScoring::LOG_EPS = log(DBL_EPSILON) |
| 183 | 1800031d0 | TU67 | vikit/DBoW2/first Ceres objs (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047f148 -> 18047f14c) |
| 184 | 1800031e0 | TU68 | vikit/DBoW2/first Ceres objs (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047f154 -> 18047f158) |
| 185 | 1800031f0 | TU69 | vikit/DBoW2/first Ceres objs (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047f160 -> 18047f164) |
| 186 | 180003200 | TU70 | vikit/DBoW2/first Ceres objs (see list) | Eigen 3.4 `placeholders::end = lastp1` (word 18047f16c -> 18047f170) |
| 187 | 180003210 | | lib:Ceres miniglog/glog/logging.cc (TU70/71) | std::set<google::LogSink*> log_sinks_global  => Ceres built with MINIGLOG |
| 188 | 180003250 | TU71 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f18c -> 18047f190) |
| 189 | 180003260 | TU72 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f19c -> 18047f1a0) |
| 190 | 180003270 | TU73 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f1a8 -> 18047f1ac) |
| 191 | 180003280 | TU74 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f1b4 -> 18047f1b8) |
| 192 | 180003290 | TU75 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f1c0 -> 18047f1c4) |
| 193 | 1800032a0 | TU76 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f1cc -> 18047f1d0) |
| 194 | 1800032b0 | TU77 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f1d8 -> 18047f1dc) |
| 195 | 1800032c0 | TU78 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f1e4 -> 18047f1e8) |
| 196 | 1800032d0 | TU79 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f1f0 -> 18047f1f4) |
| 197 | 1800032e0 | TU80 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f1fc -> 18047f200) |
| 198 | 1800032f0 | TU81 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f208 -> 18047f20c) |
| 199 | 180003300 | TU82 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f218 -> 18047f21c) |
| 200 | 180003310 | TU83 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f224 -> 18047f228) |
| 201 | 180003320 | TU84 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f230 -> 18047f234) |
| 202 | 180003330 | TU85 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f23c -> 18047f240) |
| 203 | 180003340 | TU86 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f248 -> 18047f24c) |
| 204 | 180003350 | TU87 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f258 -> 18047f25c) |
| 205 | 180003360 | TU88 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f268 -> 18047f26c) |
| 206 | 180003370 | TU89 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f274 -> 18047f278) |
| 207 | 180003380 | TU90 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f280 -> 18047f284) |
| 208 | 180003390 | TU91 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f28c -> 18047f290) |
| 209 | 1800033a0 | TU92 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f298 -> 18047f29c) |
| 210 | 1800033b0 | TU93 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f2a4 -> 18047f2a8) |
| 211 | 1800033c0 | TU94 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f2b0 -> 18047f2b4) |
| 212 | 1800033d0 | TU95 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f2bc -> 18047f2c0) |
| 213 | 1800033e0 | TU96 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f2c8 -> 18047f2cc) |
| 214 | 1800033f0 | TU97 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f2d4 -> 18047f2d8) |
| 215 | 180003400 | TU98 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f2e0 -> 18047f2e4) |
| 216 | 180003410 | TU99 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f2ec -> 18047f2f0) |
| 217 | 180003420 | TU100 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f2f8 -> 18047f2fc) |
| 218 | 180003430 | TU101 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f304 -> 18047f308) |
| 219 | 180003440 | TU102 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f310 -> 18047f314) |
| 220 | 180003450 | TU103 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f31c -> 18047f320) |
| 221 | 180003460 | TU104 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f328 -> 18047f32c) |
| 222 | 180003470 | TU105 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f334 -> 18047f338) |
| 223 | 180003480 | TU106 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f340 -> 18047f344) |
| 224 | 180003490 | TU107 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f34c -> 18047f350) |
| 225 | 1800034a0 | TU108 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f358 -> 18047f35c) |
| 226 | 1800034b0 | TU109 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f364 -> 18047f368) |
| 227 | 1800034c0 | TU110 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f374 -> 18047f378) |
| 228 | 1800034d0 | TU111 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f380 -> 18047f384) |
| 229 | 1800034e0 | TU112 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f38c -> 18047f390) |
| 230 | 1800034f0 | TU113 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f39c -> 18047f3a0) |
| 231 | 180003500 | TU114 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f3a8 -> 18047f3ac) |
| 232 | 180003510 | TU115 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f3b4 -> 18047f3b8) |
| 233 | 180003520 | TU116 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f3c0 -> 18047f3c4) |
| 234 | 180003530 | TU117 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f3d0 -> 18047f3d4) |
| 235 | 180003540 | TU118 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f3dc -> 18047f3e0) |
| 236 | 180003550 | TU119 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f3ec -> 18047f3f0) |
| 237 | 180003560 | TU120 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f3f8 -> 18047f3fc) |
| 238 | 180003570 | TU121 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f408 -> 18047f40c) |
| 239 | 180003580 | TU122 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f414 -> 18047f418) |
| 240 | 180003590 | TU123 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f420 -> 18047f424) |
| 241 | 1800035a0 | TU124 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f430 -> 18047f434) |
| 242 | 1800035b0 | TU125 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f440 -> 18047f444) |
| 243 | 1800035c0 | TU126 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f450 -> 18047f454) |
| 244 | 1800035d0 | TU127 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f45c -> 18047f460) |
| 245 | 1800035e0 | TU128 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f468 -> 18047f46c) |
| 246 | 1800035f0 | TU129 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f478 -> 18047f47c) |
| 247 | 180003600 | TU130 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f484 -> 18047f488) |
| 248 | 180003610 | TU131 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f490 -> 18047f494) |
| 249 | 180003620 | TU132 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f49c -> 18047f4a0) |
| 250 | 180003630 | TU133 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f4a8 -> 18047f4ac) |
| 251 | 180003640 | TU134 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f4b8 -> 18047f4bc) |
| 252 | 180003650 | TU135 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f4c4 -> 18047f4c8) |
| 253 | 180003660 | TU136 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f4d0 -> 18047f4d4) |
| 254 | 180003670 | TU137 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f4e0 -> 18047f4e4) |
| 255 | 180003680 | TU138 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f4f0 -> 18047f4f4) |
| 256 | 180003690 | TU139 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f500 -> 18047f504) |
| 257 | 1800036a0 | TU140 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f510 -> 18047f514) |
| 258 | 1800036b0 | TU141 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f520 -> 18047f524) |
| 259 | 1800036c0 | TU142 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f530 -> 18047f534) |
| 260 | 1800036d0 | TU143 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f540 -> 18047f544) |
| 261 | 1800036e0 | TU144 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f550 -> 18047f554) |
| 262 | 1800036f0 | TU145 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f560 -> 18047f564) |
| 263 | 180003700 | TU146 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f570 -> 18047f574) |
| 264 | 180003710 | TU147 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f580 -> 18047f584) |
| 265 | 180003720 | TU148 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f590 -> 18047f594) |
| 266 | 180003730 | TU149 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f5a0 -> 18047f5a4) |
| 267 | 180003740 | TU150 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f5b0 -> 18047f5b4) |
| 268 | 180003750 | TU151 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f5c0 -> 18047f5c4) |
| 269 | 180003760 | TU152 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f5d0 -> 18047f5d4) |
| 270 | 180003770 | TU153 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f5e0 -> 18047f5e4) |
| 271 | 180003780 | TU154 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f5f0 -> 18047f5f4) |
| 272 | 180003790 | TU155 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f600 -> 18047f604) |
| 273 | 1800037a0 | TU156 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f610 -> 18047f614) |
| 274 | 1800037b0 | TU157 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f620 -> 18047f624) |
| 275 | 1800037c0 | TU158 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f630 -> 18047f634) |
| 276 | 1800037d0 | TU159 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f63c -> 18047f640) |
| 277 | 1800037e0 | TU160 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f648 -> 18047f64c) |
| 278 | 1800037f0 | TU161 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f658 -> 18047f65c) |
| 279 | 180003800 | TU162 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f668 -> 18047f66c) |
| 280 | 180003810 | TU163 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f678 -> 18047f67c) |
| 281 | 180003820 | TU164 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f688 -> 18047f68c) |
| 282 | 180003830 | TU165 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f698 -> 18047f69c) |
| 283 | 180003840 | TU166 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f6a8 -> 18047f6ac) |
| 284 | 180003850 | TU167 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f6b8 -> 18047f6bc) |
| 285 | 180003860 | TU168 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f6c8 -> 18047f6cc) |
| 286 | 180003870 | TU169 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f6d8 -> 18047f6dc) |
| 287 | 180003880 | TU170 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f6e8 -> 18047f6ec) |
| 288 | 180003890 | TU171 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f6f8 -> 18047f6fc) |
| 289 | 1800038a0 | TU172 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f708 -> 18047f70c) |
| 290 | 1800038b0 | TU173 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f718 -> 18047f71c) |
| 291 | 1800038c0 | TU174 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f728 -> 18047f72c) |
| 292 | 1800038d0 | TU175 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f738 -> 18047f73c) |
| 293 | 1800038e0 | TU176 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f748 -> 18047f74c) |
| 294 | 1800038f0 | TU177 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f758 -> 18047f75c) |
| 295 | 180003900 | TU178 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f768 -> 18047f76c) |
| 296 | 180003910 | TU179 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f778 -> 18047f77c) |
| 297 | 180003920 | TU180 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f788 -> 18047f78c) |
| 298 | 180003930 | TU181 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f798 -> 18047f79c) |
| 299 | 180003940 | TU182 | lib:Ceres 2.1.0 object | Eigen 3.4 `placeholders::end = lastp1` (word 18047f7a8 -> 18047f7ac) |
| 300 | 180004280 | | lib:glog 0.5.0 logging.cc | FLAGS default from env GLOG_timestamp_in_logfile_name |
| 301 | 180004340 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (byte_18048E87D -> byte_18048E872) |
| 302 | 180004320 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (byte_18048E87D -> byte_18048E871) |
| 303 | 180004a40 | | lib:glog 0.5.0 logging.cc | FlagRegisterer("timestamp_in_logfile_name") |
| 304 | 1800040f0 | | lib:glog 0.5.0 logging.cc | FLAGS default from env GLOG_logtostderr |
| 305 | 180003a70 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (env_3 -> byte_18048E880) |
| 306 | 180003dd0 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (env_3 -> byte_18048E878) |
| 307 | 1800048b0 | | lib:glog 0.5.0 logging.cc | FlagRegisterer("logtostderr") |
| 308 | 180003e00 | | lib:glog 0.5.0 logging.cc | FLAGS default from env GLOG_alsologtostderr |
| 309 | 1800039b0 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (env -> byte_18048E870) |
| 310 | 180003b20 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (env -> byte_18048E87E) |
| 311 | 1800044f0 | | lib:glog 0.5.0 logging.cc | FlagRegisterer("alsologtostderr") |
| 312 | 180003e90 | | lib:glog 0.5.0 logging.cc | FLAGS default from env GLOG_colorlogtostderr |
| 313 | 1800039c0 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (byte_18048E86E -> byte_18048E876) |
| 314 | 180003b30 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (byte_18048E86E -> byte_18048E875) |
| 315 | 180004540 | | lib:glog 0.5.0 logging.cc | FlagRegisterer("colorlogtostderr") |
| 316 | 1800049a0 | | lib:glog 0.5.0 logging.cc | FlagRegisterer("stderrthreshold") |
| 317 | 180003aa0 | | lib:glog 0.5.0 logging.cc | FLAGS default from env GLOG_alsologtoemail |
| 318 | 1800044a0 | | lib:glog 0.5.0 logging.cc | FlagRegisterer("alsologtoemail") |
| 319 | 180004350 | | lib:glog 0.5.0 logging.cc | atexit(sub_1803A6570) (dtor of const-initialised static) |
| 320 | 1800039a0 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (qword_18048E888 -> qword_18048E8D0) |
| 321 | 180003ef0 | | lib:glog 0.5.0 logging.cc | FLAGS default from env GLOG_log_prefix |
| 322 | 180003a00 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (byte_18048E882 -> byte_18048E86F) |
| 323 | 180003cf0 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (byte_18048E882 -> byte_18048E874) |
| 324 | 180004680 | | lib:glog 0.5.0 logging.cc | FlagRegisterer("log_prefix") |
| 325 | 1800041d0 | | lib:glog 0.5.0 logging.cc | FLAGS default from env GLOG_minloglevel |
| 326 | 180003a90 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (env_5 -> dword_18047F984) |
| 327 | 180003df0 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (env_5 -> dword_1804871CC) |
| 328 | 180004950 | | lib:glog 0.5.0 logging.cc | FlagRegisterer("minloglevel") |
| 329 | 180003fb0 | | lib:glog 0.5.0 logging.cc | FLAGS default from env GLOG_logbuflevel |
| 330 | 180003a20 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (env_0 -> dword_18047F994) |
| 331 | 180003d10 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (env_0 -> dword_18047F968) |
| 332 | 180004720 | | lib:glog 0.5.0 logging.cc | FlagRegisterer("logbuflevel") |
| 333 | 180004000 | | lib:glog 0.5.0 logging.cc | FLAGS default from env GLOG_logbufsecs |
| 334 | 180003a30 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (env_1 -> dword_180487068) |
| 335 | 180003d20 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (env_1 -> dword_18048706C) |
| 336 | 180004770 | | lib:glog 0.5.0 logging.cc | FlagRegisterer("logbufsecs") |
| 337 | 180004050 | | lib:glog 0.5.0 logging.cc | FLAGS default from env GLOG_logemaillevel |
| 338 | 180003a40 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (env_2 -> dword_18048E8C4) |
| 339 | 180003d30 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (env_2 -> dword_18047F97C) |
| 340 | 1800047c0 | | lib:glog 0.5.0 logging.cc | FlagRegisterer("logemaillevel") |
| 341 | 180003d50 | | lib:glog 0.5.0 logging.cc | FLAGS default from env GLOG_logmailer |
| 342 | 180004860 | | lib:glog 0.5.0 logging.cc | FlagRegisterer("logmailer") |
| 343 | 180004390 | | lib:glog 0.5.0 logging.cc | atexit(sub_1803A65B0) (dtor of const-initialised static) |
| 344 | 180003a60 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (qword_18047F998 -> qword_18048E8C8) |
| 345 | 1800040a0 | | lib:glog 0.5.0 logging.cc | FLAGS default from env GLOG_logfile_mode |
| 346 | 180003a50 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (PMode_0 -> PMode) |
| 347 | 180003d40 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (PMode_0 -> PMode_1) |
| 348 | 180004810 | | lib:glog 0.5.0 logging.cc | FlagRegisterer("logfile_mode") |
| 349 | 180003bc0 | | lib:glog 0.5.0 logging.cc | FLAGS default from env GLOG_log_dir |
| 350 | 1800045e0 | | lib:glog 0.5.0 logging.cc | FlagRegisterer("log_dir") |
| 351 | 180004370 | | lib:glog 0.5.0 logging.cc | atexit(sub_1803A6590) (dtor of const-initialised static) |
| 352 | 1800039e0 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (qword_18047F958 -> qword_18047F950) |
| 353 | 180003c70 | | lib:glog 0.5.0 logging.cc | FLAGS default from env GLOG_log_link |
| 354 | 180004630 | | lib:glog 0.5.0 logging.cc | FlagRegisterer("log_link") |
| 355 | 180004380 | | lib:glog 0.5.0 logging.cc | atexit(sub_1803A65A0) (dtor of const-initialised static) |
| 356 | 1800039f0 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (qword_1804871C0 -> qword_18047F960) |
| 357 | 180004180 | | lib:glog 0.5.0 logging.cc | FLAGS default from env GLOG_max_log_size |
| 358 | 180003a80 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (env_4 -> dword_18047F990) |
| 359 | 180003de0 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (env_4 -> dword_1804871C8) |
| 360 | 180004900 | | lib:glog 0.5.0 logging.cc | FlagRegisterer("max_log_size") |
| 361 | 180004220 | | lib:glog 0.5.0 logging.cc | FLAGS default from env GLOG_stop_logging_if_full_disk |
| 362 | 180004330 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (byte_18048E887 -> byte_18048E873) |
| 363 | 180004310 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (byte_18048E887 -> byte_18048E884) |
| 364 | 1800049f0 | | lib:glog 0.5.0 logging.cc | FlagRegisterer("stop_logging_if_full_disk") |
| 365 | 180003b40 | | lib:glog 0.5.0 logging.cc | FLAGS default from env GLOG_log_backtrace_at |
| 366 | 180004590 | | lib:glog 0.5.0 logging.cc | FlagRegisterer("log_backtrace_at") |
| 367 | 180004360 | | lib:glog 0.5.0 logging.cc | atexit(sub_1803A6580) (dtor of const-initialised static) |
| 368 | 1800039d0 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (qword_18047F948 -> qword_18047F940) |
| 369 | 180003f50 | | lib:glog 0.5.0 logging.cc | FLAGS default from env GLOG_log_utc_time |
| 370 | 180003a10 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (byte_18048E87A -> byte_18048E879) |
| 371 | 180003d00 | | lib:glog 0.5.0 logging.cc | FLAGS_x = FLAGS_x default copy (byte_18048E87A -> byte_18048E885) |
| 372 | 1800046d0 | | lib:glog 0.5.0 logging.cc | FlagRegisterer("log_utc_time") |
| 373 | 180004470 | | lib:glog 0.5.0 logging.cc | static Mutex (InitializeCriticalSection) |
| 374 | 180004460 | | lib:glog 0.5.0 logging.cc | atexit(sub_1803A66A0) (dtor of const-initialised static) |
| 375 | 180003950 | | lib:glog 0.5.0 logging.cc | atexit(sub_1803A6480) (dtor of const-initialised static) |
| 376 | 180003960 | | lib:glog 0.5.0 logging.cc | atexit(sub_1803A64F0) (dtor of const-initialised static) |
| 377 | 180003970 | | lib:glog 0.5.0 logging.cc | static Mutex (InitializeCriticalSection) |
| 378 | 180004450 | | lib:glog 0.5.0 logging.cc | atexit(sub_1803A6630) (dtor of const-initialised static) |
| 379 | 180004420 | | lib:glog 0.5.0 logging.cc | static Mutex (InitializeCriticalSection) |
| 380 | 1800043a0 | | lib:glog logging.cc | LogMessage::LogStream fatal_msg_data_exclusive (30000-byte buffer) |
| 381 | 1800043e0 | | lib:glog logging.cc | LogMessage::LogStream fatal_msg_data_shared (30000-byte buffer) |
| 382 | 180004a90 | | lib:glog 0.5.0 vlog_is_on.cc | FLAGS default from env GLOG_v |
| 383 | 180004b70 | | lib:glog 0.5.0 vlog_is_on.cc | FLAGS_x = FLAGS_x default copy (env_6 -> env_8) |
| 384 | 180004ae0 | | lib:glog 0.5.0 vlog_is_on.cc | FLAGS_x = FLAGS_x default copy (env_6 -> env_7) |
| 385 | 180004ba0 | | lib:glog 0.5.0 vlog_is_on.cc | FlagRegisterer("v") |
| 386 | 180004af0 | | lib:glog 0.5.0 vlog_is_on.cc | FLAGS default from env GLOG_vmodule |
| 387 | 180004bf0 | | lib:glog 0.5.0 vlog_is_on.cc | FlagRegisterer("vmodule") |
| 388 | 180004b90 | | lib:glog 0.5.0 vlog_is_on.cc | atexit(sub_1803A66C0) (dtor of const-initialised static) |
| 389 | 180004b80 | | lib:glog 0.5.0 vlog_is_on.cc | FLAGS_x = FLAGS_x default copy (qword_18048EA40 -> qword_18048EA48) |
| 390 | 180004c40 | | lib:glog 0.5.0 vlog_is_on.cc | static Mutex (InitializeCriticalSection) |
| 391 | 180004c70 | | lib:glog 0.5.0 utilities.cc | FLAGS default from env GLOG_symbolize_stacktrace |
| 392 | 180004ce0 | | lib:glog 0.5.0 utilities.cc | FLAGS_x = FLAGS_x default copy (byte_18048EA9A -> byte_18048EA99) |
| 393 | 180004cd0 | | lib:glog 0.5.0 utilities.cc | FLAGS_x = FLAGS_x default copy (byte_18048EA9A -> byte_18048EA9B) |
| 394 | 180004d90 | | lib:glog 0.5.0 utilities.cc | FlagRegisterer("symbolize_stacktrace") |
| 395 | 180004cf0 | | lib:glog utilities.cc | g_main_thread_pid = getpid() |
| 396 | 180004d10 | | lib:glog 0.5.0 utilities.cc | atexit(sub_1803A6790) (dtor of const-initialised static) |
| 397 | 180004d20 | | lib:glog utilities.cc | g_my_user_name from $USERNAME / "invalid-user" |
| 398 | 180004de0 | | lib:Pangolin (display/handler obj #1) | StaticHandler |
| 399 | 180004df0 | | lib:Pangolin obj #1 | StaticHandlerScroll |
| 400 | 180004e70 | | lib:Pangolin (obj #2) | StaticHandler |
| 401 | 180004e80 | | lib:Pangolin obj #2 | StaticHandlerScroll |
| 402 | 180004e00 | | lib:Pangolin obj #2 (display.cpp?) | std::map<std::string,...> (node 0x50) e.g. contexts |
| 403 | 180004e40 | | lib:MSVC ConcRT/ppltasks | ??__E_Task_cv_mutex |
| 404 | 180004e90 | | lib:Pangolin pixel formats | PixelFormat table "GRAY8".."GRAY32F" ... |
| 405 | 180005560 | | lib:Pangolin video/packetstream objs | static const std::string "PANGO" |
| 406 | 180005590 | | lib:Pangolin video/packetstream objs | static const std::string "PANGO" |
| 407 | 1800055c0 | | lib:Pangolin | static const std::string "raw_video" |
| 408 | 1800055f0 | | lib:Pangolin video/packetstream objs | static const std::string "PANGO" |
| 409 | 180005620 | | lib:boost 1.74 serialization extended_type_info.cpp | bs::singleton<std::multiset<bs::extended_type_info const *,bs::detail::key_compare,std::allocator<bs::extended_type_info const *>>> (mutable instance) |
| 410 | 180005640 | | lib:boost serialization extended_type_info_typeid.cpp | bs::singleton<std::multiset<bs::typeid_system::extended_type_info_typeid_0 const *,bs::typeid_system::type_compare,std::allocator<bs::typeid_system::extended_type_info_typeid_0 const *>>> |
| 411 | 180005660 | | lib:boost serialization void_cast.cpp | bs::singleton<std::set<bs::void_cast_detail::void_caster const *,bs::void_cast_detail::void_caster_compare,std::allocator<bs::void_cast_detail::void_caster const *>>> |
| 412 | 180005680 | | lib:boost serialization binary_iarchive.cpp | bs::singleton<ba::detail::extra_detail::map<ba::binary_iarchive>> |
| 413 | 1800056a0 | | lib:boost serialization binary_oarchive.cpp | bs::singleton<ba::detail::extra_detail::map<ba::binary_oarchive>> |
| 414 | 1800056c0 | | lib:boost 1.74 filesystem operations.cpp | create_hard_link_api = GetProcAddress(kernel32,"CreateHardLinkW") |
| 415 | 1800056f0 | | lib:boost filesystem operations.cpp | create_symbolic_link_api = GetProcAddress(kernel32,"CreateSymbolicLinkW") |
| 416 | 180005740 | | lib:gflags 2.2.2 gflags.cc | empty std::string flag storage init |
| 417 | 180005890 | | lib:gflags gflags.cc | FlagRegisterer("flagfile") |
| 418 | 180005840 | | lib:gflags 2.2.2 gflags.cc | atexit(sub_1803A6E80) (dtor of const-initialised static) |
| 419 | 180005720 | | lib:gflags 2.2.2 gflags.cc | FLAGS_x = FLAGS_x default copy (off_18046BDF8 -> qword_18048FA58) |
| 420 | 180005760 | | lib:gflags 2.2.2 gflags.cc | empty std::string flag storage init |
| 421 | 180005980 | | lib:gflags gflags.cc | FlagRegisterer("fromenv") |
| 422 | 180005850 | | lib:gflags 2.2.2 gflags.cc | atexit(sub_1803A6E90) (dtor of const-initialised static) |
| 423 | 180005730 | | lib:gflags 2.2.2 gflags.cc | FLAGS_x = FLAGS_x default copy (off_18046BE10 -> qword_18048FA50) |
| 424 | 180005780 | | lib:gflags 2.2.2 gflags.cc | empty std::string flag storage init |
| 425 | 180005a70 | | lib:gflags gflags.cc | FlagRegisterer("tryfromenv") |
| 426 | 180005860 | | lib:gflags 2.2.2 gflags.cc | atexit(sub_1803A6EA0) (dtor of const-initialised static) |
| 427 | 1800057c0 | | lib:gflags 2.2.2 gflags.cc | FLAGS_x = FLAGS_x default copy (off_18046BE28 -> qword_18048FA68) |
| 428 | 1800057a0 | | lib:gflags 2.2.2 gflags.cc | empty std::string flag storage init |
| 429 | 180005b60 | | lib:gflags gflags.cc | FlagRegisterer("undefok") |
| 430 | 180005870 | | lib:gflags 2.2.2 gflags.cc | atexit(sub_1803A6EB0) (dtor of const-initialised static) |
| 431 | 1800057d0 | | lib:gflags 2.2.2 gflags.cc | FLAGS_x = FLAGS_x default copy (off_18046BE40 -> qword_18048FA88) |
| 432 | 180005880 | | lib:gflags gflags.cc | gflags_exitfunc = &exit |
| 433 | 1800057e0 | | lib:gflags gflags.cc | static string argv0("UNKNOWN") |
| 434 | 180005830 | | lib:gflags 2.2.2 gflags.cc | atexit(sub_1803A6E10) (dtor of const-initialised static) |
| 435 | 180005c50 | | lib:gflags 2.2.2 gflags.cc | atexit(sub_1803A6ED0) (dtor of const-initialised static) |
| 436 | 180005820 | | lib:gflags 2.2.2 gflags.cc | atexit(sub_1803A6D20) (dtor of const-initialised static) |
| 437 | 180005c60 | | lib:gflags 2.2.2 gflags.cc | atexit(sub_1803A6F40) (dtor of const-initialised static) |
| 438 | 180005cf0 | | lib:gflags 2.2.2 gflags_reporting.cc | FlagRegisterer("help") |
| 439 | 180005d40 | | lib:gflags 2.2.2 gflags_reporting.cc | FlagRegisterer("helpfull") |
| 440 | 180005e80 | | lib:gflags 2.2.2 gflags_reporting.cc | FlagRegisterer("helpshort") |
| 441 | 180005cb0 | | lib:gflags 2.2.2 gflags_reporting.cc | empty std::string flag storage init |
| 442 | 180005de0 | | lib:gflags 2.2.2 gflags_reporting.cc | FlagRegisterer("helpon") |
| 443 | 180005ce0 | | lib:gflags 2.2.2 gflags_reporting.cc | atexit(sub_1803A6FC0) (dtor of const-initialised static) |
| 444 | 180005c80 | | lib:gflags 2.2.2 gflags_reporting.cc | FLAGS_x = FLAGS_x default copy (off_18046BED8 -> qword_18048FB48) |
| 445 | 180005c90 | | lib:gflags 2.2.2 gflags_reporting.cc | empty std::string flag storage init |
| 446 | 180005d90 | | lib:gflags 2.2.2 gflags_reporting.cc | FlagRegisterer("helpmatch") |
| 447 | 180005cd0 | | lib:gflags 2.2.2 gflags_reporting.cc | atexit(sub_1803A6FB0) (dtor of const-initialised static) |
| 448 | 180005c70 | | lib:gflags 2.2.2 gflags_reporting.cc | FLAGS_x = FLAGS_x default copy (off_18046BEF0 -> qword_18048FB50) |
| 449 | 180005e30 | | lib:gflags 2.2.2 gflags_reporting.cc | FlagRegisterer("helppackage") |
| 450 | 180005ed0 | | lib:gflags 2.2.2 gflags_reporting.cc | FlagRegisterer("helpxml") |
| 451 | 180005f20 | | lib:gflags 2.2.2 gflags_reporting.cc | FlagRegisterer("version") |
| 452 | 180005f70 | | lib:gflags 2.2.2 gflags_completions.cc | empty std::string flag storage init |
| 453 | 180006000 | | lib:gflags 2.2.2 gflags_completions.cc | FlagRegisterer("tab_completion_word") |
| 454 | 180005fa0 | | lib:gflags 2.2.2 gflags_completions.cc | atexit(sub_1803A6FD0) (dtor of const-initialised static) |
| 455 | 180005f90 | | lib:gflags 2.2.2 gflags_completions.cc | FLAGS_x = FLAGS_x default copy (off_18046BF10 -> qword_18048FBA0) |
| 456 | 180005fb0 | | lib:gflags 2.2.2 gflags_completions.cc | FlagRegisterer("tab_completion_columns") |
| 461 | 180395f54 | | lib:MSVC CRT (.CRT$XCZ-ish / XI) | __scrt_initialize_thread_safe_statics (after __xc_z) |
