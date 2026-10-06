# Chunk c06_common_depthfilter — 0x18009A530 .. 0x1800AF5E0

## Summary

Contents (object files, in image order; inside every object the COMDATs appear sorted by
decorated name, which is a useful naming hint — see "Object boundaries"):

| object (original tree `src/...`) | range | project functions |
|---|---|---|
| `common/point.cpp` (tail; ctor/dtor are just before the chunk) | 0x18009A530 .. 0x18009C6F0 | Point::addObservation, getCloseViewObs, getTriangulationParallax, jacobian_xyz2f/uv (COMDAT), optimize, removeObservation, updateHessianGradientUnitPlane/Sphere |
| `common/portable_archive/portable_binary_iarchive.cpp` (boost 1.74 example, project copy) | 0x18009C6F0 .. 0x18009D800 | load_impl, load_override(class_name_type&), init (+ exception::what from the .hpp) |
| `common/portable_archive/portable_binary_oarchive.cpp` | 0x18009D800 .. 0x18009DBE0 | save_impl |
| `direct/depth_filter.cpp` | 0x18009DBE0 .. 0x1800A3900 | DepthFilter (ctor x2, dtor, stopThread, addKeyframe, reset, updateSeeds, updateSeedsLoop, GetFramesWithoutSeeds), depth_filter_utils::{initializeSeeds, updateSeed, updateFilterVogiatzis, computeTau}, seed::getSigma2FromDepthSigma (COMDAT), 3 file-local helpers |
| `direct/feature_alignment.cpp` | 0x1800A3900 .. 0x1800AA150 | align1D (rewritten), align2D |
| `direct/feature_detection.cpp` | 0x1800AA150 .. 0x1800AAC30 | AbstractDetector ctor, FastGradDetector::detect, OccupandyGrid2D ctor/reset (COMDAT) |
| `direct/feature_detection_utils.cpp` | 0x1800AAC30 .. 0x1800ADD10 | makeDetector, fillFeatures, fastDetector, edgeletDetector_V2 (+2 parallel_for_ lambdas), getAngleAtPixelUsingHistogram, smoothOrientationHistogram |
| `direct/matcher.cpp` (head; continues in c07 at 0x1800AF5E0) | 0x1800ADD10 .. (0x1800B0450) | ZMSSD<4> ctor (COMDAT), createPatchFromPatchWithBorder (COMDAT), depthFromTriangulation (+lambda), findEpipolarMatchDirect(T), findLocalMatch |

Drafts (`draft/c06_common_depthfilter/`):
- `common/point.h`, `common/point.cpp` — Point (+ KeypointIdentifier)
- `common/seed.h` — Pimax seed helpers (float, changed constants)
- `common/occupancy_grid_2d.h` — Pimax OccupandyGrid2D (integer cell index)
- `common/portable_archive/*` — verbatim boost 1.74 example files, `init()` modified
- `direct/depth_filter.h/.cpp`
- `direct/feature_alignment.h/.cpp`
- `direct/feature_detection_types.h`, `direct/feature_detection.h/.cpp`,
  `direct/feature_detection_utils.h/.cpp`
- `direct/matcher_c06.cpp` — the matcher functions of this chunk (header = c07's `matcher.h`
  with the corrections listed under "Matcher")

Upstream fidelity: Point and the boost archive are close to upstream; the depth filter, align1D,
the feature detector and findEpipolarMatchDirect are heavily Pimax-modified (float maths,
new logic, new constants). Everything was compared against pseudocode and, for all
float/double-sensitive code (Point, initializeSeeds, updateSeed, Vogiatzis, computeTau, align1D,
align2D, fastDetector, depthFromTriangulation, findEpipolarMatchDirect), against the disassembly.

Confidence: high for control flow and constants everywhere; high for the float/double mix in
Point, DepthFilter, align1D/align2D, depthFromTriangulation; medium for the exact spelling of
the minkindr cast/rotate expressions in findEpipolarMatchDirect (results are the same for the
alternatives, see the function notes), medium for member/function names marked TODO(verify).

## Function table
| address | proposed qualified name | signature | kind | upstream status | summary |
|---|---|---|---|---|---|
| 0x18009A530 | `pimax::totem::Point::addObservation` | `void (const FramePtr& frame, size_t feature_index)` | project | modified: obs_ is unordered_map<int,KeypointIdentifier>; null frame -> LOGE+return; no CHECK_EQ on duplicate | insert {frame->id_, KeypointIdentifier(frame, idx)} if the frame id is not yet observed |
| 0x18009A6B0 | `std::allocator<std::pair<FramePtr,size_t>>::allocate` | `(size_t n) -> 24-byte elements` | lib:STL | - | _Allocate for vector<pair<FramePtr,size_t>> (Point::optimize) |
| 0x18009A730 | `std::allocator<std::pair<FramePtr,size_t>>::deallocate` | `(void*, size_t)` | lib:STL | - |  |
| 0x18009A780 | `pimax::totem::Point::getCloseViewObs` | `bool (const Eigen::Vector3d& framepos, FramePtr& ref_frame, size_t& ref_feature_index) const` | project | modified: float maths, start -1.1, threshold 0.5, unlockable frames skipped, LOGD | best cos(view angle) over observations |
| 0x18009AA50 | `pimax::totem::Point::getTriangulationParallax` | `double () const` | project | modified: acos(clamp(min abs(cos))), float positions, LOGE, no CHECK | parallax of the point w.r.t. its first observation |
| 0x18009ADE0 | `pimax::totem::Point::jacobian_xyz2f` | `static void (const Vector3d&, const Matrix3d&, Matrix3d&)` | project (header inline, COMDAT) | identical | d(normalized p)/d(p_w) |
| 0x18009B040 | `pimax::totem::Point::jacobian_xyz2uv` | `static void (const Vector3d&, const Matrix3d&, Matrix23d&)` | project (header inline, COMDAT) | identical | unit-plane projection jacobian |
| 0x18009B1E0 | `Eigen::LDLT<Matrix3d>::solve (SolverBase::solve)` | `Solve<> (const MatrixBase&) const` | lib:Eigen | - | returns solve expression after m_isInitialized assert |
| 0x18009B240 | `pimax::totem::Point::optimize` | `void (size_t n_iter, bool using_bearing_vector)` | project | modified: frames locked once into vector<pair<FramePtr,size_t>>, float pos, NaN test replaced by dp.squaredNorm()>1, LOGE | Gauss-Newton refinement of the point position |
| 0x18009BBE0 | `pimax::totem::Point::removeObservation` | `void (int frame_id)` | project | modified: obs_.erase(frame_id) on unordered_map |  |
| 0x18009BD50 | `std::basic_stringbuf<char>::str` | `std::string () const` | lib:STL | - | COMDAT shared with Ceres |
| 0x18009BE10 | `pimax::totem::Point::updateHessianGradientUnitPlane` | `void (const Ref<BearingVector>&, const Vector3d&, const Matrix3d&, Matrix3d&, Vector3d&, double&)` | project | modified: float bearing (project2 in float, widened) |  |
| 0x18009C130 | `pimax::totem::Point::updateHessianGradientUnitSphere` | `void (same)` | project | modified: float bearing widened to double |  |
| 0x18009C6F0 | `std::string::_Reallocate_grow_by<resize lambda>` | `(size_t, ..., char)` | lib:STL | - | string::resize growth |
| 0x18009C880 | `boost::serialization::throw_exception<boost::archive::archive_exception>` | `[[noreturn]] void (const archive_exception&)` | lib:boost | - |  |
| 0x18009C8B0 | `boost::serialization::throw_exception<portable_binary_iarchive_exception>` | `[[noreturn]] void (const E&)` | lib:boost | - |  |
| 0x18009C8E0 | `boost::archive::basic_binary_iprimitive<portable_binary_iarchive,char,traits>::basic_binary_iprimitive` | `(std::streambuf&, bool no_codecvt)` | lib:boost (explicit instantiation in portable_binary_iarchive.cpp) | - |  |
| 0x18009CA10 | `portable_binary_iarchive_exception::portable_binary_iarchive_exception(const&)` | `copy ctor (virtual base)` | lib:boost example header | identical | compiler-generated copy ctor |
| 0x18009CAB0 | `portable_binary_iarchive_exception::portable_binary_iarchive_exception` | `(exception_code c)` | boost example header | identical | archive_exception(other_exception) + m_exception_code |
| 0x18009CB40 | `boost::archive::basic_binary_iprimitive<...>::~basic_binary_iprimitive` | `()` | lib:boost | - |  |
| 0x18009CB90 | `boost::archive::basic_streambuf_locale_saver<char>::~basic_streambuf_locale_saver` | `()` | lib:boost | - | pubsync + pubimbue |
| 0x18009CC10 | `boost::archive::codecvt_null<char>::~codecvt_null` | `()` | lib:boost | - |  |
| 0x18009CC30 | `std::locale::~locale (facet release helper)` | `()` | lib:STL | - | unwind helper |
| 0x18009CC60 | `boost::archive::archive_exception-derived dtor (vbase)` | `()` | lib:boost | - |  |
| 0x18009CC90 | `portable_binary_iarchive_exception::~portable_binary_iarchive_exception (vbase part)` | `()` | lib:boost | - |  |
| 0x18009CCC4 | `[thunk] portable_binary_iarchive_exception vbase dtor adjustor` | `()` | lib:boost | - | vtable 0x1803B1A50 slot 0 |
| 0x18009CCD0 | `boost::archive::codecvt_null<wchar_t>::`vector deleting dtor' (ICF)` | `()` | lib:boost | - |  |
| 0x18009CCE0 | `boost::archive::codecvt_null<char>::`scalar deleting dtor'` | `(unsigned)` | lib:boost | - |  |
| 0x18009CD20 | `portable_binary_iarchive_exception::`scalar deleting dtor'` | `(unsigned)` | lib:boost | - |  |
| 0x18009CD80 | `std::allocator<wchar_t>::allocate (2-byte elements)` | `(size_t)` | lib:STL | - | COMDAT used by boost::filesystem |
| 0x18009CDF0 | `boost::archive::codecvt_null<wchar_t>::do_always_noconv (ICF 'return false')` | `bool () const` | lib:boost | - | folded with many `return 0;` virtuals |
| 0x18009CE00 | `boost::archive::codecvt_null<wchar_t>::do_max_length` | `int () const` | lib:boost | - |  |
| 0x18009CE10 | `boost::archive::detail::archive_serializer_map<portable_binary_iarchive>::erase` | `static void (const basic_serializer*)` | lib:boost | - |  |
| 0x18009CE60 | `boost::archive::detail::archive_serializer_map<portable_binary_iarchive>::find` | `static const basic_serializer* (const extended_type_info&)` | lib:boost | - |  |
| 0x18009CE80 | `boost::serialization::singleton<extra_detail::map<portable_binary_iarchive>>::get_const_instance (thunk)` | `()` | lib:boost | - |  |
| 0x18009CE90 | `boost::serialization::singleton<extra_detail::map<portable_binary_iarchive>>::get_instance` | `()` | lib:boost | - | function-local static 0x18047DB78 |
| 0x18009CF80 | `boost::serialization::singleton_module::get_lock` | `()` | lib:boost | - |  |
| 0x18009CF90 | `singleton<map<portable_binary_iarchive>>::get_mutable_instance` | `()` | lib:boost | - |  |
| 0x18009CFC0 | `portable_binary_iarchive::init` | `void (unsigned int flags)` | third-party (boost 1.74 example, project copy) | modified: no BOOST_ARCHIVE_VERSION()<input_library_version check | signature + library version + endian flags |
| 0x18009D260 | `archive_serializer_map<portable_binary_iarchive>::insert` | `static bool (const basic_serializer*)` | lib:boost | - |  |
| 0x18009D2B0 | `singleton<map<portable_binary_iarchive>>::is_destroyed` | `()` | lib:boost | - |  |
| 0x18009D2C0 | `boost::serialization::singleton_module::is_locked` | `()` | lib:boost | - |  |
| 0x18009D2D0 | `basic_binary_iprimitive<portable_binary_iarchive,...>::load(std::string&)` | `void (std::string&)` | lib:boost | - | load_impl(len,8) + resize + load_binary |
| 0x18009D340 | `basic_binary_iprimitive<portable_binary_iarchive,...>::load_binary` | `void (void*, size_t)` | lib:boost | - | sgetn, throws input_stream_error |
| 0x18009D3B0 | `portable_binary_iarchive::load_impl` | `void (boost::intmax_t& l, char maxsize)` | third-party (boost example) | identical |  |
| 0x18009D4F0 | `portable_binary_iarchive::load_override(class_name_type&)` | `void (class_name_type&)` | third-party (boost example) | identical |  |
| 0x18009D6E0 | `boost::serialization::singleton_module::lock` | `()` | lib:boost | - |  |
| 0x18009D6F0 | `std::string::resize` | `(size_t, char)` | lib:STL | - |  |
| 0x18009D790 | `boost::serialization::singleton_module::unlock` | `()` | lib:boost | - |  |
| 0x18009D798 | `[thunk] portable_binary_iarchive_exception::what vbase adjustor` | `()` | lib:boost | - | vtable slot 1 |
| 0x18009D7B0 | `portable_binary_iarchive_exception::what` | `const char* () const` | boost example header | identical | "integer cannot be represented"; assert(false) at portable_binary_iarchive.hpp:57 |
| 0x18009D800 | `archive_serializer_map<portable_binary_oarchive>::erase` | `static void (const basic_serializer*)` | lib:boost | - | start of portable_binary_oarchive.cpp object |
| 0x18009D850 | `singleton<map<portable_binary_oarchive>>::get_const_instance (thunk)` | `()` | lib:boost | - |  |
| 0x18009D860 | `singleton<map<portable_binary_oarchive>>::get_instance` | `()` | lib:boost | - | static 0x18047DB98 |
| 0x18009D950 | `singleton<map<portable_binary_oarchive>>::get_mutable_instance` | `()` | lib:boost | - |  |
| 0x18009D980 | `archive_serializer_map<portable_binary_oarchive>::insert` | `static bool (const basic_serializer*)` | lib:boost | - |  |
| 0x18009D9D0 | `singleton<map<portable_binary_oarchive>>::is_destroyed` | `()` | lib:boost | - |  |
| 0x18009D9E0 | `basic_binary_oprimitive<portable_binary_oarchive,...>::save_binary` | `void (const void*, size_t)` | lib:boost | - | sputn, throws output_stream_error |
| 0x18009DA50 | `portable_binary_oarchive::save_impl` | `void (boost::intmax_t l, char maxsize)` | third-party (boost example) | identical |  |
| 0x18009DBE0 | `Eigen::Ref<const SeedState>::Ref(const Ref<SeedState>&)` | `ctor` | lib:Eigen | - | start of depth_filter.cpp object; one per seed::xxx(state) call |
| 0x18009DC50 | `std::vector<std::shared_ptr<Frame>>::_Assign_range` | `(first,last)` | lib:STL | - | vector<FramePtr>::operator= |
| 0x18009DE00 | `std::_Copy_unchecked<const shared_ptr<Frame>*>` | `(first,last,dest)` | lib:STL | - |  |
| 0x18009DEA0 | `std::vector<fast::fast_xy>::_Emplace_reallocate (ICF with vector<int>)` | `(where, const T&)` | lib:STL | - |  |
| 0x18009E090 | `std::vector<std::pair<int,fast::fast_xy>>::_Emplace_reallocate<int&,fast_xy&>` | `(where, int&, fast_xy&)` | lib:STL | - | corners.emplace_back(score, xy) |
| 0x18009E290 | `std::vector<std::shared_ptr<Frame>>::_Emplace_reallocate<const shared_ptr&>` | `(where, const FramePtr&)` | lib:STL | - |  |
| 0x18009E450 | `std::list<pair<const int,vector<fast_xy>>>::_Free_non_head` | `(alloc, head)` | lib:STL | - | unordered_map<int,vector<fast_xy>> node cleanup |
| 0x18009E4F0 | `std::thread::_Invoke<tuple<void (DepthFilter::*)(), DepthFilter*>>` | `static unsigned (void*)` | lib:STL | - | thread entry (ICF: also used by Headset and LoadSavePlatMap threads) |
| 0x18009E520 | `std::_Med3_unchecked<pair<int,fast_xy>*, bool(*)(pair,pair)>` | `(...)` | lib:STL | - | std::sort internals |
| 0x18009E5D0 | `std::_Partition_by_median_guess_unchecked<pair<int,fast_xy>*, fnptr>` | `(...)` | lib:STL | - |  |
| 0x18009E870 | `std::_Pop_heap_hole_by_index<pair<int,fast_xy>*, fnptr>` | `(...)` | lib:STL | - |  |
| 0x18009E970 | `std::_Sort_unchecked<pair<int,fast_xy>*, bool(*)(pair,pair)>` | `(first,last,ideal,pred)` | lib:STL | - |  |
| 0x18009ECA0 | `std::unordered_map<int,std::vector<fast::fast_xy>>::try_emplace / operator[]` | `(key)` | lib:STL | - | "unordered_map/set too long" |
| 0x18009EF70 | `std::remove_if<deque<FramePtr>::iterator, lambda(not in map)>` | `(first,last,pred)` | lib:STL | - | inlined lambda from DepthFilter::addKeyframe |
| 0x18009F180 | `Eigen::CwiseNullaryOp<scalar_constant_op<float>,Vector2f>::CwiseNullaryOp` | `(2,1,op)` | lib:Eigen | - | Vector2f::Constant / Zero |
| 0x18009F230 | `Eigen::CwiseNullaryOp<scalar_constant_op<float>,Matrix<float,-1,-1>>::CwiseNullaryOp` | `(rows,cols,op)` | lib:Eigen | - | block.setConstant |
| 0x18009F280 | `pimax::totem::DepthFilter::DepthFilter` | `(const DepthFilterOptions&)` | project | modified: no feature detectors, no log, startThread() inlined |  |
| 0x18009F650 | `pimax::totem::DepthFilter::DepthFilter` | `(const DepthFilterOptions&, DetectorOptions, const std::shared_ptr<CameraBundle>&)` | project | modified: delegates only, detector creation removed |  |
| 0x18009F680 | `std::unordered_map<int,vector<fast_xy>>::~unordered_map (_Hash dtor)` | `()` | lib:STL | - |  |
| 0x18009F700 | `std::_List_node_emplace_op2<pair<const int,vector<fast_xy>>>::~_List_node_emplace_op2` | `()` | lib:STL | - | EH helper |
| 0x18009F790 | `std::deque<DepthFilter::Job>::_Tidy (~deque)` | `()` | lib:STL | - |  |
| 0x18009F890 | `std::list<pair<const int,vector<fast_xy>>>::~list` | `()` | lib:STL | - |  |
| 0x18009F8C0 | `[thunk] std::deque<Job>::~deque` | `()` | lib:STL | - |  |
| 0x18009F8D0 | `std::unique_lock<std::mutex>::~unique_lock` | `()` | lib:STL | - |  |
| 0x18009F8E0 | `std::_Tidy_guard / unique_ptr<_Container_proxy> dtor` | `()` | lib:STL | - | free(ptr) helper |
| 0x18009F900 | `std::unique_ptr<std::thread>::~unique_ptr` | `()` | lib:STL | - | terminate() if joinable |
| 0x18009F930 | `[thunk] std::unordered_map<int,vector<fast_xy>>::~unordered_map` | `()` | lib:STL | - |  |
| 0x18009F940 | `pimax::totem::DepthFilter::~DepthFilter` | `()` | project | modified: stopThread() inlined with Pimax log lines |  |
| 0x18009FAF0 | `pimax::totem::DepthFilter::Job::~Job` | `()` | project (compiler-generated) | - | releases ref_frame, cur_frame, frame_bundle |
| 0x18009FBB0 | `[thunk] _Cnd_destroy_in_situ` | `()` | lib:CRT | - |  |
| 0x18009FBC0 | `pimax::totem::DepthFilter::`scalar deleting dtor'` | `(unsigned)` | project (compiler-generated) | - | vtable 0x1803B1E80 slot 0; Eigen aligned delete (free) |
| 0x18009FC10 | `pimax::totem::DepthFilter::GetFramesWithoutSeeds (name TODO)` | `bool (std::vector<FramePtr>& frames)` | project | new | copy frames_without_seeds_vec_ under jobs_mut_ |
| 0x18009FCB0 | `std::deque<DepthFilter::Job>::_Growmap` | `(size_t)` | lib:STL | - | "deque<T> too long" |
| 0x18009FE90 | `pimax::totem::DepthFilter::addKeyframe` | `void (const FrameBundlePtr&, const Map&)` | project | modified: bundle, resize by grid cells, queue swap, map pruning of no-seed frames, Job(SEED_INIT,bundle) |  |
| 0x1800A0480 | `Eigen::DenseBase<CwiseBinaryOp<cmp_EQ,Matrix3d,Matrix3d>>::all (hasNaN helper)` | `bool () const` | lib:Eigen | - | R.hasNaN() in initializeSeeds (decompiler shows `return 0`) |
| 0x1800A0520 | `depth_filter_utils::(anon)::compareCornerScore (name TODO; 6-byte function not in funcs.json)` | `bool (pair<int,fast_xy>, pair<int,fast_xy>)` | project | new | a.first > b.first (std::sort comparator, function pointer) |
| 0x1800A0530 | `pimax::totem::depth_filter_utils::computeTau` | `FloatType (const Transformation&, const BearingVector&, FloatType z, FloatType px_error_angle)` | project | modified: float maths, float return |  |
| 0x1800A0750 | `depth_filter_utils::(anon)::detectFastCorners (name TODO)` | `void (const cv::Mat& img, vector<pair<int,fast_xy>>& corners)` | project | new | FAST-10 (thr 20) + score + 3x3 NMS, sorted by score |
| 0x1800A0A60 | `pimax::totem::seed::getSigma2FromDepthSigma` | `FloatType (FloatType depth, FloatType depth_sigma)` | project (header inline, COMDAT) | modified: float, eps 1e-12f |  |
| 0x1800A0AB0 | `pimax::totem::depth_filter_utils::initializeSeeds` | `void (const FrameBundlePtr&)` | project | rewritten (Pimax) | FAST per camera, mask/grid bookkeeping, cross-camera grid blocking, seed init, LOGD |
| 0x1800A2100 | `depth_filter_utils::(anon)::isRotationMatrix (name TODO)` | `bool (const Eigen::Matrix3d&)` | project | new | abs(det-1)<=1e-6 && sqnorm(R*R^T-I)<=1e-12 |
| 0x1800A2300 | `pimax::totem::DepthFilter::reset` | `void ()` | project | modified: also clears the no-seed frame lists, no log |  |
| 0x1800A2510 | `Eigen::internal::compute_inverse<Matrix3d,...>::run (via product evaluator)` | `(...)` | lib:Eigen | - | R.inverse()*f |
| 0x1800A2730 | `pimax::totem::DepthFilter::stopThread` | `void ()` | project | modified: LOGI messages, joinable() test, "DepthFilter: end" |  |
| 0x1800A2810 | `pimax::totem::depth_filter_utils::updateFilterVogiatzis` | `bool (FloatType z, FloatType tau2, FloatType mu_range, Eigen::Ref<SeedState>&)` | project | modified: float literals, no logs, no sigma2<0 / mu<0 checks |  |
| 0x1800A2AC0 | `pimax::totem::depth_filter_utils::updateSeed` | `bool (const Frame&, Frame&, const size_t&, Matcher&, FloatType, bool, bool, bool)` | project | modified: see notes (z<0 check, static mask, outlier rules, constants) |  |
| 0x1800A3270 | `pimax::totem::DepthFilter::updateSeeds` | `size_t (const vector<FramePtr>&, const FramePtr&)` | project | modified: skips/records frames without seeds, single threshold, no log |  |
| 0x1800A3630 | `pimax::totem::DepthFilter::updateSeedsLoop` | `void ()` | project | modified: move-pop, bundle seed init, Sleep(0) |  |
| 0x1800A3900 | `std::vector<Eigen::Vector2f>::_Emplace_reallocate<Vector2f>` | `(where, Vector2f&&)` | lib:STL | - | start of feature_alignment.cpp object (each_step->push_back) |
| 0x1800A3B00 | `Eigen::LDLT<Matrix4f>::_solve_impl` | `void (const Vector4f&, Vector4f&) const` | lib:Eigen | - |  |
| 0x1800A3D60 | `Eigen::MatrixBase<Block<Matrix2f>>::applyOnTheLeft(JacobiRotation<float>)` | `(p,q,j)` | lib:Eigen | - | JacobiSVD 2x2 helper (also used by 0x1800FEBA0) |
| 0x1800A3E60 | `Eigen::MatrixBase<Matrix3f>::applyOnTheLeft(p,q,JacobiRotation<float>)` | `(p,q,j)` | lib:Eigen | - | JacobiSVD<Matrix3f> |
| 0x1800A3FB0 | `Eigen::MatrixBase<Matrix3f>::applyOnTheRight(p,q,JacobiRotation<float>)` | `(p,q,j)` | lib:Eigen | - | JacobiSVD<Matrix3f> |
| 0x1800A4100 | `Eigen::LDLT<Matrix4f>::compute` | `LDLT& (const EigenBase&)` | lib:Eigen | - |  |
| 0x1800A43C0 | `std::fill_n<float*> / Eigen setConstant kernel (float)` | `(dst, n, value)` | lib:STL/Eigen | - | COMDAT used by many |
| 0x1800A4480 | `Eigen::JacobiRotation<float>::makeJacobi` | `bool (const MatrixBase&, p, q)` | lib:Eigen | - | used by 0x1800D6330/0x1800D6570 |
| 0x1800A45D0 | `Eigen::DenseBase<Diagonal<Matrix4f>>::maxCoeff(Index*) (cwiseAbs visitor)` | `float (Index*)` | lib:Eigen | - | LDLT pivot search |
| 0x1800A4760 | `Eigen::DenseBase<...float...>::maxCoeff(Index*) visitor` | `float (Index*)` | lib:Eigen | - | JacobiSVD / 0x1800FEBA0 |
| 0x1800A48A0 | `Eigen::DenseBase<Vector4f-like>::maxCoeff (redux)` | `float () const` | lib:Eigen | - |  |
| 0x1800A4970 | `Eigen::internal::real_2x2_jacobi_svd<Matrix3f,float,Index>` | `void (const Matrix3f&, p, q, JacobiRotation*, JacobiRotation*)` | lib:Eigen | - |  |
| 0x1800A4C90 | `Eigen::internal::ldlt_inplace<Lower>::unblocked helper (abs-sum redux)` | `(...)` | lib:Eigen | - |  |
| 0x1800A4E70 | `Eigen::internal::swap_assign (row/column swap kernel, float)` | `(...)` | lib:Eigen | - | LDLT transpositions; also used by 0x1800FF740/0x180102720 |
| 0x1800A4F20 | `Eigen::internal::div_assign kernel (float block /= scalar)` | `(...)` | lib:Eigen | - | LDLT |
| 0x1800A4FB0 | `Eigen::internal::sub_assign kernel (float block -= product)` | `(...)` | lib:Eigen | - | LDLT |
| 0x1800A5160 | `Eigen::TriangularView<Matrix4f,UnitLower>::solveInPlace` | `(Vector4f&)` | lib:Eigen | - | LDLT solve |
| 0x1800A5240 | `Eigen::TriangularView<Matrix4f^T,UnitUpper>::solveInPlace` | `(Vector4f&)` | lib:Eigen | - | LDLT solve |
| 0x1800A5330 | `Eigen::internal::ldlt_inplace<Lower>::unblocked<Matrix4f,...>` | `bool (Matrix4f&, Transpositions&, Vector4f&, SignMatrix&)` | lib:Eigen | - |  |
| 0x1800A6300 | `Eigen::internal::variable_if_dynamic<Index,4>/Map ctor helper (1x4 block)` | `(...)` | lib:Eigen | - | ICF-shared with Ceres |
| 0x1800A63B0 | `Eigen::SVDBase<JacobiSVD<Matrix3f>>::_check_compute_assertions` | `void () const` | lib:Eigen | - |  |
| 0x1800A63E0 | `pimax::totem::feature_alignment::align1D` | `bool (const cv::Mat&, const Ref<GradientVector>&, uint8_t*, uint8_t*, int, bool, bool, Keypoint*, double*)` | project | rewritten (Pimax) | weighted IRLS 1-D alignment, see notes |
| 0x1800A84F0 | `pimax::totem::feature_alignment::align2D` | `bool (const cv::Mat&, uint8_t*, uint8_t*, int, bool, bool, Keypoint&, bool, std::vector<Vector2f>*)` | project | modified: float literals, LDLT solve instead of inverse |  |
| 0x1800A9250 | `Eigen::JacobiSVD<Matrix3f>::allocate` | `void (Index rows, Index cols, unsigned opts)` | lib:Eigen | - |  |
| 0x1800A9410 | `Eigen::JacobiSVD<Matrix3f>::compute` | `JacobiSVD& (const MatrixType&, unsigned)` | lib:Eigen | - |  |
| 0x1800A9E70 | `Eigen::SVDBase<JacobiSVD<Matrix3f>>::computeU` | `bool () const` | lib:Eigen | - |  |
| 0x1800A9E90 | `Eigen::SVDBase<JacobiSVD<Matrix3f>>::computeV` | `bool () const` | lib:Eigen | - |  |
| 0x1800A9EB0 | `Eigen::MatrixBase<Matrix3f>::determinant` | `float () const` | lib:Eigen | - |  |
| 0x1800A9F40 | `Eigen::internal::compute_inverse<Matrix3f,Matrix3f,3>::run` | `(const Matrix3f&, Matrix3f&)` | lib:Eigen | - |  |
| 0x1800AA150 | `pimax::totem::AbstractDetector::AbstractDetector` | `(const DetectorOptions&, const CameraPtr&)` | project | identical | start of feature_detection.cpp object |
| 0x1800AA2F0 | `pimax::totem::OccupandyGrid2D::OccupandyGrid2D` | `(int cell_size, int n_cols, int n_rows)` | project (header inline, COMDAT) | identical |  |
| 0x1800AA430 | `[thunk] std::vector<Corner>::~vector` | `()` | lib:STL | - |  |
| 0x1800AA440 | `pimax::totem::OccupandyGrid2D::~OccupandyGrid2D` | `()` | project (compiler-generated) | - | frees feature_occupancy_ and occupancy_ |
| 0x1800AA4F0 | `pimax::totem::AbstractDetector::`scalar deleting dtor'` | `(unsigned)` | project (compiler-generated) | - | vtable slot 0 of AbstractDetector and FastGradDetector |
| 0x1800AA540 | `std::allocator<unsigned int>::allocate (vector<bool> words)` | `(size_t)` | lib:STL | - | COMDAT shared with Ceres |
| 0x1800AA5D0 | `std::allocator<Keypoint>::allocate (8-byte elements)` | `(size_t)` | lib:STL | - |  |
| 0x1800AA660 | `std::vector<Corner>::_Tidy` | `()` | lib:STL | - |  |
| 0x1800AA6E0 | `pimax::totem::FastGradDetector::detect` | `void (const ImgPyr&, const cv::Mat&, size_t, Keypoints&, Scores&, Levels&, Gradients&, FeatureTypes&)` | project | modified: two FAST passes, mask in fastDetector, edgelet switch/threshold | vtable 0x1803B27D0 slot 1 |
| 0x1800AAB80 | `pimax::totem::OccupandyGrid2D::reset` | `void ()` | project (header inline, COMDAT) | identical | std::fill(occupancy_, false) |
| 0x1800AAC30 | `std::_Buffered_inplace_merge_divide_and_conquer2<size_t*, lambda>` | `(...)` | lib:STL | - | start of feature_detection_utils.cpp object; std::stable_sort internals (fillFeatures) |
| 0x1800AADF0 | `std::_Buffered_inplace_merge_unchecked<size_t*, lambda>` | `(...)` | lib:STL | - |  |
| 0x1800AAF20 | `std::_Buffered_inplace_merge_divide_and_conquer<size_t*, lambda>` | `(...)` | lib:STL | - |  |
| 0x1800AB240 | `std::_Uninitialized_chunked_merge_unchecked2<size_t*, lambda>` | `(...)` | lib:STL | - |  |
| 0x1800AB490 | `std::_Chunked_merge_unchecked<size_t*, lambda>` | `(...)` | lib:STL | - |  |
| 0x1800AB580 | `std::vector<Keypoint,Eigen::aligned_allocator>::_Emplace_reallocate<const int&,const int&>` | `(where, x, y)` | lib:STL | - | keypoint_vec.emplace_back(c.x, c.y) |
| 0x1800AB700 | `std::vector<GradientVector,Eigen::aligned_allocator>::_Emplace_reallocate<float,float>` | `(where, cos, sin)` | lib:STL | - |  |
| 0x1800AB880 | `std::_Insertion_sort_unchecked<size_t*, lambda>` | `(...)` | lib:STL | - |  |
| 0x1800AB960 | `std::_Stable_sort_unchecked<size_t*, lambda>` | `(first,last,count,buf,cap,pred)` | lib:STL | - |  |
| 0x1800ABA70 | `std::basic_string<char>::basic_string(const char*)` | `ctor` | lib:STL | - | COMDAT (OpenCV persistence, loop closing ...) |
| 0x1800ABAB0 | `[thunk] cv::Mat::~Mat` | `()` | lib:OpenCV | - |  |
| 0x1800ABAC0 | `struct-with-3-cv::Mat destructor (+16,+112,+208)` | `()` | lib/compiler-generated | - | unwind helper (loop closing) |
| 0x1800ABAF0 | `cv::FileNode / _InputArray::getMat-style inline helper (returns Mat)` | `cv::Mat (…)` | lib:OpenCV inline | - | calls vtable+16 with (-1, 1); used by calibration loader |
| 0x1800ABB50 | `edgeletDetector_V2::<lambda #2>::operator()` | `void (const cv::Range&) const` | project | modified (parallelised nonmax suppression) | 8-neighbour NMS per row, grid/corner update |
| 0x1800ABDE0 | `std::_Func_impl_no_alloc<lambda #2,void,const cv::Range&>::_Copy` | `(void*) const` | lib:STL | - |  |
| 0x1800ABE30 | `std::_Func_impl_no_alloc<lambda #1,void,const cv::Range&>::_Copy` | `(void*) const` | lib:STL | - |  |
| 0x1800ABE80 | `std::_Func_impl_no_alloc<lambda #2>::_Delete_this` | `(bool)` | lib:STL | - |  |
| 0x1800ABE90 | `std::_Func_impl_no_alloc<lambda #1>::_Delete_this` | `(bool)` | lib:STL | - |  |
| 0x1800ABEA0 | `std::_Func_impl_no_alloc<lambda #2>::_Do_call` | `void (const cv::Range&)` | lib:STL | - | calls 0x1800ABB50 |
| 0x1800ABEB0 | `std::_Func_impl_no_alloc<lambda #1>::_Do_call (lambda #1 body inlined)` | `void (const cv::Range&)` | project (lambda body) | modified | magnitude/angle per row (edgeletDetector_V2) |
| 0x1800AC070 | `std::_Ref_count_resource<FastGradDetector*,std::default_delete<FastGradDetector>>::_Get_deleter` | `void* (const type_info&) const` | lib:STL | - | vtable 0x1803B2808 slot 3 |
| 0x1800AC0A0 | `std::vector<Keypoint/GradientVector,aligned_allocator>::_Reallocate_exactly (reserve)` | `(size_t)` | lib:STL | - |  |
| 0x1800AC140 | `std::_Func_impl_no_alloc<lambda #2>::_Target_type` | `() const` | lib:STL | - |  |
| 0x1800AC150 | `std::_Func_impl_no_alloc<lambda #1>::_Target_type` | `() const` | lib:STL | - |  |
| 0x1800AC160 | `std::vector<bool>::_Xran` | `[[noreturn]]` | lib:STL | - | "invalid vector<bool> subscript" |
| 0x1800AC180 | `Eigen::aligned_allocator<8-byte T>::allocate` | `(size_t)` | lib:Eigen | - |  |
| 0x1800AC1F0 | `__crt_internal_free_policy::operator()` | `(const wchar_t*)` | lib:CRT | - |  |
| 0x1800AC200 | `pimax::totem::feature_detection_utils::edgeletDetector_V2` | `void (const ImgPyr&, int, int, int, int, Corners&, OccupandyGrid2D&)` | project | modified: no blur, parallel_for_, stride fix, angle only for strong pixels |  |
| 0x1800AC5E0 | `pimax::totem::feature_detection_utils::fastDetector` | `void (const ImgPyr&, int, int, size_t, size_t, const cv::Mat& mask, Corners&, OccupandyGrid2D&)` | project | modified: mask filtering, saturation rejection, int cell index, no CHECKs |  |
| 0x1800ACB30 | `pimax::totem::feature_detection_utils::fillFeatures` | `void (const Corners&, const FeatureType&, const double&, size_t, Keypoints&, Scores&, Levels&, Gradients&, FeatureTypes&, OccupandyGrid2D&)` | project | modified: no mask, no CHECKs, stable_sort, 2-arg emplace_back |  |
| 0x1800AD4A0 | `pimax::totem::feature_detection_utils::getAngleAtPixelUsingHistogram` | `float (const cv::Mat&, const Eigen::Vector2i&, size_t)` | project | modified: float histogram/result, float pi2 | angleHistogram/gradientAndMagnitudeAtPixel/getDominantAngle inlined |
| 0x1800AD840 | `pimax::totem::feature_detection_utils::makeDetector` | `AbstractDetector::Ptr (const DetectorOptions&, const CameraPtr&)` | project | modified: always FastGradDetector |  |
| 0x1800AD920 | `cv::parallel_for_(const Range&, std::function<void(const Range&)>, double)` | `inline OpenCV wrapper` | lib:OpenCV inline | - | ParallelLoopBodyLambdaWrapper |
| 0x1800ADA80 | `pimax::totem::feature_detection_utils::angle_hist::smoothOrientationHistogram` | `void (AngleHistogram&)` | project | modified: float histogram |  |
| 0x1800ADD10 | `std::_Integral_to_string<char,int> (std::to_string(int))` | `std::string (int)` | lib:STL | - | COMDAT used by loop closing; first entry of the matcher.cpp object |
| 0x1800ADDF0 | `Eigen::Ref<const Vector2d>::Ref(const CwiseUnaryOp<cast<float,double>,Vector2f>&)` | `ctor` | lib:Eigen | - | px.cast<double>() passed to camera API |
| 0x1800ADE80 | `Eigen::Ref<const Vector3d>::Ref(cast<float,double> of Vector3f)` | `ctor` | lib:Eigen | - |  |
| 0x1800ADEF0 | `Eigen::Ref<const Vector2f>/Ref<GradientVector>::Ref(Vector2f&)` | `ctor` | lib:Eigen | - |  |
| 0x1800ADF30 | `pimax::totem::patch_score::ZMSSD<4>::ZMSSD` | `(uint8_t* ref_patch)` | project (header template) | identical | sumA_, sumAA_ (auto-vectorised + scalar fallback) |
| 0x1800AE070 | `Eigen::internal::evaluator<Inverse<Matrix2d>>::evaluator` | `ctor` | lib:Eigen | - |  |
| 0x1800AE150 | `matcher_utils::depthFromTriangulation::<lambda>::operator()` | `Matcher::MatchResult () const` | project | identical body (upstream least-squares solve), now a fallback |  |
| 0x1800AE5A0 | `pimax::totem::patch_utils::createPatchFromPatchWithBorder` | `void (const uint8_t*, int, uint8_t*)` | project (header inline) | identical |  |
| 0x1800AE610 | `pimax::totem::matcher_utils::depthFromTriangulation` | `Matcher::MatchResult (const Transformation&, const Vector3d&, const Vector3d&, FloatType*)` | project | modified: closed form + fallback lambda, float depth |  |
| 0x1800AE820 | `pimax::totem::Matcher::findEpipolarMatchDirect` | `MatchResult (const Frame&, const Frame&, const Transformation&, const FeatureWrapper&, double, double, double, FloatType&)` | project | modified: see notes |  |
| 0x1800AF4B0 | `pimax::totem::Matcher::findLocalMatch` | `MatchResult (const Frame&, const Ref<GradientVector>&, int, Keypoint&)` | project | identical (float keypoint) |  |

Notes on the table: "project (header inline, COMDAT)" = code from a project header that the
compiler emitted out of line in this object (it has to exist in the rebuilt source, normally as
the inline header function). All `lib:*` rows need no source.

## Object boundaries / naming evidence

- Point ctor 0x180099F80 / dtor 0x18009A160 sit before the chunk; `?addObservation@Point` is the
  first `?`-name of point.obj (all `??0/??1` names sort before it).
- Inside each object the functions are ordered by decorated name (same observation as chunk
  c07). Checked on depth_filter.obj: `??$..` Eigen/STL templates, `??0DepthFilter` (two ctors),
  `??1...` dtors, `??_GDepthFilter`, **0x18009FC10**, `?_Growmap@deque`, `?addKeyframe`, `?all`
  (hasNaN), **0x1800A0520**, `?computeTau`, **0x1800A0750**, `?getSigma2FromDepthSigma`,
  `?initializeSeeds`, **0x1800A2100**, `?reset`, `?run@compute_inverse`, `?stopThread`,
  `?updateFilterVogiatzis`, `?updateSeed`, `?updateSeeds`, `?updateSeedsLoop`.  Hence the
  unnamed ones: 0x18009FC10 starts with an upper-case letter (named `GetFramesWithoutSeeds`
  here), 0x1800A0520 sorts between `all` and `computeTau` (`compareCornerScore`), 0x1800A0750
  between `computeTau` and `getSigma2...` (`detectFastCorners`), 0x1800A2100 between
  `initializeSeeds` and `reset` (`isRotationMatrix`).  Same pattern in
  feature_detection_utils.obj (`edgeletDetector_V2 < fastDetector < fillFeatures <
  getAngleAtPixelUsingHistogram < makeDetector < parallel_for_ < smoothOrientationHistogram`)
  and matcher.obj (`createPatchFromPatchWithBorder < depthFromTriangulation <
  findEpipolarMatchDirect < findLocalMatch < findMatchDirect`).
- 0x1800ADD10 (`std::_Integral_to_string<char,int>`) does not fit the sort order of matcher.obj;
  it is a COMDAT only referenced by loop closing. Boundary between feature_detection_utils.obj
  and matcher.obj is therefore 0x1800ADD10 ± that one function.

## Types

All offsets are x64 MSVC byte offsets. "sure" = directly observed loads/stores.

### FloatType
Pimax `svo/common/types.h` has `FloatType = float` (sure): Keypoint/BearingVector/GradientVector/
Position/SeedState are float, Keypoints/Bearings/Gradients/SeedStates are `Matrix<float,...>`,
Scores = VectorXf. Same conclusion as c12.

### `pimax::totem::Point` — sizeof 0x88 (make_shared allocates 0x98) — no vtable
| off | type | name | evidence |
|---|---|---|---|
| 0 | int | id_ | ctor `lock xadd 0x18047DB60` (sure) |
| 4 | Vector3f | pos_ | float loads +4/+8/+12 everywhere (sure) |
| 16 | std::unordered_map<int, KeypointIdentifier> | obs_ | FNV-1a hash on int key, list head +24, size +32, buckets +40, mask +64 (sure). c12's "list at +24" is the map's internal list |
| 80 | uint64 | ? (=0) | ctor (c12: n_failed_reproj_ int at +80) TODO |
| 88 | int | ? (=0) | ctor |
| 96 | std::set<int>? (node 0x20) | ? | ctor/dtor TODO |
| 112 | uint64 | ? (=0) | ctor |
| 120 | int | ? (=0) | ctor |
| 124 | int | ? (=-1) | ctor (last_ba_update_ / last_structure_optim_?) |
| 128 | bool | ? (=false) | ctor (in_ba_graph_?) |

### `KeypointIdentifier` — 32 bytes (map node 0x38: next, prev, key +16, value +24)
| off | type | name | evidence |
|---|---|---|---|
| 0 | std::weak_ptr<Frame> | frame | weak count inc (sure) |
| 16 | int | frame_id | `frame->id_` (Frame+16) (sure) |
| 20 | int | bundle_id (Pimax-new, name TODO) | `frame+32`, which FrameBundle copies from its +252 (0x1800933E0) (sure) |
| 24 | size_t | keypoint_index_ | (sure) |

### `Frame` — members used by this chunk (owned elsewhere)
| off | type | name used here | evidence |
|---|---|---|---|
| 0 | vptr | | vtable 0x1803B1588 |
| 16 | int | id_ | frame_counter_ 0x18047DB38 |
| 20 | int | cam_index_ (name TODO) | ctor arg 6; used as key / index into FrameBundle::frames_ in initializeSeeds |
| 32 | int | bundle_id_ | set from FrameBundle (+252) |
| 36 | int | nframe_index_ | ctor -1, set by the frame processor |
| 40 | CameraPtr | cam_ | |
| 64 | Transformation | T_f_w_ | quaternion xyzw (+64..+95), position (+96..+119) |
| 128 | std::vector<cv::Mat> | img_pyr_ | `img_pyr_[0].rows/cols` (+8/+12 of the Mat) |
| 224 | double | mean_intensity_ (c12 name) | `>= 15.0` gate for FAST in initializeSeeds |
| 384 | uint16 | grid_cell_size_ (name TODO) | divides pixel coords; Frame ctor arg 10 = FrameProcessorBase+272 |
| 392 | std::vector<bool> | grid_occupancy_ (name TODO) | bit set/test (word vector +392..+415, bit count +416) |
| 424 | float | depth_min_ (name TODO) | written by 0x180094xxx (min scene depth) |
| 428 | float | depth_median_ (name TODO) | written by the same function via 0x180091700 (median of depths) |
| 456 | cv::Mat | feature_mask_ (name TODO) | clone of `cam_->mask_`, circles drawn, released after use |
| 552 | size_t | num_features_ | |
| 560 | Keypoints (Matrix2Xf) | px_vec_ | data +560, cols +568 |
| 576 | Bearings (Matrix3Xf) | f_vec_ | data +576, cols +584 |
| 608 | Scores (VectorXf) | score_vec_ | size +616 |
| 624 | Levels (VectorXi) | level_vec_ | size +632 |
| 640 | Gradients (Matrix2Xf) | grad_vec_ | cols +648 |
| 656 | std::vector<FeatureType> | type_vec_ | |
| 744 | SeedStates (Matrix4Xf) | invmu_sigma2_a_b_vec_ | cols +752 |
| 760 | float | seed_mu_range_ | |
Member functions called (other chunks): getAngleError(double) 0x180094540, getFeatureWrapper(size_t)
0x180094580, getMask() 0x180094880 (`return cam_->mask_`), isSaturated(const Vector2i&) 0x180094EC0
(name TODO: 5x5 window clamped to the image, true if >80% of pixels > 220), resizeFeatureStorage
0x180095730, pos() (inlined: `T_f_w_.inverse().getPosition().cast<float>()`), cam() (inlined).

### `FrameBundle`
+0 `std::vector<FramePtr> frames_` (sure).

### `FeatureWrapper` offsets used (c12 layout confirmed)
+0 `FeatureType& type`, +8 px (Ref<Keypoint>), +32 f (Ref<BearingVector>), +80 grad
(Ref<GradientVector>), +112 `Level& level`.

### `vk::cameras::CameraGeometryBase` offsets used
+8 width (int), +12 height (int) (AbstractDetector ctor), +16 `std::string label_` (LOGD in
initializeSeeds), +56 `cv::Mat mask_`; vtable +0x10 `bool backProject3(const Ref<const Vector2d>&,
Vector3d*) const`, +0x18 `ProjectionResult project3(const Ref<const Vector3d>&, Vector2d*,
Matrix<double,2,3>*) const` (ProjectionResult returned through a hidden pointer, status 0 =
visible).

### `Map`
Only `keyframes_` at +0 (`std::map<int, FramePtr>`, matches c11) is used (DepthFilter::addKeyframe).

### `pimax::totem::DepthFilter` — sizeof 0x160, EIGEN aligned new, vtable 0x1803B1E80 [0x18009FBC0]
| off | type | name | evidence |
|---|---|---|---|
| 0 | vptr | | |
| 8 | DepthFilterOptions (56 B) | options_ | copied with 3 xmm + 1 qword (sure) |
| 64 | std::mutex | jobs_mut_ | `_Mtx_init_in_situ(+64, 2)` (sure) |
| 144 | std::queue<Job> (deque) | jobs_ | proxy alloc, Job dtor 0x18009FAF0 (sure) |
| 184 | std::deque<FramePtr> | frames_without_seeds_ (Pimax-new, name TODO) | (sure) |
| 224 | std::vector<FramePtr> | frames_without_seeds_vec_ (Pimax-new, name TODO) | (sure) |
| 248 | std::condition_variable | jobs_condvar_ | `_Cnd_init_in_situ` (sure) |
| 320 | std::unique_ptr<std::thread> | thread_ | (sure) |
| 328 | bool | quit_thread_ | (sure) |
| 336 | std::shared_ptr<Matcher> | matcher_ | `_Ref_count<Matcher>` (sure) |
Upstream feature_detector_mut_, feature_detector_, sec_feature_detector_ do not exist.

### `DepthFilterOptions` — 56 bytes
+0 float seed_convergence_sigma2_thresh (read by updateSeeds/updateSeedsLoop, sure), +4 float
mappoint_convergence_sigma2_thresh (inferred), +8 bool use_inverse_depth, +16 size_t
max_search_level, +24 bool verbose, +25 bool use_threaded_depthfilter (sure), +26 bool
update_3d_point, +27 bool scan_epi_unit_sphere (sure), +32 size_t max_n_seeds_per_frame, +40 size_t
max_map_seeds_per_frame, +48 bool affine_est_offset (sure), +49 bool affine_est_gain (sure), +50
bool extra_map_points.  = upstream order with the two thresholds as FloatType. Defaults: not set
in this chunk.

### `DepthFilter::Job` — 80 bytes (deque element)
+0 Type (UPDATE=0, SEED_INIT=1), +8 FrameBundlePtr frame_bundle (Pimax-new), +24 FramePtr
cur_frame, +40 FramePtr ref_frame, +56 size_t ref_frame_seed_index, +64/+68/+72 float
min/max/mean_depth (SEED_INIT sets all to 1.0f, nobody reads them; UPDATE leaves them
uninitialised).

### `Matcher` — corrections/additions for c07's matcher.h
- Constructed inline in the DepthFilter ctor (malloc 0x160 = sizeof 352, memset 0 then):
  +4 align_max_iter = 10, +8 max_epi_length_optim = 2.0, +16 max_epi_search_steps = 100,
  +24 subpix_refinement = true, +25 epi_search_edgelet_filtering = true, +26
  scan_on_unit_sphere = true, **+32 epi_search_edgelet_max_angle = 0.5** (c07 assumed 0.7),
  +40 verbose = false, +41 use_affine_warp_ = true, +42 affine_est_offset_ = true, +43
  affine_est_gain_ = false, **+48 max_patch_diff_ratio = 2.5** (c07 assumed 2.0); +320..+343
  zeroed. These are the Matcher default member initialisers (TODO(verify) with c07).
- **sizeof(Matcher) = 352**: there is a second Vector3f at +332 (findEpipolarMatchDirect copies
  the unnormalised `f_cur_` there before normalising): named `f_cur_unnormalized_` here.
- `findEpipolarMatchDirect(..., FloatType& depth)` and
  `matcher_utils::depthFromTriangulation(..., FloatType* depth)` — the depth out-parameter is
  **float** (updateSeed passes a float local, depthFromTriangulation stores with cvtpd2ps).
- 0x1800AF4B0 is `Matcher::findLocalMatch` (align1D/align2D dispatch), not a second
  findEpipolarMatchDirect overload; the T-less findEpipolarMatchDirect overload is not in the
  image.
- +280/+288 (c07's `epi_length_pyramid_ca_`/`_cb_`) = |px_A − px_C| / 2^level and
  |px_C − px_B| / 2^level, where C is the projection at the current depth estimate.

### `DetectorOptions` — 80 bytes (defaults from 0x18015DAD0, other chunk)
| off | type | name | default |
|---|---|---|---|
| 0 | size_t | cell_size | 20 |
| 8 | int | max_level | 2 |
| 12 | int | min_level | 0 |
| 16 | int | border | 8 |
| 20 | DetectorType | detector_type | not written (ignored by makeDetector) |
| 24 | double | threshold_primary | 20.0 (FAST pass 1, strong-corner count) |
| 32 | double | threshold_secondary | 10.0 (FAST pass 2, corner score threshold) |
| 40 | bool | disable_edgelets (Pimax-new, name TODO) | false |
| 48 | double | threshold_edgelet (Pimax-new, name TODO) | 200.0 |
| 56 | int,int | sampling_level, level | 0, 0 (one qword store) |
| 64 | size_t | sec_grid_fineness | 1 |
| 72 | double | threshold_shitomasi | 50.0 |

### `AbstractDetector` / `FastGradDetector` — sizeof 0xE8 (plain new)
+0 vptr, +8 DetectorOptions options_, +88 OccupandyGrid2D grid_, +160 OccupandyGrid2D
closeness_check_grid_. vtables: AbstractDetector 0x1803B2720 [0x1800AA4F0, _purecall],
FastGradDetector 0x1803B27D0 [0x1800AA4F0, 0x1800AA6E0]. Owned through
`shared_ptr` built from a `unique_ptr` (`_Ref_count_resource<FastGradDetector*,
default_delete<FastGradDetector>>`, vtable 0x1803B2808).

### `OccupandyGrid2D` — 72 bytes
+0 int cell_size, +4 int n_cols, +8 int n_rows, +16 std::vector<bool> occupancy_ (size at +40),
+48 std::vector<Keypoint> feature_occupancy_. Pimax getCellIndex = `(scale*x)/cell_size +
n_cols*((scale*y)/cell_size)` in int arithmetic.

### `Corner` — 20 bytes: x, y, level (int), score, angle (float).

### `angle_hist::AngleHistogram` — `std::array<float, 36>` (Pimax: float).

### Transformation semantics (owned by the minkindr fork chunk)
`cur_frame.T_f_w_ * ref_frame.T_f_w_.inverse()` in updateSeed compiles to: copy of each operand
with its quaternion renormalised (inline `if (|q|^2 > 0) q /= |q|`), inverse 0x180013040, product
0x180009B80 which renormalises its result. `QuatTransformationTemplate<double>::cast<float>()` is
0x18008A270, `RotationQuaternionTemplate<double>::cast<float>()` 0x18008A460 (matrix → float
quaternion → normalise, i.e. upstream minkindr), `RotationQuaternionTemplate<float>::rotate`
0x180095ED0.

## External interfaces

Calls out of the chunk (project/semi-project; Eigen/STL/OpenCV/CRT helpers omitted unless
interesting):
| address | meaning |
|---|---|
| 0x18000C120 / 0x18000F500 / 0x18000C2C0 | LOGD / LOGI / LOGE (logger 0x18046A000) |
| 0x180013040 | QuatTransformation<double>::inverse |
| 0x180014A20 | RotationQuaternion<double>::rotate (Eigen q*v) |
| 0x1800423F0 | QuatTransformation<double>::getRotationMatrix (→ 0x180029DF0 toRotationMatrix) |
| 0x180009B80 | QuatTransformation<double>::operator* (renormalising) |
| 0x18008A270 / 0x18008A460 / 0x180095ED0 | Transformation cast<float>, Rotation cast<float>, float rotate |
| 0x180094540 | Frame::getAngleError(double) |
| 0x180094580 | Frame::getFeatureWrapper(size_t) |
| 0x180094880 | Frame::getMask() |
| 0x180094EC0 | Frame::isSaturated(const Eigen::Vector2i&) (name ours) |
| 0x180095730 | Frame::resizeFeatureStorage(size_t) |
| 0x1801AFA30 / 0x1801AF880 / 0x1801AFA50 | rpg fast: corner_detect_10(_sse2) wrapper (w<22 → plain, h<7 → nothing), fast_corner_score_10, fast_nonmax_3x3 |
| 0x1800B0600 / 0x1800B05B0 / 0x1800B0C10 | warp::getWarpMatrixAffine / getBestSearchLevel / warpAffine (c07) |
| 0x1800AFD70 | Matcher::scanEpipolarUnitPlane (c07) |
| 0x1800B0240 | std::vector<cv::Mat>::size (c07, lib) |
| 0x180097F70 / 0x180097FF0 / 0x18009A480 / 0x1800981E0 / 0x1800985A0 / 0x180098860 | (previous chunk, lib) vector<pair<FramePtr,size_t>> helpers, LDLT<Matrix3d> solve/compute, unordered_map<int,KeypointIdentifier> emplace |
| 0x18035C050 / 0x18035C120 / 0x18035C770 / 0x18035C880 / 0x18035CA50 / 0x18035CA90 / 0x18035CBE0 / 0x18035CC80 / 0x18035C890 | boost serialization library (archive_exception copy/ctor/dtor/what, BOOST_ARCHIVE_SIGNATURE, serializer map erase/find/insert, codecvt_null<wchar_t> ctor) |
| cv:: | circle, Mat::clone/empty/release/operator=, Scharr, parallel_for_, ParallelLoopBody::~ParallelLoopBody |

Callers into the chunk (for the coordinator):
| function | callers |
|---|---|
| Point::addObservation | 0x180128FA0 (seeds → features), 0x180147110 (stereo triangulation) |
| Point::getCloseViewObs | 0x1801422D0 (reprojector) |
| Point::getTriangulationParallax | 0x180010440 (ceres backend, new landmarks) |
| Point::optimize | 0x1801188C0, 0x180118D80 (frame_processor_base) |
| Point::removeObservation | 0x18002A6D0 (estimator), 0x1800B49A0, 0x18012E9F0 (map), 0x18013D8B0 |
| DepthFilter(options, DetectorOptions, cams) | 0x1800DFDF0 (FrameProcessorBase ctor) |
| DepthFilter::addKeyframe(bundle, *map_) | 0x1800B2F80, 0x1800B4180 |
| DepthFilter::updateSeeds | 0x1800B2F80, 0x1800B4290 |
| DepthFilter::GetFramesWithoutSeeds | 0x1800B2F80 |
| DepthFilter::reset | 0x18011B420, 0x18011B8F0 |
| DepthFilter::stopThread | 0x18016A280 |
| depth_filter_utils::updateSeed | 0x1801422D0 (reprojector), updateSeedsLoop, updateSeeds |
| feature_detection_utils::makeDetector | 0x1800B2460, 0x18012B4E0, 0x1801A7DF0 |
| feature_detection_utils::getAngleAtPixelUsingHistogram | 0x180128FA0 too |
| Matcher::findEpipolarMatchDirect | updateSeed, 0x180147110, 0x18018D770 (loop closing) |
| Matcher::findLocalMatch | findEpipolarMatchDirect, 0x1800AF5E0 |
| feature_alignment::align1D / align2D | findLocalMatch, 0x1800AF5E0 |

## Globals

| address | meaning |
|---|---|
| 0x18046A000 | logger instance |
| 0x18046A158 / 0x18046A15C | `int` 4 / 8 — align1D half patch / patch size (non-const .data, Pimax-new) |
| 0x18047DC40 | `float[64]` align1D Gaussian weights |
| 0x18047DD40 | `bool` weights initialised (plain flag) |
| 0x18047DBC8 / 0x18047DBCC | updateSeed `static FloatType px_error_angle` + init guard |
| 0x18047DBD0 / 0x18047DC30 | updateSeed `static cv::Mat mask` (clone of the first camera's mask) + guard; dtor registered with atexit |
| 0x18047DB78 / 0x18047DB98 | boost singletons map<portable_binary_iarchive/oarchive> |
| 0x18047DB60 | PointIdProvider::last_id_ (ctor, previous chunk) |

## Constants / config defaults

- Point: getCloseViewObs start −1.1 (0x1803B19C0), accept ≥ 0.5; optimize eps 1e-10, rollback
  if |dp|² > 1.0; jacobian_xyz2f factor −1.0 (0x1803B19D0).
- seed: getInvMaxDepth floor 0.01f (0x1803B1F74), getSigma2FromDepthSigma eps 1e-12f
  (0x1803B1F70), isConverged factor 0.5 (0x1803AF2B8), init sigma2 = r²/36.0 (0x1803B1F80), a=b=10.0f.
- initializeSeeds: FAST threshold 20 (detect & score), brightness gate mean_intensity_ ≥ 15.0
  (0x1803AF8C8), max 256 features per frame (`> 0xFF` break), mask circles radius 5 (existing
  features) / 3 (new seeds), depth_min factor 0.4f (0x1803B1F78), rotation check |det−1| ≤ 1e-6,
  ||RRᵀ−I||² ≤ 1e-12.
- updateSeed: px_noise 1.0, initial depth 100.0f (0x1803B1F90), outlier if b > 16.5f
  (0x1803B1F88) or depth > 50.0f (0x1803B1F8C).
- Vogiatzis: float literals 1.0f, 2.0f (0x1803B1F7C); normPdf uses sqrt(2π) in double.
- computeTau: π (0x1803B1390) in double.
- align1D: σ = N/3 (3.0f 0x1803B26B0), centre N·0.5−0.5, Sobel/8 (0.125f 0x1803B2694),
  regulariser 1e-5f (0x1803B2688), det threshold 1e-10f (0x1803B267C), SVD cut 1e-6f
  (0x1803B2684), chi2 initial DBL_MAX, rel. change 1e-4 (+1e-10), divergence ×1.5 after iter 3,
  Huber 5.0f after iter 2, alpha ∈ [0.5, 2.0], mean_diff ∈ [−30, 30], step² < 5e-7 converged,
  step² > 0.01f half-step back, patch mean ∈ [10, 245] (0x1803B12F0 / 0x1803B26B8), 1/64.
- align2D: 0.5f, min_update² 0.0009f (0x1803B268C).
- FastGradDetector: second FAST pass only if 8 < n_strong < max_n_features.
- fastDetector: saturation radius 2/1/0 for level 0/1/≥2, pixel > 200, fraction > 0.8.
- edgeletDetector_V2: angle factor 10/(2π) = 1.5915494f.
- angle histogram: 36 bins, pi2 = (float)2π (6.2831854820…), smoothing 0.25/0.5/0.25.
- depthFromTriangulation: det threshold 1e-6, borderline |X−1e-6| ≤ 1e-12 → fallback.
- findEpipolarMatchDirect: zmssd threshold 128000, getWarpMatrixAffine depth 1/max(1e-6, d),
  epi length 2.0 for the local search.

## Quirks / bugs to preserve

1. updateSeed initialises `depth = 100.0f` and marks the seed as outlier when
   `depth > 50.0f` after a failed match. The matcher only writes `depth` on success, so **every
   failed epipolar match turns the seed into an outlier** (the b-counter test is redundant).
2. updateSeed's visibility test uses a function-static clone of the mask of the camera seen in
   the *first* call (wrong for the second camera if masks differ); px_error_angle likewise static.
3. initializeSeeds: `visited_cams` is never cleared and gets one push per (corner, camera) pair;
   the other-camera grid blocking projects `R_k * R_c^{-1} f + t_k` (a ray direction without depth
   or camera centre) and then marks the cell of the *original* pixel using the *current* camera's
   cell size/width; `frame->T_f_w_` quaternion is renormalised in place each time; the log prints
   `n_new + 1`; px_vec_ is written before backProject3 and stays (but num_features_ is not
   advanced) when back-projection fails.
4. updateFilterGaussian has no effect besides its NaN test (state by value).
5. align1D: x-gradient kernel pairs the corners diagonally (BR−TL, BL−TR); global weight table
   initialised without synchronisation; signed determinant test.
6. edgeletDetector_V2 runs the non-max suppression with cv::parallel_for_ over rows while writing
   `corners.at(k)` — two rows hitting the same grid cell race (non-deterministic winner when
   OpenCV runs in parallel; set cv::setNumThreads(1) for reproducible A/B runs).
7. getTriangulationParallax dereferences `obs_.begin()` without an emptiness check.
8. Point::optimize: NaN updates are not rolled back (only |dp|² > 1 is).
9. portable_binary_iarchive::init accepts archives of any library version.
10. DepthFilter::updateSeeds (threaded) never queues jobs for reference frames recorded as
    "without seeds"; the record is only pruned when the map's keyframe count differs.
11. updateSeedsLoop calls `Sleep(0)` after every job.
12. findEpipolarMatchDirect returns kFailAngle (7) when backProject3 fails.

## Per-function notes (non-obvious decisions)

- **Point::optimize** frames vector is `std::vector<std::pair<FramePtr,size_t>>` (24-byte
  elements, `reserve(obs_.size())`, `push_back(std::make_pair(...))`).  `vk::norm_max(dp)`
  materialises a VectorXd (aligned malloc) as upstream.
- **getTriangulationParallax**: `std::min(fabs(dot), min_cos)` and
  `acos(std::max(0.0, std::min(min_cos, 1.0)))` — operand order chosen to reproduce the NaN
  behaviour of the compiled selects.
- **DepthFilter ctor** `matcher_(new Matcher())` (the `free(0)` in the pseudocode is the
  `_Temporary_owner` of the shared_ptr ctor).  The 3-argument ctor receives DetectorOptions by
  value (caller copies 80 bytes) — TODO(verify).
- **addKeyframe** clears the queue with a named `JobQueue` swapped into `jobs_` (destroyed at
  scope end, after the push).
- **updateSeed** argument evaluation of findEpipolarMatchDirect follows MSVC right-to-left
  order: getInvMaxDepth, getInvMinDepth, getInvDepth, each creating an `Eigen::Ref<const
  SeedState>` (0x18009DBE0).  `xyz_f = T_cur_ref * (getDepth(state) * f.cast<double>())` — the
  depth is widened before the multiply.
- **findEpipolarMatchDirect**: A (d_min) and C (d_estimate) are
  `T.getRotation().cast<float>().rotate(f) + T.cast<float>().getPosition()*d`, B (d_max) and the
  case-2 C are `T.cast<float>().getRotation().rotate(f) + T.cast<float>().getPosition()*d` (call
  pattern of 0x18008A270/0x18008A460; numerically identical alternatives). The scalar is
  converted to float before the multiply (cvtpd2ps), i.e. Eigen's `operator*(const float&)`.
- **depthFromTriangulation** dot products are written out explicitly (sequential x,y,z sums as
  in the binary).
- **edgeletDetector_V2** lambdas use `[&]`; closure layouts (capture order) checked:
  #1 {dx, dy, score, angle, border, max_col, threshold, angle_factor}, #2 {score, border, grid,
  scale, threshold, stride, corners, level, img_pyr}.
- **fillFeatures** uses `std::stable_sort` (MSVC `_Stable_sort_unchecked` with 512-entry
  optimistic buffer, insertion sort ≤ 32), not `std::sort`.

## Open questions

- Names of all Pimax-new members/functions marked TODO(verify) (Frame +20/+384/+392/+424/+428/
  +456, KeypointIdentifier +20, DepthFilter +184/+224 and 0x18009FC10, DetectorOptions +40/+48,
  Matcher +332, the three anonymous helpers in depth_filter.cpp, align1D statics).
- Whether the FAST entry used is `fast_corner_detect_10_sse2` or `fast_corner_detect_10`
  (0x1801AFA30 is the sse2-style wrapper).
- Point members +80..+128 (only seen in the ctor of the previous chunk).
- DepthFilterOptions defaults and the exact DetectorOptions field names (+56, +72).
- Whether `Job`'s SEED_INIT depths come from a ctor default or explicit 1.0f arguments.
