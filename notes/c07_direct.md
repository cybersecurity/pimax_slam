# c07_direct — [0x1800AF5E0, 0x1800E46A0)

437 functions in the range. 54 of them are project code: 14 written functions, plus templates and header code
(serialization, Sophus, VINS utility), plus implicit or inline constructors. The other 383 are library template
instantiations.

## What the chunk actually contains (correction to the expected contents)

The brief expected `feature_alignment`, `patch_warp`, `feature_detection`, `depth_estimation` and similar here. Those
objects are **before** this range, in c06. This range holds:

| object | range | contents |
|---|---|---|
| `src/direct/matcher.cpp` (tail) | [.., 0x1800B0450) | `Matcher::findMatchDirect`, `Matcher::scanEpipolarUnitPlane`, `vector<cv::Mat>::size`, `Matcher::updateZMSSD`. The earlier part of the object (createPatchFromPatchWithBorder 0x1800AE5A0, depthFromTriangulation, both `findEpipolarMatchDirect` 0x1800AE820 / 0x1800AF4B0) is in c06. |
| `src/direct/patch_warp.cpp` | [0x1800B0450, 0x1800B1480) | `getBestSearchLevel`, `getWarpMatrixAffine`, `warpAffine`, plus one vector instantiation. The ICF-folded `pangolin::Handler` deleting dtor body landed at 0x1800B1450. |
| `src/frontend/frame_processor.cpp` | [0x1800B1480, 0x1800B57D0) | `FrameProcessor` (= upstream `FrameHandlerStereo`) with its template instantiations. |
| `src/frontend/frame_processor_base.cpp` (head) | [0x1800B57D0, 0x1800E46A0) → continues in c08 | Function templates (`??$…`) in mangled-name order, the boost `oserializer` ctors, then the non-template constructors (`??0…`): **`FrameProcessorBase::FrameProcessorBase` 0x1800DFDF0**, IMUFactor, IntegrationBase, ImageFrame, KeyFrame and others. Then std exception ctors and destructors (`??1…`). c08 starts with `~FrameProcessorBase` (0x1800E46A0). |

**Object-boundary rule (important for every chunk).** Inside an object file, MSVC laid the COMDATs out **sorted by
decorated name**:
- `??$…` function templates come first, then `??0?$…`, then `??0X` ctors, `??1…` dtors, `??4…`, `??_G…`, then
  `?member@…` in alphabetical order.
- Example from frame_processor.cpp: ctor, deleting dtor, makeKeyframe, processFirstFrame, processFrame,
  processFrameBundle, re…, resetAll.
- A new object starts where the sort order restarts at `??$`. That is how 0x1800B0450, 0x1800B1480 and 0x1800B57D0
  were found.
- It also explains why a function used by frame_processor_base.cpp appears here. Every function from 0x1800B57D0 on
  is reached only from FrameProcessorBase methods (0x180110490, 0x1800F2070, 0x1800ECFD0, …) or from the dynamic
  initializers of `kStageName` / `kTrackingQualityName` / `kUpdateResultName` (0x180002410, 0x1800025A0, 0x180002710 →
  0x1800D0090).

## Drafts (draft/c07_direct/)

| file | content |
|---|---|
| `direct/matcher.h`, `direct/matcher.cpp` | Pimax Matcher layout; 0x1800AF5E0, 0x1800AFD70, 0x1800B0270 |
| `direct/patch_warp.h`, `direct/patch_warp.cpp` | 0x1800B05B0, 0x1800B0600, 0x1800B0C10 |
| `frontend/frame_processor.h`, `frontend/frame_processor.cpp` | all FrameProcessor functions |
| `frontend/frame_processor_base.h` | **Partial**: complete FrameProcessorBase member list with offsets and initialisers (from the ctor), BaseOptions, ImageFrame, S96, S552, ReprojectResult |
| `frontend/frame_processor_base_ctor.cpp` | 0x1800DFDF0 FrameProcessorBase ctor; S136; notes on KeyFrame() and T232 copy |
| `frontend/frame_processor_base_part_templates.cpp` | Sophus FormatStream/FormatString/defaultEnsure, VINS Utility (deltaQ, skewSymmetric, Qleft, Qright), eulerToRotation, PlatMap keyframe-graph save/load (the 0x1800B57D0..0x1800C0C70 project templates were done by a helper fork) |
| `loop_closing/serialization_part.cpp` | cv::Mat / BowVector / QuatTransformation save+load, KeyFrame::serialize |
| `vio/imu_integration_ctors.cpp` | IntegrationBase ctor 0x1800E20B0 and IMUFactor ctor 0x1800E1950, with the member lists they prove |

Line counts: notes ~820, drafts 2269 (matcher 361, patch_warp 207, frame_processor 590, frame_processor_base.h 342, base ctor 154, templates 342, serialization 186, vio 87).

## Function table

Columns: address \| proposed name \| signature \| kind \| upstream status \| summary.

| address | proposed qualified name | signature | kind | upstream | summary |
|---|---|---|---|---|---|
| | **object: matcher.cpp** | | | | |
| 0x1800AF5E0 | Matcher::findMatchDirect | MatchResult(const Frame& ref, const Frame& cur, const FeatureWrapper& ref_ftr, const FloatType& ref_depth, Keypoint& px_cur) | project | modified: float keypoints, always warpAffine, corner path lost kFailTooFar, backProject3 result checked | affine-warped patch + align1D/align2D refinement (matcher.cpp:221 VLOG) |
| 0x1800AFD70 | Matcher::scanEpipolarUnitPlane | void(const Frame&, const Vector3d& A, const Vector3d& B, const Vector3d& C, const PatchScore&, int level, Keypoint* best, int* zmssd_best) | project | modified: window min(15,len/0.7) each side of C, no reversal/clamp, skips invisible/out-of-image | epipolar ZMSSD scan on the unit plane |
| 0x1800B0240 | std::vector<cv::Mat>::size | size_t() const | lib:std::vector<cv::Mat>::size (not inlined) | - | (end-begin)/96 |
| 0x1800B0270 | Matcher::updateZMSSD | bool(const Frame&, const Vector2i& pxi, int level, const PatchScore&, int* best) | project | identical | ZMSSD<4>::computeScore inlined (threshold 128000) |
| | **object: patch_warp.cpp** | | | | |
| 0x1800B0450 | std::vector<Eigen::Vector2f>::_Resize_reallocate | - | lib:std::vector<Vector2f>::_Resize_reallocate (patch_warp.cpp object start) | - | from warpAffine's thread_local buffer resize |
| 0x1800B05B0 | warp::getBestSearchLevel | int(const Matrix2d& A_cur_ref, int max_level) | project | identical | det>3 halving loop |
| 0x1800B0600 | warp::getWarpMatrixAffine | void(const CameraPtr&, const CameraPtr&, const Ref<Keypoint>&, const Ref<BearingVector>&, double depth, const Transformation&, int level, Matrix2d*) | project | modified: half patch 8 (was 5), no CHECK_NOTNULL, float xyz_ref | affine warp from du/dv backprojection |
| 0x1800B0C10 | warp::warpAffine | bool(const Matrix2d&, const cv::Mat&, const Ref<Keypoint>&, int level_ref, int search_level, int halfpatch, uint8_t*) | project | modified: no NaN log, ref px bounds pre-check, thread_local position buffer, xi<=0 rejected, (int) truncation in pass 2 | two-pass bilinear patch warp |
| 0x1800B1450 | pangolin::Handler scalar deleting dtor (ICF-folded empty-class body) | void*(uint) | lib:pangolin/ICF | - | sets Handler vftable, frees |
| | **object: frame_processor.cpp** | | | | |
| 0x1800B1480 | cv::_InputArray::_InputArray(const std::vector<cv::Point2f>&) | - | lib:OpenCV inline (frame_processor.cpp object start) | - | flags 0x8103000D |
| 0x1800B14A0 | std::vector<cv::Point2f>::_Emplace_reallocate<float&,float&> | - | lib:std | - | from removeOutliersByFundamentalMat emplace_back |
| 0x1800B16B0 | std::vector<S28>::_Emplace_reallocate<int&,int&,int&> | - | lib:std (28-byte element {int,int,float,float=-1,int64=0,int=-1}; caller 0x18017E300) | - | cv::KeyPoint-like 28-byte POD built from (x,y,size) |
| 0x1800B1820 | std::list<pair<const int,std::vector<size_t>>>::_Free_non_head | - | lib:std (unordered_map<int,vector<size_t>>) | - |  |
| 0x1800B18C0 | std::_Guess_median_unchecked<int*,less<>> | - | lib:std::sort | - |  |
| 0x1800B1A10 | std::_Sort_unchecked<int*,less<>> | - | lib:std::sort | - | introsort on vector<int> (makeKeyframe) |
| 0x1800B1E20 | std::unordered_map<int,std::vector<size_t>>::_Try_emplace (operator[]) | - | lib:std | - |  |
| 0x1800B20F0 | std::_Uninitialized_move<S28*> | - | lib:std | - |  |
| 0x1800B2170 | std::unordered_map<int,FramePtr>::emplace(const value_type&) | - | lib:std | - |  |
| 0x1800B2460 | FrameProcessor::FrameProcessor | (BaseOptions, DepthFilterOptions, DetectorOptions, InitializationOptions, StereoTriangulationOptions, ReprojectorOptions, FeatureTrackerOptions, CameraBundle::Ptr, int, const bool&) | project | modified: 2 extra args forwarded to base; unique_ptr->shared_ptr | creates StereoTriangulation(stereo_options, makeDetector(..., cam0)) |
| 0x1800B26E0 | std::_Hash<unordered_map<int,vector<size_t>>>::~_Hash | - | lib:std | - |  |
| 0x1800B2760 | std::_List_node_emplace_op2<...pair<const int,vector<size_t>>>::~ | - | lib:std | - |  |
| 0x1800B27F0 | std::_Tree<std::set<int>>::~_Tree | - | lib:std | - |  |
| 0x1800B2850 | std::list<pair<const int,vector<size_t>>>::~list | - | lib:std | - |  |
| 0x1800B2880 | std::unique_ptr<list node {..., shared_ptr @32, cv::Mat @48}>::~unique_ptr | - | lib:std (callers initialization 0x18012BAB0) | - |  |
| 0x1800B2900 | std::unordered_map<int,vector<size_t>>::~unordered_map (thunk) | - | lib:std | - |  |
| 0x1800B2910 | std::vector<S28>::~vector (thunk to _Tidy) | - | lib:std | - |  |
| 0x1800B2920 | std::shared_ptr<FrameBundle>::operator=(const shared_ptr&) | - | lib:std | - | used for last_kf_frames_ = new_frames_ (many callers) |
| 0x1800B29B0 | FrameProcessor::`scalar deleting destructor' | void*(uint) | project (implicit dtor) | identical | ~vector last_df_keyframes_, release stereo_triangulation_, ~FrameProcessorBase 0x1800E46A0, free |
| 0x1800B2A60 | std::vector<S28>::_Change_array | - | lib:std | - |  |
| 0x1800B2B10 | std::_Ref_count_resource<StereoTriangulation*,default_delete>::_Destroy | - | lib:std | - | ~cv::Mat @48, release detector @32, free |
| 0x1800B2B90 | std::_Ref_count_resource<StereoTriangulation*,default_delete>::_Get_deleter | - | lib:std | - |  |
| 0x1800B2BC0 | std::vector<S28>::_Tidy | - | lib:std | - |  |
| 0x1800B2C40 | std::vector<bool>::_Trim | - | lib:std (caller 0x180108CA0) | - |  |
| 0x1800B2CF0 | std::allocator<S28>::allocate | - | lib:std | - |  |
| 0x1800B2D60 | std::allocator<S28>::deallocate | - | lib:std | - |  |
| 0x1800B2DA0 | std::deque<int>::erase(const_iterator,const_iterator) | - | lib:std (Map+128 keyframe-id deque) | - |  |
| 0x1800B2F80 | FrameProcessor::makeKeyframe | UpdateResult(int input_value) | project | modified (rewritten) | redundant-KF detection, stereo/temporal triangulation, depth filter, map size limit / prior-map pruning |
| 0x1800B4180 | FrameProcessor::processFirstFrame | UpdateResult() | project | modified | all frames -> keyframes, depth filter, "Init: Selected first frame." |
| 0x1800B4290 | FrameProcessor::processFrame | UpdateResult() | project | modified (heavily) | reprojection stats, F-matrix outlier rejection, pose/structure optim, KF selection |
| 0x1800B4970 | FrameProcessor::processFrameBundle | UpdateResult() | project | identical | dispatch on stage_ |
| 0x1800B49A0 | FrameProcessor::removeOutliersByFundamentalMat (name ours) | void() | project | new | track-id matches cur/last, findFundamentalMat RANSAC 1.5/0.99, drop outliers |
| 0x1800B5760 | FrameProcessor::resetAll | void() | project | modified | "resetAll", reset counter +3116, clear last_df_keyframes_, resetVisionFrontendCommon |
| | **object: frame_processor_base.cpp** | | | | |
| 0x1800B57D0 | std::_Func_impl_no_alloc<lambda>::… (ICF-folded, first COMDAT of frame_processor_base.cpp) | - | lib:std::function | - |  |
| 0x1800B5800 | cv::_OutputArray::_OutputArray(std::vector<int>&) | - | lib:OpenCV inline | - | flags 0x82030004 |
| 0x1800B5820 | std::pair<const int,X>::pair(int&, X&&) | - | lib:std | - | X = {shared_ptr, int, int, 8 bytes}; caller 0x180110490 |
| 0x1800B5860 | std::_Tree_temp_node for map<double,ImageFrame> (emplace) | - | lib:std::map | - | node 0x1F0; map at FrameProcessorBase+3480 |
| 0x1800B5AD0 | Concurrency::task<unsigned char> ctor (_Task_async_state) | - | lib:PPL/std::async | - |  |
| 0x1800B5D60 | Eigen quaternionbase_assign_impl<Block<Matrix4d,3,3>>::run | - | lib:Eigen | - | Quaterniond from 3x3 block of Matrix4d |
| 0x1800B5F80 | std::_Task_async_state<void>::_Task_async_state(lambda&&) | - | lib:std::async | - | std::async(launch::async,[this,i]{...}) in 0x180119260 |
| 0x1800B6150 | Eigen::PermutationMatrix<15>::operator=(Transpositions) | - | lib:Eigen | - |  |
| 0x1800B6230 | binary_iarchive load nvp<collection_size_type> | - | lib:boost | - | lib version <=5: 4 bytes else 8 |
| 0x1800B6360 | binary_iarchive load nvp<item_version_type> | - | lib:boost | - |  |
| 0x1800B6490 | binary_iarchive load 4 bytes | - | lib:boost | - |  |
| 0x1800B6520 | binary_iarchive load 8 bytes | - | lib:boost | - |  |
| 0x1800B65B0 | binary_oarchive save 8 bytes | - | lib:boost | - |  |
| 0x1800B6680 | Eigen operator<< (M*M^T - I) | - | lib:Eigen print | - | minkindr isValidRotationMatrix VLOG(200) |
| 0x1800B6A60 | Eigen operator<< (Matrix3d) | - | lib:Eigen | - | minkindr CHECK(isValidRotationMatrix) << matrix |
| 0x1800B6DA0 | Eigen operator<< (M*M^T) | - | lib:Eigen | - |  |
| 0x1800B71D0 | Eigen operator<< (Transpose<const Vector3d>) | - | lib:Eigen | - |  |
| 0x1800B7540 | archive save nvp of a 4-byte value | - | lib:boost | - |  |
| 0x1800B75F0 | return *p (ICF) | - | lib | - |  |
| 0x1800B7600 | archive load 1-byte array element by element | - | lib:boost | - | portable path |
| 0x1800B7710 | archive load 4 bytes | - | lib:boost | - |  |
| 0x1800B77D0 | archive save 4 bytes | - | lib:boost | - |  |
| 0x1800B78A0 | archive load 8 bytes | - | lib:boost | - |  |
| 0x1800B7960 | archive save 1-byte array element by element | - | lib:boost | - |  |
| 0x1800B7A80 | archive save 8 bytes | - | lib:boost | - |  |
| 0x1800B7B50 | std::hash<int>::operator() (FNV-1a) | - | lib:std | - |  |
| 0x1800B7BA0 | Sophus::details::FormatStream<Transpose<const Vector3d>,double&,double&> | - | project (header template) | modified | "{}"/"{.N}" placeholders |
| 0x1800B8020 | Sophus::details::FormatStream<double&> | - | project (header template) | modified |  |
| 0x1800B8490 | Sophus::details::FormatStream<double&,double&> | - | project (header template) | modified |  |
| 0x1800B8900 | Sophus::details::FormatString<Transpose<const Vector3d>,double&,double&> | - | project (header template) | identical |  |
| 0x1800B8A50 | Utility::Qleft<Quaterniond> | - | project (VINS header template) | identical (VINS) | caller 0x1800ECFD0 |
| 0x1800B8C00 | Utility::Qright<Quaterniond> | - | project (VINS header template) | identical (VINS) |  |
| 0x1800B8DB0 | load<binary_iarchive>(vector<vector<shared_ptr<KeyFrame>>>&) | - | project (serialization) | new | PlatMap+104 |
| 0x1800B9270 | save<binary_oarchive>(const vector<vector<shared_ptr<KeyFrame>>>&) | - | project (serialization) | new |  |
| 0x1800B9520 | load<portable_binary_iarchive>(vector<vector<shared_ptr<KeyFrame>>>&) | - | project (serialization) | new |  |
| 0x1800B99E0 | save<portable_binary_oarchive>(…) | - | project (serialization) | new |  |
| 0x1800B9C90 | std::list<{int, 16-aligned 16 bytes}>::assign | - | lib:std | - |  |
| 0x1800B9E50 | vector<vector<shared_ptr<T>>>::_Assign_range | - | lib:std | - |  |
| 0x1800BA190 | vector<Vector3d,aligned_allocator>::_Assign_range | - | lib:std | - |  |
| 0x1800BA360 | vector<Matrix3d,aligned_allocator>::_Assign_range | - | lib:std | - |  |
| 0x1800BA570 | map<int,vector<pair<int,Matrix<double,7,1>>>>::_Copy | - | lib:std | - | ImageFrame::points copy |
| 0x1800BA620 | map<int,vector<pair<int,Matrix<double,7,1>>>>::_Copy_nodes | - | lib:std | - |  |
| 0x1800BA7F0 | std::set<int>::_Copy_nodes | - | lib:std | - |  |
| 0x1800BA8B0 | _Copy_unchecked<vector<shared_ptr<T>>*> | - | lib:std | - |  |
| 0x1800BA9F0 | copy(list<shared_ptr<T>> -> shared_ptr<T>*) | - | lib:std | - |  |
| 0x1800BAA90 | _Destroy_range<std::future<void>> | - | lib:std | - |  |
| 0x1800BAB00 | _Destroy_range<vector<8 bytes>> | - | lib:std | - |  |
| 0x1800BAB90 | _Destroy_range<vector<shared_ptr<T>>> | - | lib:std | - |  |
| 0x1800BAC30 | map<double,shared_ptr<T>>::emplace | - | lib:std | - | FrameProcessorBase+3464 |
| 0x1800BAD70 | deque<32-byte POD,aligned_allocator>::push_back | - | lib:std | - | FrameProcessorBase+2784 |
| 0x1800BAE60 | vector<Vector3d,aligned_allocator>::_Emplace_reallocate (float->double cast) | - | lib:std | - |  |
| 0x1800BB030 | vector<Vector3d>::_Emplace_reallocate (float->double cast) | - | lib:std | - |  |
| 0x1800BB200 | vector<16-byte POD>::_Emplace_reallocate | - | lib:std | - |  |
| 0x1800BB390 | vector<12-byte POD>::_Emplace_reallocate | - | lib:std | - |  |
| 0x1800BB590 | vector<128-byte POD>::_Emplace_reallocate | - | lib:std | - |  |
| 0x1800BB7E0 | vector<shared_ptr<T>>::_Emplace_reallocate (copy) | - | lib:std | - |  |
| 0x1800BBA20 | vector<vector<shared_ptr<KeyFrame>>>::_Emplace_reallocate | - | lib:std | - |  |
| 0x1800BBBB0 | vector<{int,int}>::_Emplace_reallocate (emplace) | - | lib:std | - |  |
| 0x1800BBD10 | vector<{8 bytes,int}>::_Emplace_reallocate | - | lib:std | - |  |
| 0x1800BBE70 | vector<Vector3d>::_Emplace_reallocate | - | lib:std | - |  |
| 0x1800BC020 | vector<8-byte POD>::_Emplace_reallocate | - | lib:std | - |  |
| 0x1800BC180 | vector<vector<KeyFrame*>>::_Emplace_reallocate | - | lib:std | - |  |
| 0x1800BC2E0 | vector<std::future<void>>::_Emplace_reallocate | - | lib:std | - |  |
| 0x1800BC580 | vector<shared_ptr<T>>::_Emplace_reallocate (move) | - | lib:std | - |  |
| 0x1800BC6F0 | vector<shared_ptr<KeyFrame>>::_Emplace_reallocate (move) | - | lib:std | - |  |
| 0x1800BC8B0 | vector<unique_ptr<Reprojector>>::_Emplace_reallocate | - | lib:std | - | reprojectors_ push |
| 0x1800BCA00 | _Tree<trivial>::~_Tree / _Erase_tree | - | lib:std | - |  |
| 0x1800BCA80 | _Tree<trivial>::_Erase_tree | - | lib:std | - |  |
| 0x1800BCAE0 | _Tree::_Erase_tree (map<int,vector<pair<int,Matrix<double,7,1>>>>) | - | lib:std | - |  |
| 0x1800BCBA0 | _Tree::_Erase_tree (value dtor 0x1800E44E0) | - | lib:std | - |  |
| 0x1800BCC20 | _Tree::_Erase_tree (map<double,shared_ptr<T>>) | - | lib:std | - |  |
| 0x1800BCCD0 | _Tree::_Erase_tree (map<double,ImageFrame>) | - | lib:std | - |  |
| 0x1800BCD50 | _Tree::_Erase_tree (set<shared_ptr<T>>) | - | lib:std | - |  |
| 0x1800BCE00 | map<unsigned,X>::_Find_hint | - | lib:std | - |  |
| 0x1800BD000 | unordered_map<int,X>::_Find_last | - | lib:std | - |  |
| 0x1800BD070 | map<double,X>::_Find_lower_bound | - | lib:std | - |  |
| 0x1800BD0C0 | list<trivial>::_Free_non_head | - | lib:std | - |  |
| 0x1800BD100 | list<shared_ptr<T>>::_Free_non_head | - | lib:std | - |  |
| 0x1800BD190 | map<double,shared_ptr<T>> node free | - | lib:std | - |  |
| 0x1800BD200 | _Guess_median_unchecked<double*> | - | lib:std::sort | - |  |
| 0x1800BD3A0 | vector<4-byte POD>::_Insert_range | - | lib:std | - |  |
| 0x1800BD5D0 | std::sort part (shared_ptr<T>, lambda a->(+16) > b->(+16)) | - | lib:std::sort | - | sort in 0x1800E61D0 |
| 0x1800BD7D0 | std::sort part (pair<double,int>) | - | lib:std::sort | - | sort in 0x1800F5170 |
| 0x1800BD950 | std::sort part (shared_ptr<T>, lambda) | - | lib:std::sort | - |  |
| 0x1800BDBF0 | std::sort part (pair<double,int>) | - | lib:std::sort | - |  |
| 0x1800BDCC0 | std::sort part (shared_ptr<T>, lambda) | - | lib:std::sort | - |  |
| 0x1800BDD40 | _Move_backward_unchecked<shared_ptr<T>*> | - | lib:std | - |  |
| 0x1800BDDF0 | _Move_unchecked<T232*> (T232 move-assign) | - | lib:std | - |  |
| 0x1800BE050 | std::sort part (double*) | - | lib:std::sort | - | sort in 0x18013ED70 / 0x1800F19D0 |
| 0x1800BE350 | std::sort part (pair<double,int>) | - | lib:std::sort | - |  |
| 0x1800BE910 | std::sort part (shared_ptr<T>, lambda) | - | lib:std::sort | - |  |
| 0x1800BEBE0 | std::sort part (pair<double,int>) | - | lib:std::sort | - |  |
| 0x1800BED20 | std::sort part (shared_ptr<T>, lambda) | - | lib:std::sort | - |  |
| 0x1800BEF30 | vector<4-byte>::_Resize_reallocate (zero fill) | - | lib:std | - |  |
| 0x1800BF000 | vector<8-byte>::_Resize_reallocate (zero fill) | - | lib:std | - |  |
| 0x1800BF0D0 | vector<12-byte>::_Resize_reallocate (zero fill) | - | lib:std | - |  |
| 0x1800BF200 | vector<{int,int}>::_Resize_reallocate (zero fill) | - | lib:std | - |  |
| 0x1800BF2E0 | vector<std::set<int>>::_Resize_reallocate | - | lib:std | - |  |
| 0x1800BF5B0 | vector<vector<…>>::_Resize_reallocate | - | lib:std | - |  |
| 0x1800BF6D0 | vector<vector<…>>::_Resize_reallocate | - | lib:std | - |  |
| 0x1800BF8B0 | vector<vector<shared_ptr<T>>>::_Resize_reallocate | - | lib:std | - |  |
| 0x1800BFB10 | std::sort part (double*) | - | lib:std::sort | - |  |
| 0x1800BFE30 | std::sort part (pair<double,int>) | - | lib:std::sort | - |  |
| 0x1800C0070 | std::sort part (shared_ptr<T>, lambda) | - | lib:std::sort | - |  |
| 0x1800C0280 | unordered_map<int,int>::try_emplace | - | lib:std | - |  |
| 0x1800C04E0 | unordered_map<int, Eigen fixed (<=12 bytes)>::try_emplace | - | lib:std | - |  |
| 0x1800C0740 | unordered_map<int,shared_ptr<T>>::try_emplace | - | lib:std | - |  |
| 0x1800C09A0 | _Uninitialized_move<vector<8 bytes>*> | - | lib:std | - |  |
| 0x1800C0A20 | PartialPivLU<Matrix<double,15,15>>::inverse (_solve_impl on Identity) | - | lib:Eigen | - | caller 0x18011D5C0 |
| 0x1800C0C70 | Eigen::MatrixBase<Block<…float…>>::applyHouseholderOnTheLeft | - | lib:Eigen (float, JacobiSVD/QR internals; callers 0x1800C8C10, 0x1800CD4B0) | - |  |
| 0x1800C1880 | Eigen::MatrixBase<Block<…float…>>::applyHouseholderOnTheRight | - | lib:Eigen (float; callers 0x1800C9120, 0x1800CDF80) | - |  |
| 0x1800C2580 | Eigen::MatrixBase<Block<…double…>>::applyHouseholderOnTheLeft | - | lib:Eigen (double; callers 0x1800C9660, 0x1800CEEA0) | - |  |
| 0x1800C3540 | Eigen applyHouseholderOnTheLeft (float, other block type) | - | lib:Eigen (caller 0x180101D70) | - |  |
| 0x1800C41A0 | Eigen applyHouseholderOnTheRight (float, other block type) | - | lib:Eigen (caller 0x180102720) | - |  |
| 0x1800C4F90 | Eigen applyHouseholderOnTheLeft (double, other block type) | - | lib:Eigen (caller 0x1801035F0) | - |  |
| 0x1800C5FD0 | Eigen applyHouseholderOnTheLeft/Right (float, Map/Transpose variant) | - | lib:Eigen (caller 0x1800CD4B0) | - |  |
| 0x1800C6B00 | Eigen applyHouseholderOnTheRight (float, Transpose variant) | - | lib:Eigen (caller 0x1800CDF80) | - |  |
| 0x1800C76A0 | Eigen applyHouseholderOnTheLeft (double, Transpose variant) | - | lib:Eigen (caller 0x1800CEEA0) | - |  |
| 0x1800C85F0 | Eigen::MatrixBase<…double 2xN>::applyOnTheLeft(p,q,JacobiRotation<double>) | - | lib:Eigen (JacobiSVD 2x2 step) | - |  |
| 0x1800C86F0 | Eigen::MatrixBase<…float>::applyOnTheRight(p,q,JacobiRotation<float>) | - | lib:Eigen | - |  |
| 0x1800C8800 | Eigen::MatrixBase<MatrixXf>::applyOnTheRight(p,q,JacobiRotation<float>) | - | lib:Eigen | - |  |
| 0x1800C8990 | Eigen::MatrixBase<Matrix2d>::applyOnTheRight(p,q,JacobiRotation<double>) | - | lib:Eigen | - |  |
| 0x1800C8AB0 | Eigen::MatrixBase<Matrix3d>::applyOnTheRight(p,q,JacobiRotation<double>) | - | lib:Eigen | - |  |
| 0x1800C8C10 | Eigen::internal::qr_preconditioner_impl<Matrix<float,2,3>,ColPivHouseholderQR,…>::run | - | lib:Eigen (JacobiSVD<Matrix<float,2,3>>, setFromTwoVectors<float>) | - |  |
| 0x1800C9120 | Eigen::internal::qr_preconditioner_impl<MatrixXf,ColPivHouseholderQR,…>::run | - | lib:Eigen | - |  |
| 0x1800C9660 | Eigen::internal::qr_preconditioner_impl<Matrix<double,2,3>,ColPivHouseholderQR,…>::run | - | lib:Eigen | - |  |
| 0x1800C9B70 | Eigen::ColPivHouseholderQR<Matrix<float,3,2>>::computeInPlace | - | lib:Eigen | - |  |
| 0x1800CA330 | Eigen::ColPivHouseholderQR<MatrixXf>::computeInPlace | - | lib:Eigen | - |  |
| 0x1800CAC50 | Eigen::ColPivHouseholderQR<Matrix<double,3,2>>::computeInPlace | - | lib:Eigen | - |  |
| 0x1800CB3F0 | Concurrency::details::atomic_exchange<long> | - | lib:concrt | - |  |
| 0x1800CB400 | Eigen cast Vector4f -> Vector4d (Quaternionf::cast<double> coeffs) | - | lib:Eigen | - |  |
| 0x1800CB470 | std::_Deque_unchecked_const_iterator compare (debug check 0x1B6) | - | lib:std | - |  |
| 0x1800CB4B0 | std::set<…>::insert range from vector<vector<T>> (std::_Tree insert) | - | lib:std (caller 0x180119260) | - |  |
| 0x1800CB6D0 | Eigen::LLT<Matrix<double,15,15>>::compute | - | lib:Eigen (l1 norm @+1800, m_isInitialized @+1808, m_info @+1812; caller 0x1800ECFD0) | - |  |
| 0x1800CB970 | Eigen::PlainObjectBase<MatrixXf>::resize+assign (lazyAssign) | - | lib:Eigen | - |  |
| 0x1800CBAD0 | std::unordered_map<int,…>::contains/find (FNV-1a) | - | lib:std | - |  |
| 0x1800CBB70 | std::_Task_async_state / promise set_value helper (PPL) | - | lib:std::async | - |  |
| 0x1800CBC40 | Sophus::defaultEnsure<Transpose<const Vector3d>,double&,double&> | void(const char* function, const char* file, int line, const char* description, Args&&...) | project (header template) | modified: "mlog ensure failed" prefix | printf + FormatString to std::cout + abort; caller SO3::exp 0x18010A400 |
| 0x1800CBD00 | Utility::deltaQ<Product<Matrix3d,(Vector3d-Vector3d)>> | Quaterniond(const MatrixBase<…>&) | project (VINS header template) | identical (VINS) | half theta -> (1, θ/2) |
| 0x1800CBE50 | std::shared_ptr release helper (dtor of {T, shared_ptr} pair) | - | lib:std | - |  |
| 0x1800CBEA0 | std::unordered_map<int,int>::_Try_emplace | - | lib:std | - |  |
| 0x1800CC140 | std::unordered_map<int,…>::_Try_emplace (24-byte value) | - | lib:std | - |  |
| 0x1800CC410 | std::unordered_map<int,…>::_Try_emplace | - | lib:std | - |  |
| 0x1800CC680 | std::unordered_map<int64/ptr,…>::_Try_emplace (8-byte key) | - | lib:std (callers optimizeStructure 0x1801188C0/0x180118D80) | - |  |
| 0x1800CC9B0 | std::unordered_map<int,…>::_Try_emplace | - | lib:std | - |  |
| 0x1800CCC20 | std::unordered_map<int,…>::_Try_emplace (40-byte node value) | - | lib:std | - |  |
| 0x1800CCF00 | std::unordered_map<int,…>::_Try_emplace | - | lib:std | - |  |
| 0x1800CD170 | std::vector<S80>::emplace_back(const S80&) | - | lib:std (80-byte POD; caller addFrameBundle 0x1800FB650) | - |  |
| 0x1800CD1D0 | std::vector<shared_ptr<T>>::push_back(const shared_ptr&) | - | lib:std | - |  |
| 0x1800CD210 | std::deque<…>::_Emplace_back / push_back | - | lib:std | - |  |
| 0x1800CD2D0 | std::deque<…,aligned_allocator>::push_back | - | lib:std | - |  |
| 0x1800CD3D0 | std::_Tree<…>::erase(iterator) | - | lib:std | - |  |
| 0x1800CD460 | Eigen::internal::… set two unit coefficients (float) | - | lib:Eigen (JacobiSVD helper) | - |  |
| 0x1800CD4B0 | Eigen::ColPivHouseholderQR<Matrix<float,2,3>>::householderQ / HouseholderSequence::evalTo | - | lib:Eigen | - |  |
| 0x1800CDE20 | Eigen::PlainObjectBase<MatrixXf>::resize (setIdentity helper) | - | lib:Eigen | - |  |
| 0x1800CDF80 | Eigen::HouseholderSequence<MatrixXf,…>::evalTo | - | lib:Eigen | - |  |
| 0x1800CEE40 | Eigen::internal::… set two unit coefficients (double) | - | lib:Eigen | - |  |
| 0x1800CEEA0 | Eigen::HouseholderSequence<Matrix<double,2,3>…>::evalTo | - | lib:Eigen | - |  |
| 0x1800CF830 | Eigen product evaluator Matrix<double,15,3> = … (resize 45) | - | lib:Eigen (caller preintegration 0x1800ECFD0) | - |  |
| 0x1800CF940 | Eigen product evaluator Matrix<double,15,7> = … (resize 105) | - | lib:Eigen (caller 0x1800ECFD0) | - |  |
| 0x1800CFA50 | Eigen product evaluator Matrix<double,15,15> = … (resize 225) | - | lib:Eigen (caller 0x180104140) | - |  |
| 0x1800CFB50 | Eigen::internal::manage_caching_sizes / gemm blocking (static thread-safe init) | - | lib:Eigen | - |  |
| 0x1800CFF50 | std::_Tree<…>::_Find_hint / lower_bound helper | - | lib:std | - |  |
| 0x1800CFFF0 | std::unordered_map<int,…>::find | - | lib:std | - |  |
| 0x1800D0090 | std::unordered_map<Enum,std::string,EnumClassHash>::insert(initializer_list) | - | lib:std (only callers: dynamic initializers 0x180002410/0x1800025A0/0x180002710 = kStageName/kTrackingQualityName/kUpdateResultName of frame_processor_base.cpp) | - |  |
| 0x1800D0350 | std::map<…>::emplace (vector value) | - | lib:std | - |  |
| 0x1800D0460 | std::unordered_map<int,…>::insert(range) (T232 copy) | - | lib:std | - |  |
| 0x1800D06E0 | std::unordered_map<int64,…>::insert(range) | - | lib:std | - |  |
| 0x1800D0A40 | std::deque<…>::_Insert_range | - | lib:std | - |  |
| 0x1800D0F10 | boost load collection std::vector<KeyFrame*> (binary_iarchive) | - | lib:boost::serialization | - |  |
| 0x1800D10F0 | boost load collection std::vector<KeyFrame*> (portable_binary_iarchive) | - | lib:boost::serialization | - |  |
| 0x1800D1290 | boost load collection std::vector<cv::Point2f> (portable) | - | lib:boost::serialization | - |  |
| 0x1800D1400 | boost load collection std::map<unsigned,double> (binary) | - | lib:boost::serialization | - |  |
| 0x1800D1630 | boost load collection std::map<unsigned,double> (portable) | - | lib:boost::serialization | - |  |
| 0x1800D1870 | Eigen::MatrixBase<…float>::makeHouseholder | - | lib:Eigen | - |  |
| 0x1800D1CF0 | Eigen::MatrixBase<…float>::makeHouseholder (other block) | - | lib:Eigen | - |  |
| 0x1800D21C0 | Eigen::MatrixBase<…double>::makeHouseholder | - | lib:Eigen | - |  |
| 0x1800D2630 | Eigen::JacobiRotation<double>::makeJacobi(…) | - | lib:Eigen | - |  |
| 0x1800D2780 | Eigen::internal::householder_qr / colpiv inner update (float) | - | lib:Eigen | - |  |
| 0x1800D3250 | Eigen::internal::colpiv QR inner update (float, MatrixXf) | - | lib:Eigen | - |  |
| 0x1800D3D50 | Eigen::internal::colpiv QR inner update (double) | - | lib:Eigen | - |  |
| 0x1800D4830 | std::pair<const int,std::shared_ptr<T>>::pair(int&, shared_ptr&) | - | lib:std | - |  |
| 0x1800D4860 | Eigen::DenseBase<VectorXd>::maxCoeff(Index*) | - | lib:Eigen | - |  |
| 0x1800D49E0 | Eigen::DenseBase<…double>::maxCoeff(Index*) | - | lib:Eigen | - |  |
| 0x1800D4B20 | Eigen::internal::general_matrix_vector_product (float, rank-1/gemv kernel) | - | lib:Eigen | - |  |
| 0x1800D4E60 | Eigen gemv kernel (float) | - | lib:Eigen | - |  |
| 0x1800D51A0 | Eigen gemv kernel (float) | - | lib:Eigen | - |  |
| 0x1800D54E0 | Eigen product Matrix3d*Vector (double) evaluator | - | lib:Eigen (caller 0x180128610) | - |  |
| 0x1800D5780 | Eigen::internal::… 2x2 determinant / permutation sign (double) | - | lib:Eigen | - |  |
| 0x1800D5840 | Eigen::internal::… (float) small permutation helper | - | lib:Eigen | - |  |
| 0x1800D5990 | Eigen::internal::… (double) small permutation helper | - | lib:Eigen | - |  |
| 0x1800D5AD0 | Eigen::internal::print_matrix<Transpose<const Vector3d>> | - | lib:Eigen | - |  |
| 0x1800D5EC0 | Eigen::internal::print_matrix<Matrix3d> | - | lib:Eigen | - |  |
| 0x1800D6330 | Eigen::internal::real_2x2_jacobi_svd<Matrix<float,2,3>> | - | lib:Eigen | - |  |
| 0x1800D6570 | Eigen::internal::real_2x2_jacobi_svd<MatrixXf> | - | lib:Eigen | - |  |
| 0x1800D67D0 | Eigen::internal::real_2x2_jacobi_svd<Matrix<double,2,3>> | - | lib:Eigen | - |  |
| 0x1800D6AE0 | std::reverse / deque reverse helper | - | lib:std | - |  |
| 0x1800D6BC0 | Eigen::internal::… householder block apply (float) | - | lib:Eigen | - |  |
| 0x1800D6CA0 | Eigen::internal::… householder block apply (float, MatrixXf) | - | lib:Eigen | - |  |
| 0x1800D6DD0 | Eigen::internal::… householder block apply (double) | - | lib:Eigen | - |  |
| 0x1800D6EB0 | Eigen redux (dot product) kernel float | - | lib:Eigen | - |  |
| 0x1800D7100 | Eigen redux (dot product) kernel float | - | lib:Eigen | - |  |
| 0x1800D7360 | Eigen redux (dot product) kernel float | - | lib:Eigen | - |  |
| 0x1800D75C0 | Eigen redux (dot product) kernel float | - | lib:Eigen | - |  |
| 0x1800D7810 | Eigen redux (dot product) kernel float | - | lib:Eigen | - |  |
| 0x1800D7A70 | Eigen redux (dot product) kernel double | - | lib:Eigen | - |  |
| 0x1800D7CC0 | Eigen redux (dot product) kernel double | - | lib:Eigen | - |  |
| 0x1800D7F20 | Eigen redux (squaredNorm) kernel float | - | lib:Eigen | - |  |
| 0x1800D80F0 | Eigen redux (squaredNorm) kernel float | - | lib:Eigen | - |  |
| 0x1800D82A0 | Eigen redux cwiseAbs().maxCoeff() float (NaN-aware) | - | lib:Eigen | - |  |
| 0x1800D84E0 | Eigen redux maxCoeff float | - | lib:Eigen | - |  |
| 0x1800D86C0 | Eigen::internal::triangular/householder helper (float) | - | lib:Eigen | - |  |
| 0x1800D87C0 | Eigen::internal::… (float, uses 0x1800CFB50 blocking) | - | lib:Eigen | - |  |
| 0x1800D88E0 | Eigen::internal::… (float, uses 0x1800CFB50 blocking) | - | lib:Eigen | - |  |
| 0x1800D8A00 | Eigen::internal::… (float MatrixXf) | - | lib:Eigen | - |  |
| 0x1800D8B20 | Eigen::internal::… (float MatrixXf) | - | lib:Eigen | - |  |
| 0x1800D8C40 | Eigen::internal::… (float MatrixXf) | - | lib:Eigen | - |  |
| 0x1800D8DB0 | Eigen::internal::… (double) | - | lib:Eigen | - |  |
| 0x1800D8EB0 | Eigen::internal::… (double) | - | lib:Eigen | - |  |
| 0x1800D8FD0 | Eigen::internal::… (double) | - | lib:Eigen | - |  |
| 0x1800D90F0 | Eigen redux maxCoeff over Matrix<double,15,15> (vectorized, max_pd) | - | lib:Eigen (caller preintegration 0x1800ECFD0) | - |  |
| 0x1800D9370 | Eigen redux minCoeff over Matrix<double,15,15> (min_pd) | - | lib:Eigen (caller 0x1800ECFD0) | - |  |
| 0x1800D95F0 | Eigen::internal::generic_product_impl::scaleAndAddTo (float) | - | lib:Eigen | - |  |
| 0x1800D9770 | Eigen::internal::generic_product_impl::scaleAndAddTo (float) | - | lib:Eigen | - |  |
| 0x1800D98F0 | Eigen::internal::generic_product_impl::scaleAndAddTo (double) | - | lib:Eigen | - |  |
| 0x1800D9A70 | Eigen::internal::generic_product_impl::scaleAndAddTo (float) | - | lib:Eigen | - |  |
| 0x1800D9C30 | Eigen::internal::generic_product_impl::scaleAndAddTo (float) | - | lib:Eigen | - |  |
| 0x1800D9E00 | Eigen dense_assignment_loop (float, sub-assign) | - | lib:Eigen | - |  |
| 0x1800D9F20 | Eigen dense_assignment_loop (float, sub-assign) | - | lib:Eigen | - |  |
| 0x1800DA040 | Eigen dense_assignment_loop (float) | - | lib:Eigen | - |  |
| 0x1800DA130 | Eigen dense_assignment_loop (double) | - | lib:Eigen | - |  |
| 0x1800DA250 | Eigen dense_assignment_loop (float) | - | lib:Eigen | - |  |
| 0x1800DA320 | Eigen dense_assignment_loop | - | lib:Eigen | - |  |
| 0x1800DA3E0 | Eigen dense_assignment_loop (float, add) | - | lib:Eigen | - |  |
| 0x1800DA570 | Eigen dense_assignment_loop (float, product coeff) | - | lib:Eigen | - |  |
| 0x1800DA720 | Eigen dense_assignment_loop (float) | - | lib:Eigen | - |  |
| 0x1800DA840 | Eigen dense_assignment_loop (double, product coeff) | - | lib:Eigen | - |  |
| 0x1800DA9F0 | Eigen dense_assignment_loop (scale) | - | lib:Eigen | - |  |
| 0x1800DAAB0 | Eigen dense_assignment_loop | - | lib:Eigen | - |  |
| 0x1800DAB50 | Eigen dense_assignment_loop | - | lib:Eigen | - |  |
| 0x1800DAC10 | Eigen dense_assignment_loop (scale, 8 callers) | - | lib:Eigen | - |  |
| 0x1800DACD0 | Eigen dense_assignment_loop | - | lib:Eigen | - |  |
| 0x1800DAD90 | Eigen::internal::gemv_dense_selector::run (float) | - | lib:Eigen | - |  |
| 0x1800DAF10 | Eigen::internal::gemv_dense_selector::run (float) | - | lib:Eigen | - |  |
| 0x1800DB090 | Eigen::internal::gemv_dense_selector::run (double) | - | lib:Eigen | - |  |
| 0x1800DB230 | boost::serialization::load<binary_iarchive>(DBoW2::BowVector&) | void(Archive&, BowVector&, unsigned) | project (serialization template) | new | via std::map<unsigned,double> |
| 0x1800DB460 | KeyFrame::serialize<binary_iarchive> | void(Archive&, unsigned) | project (serialization template) | new | load of KeyFrame fields |
| 0x1800DB6F0 | boost::serialization::load<binary_iarchive>(cv::Mat&) | void(Archive&, cv::Mat&, unsigned) | project (serialization template) | new | cols/rows/type/continuous + data |
| 0x1800DB8D0 | boost::serialization::save<binary_oarchive>(const QuatTransformation&) | void(Archive&, const Transformation&, unsigned) | project (serialization template) | new | w,x,y,z + Vector3d |
| 0x1800DBB10 | boost::serialization::save<binary_oarchive>(const DBoW2::BowVector&) | - | project (serialization template) | new |  |
| 0x1800DBD40 | KeyFrame::serialize<binary_oarchive> | - | project (serialization template) | new |  |
| 0x1800DBFC0 | boost::serialization::save<binary_oarchive>(const cv::Mat&) | - | project (serialization template) | new |  |
| 0x1800DC190 | boost::serialization::load<portable_binary_iarchive>(DBoW2::BowVector&) | - | project (serialization template) | new |  |
| 0x1800DC3C0 | KeyFrame::serialize<portable_binary_iarchive> | - | project (serialization template) | new |  |
| 0x1800DC670 | boost::serialization::load<portable_binary_iarchive>(cv::Mat&) | - | project (serialization template) | new |  |
| 0x1800DC8A0 | boost::serialization::save<portable_binary_oarchive>(const QuatTransformation&) | - | project (serialization template) | new |  |
| 0x1800DCAE0 | boost::serialization::save<portable_binary_oarchive>(const DBoW2::BowVector&) | - | project (serialization template) | new |  |
| 0x1800DCD10 | KeyFrame::serialize<portable_binary_oarchive> | - | project (serialization template) | new |  |
| 0x1800DCFE0 | boost::serialization::save<portable_binary_oarchive>(const cv::Mat&) | - | project (serialization template) | new |  |
| 0x1800DD270 | Eigen::QuaternionBase<Quaternionf>::setFromTwoVectors | - | lib:Eigen (uses JacobiSVD<Matrix<float,2,3>> 0x1800FEBA0) | - |  |
| 0x1800DD6E0 | Eigen::QuaternionBase<Quaterniond>::setFromTwoVectors | - | lib:Eigen (uses JacobiSVD<Matrix<double,2,3>> 0x1800DE5A0) | - |  |
| 0x1800DDA80 | Utility::skewSymmetric<Vector3d> | Matrix3d(const MatrixBase<Vector3d>&) | project (VINS header template) | identical (VINS) | comma initializer |
| 0x1800DDBB0 | Eigen::internal::triangular_solve_matrix<double,15,…>::run wrapper (Lower) | - | lib:Eigen (PartialPivLU<15x15> solve) | - |  |
| 0x1800DDC50 | Eigen::internal::triangular_solve_matrix<double,15,…>::run wrapper (Upper) | - | lib:Eigen | - |  |
| 0x1800DDCF0 | eulerToRotation<double> (name ours) | Matrix3d(const Vector3d& e, bool is_rad) | project (header template) | new | Rz(e0)*Rx(e1)*Ry(e2), deg->rad unless is_rad |
| 0x1800DE070 | Eigen::ColPivHouseholderQR<MatrixXf>::ColPivHouseholderQR(Index rows, Index cols) | - | lib:Eigen | - |  |
| 0x1800DE1C0 | Eigen::MapBase<Block<…>> ctor (size checks) | - | lib:Eigen | - |  |
| 0x1800DE250 | Eigen::Map<Matrix<double,1,3>> ctor | - | lib:Eigen | - |  |
| 0x1800DE300 | Eigen::Map<Matrix<double,15,7>> ctor | - | lib:Eigen | - |  |
| 0x1800DE3B0 | Eigen::DenseStorage<float,Dynamic> copy ctor | - | lib:Eigen | - |  |
| 0x1800DE400 | Eigen::JacobiSVD<MatrixXd>::JacobiSVD(const MatrixXd&, unsigned) (compute 0x1800FF740) | - | lib:Eigen | - |  |
| 0x1800DE5A0 | Eigen::JacobiSVD<Matrix<double,2,3>>::JacobiSVD(m, ComputeFullV) | - | lib:Eigen | - |  |
| 0x1800DE780 | Eigen::Map<Matrix<double,1,15>> ctor | - | lib:Eigen | - |  |
| 0x1800DE830 | Eigen::Map<Matrix<double,15,1>> ctor | - | lib:Eigen | - |  |
| 0x1800DE8E0 | Eigen::Map<Matrix<double,15,15>> ctor | - | lib:Eigen | - |  |
| 0x1800DE990 | Eigen::Quaterniond::Quaterniond(w,x,y,z) (ctor) | - | lib:Eigen | - |  |
| 0x1800DEA10 | kindr::minimal::RotationQuaternionTemplate<double>::RotationQuaternionTemplate(const Matrix3d&) | - | lib:minkindr (CHECK isValidRotationMatrix, rotation-quaternion-inl.h:102) | - |  |
| 0x1800DEAD0 | Eigen::Transform<double,3,Affine>::makeAffine (last row 0,0,0,1) | - | lib:Eigen | - |  |
| 0x1800DEB20 | std::_Func_impl / boost singleton helper (noreturn, ICF) | - | lib:std/boost | - |  |
| 0x1800DEB50 | std::basic_ifstream<char>::basic_ifstream() | - | lib:std (callers prior-position loader 0x180115F00 etc.) | - |  |
| 0x1800DEC70 | Eigen gemv/gemm micro kernel (float) | - | lib:Eigen | - |  |
| 0x1800DEE60 | Eigen gemv/gemm micro kernel (float) | - | lib:Eigen | - |  |
| 0x1800DF050 | Eigen gemv/gemm micro kernel (float) | - | lib:Eigen | - |  |
| 0x1800DF240 | std::map<double,ImageFrame>::map() | - | lib:std (node 0x1F0) | - |  |
| 0x1800DF280 | boost::archive::detail::oserializer<binary_oarchive, std::pair<const unsigned,double>>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF2B0 | boost::archive::detail::oserializer<binary_oarchive, Eigen::Vector3d>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF2E0 | boost::archive::detail::oserializer<binary_oarchive, cv::Point3f>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF310 | boost::archive::detail::oserializer<binary_oarchive, cv::Point2f>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF340 | boost::archive::detail::oserializer<binary_oarchive, QuatTransformationTemplate<double>>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF370 | boost::archive::detail::oserializer<binary_oarchive, std::map<unsigned,double>>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF3A0 | boost::archive::detail::oserializer<binary_oarchive, std::vector<int>>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF3D0 | boost::archive::detail::oserializer<binary_oarchive, std::vector<KeyFrame*>>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF400 | boost::archive::detail::oserializer<binary_oarchive, std::vector<cv::Point3f>>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF430 | boost::archive::detail::oserializer<binary_oarchive, std::vector<cv::Point2f>>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF460 | boost::archive::detail::oserializer<binary_oarchive, std::vector<std::vector<KeyFrame*>>>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF490 | boost::archive::detail::oserializer<binary_oarchive, std::vector<cv::Mat>>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF4C0 | boost::archive::detail::oserializer<binary_oarchive, DBoW2::BowVector>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF4F0 | boost::archive::detail::oserializer<binary_oarchive, pimax::totem::KeyFrame>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF520 | boost::archive::detail::oserializer<binary_oarchive, cv::Mat>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF550 | boost::archive::detail::oserializer<binary_oarchive, pimax::totem::PlatMap>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF580 | boost::archive::detail::oserializer<portable_binary_oarchive, std::pair<const unsigned,double>>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF5B0 | boost::archive::detail::oserializer<portable_binary_oarchive, Eigen::Vector3d>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF5E0 | boost::archive::detail::oserializer<portable_binary_oarchive, cv::Point3f>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF610 | boost::archive::detail::oserializer<portable_binary_oarchive, cv::Point2f>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF640 | boost::archive::detail::oserializer<portable_binary_oarchive, QuatTransformationTemplate<double>>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF670 | boost::archive::detail::oserializer<portable_binary_oarchive, std::map<unsigned,double>>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF6A0 | boost::archive::detail::oserializer<portable_binary_oarchive, std::vector<int>>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF6D0 | boost::archive::detail::oserializer<portable_binary_oarchive, std::vector<KeyFrame*>>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF700 | boost::archive::detail::oserializer<portable_binary_oarchive, std::vector<cv::Point3f>>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF730 | boost::archive::detail::oserializer<portable_binary_oarchive, std::vector<cv::Point2f>>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF760 | boost::archive::detail::oserializer<portable_binary_oarchive, std::vector<std::vector<KeyFrame*>>>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF790 | boost::archive::detail::oserializer<portable_binary_oarchive, std::vector<cv::Mat>>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF7C0 | boost::archive::detail::oserializer<portable_binary_oarchive, DBoW2::BowVector>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF7F0 | boost::archive::detail::oserializer<portable_binary_oarchive, pimax::totem::KeyFrame>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF820 | boost::archive::detail::oserializer<portable_binary_oarchive, cv::Mat>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF850 | boost::archive::detail::oserializer<portable_binary_oarchive, pimax::totem::PlatMap>::oserializer() | - | lib:boost::serialization (singleton instance ctor, no direct callers) | - |  |
| 0x1800DF880 | 16-byte copy helper (ICF) | - | lib | - |  |
| 0x1800DF890 | std::unique_lock<std::mutex>::unique_lock(mutex&) | - | lib:std | - |  |
| 0x1800DF8D0 | std::unordered_map<…>::unordered_map() | - | lib:std | - |  |
| 0x1800DF950 | std::unordered_map<int,…>::unordered_map() (FrameProcessorBase+3608) | - | lib:std | - |  |
| 0x1800DF9D0 | std::unordered_map<…>::unordered_map() | - | lib:std | - |  |
| 0x1800DFA50 | std::vector<POD>::vector(const vector&) (memmove copy) | - | lib:std | - |  |
| 0x1800DFAE0 | std::vector<8-byte POD>::vector(const vector&) | - | lib:std | - |  |
| 0x1800DFB70 | std::vector<KeyFrame*>::vector(const vector&) | - | lib:std | - |  |
| 0x1800DFC00 | std::vector<…>::vector(const vector&) | - | lib:std (T232 copy) | - |  |
| 0x1800DFCA0 | std::vector<vector<…>>::vector(size_t) | - | lib:std | - |  |
| 0x1800DFD40 | std::vector<shared_ptr<KeyFrame>>::vector(const vector&) | - | lib:std | - |  |
| 0x1800DFDF0 | FrameProcessorBase::FrameProcessorBase | (BaseOptions, ReprojectorOptions, DepthFilterOptions, DetectorOptions, InitializationOptions, FeatureTrackerOptions, CameraBundle::Ptr, int, const bool&) | project | modified | member init (see layout), PerformanceMonitor, modules, grid, prior-position file |
| 0x1800E1950 | IMUFactor::IMUFactor | (const shared_ptr<IntegrationBase>&, const Vector3d&) | project (inline ctor) | modified (VINS) | SizedCostFunction<15,7,3,3,3,7,3,3,3,3> |
| 0x1800E1AA0 | ImageFrame::ImageFrame(const ImageFrame&) | - | project (implicit copy ctor) | - | 448-byte VINS ImageFrame |
| 0x1800E1CC0 | ImageFrame::ImageFrame() | - | project (inline ctor) | identical (VINS) + extra members |  |
| 0x1800E1ED0 | S96::S96() (name unknown) | - | project (implicit ctor) | - | 96 bytes zero (FrameProcessorBase+536, 0x18015E250) |
| 0x1800E1FD0 | S136::S136() (name unknown) | - | project (implicit ctor) | - | gravity (0,0,9.80667), zeros, identity Transformation |
| 0x1800E20B0 | IntegrationBase::IntegrationBase | (const Vector3d& acc0, gyr0, ba, bg) | project (inline ctor) | modified (VINS) | noise 18x18 from member noise params |
| 0x1800E26D0 | KeyFrame::KeyFrame() | - | project (implicit ctor) | - | loop-closing keyframe, 3 cv::Mat, 2 Transformations, BowVector, vectors |
| 0x1800E2910 | T232::T232(const T232&) (name unknown) | - | project (implicit copy ctor) | - | 232-byte struct copy |
| 0x1800E2B10 | S552::S552() (name unknown) | - | project (implicit ctor) | - | Matrix4d identity, 5 x vector<double>(3,0), WORD 1024 |
| 0x1800E2CF0 | std::_System_error::_System_error(error_code, const string&) | - | lib:std | - |  |
| 0x1800E2ED0 | std::_System_error copy ctor | - | lib:std | - |  |
| 0x1800E2F20 | std::pair<…, std::vector<8-byte>> copy ctor | - | lib:std | - |  |
| 0x1800E2FC0 | std::_Task_async_state<void>/_Packaged_state ctor (mutex+condvar+shared_ptr) | - | lib:std::async | - |  |
| 0x1800E3150 | std::exception_ptr::exception_ptr(const exception_ptr&) | - | lib:std | - |  |
| 0x1800E3170 | Concurrency::invalid_operation copy ctor | - | lib:concrt | - |  |
| 0x1800E31B0 | Concurrency::invalid_operation::invalid_operation(const char*) | - | lib:concrt | - |  |
| 0x1800E3200 | std::runtime_error copy ctor | - | lib:std | - |  |
| 0x1800E3240 | std::runtime_error::runtime_error(const char*) | - | lib:std | - |  |
| 0x1800E3290 | std::system_error copy ctor | - | lib:std | - |  |
| 0x1800E32F0 | std::system_error::system_error(error_code) | - | lib:std | - |  |
| 0x1800E33A0 | Eigen::JacobiSVD<MatrixXd>::~JacobiSVD | - | lib:Eigen | - |  |
| 0x1800E3400 | ~X() implicit dtor of a project struct {.., std::map @16, cv::Mat @32/@128/@232} | - | project (implicit dtor) | - | TODO(verify) type |
| 0x1800E34E0 | Eigen::ColPivHouseholderQR<…>::~ (3 frees) | - | lib:Eigen | - |  |
| 0x1800E3510 | std::unique_ptr / vector storage free | - | lib:std | - |  |
| 0x1800E3530 | std::_Hash<…>::~_Hash | - | lib:std | - |  |
| 0x1800E35E0 | std::_Hash<…>::~_Hash | - | lib:std | - |  |
| 0x1800E3660 | std::list node holder dtor (shared_ptr value) | - | lib:std | - |  |
| 0x1800E36E0 | std::_List_node_insert_op2<…>::~ | - | lib:std | - |  |
| 0x1800E3730 | Concurrency::details::_PPLTaskHandle<…>::~_PPLTaskHandle | - | lib:concrt | - |  |
| 0x1800E37B0 | std::_Associated_state<int>::~_Associated_state | - | lib:std::async | - |  |
| 0x1800E3830 | std::_State_manager<…>::~ (future) | - | lib:std::async | - |  |
| 0x1800E3880 | std::unique_ptr<JacobiSVD-like>::~unique_ptr | - | lib:std | - |  |
| 0x1800E38B0 | std::unique_ptr<T>::~unique_ptr | - | lib:std | - |  |
| 0x1800E38C0 | std::_Tree_temp_node / map dtor helper | - | lib:std | - |  |
| 0x1800E38E0 | std::_Tree<…>::~_Tree | - | lib:std | - |  |
| 0x1800E3920 | std::_Tree_temp_node<pair<const double,ImageFrame>>::~ (calls ~ImageFrame 0x1800E4E10) | - | lib:std | - |  |
| 0x1800E3960 | std::vector<std::set<int>>::_Destroy (elements) | - | lib:std | - |  |
| 0x1800E39A0 | std::_Destroy_range<std::vector<…>> | - | lib:std | - |  |
| 0x1800E39E0 | std::deque<shared_ptr<T>>::~deque / _Tidy | - | lib:std | - |  |
| 0x1800E3B10 | std::list<…>::~list | - | lib:std | - |  |
| 0x1800E3B70 | std::list<shared_ptr<T>>::~list | - | lib:std | - |  |
| 0x1800E3BA0 | std::_Tree<…>::~_Tree | - | lib:std | - |  |
| 0x1800E3C00 | std::map<double,shared_ptr<T>>::~map | - | lib:std | - |  |
| 0x1800E3C30 | std::map<double,ImageFrame>::~map | - | lib:std | - |  |
| 0x1800E3C60 | std::pair<const double,ImageFrame>::~pair | - | lib:std | - |  |
| 0x1800E3C70 | Eigen solver dtor (7 frees) | - | lib:Eigen | - |  |
| 0x1800E3CE0 | Eigen solver dtor (8 frees) | - | lib:Eigen | - |  |
| 0x1800E3D50 | std::set<shared_ptr<T>>::~set | - | lib:std | - |  |
| 0x1800E3D80 | std::unique_ptr<…>::~unique_ptr | - | lib:std | - |  |
| 0x1800E3DF0 | std::unique_ptr<Map>::~unique_ptr (Map dtor 0x18012CEB0) | - | lib:std | - |  |
| 0x1800E3E20 | std::unique_ptr<PoseOptimizer-like>::~unique_ptr | - | lib:std | - |  |
| 0x1800E3EF0 | std::unique_ptr<Reprojector>::~unique_ptr | - | lib:std | - |  |
| 0x1800E4030 | std::unique_ptr<Concurrency::details::_Threadpool_chore>::~ | - | lib:concrt | - |  |
| 0x1800E4060 | std::unordered_map<…>::~unordered_map (thunk) | - | lib:std | - |  |
| 0x1800E4070 | std::unordered_map<…>::~unordered_map (thunk) | - | lib:std | - |  |
| 0x1800E4080 | std::vector<std::vector<…>>::~vector | - | lib:std | - |  |
| 0x1800E4140 | std::vector<12-byte POD>::~vector (thunk) | - | lib:std | - |  |
| 0x1800E4150 | std::vector<24-byte POD>::~vector | - | lib:std | - |  |
| 0x1800E41D0 | std::vector<12-byte POD>::~vector | - | lib:std | - |  |
| 0x1800E4250 | std::vector<128-byte POD>::~vector | - | lib:std | - |  |
| 0x1800E42B0 | std::vector<std::future<void>>::~vector | - | lib:std | - |  |
| 0x1800E4320 | std::vector<std::set<int>>::~vector | - | lib:std | - |  |
| 0x1800E43C0 | std::vector<std::unique_ptr<Reprojector>>::~vector | - | lib:std | - |  |
| 0x1800E4450 | std::vector<std::vector<8-byte>>::~vector | - | lib:std | - |  |
| 0x1800E44E0 | std::vector<std::map<…>>::~vector | - | lib:std | - |  |
| 0x1800E45A0 | std::vector<std::vector<shared_ptr<T>>>::~vector | - | lib:std | - |  |
| 0x1800E4630 | BaseOptions::~BaseOptions (std::string trace_dir @+168) | - | project (implicit dtor) | - |  |

## Types

### Matcher (sizeof 336) — sure unless marked
| offset | type | name | evidence |
|---|---|---|---|
| 0 | bool | options_.align_1d | upstream order |
| 4 | int | options_.align_max_iter | findMatchDirect passes `*(a1+4)` to align1D/align2D |
| 8 | double | options_.max_epi_length_optim | upstream order |
| 16 | size_t | options_.max_epi_search_steps | upstream order (no longer read by scanEpipolarUnitPlane) |
| 24/25/26 | bool | subpix_refinement / epi_search_edgelet_filtering / scan_on_unit_sphere | upstream order |
| 32 | double | options_.epi_search_edgelet_max_angle | upstream order |
| 40/41 | bool | verbose / use_affine_warp_ | upstream order; use_affine_warp_ is never read |
| 42/43 | bool | affine_est_offset_ / affine_est_gain_ | passed to align1D/align2D |
| 48 | double | options_.max_patch_diff_ratio | `> ratio * 8.0` |
| 56 | uint8_t[64] | patch_ | createPatchFromPatchWithBorder dst |
| 120 | uint8_t[100] | patch_with_border_ | warpAffine dst |
| 224 | Matrix2d | A_cur_ref_ | getWarpMatrixAffine out |
| 256 | Vector2d | epi_image_ | 0x1800AE820 |
| 272 | double | epi_length_pyramid_ | n_steps = len/0.7 |
| 280 | double | **new** epi length C→A on the search level (our name `epi_length_pyramid_ca_`) | written by 0x1800AE820, read by 0x1800AFD70 |
| 288 | double | **new** epi length C→B on the search level (`epi_length_pyramid_cb_`) | ditto |
| 296 | double | h_inv_ | align1D out |
| 304 | int | search_level_ | |
| 308 | bool | reject_ | 0x1800AE820 |
| 312 | Vector2f | px_cur_ | |
| 320 | Vector3f | f_cur_ | |

Pimax `FloatType` is `float`: Keypoint, BearingVector and GradientVector are float, while Transformation stays
double.

### FeatureWrapper (offsets used)
- `type&` at +0; isEdgelet mask 0x49 = {kEdgeletSeed 0, kEdgeletSeedConverged 3, kEdgelet 6}.
- `px` Ref data at +8.
- `f` Ref data at +32.
- `grad` Ref data at +80.
- `level&` at +112.

### Frame (offsets used here; owner is the common chunk)
| offset | meaning (our name) | evidence |
|---|---|---|
| +16 | int id_ | logs, removeObservation, deque search |
| +32 | int "bundleId()" (TODO name) | keyframe if `cur.+32 >= lastkf.+32 + 33` |
| +40 | CameraPtr cam_ | width +8 / height +12 / type +48 of the camera |
| +64 | Transformation T_f_w_ | q +64..96, t +96..120 |
| +128 | std::vector<cv::Mat> img_pyr_ (96-byte Mat) | |
| +176 | bool is_keyframe_ | setKeyframe 0x180096E80 sets it, then calls 0x180096730 |
| +224 | double img_mean_ (TODO name: image quality / brightness) | `> 15.0` gate on triangulation; scene-depth helper returns false if `< 15` |
| +234 | bool is_redundant_kf_ (Pimax) | makeKeyframe |
| +240 | int64 timestamp_ (ns) | `/1e9` |
| +256 | Transformation T_imu_cam (T_body_cam_) | T_world_imu() |
| +424 / +428 | float min depth / median depth | 0x180094890 |
| +552 | size_t num_features_ | |
| +560 | Keypoints px_vec_ (2×N float; data, cols @+568) | |
| +624 | VectorXi (≤1 filter; level_vec_? TODO) | removeOutliersByFundamentalMat |
| +680 | std::vector<PointPtr> landmark_vec_ | |
| +704 | VectorXi track_id_vec_ (TODO: landmark id?) | −1 = none; numLandmarks counts > −1 |
| +720 | std::vector<SeedRef> (24 bytes each) | |

### FrameBundle (offsets used)
| offset | meaning |
|---|---|
| +0 | `std::vector<FramePtr> frames_` |
| +228 | bool is_stationary_ (TODO name) |
| +229 | bool low_feature_kf_ |
| +232 | size_t num_tracked_ |
| +248 | bool is_keyframe_ |

- `at(i)`: out-of-line copy at 0x1800116D0, checked.
- `numLandmarks()`: 0x180095180.

### Map (offsets used)
- +0: `std::map<int, FramePtr> keyframes_`. `size()` is the map size at +8.
- +128: `std::deque<int> keyframe_ids_`. MSVC deque layout: proxy +128, map +136, mapsize +144, off +152, size +160.
- Map ctor is 0x18012CE00 (184 bytes); the dtor is 0x18012CEB0.

### Camera
- +8: image width (uint32).
- +12: image height (uint32).
- +48: Type (0 = pinhole).
- vtable slot +16: `bool backProject3(const Ref<const Vector2d>&, Vector3d*)`.
- vtable slot +24: `ProjectionResult project3(const Ref<const Vector3d>&, Vector2d*, Matrix<double,2,3>*)`.
  `ProjectionResult` is returned through a hidden pointer; status 0 = KEYPOINT_VISIBLE.

### FrameProcessor (sizeof 4080; vtable 0x1803B2A90)
- Vtable slots:
  0. dtor 0x1800B29B0
  1. 0x180127550 (base)
  2. processFrameBundle 0x1800B4970
  3. resetAll 0x1800B5760
  4. resetBackend 0x18011B380
  5. setTrackingQuality 0x180128260
  6. getMotionPrior 0x18010A690
  7. processFirstFrame 0x1800B4180
  8. processFrame 0x1800B4290
  9. makeKeyframe(int) 0x1800B2F80
- Members:
  - +4032: `StereoTriangulationPtr stereo_triangulation_` (control block `_Ref_count_resource<StereoTriangulation*, default_delete>`, vtable 0x1803B2BC8).
  - +4048: int forced_kf_count_.
  - +4052: int low_match_count_.
  - +4056: `std::vector<FramePtr> last_df_keyframes_`.

### StereoTriangulation (0x90 bytes)
- Ctor 0x180145320.
- +0: 32-byte StereoTriangulationOptions, copied.
- +32: DetectorPtr.
- +48: cv::Mat.
- `compute(FramePtr&, FramePtr&, bool)` is 0x180147110.

### FrameProcessorBase (sizeof 4032) — layout from ctor 0x1800DFDF0
The full member list is in `draft/c07_direct/frontend/frame_processor_base.h`, in offset order with initialisers.
Items worth noting:
- **+8** is not initialised; **+16** = 0.
- **BaseOptions at +24** (256 bytes, upstream member order). Offsets:
  - +0 max_n_kfs
  - +136 `img_align_max_num_features` is **int**
  - +160 poseoptim_using_unit_sphere
  - +164 structure_optimization_max_pts
  - +168 trace_dir (std::string)
  - +200 quality_min_fts
  - +227 trace_statistics
  - +248 **uint16 grid_size** (Pimax)
- **Shared pointers and modules:**
  - +280 cams_; +296 new_frames_; +312 last_frames_; +328 (16-byte shared_ptr, unknown); **+344 last_kf_frames_**.
  - +360 `cv::Ptr<cv::CLAHE>` = createCLAHE(3.0, Size(8,8)).
  - +376 t_lastimu_newimu_; +400 T_world_imuinit; +464 skip_prior_position_ flag.
  - +472 `vector<unique_ptr<Reprojector>>`; +496 `unique_ptr<PoseOptimizer>`; +504 `unique_ptr<DepthFilter>`;
    +512 `unique_ptr<AbstractInitialization>`; +520 16-byte shared_ptr (imu_handler_?).
- **Small counters:** +632 counter (reset in processFirstFrame); +636 low_match_ratio_frames_.
- **Synchronisation and I/O:**
  - std::mutex at +696, +1336, +1448, +1536 and +1896 (80 bytes each).
  - condition_variable at +1824.
  - three std::ofstream at +1976, +2240 and +2504 (264 bytes each).
- **Other members:**
  - Aligned deque at +2784.
  - Transformation(Quaterniond::Identity(), Zero) at +2912.
  - stage_ at +2976; map_ at +2984 (created from a unique_ptr).
  - tracking_quality_ at +3008; overlap_kfs_ at +3032; grid `vector<bool>` at +3080.
  - input_value_ at +3112 (set by addFrameBundle).
  - redundant_kf_count_ at +3116.
  - R_imu_world_ at +3136; R_imulast_world_ at +3168.
  - have_motion_prior_ at +3200; T_newimu_lastimu_prior_ at +3216.
  - use_prior_in_pose_optim_ at +3336.
  - prior_map_ at +3408 (`shared_ptr`, object has int mode at +976 and `deque<FramePtr>` at +120).
  - map_mode_ at +3424 (== 2 → prior-map mode).
  - map_mode_flag_ at +3456 (default true).
  - `std::map<double, ImageFrame> all_image_frame_` at +3480.
  - mesher_ (`make_shared<Mesher>`) at +3592.
  - image_rect_ (`Rect2f(0, 0, w, h)`) at +3728.
  - `steady_clock::now()` at +3984.
- Members after +3536 hold mostly ground-plane, relocalization and IMU parameters with defaults (see Constants).

### Other project structs met here
- **BaseOptions** (256 bytes): above. The implicit dtor is 0x1800E4630.
- **ImageFrame** (VINS, Pimax-extended, 448 bytes):
  - VINS members: points map +0, t +16, R +24, T +96, `shared_ptr<IntegrationBase>` pre_integration +120, is_key_frame +136.
  - Pimax extras: 4 × Transformation (+144, +208, +272, +336) and 2 × Vector3d (+400, +424).
  - Default ctor 0x1800E1CC0, copy ctor 0x1800E1AA0, dtor 0x1800E4E10 (c08).
- **IntegrationBase** (10576 bytes, created with `make_shared`). Ctor 0x1800E20B0; full member offsets are in
  `vio/imu_integration_ctors.cpp`. Pimax adds the members G (0, 0, 9.80667), ACC_N 0.04, ACC_W 0.004, GYR_N 0.008 and
  GYR_W 0.0008 at +0..+56. noise is a fixed 18×18 at +7824.
- **IMUFactor** (80 bytes, `SizedCostFunction<15,7,3,3,3,7,3,3,3,3>`):
  - +40: `shared_ptr<IntegrationBase>`.
  - +56: Vector3d.
  - Ctor 0x1800E1950; the vtable is pimax::totem::IMUFactor.
- **KeyFrame** (loop-closing / PlatMap; ≥ 760 bytes; default ctor 0x1800E26D0).
  - Serialized fields:
    - +0 / +4 / +8 / +12: int.
    - +24: double.
    - +128: Transformation.
    - +352: `vector<Point2f>`.
    - +376: `vector<Mat>`.
    - +400: `vector<int>`.
    - +424: BowVector.
    - +536: `vector<Mat>`.
    - +584: `vector<Point2f>`.
    - +560: `vector<int>` (written after +584).
    - +608: `vector<Point3f>`.
    - +752: double.
  - Not serialized: cv::Mat at +32 / +256 / +440, Transformation at +192, vectors at +632..+752.
- **PlatMap**: the serialized member is +104, `vector<vector<shared_ptr<KeyFrame>>>`, stored as
  `vector<vector<KeyFrame*>>`.
- **S96** (0x1800E1ED0): 96 zero bytes. **S136** (0x1800E1FD0): gravity + 2 × Vector3d + Transformation.
  **S552** (0x1800E2B10): Matrix4d identity + S128 + vectors.
- **T232** (copy ctor 0x1800E2910; move-assign layout from the fork): used by addFrameBundle and loop closing.
- **S28** (vector element, 0x1800B16B0): {int, int, float, float = −1, int64 = 0, int = −1}. Looks like a
  cv::KeyPoint-like struct; caller 0x18017E300.
- **ReprojectResult** (72 bytes, returned by projectMapInFrame 0x180119260):
  - n_matches +0.
  - n_points +8.
  - ave_success_num float +16.
  - stat24 +24, added to n_matches to give n_tracked.
  - stat32 +32.
  - stat48_ratio float +48.

## External interfaces (calls out of c07)
| address | meaning |
|---|---|
| 0x1800AE5A0 | matcher_utils::createPatchFromPatchWithBorder(pwb, 8, patch) |
| 0x1800A84F0 | feature_alignment::align2D(img, pwb, patch, n_iter, offset, gain, Keypoint& px, bool no_simd=false, vector<Vector2f>* = nullptr) |
| 0x1800A63E0 | feature_alignment::align1D(img, Ref<GradientVector> dir, pwb, patch, n_iter, offset, gain, Keypoint* px, double* h_inv) |
| 0x1800AE070 | Matrix2d::inverse helper |
| 0x180013040 / 0x180014A20 / 0x180009C80 / 0x180009B80 / 0x180024870 / 0x1800089C0 / 0x180008930 | minkindr: Transformation::inverse; RotationQuaternion::rotate; quaternion products (both); Transformation*Vector3d; RotationQuaternion(const Quaterniond&) with CHECKs; Transformation() |
| 0x180091A60, 0x180008720, 0x180008B90, 0x1800087D0, 0x180099E90, 0x18009F180, 0x1800080B0, 0x1800ADDF0 | Eigen Map/Block/Ref/constant constructors (lib) |
| 0x1800116D0 | FrameBundle::at(size_t) (std::vector<FramePtr>::at) |
| 0x18007A9D0 | std::vector<std::vector<FramePtr>>::at |
| 0x180095180 | FrameBundle::numLandmarks() |
| 0x180096E80 | Frame::setKeyframe() |
| 0x180094890 | frame_utils scene-depth helper (bool, writes Frame+424/+428; LOGW "Frame %d has no obs!") — our name `computeSceneDepth` |
| 0x18009BBE0 | Point::removeObservation(int frame_id) |
| 0x18012D680 / 0x18012E9F0 / 0x18012EC70 / 0x18012E930 | Map::addKeyframe(const FramePtr&) / removeKeyframe(int) / removeOldestKeyframe() / overlap check(frame, kf) → bool |
| 0x18009FE90 / 0x1800A3270 / 0x18009FC10 / 0x18009F650 | DepthFilter::addKeyframe(const FrameBundlePtr&, Map*) / updateSeeds(const vector<FramePtr>&, const FramePtr&) / get keyframe list(vector<FramePtr>*) → bool / ctor(opts, detector opts, cams) |
| 0x180147110 / 0x180145320 | StereoTriangulation::compute(FramePtr&, FramePtr&, bool) / ctor |
| 0x1800AD840 | feature_detection_utils::makeDetector(opts, cam) (always FastGradDetector) |
| 0x1801B41A0 | vk::NCamera::getCameraShared(idx) |
| 0x180119260 / 0x180118740 / 0x180118D80 / 0x1801188C0 / 0x180128FA0 / 0x1800F4BD0 / 0x18011B420 / 0x180115F00 | FrameProcessorBase: projectMapInFrame / optimizePose(bool) / optimizeStructure(frames, max_pts, iter) / second structure optimizer (prior-map mode) / upgradeSeedsToFeatures / keyframe-removal predicate / resetVisionFrontendCommon / load "/pimax_prior_position.txt" |
| 0x1800E46A0 | FrameProcessorBase::~FrameProcessorBase |
| 0x1801412F0 / 0x18013B2B0 / 0x180132D90 / 0x18012BF30 | Reprojector(opts, cam_idx) / PoseOptimizer::getDefaultSolverOptions / PoseOptimizer ctor / initialization_utils::makeInitializer |
| 0x18012CE00 / 0x18019C020 | Map ctor / Mesher ctor |
| 0x1801B4970 / 0x1801B4DD0 / 0x1801B4F90 / 0x1801B4EF0 / 0x1801B5080 | vk::PerformanceMonitor ctor / dtor / addTimer / addLog / init(name, dir) |
| 0x18011CF30 / 0x18011CD60 / 0x18011D130 | vector<vector<FramePtr>>::resize / vector<set<int>>::resize / vector<bool>::resize |
| 0x180016B90 | printf |
| 0x18000C120 / 0x18000F500 | LOGD / LOGI |
| 0x180354E20 / 0x18035AC70 / 0x1803551B0 / 0x180006290 | glog LogMessage ctor / stream / dtor / operator<<(const char*) |
| 0x18009DA50 | portable_binary_oarchive save(int) |
| 0x18035D4A0 / 0x18035EEF0 | boost basic_oarchive::save_object / basic_iarchive::load_object |
| 0x1800FEBA0, 0x1800FF740 | JacobiSVD compute (float 2×3 / MatrixXd) — lib |

## Globals
| address | meaning |
|---|---|
| 0x18046A000 | logger instance |
| 0x18048EA8C | FLAGS_v (glog) — `VLOG(300)` in findMatchDirect |
| 0x18047DDA0 / 0x18047DDA8 | `std::shared_ptr<vk::PerformanceMonitor> g_permon` (object / ctrl) |
| TlsIndex, TLS +16..+40 | `thread_local std::vector<Eigen::Vector2f>` in warpAffine (guard bit at +40, dtor 0x1803A4A80) |
| 0x18046C5B8 | type_info used by `_Ref_count_resource<StereoTriangulation*>::_Get_deleter` |

## Constants (checked with `rd`)
- **findMatchDirect**
  - Boundary 6 (kHalfPatchSize + 2).
  - Too-far threshold `max_patch_diff_ratio * 8.0`.
  - Return codes: 3 visibility, 4 warp, 5 alignment, 10 too far.
  - VLOG level 300, matcher.cpp line 221.
- **scanEpipolarUnitPlane**
  - Step `len/0.7`.
  - Per-side cap 15.0.
  - Patch margin 8 (kPatchSize).
- **ZMSSD**: threshold 128000 (2000 × 64).
- **getWarpMatrixAffine**
  - Offsets {8,0} at 0x1803B2990 and {0,8} at 0x1803B29A0.
  - Divisor {8,8} at 0x1803B29B0.
- **getBestSearchLevel**: 3.0, 0.25.
- **warpAffine**: 1.0f at 0x1803AE9DC.
- **processFrame**
  - 0.1 at 0x1803B2C00 (double).
  - 0.72f at 0x1803B2BEC; 0.65f at 0x1803B2BE8, used when n_matches > 300 (0x12C).
  - n_points < 150 (0x96).
  - Counter > 3.
  - 3.5f at 0x1803B2C28; 0.8f at 0x1803B2BF0.
  - Bundle id + 33.
  - 1e9 at 0x1803ADDF8.
  - 0.25 at 0x1803AF2B0.
  - 0.4f widened to double, at 0x1803B2C10.
  - Map size < 8.
  - numLandmarks > 10.
  - optimizeStructure(…, structure_optimization_max_pts, 5); prior-map variant (…, 10, 5).
- **makeKeyframe**
  - 0.05 at 0x1803AF2A8; 0.3 at 0x1803B2C08; 1.0 at 0x1803ADDD0; 1e-6 at 0x1803AF298; abs mask 0x1803ADE90.
  - 15.0 at 0x1803AF8C8.
  - numLandmarks thresholds ≥ 200, < 150, < 100.
  - Redundant count ≥ 3.
  - Translation 0.01 at 0x1803B2BF8.
  - 4-camera rig check `size() == 4`.
- **removeOutliersByFundamentalMat**
  - Minimum 15 features and 15 matches.
  - Filter value ≤ 1.
  - `FM_RANSAC` (8), 1.5, 0.99.
- **FrameProcessorBase ctor**
  - CLAHE 3.0 (0x1803AF8A0) and 8×8.
  - Floats 0.01f, 0.0075f, 0.006f, 0.001f, 8e-5f.
  - 20, 6, 3, 0.02, 0.08, 0.15, 300, 2.0, 0.01, several −1.0, −1 and true defaults (see header).
- **IntegrationBase**: 9.80667, 0.04, 0.004, 0.008, 0.0008.
- **eulerToRotation**: 180.0 at 0x1803B6D38; π at 0x1803B1390.

## Quirks to preserve
1. **findMatchDirect**
   - The corner (align2D) path has no kFailTooFar check.
   - A failing backProject3 returns kFailAlignment after px_cur and px_cur_ were already written.
   - `use_affine_warp_` is ignored.
2. **scanEpipolarUnitPlane**
   - `n_steps` is not clamped to `max_epi_search_steps`. The loop length is `min(15, len_ca/0.7) + min(15, len_cb/0.7)`.
   - If `n_steps == 0` (len < 0.7), the step divides by 0 (inf/NaN) and is still used.
3. **warpAffine**
   - Rejects `xi <= 0` / `yi <= 0`; upstream rejected only `< 0`.
   - Pass 2 uses `(int)` truncation rather than floor; this is equivalent only because pass 1 guaranteed positive values.
4. **processFrame**
   - The `"# tracking_result: %d, %d, %f, %f, %f, %d, %d"` log passes size_t values for `%d`.
   - The 0.4 s keyframe spacing is compared against the float literal 0.4f widened to double.
   - `last_kf_frames_->frames_[0]` and `new_frames_->frames_[0]` are dereferenced without checks. This is safe only
     because the tracking stage always follows processFirstFrame.
5. **makeKeyframe**
   - In the temporal-triangulation block, `last_kf_frames_->at(0)` is dereferenced before the `if (last_kf_frames_)`
     null check.
   - frames[2]/[3] are accessed with at() without a `size() == 4` check, so this throws on 2-camera rigs when
     input_value < 1 and < 100 landmarks.
   - With map_mode_ == 2, if `std::find` does not find redundant_ids[0] in Map::keyframe_ids_, the id stays in
     redundant_ids. The next iteration removes the same id again; Map::removeKeyframe then logs "Cannot find the
     keyframe with id ...".
6. **removeOutliersByFundamentalMat**: calls removeObservation on a possibly null landmark when the frame is a keyframe.
7. **FrameProcessorBase ctor**
   - The grid divisor `options_.grid_size` is uint16. If it is 0, this is a division by zero.
   - The vector<bool> is resized to `n_cells` and then cleared bit by bit again.
8. **IntegrationBase**: the noise parameters are per-instance members; VINS used globals.
9. **cv::Mat serialization**: `data_size` is `unsigned int`, so it truncates matrices larger than 4 GB.

## Open questions / TODO(verify)
- Names: Frame +32 (bundle id?), +224 (image mean?), +234, +624, +704; FrameBundle +228 / +229; FrameProcessorBase
  +632, +3112, +3424, +3456, +3408 type; DepthFilter 0x18009FC10; FrameProcessorBase 0x1800F4BD0 and 0x1801188C0. All
  need confirmation by c08 or the common chunk.
- Whether `frame->T_f_w_` renormalisation in processFrame (stationary path) is explicit or comes from a Pimax
  `set_T_w_imu`.
- Whether initPerformanceMonitor exists as a separate inline member (inlined into the ctor here).
- The exact spelling of the unique_ptr temporaries used for `reprojectors_` / `pose_optimizer_` / `map_` /
  `stereo_triangulation_`. The binary shows unique_ptr move-assignment with a self-check, and
  `_Ref_count_resource` for the shared_ptr members.
- The `quaternionAngle` helper (fabs applied twice) may be a Pimax header function. No out-of-line copy exists.
- The Eigen version: nothing here pins 3.3 vs 3.4 (`eulerToRotation` element stores are compatible with both).
