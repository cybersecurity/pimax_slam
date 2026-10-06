# c15_platmap — 0x180172460 .. 0x180185e40

Drafts (`draft/c15_platmap/`):
- `loop_closing/platmap.h`, `loop_closing/platmap.cpp` — `KeyFrame`, `MapIndex`, `PlatMap`, every
  boost::serialization `serialize/save/load` used by the map file (PlatMap, KeyFrame,
  vector<vector<shared_ptr<KeyFrame>>>, cv::Mat, cv::Point_, cv::Point3_, Eigen::Matrix (only
  Vector3d is instantiated — not Matrix3d), kindr QuatTransformation, DBoW2::BowVector via
  std::map<unsigned,double>).
- `loop_closing/bow.h`, `loop_closing/bow.cpp` — Pimax versions of upstream `bow.cpp` functions.
- `loop_closing/geometric_verification.cpp` — `commonLandMarkCheck`.
- `loop_closing/utility.h` — `Utility::ypr2R` (VINS-style, with `is_radian` flag).
- `loop_closing/loop_closing_c15.h`, `loop_closing/loop_closing_ctor.cpp` — `LoopClosureOptions`,
  `LoopClosing` layout as seen by the ctor/dtor, `LoopClosing::LoopClosing`, `~LoopClosing`.
- `third_party/DBoW2/TemplatedVocabulary_pimax_patch.h` — the two Pimax changes to DBoW2
  (`load(const std::string&)` modified, `fromStream(std::istream&)` added; DBoW3 port).
- `common/portable_archive/*` — **verbatim** copies of boost 1.74.0
  `libs/serialization/example/portable_binary_{archive.hpp,iarchive.hpp,iarchive.cpp,oarchive.hpp,oarchive.cpp}`
  (identified, see below; the compiled code lives at 0x18009c8xx–0x18009dxxx, outside this chunk).
- `test/gen_platmap_sample.cpp` — generator of synthetic PlatMap archives (native + portable) with
  the same serialize bodies, used to validate `tools/parse_platmap.py`.

`tools/parse_platmap.py` — parser for PlatMap archives (native binary and portable) and the YAML index.

## Overview

The range is **not** a single object file. In address order it contains:

1. 0x180172460–0x180178aa0: the `DBoW2::TemplatedVocabulary<cv::Mat, DBoW2::FORB>` instantiation
   (all 18 virtuals + helpers, vtable 0x1803bdb18) mixed with the Pimax versions of the upstream
   `svo_online_loopclosing/src/bow.cpp` functions (`changeStructure`, `extractBoWFeaturesFromImage`,
   `createBOW`, `compareBOWs`, `getNodeID`, `extractFeaturesFromSVOKeypoints`) and their STL helpers.
   The DBoW2 template is upstream (reference/DBoW2 @3924753) **except** `load(const std::string&)`
   (0x180176910) and the new non-virtual `fromStream(std::istream&)` (0x180175d80), which are a
   port of DBoW3 (binary `.dbow` vocabularies, quicklz-compressed). `../slam/voc_GEN_8X4.dbow`
   starts with the DBoW3 signature 0x14B1863F81 (88877711233), compressed=1,
   nnodes=0x1249=4681 (k=8, L=4: 1+8+64+512+4096).
2. 0x180178d10–0x18017f4a0: mostly STL/Eigen/OpenCV template code whose first user is
   loop_closing.cpp (many callers are ReLocalize 0x18018b600/0x18018d770, the LoadSavePlatMap
   thread 0x18018b3d0, etc. in the next chunk), plus `commonLandMarkCheck` (0x180178e40),
   `Utility::ypr2R` (0x18017ee10) and `std::make_shared<PlatMap>` (0x18017e8e0, PlatMap ctor).
3. 0x18017f4c0 `KeyFrame::KeyFrame`, 0x18017f730 `LoopClosing::LoopClosing` (5.5 KB),
   0x180180cd0 `LoopClosureOptions` copy ctor, 0x180181ae0 `LoopClosing::~LoopClosing`.
4. 0x180182960–0x180185110: `PlatMap` methods (dtor, UpdateMap, BuildMapIndex, load, loadIndex,
   load_bin, save, saveIndex, Map2Vec, Vec2Map).
5. 0x180185490–0x180185c20: STL tails. LoopClosing methods continue at 0x180185e40 (next chunk).

The boost serializer *instantiations* (`oserializer/iserializer/pointer_*serializer<{binary,
portable_binary}_{o,i}archive, T>` vtables 0x1803b40f0–0x1803b6c08, bodies 0x180116xxx–0x180127xxx,
and the serialize bodies 0x1800b65b0 .. 0x1800dcfe0) are emitted **outside** this chunk (an earlier
object uses them first); they were decompiled here anyway because the file format depends on them,
and are reconstructed in `platmap.h`. Address list in "External interfaces".

Source placement guess: the `bow.cpp` functions and the DBoW2 instantiation belong to
`src/loop_closing/bow.cpp`(?), the rest to `src/loop_closing/loop_closing.cpp`; PlatMap/KeyFrame
are probably declared in a header (`platmap.h`?) included by loop_closing.cpp. TODO(verify) file split.

Counts: 213 functions; 26 project (incl. implicit ctor/dtors and one lambda body belonging to
ReLocalize), 187 lib (of which 32 are DBoW2 template members / scoring helpers, 2 of them
Pimax-modified/added and reconstructed as a patch).

## Function table

| address | size | proposed qualified name | signature | kind | upstream status | summary |
|---|---|---|---|---|---|---|
| 180172460 | 667 | std::unordered_map<pair<int64,int64>,vector<size_t>,PairHash>::_Try_emplace |  | lib:STL unordered_map (key pair<long long,long long>) | - | operator[] core; caller extractFeaturesFromSVOKeypoints |
| 180172700 | 157 | std::_Uninitialized_copy<DBoW2::Node> |  | lib:STL | - | copy-construct Node range (152-byte Node) |
| 1801727A0 | 127 | std::_Uninitialized_move<vector<unsigned>> |  | lib:STL | - | vector<vector<unsigned>> realloc |
| 180172820 | 81 | std::runtime_error::runtime_error(const std::string&) |  | lib:CRT/STL | - | used by Vocabulary::load |
| 180172880 | 132 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::~TemplatedVocabulary |  | lib:DBoW2 template | identical | delete m_scoring_object, ~m_words, ~m_nodes |
| 180172910 | 115 | std::_Hash<unordered_map<pair<int64,int64>,vector<size_t>>>::~_Hash |  | lib:STL | - |  |
| 180172990 | 137 | std::_List_node_emplace_op2<pair<const pair<int64,int64>,vector<size_t>>>::~ |  | lib:STL | - | node holder dtor |
| 180172A20 | 53 | std::_Destroy_range<DBoW2::Node> |  | lib:STL | - |  |
| 180172A60 | 16 | std::_Destroy_range<vector<unsigned>> (wrapper) |  | lib:STL | - |  |
| 180172A70 | 35 | std::list<pair<const pair<int64,int64>,vector<size_t>>>::_Tidy |  | lib:STL | - |  |
| 180172AA0 | 5 | thunk -> 0x180172910 |  | lib:STL | - |  |
| 180172AB0 | 180 | std::vector<DBoW2::Node>::_Tidy |  | lib:STL | - |  |
| 180172B70 | 138 | std::vector<vector<unsigned>>::_Tidy |  | lib:STL | - |  |
| 180172C00 | 103 | DBoW2::TemplatedVocabulary::Node::~Node |  | lib:DBoW2 template | identical | ~descriptor Mat, ~children |
| 180172C70 | 186 | cv::operator<<(FileStorage&, const char*) |  | lib:OpenCV inline | - | fs << String(str) |
| 180172D30 | 52 | DBoW2::TemplatedVocabulary<cv::Mat,FORB> scalar deleting dtor |  | lib:DBoW2 (vtable slot 0) | identical |  |
| 180172D70 | 43 | DBoW2::GeneralScoring scalar deleting dtor |  | lib:DBoW2 | identical | shared slot 2 of all scoring vtables |
| 180172DA0 | 3567 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::HKmeansStep | (NodeId parent_id, const vector<pDescriptor>&, int current_level) | lib:DBoW2 template | identical | recursive k-means step |
| 180173B90 | 221 | std::vector<DBoW2::Node>::_Change_array |  | lib:STL | - |  |
| 180173C70 | 192 | std::vector<vector<unsigned>>::_Change_array |  | lib:STL | - |  |
| 180173D30 | 52 | std::_Destroy_range<DBoW2::Node> |  | lib:STL | - |  |
| 180173D70 | 20 | std::_Destroy_range<vector<unsigned>> (alloc variant) |  | lib:STL | - |  |
| 180173D90 | 410 | std::_Hash<unordered_map<pair<int64,int64>,...>>::_Forced_rehash |  | lib:STL | - |  |
| 180173F30 | 131 | std::vector<DBoW2::Node>::_Reallocate_exactly |  | lib:STL | - | reserve |
| 180173FC0 | 131 | std::vector<vector<unsigned>>::_Reallocate_exactly |  | lib:STL | - | reserve |
| 180174050 | 226 | std::vector<cv::KeyPoint>::_Reallocate_exactly |  | lib:STL | - | reserve |
| 180174140 | 124 | std::_Uninitialized_value_construct_n<DBoW2::Node> |  | lib:STL | - | resize grow |
| 1801741C0 | 196 | std::_Uninitialized_move<DBoW2::Node> |  | lib:STL | - |  |
| 180174290 | 111 | std::allocator<DBoW2::Node>::allocate |  | lib:STL | - | 152*n, 32-byte aligned big block |
| 180174300 | 283 | pimax::totem::changeStructure | void (const cv::Mat& plain, std::vector<cv::Mat>* out) | project | modified: reserve+push_back(row(i)) instead of resize+assign (appends) | bow.cpp |
| 180174420 | 95 | std::vector<DBoW2::Node>::clear |  | lib:STL | - |  |
| 180174480 | 19 | pimax::totem::compareBOWs | double (const BowVector&, const BowVector&, const OrbVocabulary&) | project | identical | voc.score(v1,v2) (m_scoring_object->score) |
| 1801744A0 | 733 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::create(const vector<vector<Mat>>&) |  | lib:DBoW2 template (vtable slot 3) | identical |  |
| 180174780 | 15 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::create(training, int k, int L) |  | lib:DBoW2 template (slot 2) | identical |  |
| 180174790 | 66 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::create(training, k, L, weighting, scoring) |  | lib:DBoW2 template (slot 1) | identical |  |
| 1801747E0 | 237 | pimax::totem::createBOW | void (const vector<Mat>& feature, const OrbVocabulary& voc, BowVector* v, vector<int>* node_id) | project | identical (getNodeID inlined, levelup 0) | bow.cpp |
| 1801748D0 | 324 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::createScoringObject |  | lib:DBoW2 template | identical | new L1/L2/ChiSquare/KL/Bhattacharyya/DotProduct scoring (8-byte objects) |
| 180174A20 | 233 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::createWords |  | lib:DBoW2 template | identical |  |
| 180174B10 | 66 | std::allocator<DBoW2::Node>::deallocate |  | lib:STL | - | EH unwind |
| 180174B60 | 12 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::empty |  | lib:DBoW2 template (slot 5) | identical |  |
| 180174B70 | 1517 | pimax::totem::extractBoWFeaturesFromImage | void (const cv::Mat& image, vector<Point2f>* kps, vector<Mat>* feature) | project | modified: static ORB(500,1.2,4,31,0,2,HARRIS,31,20); response-sorted multimap + 2px OccupandyGrid2D thinning; kps not cleared | bow.cpp |
| 180175160 | 3104 | pimax::totem::extractFeaturesFromSVOKeypoints | void (const Mat& image, vector<Point3f>* lm, vector<int>* lm_ids, vector<int>* track_ids, vector<Point2f>* kps, vector<Mat>* features, Mat* descriptors) | project | modified heavily (see notes) | bow.cpp |
| 180175D80 | 1441 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::fromStream(std::istream&) |  | lib:DBoW2 template, PIMAX-ADDED (DBoW3 port) | new | binary .dbow loader, quicklz chunks; reconstructed in third_party/DBoW2 patch |
| 180176330 | 227 | pimax::totem::getNodeID | void (const vector<Mat>&, const OrbVocabulary&, int levelup, vector<int>* node_ids) | project | identical | bow.cpp |
| 180176420 | 50 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::getParentNode |  | lib:DBoW2 template (slot 11) | identical |  |
| 180176460 | 41 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::getWord |  | lib:DBoW2 template (slot 12) | identical |  |
| 180176490 | 16 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::getWordWeight |  | lib:DBoW2 template (slot 13) | identical |  |
| 1801764A0 | 5 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::initiateClusters (thunk) |  | lib:DBoW2 template (slot 17) | identical | -> initiateClustersKMpp |
| 1801764B0 | 1116 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::initiateClustersKMpp |  | lib:DBoW2 template | identical | rand()-based kmeans++ (RandomInt/RandomValue inlined) |
| 180176910 | 1083 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::load(const std::string&) |  | lib:DBoW2 template, PIMAX-MODIFIED | modified: binary signature check -> fromStream, else FileStorage | see patch |
| 180176D50 | 950 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::load(const FileStorage&, const string& name) |  | lib:DBoW2 template (slot 15) | identical |  |
| 180177110 | 68 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::TemplatedVocabulary(const std::string&) |  | lib:DBoW2 template | identical | only caller LoopClosing ctor |
| 180177160 | 9 | DBoW2::L1Scoring::mustNormalize (folded: ChiSquare/KL/Bhattacharyya) |  | lib:DBoW2 | identical | norm=L1, true |
| 180177170 | 9 | DBoW2::DotProductScoring::mustNormalize |  | lib:DBoW2 | identical | norm=L1, false |
| 180177180 | 9 | DBoW2::L2Scoring::mustNormalize |  | lib:DBoW2 | identical | norm=L2, true |
| 180177190 | 168 | std::vector<DBoW2::Node>::emplace_back(Node&&) |  | lib:STL | - | m_nodes.push_back(Node(0)) |
| 180177240 | 240 | std::vector<DBoW2::Node>::resize |  | lib:STL | - |  |
| 180177330 | 3627 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::save(FileStorage&, const string& name) |  | lib:DBoW2 template (slot 14) | identical |  |
| 180178160 | 1148 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::setNodeWeights |  | lib:DBoW2 template | identical | TF_IDF/IDF via vector<bool> counted |
| 1801785E0 | 13 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::size |  | lib:DBoW2 template (slot 4) | identical |  |
| 1801785F0 | 44 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::stopWords |  | lib:DBoW2 template (slot 16) | identical |  |
| 180178620 | 33 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::transform(const Mat&, WordId&) |  | lib:DBoW2 template (slot 6) | identical |  |
| 180178650 | 512 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::transform(const Mat&, WordId&, WordValue&, NodeId*, int levelsup) |  | lib:DBoW2 template (slot 7) | identical |  |
| 180178850 | 71 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::transform(const Mat&) -> WordId |  | lib:DBoW2 template (slot 8) | identical |  |
| 1801788A0 | 505 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::transform(const vector<Mat>&, BowVector&) |  | lib:DBoW2 template (slot 10) | identical |  |
| 180178AA0 | 617 | DBoW2::TemplatedVocabulary<cv::Mat,FORB>::transform(const vector<Mat>&, BowVector&, FeatureVector&, int) |  | lib:DBoW2 template (slot 9) | identical |  |
| 180178D10 | 20 | cv::_InputArray::_InputArray(const std::vector<cv::Point3f>&) |  | lib:OpenCV inline | - | flags 0x81030015; used by ReLocalize 0x18018d770 |
| 180178D30 | 260 | std::vector<T*>::_Resize_reallocate (value-init) |  | lib:STL (COMDAT shared with Ceres) | - | callers 0x1801cdbe0, 0x180256a60 |
| 180178E40 | 165 | pimax::totem::commonLandMarkCheck | bool (const vector<int>& ids1, const vector<int>& ids2, double th) | project | modified: empty ids1 -> false, no isnan term | caller 0x180185e40 |
| 180178EF0 | 508 | std::vector<Eigen::VectorXd>::_Assign_range |  | lib:STL/Eigen | - | PlatMap::D_list_ = LoopClosing::D_list_ |
| 1801790F0 | 307 | std::copy<VectorXd> (assign loop) |  | lib:STL/Eigen | - |  |
| 180179230 | 74 | Eigen::VectorXd copy-construct (in place) |  | lib:Eigen | - |  |
| 180179280 | 92 | Eigen::MatrixXd copy-construct |  | lib:Eigen (COMDAT, Ceres callers) | - |  |
| 1801792E0 | 51 | std::_Destroy_range<VectorXd> |  | lib:STL | - |  |
| 180179320 | 50 | std::_Destroy_range<VectorXd> (alloc variant) |  | lib:STL | - |  |
| 180179360 | 50 | std::_Destroy_range<MatrixXd> |  | lib:STL | - |  |
| 1801793A0 | 69 | fprintf (CRT inline) |  | lib:CRT | - | callers glog/ceres |
| 1801793F0 | 72 | Eigen aligned placement copy (Transformation-like, 16B-aligned) |  | lib:Eigen | - | ReLocalize |
| 180179440 | 54 | std::unique_ptr<T (virtual dtor)>::operator=(unique_ptr&&) |  | lib:STL | - | ReLocalize |
| 180179480 | 50 | std::unique_ptr<T (Eigen aligned, free)>::operator=(&&) |  | lib:STL | - | ReLocalize |
| 1801794C0 | 67 | std::unique_ptr<std::thread>::operator=(&&) |  | lib:STL | - | terminate() if old thread joinable; startAllThread |
| 180179510 | 90 | std::pair<int, std::map<...>>::operator=(&&) |  | lib:STL | - | ReLocalize sort helper |
| 180179570 | 352 | cv::MatCommaInitializer_<double>::MatCommaInitializer_(Mat_<double>*) + first value |  | lib:OpenCV inline | - | K = (Mat_<double>(3,3) << ...) |
| 1801796D0 | 92 | std::operator+(const char*, std::string&&) |  | lib:STL | - |  |
| 180179730 | 634 | std::vector<vector<shared_ptr<KeyFrame>>>::_Assign_range |  | lib:STL | - | kf_map_[id] = {kfs} |
| 1801799B0 | 197 | std::_Tree<map<int,MapIndex>>::_Copy_nodes |  | lib:STL | - | map_index_ = local map (loadIndex) |
| 180179A80 | 363 | std::vector<cv::Point2f>::_Emplace_reallocate(double&,double&) |  | lib:STL | - |  |
| 180179BF0 | 486 | std::vector<cv::Point3f>::_Emplace_reallocate(double&,double&,double&) |  | lib:STL | - |  |
| 180179DE0 | 557 | std::vector<Eigen::VectorXd>::_Emplace_reallocate(const VectorXd&) |  | lib:STL | - | D_list_.push_back |
| 18017A010 | 658 | std::vector<T24>::_Emplace_reallocate (T24 = 3 x 8-byte pairs) |  | lib:STL | - | ReLocalize |
| 18017A2B0 | 341 | std::vector<32-byte POD>::_Emplace_reallocate |  | lib:STL | - | caller 0x1801891b0 |
| 18017A410 | 453 | std::vector<24-byte POD>::_Emplace_reallocate |  | lib:STL | - | caller 0x1801891b0 |
| 18017A5E0 | 386 | std::vector<16-byte POD>::_Emplace_reallocate |  | lib:STL | - | ReLocalize |
| 18017A770 | 405 | std::vector<cv::Point3f>::_Emplace_reallocate(const Point3f&) |  | lib:STL | - |  |
| 18017A910 | 120 | std::_Tree<map<int,int,less,Eigen::aligned_allocator>>::_Erase |  | lib:STL | - | recursive, free() |
| 18017A990 | 620 | std::_Tree<multimap<float,...>>::_Find_hint |  | lib:STL | - | ReLocalize |
| 18017AC00 | 202 | std::_Hash<unordered_map<string,X>>::_Find_last |  | lib:STL | - |  |
| 18017ACD0 | 150 | std::list<pair<const string,X>>::_Free_non_head |  | lib:STL | - |  |
| 18017AD70 | 109 | std::list node (string key) deallocate |  | lib:STL | - |  |
| 18017ADE0 | 38 | std::_Get_size_of_n<8> |  | lib:STL | - |  |
| 18017AE10 | 815 | std::vector<cv::Point2f>::_Insert_range |  | lib:STL | - | recovery_kf |
| 18017B140 | 873 | std::vector<vector<shared_ptr<KeyFrame>>>::_Insert_range |  | lib:STL | - | Map2Vec |
| 18017B4B0 | 806 | std::vector<cv::Mat>::_Insert_range |  | lib:STL | - | recovery_kf |
| 18017B7E0 | 1268 | std::_Insertion_sort_unchecked<pair<int,map<float,..>>*, ReLocalize lambda> |  | lib:STL sort | - | comparator 0x1801826f0 |
| 18017BCE0 | 254 | std::_Med3_unchecked<...same sort> |  | lib:STL sort | - |  |
| 18017BDE0 | 45 | std::copy<cv::Point2f> |  | lib:STL | - |  |
| 18017BE10 | 119 | std::_Move_unchecked<vector<shared_ptr<KeyFrame>>*> |  | lib:STL | - | vector::erase |
| 18017BE90 | 75 | std::copy<cv::Mat> (operator=) |  | lib:STL | - |  |
| 18017BEE0 | 3911 | std::_Partition_by_median_guess_unchecked<...same sort> |  | lib:STL sort | - |  |
| 18017CE30 | 664 | std::_Pop_heap_hole_by_index<...same sort> |  | lib:STL sort | - |  |
| 18017D0D0 | 603 | std::_Push_heap_by_index<...same sort> |  | lib:STL sort | - |  |
| 18017D330 | 758 | std::_Sort_unchecked<...same sort> |  | lib:STL sort | - | recursive; caller 0x18018b600 (ReLocalize) |
| 18017D630 | 738 | std::unordered_map<int, std::map<..>>::_Try_emplace |  | lib:STL | - | ReLocalize |
| 18017D920 | 631 | std::unordered_map<string, enum>::operator[] |  | lib:STL | - | kStrToScaleRetMap / kStrToGlobalMapType lookup in ctor |
| 18017DBA0 | 96 | std::_Uninitialized_copy<vector<shared_ptr<KeyFrame>>> |  | lib:STL | - |  |
| 18017DC00 | 96 | std::_Uninitialized_copy<cv::Mat> |  | lib:STL | - |  |
| 18017DC60 | 532 | std::deque<T112>::_Assign_range |  | lib:STL | - | ReLocalize (T112: project struct, copy 0x180182450) |
| 18017DE80 | 105 | std::distance<map iterator> |  | lib:STL | - | ReLocalize |
| 18017DEF0 | 775 | std::unordered_map<string, bool>::_Try_emplace(string&&) |  | lib:STL | - | caller 0x180185e40 |
| 18017E200 | 242 | std::deque<T112>::emplace_back(int,int64,int,56-byte) |  | lib:STL | - | ReLocalize |
| 18017E300 | 95 | std::vector<cv::KeyPoint>::emplace_back(float,float,int) |  | lib:STL | - | KeyPoint(x,y,(float)size); ReLocalize |
| 18017E360 | 59 | std::vector<cv::Point2f>::emplace_back(double,double) |  | lib:STL | - |  |
| 18017E3A0 | 91 | std::vector<cv::Point3f>::emplace_back(double,double,double) |  | lib:STL | - |  |
| 18017E400 | 33 | std::vector<8-byte>::push_back |  | lib:STL | - | ReLocalize |
| 18017E430 | 40 | std::vector<12-byte>::push_back |  | lib:STL | - | ReLocalize |
| 18017E460 | 46 | std::vector<cv::Point3f>::push_back |  | lib:STL | - |  |
| 18017E490 | 39 | std::vector<cv::Point2f>::push_back |  | lib:STL | - |  |
| 18017E4C0 | 52 | std::vector<cv::Point3f>::emplace_back(converting) |  | lib:STL | - |  |
| 18017E500 | 33 | std::vector<8-byte>::push_back |  | lib:STL | - |  |
| 18017E530 | 660 | std::unordered_map<string, enum>::insert(first,last) |  | lib:STL | - | used by .text$di 0x180002bf0/0x180002df0 (initializer_list ctors) |
| 18017E7D0 | 144 | std::make_shared<ceres_backend::General3DParameterBlock> |  | lib:STL | - | ReLocalize |
| 18017E860 | 125 | std::make_shared<ceres_backend::Map>(bool) |  | lib:STL | - | alloc 0x590; ReLocalize |
| 18017E8E0 | 396 | std::make_shared<pimax::totem::PlatMap>() |  | project (PlatMap ctor inlined) | new | sizeof(PlatMap)=0xE0; max_kf_num_=1000; clears |
| 18017EA70 | 141 | std::make_shared<ceres_backend::PoseParameterBlock> |  | lib:STL | - |  |
| 18017EB00 | 259 | std::make_shared<ceres_backend::ReprojectionError> |  | lib:STL | - | alloc 0x100 |
| 18017EC10 | 159 | aligned new pimax::totem::Frame(...) (0x340 bytes) |  | lib:new-expression | - | Frame ctor 0x180092420; ReLocalize |
| 18017ECB0 | 351 | aligned new + value-init of a 352-byte project struct |  | lib:new-expression | - | defaults int@4=10, double@8=2.0, size_t@16=100, double@32=0.5, double@48=2.5; ReLocalize; TODO type |
| 18017EE10 | 892 | pimax::totem::Utility::ypr2R<Eigen::Vector3d> | Matrix3d (const MatrixBase<Vector3d>& ypr, bool is_radian) | project template | modified (VINS ypr2R + is_radian flag) | Rz*Ry*Rx; caller ReLocalize |
| 18017F190 | 73 | Eigen::Matrix<double,2,Dynamic> copy-construct |  | lib:Eigen | - |  |
| 18017F1E0 | 29 | trivial ctor {double,double,double,int64} |  | lib:inline ctor | - | ReLocalize |
| 18017F200 | 41 | std::lock_guard<std::mutex>::lock_guard |  | lib:STL | - |  |
| 18017F230 | 219 | std::map<int,..> copy-ctor |  | lib:STL | - | ReLocalize |
| 18017F310 | 285 | std::vector<shared_ptr<T>> copy-ctor |  | lib:STL | - |  |
| 18017F430 | 98 | closure ctor (11 captures) |  | lib:lambda object | - | ReLocalize |
| 18017F4A0 | 28 | ceres::HuberLoss::HuberLoss(double a) |  | lib:Ceres inline | - | a_=a, b_=a*a |
| 18017F4C0 | 622 | pimax::totem::KeyFrame::KeyFrame | (int nframe_id, int cam_id, int frame_id, int map_id) | project | new (upstream KeyFrame(int)) | member inits + 4 assignments |
| 18017F730 | 5531 | pimax::totem::LoopClosing::LoopClosing | (const LoopClosureOptions&, const CameraBundlePtr& cams, const std::string& map_tag, LoopClosingMode mode) | project | modified (see notes) | 5.5 KB |
| 180180CD0 | 880 | pimax::totem::LoopClosureOptions::LoopClosureOptions(const&) |  | project (implicit copy ctor) | modified (extra fields) | 656-byte options struct |
| 180181040 | 44 | std::pair<shared_ptr<T>,int>::pair(const shared_ptr&, int) |  | lib:STL | - |  |
| 180181070 | 20 | cv::_InputArray::_InputArray(const Mat&) |  | lib:OpenCV inline | - | flags 0x01010000 |
| 180181090 | 20 | cv::_OutputArray::_OutputArray(Mat&) |  | lib:OpenCV inline | - | flags 0x02010000 |
| 1801810B0 | 97 | std::_Hash<unordered_map<int,map<..>>>::~_Hash |  | lib:STL | - |  |
| 180181120 | 115 | std::_Hash<unordered_map<string,X>>::~_Hash |  | lib:STL | - | LoopClosing+2672, kStrTo* atexit |
| 1801811A0 | 60 | list node holder dtor <pair<const int,map>> |  | lib:STL | - |  |
| 1801811E0 | 134 | list node holder dtor <pair<const string,X>> |  | lib:STL | - |  |
| 180181270 | 13 | ~vector<pair<int,map<..>>> (null-checked) |  | lib:STL | - |  |
| 180181280 | 92 | std::map<..>::~map |  | lib:STL | - | caller 0x1801899c0 |
| 1801812E0 | 72 | std::_Destroy_range<pair<int,map<..>>> |  | lib:STL | - |  |
| 180181330 | 28 | boost::archive::binary_iarchive::~binary_iarchive (base part) |  | lib:boost | - | EH unwind |
| 180181350 | 28 | boost::archive::binary_oarchive::~binary_oarchive (base part) |  | lib:boost | - | EH unwind |
| 180181370 | 5 | portable_binary_iarchive dtor thunk |  | lib:boost example | - |  |
| 180181380 | 196 | std::deque<POD>::_Tidy |  | lib:STL | - | LoopClosing+1648 |
| 180181450 | 244 | std::deque<shared_ptr<T>>::_Tidy |  | lib:STL | - | LoopClosing+1688 |
| 180181550 | 196 | std::deque<POD>::_Tidy |  | lib:STL | - | LoopClosing+232 |
| 180181620 | 244 | std::deque<vector<shared_ptr<T>>>::_Tidy |  | lib:STL | - | LoopClosing+1280 |
| 180181720 | 112 | std::list<pair<const int,map<..>>>::_Tidy |  | lib:STL | - |  |
| 180181790 | 35 | std::list<pair<const string,X>>::_Tidy |  | lib:STL | - |  |
| 1801817C0 | 39 | std::map<int,int,less,aligned_allocator>::~map |  | lib:STL | - |  |
| 1801817F0 | 42 | std::map<int,vector<vector<shared_ptr<KeyFrame>>>>::~map |  | lib:STL | - | PlatMap::kf_map_ |
| 180181820 | 5 | thunk -> 0x1801810b0 |  | lib:STL | - |  |
| 180181830 | 5 | thunk -> 0x180181120 |  | lib:STL | - |  |
| 180181840 | 5 | thunk -> 0x180185a50 |  | lib:STL | - |  |
| 180181850 | 123 | std::vector<96-byte POD>::_Tidy |  | lib:STL | - | LoopClosing+1744 |
| 1801818D0 | 254 | std::vector<vector<16-byte POD>>::_Tidy |  | lib:STL | - | ReLocalize |
| 1801819D0 | 256 | LcStruct112::~LcStruct112 (implicit) |  | project (implicit dtor) | - | LoopClosing+1328/+1448 |
| 180181AD0 | 15 | cv::BFMatcher::~BFMatcher |  | lib:OpenCV inline | - |  |
| 180181AE0 | 1833 | pimax::totem::LoopClosing::~LoopClosing | () | project | modified: calls stopAllThread() | + implicit member dtors |
| 180182210 | 519 | cv::Mat_<double>::operator=(const Mat&) |  | lib:OpenCV inline | - | assert DataType<_Tp>::channels == m.channels() \|\| m.empty() |
| 180182420 | 36 | cv::MatExpr::operator cv::Mat() |  | lib:OpenCV inline | - | op->assign(*this, m, -1) |
| 180182450 | 92 | T112::operator=(const T112&) (implicit) |  | lib:implicit copy-assign | - | ReLocalize correction struct |
| 1801824B0 | 285 | std::map<int,MapIndex>::operator[] |  | lib:STL | - | default {-1,0,-1.0,0,0} |
| 1801825D0 | 251 | std::map<int,vector<vector<shared_ptr<KeyFrame>>>>::operator[] |  | lib:STL | - |  |
| 1801826D0 | 12 | std::vector<12-byte>::operator[] |  | lib:STL | - |  |
| 1801826E0 | 12 | std::vector<24-byte>::operator[] |  | lib:STL | - |  |
| 1801826F0 | 383 | ReLocalize sort comparator lambda (in 0x18018b600) | bool (const pair<int,map<float,X>>& a, const ...& b) | project lambda (body belongs to ReLocalize chunk) | new | mean of map keys, descending; empty maps compare by size |
| 180182870 | 43 | std::_Ref_count_obj2<KeyFrame> scalar deleting dtor |  | lib:STL | - |  |
| 1801828A0 | 43 | std::_Ref_count_obj2<PlatMap> scalar deleting dtor |  | lib:STL | - |  |
| 1801828D0 | 62 | cv::BFMatcher scalar deleting dtor |  | lib:OpenCV inline | - |  |
| 180182910 | 79 | LoopClosing scalar deleting dtor |  | project (compiler-generated, vtable slot 0) | - | free() (aligned operator delete) |
| 180182960 | 324 | pimax::totem::PlatMap::~PlatMap (scalar deleting form) |  | project (implicit dtor) | new |  |
| 180182AB0 | 61 | boost::archive::binary_iarchive scalar deleting dtor |  | lib:boost | - |  |
| 180182AF0 | 61 | boost::archive::binary_oarchive scalar deleting dtor |  | lib:boost | - |  |
| 180182B30 | 61 | portable_binary_iarchive scalar deleting dtor |  | lib:boost example | - |  |
| 180182B70 | 457 | pimax::totem::PlatMap::UpdateMap | void (const vector<vector<KeyFramePtr>>& kf_list, int map_id) | project | new | merge session into kf_map_, then Map2Vec |
| 180182D40 | 571 | pimax::totem::PlatMap::BuildMapIndex | void () | project | new |  |
| 180182F80 | 921 | pimax::totem::PlatMap::load | void (const std::string& filename) | project | new | binary_iarchive; rethrows std::exception |
| 180183320 | 1915 | pimax::totem::PlatMap::loadIndex | bool (const std::string& filename) | project | new | YAML index, version "1.0.0", <=3 maps |
| 180183AA0 | 1055 | pimax::totem::PlatMap::load_bin | void (const std::string& filename) | project | new | portable_binary_iarchive ("PBA") |
| 180183EC0 | 1737 | pimax::totem::PlatMap::save | void (const std::string& filename) | project | new | binary_oarchive to <file>-bk, rename |
| 180184590 | 2462 | pimax::totem::PlatMap::saveIndex | void (const std::string& filename) | project | new | YAML index |
| 180184F30 | 474 | pimax::totem::PlatMap::Map2Vec | void (int map_id /*unused*/) | project | new | trim to max_kf_num_, flatten, BuildMapIndex |
| 180185110 | 889 | pimax::totem::PlatMap::Vec2Map | void () | project | new |  |
| 180185490 | 142 | std::vector<32-byte POD>::_Change_array |  | lib:STL | - |  |
| 180185520 | 9 | std::_Ref_count_obj2<KeyFrame>::_Destroy |  | lib:STL | - | -> ~KeyFrame 0x1800e4ee0 |
| 180185530 | 11 | std::_Ref_count_obj2<PlatMap>::_Destroy |  | lib:STL | - | -> ~PlatMap 0x180182960 |
| 180185540 | 627 | std::_Hash<unordered_map<string,X>>::_Forced_rehash |  | lib:STL | - |  |
| 1801857C0 | 469 | std::deque<T>::_Growmap |  | lib:STL | - |  |
| 1801859A0 | 175 | std::_Hash<unordered_map<string,X>>::_Check_rehash |  | lib:STL | - |  |
| 180185A50 | 199 | ~vector<pair<int,map<..>>> (COMDAT shared with Ceres) |  | lib:STL | - |  |
| 180185B20 | 63 | std::copy<8-byte POD> |  | lib:STL | - |  |
| 180185B60 | 105 | std::_Uninitialized_copy<24-byte POD> |  | lib:STL | - |  |
| 180185BD0 | 68 | std::_Uninitialized_move<vector<shared_ptr<KeyFrame>>> |  | lib:STL | - |  |
| 180185C20 | 541 | std::unordered_map<string,X>::erase(first,last) |  | lib:STL | - | caller 0x180187190 |

### Notes on individual project functions

- **extractBoWFeaturesFromImage 0x180174b70** (upstream: per-call `ORB::create()`, all keypoints):
  `feature->clear()` (keypoints_pt2f is *not* cleared); function-local
  `static cv::Ptr<cv::ORB> orb = ORB::create(500, 1.2f, 4, 31, 0, 2, HARRIS_SCORE, 31, 20)`
  (guard dword_18047EE48, object qword_18047EE38, atexit 0x1803a6150); `detectAndCompute(image,
  noArray(), kps, desc)`; `changeStructure(desc, &rows)`; keypoints indexed in a
  `std::multimap<double,int,std::greater<double>>` (response → index, node 0x30); then
  `OccupandyGrid2D grid(2, ceil(cols*0.5), ceil(rows*0.5))` (svo direct type, ctor 0x1800aa2f0,
  dtor 0x1800aa440); in descending response order a keypoint is kept iff its 2x2 cell
  (`(int)x/2 + n_cols*((int)y/2)`) is free: push descriptor row, push `kp.pt`, mark cell.
- **extractFeaturesFromSVOKeypoints 0x180175160**: 7 args (bearing vectors, depths, feature types,
  original indices removed). Returns immediately if any output pointer is null. Second static ORB
  (dword_18047EE60 / qword_18047EE50, same parameters). Builds one `cv::KeyPoint(pt, 1, -1, 0, 0,
  class_id)` per input point, `class_id` = lowest index of an identical point (|dx|,|dy| < 1e-6f)
  found through an `unordered_map<pair<int64,int64>, vector<size_t>, PairHash>` keyed by
  `floor(x / (double)1e-6f)` (3x3 neighbourhood); non-finite points keep their own index;
  index > INT_MAX → −1. Then clears kps/landmarks/ids/trackIDs, and only if
  `!keypoints.empty() && !image.empty()`: `orb->compute`, `changeStructure(*desc, features)`,
  and re-associates landmark / id / trackID for every surviving keypoint via class_id (fast path)
  or linear `find_if` (|dx|,|dy| < 1e-6f), pushing only if the index is valid in all three old
  vectors (the keypoint itself is pushed unconditionally — so `svo_keypoints` can be longer than
  the landmark vectors).
- **commonLandMarkCheck 0x180178e40**: returns false for empty `track_IDs1`, otherwise
  `common/size < th` (no isnan clause).
- **Utility::ypr2R 0x18017ee10**: `if(!is_radian) a = a/180.0*M_PI` for y,p,r; `Rz(y)*Ry(p)*Rx(r)`.
- **LoopClosing::LoopClosing 0x18017f730** (see `loop_closing_ctor.cpp`): member init order
  (decompile): +16=0, +20=true, plat_map_ null, two mutexes (+40,+120), vector +200, +224=false,
  deque +232 (proxy alloc), +272=10, +276=false, T_C_B_/T_B_C_ identity, mask_ Mat, aruco Ptrs,
  `cams_` copy, options copy (0x180180cd0), +1216/+1224, …, `orb_ = ORB::create(500,1.2f,4,31,0,2,
  HARRIS,31,20)`, `beblid_ = make_shared<BEBLID>(256, 0.75f)` (0x180170570), voc_ default
  (k=10, L=5, TF_IDF, L1_NORM; createScoringObject), K/D vectors, mutex +2920, …
  Body: mode switch; `voc_ = OrbVocabulary(voc_path + voc_name)` (TemplatedVocabulary(string)
  0x180177110 then the upstream operator= inlined: m_k, m_L, m_scoring, m_weighting,
  createScoringObject, m_nodes/m_words clear, m_nodes = voc.m_nodes, createWords; temp
  destroyed 0x180172880); per camera K (Mat_<double> 3x3 from intrinsics 0,2,1,3) and distortion
  VectorXd; `mask_ = cam0->getMask().clone()`; `DetectorParameters::create()`,
  `getPredefinedDictionary(3 = DICT_4X4_1000)`; `kStrToScaleRetMap[scale_ret_app]`,
  `kStrToGlobalMapType[global_map_type]`; `system("exec rm -r " + image_log_base_path + "*")` only
  if `enable_image_logging`; `startAllThread()` (0x180195b70); if `options_.use_plat_map` (+393):
  `make_shared<PlatMap>()`, `max_kf_num_ = (int)options_.max_kf_num`, copy K/D lists, stale-backup
  check (`<map_path><tag><map_name>.bk`), mode log (LOGW), `if (loadIndex()) load();`.
- **LoopClosing::~LoopClosing 0x180181ae0**: `stopAllThread()` (0x180195e60) then implicit member
  destruction (the std::thread at +3056 → `terminate()` if still joinable; the three
  `unique_ptr<std::thread>` likewise).

## Types

### DBoW2::TemplatedVocabulary<cv::Mat, FORB> (80 bytes, upstream layout)
| off | type | name |
|---|---|---|
| 0 | vptr | 0x1803bdb18, 18 slots: 0 dtor, 1 create(tr,k,L,w,s), 2 create(tr,k,L), 3 create(tr), 4 size, 5 empty, 6 transform(f,WordId&), 7 transform(f,id,weight,nid*,levelsup), 8 transform(f)->WordId, 9 transform(fs,bow,fv,levelsup), 10 transform(fs,bow), 11 getParentNode, 12 getWord, 13 getWordWeight, 14 save(fs,name), 15 load(fs,name), 16 stopWords, 17 initiateClusters (MSVC groups overloads in reverse declaration order) |
| 8 | int | m_k |
| 12 | int | m_L |
| 16 | WeightingType | m_weighting |
| 20 | ScoringType | m_scoring |
| 24 | GeneralScoring* | m_scoring_object |
| 32 | vector<Node> | m_nodes (Node = 152 bytes: id +0, weight +8, children +16, parent +40, descriptor Mat +48, word_id +144) |
| 56 | vector<Node*> | m_words |

Scoring objects are 8-byte (vptr only); vtables GeneralScoring 0x1803bda38, L1 0x1803bda58,
L2 0x1803bda78, ChiSquare 0x1803bda98, KL 0x1803bdab8, Bhattacharyya 0x1803bdad8,
DotProduct 0x1803bdaf8; slot 2 (deleting dtor) is 0x180172d70 for all; slot 1 mustNormalize:
L1/ChiSquare/KL/Bhattacharyya 0x180177160, L2 0x180177180, DotProduct 0x180177170; slot 0
`score` lives in the DBoW2 object (outside).

### KeyFrame — sizeof 0x300 (768), `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`
make_shared allocation 0x310 (0x180185e40); boost heap allocation malloc(0x300) (0x1801102f0);
default ctor 0x1800e26d0; dtor 0x1800e4ee0 (both outside). `_Ref_count_obj2<KeyFrame>` slots
0x180185520 (_Destroy) / 0x180182870. All offsets sure (ctor + serializer).

| off | type | name (guess where no upstream) | serialized | evidence |
|---|---|---|---|---|
| 0 | int | map_id_ | #1 | ctor arg 4 = LoopClosing::map_id_; key of kf_map_/map_index_ |
| 4 | int | NframeID_ | #2 | renumbered after load; ctor arg 1 |
| 8 | int | frame_id_ | #3 | ctor arg 3 = Frame::id_ (+16) |
| 12 | int | cam_id_ | #4 | ctor arg 2 = frame +20 |
| 16 | size_t | lc_frame_count_ (?) | – | untouched by ctors |
| 24 | double | timestamp_sec_abs_ | #5 | → MapIndex::newest_timestamp_ |
| 32 | cv::Mat | keyframe_image_ | – | |
| 128 | Transformation | T_w_c_ | #6 | identity init (w at +152) |
| 192 | Transformation | ? | – | identity init (w at +216) |
| 256 | cv::Mat | ? | – | |
| 352 | vector<Point2f> | bow_keypoints_ | #7 | recovery_kf → mixed_keypoints_ |
| 376 | vector<Mat> | bow_features_ | #8 | recovery_kf → mixed_features_ |
| 400 | vector<int> | bow_node_ids_ | #9 | recovery_kf → mixed_node_ids_ |
| 424 | DBoW2::BowVector | vec_bow_ | #10 | ctor 0x1801b5df0 |
| 440 | cv::Mat | svo_features_mat_ | – | |
| 536 | vector<Mat> | svo_features_ | #11 | |
| 560 | vector<int> | svo_node_ids_ | #13 | written after +584 |
| 584 | vector<Point2f> | svo_keypointsvector_ | #12 | |
| 608 | vector<Point3f> | svo_landmarksvector_cam_ | #14 | |
| 632 | vector<?> | ? (svo_landmark_ids_?) | – | 24 bytes zeroed |
| 656 | vector<?> | ? (svo_trackIDsvector_?) | – | 24 bytes zeroed |
| 680 | vector<Point2f> | mixed_keypoints_ | – | recovery_kf |
| 704 | vector<int> | mixed_node_ids_ | – | recovery_kf |
| 728 | vector<Mat> | mixed_features_ | – | recovery_kf |
| 752 | size_t | num_bow_features_ | #15 | 8-byte integer in archive; recovery_kf compares `+752 + svo_features_.size()` with `mixed_features_.size()` |

Upstream differences: map_id_/cam_id_ new, svo bearing/depth/featuretype/index vectors removed,
second Transformation and Mat added, `num_bow_features_` is size_t (upstream int), `skip_frame_` gone.

### MapIndex (value of `std::map<int, MapIndex>`, node 0x40, value at node+40)
| off | type | name | default |
|---|---|---|---|
| 0 | int | map_id_ | −1 |
| 4 | int | kf_nums_ | 0 |
| 8 | double | newest_timestamp_ | −1.0 |
| 16 | int | startIdx_ | 0 |
| 20 | int | endIdx_ | 0 |

### PlatMap — sizeof 0xE0 (224) (make_shared alloc 0xF0), no vtable
`_Ref_count_obj2<PlatMap>` vtable 0x1803bef50 (slots 0x180185530 _Destroy, 0x1801828a0 _Delete_this).

| off | type | name | evidence |
|---|---|---|---|
| 0 | std::mutex | mtx_ | Mtx_init_in_situ; UpdateMap lock |
| 80 | int | max_kf_num_ = 1000 | ctor; Map2Vec; LoopClosing sets it from options+296 |
| 88 | map<int, vector<vector<KeyFramePtr>>> | kf_map_ | Vec2Map/Map2Vec/UpdateMap |
| 104 | vector<vector<KeyFramePtr>> | kf_list_ | the only serialized member |
| 128 | map<int, MapIndex> | map_index_ | BuildMapIndex/loadIndex/saveIndex |
| 144 | vector<cv::Mat> | K_list_ | copied from LoopClosing+2864 |
| 168 | vector<Eigen::VectorXd> | D_list_ | copied from LoopClosing+2888 |
| 192 | std::string | map_version_ | loadIndex |

### LoopClosureOptions (656 bytes) and LoopClosing (sizeof 0xC10 = 3088)
See `draft/c15_platmap/loop_closing/loop_closing_c15.h` for the full offset tables (every offset
the ctor/dtor touch). Highlights (sure): options +296 size_t `max_kf_num` (×5 in lab modes),
+393 bool `use_plat_map`, +512 `map_path`, +560 `map_name`, +592 `map_index_name`;
LoopClosing +16 map_id_, +20 enable_save_map_, +24 plat_map_, +416 mask_, +512/+528 aruco Ptrs,
+544 cams_, +560 options_, +1216 mode_, +1224 map_tag_, +1728 cur_kf_to_lc_kf_bundle_id_map_
(aligned_allocator, upstream), +1768 mutex + +1848 kf_list_loop_ (vector<vector<KeyFramePtr>>,
handed to `PlatMap::UpdateMap` by 0x18018b3d0 when ≥200 groups), +2152/+2328/+2504 three
`unique_ptr<std::thread>` each with condition_variable/stop-flag/mutex, +2736 ORB,
+2752 shared_ptr<BEBLID>, +2784 OrbVocabulary voc_, +2864 K_list_, +2888 D_list_,
+2912 scale_retrieval_approach_, +3048 global_map_type_, +3056 std::thread, +3072/+3073/+3074
flags. vtable 0x1803be300 (slot 0 = 0x180182910). Upstream `options_` layout +0..+255 unchanged.

`LoopClosingMode` (uint8 ctor arg): 0 kNormal, 1 kLabMap, 2 kLabLoc.

## On-disk formats

### Files (all under `options.map_path + map_tag_`)
| file | writer | reader |
|---|---|---|
| `<map_name>` = `platMap_orborb_K8L4.bin` | PlatMap::save (native boost binary_oarchive) via LoopClosing::save 0x180193e00 | PlatMap::load via LoopClosing::load 0x180188900 |
| `<map_name>.pba` | nothing in the binary (a dead-stripped portable save existed: its oserializer singleton is still registered) | PlatMap::load_bin — **only** as fallback when PlatMap::load throws a std::exception |
| `<map_name>-bk` | temporary of PlatMap::save, renamed onto `<map_name>` with boost::filesystem::rename; removed on error | – |
| `<map_name>.bk` | nothing | LoopClosing ctor: if it exists (`ifstream.good()`), it is removed together with `<map_name>` and `<map_index_name>` (log "…bin.bk is not existed") — note `.bk` ≠ `-bk` |
| `<map_index_name>` = `platMap_orborb_K8L4.yaml` | PlatMap::saveIndex (cv::FileStorage YAML) via 0x180194020 | PlatMap::loadIndex via 0x180188ce0 |

`map_tag_` is the ctor string (kNormal) or `"Lab"` (kLabMap/kLabLoc).

NOTE: `../slam/pimax_database.bin` (the sample given for verification) is **not** a PlatMap: it is
the LedObjectPoseEstimator controller LED vote map (cereal BinaryInputArchive, see
`../LedObjectPoseEstimator/notes/camera_retrieve_3dof.md`, DataBase::LoadMap); it does not start
with a boost archive signature (first u64 = 184547). `tools/parse_platmap.py` reports this and
exits 2. The parser was instead validated on synthetic native and portable archives produced by
`draft/c15_platmap/test/gen_platmap_sample.cpp` (same serialize bodies, boost 1.92 on Linux) —
both parse completely, zero trailing bytes, expected values.

### PlatMap archive, native (`boost::archive::binary_oarchive`, boost 1.74, MSVC x64, little endian)
Primitive encoding: everything raw little-endian, `int` 4, `size_t` 8, `bool` 1, float 4, double 8.
boost bookkeeping types: tracking_type = u8 bool, version_type = u32, class_id_type = i16,
object_id_type = u32, collection_size_type = u64, item_version_type = u32,
library_version_type = u16, std::string = u64 length + bytes.

```
u64 22, "serialization::archive"              basic_binary_oarchive::init
u16 18                                        library version (boost 1.74)
u8 4, u8 4, u8 4, u8 8                        sizeof int, long (MSVC: 4), float, double
i32 1                                         endianness probe
u64 7, "PlatMap"                              tag string (PlatMap::save: oa << std::string("PlatMap"))
-- PlatMap (by value, class id 0) ------------------------------------------
u8 0, u32 0                                   class info: tracking=0, version=0
  -- std::vector<std::vector<KeyFrame*>> (temporary built from kf_list_, class id 1)
  u8 0, u32 0                                 class info
  u64 n_groups, u32 item_version(0)
  repeat n_groups:                            -- std::vector<KeyFrame*> (class id 2)
    [first group only: u8 0, u32 0]           class info
    u64 n_cams, u32 item_version(0)
    repeat n_cams:                            -- KeyFrame* (pointer, non-polymorphic)
      null:            i16 -1
      first KeyFrame:  i16 3 (new class id), u8 1 (tracking), u32 0 (version), u32 oid, <KeyFrame>
      later KeyFrames: i16 3 (class_id_reference), u32 oid, <KeyFrame>
                       (oid = 0,1,2,… in order; an oid below the number already read would be a
                        back-reference without data — does not occur, every KeyFrame* is unique)
<KeyFrame> (KeyFrame::serialize):
  i32 map_id_, i32 NframeID_, i32 frame_id_, i32 cam_id_
  f64 timestamp_sec_abs_
  QuatTransformation   [first: u8 0,u32 0]  f64 qw, f64 qx, f64 qy, f64 qz,
     Vector3d position [first: u8 0,u32 0]  i64 rows(=3), i64 cols(=1), f64 x, f64 y, f64 z
  vector<Point2f> bow_keypoints_   [first: u8 0,u32 0] u64 n, u32 0, n x ([first Point2f: u8 0,u32 0] f32 x, f32 y)
  vector<Mat> bow_features_        [first: u8 0,u32 0] u64 n, u32 0, n x <Mat>
  vector<int> bow_node_ids_        (no class info; array-optimised) u64 n, n x i32
  BowVector vec_bow_               [first: u8 0,u32 0]
     map<uint,double>              [first: u8 0,u32 0] u64 n, u32 0, n x ([first pair: u8 0,u32 0] u32 word_id, f64 weight)
  vector<Mat> svo_features_        u64 n, u32 0, n x <Mat>
  vector<Point2f> svo_keypointsvector_     u64 n, u32 0, n x (f32 x, f32 y)
  vector<int> svo_node_ids_                u64 n, n x i32
  vector<Point3f> svo_landmarksvector_cam_ [first: u8 0,u32 0] u64 n, u32 0, n x ([first Point3f: u8 0,u32 0] f32 x,y,z)
  u64 num_bow_features_
<Mat> = [first Mat: u8 0,u32 0] i32 cols, i32 rows, i32 type, u8 continuous,
        rows*cols*elemSize bytes (row by row if !continuous)        (ORB descriptors: 1x32 CV_8U)
```
"[first: …]" = boost class preamble, written only the first time an object of that C++ type is
serialized in the archive (per type, not per field). Class ids (registration order, typical):
0 PlatMap, 1 vector<vector<KeyFrame*>>, 2 vector<KeyFrame*>, 3 KeyFrame, then members as first
met — only the KeyFrame id (3) appears in the stream. All versions are 0 (no BOOST_CLASS_VERSION
anywhere; `version()` returns 0, 0x180129860). Tracking is 1 only for KeyFrame (a pointer
serializer exists for it); `class_info()` is false only for `std::vector<int>` (0x18009cdf0).

### PlatMap archive, portable (`portable_binary_oarchive`, file `*.pba`)
Same object stream; differences:
- every integer (int, size_t, Eigen::Index, bool, tracking, version, class id, object id,
  collection size, item version, string length, library version) is a **portable int**:
  `i8 n` (0 ⇒ value 0, no more bytes; |n| = number of following little-endian magnitude bytes;
  n < 0 ⇒ negative value), e.g. 0 → `00`, 1 → `01 01`, 22 → `01 16`, −1 → `ff 01`,
  300 → `02 2c 01`;
- float/double/char/unsigned char raw (Mat bytes and Eigen/quaternion doubles raw);
- `vector<int>` is NOT array-optimised: count, item_version, then n portable ints;
- header: `01 16 "serialization::archive"`, library version `01 12`, then one raw byte
  `m_flags >> 8` (0 ⇒ little endian; 0x40 would mean big endian); no sizeof bytes.

Identification of `src/common/portable_archive/portable_binary_[io]archive.hpp`: boost **1.74.0**
`libs/serialization/example` files, unmodified — `load_impl` 0x18009d3b0, `save_impl` 0x18009da50,
`init` 0x18009cfc0 (signature check, library version read with max size 2 + the 1.74
`library_version_type.hpp` assert "t_ <= boost::integer_traits<base_type>::const_max", flags byte),
`load(std::string)` 0x18009d2d0, exception class `portable_binary_iarchive_exception`
(vtable 0x1803b1b50), `m_flags` at +104, `endian_big` = 0x4000.

### YAML index (cv::FileStorage, `WRITE|FORMAT_YAML` = 17 / `READ|FORMAT_YAML` = 16)
```
%YAML:1.0
---
map_version_: "1.0.0"
map_index_:
   -
      map_id_: <int = map key>
      kf_nums_: <int>
      newest_timestamp_: <double>
      startIdx_: <int>          (index of the first group of this map in kf_list_)
      endIdx_: <int>            (one past the last group)
   - ...
```
loadIndex fails (returns false, nothing loaded) if `map_index_` is missing/empty, `map_version_`
is missing ("Wont load the map because of none map version.") or ≠ "1.0.0" ("…illegal map
version."), or if there are more than 3 entries. saveIndex writes `map_id_` from the map **key**.

### `.dbow` vocabulary (DBoW3 binary, read by fromStream 0x180175d80)
`u64 0x14B1863F81`, `u8 compressed`, `u32 nnodes`; if compressed: `u32 nChunks`, then per chunk a
quicklz level-1 block (9-byte header, `qlz_size_compressed`, ≤10000 bytes decompressed); the
(decompressed) payload is `i32 k, i32 L, i32 scoring, i32 weighting`, then for nodes 1..nnodes−1:
`u32 nid, u32 parent, f64 weight, descriptor(i32 cols, i32 rows, i32 type, cols*elemSize bytes)`,
then `u32 nwords` and `nwords x (u32 word_id, u32 node_id)`. Note: m_scoring is read **before**
m_weighting (DBoW3 order).

## Algorithms (PlatMap)

- **UpdateMap(kf_list, map_id)** (lock): for every group with `kfs[0] != null` (kfs[0] read without
  empty check) and `0 <= kfs[0]->map_id_ <= map_id`: new map → `kf_map_[id] = {kfs}`, else append
  unless an equal group (vector<shared_ptr> ==) exists. Then `Map2Vec(map_id)`.
- **Map2Vec**: for each map drop the oldest `size - max_kf_num_` groups; rebuild kf_list_ as the
  concatenation of kf_map_ in ascending map id; BuildMapIndex.
- **Vec2Map**: kf_map_.clear(); groups with size ≥ 1 and no null pointer are appended to
  `kf_map_[kfs[0]->map_id_]`.
- **BuildMapIndex**: map_index_.clear(); per group i (non-empty, no null): first time a map id is
  seen → {id, 1, ts, i, i+1}; else kf_nums_++, newest_timestamp_ = ts, endIdx_++ (assumes the
  groups of one map are contiguous).
- **load / load_bin**: clear kf_list_ and map_index_; archive → kf_list_; `recovery_kf`
  (0x18018c070, rebuilds mixed_* vectors), Vec2Map, BuildMapIndex; then NframeID_ of every
  KeyFrame is shifted so that the minimum becomes 1, and the global `Frame::frame_counter_`
  (dword_18047DB38, used by the Frame ctor 0x180091da0) is advanced by
  `(max−min+1) * kf_list_[0].size()`.

## External interfaces (called from this chunk, outside it)

| address | meaning |
|---|---|
| 0x18000C2C0 / 0x18000F500 / 0x18000F6A0 | LOGE / LOGI / LOGW (logger 0x18046A000) |
| 0x180011570 | std::string::assign(const char*, size_t) |
| 0x1800904b0, 0x1800fdd60 | std::operator+(string, string) / string&& append |
| 0x180017630, 0x180016530, 0x180022d20, 0x18000be20 | stringstream <<, str(), ctor, dtor |
| 0x1800deb50, 0x180097c60, 0x180097ec0, 0x180118600 | ifstream/ofstream ctor, filebuf open/close |
| 0x1800e88d0 | ~ifstream |
| 0x18009c8e0 / 0x18009cb40 | basic_binary_iprimitive ctor / dtor (both archive kinds) |
| 0x18035eb60, 0x18035edb0, 0x18035eef0, 0x18035fa60, 0x18035fc10, 0x18035ff00 | boost basic_iarchive ctor/dtor/load_object, basic_binary_iarchive::init, iprimitive::init, load(std::string) |
| 0x18035cf80, 0x18035d280, 0x18035d4a0, 0x1803602e0, 0x180360440, 0x180360640 | boost basic_oarchive ctor/dtor/save_object, oarchive init, oprimitive init, save(std::string) |
| 0x1803622b0, 0x180360a70, 0x180365280, 0x180365320, 0x1803650a0 | boost::filesystem path conversion, rename, status (exists), remove |
| 0x18009cfc0, 0x18009d2d0, 0x18009d3b0, 0x18009da50 | portable_binary_iarchive::init, load(std::string), load_impl, portable_binary_oarchive::save_impl |
| 0x1800b9520 / 0x1800b99e0 | load / save of `vector<vector<shared_ptr<KeyFrame>>>` (PlatMap::serialize body, portable) |
| 0x1800dc3c0 / 0x1800dcd10 | KeyFrame::serialize load/save (portable); native: 0x1800db460 / 0x1800dbd40 |
| 0x1800dc670 / 0x1800dcfe0 | cv::Mat load/save (portable) |
| 0x1800dc190 / 0x1800dcae0 | BowVector load/save (portable); map<uint,double> load 0x1800d1630 |
| 0x180117040 / 0x1801263b0, 0x180117280 / 0x1800dc8a0 | Vector3d, QuatTransformation load/save (portable) |
| 0x180116xxx–0x180127xxx | all i/oserializer::load/save_object_data (vtables 0x1803b40f0–0x1803b6c08) |
| 0x1800089c0 | kindr RotationQuaternion(const Eigen::Quaterniond&) with norm CHECKs |
| 0x18018c070 | recovery_kf(PlatMap*) (next chunk) |
| 0x180188ce0, 0x180188900, 0x180193e00, 0x180194020 | LoopClosing::loadIndex / load / save / saveIndex wrappers |
| 0x180195b70 / 0x180195e60 | LoopClosing::startAllThread / stopAllThread |
| 0x180170570 | std::make_shared<BEBLID>(int 256, float 0.75f) |
| 0x1801b41a0 | NCamera::getCameraShared(size_t) |
| 0x18015a980 | MatCommaInitializer_<double>::operator, (advance) |
| 0x1801635f0 | vector<cv::Mat>::_Assign_range |
| 0x18015a2d0 / 0x180092cf0 / 0x18015a4f0 | ~vector<VectorXd>, ~vector<Mat>, ~LoopClosureOptions |
| 0x1800aa2f0 / 0x1800aa440 | OccupandyGrid2D ctor / dtor |
| 0x1801b70f0 | DBoW2::FORB::distance |
| 0x1801b71e0 / 0x1801b75c0 | FORB::fromString / toString |
| 0x1801b6d90 | descriptor fromStream (cols, rows, type, data) |
| 0x1801b6e30 / 0x1801b70d0 | qlz_decompress / qlz_size_compressed |
| 0x1801b5e30 / 0x1801b5f20 / 0x1801b6020 / 0x1801b61e0 | BowVector::addWeight / addIfNotExist / normalize, FeatureVector::addFeature |
| 0x1801b5df0 | BowVector ctor |
| 0x180170e50 | PairHash for pair<int64,int64> |
| 0x1800e26d0 / 0x1800e4ee0 | KeyFrame default ctor / dtor |
| 0x180092420 | Frame::Frame (via 0x18017ec10) |

## Globals

| address | meaning |
|---|---|
| 0x18046A000 | logger instance |
| dword_18047DB38 | `Frame::frame_counter_` (Frame ctor 0x180091da0: `id_ = frame_counter_++`) |
| qword_18047EE38 / dword_18047EE48 | static ORB in extractBoWFeaturesFromImage + init guard |
| qword_18047EE50 / dword_18047EE60 | static ORB in extractFeaturesFromSVOKeypoints + guard |
| Buf2 / Size_0 / n0x10 (.data) | `kPlatMapVersion` std::string "1.0.0" (dyn-init 0x180002bb0, chars at 0x1803bdd9c) |
| n1065353216_4 (+qword_18047EE98..) | `kStrToGlobalMapType` unordered_map (init 0x180002bf0, atexit 0x1803a6260) |
| n1065353216_5 (+qword_18047EEF8..) | `kStrToScaleRetMap` unordered_map (init 0x180002df0) |

## Constants

- ORB (3 instances: LoopClosing::orb_ and two function statics): `nfeatures 500, scaleFactor 1.2f
  (0x1803bdd58), nlevels 4, edgeThreshold 31, firstLevel 0, WTA_K 2, HARRIS_SCORE, patchSize 31,
  fastThreshold 20`.
- BEBLID: `make_shared<BEBLID>(256, 0.75f)` (0.75 at 0x1803befb8).
- aruco dictionary 3 (DICT_4X4_1000).
- Vocabulary member default: k=10, L=5, TF_IDF, L1_NORM.
- extractFeaturesFromSVOKeypoints: grid cell `(double)1e-6f` = 9.999999974752427e-07
  (0x1803bdd48); equality tolerance `1e-6f` (0x1803b2684); reserve rounding 0.5f (0x1803b12e4).
- extractBoWFeaturesFromImage: occupancy cell 2 px, dims `ceil(cols*0.5)`, `ceil(rows*0.5)`
  (0.5 at 0x1803af2b8).
- ypr2R: 180.0 (0x1803b6d38), π (0x1803b1390).
- PlatMap: `max_kf_num_` default 1000; loadIndex accepts ≤ 3 maps; version "1.0.0";
  LoadSavePlatMap thread hands over at ≥ 200 groups (0x18018b3d0, next chunk).
- LoopClosing: +272 = 10, +1320 = 3 (meaning unknown).
- "Lab" map tag string at 0x1803be308.

## Quirks / bugs to preserve

1. LoopClosing ctor starts the worker threads (`startAllThread`) **before** `plat_map_` is created
   and before the map is loaded.
2. Stale-backup check looks for `<map>.bk`, but `PlatMap::save` uses `<map>-bk`; the check thus
   never matches a leftover temp file. When `.bk` exists the log says "…bin.bk is not existed"
   (LOGE) and map + index are deleted.
3. `PlatMap::load` rethrows `std::exception` after clearing; `load_bin` swallows it. Both swallow
   `...`. After a swallowed error `load` still runs the renumbering code and reads
   `kf_list_[0]` of an empty vector (UB; the binary dereferences begin()).
4. `UpdateMap` reads `kfs[0]` without an emptiness check.
5. `load` logs with `%d` for a size_t (`kf_list_ is empty! kf_list_ size %d`), load_bin with `%zu`.
6. `extractBoWFeaturesFromImage` does not clear `keypoints_pt2f` (appends); `changeStructure`
   appends (no clear/resize).
7. `extractFeaturesFromSVOKeypoints` pushes every surviving keypoint even when no landmark could be
   re-associated, so `svo_keypoints` and `svo_landmarks/ids/trackIDs` can diverge in length.
8. `commonLandMarkCheck` returns false for empty `track_IDs1` (upstream would return true via NaN).
9. `BowVector` load does not clear the target first.
10. `Map2Vec` takes `map_id` but ignores it; `saveIndex` writes the map key as `map_id_`.
11. `loadIndex` returns false (no load at all) if the index lists more than 3 maps.
12. The camera loop in the LoopClosing ctor iterates the `cams` **parameter**; `mask_` is taken
    from camera 0; T_C_B_/T_B_C_ are never set (identity).
13. `PlatMap::max_kf_num_` (int) is assigned from a size_t option (truncation).
14. The portable writer (`save_bin`) does not exist in the binary although its oserializer
    singleton is registered; a `.pba` file can only come from an older build or another tool.

## Open questions / TODO(verify)

- Names of the Pimax option fields (LoopClosureOptions +256..+655 except the few identified) and of
  most LoopClosing members; true element types of several LoopClosing containers (deques at
  +232/+1648/+1688, vectors +1256, +1744, +1872, +2032.., unordered_map +2672 value type).
- KeyFrame fields +8/+12/+16/+192/+256/+632/+656 names (and element types of +632/+656).
- File split (bow.cpp vs loop_closing.cpp vs platmap.h) and whether PlatMap methods were inline.
- Name of 0x180182b70 (`UpdateMap` is a guess) and the owner of `recovery_kf` (0x18018c070).
- The 352-byte object created in 0x18017ecb0 (ReLocalize) — type unknown.
- `save_bin` body (dead-stripped) — the draft is a guess, only for the singleton registration.
