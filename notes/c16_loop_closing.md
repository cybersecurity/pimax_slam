# c16_loop_closing — 0x180185E40 .. 0x180197E70

Drafts: `draft/c16_loop_closing/loop_closing/{loop_closing.h, loop_closing.cpp, map_alignment.h, map_alignment.cpp}`,
`draft/c16_loop_closing/plane/{histogram.h, histogram.cpp}`.

## Object boundaries inside the range

* **0x180185E40 – ~0x180197550 `loop_closing.obj`** (continuation; the ctor 0x18017F730, dtor 0x180181AE0 and all
  PlatMap/KeyFrame code 0x18017E8E0..0x180185C20 belong to chunk c15). Includes COMDATs first used here
  (std::to_string(float/long long), boost archive vtable thunks, _scprintf/sprintf inlines, ceres_map.h inlines).
* **0x1801975B0 `map_alignment.obj`**: only `MapAlignmentSE3::MapAlignmentSE3` survived /OPT:REF.
* **0x1801976B0 – … `histogram.obj`** (Kimera-VIO `Histogram`, used by plane/mesher): 0x1801976B0 (vector realloc
  COMDAT used by 0x180198460/0x180198960), Histogram ctor/default ctor/dtor/operator=; continues into c17
  (0x180197E70 ".yaml"/"histogram_").
* **BEBLID is NOT in this range** (vtables 0x1803B8A28/0x1803B8A70 point to 0x18016FBF4..0x180170654, i.e. before
  the loop_closing object; `make_shared<BEBLID>(256, 0.75f)` = 0x180170570). Nothing BEBLID-related reconstructed here.

## Function table

kind/status: `project` = reconstructed in the draft. "new" = pimax-new, "modified" = upstream-modified.

| address | proposed qualified name | signature | kind | upstream status | summary |
|---|---|---|---|---|---|
| 0x180185E40 | `LoopClosing::addFrameToPR` | `void (const FrameBundlePtr& last_frames, const Transformation& T_w_odom)` | project | modified: 2 cams, pose-cell key map, quality gates (num_features_>=opt+288, mean_intensity_ in [25,220]), condition-variable worker instead of detached threads, kf_list_ capped at max_kf_num, BoW moved out | creates 2 KeyFrames, appends to kf_list_, wakes LC worker; VLOG(40) "Last thread still running" (l.1351) |
| 0x180186A50 | `LoopClosing::reLocalize` | `void (const FrameBundlePtr&, const Transformation& T_w_odom)` | project | new | gate on use_plat_map/plat_map_ready_/reloc_busy_ CAS; queue KFs, notify reloc worker; 4 catch handlers (funclets 0x1803A10A0..0x1803A11B0) |
| 0x180186CB0 | `drawText` (file static) | `void (cv::Mat&, const std::string&, cv::Point, double, cv::Scalar, int)` | project | new | putText(FONT_HERSHEY_SIMPLEX, LINE_8, false) |
| 0x180186D40 | `std::string::assign(size_t n, char c)` | | lib:STL | - | used by _Floating_to_string |
| 0x180186EB0 | `std::vector<bool>::back()` (reference) | | lib:STL | - | |
| 0x180186F30 | `LoopClosing::computePoseDiff` | `void (double* dist, double* angle_deg, const Transformation&, const Transformation&)` | project | new | float translation distance, rotation angle acos((tr(R1ᵀR2)-1)/2) in deg; caller 0x1800FB650 |
| 0x180187190 | `std::unordered_map<std::string,bool>::clear` | | lib:STL | - | |
| 0x180187220 | `std::vector<std::vector<KeyFramePtr>>::clear` | | lib:STL | - | used by c15 catch funclets |
| 0x180187280 | `std::vector<cv::Mat>::clear` | | lib:STL | - | |
| 0x1801872E0 | `LoopClosing::clearReLocFrames` | `void ()` | project | new | lock +40, reloc_kf_list_.clear() |
| 0x180187370 | `LoopClosing::backProject` | `void (Eigen::Matrix2Xd px, CameraPtr cam, Eigen::Matrix3Xf* f)` | project | new | cam->backProject3 (vtable slot 1, batch) → cast<float> |
| 0x180187610 | `std::map<int, vector<vector<KeyFramePtr>>>::erase(const int&)` | | lib:STL | - | PlatMap::kf_map_.erase |
| 0x180187830 | `LoopClosing::extractAndConvert` | see header | project | modified (see draft comment) | level_vec_(i)<=1, track id filter |
| 0x180187C90 | `LoopClosing::svoFrameToKeyframe` | `void (const FramePtr&, KeyFrame*, const Transformation&, bool)` | project | modified | clear SVO info, ids, T_imu_cam → KF+192, image clone, extractAndConvert |
| 0x180187E30 | `LoopClosing::getPoseKey` | `std::string (const Transformation&)` | project | new | "ix_iy_iz_iyaw_ipitch_iroll" (cells 0.15 m / 15°, +1e-6) |
| 0x180188720 | `std::string::insert(0, const char*, n)` | | lib:STL | - | operator+(const string&, string&&) |
| 0x180188830 | `LoopClosing::loopClosingThread` | `void ()` | project | new | wait lc_cond_, runPROnLatestKeyframe(opt.ignored_past_frames, run_lc_on_this_frame_) |
| 0x180188900 | `LoopClosing::load` | `void ()` | project | new | PlatMap::load, fallback load_bin(path+".pba") |
| 0x180188CE0 | `LoopClosing::loadIndex` | `bool ()` | project | new | PlatMap::loadIndex; map_id_ = max indexed id + 1; kLabLoc → loadTagIndex(same yaml) |
| 0x1801890E0 | `LoopClosing::loadSavePlatMapThread` | `void ()` | project | new | cmd 0 load / 1 save / 2 save+exit |
| 0x1801891B0 | `LoopClosing::loadTagIndex` | `bool (const std::string&)` | project | new | FileStorage "tag_index_" → tag_load_ |
| 0x180189730 | `boost::archive::detail::common_iarchive<binary_iarchive>::vload(class_id_type&)`-like (library-version dependent int16/int32 read) | | lib:boost | - | called from 0x180196FF0 |
| 0x1801899C0 | `LoopClosing::bundleAdjustKfList` | `bool (std::vector<std::vector<KeyFramePtr>>&)` | project | new | ceres BA of bundle poses (points/extrinsics constant), rewrites T_w_c_ and landmarks |
| 0x18018B3D0 | `LoopClosing::reLocalizeThread` | `void ()` | project | new | PlatMap::UpdateMap+save at 200 bundles; runReLocalization |
| 0x18018B600 | `LoopClosing::getReLocCorrection` | `bool (ReLocCorrectionInfo*)` | project | new | group/rank queued corrections; PIMAX_LC_BSTSIZE (static, default 4) |
| 0x18018C070 | `recovery_kf` | `void (PlatMap*)` | project | new | rebuild mixed_* (c15 calls it "recovery_kf") |
| 0x18018C360 | `std::vector<cv::Point2f>::reserve` | | lib:STL | - | |
| 0x18018C3A0 | `LoopClosing::resetLoopClosing` | `void ()` | project | new | clear flags, kf_list_, pose_key_map_, svo_keyframe_count_=0 |
| 0x18018C4E0 | `LoopClosing::resetReLocalize` | `void ()` | project | new | session end, map deletion, UpdateMap, save request, ++map_id_ |
| 0x18018C850 | `LoopClosing::runPROnLatestKeyframe` | `void (size_t, bool)` (both unused) | project | modified: database building only (LC-1/2/3 VLOGs l.658/694/707/723), detection removed | |
| 0x18018D770 | `LoopClosing::runReLocalization` | `void ()` | project | new (26 KB, complete) | see pipeline below |
| 0x180193E00 | `LoopClosing::save` | `void ()` | project | new | saveIndex + PlatMap::save if kf_list_.size()>20 && save_map_enabled_ |
| 0x180194020 | `LoopClosing::saveIndex` | `bool ()` | project | new | PlatMap::saveIndex (+ saveTagIndex in kLabMap) |
| 0x180194200 | `LoopClosing::saveTagIndex` | `bool (const std::string&)` | project | new | rewrites yaml: map_index_ + tag_index_ |
| 0x180195AB0 | `ceres_backend::Map::setParameterBlockConstant(std::shared_ptr<ParameterBlock>)` | header inline | project (c02 header) | - | not re-drafted |
| 0x180195B20 | `std::vector<cv::Point3f>::size` | | lib:STL | - | out-of-line instance |
| 0x180195B50 | `ceres_backend::Map::solve()` | header inline | project (c02 header) | - | `Solve(options, problem_.get(), &summary)` |
| 0x180195B70 | `LoopClosing::startAllThread` | `void ()` | project | new | 3 unique_ptr<std::thread> |
| 0x180195E60 | `LoopClosing::stopAllThread` | `void ()` | project | new | join all; resetReLocalize with cmd 2; g_logger.Close() |
| 0x180196230 | `std::to_string(float)` | | lib:STL | - | wrapper → 0x180196250 |
| 0x180196250 | `std::_Floating_to_string<float>("%f")` | | lib:STL | - | |
| 0x1801962E0 | `std::to_string(long long)` | | lib:STL | - | |
| 0x1801963D0 | `LoopClosing::triangulate` | see header | project | new | DLT + checks |
| 0x180196B60 | `LoopClosing::updateSVOPointsDescriptors` | `void (const KeyFramePtr&, bool)` | project | modified: KeyFramePtr instead of index, null check, reduced extractor signature | |
| 0x180196D50/D70/E10/F50/FF0 | `boost::archive::binary_iarchive` vtable slots (dtor thunks, vload overrides) | | lib:boost | - | vtable 0x1803BDE80 |
| 0x180197010/030/080/100/180 | `portable_binary_iarchive` vtable slots | | lib:boost (project portable archive, c06/c15) | - | vtable 0x1803BDEC0 |
| 0x1801971D0/2E0/390/440 | `boost::archive::binary_oarchive` vtable slots | | lib:boost | - | vtable 0x1803BDE30 |
| 0x1801974F0 | `_scprintf` (inline stdio) | | lib:CRT | - | |
| 0x180197550 | `sprintf_s`/`_snprintf` inline (FLIRT "swprintf_0") | | lib:CRT | - | |
| 0x1801975B0 | `MapAlignmentSE3::MapAlignmentSE3` | `(const MapAlignmentOptions&)` | project | identical (opengv members removed) | conservativeResize(3,100) x2 |
| 0x1801976B0 | `std::vector<24-byte T>::_Emplace_reallocate` | | lib:STL | - | histogram.obj COMDAT |
| 0x180197940 | `Histogram::Histogram(int, const vector<int>&, cv::Mat, int, const vector<int>&, const vector<array<float,2>>&, bool, bool)` | | project | Kimera-modified | |
| 0x180197BB0 | `Histogram::Histogram()` | | project | Kimera | |
| 0x180197C20 | `Histogram::~Histogram()` | | project | Kimera-modified (scalar delete of ranges_[i]) | |
| 0x180197CC0 | `Histogram::operator=(const Histogram&)` | | project | Kimera | histogram_ not copied |

## runReLocalization (0x18018D770) pipeline — fully reconstructed

1. `vk::Timer timer_total, timer_each`; copy `reloc_kf_list_[0..1]` (return if < 2) under +40.
2. ReLoc-1 VLOG (l.1952, sum of landmarks). For each KF: BoW extract if `bow_features_` empty (sets
   num_bow_features_), `updateSVOPointsDescriptors(kf,true)` if `mixed_features_` empty.
3. `[Reloc-2] Frame index` VLOG (l.1970) per KF; **return** if `svo_keypointsvector_.size() < opt.min_bow_features (20)`.
4. ReLoc-2 VLOG (l.1981); createBOW for KFs with empty `vec_bow_`; RecLoc-3 VLOG (l.1992, typo "RecLoc" kept).
5. `last_reloc_kfs_ = cur_kfs` (+1872). Candidate scoring over **all** `plat_map_->kf_list_` bundles with ≥ 2 KFs:
   score = Σ_c compareBOWs(cur[c], cand[c]); `multimap<double,size_t,greater>`; VLOG l.2029 prints score,
   opt.reloc_min_score (float +968) and opt.reloc_expected_score (+824) — **no threshold is applied**.
6. stop(); VLOG l.2035; `t_group = stop()`; VLOG "ReLoc-4.3 spatial grouping finish\n" l.2066 (nothing done).
7. Env params (re-read every call): HPS=4 (only printed), RE=2, DT=18, VNT=10, OIN=20 (only printed), RATIO=80,
   SNT=5, DRAW=0, TM=10; `ratio = RATIO/100.0f`; `[Reloc param]` LOGW printed once (static bool 0x18047EF20).
8. For the first `min(n_cand, min(TM, opt.reloc_max_candidates(20)))` candidates (index < kf_list_.size(), ≥ 2 KFs,
   every KF with ≥ opt.reloc_min_bow_keypoints (30) BoW kps):
   * BFMatcher(NORM_HAMMING): knn(cur0 BoW, map0 BoW, 2) ratio `ratio*d1 > d0` (float);
     knn(map0, map1, 2) ratio `d1*0.9 > d0` (double); join → (cur0, map0, map1) keypoint triplets; need ≥ SNT.
   * once: cam0/cam1 = cams_->getCameraShared(0/1); T_c1_c0 = cur1.T_w_c⁻¹·cur0.T_w_c; P0=[I|0], P1=[R|t];
     K = eye(3,3,**CV_32F**), dist = zeros(4,1,CV_32F).
   * back-project map0/map1/cur pixels (Matrix3Xf), DLT-triangulate each map0/map1 pair with the CURRENT rig
     extrinsics, keep z ≤ 15 → 3D (map cam0) / 2D (cur bearing x,y, not divided by z); need ≥ SNT.
   * `solvePnPRansac(.., K, dist, rvec, tvec, false, 200, (float)RE, 0.999, inliers, ITERATIVE)`; R_wc=R⁻¹,
     t_wc=-R⁻¹t; abort candidate if |t_wc| > 2 ("eigenTVecWorld norm gt 2").
   * inliers → points in cur0 (T_cur0_map0) and predicted pixels in cur1; once: two `unique_ptr<Frame>` (ids 0/1,
     ts·1e9 as int64, cams, KF T_w_c) with 1-level pyramids + `unique_ptr<Matcher>` (max_epi_search_steps=500,
     subpix_refinement=true); per inlier with depth in (0,50]: `findEpipolarMatchDirect(*f0,*f1,T_c1_c0, ftr(kCorner),
     1/d, 1/(1.2d), 1/(0.8d), d)`; valid if squared px distance to prediction ≤ DT.
   * if valid ≥ VNT: ceres_backend::Map(false); HuberLoss(1.0) allocated and **leaked** (never passed);
     PoseParameterBlock ids 0 (identity, const), 1 (T_c0_c1, const), 2 (T_map0_cur0, variable);
     General3DParameterBlock id kk+10 (const, Trivial); two ReprojectionErrors per point (loss nullptr);
     options DENSE_SCHUR, progress off, DOGLEG, 100 it, 1 thread, function_tolerance 1e-6; catch → LOGW + next.
   * if CONVERGENCE and |t_opt − t_pnp| ≤ 0.2: T_corr = cand0.T_w_c · T_opt · cur0.T_w_c⁻¹, normalize, keep yaw only
     (R2ypr(R,false)→ypr2R((yaw,0,0),false)), confidence = Σd²/valid, LOGW "Adding correction info", emplace into
     `reLoc_correction_info_` under +120. **No break**: later candidates may add more corrections.
   * DRAW: 2×2 colour mosaic, texts, circles, lines, `cv::imshow("Good Matches <rank>")`, `cv::waitKey(0)` (blocks).
9. release KF images unless opt.keep_reloc_images; VLOG l.2638 "rerank_pass=0, cost time: " (stop − t_group).

## Types

### LoopClosing (sizeof 0xC10) — offsets used in c16 (all others: see header / c15)
| offset | type | name | evidence |
|---|---|---|---|
| 16 | int | map_id_ | KeyFrame ctor arg, ++ in resetReLocalize (sure) |
| 20 | bool | save_map_enabled_ | save() (sure) |
| 24 | shared_ptr<PlatMap> | plat_map_ | (sure) |
| 40 | std::mutex | reloc_frames_mutex_ | (sure) |
| 120 | std::mutex | reloc_info_lock_ | lock_guard in reloc (sure) |
| 200 | vector<KeyFramePtr> | reloc_kf_list_ | (sure) |
| 224 | bool | reloc_success_ | set in getReLocCorrection, cleared in resetReLocalize (sure) |
| 232 | deque<ReLocCorrectionInfo> | reLoc_correction_info_ | 112-byte elements, frontend log name (sure) |
| 544 | CameraBundlePtr | cams_ | getCameraShared (sure) |
| 560 | LoopClosureOptions | options_ | (sure) |
| 1216 | LoopClosingMode (u8) | mode_ | (sure) |
| 1224 | std::string | map_tag_ | path = map_path + map_tag_ + name (sure) |
| 1328 / 1448 | TagIndex (120 B) | tag_save_ / tag_load_ | valid +0, T +8 (vector<Vector3d>), Q +32 (vector<Vector4d>), board_idx +56, string +80, frame_id_ +112, map_id_ +116 (sure) |
| 1768 | std::mutex | kf_list_mutex_ | (sure) |
| 1848 | vector<vector<KeyFramePtr>> | kf_list_ | (sure) — c15 calls it kf_list_loop_ |
| 1872 | vector<KeyFramePtr> | last_reloc_kfs_ | assigned from reloc KFs (sure type) |
| 1896 | vector<int> | last_run_lc_frame_trackIDs_ | (sure) |
| 1960 | aligned vector | cur_loop_check_viz_info_ | clear() in addFrameToPR |
| 2152/2160/2232/2240 | unique_ptr<thread>, cv, atomic<bool>, mutex | lc_* | thread fn 0x180188830 (sure) |
| 2320 | bool | run_lc_on_this_frame_ | (sure) |
| 2328/2336/2408/2416 | | reloc_* | thread fn 0x18018B3D0 (sure) |
| 2496 | atomic<int> | platmap_cmd_ | xchg stores (sure) |
| 2504/2512/2584/2592 | | load_save_* / platmap_* | thread fn 0x1801890E0 (sure) |
| 2672 | unordered_map<string,bool> | pose_key_map_ | node 0x38, insert({key,true}) (sure) |
| 2768 / 2776 | size_t | lc_frame_count_ / svo_keyframe_count_ | (sure) |
| 2784 | OrbVocabulary | voc_ | (sure) |
| 2920 | std::mutex | completed_flags_mutex_ | (sure) |
| 3000 | vector<bool> | completed_flags_ | bit ops on +3000/+3024 (sure) |
| 3032 / 3040 | double | prox_dist_thresh_ / cumulative_distance_ | (sure) |
| 3056 | std::thread | (unused) | dtor terminate check |
| 3072 | bool | platmap_built_ | plain store (sure) |
| 3073 | atomic<bool> | plat_map_ready_ | xchg (sure) |
| 3074 | atomic<bool> | reloc_busy_ | lock cmpxchg / xchg (sure) |

**Corrections to `draft/c15_platmap/loop_closing/loop_closing_c15.h`:** +2328 is the **ReLocalize** thread (cv +2336,
stop +2408, mutex +2416) and +2504 the **LoadSavePlatMap** thread (cv +2512, stop +2584, mutex +2592); the stop
flags and +3073/+3074 are `std::atomic<bool>`; +2496 is `std::atomic<int>` (command); +232 deque holds 112-byte
`ReLocCorrectionInfo`; +3000 is `std::vector<bool>` (+3024 is its bit count, not a separate member), +3032/+3040
are doubles; +2672 value type is `bool`; +2768/+2776 are lc_frame_count_/svo_keyframe_count_; +1872 is
`vector<KeyFramePtr>`; +1328/+1448 are 120-byte TagIndex structs (ints at +112/+116 belong to them);
+1744 elements are 96 bytes = upstream `ClosedLoop`; +1936 = map_alignment_se3_ (c14 writes it, assuming its
`v16` is the make_shared control block).

### LoopClosureOptions (656 B) — Pimax fields used by c16 (option offset / LC offset / default)
min_bow_features +256/816 (20, size_t) · reloc_expected_score +264/824 (0.6) · key_pos_resolution +272/832 (0.15) ·
key_ang_resolution +280/840 (15.0) · min_num_features +288/848 (40) · max_kf_num +296/856 (1500; ×5 in lab modes) ·
use_plat_map +393/953 (true) · skip_loop_detection +394/954 (true) · reloc_min_score +408/968 (0.5f) ·
reloc_max_candidates +412/972 (20) · reloc_min_bow_keypoints +416/976 (30) · keep_reloc_images +504/1064 (false) ·
map_path +512/1072 · min_kf_to_save_map +544/1104 (50) · max_num_maps +552/1112 (2) · map_name +560/1120 ·
map_index_name +592/1152. Upstream: beta +80/640 (factory 1.0), ignored_past_frames +88/648 (15),
enable_image_logging +161/721, image_log_base_path +168/728, proximity_dist_ratio +200/760 (0.01),
proximity_offset +208/768 (factory 0.2). All other Pimax defaults are in the header (unused here).

### ReLocCorrectionInfo (112 B)
+0 int map_id_, +8 double timestamp_, +16 float confidence_, +32 Transformation, +96 int -1, +100 int -1, +104 int64 0.
emplace ctor (int, double, float, const Transformation&) = 0x18017E200; copy = 0x180182450.

### KeyFrame (c15 names) — offsets used: map_id_ +0, NframeID_ +4, frame_id_ +8, cam_id_ +12 (= Frame+20, which
c11/c12 call `bundle_id_` → naming conflict to reconcile), lc_frame_count_ +16, timestamp_sec_abs_ +24,
keyframe_image_ +32, T_w_c_ +128, T_unk_192_ +192 (= Frame::T_imu_cam(), i.e. **T_b_c**), bow_* +352/+376/+400,
vec_bow_ +424, svo_features_mat_ +440, svo_features_ +536, svo_node_ids_ +560, svo_keypointsvector_ +584,
svo_landmarksvector_cam_ +608, svo_landmark_ids_ +632, svo_trackIDsvector_ +656, mixed_keypoints_ +680,
mixed_node_ids_ +704, mixed_features_ +728, num_bow_features_ +752.

### Frame (fields read here): id_ +16, bundle_id_ +20, T_f_w_ +64, img_pyr_ +128, is_keyframe_ +176,
mean_intensity_ +224, timestamp ns +240 (int64), T_body_cam_ +256, num_features_ +552, px_vec_ +560 (2xN float),
level_vec_ +624, landmark_vec_ +680, track_id_vec_ +704. FrameBundle: frames_ +0, bundle id +252.
Frame ctor used: `Frame(int id, int64 ts_ns, const CameraPtr&, const Transformation& T_w_f)` (0x180092420, stores
T_f_w_ = arg.inverse()); allocated via `make_unique` (aligned malloc 0x340).

### MapAlignmentSE3 (0xA0): points_new_ +0 / points_old_ +16 (Matrix<float,3,X>), num_points_ +32,
num_points_ransac_ +36, max_num_points_ +40 (100), ransac3d_min_pts_ +44, ransac3d_inlier_percent_ +48,
options_ +56 {int, double}, t_rel_combined_ +80, ransac_inliers_ +144. Factory options {8, 40.0}.

### Histogram (432 B): n_images_ +0, channels_ +8, mask_ +16, dims_ +112 (size_t), hist_size_ +120, ranges_ +128,
uniform_ +136, accumulate_ +137, Mats +144/+240/+336.

## External interfaces (calls out of the chunk)
| address | meaning |
|---|---|
| 0x18017F4C0 | `KeyFrame(int nframe_id, int cam_id, int frame_id, int map_id)` (c15) |
| 0x180178E40 | `commonLandMarkCheck(const vector<int>&, const vector<int>&, double)` (c15) |
| 0x18017AC00 / 0x18017DEF0 | unordered_map<string,bool> find / insert |
| 0x180182B70 | `PlatMap::UpdateMap(kf_list, map_id)` (locks PlatMap::mtx_) |
| 0x180182F80 / 0x180183AA0 | `PlatMap::load / load_bin` |
| 0x180183320 | `PlatMap::loadIndex` |
| 0x180183EC0 / 0x180184590 | `PlatMap::save / saveIndex` |
| 0x180184F30 | `PlatMap::Map2Vec(int map_id)` |
| 0x180174B70 / 0x1801747E0 / 0x180174480 / 0x180176330 / 0x180175160 | bow.cpp: extractBoWFeaturesFromImage / createBOW / compareBOWs / getNodeID / extractFeaturesFromSVOKeypoints |
| 0x180014A20 / 0x180009C80 / 0x180009B80 / 0x180013040 / 0x180024870 / 0x180014680 / 0x180029DF0 | minkindr: rotate / quat product / Transformation* (normalizing) / inverse / transform / normalize / toRotationMatrix |
| 0x1800F3260 | `R2ypr(const Matrix3d&, bool in_radian)` (degrees when false) |
| 0x18017EE10 | `ypr2R(const Vector3d&, bool in_radian)` |
| 0x1800DEA10 / 0x1800089C0 | RotationQuaternion(const Matrix3d&) / (const Quaterniond&) with CHECKs |
| 0x1801B41A0 | `NCamera::getCameraShared(size_t)` |
| 0x1801B3C00 | `CameraGeometryBase(int w, int h)`; camera vtable slot 1 backProject3 (batch), slot 3 project3 |
| 0x1800942B0 | `frame_utils::createImgPyramid(img, n_levels, pyr)` |
| 0x1800AE820 | `Matcher::findEpipolarMatchDirect(ref, cur, T_cur_ref, ftr, d_est_inv, d_min_inv, d_max_inv, double& depth)` |
| 0x18017F430 | FeatureWrapper ctor with 10 refs (type, px, f(raw), f(normalized), grad, score, level, landmark, seed_ref, track_id) — the binary passes the un-normalized bearing 3rd, normalized 4th |
| 0x180181040 | SeedRef(FramePtr, int) |
| 0x180018E90 | `ceres_backend::Map(bool)`; 0x18001B4D0 addParameterBlock; 0x18001B840 addResidualBlock; 0x1800202C0 setParameterBlockConstant(id) |
| 0x18008D960 / 0x18008DA10 | PoseParameterBlock(T, id) / estimate() |
| 0x18002CFB0 | General3DParameterBlock(const Vector3d&, uint64 id, bool=true) |
| 0x180008F50 | ReprojectionError(CameraConstPtr, const Vector2d&, const Matrix2d& info) — **make_shared(cam, obs) in 0x18017EB00 builds Identity inside → the ctor needs a default `information = Identity()` argument** |
| 0x18017F4A0 | ceres::HuberLoss(a) inline ctor; CauchyLoss vtable used inline |
| 0x18002CC00 | vk::Timer::stop() — returns `(double)(int)duration_ns * 1e-9` (int truncation!, vikit chunk) |
| 0x180014730 | steady_clock::now() (non-inlined) |
| 0x180354E20 / 0x18035AC70 / 0x1803551B0 | glog LogMessage(file,line) / stream() / ~LogMessage; `env_8` = FLAGS_v |
| 0x18000C2C0 / 0x18000F500 / 0x18000F6A0 | LOGE / LOGI / LOGW |

Globals: `0x18046A000` g_logger (Close() inlined in stopAllThread: `0x18046A0B0` = filebuf _Myfile),
`0x18047EF20` static bool "[Reloc param] printed", `0x18047EF28` init guard of static `best_size`
(PIMAX_LC_BSTSIZE).

## Constants
getPoseKey: +1e-6 on both resolutions, deg = rad/π·180. addFrameToPR: intensity 25.0 / 220.0, 2 cameras hard-coded.
reLocalizeThread: UpdateMap+save at kf_list_.size() ≥ 200. save(): > 20 bundles. getReLocCorrection: ≥ 5
corrections, weight = min(1/conf, 1e6f), key = (int)(|t|·100/10), best_size default 4.
BA: 640×480 cameras, CauchyLoss(1.0), DENSE_SCHUR / DOGLEG / 15 it / 1 thread. Reloc: see pipeline.
triangulate: |X₃| < 1e-8 reject. Solver::Options MSVC offsets: trust_region 88, max_iter 104, threads 120,
function_tolerance 184, linear_solver 208, minimizer_progress_to_stdout 376 (MSVC unordered_set is 64 B).

## Quirks / bugs to preserve
* addFrameToPR: `if (!kf) return;` after make_shared; distance for cumulative_distance_ uses camera-0 positions.
* computePoseDiff: translation difference computed in float.
* extractAndConvert: no null check on landmark_vec_[i]; filter is `level_vec_(i) <= 1`; landmark ids = track ids.
* load(): reports success even after the `.pba` fallback; uses `!good()` before close.
* loadIndex: tag index read from the platMap index yaml, not from `tag.yaml`.
* saveTagIndex: rewrites the yaml without `map_version_` (next loadIndex fails) and uses keys with trailing spaces.
* getReLocCorrection: averaging loop never advances its iterator.
* resetReLocalize: LOGI labels swapped (map_size=kf_list_.size(), kf_size=map_index_.size()).
* runReLocalization: no BoW score threshold; HuberLoss leak; corrections keep accumulating over candidates;
  bearing x/y used as normalized coordinates (no /z) for PnP and triangulation; K/dist are CV_32F;
  DRAW blocks on `waitKey(0)`; triangulate returns **true** for points behind cam1.
* Histogram dtor frees `new float[2]` with scalar `delete`.

## Open questions / TODO(verify)
* Exact names of Pimax option fields and of R2ypr/ypr2R helpers (0x1800F3260/0x18017EE10).
* FeatureWrapper ctor parameter order (3rd/4th bearing arguments) vs c06/c13 definitions.
* `tracked`/reprojection vectors could be `std::vector<Keypoint>` instead of `cv::Point2f` (same codegen except
  the push_back of matcher->px_cur_).
* Whether `vk::Timer` in this build truncates to int (0x18002CC00) — vikit chunk.
* KeyFrame+12 naming (cam_id_ vs bundle_id_).
