# c12_poseopt_reprojector — [0x18013C500, 0x180147110)

The chunk covers the end of `pose_optimizer.obj` (vikit `MiniLeastSquaresSolver<6,Transformation,PoseOptimizer>`
GN/LM bodies, `PoseOptimizer::run/removeOutliers/setRotationPrior/update`), all of `reprojector.obj`
(`Reprojector`, `reprojector_utils`, ORB-style rotation check, grid NMS) and the first part of
`stereo_triangulation.obj` (ctor, a Pimax triangulation helper, a std-dev helper and its STL/Eigen
instantiations). `StereoTriangulation::compute` itself (0x180147110) is the next chunk.

Draft sources (`draft/c12_poseopt_reprojector/`):

| file | contents |
|---|---|
| `vikit/solver/mini_least_squares_solver.h` | upstream header + layout comment |
| `vikit/solver/implementation/mini_least_squares_solver.hpp` | GN/LM (and the other template members); **all LOG/VLOG `__LINE__`s match the binary** |
| `frontend/pose_optimizer.h/.cpp` | PoseOptimizer (VLOG(5) on line 62 as in the binary) |
| `frontend/reprojector.h/.cpp` | Reprojector + reprojector_utils (VLOG(10) on line 121 as in the binary) + `Frame::getSeedDepth` |
| `frontend/stereo_triangulation_c12.cpp` | StereoTriangulation ctor, `triangulate()` helper, `computeStd()` — to be merged into stereo_triangulation.cpp |
| `common/c12_assumed_types.h` | the slice of Frame/Point/FeatureWrapper/FrameBundle/camera this chunk relies on (with offsets) |

## Function table

kind `project` = reconstructed in the draft. Upstream status refers to rpg_svo_pro_open.

| address | proposed qualified name | signature | kind | upstream status | summary |
|---|---|---|---|---|---|
| 0x18013C500 | `vk::solver::MiniLeastSquaresSolver<6,Transformation,PoseOptimizer>::optimizeGaussNewton` | `void (State&)` | project (template) | modified: singular warning prints only the message (no `H =`/`g =`), file is shorter (lines 59/65/78/84) | GN loop with rollback on error increase |
| 0x18013CD60 | `…::optimizeLevenbergMarquardt` | `void (State&)` | project (template) | identical (lines 98/150/162/172) | LM loop |
| 0x18013D8B0 | `pimax::totem::PoseOptimizer::removeOutliers` | `void (double, Frame*, std::vector<double>*, size_t*, size_t*)` | project | modified: CHECK_NOTNULL→LOGE "removeOutliers has NULL\n"; thresholds non-static; landmark test by `track_id_vec_(i) > -1`; float positions; residual outputs zero-initialised; keyframes call `point->removeObservation(frame->id_)`; sets `track_id_vec_(i) = -1` | reproject every feature at the optimised pose, drop outliers |
| 0x18013EA10 | lib:Eigen `triangular_solver_selector<…UnitLower…>` (6x6 LDLT forward subst.) | | lib:Eigen | - | called from LDLT::solve (0x18012F7E0, c11) |
| 0x18013EBD0 | lib:Eigen `triangular_solver_selector<…UnitUpper…>` (6x6 LDLT backward subst.) | | lib:Eigen | - | " |
| 0x18013ED70 | `PoseOptimizer::run` | `size_t (const FrameBundle::Ptr&, double reproj_thresh_px, bool remove_outliers)` | project | modified: CHECKs→LOGE "PoseOptimizer: No features in frames\n" (no return); no "Initial measurement sigma" VLOG; new `remove_outliers` flag gates removeOutliers/VLOG/stats/tracing; returns `n_meas_ - corners - edges` | pose-only optimisation entry |
| 0x18013F580 | `PoseOptimizer::setRotationPrior` | `void (const Quaternion&, double)` | project | identical | prior with info diag(0,0,0,1,1,1) |
| 0x18013F830 | `PoseOptimizer::update` | `void (const State&, const UpdateVector&, State&)` | project | identical | `exp(dx)*T`, renormalise |
| 0x18013FA30 | lib:std `vector<void*>::_Xlen` thunk | | lib:std | - | |
| 0x18013FA50 | lib:std `_Destroy_range<Reprojector::Candidate>` | | lib:std | - | releases `ref_frame` (stride 72) |
| 0x18013FAD0 | lib:std `vector<Candidate>::_Emplace_reallocate` | | lib:std | - | |
| 0x18013FC90 | lib:std `_Insertion_sort_unchecked<Candidate*, reprojectFrames-lambda>` | | lib:std | - | |
| 0x18013FFC0 | lib:std `_Make_heap_unchecked<Candidate*, lambda>` | | lib:std | - | |
| 0x1801403C0 | lib:std `_Med3_unchecked<Candidate*, lambda>` | | lib:std | - | shows the comparator (type >, n_obs >, depth <) |
| 0x180140470 | lib:std `_Move_unchecked<Candidate*>` | | lib:std | - | used by `erase` |
| 0x180140500 | lib:std `_Partition_by_median_guess_unchecked<Candidate*, lambda>` | | lib:std | - | |
| 0x180140850 | lib:std `_Pop_heap_hole_by_index<Candidate*, lambda>` | | lib:std | - | |
| 0x180140AA0 | lib:std `_Sort_heap_unchecked<Candidate*, lambda>` | | lib:std | - | |
| 0x180140CE0 | lib:std `_Sort_unchecked<Candidate*, lambda>` | | lib:std | - | called directly from reprojectFrames |
| 0x180140E20 | lib:std `_Uninitialized_move<Candidate*>` | | lib:std | - | |
| 0x180140EE0 | lib:std `vector<Candidate>::push_back(const&)` | | lib:std | - | |
| 0x180140F60 | lib:std `swap<Candidate>` (iter_swap) | | lib:std | - | |
| 0x180141120 | lib:std `operator+(std::string&&, std::string&&)` | | lib:std | - | COMDAT shared with loop closing etc. |
| 0x1801412D0 | lib:std `vector<int>::vector()` (unknown_libname_167) | | lib:std | - | element ctor for the TLS `rot_hist[30]` array |
| 0x1801412F0 | `Reprojector::Reprojector` | `(const ReprojectorOptions&, size_t camera_index)` | project | identical (layout differs, see Types) | copies 96 B options, zeroes rest, camera_index_@+232 |
| 0x1801413A0 | lib:std `vector<Candidate>::_Tidy` (dtor) | | lib:std | - | |
| 0x180141430 | lib:Eigen `call_assignment` Block<Matrix<float,4,-1>,4,1> = Block (aligned 16 B packet copy) | | lib:Eigen | - | `invmu_sigma2_a_b_vec_.col(n) = …` |
| 0x1801414A0 | `Reprojector::Candidate::operator=(const Candidate&)` | compiler-generated | lib:compiler-generated | - | |
| 0x180141510 | `Reprojector::Candidate::~Candidate` (scalar deleting) | compiler-generated | lib:compiler-generated | - | |
| 0x1801415B0 | `reprojector_utils::IC_Angle` | `float (const cv::Mat&, const Keypoint&, const std::vector<int>& u_max)` | project | new (ORB-SLAM IC_Angle verbatim; cvRound on the point) | patch orientation via intensity centroid |
| 0x180141910 | lib:std `vector<Candidate>::_Change_array` | | lib:std | - | |
| 0x1801419D0 | lib:std `_Destroy_range` forwarder | | lib:std | - | |
| 0x1801419F0 | lib:std `vector<Candidate>::_Reallocate_exactly` (reserve) | | lib:std | - | |
| 0x180141A80 | lib:std `allocator<Candidate>::allocate` | | lib:std | - | |
| 0x180141B00 | `reprojector_utils::computeUmax` | `std::vector<int> ()` | project | new (ORB-SLAM ORBextractor umax, HALF_PATCH_SIZE 15) | static-init helper |
| 0x180141BF0 | lib:std `allocator<Candidate>::deallocate` | | lib:std | - | |
| 0x180141C40 | `reprojector_utils::getCandidate` | `bool (const FramePtr& cur, const FramePtr& ref, const size_t& idx, Candidate&)` | project | modified (see below) | project a landmark/seed into cur frame |
| 0x1801420C0 | `Frame::getSeedDepth` | `FloatType (size_t) const` | project (inline, frame.h) | identical (float) | `1.0 / invmu` narrowed to float |
| 0x180142190 | `reprojector_utils::checkPatchExposure` (name invented) | `bool (const FramePtr& ref, const FramePtr& cur, const Keypoint& px_cur)` | project | new | mask test + 5x5 saturation test when a frame is dark/bright |
| 0x1801422D0 | `reprojector_utils::matchCandidate` | `bool (const FramePtr&, Candidate&, Matcher&, FeatureWrapper&, float* angle_ref, float* angle_cur, const std::vector<int>& umax, FloatType seed_sigma2_thresh)` | project | modified (see below) | direct match of one candidate |
| 0x180142A60 | `reprojector_utils::matchCandidates` | `void (const FramePtr&, size_t max_n, bool offset, bool gain, Candidates&, OccupandyGrid2D* (unused), Statistics&, FloatType thresh)` | project | modified (see below) | matching loop + rotation consistency |
| 0x180143380 | `Reprojector::reprojectFrames` | `void (const FramePtr&, const std::vector<FramePtr>&, std::vector<PointPtr>& trash, const bool& need_imu_init)` | project | modified (rewritten) | candidate generation |
| 0x180143DB0 | `reprojector_utils::filterCandidatesByGrid` (name invented) | `void (Candidates& in, Candidates& out)` | project | new | 16 px NMS over sorted candidates |
| 0x180144340 | lib:std `vector<T144>::_Emplace_reallocate` (144-byte POD used by StereoTriangulation::compute) | | lib:std | - | |
| 0x1801444E0 | lib:std `vector<8-byte>::_Emplace_reallocate` | | lib:std | - | (stereo compute) |
| 0x180144620 | lib:std `vector<FeatureType>::_Insert_range` | | lib:std | - | `type_vec_.insert(…)` in compute |
| 0x1801448C0 | lib:std `shuffle<size_t*, mt19937&>` (random_shuffle replacement) | | lib:std | - | MT19937 inline (0x9908B0DF) |
| 0x180144B80 | lib:Eigen `apply_rotation_in_the_plane` rows of Matrix4d (`applyOnTheLeft`) | | lib:Eigen | - | JacobiSVD |
| 0x180144D00 | lib:Eigen `applyOnTheRight` Matrix4d | | lib:Eigen | - | JacobiSVD |
| 0x180144E90 | lib:std `allocator<T144>::construct` (copy) | | lib:std | - | |
| 0x180144EF0 | lib:Eigen `internal::real_2x2_jacobi_svd<Matrix4d>` | | lib:Eigen | - | |
| 0x180145200 | lib:Eigen `dense_assignment_loop` scalar head/tail (int segment copy) | | lib:Eigen | - | `level_vec_.segment() = …` |
| 0x180145230 | lib:Eigen `unaligned_dense_assignment_loop` (int, unrolled x4) | | lib:Eigen | - | |
| 0x180145320 | `StereoTriangulation::StereoTriangulation` | `(const StereoTriangulationOptions&, const AbstractDetector::Ptr&)` | project | modified: extra `cv::Mat` member at +48 (object 144 B) | |
| 0x180145370 | lib:std `vector<T144>::_Tidy` | | lib:std | - | |
| 0x1801453F0 | lib:std `vector<T352>::_Tidy` (352-byte elements, atexit 0x18039E1C0) | | lib:std | - | owner unknown (a static vector) |
| 0x180145470 | `StereoTriangulation::triangulate` (name invented) | `double (const FramePtr& f0, const FramePtr& f1, const Transformation& T_f1f0, const Matrix<double,3,4>& P0, const Matrix<double,3,4>& P1, const Matcher&, const FeatureWrapper& ref_ftr, Vector3d& x3D) const` | project | new | DLT triangulation + depth/reprojection checks; returns depth or -1..-5 |
| 0x180145D30 | lib:std `vector<T144>::_Change_array` | | lib:std | - | |
| 0x180145DF0 | lib:std `vector<T144>::_Reallocate_exactly` | | lib:std | - | |
| 0x180145E80 | lib:std `vector<8-byte>::_Reallocate_exactly` | | lib:std | - | |
| 0x180145EF0 | lib:std `vector<12-byte>::_Reallocate_exactly` | | lib:std | - | |
| 0x180145F90 | lib:std `_Uninitialized_move<T144>` | | lib:std | - | |
| 0x180146040 | lib:std `_Copy_memmove` | | lib:std | - | |
| 0x180146070 | lib:std `_Uninitialized_move<T144>` (2nd) | | lib:std | - | |
| 0x1801460F0 | lib:Eigen `JacobiSVD<Matrix4d>::allocate` | | lib:Eigen | - | |
| 0x1801462F0 | lib:std `allocator<T144>::allocate` | | lib:std | - | |
| 0x180146360 | `StereoTriangulation::computeStd` (name invented; could be a file-static) | `double (const std::vector<double>&, double mean)` | project | new | `sqrt(sum pow(v-mean,2) / n)` |
| 0x180146440 | lib:Eigen `JacobiSVD<Matrix4d,ColPivHouseholderQRPreconditioner>::compute` | | lib:Eigen | - | also used by 0x1801963D0 |

## Detailed behaviour (Pimax changes)

### vikit solver (0x18013C500 / 0x18013CD60)
* Line numbers (glog `__LINE__`): GN singular 59, GN failure 65, GN success 78, GN converged 84,
  LM init 98, LM singular 150, LM success 162, LM failure 172 — the draft `.hpp` reproduces them exactly.
  `__FILE__` = `E:\code_codex\pimax_slam\beta111_5a7902_dll\thirdparty\vikit\vikit_solver\include\vikit\solver\implementation/mini_least_squares_solver.hpp`.
* GN singular branch: `LOG(WARNING) << "Matrix is close to singular! Stop Optimizing.";` only. LM keeps `<< "H = " << H_ << "g = " << g_`.
* `applyPrior` dispatch: `&MiniLeastSquaresSolver::applyPrior (0x1801331A0) != &PoseOptimizer::applyPrior (vcall{8} thunk)` survives
  in the binary as a real compare (always true) followed by a virtual call through vtable slot 1.
* `solve()` = `solveDefaultImpl` inlined: `LDLT` compute (0x18012FAF0) + solve (0x18012F7E0) + `isnan(dx[0])`.

### PoseOptimizer::run (0x18013ED70)
1. `if (!frame_bundle->numFeatures()) LOGE("PoseOptimizer: No features in frames\n");` — continues.
2. `focal_length_ = at(0)->getErrorMultiplier()` (`at()` throws if empty), `frame_bundle_ = …`, `T_imu_world = at(0)->T_imu_world()`.
3. `start_errors` via evaluateErrorImpl; `measurement_sigma_ = scale_estimator_.compute(start_errors)` (float → double). No VLOG.
4. optimize (strategy 0 GN / 1 LM); `frame->T_f_w_ = frame->T_cam_imu()*T_imu_world` for every frame.
5. Only if `remove_outliers`: removeOutliers on every frame, `VLOG(5)` (line 62) "PoseOptimzer: drop …", median stats
   (`vk::getMedian`; the binary computes the median index as `(int)floor((double)(int)(n/2))` — vikit header detail),
   tracing to `ofs_reproj_errors_` (`i*error_scale << ", "`, `std::endl`).
6. return `n_meas_ - n_deleted_corners - n_deleted_edges` (both 0 if !remove_outliers).
* Caller: `FrameProcessorBase` 0x180118740 (`run(pose_optimizer_, frame_bundle, a3, a2)` after optional `setRotationPrior`), reads `stats_` at +0x388/+0x380.

### removeOutliers (0x18013D8B0)
* threshold_uplane = thresh/focal_length_, threshold_bearing = |2 sin(0.5*frame->getAngleError(thresh))| — **recomputed every call**
  (upstream used function-local statics that froze the first value).
* Landmark test is `frame->track_id_vec_(i) > -1` (not `landmark_vec_[i] != nullptr`).
* Seed position: `ref.keyframe->T_world_cam().cast<float>() * ref.keyframe->getSeedPosInFrame(ref.seed_id)` (float, minkindr cast = R→Matrix3f→quaternion, renormalised).
* Residual branch order identical to upstream (edgelet: UnitPlane/BearingVectorDiff/ImagePlane; corner: same), measurement sigma 0.0.
* Outlier: count, `type = kOutlier`, `seed_ref.keyframe.reset()`, **if `frame->is_keyframe_`: `landmark_vec_[i]->removeObservation(frame->id_)`** (copy of the shared_ptr, null-checked), `landmark_vec_[i] = nullptr`, **`track_id_vec_(i) = -1`**.

### Reprojector::reprojectFrames (0x180143380) — rewritten
1. `if (cur_frame->mean_intensity_ /*+224*/ < 10.0) return;` (nothing reset).
2. `max = need_imu_init ? options_.max_map_features_per_frame : options_.max_n_features_per_frame;` `resizeFeatureStorage(max)`; `stats_.reset()`; `candidates_.clear()`.
   `need_imu_init` is `FrameProcessorBase+0xD80` (passed by reference; set to 0 after "imu_initial true").
3. Landmarks, for every non-null ref keyframe: `ref_imu_z = (float)T_world_imu().z`; for each feature with `track_id != -1` and type != kOutlier:
   skip if point id already in `unordered_set<int>`; if `ref_imu_z - 2.0 <= point->pos_.z || need_imu_init`:
   trash if `(n_failed > 30 && n_succeeded < n_failed) || obs_.size() < 2`; else depth gate `(0, 10]` m in the ref frame
   (float `T_f_w_.cast<float>()`), then `getCandidate` → push_back + insert id. Points that fail the height gate are silently skipped.
4. Converged corner/edgelet seeds (types 3,4) of every ref frame (**no null check here**) → getCandidate → push_back.
5. `VLOG(10) << "all candidates num: " << n << std::endl;` (line 121), `std::sort` with
   `lhs.type > rhs.type || (== && lhs.n_obs > rhs.n_obs) || (== && == && lhs.depth < rhs.depth)`.
6. `filterCandidatesByGrid(candidates_, local)` then `matchCandidates(cur_frame, max, offset(+69), gain(+70), local, grid_.get() (null, unused), lm_stats, seed_sigma2_thresh(+64, float))`; `stats_.add(lm_stats)`.
* Gone vs upstream: separate converged/unconverged seed stages, setGridCellsOccupied, enough-features shortcuts, global-map branch, per-stage VLOG(5), ">20% unconverged" warning, the grid itself (never allocated).

### getCandidate (0x180141C40)
* has_landmark = `ref->track_id_vec_(idx) > -1`. Landmark: `xyz = point->pos_.cast<double>()`, `n_reproj = n_succeeded - n_failed`.
  Seed: `if (ref->score_vec_(idx) > 10.0f) return false;` `xyz = ref->T_world_cam() * getSeedPosInFrame(idx).cast<double>()`.
* Depth gate in the current frame: `depth = (1.0 - 2.0*(x²+y²))*p.z + 2.0*((zx - wy)*p.x + (zy + wx)*p.y) + t.z` on `cur->T_f_w_`; reject `depth <= 0 || depth > 10`.
  (Same hand-written formula, with double literals, is used in reprojectFrames for the float case — see `depthInFrame()`.)
* `cur->isVisible(xyz, &px /*Vector2d*/)`; on failure seeds get `score_vec_(idx) += 3.0f`.
* `candidate = Candidate(ref, idx, px.cast<float>(), n_reproj, score_vec_(idx), type_vec_[idx], has_landmark ? obs_.size() : 0, depth)`.

### matchCandidate (0x1801422D0)
* `*angle_ref = *angle_cur = 0`; CHECKs → `LOGE("c.ref_frame.get() null\n")`, `LOGE("c.ref_index >=c.ref_frame->num_features_\n")`, return false.
* Seed (`track_id == -1`): getFeatureWrapper first; converged (3,4): `findMatchDirect(*ref, *cur, ref_ftr, getSeedDepth, c.cur_px)`;
  failure → `ref->score_vec_(idx) += 1.0f`, false; then `checkPatchExposure(c.ref_frame, frame, matcher.px_cur_)`.
  Unconverged (0,1): `depth_filter_utils::updateSeed(*cur, *ref, idx, matcher, thresh(float), false, false /*, default true*/)`.
  Anything else (map-point seeds) → return false (upstream CHECK(false)).
* Landmark: `getCloseViewObs(frame->pos() [float→double], ref_frame, idx)`; `LOGE("ref_ftr.landmark.get()\n")` if null;
  ref_depth = float norm of `ref_frame->pos() - landmark->pos()`; findMatchDirect failure → `n_failed_reproj_++`;
  `checkPatchExposure(ref_frame, frame, px_cur)`; if both `ref_ftr.px` and `matcher.px_cur_` are inside (16,624)x(16,464):
  `angle_ref = IC_Angle(c.ref_frame->img(), ref_ftr.px, umax)` (**quirk: image of c.ref_frame, pixel of the close-view frame**),
  `angle_cur = IC_Angle(frame->img(), matcher.px_cur_, umax)`; `n_succeeded_reproj_ += 1`; `feature.landmark = point`; track_id = `point->id_`.
* Edgelet: `feature.grad = (matcher.A_cur_ref_.cast<float>() * grad_ref).normalized()`.
* Writes type, px, f, level, track_id, score (`c.score`), copies `invmu_sigma2_a_b_vec_` column. `in_ba_graph_vec_` copy removed.

### matchCandidates (0x180142A60)
* Local `Matcher` (defaults inlined: align_max_iter 10, max_epi_length_optim 2.0, max_epi_search_steps 100, 0.5, 2.5 …; offset/gain from args).
* `static std::vector<int> umax = computeUmax();` (object 0x18047ED70, guard 0x18047ED88, atexit 0x1803A6050).
  `thread_local static std::vector<int> rot_hist[30];` (TLS, dtor 0x1803A6010); each call: `clear()` + `reserve(500)`.
* Per candidate: `++i`; `PointPtr point = c.ref_frame->landmark_vec_[(int)c.ref_index]` (copy, int index); `++n_trials`;
  `getEmptyFeatureWrapper`; on match: landmark → `sum_lm_succeeded_reproj += n_succeeded_reproj_`, `sum_lm_obs += obs_.size()`, `++n_lm_matches`, else `++n_seed_matches`;
  `++n_matches; ++frame->num_features_`; break if `max>0 && num_features_ >= max`; then if both angles != 0:
  `rot = ref - cur; if (rot < 0) rot += 360; bin = (int)roundf(rot * (30/360.f)); if (bin==30) bin=0;`
  `if ((unsigned)bin > 30) LOGW(" bin < 0 or bin > 30, bin value is %d \n", bin); else { rot_hist[bin].push_back(num_features_-1); ++n_rot; }`.
* After the loop, if `n_rot > 5`: ORB `ComputeThreeMaxima` **with all comparisons inverted** (three least-populated bins, INT_MAX init;
  `if (min2 > 0.1f*min1) ind2 = ind3 = -1; else if (min3 > 0.1f*min1) ind3 = -1;`) and for every feature in those bins whose
  `track_id != -1`: `frame->removeLandmark(idx)` (0x180094460: `landmark_vec_.at(idx) = nullptr; track_id_vec_(idx) = -1`).
  Note: the features keep `num_features_`, type, px etc.; only the landmark link is cut.
* `candidates.erase(begin, begin + i)`.

### filterCandidatesByGrid (0x180143DB0)
`out.reserve(200)`; `unordered_map<int, vector<int>> grid; grid.reserve(n)`; key = `floor(x*0.0625f)*10000 + floor(y*0.0625f)`;
for each still-`valid` candidate in sorted order: push to `out`, then invalidate every other valid candidate in the 3x3
neighbouring cells with squared distance < 256 (16 px).

### checkPatchExposure (0x180142190)
`x=(int)px.x, y=(int)px.y` (truncation); `cur->getMask().at<uchar>(y,x) == 0` → false; if neither cur nor ref is dark/bright → true;
else count pixels in the clamped 5x5 window of `cur->img_pyr_[0]` with value <10 or >240; false iff all are.

### StereoTriangulation::triangulate (0x180145470)
DLT from `ref_ftr.uv` / `matcher.uv_cur_` (unit-plane coords) and P0/P1 (3x4); `JacobiSVD<Matrix4d>(A, ComputeFullV)`;
`|w| < 1e-8 → -1`; `z0 <= 0 → -2`; `R_f1f0.row(2)·x + P1(2,3) <= 0 → -3`; `z0 < 0 → -2`; project in cam0 (status != visible → -2),
squared error vs `ref_ftr.px` `> frame0->level_reproj_thresh_[ref_ftr.level]` → -4; `x_c1 = P1.col(3) + R*x`, `z1 < 0 → -2`,
project in cam1 (→ -2), squared error vs `matcher.px_cur_` `> frame1->level_reproj_thresh_[matcher.search_level_]` → -5; else return `x3D.z`.
Caller (0x180147110) only calls it when the epipolar depth is in (0.05, 10].

## Types

### MiniLeastSquaresSolver<6,Transformation,PoseOptimizer> (sure)
| offset | type | name | evidence |
|---|---|---|---|
| 0x000 | vfptr | | vtable 0x1803B7318 (base) / 0x1803B7328 (PoseOptimizer) |
| 0x010 | Strategy (int) | solver_options_.strategy | run: `*(a1+16)` 0=GN 1=LM |
| 0x018 | double | mu_init | LM `mu_ = *(a1+24)` |
| 0x020 | double | nu_init | LM |
| 0x028 | size_t | max_iter | loops |
| 0x030 | size_t | max_trials | LM |
| 0x038 | bool | stop_when_error_increases | GN |
| 0x039 | bool | verbose | (ctor copies 56 B) |
| 0x040 | double | eps | GN/LM |
| 0x050 | Matrix6d | H_ | |
| 0x170 | Vector6d | g_ | |
| 0x1A0 | Vector6d | dx_ | |
| 0x1D0 | bool | have_prior_ | |
| 0x1E0 | Transformation | prior_ | setRotationPrior |
| 0x220 | Matrix6d | I_prior_ | |
| 0x340 | double | chi2_ | |
| 0x348 | double | rho_ | |
| 0x350 | double | mu_ (=0.01) | ctor |
| 0x358 | double | nu_ (=2.0) | ctor |
| 0x360 | size_t | n_meas_ | |
| 0x368 | bool | stop_ | |
| 0x370 | size_t | iter_ | |
| 0x378 | size_t | trials_ | |
The 16-byte vfptr slot is MSVC's normal behaviour for a 16-aligned polymorphic class; layout == upstream.

### PoseOptimizer (sure)
| offset | type | name |
|---|---|---|
| 0x380 | double | stats_.reproj_error_after |
| 0x388 | double | stats_.reproj_error_before |
| 0x390 | FrameBundle::Ptr | frame_bundle_ |
| 0x3A0 | double | prior_lambda_ |
| 0x3A8 | MADScaleEstimator | scale_estimator_ (vfptr only; `compute` = slot 1, returns float) |
| 0x3B0 | TukeyWeightFunction | robust_weight_ (ctor 0x1801B5C30) |
| 0x3C0 | double | measurement_sigma_ |
| 0x3C8 | ErrorType (int) | err_type_ (0 UnitPlane, 1 BearingVectorDiff, 2 ImagePlane) |
| 0x3D0 | double | focal_length_ |
| 0x3D8 | std::ofstream | ofs_reproj_errors_ (is_open → +0x460) |

### Reprojector (sizeof 0xF0, Eigen aligned new)
| offset | type | name | evidence |
|---|---|---|---|
| 0 | ReprojectorOptions (96 B) | options_ | ctor copies 6 OWORDs |
| 96 | Statistics (6×size_t) | stats_ | reset/add in reprojectFrames |
| 144 | Statistics | fixed_lm_stats_ | zeroed by ctor only |
| 192 | unique_ptr<OccupandyGrid2D> | fixed_landmark_grid_ | zeroed |
| 200 | unique_ptr<OccupandyGrid2D> | grid_ | passed (null) to matchCandidates |
| 208 | vector<Candidate> | candidates_ | |
| 232 | size_t | camera_index_ | ctor arg |

ReprojectorOptions: sure fields `max_n_features_per_frame`@0, `max_map_features_per_frame`@8, `seed_sigma2_thresh` **float**@64,
`affine_est_offset`@69, `affine_est_gain`@70. Size 96 = upstream (88 with float sigma) + an unknown 8-byte field (assumed @56). The remaining
offsets are guesses (upstream order) — reconcile with the options factory.

Statistics (Pimax): `n_matches, n_trials, sum_lm_succeeded_reproj, sum_lm_obs, n_lm_matches, n_seed_matches` (names invented for fields 2..5).

Candidate (72 B, sure): `bool valid=true`@0, `FramePtr ref_frame`@8, `size_t ref_index`@24, `Vector2f cur_px`@32, `int n_reproj=0`@40,
`float score`@44, `FeatureType type`@48, `size_t n_obs`@56, `float depth`@64. Ctor takes `ref_frame` by value (copy seen).

### StereoTriangulation (sizeof 0x90): options_ (32 B) @0, feature_detector_ @32, `cv::Mat` @48 (Pimax-new).

### Frame / Point / FeatureWrapper / Matcher (offsets used by this chunk; reconcile centrally)
See `draft/c12_poseopt_reprojector/common/c12_assumed_types.h`. Highlights:
* `FloatType = float` everywhere in per-feature storage (px/f/grad/score/seed state, Point::pos_ = Vector3f @+4).
* Frame: id_@16, cam_@40, T_f_w_@64, img_pyr_@128, is_keyframe_@176, **mean_intensity_@224 (double), is_dark_@232 (<25), is_bright_@233 (>220)**
  (set in 0x1800FE… alongside `cv::blur` 3x3 + `cv::mean`), T_body_cam_@256, T_cam_body_@320, per-level double table @432 (stereo),
  num_features_@552, px_vec_@560, f_vec_@576, **2xN float @592 (new, "uv")**, score_vec_@608 (VectorXf), level_vec_@624, grad_vec_@640,
  type_vec_@656, landmark_vec_@680, **track_id_vec_@704 before seed_ref_vec_@720**, invmu_sigma2_a_b_vec_@744.
* Point: id_@0, pos_@4 (Vector3f), obs_ (std::list) @24 (size @32), n_failed_reproj_@80, n_succeeded_reproj_@84.
* FeatureWrapper: type@0, px@8, f@32, **uv@56 (new Ref<Vector2f>)**, grad@80, score@104, level@112, landmark@120, seed_ref@128, track_id@136.
* Matcher: A_cur_ref_ (Matrix2d) @224, search_level_ @304, px_cur_ @312 (Vector2f), f_cur_ @320 (Vector3f), **uv_cur_ @332 (Vector2f, new)**.
* Camera: mask_ @0x38; vtable +0x18 project3, +0x28 errorMultiplier, +0x40 getAngleError.

## External interfaces (calls out of the chunk)
| address | meaning |
|---|---|
| 0x18013A840 | `PoseOptimizer::evaluateErrorImpl(T, H*, g*, vector<float>*)` (c11) |
| 0x1801331A0 | `MiniLeastSquaresSolver::applyPrior` (base; only its address is compared) |
| 0x1801331C0 / 0x1801330F0 / 0x180132D90 | `PoseOptimizer::applyPrior` / dtor / ctor (c11) |
| 0x1801338F0, 0x180135AF0, 0x1801364D0, 0x180136EA0, 0x180138430, 0x180139AE0 | pose_optimizer_utils: EdgeletBearingVectorDiff, EdgeletImagePlane, EdgeletUnitPlane, FeatureBearingVectorDiff, FeatureImagePlane, FeatureUnitPlane (c11) |
| 0x18012FAF0 / 0x18012F7E0 | Eigen `LDLT<Matrix6d>::compute` / `solve` |
| 0x18012F3A0 / 0x18012F060 | `operator<<(ostream&, Matrix6d)` / `(…, Vector6d)` |
| 0x1800178F0 / 0x1800385B0 | Eigen aligned malloc / VectorXd assignment (norm_max temp) |
| 0x180354E20 / 0x180354E50 / 0x18035AC70 / 0x1803551B0 | glog `LogMessage(file,line)` / `LogMessage(file,line,sev)` / `stream()` / dtor; `env_8` = FLAGS_v |
| 0x180006290 | `operator<<(ostream&, const char*)` |
| 0x18000C2C0 / 0x18000F6A0 | LOGE / LOGW on logger 0x18046A000 |
| 0x180094540 / 0x180094570 | `Frame::getAngleError` / `Frame::getErrorMultiplier` (tail-call camera vtable +0x40 / +0x28) |
| 0x180094880 | `Frame::getMask()` = `&cam_->mask_` |
| 0x180094FB0 | `Frame::isVisible(const Vector3d&, Vector2d*)` — Pimax: hard-coded 16 px margin, 640x480 |
| 0x180094580 / 0x180094550 | `Frame::getFeatureWrapper(idx)` / `getEmptyFeatureWrapper()` |
| 0x180095730 | `Frame::resizeFeatureStorage(n)` |
| 0x180094460 | `Frame::removeLandmark(const size_t&)` (name invented) |
| 0x180095150 | `FrameBundle::numFeatures()` |
| 0x18009BBE0 | `Point::removeObservation(int frame_id)` |
| 0x18009A780 | `Point::getCloseViewObs(const Vector3d&, FramePtr&, size_t&)` |
| 0x1800AF5E0 | `Matcher::findMatchDirect(const Frame&, const Frame&, const FeatureWrapper&, const FloatType&, Keypoint&)` (0 = kSuccess) |
| 0x1800A2AC0 | `depth_filter_utils::updateSeed(cur, ref, idx, matcher, float thresh, bool, bool, bool)` |
| 0x180009B80 / 0x180013040 / 0x180014A20 / 0x180029DF0 / 0x180009C80 | minkindr: transform compose / inverse / quaternion rotate / getRotationMatrix / quaternion product |
| 0x18008A080 / 0x18008A610 / 0x1800089C0 / 0x180132B70 / 0x18012FE00 | Quaternionf from Matrix3f / RotationQuaternion<float> ctor / RotationQuaternion<double>(Quaterniond) / (w,x,y,z) ctor / isLessThenEpsilons4thRoot |
| 0x180020AC0, 0x18000F970, 0x180009790, 0x1800B2920, 0x1800E5BE0 | shared_ptr copy / _Decref / dtor / copy-assign / move-assign |
| 0x1800BEF30, 0x180025640, 0x180062DE0 | vector<int>(n) / reserve / _Emplace_reallocate |
| 0x18000F840, 0x180091430, 0x1800199A0, 0x18009ECA0, 0x18009A2B0, 0x18009E450 | unordered_set<int>/unordered_map<int,vector<int>> internals |
| 0x1800BB7E0 | `vector<PointPtr>::_Emplace_reallocate` (trash_points.push_back) |
| cv::fastAtan2, cv::Mat::step1, roundf, floorf, ceilf, pow, sin, cos, sqrt | imports |

Globals: 0x18046A000 logger; 0x18047ED70 `umax` static (guard 0x18047ED88); TLS `rot_hist[30]` (TLS block +0x30, init bit at +0x300);
FLAGS_v (`env_8`).

## Constants
| value | where |
|---|---|
| 0.01 / 2.0 | mu_/nu_ initial values (ctor, c11) |
| 1e-4 | LM tau |
| 1/3, 2/3, 3.0 | LM mu update (0x3FD5555555555555, 0x3FE5555555555555) |
| 10.0 | min mean intensity (reprojectFrames); depth max (getCandidate, reprojectFrames); seed score skip (`> 10.0f`) |
| 2.0 | height margin below ref IMU (m) |
| 30 | n_failed_reproj_ trash threshold; histogram length |
| 3.0f / 1.0f | seed score increments (projection failure / direct match failure) |
| 16, 624, 464 | IC_Angle pixel window (float compares) and isVisible margin |
| 15 | HALF_PATCH_SIZE; umax = {15,15,15,15,14,14,14,13,13,12,11,10,9,8,6,3} (ORB) |
| 30/360 = 0.083333336f, 360.0f, 0.1f, >5 | rotation histogram |
| 500 / 200 | rot_hist reserve / NMS output reserve |
| 0.0625f, 10000, 256.0f | NMS grid cell 16 px, key multiplier, radius² |
| 10 / 240, 5x5 | patch exposure test |
| 1e-8, -1..-5 | triangulate |

## Quirks worth preserving
* IC_Angle for the reference uses `c.ref_frame`'s image with the close-view frame's pixel.
* The rotation check removes landmarks in the **least** populated bins (inverted ORB logic) and only cuts the landmark link.
* `grid_` is never allocated; matchCandidates receives a null pointer (unused). No occupancy marking at all.
* removeOutliers thresholds are not static anymore (fixes upstream's stale-static bug) — keep non-static.
* PoseOptimizer::run logs but does not stop on an empty feature set; `at(0)` throws on an empty bundle.
* `score_vec_` of seeds is a failure counter (+3 projection fail, +1 match fail, skip > 10).
* GN singular warning without matrices, LM with matrices.
* `getCandidate` uses `track_id_vec_` (not `landmark_vec_`) to decide landmark vs seed; matchCandidate uses `== -1`, others `> -1`.

## Open questions / TODO(verify)
* Names invented: `checkPatchExposure`, `filterCandidatesByGrid`, `computeUmax`, `computeThreeMinima`, `StereoTriangulation::triangulate`,
  `computeStd`, `Frame::removeLandmark`, `uv`/`uv_cur_`, `mean_intensity_/is_dark_/is_bright_`, `level_reproj_thresh_`, Statistics fields 2..5.
* ReprojectorOptions offsets other than 0/8/64/69/70 and the extra 8-byte field.
* Whether `umax` is `const`; whether `computeUmax` is a free function or a lambda (it is a separate non-inlined function in the binary).
* The 144-byte and 352-byte element types of the STL instantiations at 0x180144340.. / 0x1801453F0 (owned by stereo/other chunks).
* `__LINE__` alignment: pose_optimizer.cpp line 62 assumes the c11 functions come *after* run() in the merged file (only the ctor and
  setRotationPrior precede it in the draft). If c11's code is merged above run(), re-pad.
