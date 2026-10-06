# c10_frame_processor_base: 0x180118740 to 0x18012A2B0

Drafts are in `draft/c10_frame_processor_base/`:
- `frontend/frame_processor_base.h`: the c10 view of `FrameProcessorBase`. Every member that c10 touches is placed at its offset. Members owned by other chunks are kept as padding. Also declares `BaseOptions`, `ReprojectResult` and `ReLocCorrectionInfo`.
- `frontend/frame_processor_base.cpp`: all FrameProcessorBase members in this range. It also holds the async reprojection lambda; its body is at 0x1800E61D0, outside this range, but it is part of `projectMapInFrame`'s source.
- `frontend/imu_processor.h/.cpp`: the first three functions of `imu_processor.cpp` (ctor, dtor, addImuMeasurement). The ImuProcessor object starts at 0x180129D60.
- `vio_common/integration_base_inl.h`: the header-inline `IntegrationBase::propagate` and `repropagate`. These are VINS functions that the linker emitted in this object.
- `common/frame_inline.h`: the header-inline `Frame::set_T_w_imu` and `FrameBundle::setIMUState`.
- `loop_closing/serialization_helpers.h`: the project boost `serialize()` templates for `Eigen::Matrix`, `cv::Point3f` and `cv::Point2f`. They are inlined into the oserializer instantiations in this range.

## Scope
The range is the tail of `frontend/frame_processor_base.cpp`, plus the head of `frontend/imu_processor.cpp` from 0x180129D60. glog `__LINE__` values in this range are 2417, 2568, 2619, 2668, 2714, 2770 and 2774, so the source file is about 2800 lines long.

These functions are not in this range, although the chunk description expected them. They come earlier in the same object, in c09:
- ctor 0x1800DFDF0
- dtor 0x1800E46A0
- `addFrameBundle` 0x1800FA6B0 ("New Frame Bundle received: %d")
- `getMotionPrior` 0x18010A690
- `loadPriorPosition` 0x180115F00 ("Prior position loaded from %s")
- the async reprojection lambda 0x1800E61D0
- `collectTrashPoints` 0x1800CB4B0
- `computePoseDifference` 0x1800FDEE0
- the quaternion slerp 0x180115430
- `Frame::numTrackedIds` 0x180118580

Address order in this object does not follow source order. For example, `optimizeStructureConsecutive` (line 2668) is placed before `optimizeStructure` (line 2619).

## Function table

| address | proposed qualified name | signature | kind | upstream status | summary |
|---|---|---|---|---|---|
| 180118740 | `FrameProcessorBase::optimizePose` | `size_t(bool use_weighted_prior)` | project | modified: trace removed, VLOG/SVO_DEBUG to LOGD, `run()` takes an extra bool | Resets the pose optimizer (inlined), sets the rotation prior when `have_motion_prior_`, runs it, and logs "PoseOptimizer: ..." with nObs printed as `%f` (quirk). |
| 1801188C0 | `FrameProcessorBase::optimizeStructureConsecutive` | `void(const FrameBundlePtr&, int max_n_pts, int max_iter)` | project | modified (Pimax variant, line 2668) | Like 180118D80, but also requires `point->n_consecutive_tracked_ (+120) >= 2`. FrameProcessor calls it with (10, 5). |
| 180118D80 | `FrameProcessorBase::optimizeStructure` | `void(const FrameBundlePtr&, int max_n_pts, int max_iter)` | project | modified: no nth_element or max_n_pts cap, no `last_structure_optim_`, `optimize(max_iter,false)`, track_id test, an unordered_set over the whole bundle | Optimises every point with `obs_.size() >= 2` that is not an edgelet. Called with (options.structure_optimization_max_pts, 5). |
| 180119230 | - | | lib: `std::deque<T>::pop_back` bookkeeping (`if(--size==0) off=0`) | - | Only referenced from an unwind funclet at 0x18039B850. |
| 180119240 | - | | lib: `std::deque<T>::pop_front` bookkeeping | - | Only referenced from an unwind funclet at 0x18039B800. |
| 180119260 | `FrameProcessorBase::projectMapInFrame` | `ReprojectResult()` (sret) | project | modified heavily (see the draft header comment) | Overlap keyframes (halved and reused for static bundles). Prepends the last keyframe-bundle frame. Sync or async `reprojectFrames`. Drops reappearing track ids. Deletes trash points only for static bundles. Sums the statistics. Logs "Not enough matched features: %lu". |
| 18011A810 | `IntegrationBase::propagate` | `void(double dt, const Vector3d& acc_1, const Vector3d& gyr_1)` | project (header-inline, VINS) | identical (VINS-Mono) | midPointIntegration (0x180104140), then copy the results, `delta_q.normalize()`, `sum_dt += dt`. |
| 18011AA90 | `FrameProcessorBase::reLocalize` | `void()` | project | new | PlatMap relocalisation through `lc_`. "input reLocalize", "reLoc_correction_info_.size()", "reLocalize success, INFO=ReLocTimer...", "reloc fail, reLoc_times_ ...". |
| 18011AFE0 | `IntegrationBase::repropagate` | `void(const Vector3d& ba, const Vector3d& bg)` | project (header-inline) | modified: samples with `dt_buf[i] < 1e-4` are skipped | Reset the state, then re-propagate the buffered samples. |
| 18011B240 | - | | lib: `std::unordered_map<..>::reserve(n)` (`_Hash` rehash for max_load_factor) | - | Callers 0x1800EC2B0 (plane merge) and 0x18019FA90. |
| 18011B330 | - | | lib: `std::vector<T*>::reserve` (8-byte elements) | - | Caller is the ground-plane code at 0x1800F64A0. |
| 18011B370 | `FrameProcessorBase::resetAll` | `void()` (vtable [3]) | project (header-inline) | identical | `{ resetVisionFrontendCommon(); }` |
| 18011B380 | `FrameProcessorBase::resetBackend` | `void()` (vtable [4]) | project | modified: the backend is only `reset()` and never released; clears the IMU window and gyro stats | Logs "FrameProcessorBase resetBackend". |
| 18011B420 | `FrameProcessorBase::resetVisionFrontendCommon` | `void()` | project | modified: no sparse_img_align or reloc keyframe; restores the IMU biases; clears Pimax state, the ground plane and the backend ground plane | Logs "resetVisionFrontendCommon". |
| 18011B8F0 | `FrameProcessorBase::resetVisionFrontendCommonWhenSetStart` | `void()` | project | new (a reduced copy of the above) | Keeps `set_reset_`, `c_kf_`, the ground plane and the backend. Logs "resetVisionFrontendCommonWhenSetStart". |
| 18011BCA0 | `FrameProcessorBase::checkTrackingHealth` (name is a guess) | `void()` | project | new | Tracking-reset watchdog that sets `set_reset_`. Logs "Reset b_m_lost_r %d, t_diff %f, ...". |
| 18011CAB0 | - | | lib: `Eigen::DenseStorage<T,Dynamic,Dynamic,Dynamic>::resize(size,rows,cols)` | - | |
| 18011CB30 | - | | lib: Eigen `PlainObjectBase::resize` fixed-size check (15) | - | |
| 18011CB60 | - | | lib: `std::vector<int>::resize` | - | Used by the boost iserializer for vector<int> and by ceres. |
| 18011CC00 | - | | lib: `std::vector<8-byte POD>::resize` | - | DBoW2 and boost. |
| 18011CCA0 | - | | lib: `std::vector<cv::Point3f>::resize` | - | |
| 18011CD60 | - | | lib: `std::vector<std::set<int>>::resize` | - | |
| 18011CE30 | - | | lib: `std::vector<std::vector<KeyFrame*>>::resize` | - | |
| 18011CF30 | - | | lib: `std::vector<std::vector<FramePtr>>::resize` | - | |
| 18011D030 | - | | lib: `std::vector<cv::Mat>::resize` | - | |
| 18011D130 | - | | lib: `std::vector<bool>::resize(n, val)` | - | |
| 18011D3B0 | - | | lib: `std::rethrow_exception` (import thunk) | - | |
| 18011D3C0 | - | | lib: Eigen `call_assignment` (resize + float block copy) | - | |
| 18011D4C0 | - | | lib: Eigen `call_assignment` (resize + transposed float copy) | - | |
| 18011D5C0 | - | | lib: Eigen `PartialPivLU<Matrix<double,15,15>>` construct/compute | - | |
| 18011D820 | - | | lib: Eigen `internal::apply_rotation_in_the_plane<float>` | - | |
| 18011D9B0 | - | | lib: Eigen `gebp_kernel<float>` | - | |
| 18011E950 | - | | lib: Eigen `gebp_kernel<float>` (variant) | - | |
| 18011F870 | - | | lib: Eigen `gebp_kernel<float>` (variant) | - | |
| 180120960 | - | | lib: Eigen `general_matrix_matrix_product<..>::run` | - | |
| 180121280 | - | | lib: Eigen gemm run (variant) | - | |
| 180121BD0 | - | | lib: Eigen gemm run (variant) | - | |
| 1801224D0 | - | | lib: Eigen gemm run (variant) | - | |
| 180122DB0 | - | | lib: Eigen `ColPivHouseholderQR<MatrixXf>::_solve_impl` | - | |
| 180123300 | - | | lib: Eigen `ColPivHouseholderQR` transposed solve | - | |
| 1801239E0 | - | | lib: minkindr `QuatTransformationTemplate<double>::getTransformationMatrix()` | - | Callers are the API timestamp checks. |
| 180123CE0 | - | | lib: Eigen assignment loop (float setZero) | - | |
| 180123E40 | - | | lib: Eigen assignment loop (float setConstant) | - | |
| 180123F40 | - | | lib: Eigen assignment loop (float strided copy) | - | |
| 180124110 | - | | lib: Eigen `triangular_solve_matrix<float>` | - | |
| 180124670 | - | | lib: Eigen gemm or gemv with static blocking (`_Init_thread_header`) | - | |
| 180125040 | `FrameProcessorBase::savePriorPosition` | `void(const Transformation& T_world_imu)` | project | new | Writes `trace_dir + "/pimax_prior_position.txt"` as "x y z\n". Called from the dtor with `T_world_correction_ * T_prior_`. |
| 180125420 | - | | lib: boost `oserializer<binary_oarchive, std::pair<const unsigned,double>>::save_object_data` | - | |
| 180125550 | - | | lib: boost `oserializer<binary_oarchive, Eigen::Vector3d>` (inlines the project `serialize` template, see draft) | - | |
| 180125640 | - | | lib: boost `oserializer<binary_oarchive, cv::Point3f>` (inlines the project serialize) | - | |
| 1801256E0 | - | | lib: boost `oserializer<binary_oarchive, cv::Point2f>` (inlines the project serialize) | - | |
| 180125760 | - | | lib: boost `oserializer<binary_oarchive, kindr Transformation>` (calls 0x1800DB8D0) | - | |
| 1801257D0 | - | | lib: boost `oserializer<binary_oarchive, std::map<unsigned,double>>` | - | |
| 180125960 | - | | lib: boost `oserializer<binary_oarchive, std::vector<int>>` | - | |
| 180125A20 | - | | lib: boost `oserializer<binary_oarchive, std::vector<KeyFrame*>>` | - | |
| 180125BE0 | - | | lib: boost `oserializer<binary_oarchive, std::vector<cv::Point3f>>` | - | |
| 180125D30 | - | | lib: boost `oserializer<binary_oarchive, std::vector<cv::Point2f>>` | - | |
| 180125E70 | - | | lib: boost `oserializer<binary_oarchive, std::vector<std::vector<KeyFrame*>>>` | - | |
| 180125FC0 | - | | lib: boost `oserializer<binary_oarchive, std::vector<cv::Mat>>` | - | |
| 180126110 | - | | lib: boost `oserializer<binary_oarchive, DBoW2::BowVector>` | - | |
| 180126180 | - | | lib: boost `oserializer<binary_oarchive, KeyFrame>` (calls 0x1800DBD40) | - | |
| 1801261F0 | - | | lib: boost `oserializer<binary_oarchive, cv::Mat>` | - | |
| 180126260 | - | | lib: boost `oserializer<binary_oarchive, PlatMap>` | - | |
| 1801262D0 | - | | lib: boost `oserializer<portable_binary_oarchive, std::pair<const unsigned,double>>` | - | |
| 1801263B0 | - | | lib: boost `oserializer<portable_binary_oarchive, Eigen::Vector3d>` (inlines the project serialize) | - | |
| 180126510 | - | | lib: boost `oserializer<portable_binary_oarchive, cv::Point3f>` (inlines the project serialize) | - | |
| 1801265B0 | - | | lib: boost `oserializer<portable_binary_oarchive, cv::Point2f>` (inlines the project serialize) | - | |
| 180126630 | - | | lib: boost `oserializer<portable_binary_oarchive, kindr Transformation>` (calls 0x1800DC8A0) | - | |
| 1801266A0 | - | | lib: boost `oserializer<portable_binary_oarchive, std::map<unsigned,double>>` | - | |
| 180126850 | - | | lib: boost `oserializer<portable_binary_oarchive, std::vector<int>>` | - | |
| 1801269B0 | - | | lib: boost `oserializer<portable_binary_oarchive, std::vector<KeyFrame*>>` | - | |
| 180126B80 | - | | lib: boost `oserializer<portable_binary_oarchive, std::vector<cv::Point3f>>` | - | |
| 180126CF0 | - | | lib: boost `oserializer<portable_binary_oarchive, std::vector<cv::Point2f>>` | - | |
| 180126E40 | - | | lib: boost `oserializer<portable_binary_oarchive, std::vector<std::vector<KeyFrame*>>>` | - | |
| 180126FB0 | - | | lib: boost `oserializer<portable_binary_oarchive, std::vector<cv::Mat>>` | - | |
| 180127120 | - | | lib: boost `oserializer<portable_binary_oarchive, DBoW2::BowVector>` | - | |
| 180127190 | - | | lib: boost `oserializer<portable_binary_oarchive, KeyFrame>` | - | |
| 180127200 | - | | lib: boost `oserializer<portable_binary_oarchive, cv::Mat>` | - | |
| 180127270 | - | | lib: boost `oserializer<portable_binary_oarchive, PlatMap>` | - | |
| 1801272E0 | - | | lib: boost `pointer_oserializer<binary_oarchive, KeyFrame>::save_object_ptr` | - | |
| 180127370 | - | | lib: boost `pointer_oserializer<portable_binary_oarchive, KeyFrame>::save_object_ptr` | - | |
| 180127400 | - | | lib: MSVC PPL `_Schedule_chore` wrapper used by `std::async` ("Fail to schedule the chore!") | - | |
| 1801274A0 | `FrameProcessorBase::setBundleAdjuster` | `void(const std::shared_ptr<CeresBackendInterface>&)` | project | modified: null check, so the type becomes kNone when ba is null | Caller is the API ("Headset tracking version %s"). |
| 180127550 | `FrameProcessorBase::setFirstFrames` | `void(const std::vector<FramePtr>&)` (vtable [1]) | project | modified: `Map::addKeyframe(f)` takes one argument | |
| 180127700 | `FrameBundle::setIMUState` | `void(const Vector3d& vel, const Vector3d& gyr_bias, const Vector3d& acc_bias)` | project (header-inline) | modified: the state is stored in the bundle (+144/+168/+192), not in each frame | Called from IMU init 0x180110490. |
| 180127750 | - | | lib: Eigen fixed `Matrix<int,2,1>` resize check plus (0,1) fill | - | Callers 0x180101D70 and 0x1801035F0. |
| 1801277B0 | `FrameProcessorBase::setInitialPose` | `void(const FrameBundlePtr&, const Eigen::Quaternionf& att, bool use_att)` | project | modified heavily | Gravity from the newest bundle IMU sample (needs > 10), reference axis (0,0,1), orthogonality check, optional external attitude with history slerp. Otherwise `T_C_B * T_world_imuinit^-1`. |
| 1801280A0 | `FrameProcessorBase::setRotationIncrementPrior` | `void(const Eigen::Quaterniond&)` | project | modified: VLOG to LOGI; plain Eigen quaternions | |
| 180128210 | `FrameProcessorBase::setRotationPrior` | `void(const Eigen::Quaterniond&)` | project | modified: VLOG to LOGI | |
| 180128260 | `FrameProcessorBase::setTrackingQuality` | `void(size_t)` (vtable [5]) | project | modified (see the draft) | "Lost %lu  features!" |
| 180128320 | - | | lib: Eigen `TriangularView` assignment | - | Caller 0x1800CDF80. |
| 1801283F0 | `Frame::set_T_w_imu` | `void(const Transformation&)` | project (header-inline) | modified: explicit renormalisation of `T_f_w_` | |
| 1801284D0 | - | | lib: Eigen `Quaternionf::toRotationMatrix` | - | |
| 180128610 | - | | lib: Eigen `partial_lu_impl<double,0,int,15>::unblocked_lu` | - | |
| 180128F60 | - | | lib: `std::unique_lock<std::mutex>::unlock` | - | |
| 180128FA0 | `FrameProcessorBase::upgradeSeedsToFeatures` | `void(const FramePtr&)` | project | modified: track ids, no map points or fixed landmarks, float seed position | VLOG(40) and VLOG(5) messages and the LOG(WARNING) "% updated seeds are unconverged." |
| 180129810 | - | | lib: Eigen `Block<..,3,1>` construction | - | |
| 180129860 | - | | lib: trivial `*out = 0; return out;` (no callers; data-referenced) | - | |
| 180129870 | - | | lib: `std::deque<ImuMeasurement>` range assign | - | Called by ImuProcessor (c11). |
| 180129A60 | - | | lib: `std::unordered_map<IMUTemporalStatus,std::string>::insert(first,last)` | - | Dynamic initialiser 0x1800028D0. |
| 180129D60 | `ImuProcessor::ImuProcessor` | `(const ImuCalibration&, const ImuInitialization&)` | project | modified: no IMUHandlerOptions | |
| 180129EC0 | `ImuProcessor::~ImuProcessor` | `()` | project | identical (empty body) | |
| 180129F40 | `ImuProcessor::addImuMeasurement` | `void(const ImuMeasurement&)` | project | modified: returns void; clears the buffer with LOGE "t rollback" when the new timestamp is older than the front; no temporal window | |
| 18012A0C0 | - | | lib: `std::deque<ImuMeasurement>::erase(first,last)` | - | |

## Function details (behaviour that differs from upstream)

### optimizePose (0x180118740)
- The inlined `pose_optimizer_->reset()` sets:
  - +832 = 1e10 (chi2)
  - +848 = +24 (mu)
  - +856 = +32 (nu)
  - +864, +880 and +888 = 0
  - byte +872 = 0
  - byte +464 = 0 (have_prior)
- `setRotationPrior` (0x18013F580) receives `new_frames_->get_T_W_B().getRotation()` inverted with **Eigen** `inverse()`, which divides by the squared norm (0x1800132A0). It also receives `options_.poseoptim_prior_lambda` (FPB+176).
- `run` (0x18013ED70) receives `(new_frames_, options_.poseoptim_thresh (FPB+168), bool)`.
- The log arguments are `stats_.reproj_error_before` (PO+904) and `stats_.reproj_error_after` (PO+896), followed by the size_t `nObs` printed with `%f`.

### projectMapInFrame (0x180119260)
1. `ReprojectResult` (72 bytes) is zero-initialised. VLOG(40) "Project map in frame." (line 2417). `g_permon->startTimer("reproject")` runs when trace_statistics is set. There is no matching stop.
2. The overlap is computed when `!new_frames_->is_static_ || last_kf_frames_->is_keyframe_ || imu_not_initialized_`:
   - `max_n_kfs = reprojector->options_.max_n_kfs` (Reprojector+32), shifted right by 1 for static bundles;
   - `overlap_kfs_.at(c).clear()`;
   - `map_->getClosestNKeyframesWithOverlap(new_frames_->at(c), max_n_kfs /*const size_t&*/, &overlap_kfs_.at(c))`.

   Otherwise `overlap_kfs_ = last_overlap_kfs_`.
3. Asynchronous path (`options_.use_async_reprojectors`). Unlike upstream, it does not also require more than one camera:
   - `std::vector<std::future<void>>`, with `std::async(std::launch::async, [this, c]{...})`;
   - the lambda at 0x1800E61D0 does, in order:
     - prepend `last_kf_frames_->at(c)`;
     - dedupe through `std::unordered_set<FramePtr>` (assign back in set iteration order);
     - `std::sort` by `id_` descending;
     - `reprojectFrames`;
   - then `.get()` on each future.
4. Synchronous path: prepend `last_kf_frames_->at(c)` (no dedupe or sort), then `reprojectors_.at(c)->reprojectFrames(new_frames_->at(c), overlap_kfs_.at(c), trash_points_.at(c), imu_not_initialized_)`. The fifth argument is passed by address.
5. `last_overlap_kfs_ = overlap_kfs_`.
6. Track-id filter. `cur` is a local `std::vector<std::set<int>>`, resized to numCameras.
   - For each feature with a track id: if the id is in `track_ids_last_last_[c]`, is not in `track_ids_last_[c]`, and `(last_frames_->is_static_ || frame_flag234_cnt_ > 6)`:
     - push the landmark (if any) into `trash_points_[c]`;
     - set the landmark to null and the track id to -1.
   - Otherwise insert the id into `cur[c]`.
   - Then `track_ids_last_last_.swap(track_ids_last_); track_ids_last_.swap(cur)`.
7. `trash_set = collectTrashPoints(new_frames_->is_static_, trash_points_)` (0x1800CB4B0). The following steps run only for static bundles:
   - clear the matching landmarks in `new_frames_`;
   - clear them in every bundle of `kf_bundles_` (+3464);
   - call `map_->safeDeletePoint` for each point.

   After that, all `trash_points_` vectors are cleared.
8. The statistics are summed over the reprojectors: stats +96/+104/+112/+120/+128/+136, plus fixed_lm_stats_ +144.
   - VLOG(40) "Reprojection:\t nPoints = ..\t\t nMatches = .." (line 2568).
   - `n_total = n_matches + (fixed > 10 ? fixed : 0)`. When it is below `quality_min_fts`, LOGW "Not enough matched features: %lu\n".
   - Result fields: see `ReprojectResult` in the header. The float ratios use the `(float)size_t` conversion.

### reLocalize (0x18011AA90)
LOGI "input reLocalize". The function needs `last_frames_ && last_last_frames_ && !imu_not_initialized_`, then:
- `new_frames_->frames_.at(0)->img_mean_ (+224) >= 25` and `numTrackedIds() >= 30` (written as early returns with `<`);
- `lc_` set;
- `lc_+953` set;
- `reloc_session_count_ (+3940) > 0`.

If the PlatMap is empty (`(lc_+24)->+96 == 0`), it sets `reloc_enabled_=false; reLoc_times_=0` and returns. Otherwise the relocalisation runs when:
- `!imu_not_initialized_`;
- `reloc_enabled_` (+3936);
- `!(t_new - last_reloc_success_time_ < 2.0)` (+3952/+3960);
- `!(t_last - last_reloc_try_time_ < 0.01)` (+3968/+3976).

The comparisons are written as `!(a<b)` so that NaN behaves the same as in the binary.

On success (`lc_->reLocalize(&info)`, 0x18018B600):
- LOGW size of `lc_->reLoc_correction_info_` (deque at lc_+232; size at lc_+264);
- `T_world_correction_ (+2912) = info.T * T_world_correction_`;
- `new_frames_->is_relocalized_ (+64) = true`;
- `last_reloc_success_time_ = t_new`;
- `lc_+16 = info.kf_id`;
- `reloc_enabled_=false; reLoc_times_=0`;
- clear the deque;
- `t2 = reloc_timer_.stop()`;
- LOGW "reLocalize success ... t1=%f t2=%f" (t1 = +4008).

On failure:
- `lc_->addReLocFrames(last_frames_, T_world_correction_)` (0x180186A50);
- `++reLoc_times_`;
- `last_reloc_try_time_ = t_last`;
- if `reLoc_times_ > max_reLoc_times_ (300) && slam_mode_ (+464) != 2`: LOGI "reloc fail ...", then `reloc_enabled_=false; reLoc_times_=0`.

### checkTrackingHealth (0x18011BCA0)
The log names map to fields as follows (see the header):

| Log name | Rule |
|---|---|
| dark_t_ (+644) | ++ when max over frames of `img_mean_ < 15`, else reset |
| bright_t (+648) | ++ when min over frames of `img_mean_ > 220` (min starts at 255, max at 0) |
| b_m_lost_r | \|t_new - t_last\| (int64 ns) > 165000000; t_diff is printed in ms (`*1e-6`) |
| low_q_num (+632) | In tracking: `num_tracked_last_ (+16) < 20` gives ++, else reset. Outside tracking: reset |
| b_t_less | low_q_num > 10 |
| vel_fly (+640) | In tracking: `new_frames_->imu_vel_w_ (+144).norm() > 4.5` gives ++, else reset. Outside tracking: reset |
| b_vel_fly | vel_fly > 12 |
| low_m_r_num (+636) | Read only; FrameProcessor updates it |
| v_fast_times_ (+656) | `computePoseDifference(T_W_B_new, T_W_B_last)` (0x1800FDEE0). If `dist / (t_new - t_last + 1e-6) > 4.5`, post-increment |
| b_big_draft | Set when the old v_fast value is >= 12. Otherwise set when `feature_d_m_ (+652) > 6` |
| b_light | Tracking && (dark_t_ > 6 \|\| bright_t_ > 6). Not printed |
| low_m_ba (+660) | Reset when the IMU is not initialised, or `last_frames_->is_static_`, or `last_frames_->numLandmarksInBA() >= 10`; else ++ |
| e_f_f (+680) | Number of frames with `numTrackedIds()==0` >= 3 gives ++, else reset |
| low_m_f (+664) | Only on keyframe bundles: `force_stereo_triangulation_ (+229) && |vel| > 0.1` gives ++, else reset |
| c_kf (+688) | Keyframe && tracking && IMU initialised && !static gives ++, else reset |
| b_dist (+668) | `(double)numTrackedLandmarks() > 0.8*(double)numLandmarks() && tracking` gives ++, else reset |
| low_marks (+672) | `numTrackedIds() >= 10` resets, else ++ |
| delt_z | IMU initialised: `fabs(float ground_height_ (+3584) - (float)T_W_B_new.z)`, computed in float |
| low_c (+676) | IMU initialised && |vel| > 1 && fewer than 30 common point ids with `last_last_frames_` gives ++, else reset |
| acc_b_n | Constant 0.0 |

The reset fires when any of these holds: b_m_lost_r, low_q_num > 10, vel_fly > 12, low_m_r_num > 5, b_big_draft, b_light, `exposure_level_ (+3112) > 75`, low_m_ba > 10, low_m_f > 6, b_dist > 6, low_marks > 6, delt_z > 5.0, low_c > 10, e_f_f > 6, p_diff_ (+684), c_kf > 10, or (IMU not initialised && numTrackedIds <= 10).

On reset:
- LOGW with the long message;
- LOGW "Reset vo reset" for the last case;
- `set_reset_ = true`;
- zero 632, 636, 640, 652, 656, +16, 660, 664, 688, 668, 680 and 684;
- dark, bright, low_marks and low_c are **not** reset.

### setInitialPose (0x1801277B0)
The no-IMU path runs when the bundle has 10 or fewer IMU samples (`imu_measurements_` deque at FrameBundle+24; size at +56): LOGW "...identity". In that case, for each frame, `T_f_w_ = T_C_B(i) * T_world_imuinit.inverse()`, followed by `normalize`.

With more than 10 samples, LOGI "Use inertial measurements...":
1. `z = back().linear_acceleration_.cast<double>().normalized()`.
2. `y = z.cross((0,0,1)).normalized()`, then `x = y.cross(z)`, and `C = [x y z]`.
3. If `|det(C) - 1| > 1e-5` or `!C.col(0).isApprox(C.col(1).cross(C.col(2)), 1e-12)`, LOGW "C_imu_world is not orthogonal matrix." and fall back to the identity formula.
4. Otherwise `q = Quaterniond(C)`. If `use_att`, `q = att.cast<double>()`, and when `attitude_history_ (+3520) > 2` entries, `q` is the slerp (0x180115430) of the lower_bound neighbours, renormalised.
5. For each frame: `T_f_w_ = T_C_B(i) * Transformation(q, 0)`, followed by `normalize`.

## Types

### FrameProcessorBase
The full layout is in `draft/.../frame_processor_base.h`. The offsets below are certain; they come from direct accesses in c10.

| offset | type | name | evidence |
|---|---|---|---|
| 0 | vptr | | vtable 0x1803B2E70 |
| 16 | size_t | num_tracked_last_ | watchdog `<20`; reset to 0 in resets; set in addFrameBundle |
| 24 | BaseOptions (256 B) | options_ | ctor copy. Fields used: 168 poseoptim_thresh, 176 poseoptim_prior_lambda, 188 structure_optimization_max_pts, 192 trace_dir, 224 quality_min_fts, 232 quality_max_fts_drop (int), 250 use_async_reprojectors, 251 trace_statistics, 264 global_map_lc_timeout_sec_, 272 uint16 |
| 280 | shared_ptr<NCamera> | cams_ | numCameras = (+56 - +48) >> 4; `get_T_C_B` 0x1801B41F0 |
| 296 / 312 / 328 / 344 | FrameBundlePtr | new_frames_ / last_frames_ / last_last_frames_ / last_kf_frames_ | addFrameBundle rotates 312 into 328 and 296 into 312; init sets 344 = 296 |
| 360 | cv::Ptr<CLAHE> | clahe_ | ctor |
| 376 | Vector3d | t_lastimu_newimu_ | resets; addFrameBundle |
| 400 | Transformation | T_world_imuinit | setInitialPose |
| 464 | uint8 | slam_mode_ (guessed name) | reLocalize `!= 2`; ctor `*a10`; 0 means load prior |
| 472 | vector<unique_ptr<Reprojector>> | reprojectors_ | |
| 496 / 504 / 512 | unique_ptr | pose_optimizer_ / depth_filter_ / initializer_ | reset calls |
| 520 | shared_ptr<ImuProcessor> | imu_handler_ | bias writes +208/+232 |
| 536 / 548 | Vector3f | init_gyr_bias_ / init_acc_bias_ | resets |
| 632..688 | int counters + bool 684 | watchdog | see above |
| 1624 | shared_ptr<LoopClosing> | lc_ | reLocalize |
| 1680 | vector<vector<PointPtr>> | trash_points_ | |
| 1704 / 1728 | vector<set<int>> | track_ids_last_ / track_ids_last_last_ | |
| 2784 | deque<ImuMeasurement> | imu_window_ | resetBackend; addFrameBundle stats |
| 2824 | 3 x {size_t n; double sum; double sum_sq} | gyr_stat_ | resetBackend zeroes 9 qwords |
| 2912 | Transformation | T_world_correction_ | reLocalize; dtor |
| 2976 | int | stage_ | |
| 2980 | bool | set_reset_ | |
| 2984 | shared_ptr<Map> | map_ | (Map is 0xB8 bytes) |
| 3000 | size_t | num_obs_last_ | |
| 3008 | int | tracking_quality_ | |
| 3012 | int | update_res_ | (c09) |
| 3016 | size_t | frame_counter_ | (c09) |
| 3032 / 3056 | vector<vector<FramePtr>> | overlap_kfs_ / last_overlap_kfs_ | |
| 3080 | vector<bool> | occupied_cells_ (guessed name) | ctor |
| 3112 | int | exposure_level_ (guessed name) | watchdog `>75` |
| 3116 | int | frame_flag234_cnt_ | projectMapInFrame `>6` |
| 3120 | bool | have_rotation_prior_ | |
| 3136 / 3168 | Eigen::Quaterniond | R_imu_world_ / R_imulast_world_ | rotation priors |
| 3200 / 3216 | bool / Transformation | prior_position_loaded_ / T_prior_ | ctor/loadPriorPosition |
| 3336 / 3344 | bool / Transformation | have_motion_prior_ / T_newimu_lastimu_prior_ | |
| 3408 / 3424 | shared_ptr<CeresBackendInterface> / int | bundle_adjustment_ / bundle_adjustment_type_ | |
| 3432 / 3440 / 3441 / 3448 | double / bool / bool / double | last_kf_time_sec_ / global_map_has_initial_ba_ / loss_without_correction_ / last_good_tracking_time_sec_ | ctor -1 / 0 / 0 / -1 |
| 3456 | bool | imu_not_initialized_ (guessed name; ctor 1, "imu_initial true" sets 0) | |
| 3464 | map<8-byte key, FrameBundlePtr> | kf_bundles_ | projectMapInFrame |
| 3480 | map<?, 448-byte value> | map_3480_ | resets |
| 3496 | vector<shared_ptr<?>> | vec_3496_ | resets |
| 3520 | map<double, Quaternionf> | attitude_history_ | setInitialPose; addFrameBundle keeps 10 entries |
| 3536 | int | unknown_3536_ | resets |
| 3584 | float | ground_height_ | watchdog |
| 3592 | shared_ptr<Mesher> | mesher_ | ctor |
| 3608 / 3672 / 3696 | unordered_map / vector<232-byte plane> x 2 | plane state | resetVisionFrontendCommon |
| 3720..3912 | ground-plane state | gp_* | resetVisionFrontendCommon values (see header) |
| 3920..4016 | reloc state | | reLocalize |

- Known size: at least 4056 bytes (FrameProcessor uses +4048/+4052).
- Layout differences from upstream FrameHandlerBase:
  - There is no `CallbackHost` base.
  - `need_new_kf_`, `sparse_img_align_`, `reloc_keyframe_` and `relocalization_n_trials_` are gone.
  - All the Pimax state listed above is new.

### Frame offsets used
- +16 `id_`
- +32 bundle id
- +64 `T_f_w_`
- +128 `img_pyr_`
- +176 `is_keyframe_`
- +224 `img_mean_` (double brightness)
- +234 bool flag
- +240 `timestamp_` ns
- +256 `T_body_cam_`
- +552 `num_features_`
- +560 `px_vec_` (2xN float)
- +576 `f_vec_` (3xN float)
- +624 `level_vec_` (VectorXi)
- +640 `grad_vec_`
- +656 `type_vec_`
- +680 `landmark_vec_`
- +704 `track_id_vec_` (VectorXi)
- +720 `seed_ref_vec_` (24 B)
- +744 `invmu_sigma2_a_b_vec_` (4xN float)

### FrameBundle offsets used (size 0x100)
- +0 `frames_`
- +24 `imu_measurements_` (deque<ImuMeasurement>)
- +64 `is_relocalized_` (guessed name)
- +144 `imu_vel_w_`
- +168 `imu_gyr_bias_`
- +192 `imu_acc_bias_`
- +228 `is_static_` (= the IMU-stationary result in 0x1800FB650)
- +229 `force_stereo_triangulation_`
- +232 size_t n_total_features (written by FrameProcessor)
- +248 `is_keyframe_`
- +252 `bundle_id_` (int)

### Point offsets used (size 136)
- +0 `id_`
- +4 `pos_` (Vector3f)
- +16 `obs_` (unordered_map<int, KeypointIdentifier>; `.size()` at +32)
- +96 set<int>
- +120 `n_consecutive_tracked_` (int)
- +124 last tracked bundle id (-1)
- +128 `in_ba_graph_`

The ctor is 0x180099F80; the id counter is the global 0x18047DB60.

### ImuProcessor (680 bytes)
See `imu_processor.h`:
- calib +0 (112)
- init +112 (96)
- acc_bias_ +208
- omega_bias_ +232
- mutex +256
- measurements_ +336
- temporal_imu_window_ +376
- ofstream +416

`ImuMeasurement` is 32 bytes: `{double ts; Vector3f gyr; Vector3f acc}`.

### IntegrationBase (offsets in `integration_base_inl.h`)
- +256 jacobian 15x15
- +2056 covariance 15x15
- +10416 sum_dt
- +10424 delta_p
- +10448 delta_q
- +10480 delta_v
- dt/acc/gyr buffers at +10504/+10528/+10552

### Other structs and fields
- `ReprojectResult` (72 B) and `ReLocCorrectionInfo` (T at +32, -1,-1,0 at +96..+111): see the header.
- Reprojector:
  - `options_.max_n_kfs` at +32
  - `stats_` at +96: {n_matches, n_trials, +112, +120, +128, +136}
  - `fixed_lm_stats_.n_matches` at +144
- PoseOptimizer: reset fields +832..+888 and +464; stats_ +896 (after) and +904 (before).
- LoopClosing:
  - +16 reloc kf id
  - +24 plat map pointer (map size at +96)
  - +232 `reLoc_correction_info_` deque
  - +953 enable flag

## External interfaces (called from c10)

| address | meaning |
|---|---|
| 0x18000C120 / 0x18000F500 / 0x18000F6A0 / 0x18000C2C0 | LOGD / LOGI / LOGW / LOGE (logger at 0x18046A000) |
| env_8 | glog `FLAGS_v` (VLOG level). 0x180354E20 is `LogMessage(file,line)`; 0x180354E50 is `LogMessage(file,line,sev)`; 0x18035AC70 is `stream()`; 0x1803551B0 is `~LogMessage` |
| 0x1801B56D0 | `vk::PerformanceMonitor::startTimer(const std::string&)`; g_permon at 0x18047DDA0 (ctrl 0x18047DDA8) |
| 0x18013F580 | `PoseOptimizer::setRotationPrior(const Eigen::Quaterniond&, double lambda)` |
| 0x18013ED70 | `size_t PoseOptimizer::run(const FrameBundlePtr&, double thresh, bool)` |
| 0x18009B240 | `Point::optimize(size_t n_iter, bool on_sphere)` |
| 0x18009A530 | `Point::addObservation(const FramePtr&, size_t)` ("addObservation frame NULL") |
| 0x180099F80 | `Point::Point(const Eigen::Vector3f&)` |
| 0x18012D900 | `Map::getClosestNKeyframesWithOverlap(const FramePtr&, const size_t&, std::vector<FramePtr>*)` |
| 0x18012EDE0 | `Map::safeDeletePoint(const PointPtr&)` |
| 0x18012ED20 | `Map::reset()` |
| 0x18012D680 | `Map::addKeyframe(const FramePtr&)` |
| 0x180143380 | `Reprojector::reprojectFrames(const FramePtr&, std::vector<FramePtr>&, std::vector<PointPtr>& trash, const bool& imu_not_initialized)` |
| 0x1800E61D0 | async reprojection lambda (source is in this chunk's draft) |
| 0x1800CB4B0 | `std::set<PointPtr> collectTrashPoints(bool is_static, const std::vector<std::vector<PointPtr>>&)` (static helper, c09) |
| 0x1800FDEE0 | `computePoseDifference(double* dist, double* angle_deg, const Transformation&, const Transformation&)` (distance in float, angle via acos) |
| 0x180115430 | `Eigen::Quaternionf interpolateQuaternion(qa, qb, double ta, double tb, double t)` |
| 0x18018B600 | `bool LoopClosing::reLocalize(ReLocCorrectionInfo*)` |
| 0x180186A50 | `LoopClosing::addReLocFrames(const FrameBundlePtr&, const Transformation&)` (guessed name; "INFO=ReLoc plat map not be ready.") |
| 0x18002CB90 / 0x18002CC00 | `vk::Timer::start` / `stop` (steady_clock) |
| 0x180118580 | `size_t Frame::numTrackedIds() const` (count `track_id > -1`) |
| 0x180095180 / 0x180095240 / 0x1800952D0 / 0x1800953F0 | `FrameBundle::numTrackedIds` / `numLandmarksInBA` / `numTrackedLandmarks` / `numLandmarks` |
| 0x1800149F0 + 0x180012B20 | `CeresBackendInterface::reset()` ("Backend: Reset") |
| 0x1800298A0 | reset of the backend's ground-plane state on (ba+176) |
| 0x1800A2300 | `DepthFilter::reset()` |
| initializer vtable+16 | `AbstractInitialization::reset()` |
| 0x180104140 | `IntegrationBase::midPointIntegration(...)` |
| 0x180096E80 | `Frame::setKeyframe()` |
| 0x1800928D0 | `FrameBundle::FrameBundle(const std::vector<FramePtr>&)` (aligned new 0x100) |
| 0x1800AD4A0 | `float feature_detection_utils::getAngleAtPixelUsingHistogram(const cv::Mat&, const Eigen::Vector2i&, size_t)` |
| 0x1801B41F0 | `CameraBundle::get_T_C_B(size_t)` (64-B Transformations, vector at NCamera+0) |
| 0x180009B80 / 0x180013040 / 0x180014A20 / 0x180009C80 / 0x1800089C0 / 0x180014680 / 0x180009B30 | minkindr `Transformation::operator*` (renormalises) / `inverse()` (renormalises) / `RotationQuaternion::rotate` / `RotationQuaternion::operator*` / `RotationQuaternion(Quaterniond)` with unit-norm CHECK / `normalize()` / copy-assign |
| 0x18008A270 / 0x1800932D0 | `QuatTransformation<double>::cast<float>()` / `QuatTransformation<float>::transform(Vector3f)` |
| 0x1800DB8D0, 0x1800DC8A0, 0x1800DBD40 | out-of-line serialize for Transformation (binary/portable) and KeyFrame |

Globals:
- 0x18046A000: logger.
- 0x18047DDA0/8: g_permon.
- 0x18047DB60: Point id counter.
- 0x18047ECE0..: `imu_temporal_status_names_`.

## Constants
- Watchdog:
  - brightness 15.0 / 220.0, initial max/min 0.0 / 255.0
  - 165000000 ns
  - num_tracked_last_ < 20
  - speed 4.5 (both bundle velocity and pose speed), +1e-6 s in the denominator
  - thresholds 10/12/5/6/75/10/6/6/6/10/6/10, delt_z 5.0
  - velocity 0.1 and 1.0
  - common points 30
  - landmark ratio 0.8
  - numTrackedIds 10
  - landmarks in BA 10
  - empty frames 3
- reLocalize: brightness 25.0, tracked ids 30, timestamps scaled by 1e-9. Defaults (ctor): success interval 2.0, try interval 0.01, max_reLoc_times_ 300.
- setInitialPose: >10 IMU samples, det tolerance 1e-5, isApprox precision 1e-12 (squared 1e-24), reference axis (0,0,1), attitude history > 2 entries.
- setTrackingQuality: bundle-id distance > 4.
- upgradeSeedsToFeatures: ratio 0.2, 100.
- repropagate: dt < 1e-4 is skipped.
- pose optimizer reset: chi2 1e10.
- Ground plane reset values:
  - gp_first_ = true
  - plane (0,0,1,0)
  - 0.08
  - normal (0,0,1) twice
  - 0.15
  - id -1
- Backend ground plane reset (0x1800298A0): +616=0, +620 = -1,-1, (0,0,1), (0,0,1), 0.15.

## Quirks to preserve
1. `optimizePose` prints the size_t `nObs` with `%f`.
2. `projectMapInFrame` never stops the "reproject" timer.
3. `projectMapInFrame` dereferences `last_kf_frames_` without a null check, and `setTrackingQuality` does the same.
4. Trash points are passed to `safeDeletePoint` only for static bundles. Otherwise they are just cleared, so the points stay in the map.
5. The async lambda dedupes overlap KFs through an unordered_set, then sorts them by id descending; the synchronous path does neither.
6. `optimizeStructure*` ignores the `max_n_pts` value except to return when it is 0. The unordered_set spans all frames of the bundle.
7. In `setInitialPose`, the attitude interpolation may dereference `attitude_history_.end()` when every stored timestamp is older than the frame. MSVC then reads the head node's uninitialised key and value.
8. The watchdog reset leaves dark_t_, bright_t_, low_marks_ and low_c_ untouched. "acc_b_n" is always 0.0.
9. The resets do not touch `last_last_frames_`, `last_kf_frames_` or the relocalisation state.
10. `Frame::getTimestampSec()` must be `timestamp_ * 1e-9`; the binary multiplies, while upstream divides by 1e9.
11. The Pimax minkindr `operator*` and `inverse()` renormalise the result quaternion unconditionally, after kindr's own normalisation helper.
12. `IntegrationBase::repropagate` skips buffered samples with dt < 1e-4.
13. `addImuMeasurement` clears the whole buffer when time goes backwards.
14. "Lost %lu  features!" is printed with an int argument and two spaces.
15. In `setRotationIncrementPrior` and `optimizePose`, quaternion inversion is Eigen's, which divides by the squared norm, not kindr's conjugate.

## Open questions and TODO(verify)
- Several field names are guessed:
  - `slam_mode_` (+464)
  - `exposure_level_` (+3112)
  - `frame_flag234_cnt_` (+3116; what Frame+234 means)
  - `imu_not_initialized_` (+3456)
  - `img_mean_` (Frame+224)
  - `is_static_` (FrameBundle+228)
  - `kf_bundles_` key type
  - map_3480_ value type
  - vec_3496_ element type
  - the ground-plane field names
  - Reprojector stats fields +112..+136
  - `ReprojectResult` fields
  - LoopClosing members
- The +536..+632 block (7 x Vector3f + 8 bytes): only the first two vectors are identified.
- `checkTrackingHealth` is a guessed function name.
- `BundleAdjustmentType` enum values: the binary only tests `!= 0`.
