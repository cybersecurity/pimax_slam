// pimax_slam.pi.dll -- src/loop_closing/loop_closing.h  (drafts c16 + c15; c16 supersedes
// draft/c15_platmap/loop_closing/loop_closing_c15.h where they disagree)
//
// pimax::totem::LoopClosing.
//
// Very loosely derived from rpg_svo_pro_open/svo_online_loopclosing/loop_closing.h: the
// upstream member block +1568..+2152 and the "private" block +2768..+3048 are still there
// (same order), but loop detection itself is gone; the class now does
//   * keyframe database building (addFrameToPR / runPROnLatestKeyframe),
//   * PlatMap persistence (load/save/index/tag files) on a dedicated thread,
//   * relocalization against the PlatMap (DBoW2 query -> BFMatcher -> stereo triangulation ->
//     solvePnPRansac -> direct epipolar refinement -> ceres pose refinement) on its own thread.
//
// Layout: every offset below was observed in this chunk (functions 0x180185e40..0x180196b60)
// or in the ctor 0x18017f730 / dtor 0x180181ae0 (chunk c15).  (sure) = used with a known
// type in this chunk.  This header supersedes draft/c15_platmap/loop_closing/loop_closing_c15.h
// where they disagree (see notes/c16_loop_closing.md, "Types").
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <opencv2/aruco.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "common/camera_fwd.h"
#include "common/transformation.h"
#include "common/types.h"
#include "loop_closing/bow.h"            // OrbVocabulary, extractBoWFeaturesFromImage, createBOW, ...
#include "loop_closing/map_alignment.h"
#include "loop_closing/platmap.h"        // PlatMap, KeyFrame, MapIndex

class BEBLID;   // loop_closing/beblid.h (global namespace, outside this chunk)

namespace pimax {
namespace totem {

enum class LCScaleRetMethod { kCommonLandmarks = 0, kMixedKeyPoints = 1, kNone = 2 };
enum class GlobalMapType { kBuiltInPoseGraph = 0, kExternalGlobalMap = 1, kNone = 2 };
/// LoopClosing ctor argument (c14: the C API's loc_mode, a uint8_t).
enum class LoopClosingMode : uint8_t { kNormal = 0, kLabMap = 1, kLabLoc = 2 };

/// 0x18047EEE0 {"CommonLM","MixedKP","None"} / 0x18047EE90 {"BuiltInPoseGraph",
/// "ExternalGlobalMap","None"}; defined in loop_closing.cpp (draft c00 loop_closing_globals.cpp;
/// non-const there).
extern std::unordered_map<std::string, LCScaleRetMethod> kStrToScaleRetMap;
extern std::unordered_map<std::string, GlobalMapType> kStrToGlobalMapType;

// ---------------------------------------------------------------------------------------------
// LoopClosureOptions (656 bytes, at LoopClosing+560).  Defaults: 0x180159e10 (default ctor) and
// 0x18015db90 (factory, overrides marked "factory").  Names of the Pimax fields are ours, given
// from their use in this chunk (offset = option offset; LoopClosing offset = +560).
// ---------------------------------------------------------------------------------------------
struct LoopClosureOptions
{
  bool runlc;                                   // +0   factory: true unless voc name mismatch
  std::string voc_name;                         // +8   factory: "voc_GEN_8X4.dbow"
  std::string voc_path;                         // +40
  double alpha;                                 // +72  factory 1.0
  double beta;                                  // +80  factory 1.0  (LC+640: commonLandMarkCheck thresh)
  int ignored_past_frames;                      // +88  factory 15   (LC+648: passed to runPROnLatestKeyframe)
  std::string scale_ret_app;                    // +96  factory "None"
  double bowthresh;                             // +128 factory 0.65
  double gv_3d_inlier_thresh;                   // +136 factory 0.4
  int min_num_3d;                               // +144 factory 10
  int orb_dist_thresh;                          // +148 factory 48
  double gv_2d_match_thresh;                    // +152 factory 0.1
  bool use_opengv;                              // +160 factory false
  bool enable_image_logging;                    // +161 factory false (LC+721)
  std::string image_log_base_path;              // +168 factory "/home/cc/tmp/img_dir/" (LC+728)
  double proximity_dist_ratio = 0.01;           // +200 (LC+760)
  double proximity_offset = 0.3;                // +208 factory 0.2 (LC+768)
  std::string global_map_type;                  // +216
  double force_correction_dist_thresh_meter = 0.1;  // +248 factory 0.01
  // ---- Pimax additions ----
  size_t min_bow_features = 20;                 // +256 (LC+816) LC-2 "bow features is less", [Reloc-2] kp check
  double reloc_expected_score = 0.6;            // +264 (LC+824) only printed ("param_expected_score")
  double key_pos_resolution = 0.15;             // +272 (LC+832) getPoseKey position cell [m]
  double key_ang_resolution = 15.0;             // +280 (LC+840) getPoseKey angle cell [deg]
  size_t min_num_features = 40;                 // +288 (LC+848) addFrameToPR: frame->num_features_
  size_t max_kf_num = 1500;                     // +296 (LC+856) kf_list_ cap; x5 in lab modes
  int unk_304 = 6;                              // +304 TODO(verify) not used in c16
  double unk_312 = 0.5;                         // +312
  size_t unk_320 = 3;                           // +320
  size_t unk_328 = 18;                          // +328
  int unk_336 = 20;                             // +336
  int unk_340 = 1;                              // +340
  float unk_344 = 2.0f;                         // +344
  double unk_352 = 0.99;                        // +352
  double unk_360 = 0.1;                         // +360
  double unk_368 = 0.75;                        // +368
  double unk_376 = 0.04;                        // +376
  double unk_384 = 1.0;                         // +384
  bool unk_392 = false;                         // +392
  bool use_plat_map = true;                     // +393 (LC+953)
  bool skip_loop_detection = true;              // +394 (LC+954) runPROnLatestKeyframe early exit
  size_t unk_400 = 30;                          // +400
  float reloc_min_score = 0.5f;                 // +408 (LC+968) only printed ("param_min_score")
  int reloc_max_candidates = 20;                // +412 (LC+972) cap on re-ranked candidates
  size_t reloc_min_bow_keypoints = 30;          // +416 (LC+976) per candidate KF
  double unk_424 = 0.2;                         // +424
  size_t unk_432 = 10;                          // +432
  int unk_440 = 48;                             // +440
  double unk_448 = 0.09;                        // +448
  double unk_456 = 0.12;                        // +456
  int unk_464 = 20;                             // +464
  int unk_468 = 1;                              // +468
  float unk_472 = 3.0f;                         // +472
  double unk_480 = 0.9;                         // +480
  size_t unk_488 = 8;                           // +488
  double unk_496 = 0.5;                         // +496
  bool keep_reloc_images = false;               // +504 (LC+1064) if false images are released
  std::string map_path;                         // +512 (LC+1072) factory: <output dir> + '/'
  size_t min_kf_to_save_map = 50;               // +544 (LC+1104) resetReLocalize
  size_t max_num_maps = 2;                      // +552 (LC+1112) resetReLocalize (oldest map deleted)
  std::string map_name = "platMap_orborb_K8L4.bin";        // +560 (LC+1120)
  std::string map_index_name = "platMap_orborb_K8L4.yaml"; // +592 (LC+1152)
  std::string tag_file_name = "tag.yaml";       // +624 (LC+1184) not used in c16
};
static_assert(sizeof(LoopClosureOptions) == 656, "sizeof(LoopClosureOptions)");
static_assert(offsetof(LoopClosureOptions, min_bow_features) == 256, "");
static_assert(offsetof(LoopClosureOptions, max_kf_num) == 296, "");
static_assert(offsetof(LoopClosureOptions, use_plat_map) == 393, "");
static_assert(offsetof(LoopClosureOptions, reloc_min_score) == 408, "");
static_assert(offsetof(LoopClosureOptions, keep_reloc_images) == 504, "");
static_assert(offsetof(LoopClosureOptions, map_path) == 512, "");
static_assert(offsetof(LoopClosureOptions, min_kf_to_save_map) == 544, "");
static_assert(offsetof(LoopClosureOptions, map_name) == 560, "");
static_assert(offsetof(LoopClosureOptions, tag_file_name) == 624, "");

// ---------------------------------------------------------------------------------------------
// Element of lc_correction_info_ (upstream svo_online_loopclosing loop_closing_types.h). Only read by
// FrameProcessorBase::addFrameBundle 0x1800FA6B0 (one element per deque block, w_T_new_old_ copied
// from block +16); nothing in this binary pushes to the deque any more.
// ---------------------------------------------------------------------------------------------
struct LoopCorrectionInfo
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int lc_kf_bundle_id_ = -1;        // +0
  int cur_kf_bundle_id_ = -1;       // +4
  Transformation w_T_new_old_;      // +16
};
static_assert(sizeof(LoopCorrectionInfo) == 80, "");
static_assert(offsetof(LoopCorrectionInfo, w_T_new_old_) == 16, "");

// ---------------------------------------------------------------------------------------------
// Element of reLoc_correction_info_ (std::deque, one 112-byte element per deque block).
// Built in place by emplace_back(int&, double&, float&, Transformation&) (0x18017e200),
// copied whole by 0x180182450 (operator=).  Matches the caller-side struct of chunk c10
// (+0 is the map id, written into LoopClosing::map_id_ by the frontend on success).
// ---------------------------------------------------------------------------------------------
struct ReLocCorrectionInfo
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int map_id_ = 0;                  // +0
  double timestamp_ = 0.0;          // +8   KeyFrame::timestamp_sec_abs_ of the current frame
  float confidence_ = 0.f;          // +16  mean squared epipolar-match distance (smaller = better)
  Transformation w_T_new_old_;      // +32  correction (map <- odometry), yaw-only rotation
  int lc_kf_bundle_id_ = -1;        // +96  (upstream LoopCorrectionInfo heritage) TODO(verify)
  int cur_kf_bundle_id_ = -1;       // +100
  int64_t unk_104_ = 0;             // +104
  ReLocCorrectionInfo() = default;
  ReLocCorrectionInfo(const int map_id, const double timestamp, const float confidence,
                      const Transformation& T)
    : map_id_(map_id), timestamp_(timestamp), confidence_(confidence), w_T_new_old_(T) {}
};
static_assert(sizeof(ReLocCorrectionInfo) == 112, "");
static_assert(offsetof(ReLocCorrectionInfo, w_T_new_old_) == 32, "");
static_assert(offsetof(ReLocCorrectionInfo, lc_kf_bundle_id_) == 96, "");

// ---------------------------------------------------------------------------------------------
// Fiducial/tag index (120 bytes), two instances: tag_save_ (+1328, written by saveTagIndex, filled
// by code outside c16) and tag_load_ (+1448, filled by loadTagIndex).  The 112-byte prefix has the
// dtor 0x1801819d0 (c15 "LcStruct112").
// ---------------------------------------------------------------------------------------------
struct TagIndex
{
  bool valid = false;                                   // +0  (+1328 / +1448)
  std::vector<Eigen::Vector3d> T;                       // +8  24-byte elements ("T": 3 doubles)
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> Q;  // +32 ("Q": 4 doubles)
  std::vector<int> board_idx;                           // +56
  std::string name;                                     // +80 TODO(verify) never touched in c16
  int frame_id_;                                        // +112 (+1440 / +1560)
  int map_id_;                                          // +116 (+1444 / +1564)
};
static_assert(sizeof(TagIndex) == 120, "");

using LoopVizInfo = Eigen::Matrix<float, 1, 6>;
using LoopVizInfoVec = std::vector<LoopVizInfo, Eigen::aligned_allocator<LoopVizInfo>>;

// ---------------------------------------------------------------------------------------------
// LoopClosing -- sizeof 0xC10 (3088).  vtable 0x1803be300 (slot 0 = scalar deleting dtor).
// ---------------------------------------------------------------------------------------------
class LoopClosing
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 0x18017f730 (chunk c15)
  LoopClosing(const LoopClosureOptions& loopclosure_options, const CameraBundlePtr& cams,
              const std::string& map_tag, LoopClosingMode mode);
  // 0x180181ae0 (chunk c15): calls stopAllThread() first.
  virtual ~LoopClosing();

  // ---- frontend entry points -------------------------------------------------------------
  void addFrameToPR(const FrameBundlePtr& last_frames, const Transformation& T_w_odom);   // 0x180185e40
  void reLocalize(const FrameBundlePtr& new_frames, const Transformation& T_w_odom);      // 0x180186a50
  bool getReLocCorrection(ReLocCorrectionInfo* info);                                    // 0x18018b600
  // upstream inline helpers, used by FrameProcessorBase::addFrameBundle under lc_info_lock_
  bool hasCorrectionInfo() const { return !lc_correction_info_.empty(); }
  void consumeOldestCorrection(Transformation* w_T_correction)
  {
    *w_T_correction = lc_correction_info_.front().w_T_new_old_;
    lc_correction_info_.pop_front();
  }
  void computePoseDiff(double* dist, double* angle_deg, const Transformation& T_1,
                       const Transformation& T_2);                                       // 0x180186f30
  void resetLoopClosing();                                                               // 0x18018c3a0
  void resetReLocalize();                                                                // 0x18018c4e0
  void startAllThread();                                                                 // 0x180195b70
  void stopAllThread();                                                                  // 0x180195e60

  // ---- PlatMap persistence ---------------------------------------------------------------
  void load();                                     // 0x180188900
  bool loadIndex();                                // 0x180188ce0
  bool loadTagIndex(const std::string& path);      // 0x1801891b0
  void save();                                     // 0x180193e00
  bool saveIndex();                                // 0x180194020
  bool saveTagIndex(const std::string& path);      // 0x180194200

  // ---- workers / helpers -------------------------------------------------------------------
  void loopClosingThread();                        // 0x180188830
  void reLocalizeThread();                         // 0x18018b3d0
  void loadSavePlatMapThread();                    // 0x1801890e0
  void runPROnLatestKeyframe(const size_t n_ignored_latest,
                             const bool run_lc_on_this_frame);   // 0x18018c850
  void runReLocalization();                        // 0x18018d770 (the 26 KB function)
  bool bundleAdjustKfList(std::vector<std::vector<KeyFramePtr>>& kf_list);  // 0x1801899c0
  void clearReLocFrames();                         // 0x1801872e0
  void backProject(Eigen::Matrix2Xd px, CameraPtr cam, Eigen::Matrix3Xf* f);   // 0x180187370
  bool triangulate(const Eigen::Vector3f& f0, const Eigen::Vector3f& f1, CameraPtr cam0,
                   CameraPtr cam1, const Transformation& T_c1_c0,
                   const Eigen::Matrix<double, 3, 4>& P0, const Eigen::Matrix<double, 3, 4>& P1,
                   Eigen::Vector3d* p_c0);         // 0x1801963d0
  std::string getPoseKey(const Transformation& T); // 0x180187e30
  void svoFrameToKeyframe(const FramePtr& frame, KeyFrame* kf, const Transformation& T_w_odom,
                          const bool use_all_features);         // 0x180187c90
  void updateSVOPointsDescriptors(const KeyFramePtr& kf, const bool replace_mixed_features); // 0x180196b60
  void extractAndConvert(const FramePtr& frame, double* timestamp_sec, Transformation* Twc,
                         std::vector<cv::Point2f>* keypoints,
                         std::vector<cv::Point3f>* landmarks_in_cam,
                         std::vector<int>* landmark_ids, std::vector<int>* track_ids,
                         const Transformation& T_w_odom, const bool use_all_features);  // 0x180187830

  /// inlined into FrameProcessorBase::setRecovery (c09): stores `recovery` into +1928 and +1929
  /// (one 16-bit store).  TODO(verify) name (upstream setRecoveryMode).
  inline void setRecoveryMode(const bool recovery)
  {
    ignore_next_constraint_in_pg_ = recovery;
    recovery_after_loss_ = recovery;
  }

  // ---- layout ------------------------------------------------------------------------------
  // +0 vptr; +8 is MSVC's padding of the vfptr slot (class alignment 16), not a member
  // (c15/c16 "unk_8_").
  int map_id_ = 0;                                   // +16   (sure) id of the map being built
  bool save_map_enabled_ = true;                     // +20   (sure) false in kLabLoc
  PlatMapPtr plat_map_;                              // +24   (sure)
  std::mutex reloc_frames_mutex_;                    // +40   (sure) guards reloc_kf_list_
  std::mutex reloc_info_lock_;                       // +120  (sure) guards reLoc_correction_info_
  std::vector<KeyFramePtr> reloc_kf_list_;           // +200  (sure) KFs queued for relocalization
  bool reloc_success_ = false;                       // +224  (sure)
  std::deque<ReLocCorrectionInfo> reLoc_correction_info_;  // +232 (sure, name from frontend log)
  int i272_ = 10;                                    // +272
  bool b276_ = false;                                // +276
  Transformation T_C_B_;                             // +288  (upstream)
  Transformation T_B_C_;                             // +352  (upstream)
  cv::Mat mask_;                                     // +416
  cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;  // +512
  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;        // +528
  CameraBundlePtr cams_;                             // +544  (sure)
  LoopClosureOptions options_;                       // +560 .. +1216
  LoopClosingMode mode_;                             // +1216 (sure)
  std::string map_tag_;                              // +1224 (sure) sub directory / name prefix
  std::vector<int64_t> vec_1256_;                    // +1256 TODO(verify)
  std::deque<std::vector<KeyFramePtr>> deque_1280_;  // +1280 TODO(verify)
  size_t sz_1320_ = 3;                               // +1320
  TagIndex tag_save_;                                // +1328 (sure for valid/T/Q/board_idx/ids)
  TagIndex tag_load_;                                // +1448 (sure)
  // ---- upstream block (unused by c16 unless noted) ----
  std::mutex lc_info_lock_;                          // +1568
  std::deque<LoopCorrectionInfo> lc_correction_info_;   // +1648
  std::deque<std::shared_ptr<void>> lc_matched_points_info_;  // +1688 TODO(verify) element type
  std::map<int, int, std::less<int>, Eigen::aligned_allocator<std::pair<const int, int>>>
      cur_kf_to_lc_kf_bundle_id_map_;                // +1728
  std::vector<std::array<double, 12>> lc_closed_loops_;  // +1744 96-byte elements (= ClosedLoop)
  std::mutex kf_list_mutex_;                         // +1768 (sure)
  std::vector<std::vector<KeyFramePtr>> kf_list_;    // +1848 (sure) one inner vector per bundle
  std::vector<KeyFramePtr> last_reloc_kfs_;          // +1872 (sure: assigned from the reloc KFs)
  std::vector<int> last_run_lc_frame_trackIDs_;      // +1896 (sure)
  bool suspend_lc_after_correction_ = false;         // +1920
  int suspended_frames_counter_ = 0;                 // +1924
  bool ignore_next_constraint_in_pg_ = false;        // +1928 (setRecoveryMode, chunk c09)
  bool recovery_after_loss_ = false;                 // +1929
  std::shared_ptr<MapAlignmentSE3> map_alignment_se3_;   // +1936 (make_shared in 0x18015cee0)
  bool need_to_update_pose_graph_viz_ = false;       // +1952
  LoopVizInfoVec cur_loop_check_viz_info_;           // +1960 (sure: cleared in addFrameToPR)
  LoopVizInfoVec loop_detect_viz_info_;              // +1984
  LoopVizInfoVec loop_correction_viz_info_;          // +2008
  std::vector<double> bow_timing_;                   // +2032
  std::vector<double> gv_timing_;                    // +2056
  std::vector<double> hm_timing_;                    // +2080
  std::vector<double> transformmap_timing_;          // +2104
  std::vector<int> num_queries_;                     // +2128
  // ---- Pimax worker threads ----
  std::unique_ptr<std::thread> lc_thread_;           // +2152 (sure) runs loopClosingThread
  std::condition_variable lc_cond_;                  // +2160 (sure)
  std::atomic<bool> lc_stop_{false};                 // +2232 (sure, xchg store)
  std::mutex lc_mutex_;                              // +2240 (sure)
  bool run_lc_on_this_frame_ = false;                // +2320 (sure)
  std::unique_ptr<std::thread> reloc_thread_;        // +2328 (sure) runs reLocalizeThread
  std::condition_variable reloc_cond_;               // +2336 (sure)
  std::atomic<bool> reloc_stop_{false};              // +2408 (sure)
  std::mutex reloc_mutex_;                           // +2416 (sure)
  std::atomic<int> platmap_cmd_{0};                  // +2496 (sure) 0 = load, 1 = save, 2 = save+exit
  std::unique_ptr<std::thread> load_save_thread_;    // +2504 (sure) runs loadSavePlatMapThread
  std::condition_variable platmap_cond_;             // +2512 (sure)
  std::atomic<bool> platmap_stop_{false};            // +2584 (sure)
  std::mutex platmap_mutex_;                         // +2592 (sure)
  std::unordered_map<std::string, bool> pose_key_map_;   // +2672 (sure) node 0x38
  cv::Ptr<cv::ORB> orb_;                             // +2736
  std::shared_ptr<BEBLID> beblid_;                   // +2752
  // ---- upstream "private" block ----
  size_t lc_frame_count_ = 0;                        // +2768 (sure)
  size_t svo_keyframe_count_ = 0;                    // +2776 (sure)
  OrbVocabulary voc_;                                // +2784 (sure)
  std::vector<cv::Mat> K_;                           // +2864
  std::vector<Eigen::VectorXd> D_;                   // +2888
  LCScaleRetMethod scale_retrieval_approach_;        // +2912
  std::mutex completed_flags_mutex_;                 // +2920 (sure)
  std::vector<bool> completed_flags_;                // +3000 (sure) 32 bytes
  double prox_dist_thresh_ = 0;                      // +3032 (sure)
  double cumulative_distance_ = 0;                   // +3040 (sure)
  GlobalMapType global_map_type_;                    // +3048
  std::thread thread_3056_;                          // +3056 never started in c16
  bool platmap_built_ = false;                       // +3072 (sure) plain bool
  std::atomic<bool> plat_map_ready_{false};          // +3073 (sure, xchg stores)
  std::atomic<bool> reloc_busy_{false};              // +3074 (sure, lock cmpxchg / xchg)

  static void layout_check();
};

inline void LoopClosing::layout_check()
{
  static_assert(sizeof(LoopClosing) == 0xC10, "sizeof(LoopClosing)");
  static_assert(offsetof(LoopClosing, map_id_) == 16, "");
  static_assert(offsetof(LoopClosing, save_map_enabled_) == 20, "");
  static_assert(offsetof(LoopClosing, plat_map_) == 24, "");
  static_assert(offsetof(LoopClosing, reloc_frames_mutex_) == 40, "");
  static_assert(offsetof(LoopClosing, reloc_info_lock_) == 120, "");
  static_assert(offsetof(LoopClosing, reloc_kf_list_) == 200, "");
  static_assert(offsetof(LoopClosing, reloc_success_) == 224, "");
  static_assert(offsetof(LoopClosing, reLoc_correction_info_) == 232, "");
  static_assert(offsetof(LoopClosing, i272_) == 272, "");
  static_assert(offsetof(LoopClosing, T_C_B_) == 288, "");
  static_assert(offsetof(LoopClosing, T_B_C_) == 352, "");
  static_assert(offsetof(LoopClosing, mask_) == 416, "");
  static_assert(offsetof(LoopClosing, aruco_params_) == 512, "");
  static_assert(offsetof(LoopClosing, aruco_dict_) == 528, "");
  static_assert(offsetof(LoopClosing, cams_) == 544, "");
  static_assert(offsetof(LoopClosing, options_) == 560, "");
  static_assert(offsetof(LoopClosing, mode_) == 1216, "");
  static_assert(offsetof(LoopClosing, map_tag_) == 1224, "");
  static_assert(offsetof(LoopClosing, vec_1256_) == 1256, "");
  static_assert(offsetof(LoopClosing, deque_1280_) == 1280, "");
  static_assert(offsetof(LoopClosing, sz_1320_) == 1320, "");
  static_assert(offsetof(LoopClosing, tag_save_) == 1328, "");
  static_assert(offsetof(LoopClosing, tag_load_) == 1448, "");
  static_assert(offsetof(LoopClosing, lc_info_lock_) == 1568, "");
  static_assert(offsetof(LoopClosing, lc_correction_info_) == 1648, "");
  static_assert(offsetof(LoopClosing, lc_matched_points_info_) == 1688, "");
  static_assert(offsetof(LoopClosing, cur_kf_to_lc_kf_bundle_id_map_) == 1728, "");
  static_assert(offsetof(LoopClosing, lc_closed_loops_) == 1744, "");
  static_assert(offsetof(LoopClosing, kf_list_mutex_) == 1768, "");
  static_assert(offsetof(LoopClosing, kf_list_) == 1848, "");
  static_assert(offsetof(LoopClosing, last_reloc_kfs_) == 1872, "");
  static_assert(offsetof(LoopClosing, last_run_lc_frame_trackIDs_) == 1896, "");
  static_assert(offsetof(LoopClosing, suspend_lc_after_correction_) == 1920, "");
  static_assert(offsetof(LoopClosing, ignore_next_constraint_in_pg_) == 1928, "");
  static_assert(offsetof(LoopClosing, recovery_after_loss_) == 1929, "");
  static_assert(offsetof(LoopClosing, map_alignment_se3_) == 1936, "");
  static_assert(offsetof(LoopClosing, cur_loop_check_viz_info_) == 1960, "");
  static_assert(offsetof(LoopClosing, num_queries_) == 2128, "");
  static_assert(offsetof(LoopClosing, lc_thread_) == 2152, "");
  static_assert(offsetof(LoopClosing, lc_cond_) == 2160, "");
  static_assert(offsetof(LoopClosing, lc_stop_) == 2232, "");
  static_assert(offsetof(LoopClosing, lc_mutex_) == 2240, "");
  static_assert(offsetof(LoopClosing, run_lc_on_this_frame_) == 2320, "");
  static_assert(offsetof(LoopClosing, reloc_thread_) == 2328, "");
  static_assert(offsetof(LoopClosing, reloc_stop_) == 2408, "");
  static_assert(offsetof(LoopClosing, platmap_cmd_) == 2496, "");
  static_assert(offsetof(LoopClosing, load_save_thread_) == 2504, "");
  static_assert(offsetof(LoopClosing, platmap_stop_) == 2584, "");
  static_assert(offsetof(LoopClosing, platmap_mutex_) == 2592, "");
  static_assert(offsetof(LoopClosing, pose_key_map_) == 2672, "");
  static_assert(offsetof(LoopClosing, orb_) == 2736, "");
  static_assert(offsetof(LoopClosing, beblid_) == 2752, "");
  static_assert(offsetof(LoopClosing, lc_frame_count_) == 2768, "");
  static_assert(offsetof(LoopClosing, voc_) == 2784, "");
  static_assert(offsetof(LoopClosing, K_) == 2864, "");
  static_assert(offsetof(LoopClosing, D_) == 2888, "");
  static_assert(offsetof(LoopClosing, scale_retrieval_approach_) == 2912, "");
  static_assert(offsetof(LoopClosing, completed_flags_mutex_) == 2920, "");
  static_assert(offsetof(LoopClosing, completed_flags_) == 3000, "");
  static_assert(offsetof(LoopClosing, prox_dist_thresh_) == 3032, "");
  static_assert(offsetof(LoopClosing, cumulative_distance_) == 3040, "");
  static_assert(offsetof(LoopClosing, global_map_type_) == 3048, "");
  static_assert(offsetof(LoopClosing, thread_3056_) == 3056, "");
  static_assert(offsetof(LoopClosing, platmap_built_) == 3072, "");
  static_assert(offsetof(LoopClosing, plat_map_ready_) == 3073, "");
  static_assert(offsetof(LoopClosing, reloc_busy_) == 3074, "");
}

// ---- free helpers ------------------------------------------------------------------------
// 0x180178e40 (c15, geometric_verification): returns true when few landmarks are shared.
bool commonLandMarkCheck(const std::vector<int>& track_ids_1, const std::vector<int>& track_ids_2,
                         const double thresh);
// 0x18018c070 (this chunk): rebuilds the mixed_* vectors of every KeyFrame of a loaded map.
void recovery_kf(PlatMap* plat_map);

}  // namespace totem
}  // namespace pimax
