// pimax_slam.pi.dll -- src/frontend/map.h  (from draft c11)
//
// pimax::totem::Map (fork of svo/include/svo/map.h).  sizeof(Map) == 0xB8 (operator new(0xB8) in the
// FrameProcessorBase ctor 0x1800DFDF0, wrapped in a shared_ptr via unique_ptr).  No vtable.
#pragma once

#include <deque>
#include <map>
#include <mutex>
#include <vector>

#include <cstddef>
#include <memory>
#include <utility>

#include "common/frame.h"
#include "common/point.h"
#include "common/types.h"

namespace pimax {
namespace totem {

/// Map object which saves all keyframes which are in a map.
class Map {
 public:
  typedef std::shared_ptr<Map> Ptr;
  // Pimax: ordered std::map (node 0x38, red-black tree) instead of upstream unordered_map.
  typedef std::map<int, FramePtr> Keyframes;   // Frame-Id & Pointer

  Keyframes keyframes_;                   // +0  (head ptr) / +8 (size)
  std::vector<PointPtr> points_to_delete_;// +16
  std::mutex points_to_delete_mutex_;     // +40
  int last_added_kf_id_;                  // +120
  std::deque<int> sorted_keyframe_ids_;   // +128 (proxy) .. +160 (size)
  FramePtr last_removed_kf_ = nullptr;    // +168

  Map();                                  // 0x18012CE00
  ~Map();                                 // 0x18012CEB0

  Map(const Map&) = delete;
  Map& operator=(const Map&) = delete;

  void reset();                                                        // 0x18012ED20

  /// Pimax: always appends to sorted_keyframe_ids_ (temporal_map flag removed).
  void addKeyframe(const FramePtr& new_keyframe);                      // 0x18012D680

  void removeKeyframe(const int frame_id);                             // 0x18012E9F0

  void removeOldestKeyframe();                                         // 0x18012EC70

  /// Pimax: takes the pointer by non-const reference and resets it at the end.
  void safeDeletePoint(PointPtr& pt);                                  // 0x18012EDE0

  void getOverlapKeyframes(const FramePtr& frame,
                           std::vector<std::pair<FramePtr, double>>* close_kfs) const;   // 0x18012E1B0

  /// Pimax: num_frames by const reference; completely different ranking (see .cpp).
  void getClosestNKeyframesWithOverlap(const FramePtr& cur_frame, const size_t& num_frames,
                                       std::vector<FramePtr>* visible_kfs) const;       // 0x18012D900

  /// Pimax-new: true iff all 5 key points of `kf` are visible in `frame`.
  /// TODO(verify) name. Used by the frame processor (0x1800B2F80).
  bool allKeyPointsVisible(const FramePtr& frame, const FramePtr& kf) const;            // 0x18012E930

  inline size_t size() const { return keyframes_.size(); }
  inline size_t numKeyframes() const { return keyframes_.size(); }

  // Upstream members that are not present in the image (unreferenced -> removed by /OPT:REF, or
  // deleted from the source): addPointToTrash, emptyPointsTrash, getClosestKeyframe,
  // getOldsestKeyframe, getFurthestKeyframe, getKeyframeById, getSortedKeyframes, transform,
  // checkDataConsistency, getLastRemovedKF. Inline accessors may still exist in the header.

 private:
  friend struct MapLayoutCheck;
};

struct MapLayoutCheck
{
  static_assert(sizeof(Map) == 0xB8, "sizeof(Map)");
  static_assert(offsetof(Map, points_to_delete_) == 16, "");
  static_assert(offsetof(Map, points_to_delete_mutex_) == 40, "");
  static_assert(offsetof(Map, last_added_kf_id_) == 120, "");
  static_assert(offsetof(Map, sorted_keyframe_ids_) == 128, "");
  static_assert(offsetof(Map, last_removed_kf_) == 168, "");
};

}  // namespace totem
}  // namespace pimax
