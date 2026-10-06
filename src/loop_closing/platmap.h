// pimax_slam.pi.dll -- src/loop_closing/platmap.h  (from draft c15; the free serialization
// functions moved to loop_closing/serialization_helpers.h, the KeyFrame-graph save/load is below)
//
// Pimax map persistence (PlatMap / KeyFrame).
//
// Placement note: every PlatMap/KeyFrame function of the binary is emitted inside the
// loop_closing.obj address range (0x18017e8e0 .. 0x180185490), interleaved with
// LoopClosing code.  That is what MSVC does for functions that are defined inline in a
// header (or in loop_closing.cpp itself).  We put the class here; the out-of-line
// bodies live in platmap.cpp.  TODO(verify): original file split (platmap.h/.cpp?).
//
// The boost serializer instantiations themselves (oserializer/iserializer
// <{binary,portable_binary}_{o,i}archive, T>, 0x180116xxx-0x180127xxx, and the
// serialize bodies 0x1800b65b0.., 0x1800b9520, 0x1800b99e0, 0x1800dc190.., 0x1800dc3c0,
// 0x1800dc670, 0x1800dc8a0, 0x1800dcae0, 0x1800dcd10, 0x1800dcfe0, 0x1800d1630) are
// emitted OUTSIDE this chunk (first user is an earlier object).  They are reconstructed
// here anyway because the on-disk format depends on them; see notes/c15_platmap.md.
#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/array_wrapper.hpp>

#include <DBoW2/DBoW2.h>                       // DBoW2::BowVector

#include "common/transformation.h"
#include "loop_closing/serialization_helpers.h"

namespace pimax {
namespace totem {

// ---------------------------------------------------------------------------------------
// KeyFrame  (sizeof == 0x300 = 768: make_shared alloc 0x310 in 0x180185e40, malloc(0x300)
// in pointer_iserializer heap allocator 0x1801102f0).  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// (operator new == malloc + alignment assert, delete == free).
// Upstream counterpart: svo_online_loopclosing/keyframe.h (svo::KeyFrame), heavily changed.
// ---------------------------------------------------------------------------------------
class KeyFrame
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 0x1800e26d0 (outside chunk) -- used by boost pointer loading (placement default ctor).
  // Leaves the four ints / timestamp / num_bow_features_ uninitialized.
  KeyFrame() {}

  // 0x18017f4c0
  // Called from LoopClosing (0x180185e40, 0x180186a50) as
  //   KeyFrame(frame->X(+252 of bundle?), frame->+20, frame->id_(+16), loop_closing->map_id_)
  KeyFrame(int nframe_id, int cam_id, int frame_id, int map_id);

  ~KeyFrame() {}   // 0x1800e4ee0 (outside chunk; implicit member dtors)

  // --- layout (offsets verified from ctor 0x18017f4c0 + serialize 0x1800dc3c0/0x1800dcd10)
  int map_id_;                       // +0    serialized #1; key of PlatMap::kf_map_ / map_index_
  int NframeID_;                     // +4    serialized #2; renumbered on load (see PlatMap::load)
  int frame_id_;                     // +8    serialized #3   TODO(verify) name
  int cam_id_;                       // +12   serialized #4   TODO(verify) name
  size_t lc_frame_count_;            // +16   not serialized, not initialized  TODO(verify)
  double timestamp_sec_abs_;         // +24   serialized #5 (raw 8 bytes)
  cv::Mat keyframe_image_;           // +32   not serialized
  Transformation T_w_c_;             // +128  serialized #6 (Eigen-aligned, identity)
  Transformation T_unk_192_;         // +192  not serialized (identity)  TODO(verify) name
  cv::Mat mat_256_;                  // +256  not serialized  TODO(verify)
  std::vector<cv::Point2f> bow_keypoints_;      // +352 serialized #7
  std::vector<cv::Mat> bow_features_;           // +376 serialized #8
  std::vector<int> bow_node_ids_;               // +400 serialized #9
  DBoW2::BowVector vec_bow_;                    // +424 serialized #10 (std::map, 16 bytes)
  cv::Mat svo_features_mat_;                    // +440 not serialized
  std::vector<cv::Mat> svo_features_;           // +536 serialized #11
  std::vector<int> svo_node_ids_;               // +560 serialized #13 (!) after +584
  std::vector<cv::Point2f> svo_keypointsvector_;// +584 serialized #12
  std::vector<cv::Point3f> svo_landmarksvector_cam_; // +608 serialized #14
  std::vector<int> svo_landmark_ids_;           // +632 not serialized  TODO(verify) type/name
  std::vector<int> svo_trackIDsvector_;         // +656 not serialized  TODO(verify) type/name
  std::vector<cv::Point2f> mixed_keypoints_;    // +680 rebuilt by recovery_kf (0x18018c070)
  std::vector<int> mixed_node_ids_;             // +704 rebuilt by recovery_kf
  std::vector<cv::Mat> mixed_features_;         // +728 rebuilt by recovery_kf
  size_t num_bow_features_;                     // +752 serialized #15 (integer, 8 bytes)
  // +760..+767 padding (alignment 16)

  // boost::serialization -- KeyFrame::serialize<Archive>
  //   portable: 0x1800dc3c0 (load) / 0x1800dcd10 (save); native binary: via
  //   iserializer<binary_iarchive,KeyFrame> 0x180116e90 / oserializer 0x180126180.
  // Class version 0 (no BOOST_CLASS_VERSION), tracking = track_selectively, KeyFrame is
  // only ever saved through raw pointers (vector<vector<KeyFrame*>>).
  // NOTE the field order: svo_keypointsvector_ (+584) is written BEFORE svo_node_ids_ (+560).
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar & map_id_;
    ar & NframeID_;
    ar & frame_id_;
    ar & cam_id_;
    ar & timestamp_sec_abs_;
    ar & T_w_c_;
    ar & bow_keypoints_;
    ar & bow_features_;
    ar & bow_node_ids_;
    ar & vec_bow_;
    ar & svo_features_;
    ar & svo_keypointsvector_;
    ar & svo_node_ids_;
    ar & svo_landmarksvector_cam_;
    ar & num_bow_features_;
  }
};
static_assert(sizeof(size_t) == 8, "x64 only");
static_assert(sizeof(KeyFrame) == 0x300, "sizeof(KeyFrame)");
static_assert(offsetof(KeyFrame, timestamp_sec_abs_) == 24, "");
static_assert(offsetof(KeyFrame, keyframe_image_) == 32, "");
static_assert(offsetof(KeyFrame, T_w_c_) == 128, "");
static_assert(offsetof(KeyFrame, T_unk_192_) == 192, "");
static_assert(offsetof(KeyFrame, mat_256_) == 256, "");
static_assert(offsetof(KeyFrame, bow_keypoints_) == 352, "");
static_assert(offsetof(KeyFrame, vec_bow_) == 424, "");
static_assert(offsetof(KeyFrame, svo_features_mat_) == 440, "");
static_assert(offsetof(KeyFrame, svo_features_) == 536, "");
static_assert(offsetof(KeyFrame, svo_node_ids_) == 560, "");
static_assert(offsetof(KeyFrame, svo_keypointsvector_) == 584, "");
static_assert(offsetof(KeyFrame, svo_landmarksvector_cam_) == 608, "");
static_assert(offsetof(KeyFrame, mixed_keypoints_) == 680, "");
static_assert(offsetof(KeyFrame, mixed_features_) == 728, "");
static_assert(offsetof(KeyFrame, num_bow_features_) == 752, "");
using KeyFramePtr = std::shared_ptr<KeyFrame>;

// ---------------------------------------------------------------------------------------
// MapIndex: value type of PlatMap::map_index_ (std::map<int, MapIndex>, node 0x40).
// Default values from the node constructors in 0x180183320 / 0x1801824b0.
// Written to / read from the YAML index file (platMap_orborb_K8L4.yaml).
// ---------------------------------------------------------------------------------------
struct MapIndex
{
  int map_id_ = -1;                 // +0
  int kf_nums_ = 0;                 // +4
  double newest_timestamp_ = -1.0;  // +8
  int startIdx_ = 0;                // +16  index into kf_list_ (first group of this map)
  int endIdx_ = 0;                  // +20  one past the last group
};

// Map file-format version written to / required in the YAML index ("map_version_").
// Global std::string initialised by dynamic initializer 0x180002bb0 ("1.0.0",
// storage at Buf2/Size_0 in .data).  TODO(verify) symbol name.
extern const std::string kPlatMapVersion;   // = "1.0.0" (0x18046A200; defined in loop_closing.cpp,
                                            // which also holds every PlatMap/KeyFrame body)

// ---------------------------------------------------------------------------------------
// PlatMap  (sizeof == 0xE0 = 224: make_shared alloc 0xF0 in 0x18017e8e0).
// ---------------------------------------------------------------------------------------
class PlatMap
{
public:
  // ctor inlined into std::make_shared<PlatMap>() (0x18017e8e0)
  PlatMap()
  {
    kf_map_.clear();
    kf_list_.clear();
    map_index_.clear();
  }
  // dtor: 0x180182960 (scalar deleting, called from _Ref_count_obj2<PlatMap>::_Destroy
  // 0x180185530): implicit member destruction only.
  ~PlatMap() = default;

  // 0x180182b70  -- merge a finished session (vector of KF groups) into kf_map_ and
  // rebuild kf_list_/map_index_.  Locks mtx_.  TODO(verify) name.
  void UpdateMap(const std::vector<std::vector<KeyFramePtr>>& kf_list, int map_id);
  // 0x180182d40
  void BuildMapIndex();
  // 0x180182f80  native boost::archive::binary_iarchive loader
  void load(const std::string& filename);
  // 0x180183320  YAML index loader; returns false on any problem
  bool loadIndex(const std::string& filename);
  // 0x180183aa0  portable_binary_iarchive ("PBA") loader
  void load_bin(const std::string& filename);
  // 0x180183ec0  native boost::archive::binary_oarchive writer (temp file + rename)
  void save(const std::string& filename);
  // not present in the binary (dead-stripped by /OPT:REF) but implied by the static
  // registration of oserializer<portable_binary_oarchive, PlatMap> (0x180002360).
  // TODO(verify): exact body unknown; see platmap.cpp.
  void save_bin(const std::string& filename);
  // 0x180184590  YAML index writer
  void saveIndex(const std::string& filename);
  // 0x180184f30  trims every map to max_kf_num_ groups and flattens kf_map_ -> kf_list_
  void Map2Vec(int map_id);
  // 0x180185110  kf_list_ -> kf_map_ (grouped by kf->map_id_)
  void Vec2Map();

  // --- layout ---------------------------------------------------------------------------
  std::mutex mtx_;                                                // +0   (80 bytes)
  int max_kf_num_ = 1000;                                         // +80  set from options (x5 in lab modes)
  std::map<int, std::vector<std::vector<KeyFramePtr>>> kf_map_;   // +88  node 0x40
  std::vector<std::vector<KeyFramePtr>> kf_list_;                 // +104 SERIALIZED (only member)
  std::map<int, MapIndex> map_index_;                             // +128 node 0x40
  std::vector<cv::Mat> K_list_;                                   // +144 copied from LoopClosing+2864
  std::vector<Eigen::VectorXd> D_list_;                           // +168 copied from LoopClosing+2888
  std::string map_version_;                                       // +192 filled by loadIndex

  // boost: PlatMap::serialize -> only kf_list_, written as vector<vector<KeyFrame*>>
  // (load_object_data 0x180117bc0/0x180116f30 call 0x1800b9520(ar, &kf_list_);
  //  save_object_data 0x180127270/0x180126260 call 0x1800b99e0(ar, &kf_list_)).
  // Class version 0, no tracking (never serialized through a pointer).
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    boost::serialization::split_free(ar, kf_list_, version);
  }
};
using PlatMapPtr = std::shared_ptr<PlatMap>;

static_assert(sizeof(MapIndex) == 24, "");
static_assert(sizeof(PlatMap) == 0xE0, "sizeof(PlatMap)");
static_assert(offsetof(PlatMap, max_kf_num_) == 80, "");
static_assert(offsetof(PlatMap, kf_map_) == 88, "");
static_assert(offsetof(PlatMap, kf_list_) == 104, "");
static_assert(offsetof(PlatMap, map_index_) == 128, "");
static_assert(offsetof(PlatMap, K_list_) == 144, "");
static_assert(offsetof(PlatMap, D_list_) == 168, "");
static_assert(offsetof(PlatMap, map_version_) == 192, "");

}  // namespace totem
}  // namespace pimax

// =======================================================================================
// PlatMap::kf_list_ (std::vector<std::vector<std::shared_ptr<KeyFrame>>>): saved as
// std::vector<std::vector<KeyFrame*>> so that boost's pointer tracking is used (no shared_ptr
// serializer is instantiated anywhere).  save 0x1800B9270 (binary) / 0x1800B99E0 (portable),
// load 0x1800B8DB0 (binary) / 0x1800B9520 (portable) (c07 + c15, identical drafts).
// Quirk: load does NOT clear the destination (PlatMap::load / load_bin clear it beforehand);
// every KeyFrame* becomes owned by a plain std::shared_ptr<KeyFrame>(p) (_Ref_count<KeyFrame>).
// =======================================================================================
namespace boost {
namespace serialization {

template <class Archive>
void save(Archive& ar,
          const std::vector<std::vector<std::shared_ptr<pimax::totem::KeyFrame>>>& kf_list,
          const unsigned int /*version*/)
{
  std::vector<std::vector<pimax::totem::KeyFrame*>> raw_list;
  for (const auto& kfs : kf_list)
  {
    std::vector<pimax::totem::KeyFrame*> raw;
    for (const auto& kf : kfs)
      raw.push_back(kf.get());
    raw_list.push_back(raw);
  }
  ar << raw_list;
}

template <class Archive>
void load(Archive& ar,
          std::vector<std::vector<std::shared_ptr<pimax::totem::KeyFrame>>>& kf_list,
          const unsigned int /*version*/)
{
  std::vector<std::vector<pimax::totem::KeyFrame*>> raw_list;
  ar >> raw_list;
  for (auto raw : raw_list)   // by value (the binary copies every inner vector)
  {
    std::vector<std::shared_ptr<pimax::totem::KeyFrame>> kfs;
    for (auto* kf : raw)
      kfs.push_back(std::shared_ptr<pimax::totem::KeyFrame>(kf));   // _Ref_count<KeyFrame>
    kf_list.push_back(kfs);
  }
}

}  // namespace serialization
}  // namespace boost
