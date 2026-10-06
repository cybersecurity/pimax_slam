// pimax_slam.pi.dll -- src/loop_closing/platmap.cpp  (draft c15_platmap/loop_closing/platmap.cpp)
//
// Out-of-line KeyFrame / PlatMap code.  All functions here are "pimax-new" (no upstream
// counterpart).  NOTE (layout only, no behavioural effect): in the binary every function of this
// file lies inside the loop_closing.obj range (KeyFrame ctor 0x18017f4c0 before the LoopClosing
// ctor, PlatMap methods 0x180182960..0x180185110 between ~LoopClosing and addFrameToPR), i.e. the
// original most likely defined them in loop_closing.cpp or inline in a header.
// TODO(verify) original file split.  kPlatMapVersion is defined in loop_closing.cpp (its dynamic
// initializer 0x180002bb0 belongs to TU47 = loop_closing.obj).
//
// The boost serializer instantiations (vector<vector<KeyFrame*>>, KeyFrame::serialize, cv::Mat,
// BowVector, Transformation; loop_closing/platmap.h + serialization_helpers.h) are instantiated by
// the archive operations below.
#include "loop_closing/platmap.h"

#include <algorithm>
#include <fstream>
#include <stdexcept>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/serialization/string.hpp>
#include <opencv2/core/persistence.hpp>

#include "common/frame.h"                                    // Frame::frame_counter_ (dword_18047DB38)
#include "common/logger.h"                                   // LOGD/LOGI/LOGW/LOGE
#include "common/portable_archive/portable_binary_iarchive.hpp"
#include "common/portable_archive/portable_binary_oarchive.hpp"
#include "loop_closing/loop_closing.h"                       // recovery_kf (0x18018c070)

namespace pimax {
namespace totem {

// ---------------------------------------------------------------------------------------
// 0x18017f4c0
KeyFrame::KeyFrame(int nframe_id, int cam_id, int frame_id, int map_id)
{
  NframeID_ = nframe_id;   // +4
  cam_id_ = cam_id;        // +12
  frame_id_ = frame_id;    // +8
  map_id_ = map_id;        // +0
}

// ---------------------------------------------------------------------------------------
// 0x180182b70
void PlatMap::UpdateMap(const std::vector<std::vector<KeyFramePtr>>& kf_list, int map_id)
{
  std::lock_guard<std::mutex> lock(mtx_);
  for (const auto& kfs : kf_list)
  {
    // NOTE: kfs[0] is dereferenced without checking kfs.empty() (quirk kept).
    if (kfs[0])
    {
      const int id = kfs[0]->map_id_;
      if (id <= map_id && id >= 0)
      {
        auto it = kf_map_.find(id);
        if (it == kf_map_.end())
        {
          kf_map_[kfs[0]->map_id_] = {kfs};   // initializer list built before operator[]
        }
        else
        {
          auto& groups = it->second;
          // vector<shared_ptr> operator== -> compares raw pointers element-wise
          if (std::find(groups.begin(), groups.end(), kfs) == groups.end())
            groups.push_back(kfs);
        }
      }
    }
  }
  Map2Vec(map_id);
}

// ---------------------------------------------------------------------------------------
// 0x180182d40
void PlatMap::BuildMapIndex()
{
  map_index_.clear();
  for (int i = 0; i < kf_list_.size(); ++i)
  {
    if (kf_list_[i].size() == 0)
      continue;

    bool all_valid = true;
    for (auto kf : kf_list_[i])     // by value (shared_ptr copy; ref inc/dec in binary)
    {
      if (!kf)
      {
        all_valid = false;
        break;
      }
    }
    if (!all_valid)
      continue;

    int map_id = kf_list_[i][0]->map_id_;
    if (map_index_.find(map_id) != map_index_.end())
    {
      map_index_[map_id].kf_nums_++;
      map_index_[map_id].newest_timestamp_ = kf_list_[i][0]->timestamp_sec_abs_;
      map_index_[map_id].endIdx_++;
    }
    else
    {
      MapIndex index;
      index.map_id_ = map_id;
      index.kf_nums_ = 1;
      index.newest_timestamp_ = kf_list_[i][0]->timestamp_sec_abs_;
      index.startIdx_ = i;
      index.endIdx_ = i + 1;
      map_index_[map_id] = index;
    }
  }
}

// ---------------------------------------------------------------------------------------
// 0x180182f80  -- native (non-portable) boost binary archive.
// Exceptions: std::exception is logged and RETHROWN (LoopClosing::load 0x180188900 then
// falls back to load_bin(path + ".pba")); anything else is swallowed.
void PlatMap::load(const std::string& filename)
{
  kf_list_.clear();
  map_index_.clear();
  int min_id = 0;
  int max_id = 0;
  try
  {
    std::ifstream ifs(filename, std::ios::binary);
    boost::archive::binary_iarchive ia(ifs);
    std::string tag;          // "PlatMap"
    ia >> tag;
    ia >> *this;
    ifs.close();
    if (kf_list_.empty())
      LOGE("kf_list_ is empty! kf_list_ size %d\n", kf_list_.size());
    recovery_kf(this);
    Vec2Map();
    BuildMapIndex();
  }
  catch (const std::exception& e)
  {
    kf_list_.clear();
    map_index_.clear();
    LOGE("Standard Exception load(): %s\n", e.what());
    throw;
  }
  catch (...)
  {
    kf_list_.clear();
    map_index_.clear();
  }

  // Renumber NframeID_ so the loaded frames start at 1, and advance the global Frame id
  // counter past them.  NOTE: kf_list_[0][0] / kf_list_[0] are read even if kf_list_ is
  // empty (UB in the binary too -- it reads through the null begin pointer).
  if (!kf_list_.empty())
  {
    min_id = kf_list_[0][0]->NframeID_;
    for (auto& kfs : kf_list_)
    {
      if (!kfs.empty())
      {
        int id = kfs[0]->NframeID_;
        max_id = std::max(id, max_id);
        if (id < min_id)
          min_id = kfs[0]->NframeID_;
      }
    }
    for (auto& kfs : kf_list_)
      for (auto& kf : kfs)
        kf->NframeID_ += 1 - min_id;
  }
  Frame::frame_counter_ += (max_id - min_id + 1) * kf_list_[0].size();   // dword_18047DB38
}

// ---------------------------------------------------------------------------------------
// 0x180183320  -- cv::FileStorage YAML index.  mode 16 == READ | FORMAT_YAML.
bool PlatMap::loadIndex(const std::string& filename)
{
  cv::FileStorage fs;
  fs.open(filename, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);
  if (!fs.isOpened())
  {
    LOGE("Failed to open file %s\n", filename.c_str());
    return false;
  }

  cv::FileNode index_node = fs["map_index_"];
  if (index_node.empty() || index_node.size() == 0)
    return false;

  cv::FileNode version_node = fs["map_version_"];
  if (version_node.empty() || version_node.isNone())
  {
    LOGW("Wont load the map because of none map version.\n");
    return false;
  }

  std::string version = version_node.string();
  if (version != kPlatMapVersion)
  {
    LOGW("Wont load the map because of illegal map version.\n");
    return false;
  }
  map_version_ = version;

  std::map<int, MapIndex> map_index;
  for (cv::FileNodeIterator it = index_node.begin(); it != index_node.end(); ++it)
  {
    cv::FileNode node = *it;
    MapIndex index;
    index.map_id_ = (int)node["map_id_"];
    index.kf_nums_ = (int)node["kf_nums_"];
    LOGI("load index, map_id_ %d\n", index.map_id_);
    LOGI("load index, kf_nums_ %d\n", index.kf_nums_);
    index.newest_timestamp_ = (double)node["newest_timestamp_"];
    index.startIdx_ = (int)node["startIdx_"];
    index.endIdx_ = (int)node["endIdx_"];
    map_index[index.map_id_] = index;
  }

  if (map_index.size() > 3)       // at most 3 maps are accepted
    return false;

  map_index_ = map_index;
  return true;
}

// ---------------------------------------------------------------------------------------
// 0x180183aa0  -- portable binary archive ("PBA library"); file name is usually
// "<map>.bin.pba" (LoopClosing::load fallback).  Differences to load():
//  * is_open() check with "Failed to open map file for loading." runtime_error,
//  * extra LOGW progress lines, %zu instead of %d,
//  * std::exception is logged and NOT rethrown; renumbering is inside the try block,
//  * the max/min loop does not skip empty groups.
void PlatMap::load_bin(const std::string& filename)
{
  LOGI("About to load_bin map: %s with PBA library\n", filename.c_str());
  kf_list_.clear();
  map_index_.clear();
  int max_id = 0;
  try
  {
    {
      std::ifstream ifs(filename, std::ios::binary);
      if (!ifs.is_open())
        throw std::runtime_error("Failed to open map file for loading.");
      portable_binary_iarchive ia(ifs);
      std::string tag;        // "PlatMap"
      ia >> tag;
      ia >> *this;
      ifs.close();
      if (kf_list_.empty())
        LOGE("kf_list_ is empty! kf_list_ size %zu\n", kf_list_.size());
      LOGW("recovery_kf...\n");
      recovery_kf(this);
      LOGW("Vec2Map...\n");
      Vec2Map();
      LOGW("BuildMapIndex...\n");
      BuildMapIndex();
    }

    int min_id = 0;
    if (!kf_list_.empty())
    {
      min_id = kf_list_[0][0]->NframeID_;
      for (auto& kfs : kf_list_)
      {
        int id = kfs[0]->NframeID_;
        if (max_id < id)
          max_id = kfs[0]->NframeID_;
        if (id < min_id)
          min_id = kfs[0]->NframeID_;
      }
      for (auto& kfs : kf_list_)
        for (auto& kf : kfs)
          kf->NframeID_ += 1 - min_id;
    }
    Frame::frame_counter_ += (max_id - min_id + 1) * kf_list_[0].size();
  }
  catch (const std::exception& e)
  {
    kf_list_.clear();
    map_index_.clear();
    LOGE("Standard Exception load(): %s\n", e.what());
    return;
  }
  catch (...)
  {
    kf_list_.clear();
    map_index_.clear();
  }
}

// ---------------------------------------------------------------------------------------
// 0x180183ec0  -- writes "<filename>-bk" with a native binary_oarchive, then renames it
// over <filename> (boost::filesystem::rename).  mode 34 == out | binary.
void PlatMap::save(const std::string& filename)
{
  std::string temp_filename = filename + "-bk";
  std::ofstream ofs(temp_filename, std::ios::binary);
  try
  {
    if (!ofs.is_open())
      throw std::runtime_error("Failed to open temp file for saving.");

    boost::archive::binary_oarchive oa(ofs);
    std::string tag = "PlatMap";
    oa << tag;
    oa << *this;
    ofs.close();
    LOGW("Temporary map saved to %s\n", temp_filename.c_str());

    boost::filesystem::rename(temp_filename, filename);
    LOGW("Success save map to %s \n", filename.c_str());
  }
  catch (const std::exception& e)
  {
    LOGE("Error during saving map: %s \n", std::string(e.what()).c_str());
    if (boost::filesystem::exists(temp_filename))
    {
      boost::filesystem::remove(temp_filename);
      LOGE("Temporary map file removed due to error.");
    }
  }
}

// ---------------------------------------------------------------------------------------
// NOT IN THE BINARY.  Only its oserializer<portable_binary_oarchive, PlatMap> singleton
// survived (static init 0x180002360).  Mirror of save() with the portable archive so that
// the singleton gets instantiated the same way.  TODO(verify): original body unknown;
// never called (dead-stripped), so behaviour does not matter.
void PlatMap::save_bin(const std::string& filename)
{
  std::ofstream ofs(filename, std::ios::binary);
  portable_binary_oarchive oa(ofs);
  std::string tag = "PlatMap";
  oa << tag;
  oa << *this;
}

// ---------------------------------------------------------------------------------------
// 0x180184590  -- mode 17 == WRITE | FORMAT_YAML
void PlatMap::saveIndex(const std::string& filename)
{
  cv::FileStorage fs;
  fs.open(filename, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
  fs << "map_version_" << kPlatMapVersion;
  fs << "map_index_" << "[";
  for (auto it = map_index_.begin(); it != map_index_.end(); ++it)
  {
    int map_id = it->first;
    MapIndex index = it->second;
    fs << "{";
    fs << "map_id_" << map_id;
    LOGI("save index, map_id_ %d\n", map_id);
    LOGI("save index, kf_nums_ %d\n", index.kf_nums_);
    fs << "kf_nums_" << index.kf_nums_;
    fs << "newest_timestamp_" << index.newest_timestamp_;
    fs << "startIdx_" << index.startIdx_;
    fs << "endIdx_" << index.endIdx_;
    fs << "}";
  }
  fs << "]";
  fs.release();
  LOGI("success save index\n");
}

// ---------------------------------------------------------------------------------------
// 0x180184f30  -- map_id is passed (edx) by the caller but unused.
void PlatMap::Map2Vec(int /*map_id*/)
{
  for (auto it = kf_map_.begin(); it != kf_map_.end(); ++it)
  {
    int over = (int)it->second.size() - max_kf_num_;
    if (over > 0)
      it->second.erase(it->second.begin(), it->second.begin() + over);   // drop oldest
  }
  kf_list_.clear();
  for (auto it = kf_map_.begin(); it != kf_map_.end(); ++it)
    kf_list_.insert(kf_list_.end(), it->second.begin(), it->second.end());
  BuildMapIndex();
}

// ---------------------------------------------------------------------------------------
// 0x180185110
void PlatMap::Vec2Map()
{
  kf_map_.clear();
  for (auto& kfs : kf_list_)
  {
    if (kfs.size() >= 1)
    {
      bool all_valid = true;
      for (auto kf : kfs)          // by value
      {
        if (!kf)
        {
          all_valid = false;
          break;
        }
      }
      if (!all_valid)
        continue;

      if (kf_map_.find(kfs[0]->map_id_) != kf_map_.end())
        kf_map_[kfs[0]->map_id_].push_back(kfs);
      else
        kf_map_[kfs[0]->map_id_] = {kfs};
    }
  }
}

}  // namespace totem
}  // namespace pimax
