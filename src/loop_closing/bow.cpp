// pimax_slam.pi.dll -- src/loop_closing/bow.cpp  (draft c15_platmap/loop_closing/bow.cpp)
//
// Pimax version of svo_online_loopclosing/src/bow.cpp.  In the binary these functions are
// emitted between the DBoW2 TemplatedVocabulary<cv::Mat, FORB> instantiations
// (0x180172460 .. 0x180178aa0): this object is the first user of the vocabulary template, so
// its members are instantiated here.
//
// Function table (address / upstream status):
//   0x180174300  changeStructure                    upstream-modified (reserve + push_back: appends)
//   0x180174480  compareBOWs                        upstream-identical
//   0x1801747E0  createBOW                          upstream-identical (getNodeID inlined, levelup 0)
//   0x180174B70  extractBoWFeaturesFromImage        upstream-modified (static ORB, 2x2 px thinning)
//   0x180175160  extractFeaturesFromSVOKeypoints    upstream-modified (heavily, see below)
//   0x180176330  getNodeID                          upstream-identical
#include "loop_closing/bow.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <unordered_map>
#include <utility>

#include "common/occupancy_grid_2d.h"   // OccupandyGrid2D (ctor 0x1800aa2f0 / dtor 0x1800aa440)
#include "loop_closing/pair_hash.h"     // PairHash 0x180170e50

namespace pimax {
namespace totem {

// ---------------------------------------------------------------------------------------
// 0x180174300  changeStructure  (upstream: out->resize(rows); (*out)[i] = plain.row(i);)
// QUIRK: appends to *out (no clear / resize).
void changeStructure(const cv::Mat& plain, std::vector<cv::Mat>* out)
{
  out->reserve(plain.rows);
  for (int i = 0; i < plain.rows; ++i)
  {
    out->push_back(plain.row(i));
  }
}

// ---------------------------------------------------------------------------------------
// 0x180174b70  extractBoWFeaturesFromImage
// Pimax: function-local static ORB(500, 1.2f, 4, 31, 0, 2, HARRIS, 31, 20) (guard dword_18047EE48,
// object qword_18047EE38, atexit 0x1803a6150); keypoints sorted by response (descending) and
// thinned with a 2x2-pixel occupancy grid (at most one keypoint per cell).
// QUIRK: keypoints_pt2f is NOT cleared (only feature is).
void extractBoWFeaturesFromImage(const cv::Mat& image,
                                 std::vector<cv::Point2f>* keypoints_pt2f,
                                 std::vector<cv::Mat>* feature)
{
  feature->clear();

  static cv::Ptr<cv::ORB> orb =
      cv::ORB::create(500, 1.2f, 4, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);

  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  std::vector<cv::Mat> descriptor_rows;

  orb->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
  changeStructure(descriptors, &descriptor_rows);

  std::multimap<double, int, std::greater<double>> response_to_index;   // node 0x30
  for (size_t i = 0; i < keypoints.size(); ++i)
  {
    response_to_index.emplace(keypoints[i].response, i);
  }

  // cell 2 px, dims ceil(cols * 0.5) x ceil(rows * 0.5) (0.5 at 0x1803af2b8)
  OccupandyGrid2D grid(2, std::ceil(image.cols * 0.5), std::ceil(image.rows * 0.5));
  for (auto it = response_to_index.begin(); it != response_to_index.end(); ++it)
  {
    const int idx = it->second;
    const cv::KeyPoint& kp = keypoints[idx];
    const size_t cell = grid.getCellIndex(kp.pt.x, kp.pt.y);   // (int)x/2 + n_cols*((int)y/2)
    if (!grid.occupancy_[cell])
    {
      feature->push_back(descriptor_rows[idx]);
      keypoints_pt2f->push_back(kp.pt);
      grid.occupancy_[cell] = true;
    }
  }
}

// ---------------------------------------------------------------------------------------
// 0x1801747e0  createBOW  (getNodeID inlined, levelup 0)
void createBOW(const std::vector<cv::Mat>& feature, const OrbVocabulary& voc,
               DBoW2::BowVector* v, std::vector<int>* node_id)
{
  voc.transform(feature, *v);
  if (node_id == nullptr)
  {
    return;
  }
  getNodeID(feature, voc, 0, node_id);
}

// ---------------------------------------------------------------------------------------
// 0x180174480  compareBOWs
double compareBOWs(const DBoW2::BowVector& v1, const DBoW2::BowVector& v2,
                   const OrbVocabulary& voc)
{
  double score = voc.score(v1, v2);
  return score;
}

// ---------------------------------------------------------------------------------------
// 0x180176330  getNodeID
void getNodeID(const std::vector<cv::Mat>& feature, const OrbVocabulary& voc,
               const int levelup, std::vector<int>* node_ids)
{
  node_ids->reserve(feature.size());
  for (unsigned int i = 0; i < feature.size(); i++)
  {
    node_ids->push_back(voc.getParentNode(voc.transform(feature[i]), levelup));
  }
}

// ---------------------------------------------------------------------------------------
// 0x180175160  extractFeaturesFromSVOKeypoints
// Pimax rewrite of the upstream function:
//  * all six output pointers must be non-null, otherwise nothing happens;
//  * a second function-local static ORB (guard dword_18047EE60, object qword_18047EE50, same
//    parameters);
//  * every input keypoint becomes a cv::KeyPoint(pt, 1, -1, 0, 0, class_id) where class_id
//    is the index of the FIRST (lowest-index) identical keypoint found in a 1e-6 grid
//    hash (3x3 neighbourhood), i.e. duplicates point at their first occurrence;
//    index > INT_MAX -> -1; non-finite points keep their own index;
//  * after ORB::compute, landmarks/ids/trackIDs are re-associated through class_id
//    (fast path) or a linear search over the old keypoints (|dx|,|dy| < 1e-6f).
// QUIRK: every surviving keypoint is pushed even when no landmark could be re-associated, so
// svo_keypoints can become longer than the landmark/id/trackID vectors.
void extractFeaturesFromSVOKeypoints(const cv::Mat& image,
                                     std::vector<cv::Point3f>* svo_landmarks,
                                     std::vector<int>* svo_landmark_ids,
                                     std::vector<int>* svo_trackIDs,
                                     std::vector<cv::Point2f>* svo_keypoints,
                                     std::vector<cv::Mat>* svo_features,
                                     cv::Mat* svo_descriptors)
{
  if (!svo_landmarks || !svo_landmark_ids || !svo_trackIDs || !svo_keypoints ||
      !svo_features || !svo_descriptors)
  {
    return;
  }

  std::vector<cv::Point2f> old_svo_keypoints;
  std::vector<cv::Point3f> old_svo_landmarks;
  std::vector<int> old_svo_landmark_ids;
  std::vector<int> old_svo_trackIDs;
  old_svo_keypoints = *svo_keypoints;
  old_svo_landmarks = *svo_landmarks;
  old_svo_landmark_ids = *svo_landmark_ids;
  old_svo_trackIDs = *svo_trackIDs;

  static cv::Ptr<cv::ORB> orb =
      cv::ORB::create(500, 1.2f, 4, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);

  std::vector<cv::KeyPoint> keypoints;
  keypoints.reserve(svo_keypoints->size());

  // NOTE: (double)1e-6f == 9.999999974752427e-07 is the divisor (0x1803bdd48), the
  // comparisons use the float 1e-6f (0x1803b2684).
  const double kGridSize = 1e-6f;
  std::unordered_map<std::pair<long long, long long>, std::vector<size_t>, PairHash> grid;
  grid.reserve(svo_keypoints->size());

  for (size_t i = 0; i < svo_keypoints->size(); ++i)
  {
    const float x = (*svo_keypoints)[i].x;
    size_t first_idx = i;
    if (std::isfinite(x))
    {
      const float y = (*svo_keypoints)[i].y;
      if (std::isfinite(y))
      {
        const long long gx = static_cast<long long>(std::floor(x / kGridSize));
        const long long gy = static_cast<long long>(std::floor(y / kGridSize));
        for (long long dx = -1; dx <= 1; ++dx)
        {
          for (long long dy = -1; dy <= 1; ++dy)
          {
            auto it = grid.find(std::make_pair(gx + dx, gy + dy));
            if (it != grid.end())
            {
              for (size_t j : it->second)
              {
                if (std::abs((*svo_keypoints)[j].x - x) < 1e-6f &&
                    std::abs((*svo_keypoints)[j].y - y) < 1e-6f && j < first_idx)
                {
                  first_idx = j;
                }
              }
            }
          }
        }
        grid[std::make_pair(gx, gy)].push_back(i);   // _Try_emplace 0x180172460
      }
    }
    const int class_id = first_idx > static_cast<size_t>(std::numeric_limits<int>::max())
                             ? -1
                             : static_cast<int>(first_idx);
    keypoints.push_back(cv::KeyPoint((*svo_keypoints)[i], 1.f, -1.f, 0.f, 0, class_id));
  }

  svo_keypoints->clear();
  svo_landmarks->clear();
  svo_landmark_ids->clear();
  svo_trackIDs->clear();

  if (!keypoints.empty() && !image.empty())
  {
    orb->compute(image, keypoints, *svo_descriptors);
    changeStructure(*svo_descriptors, svo_features);

    for (const cv::KeyPoint& kp : keypoints)
    {
      svo_keypoints->push_back(kp.pt);

      size_t pos;
      if (kp.class_id >= 0 && static_cast<size_t>(kp.class_id) < old_svo_keypoints.size() &&
          std::abs(old_svo_keypoints[kp.class_id].x - kp.pt.x) < 1e-6f &&
          std::abs(old_svo_keypoints[kp.class_id].y - kp.pt.y) < 1e-6f)
      {
        pos = kp.class_id;
      }
      else
      {
        auto it = std::find_if(old_svo_keypoints.begin(), old_svo_keypoints.end(),
                               [&kp](const cv::Point2f& p) {
                                 return std::abs(p.x - kp.pt.x) < 1e-6f &&
                                        std::abs(p.y - kp.pt.y) < 1e-6f;
                               });
        if (it == old_svo_keypoints.end())
          continue;
        pos = it - old_svo_keypoints.begin();
        if (pos >= old_svo_keypoints.size())
          continue;
      }

      if (pos < old_svo_landmarks.size() && pos < old_svo_landmark_ids.size() &&
          pos < old_svo_trackIDs.size())
      {
        svo_landmarks->push_back(old_svo_landmarks[pos]);
        svo_landmark_ids->push_back(old_svo_landmark_ids[pos]);
        svo_trackIDs->push_back(old_svo_trackIDs[pos]);
      }
    }
  }
}

}  // namespace totem
}  // namespace pimax
