// pimax_slam.pi.dll -- src/loop_closing/bow.h  (from draft c15)
//
// Pimax version of svo_online_loopclosing/bow.h (only the functions present in
// chunk c15_platmap; extractFeaturesFromFolder / createVoc / imgFilter / loadVoc are not
// in the binary or were inlined).
#pragma once

#include <vector>
#include <string>

#include <DBoW2/DBoW2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace pimax {
namespace totem {

using OrbVocabulary = DBoW2::TemplatedVocabulary<cv::Mat, DBoW2::FORB>;

/// The DBoW2 vocabulary (80 bytes, upstream layout; Pimax patches load(const std::string&) and
/// adds fromStream(std::istream&) -- DBoW3 binary .dbow, see third_party/DBoW2).

// 0x180174b70  (upstream-modified)
void extractBoWFeaturesFromImage(const cv::Mat& image,
                                 std::vector<cv::Point2f>* keypoints_pt2f,
                                 std::vector<cv::Mat>* feature);

// 0x1801747e0  (upstream-identical, getNodeID inlined with levelup = 0)
void createBOW(const std::vector<cv::Mat>& feature, const OrbVocabulary& voc,
               DBoW2::BowVector* v, std::vector<int>* node_id);

// 0x180174480  (upstream-identical)
double compareBOWs(const DBoW2::BowVector& v1, const DBoW2::BowVector& v2,
                   const OrbVocabulary& voc);

// 0x180176330  (upstream-identical)
void getNodeID(const std::vector<cv::Mat>& feature, const OrbVocabulary& voc,
               const int levelup, std::vector<int>* node_ids);

// 0x180175160  (upstream-modified: bearing/depth/featuretype/original-index outputs removed,
// pointer checks, duplicate-keypoint hashing, keypoint re-association via class_id)
void extractFeaturesFromSVOKeypoints(const cv::Mat& image,
                                     std::vector<cv::Point3f>* svo_landmarks,
                                     std::vector<int>* svo_landmark_ids,
                                     std::vector<int>* svo_trackIDs,
                                     std::vector<cv::Point2f>* svo_keypoints,
                                     std::vector<cv::Mat>* svo_features,
                                     cv::Mat* svo_descriptors);

// 0x180174300  (upstream-modified: reserve + push_back, appends instead of resize)
void changeStructure(const cv::Mat& plain, std::vector<cv::Mat>* out);

}  // namespace totem
}  // namespace pimax
