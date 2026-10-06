// pimax_slam.pi.dll -- src/common/frame.cpp  (from the c05_marginalization draft)
//
// Object ~0x180090180..0x180096E90 (frame.obj).  Port of svo_common/src/frame.cpp, edited to match
// the binary.  No CHECK/LOG strings except LOGW("Frame %d has no obs!\n").
// Library code emitted in this object (not reconstructed): std::vector<cv::Mat> helpers
// (0x180090180..0x1800906D0, 0x180090F90, 0x1800936A0, 0x1800939C0, 0x180093A80, 0x180093B40),
// std::nth_element<float> (0x180090940, 0x180090AD0), vector resize helpers for SeedRef /
// PointPtr / FeatureType (0x180090DB0, 0x1800910D0, 0x180091280), Eigen conservativeResize
// kernels (0x1800913E0, 0x180096000, 0x180096260, 0x1800964C0, 0x180093060), Eigen fill/copy
// kernels, minkindr float transforms (0x1800932D0, 0x180095ED0), the std::function /
// cv::ParallelLoopBodyLambdaWrapper glue for the FrameBundle lambda (0x180093020..0x180093A70),
// std::unordered_map rehash (0x180091430/0x1800937E0) and vk::getMedian<float> (0x180091700).
#include "common/frame.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vikit/math_utils.h>          // vk::getMedian

#include "common/camera.h"
#include "common/logger.h"             // LOGW (0x18000F6A0)
#include "common/point.h"
#include "common/seed.h"

namespace pimax {
namespace totem {

int Frame::frame_counter_ = 0;   // dword_18047DB38

// 0x180091DA0  upstream-modified:
//   * five extra arguments stored in the frame (+0x14, +0x18, +0x1C, +0x188 (copy), +0x180);
//   * initFrame() inlined without its CHECKs and without the colour-image branch.
// Called only from make_shared<Frame>(...) in 0x1800FB650 (frontend) with
//   (cam_i, images[i], timestamp, n_pyr_levels, i, exposure[i], gain[i], grid occupancy
//    (FrameProcessorBase +0xC08), grid size (FrameProcessorBase +0x110)).
Frame::Frame(
    const CameraPtr& cam,
    const cv::Mat& img,
    const int64_t timestamp_ns,
    const size_t n_pyr_levels,
    const int cam_index,
    const int exposure_time,
    const int gain,
    const std::vector<bool>& grid_occupancy,
    const uint16_t grid_cell_size)
  : id_(frame_counter_++) // TEMPORARY
  , cam_index_(cam_index)
  , exposure_time_(exposure_time)
  , gain_(gain)
  , cam_(cam)
  , key_pts_(5, std::make_pair(-1, Position::Zero()))
  , timestamp_(timestamp_ns)
  , grid_cell_size_(grid_cell_size)
  , grid_occupancy_(grid_occupancy)
{
  initFrame(img, n_pyr_levels);
}

// 0x180092420  upstream-identical
Frame::Frame(
    const int id,
    const int64_t timestamp_ns,
    const CameraPtr& cam,
    const Transformation& T_world_cam)
  : id_(id)
  , cam_(cam)
  , T_f_w_(T_world_cam.inverse())
  , key_pts_(5, std::make_pair(-1, Position::Zero()))
  , timestamp_(timestamp_ns)
{}

// 0x180092D60  upstream-modified: the image pyramid is released and its storage freed
// explicitly (release() of every level, clear(), shrink_to_fit()) before the members die.
Frame::~Frame()
{
  for (cv::Mat& img : img_pyr_)
  {
    img.release();
  }
  img_pyr_.clear();
  img_pyr_.shrink_to_fit();
}

// inlined into 0x180091DA0.  upstream-modified: no CHECKs, no CV_8UC3 branch.
void Frame::initFrame(const cv::Mat& img, size_t n_pyr_levels)
{
  frame_utils::createImgPyramid(img, n_pyr_levels, img_pyr_);
  accumulated_w_T_correction_.setIdentity();
}

// 0x180096E80  upstream-identical
void Frame::setKeyframe()
{
  is_keyframe_ = true;
  setKeyPoints();
}

// 0x180094460  upstream-modified: also invalidates the track id.
void Frame::deleteLandmark(const size_t& feature_index)
{
  landmark_vec_.at(feature_index) = nullptr;
  track_id_vec_(feature_index) = -1;
}

// 0x180095730  upstream-modified:
//   * n_new = num - px_vec_.cols()   (upstream: num - num_features_),
//   * extra float 3xN matrix f_vec_raw_ (the SeedRef "extra int (=0)" seen by c05 is the
//     zero-filled padding of the value-initialised element),
//   * no in_ba_graph_vec_, no "Downsizing storage not implemented" error branch.
void Frame::resizeFeatureStorage(size_t num)
{
  if(static_cast<size_t>(px_vec_.cols()) < num)
  {
    const size_t n_new = num - px_vec_.cols();

    px_vec_.conservativeResize(Eigen::NoChange, num);
    f_vec_.conservativeResize(Eigen::NoChange, num);
    f_vec_raw_.conservativeResize(Eigen::NoChange, num);
    score_vec_.conservativeResize(num);
    level_vec_.conservativeResize(num);
    grad_vec_.conservativeResize(Eigen::NoChange, num);
    invmu_sigma2_a_b_vec_.conservativeResize(Eigen::NoChange, num);
    track_id_vec_.conservativeResize(num);

    type_vec_.resize(num, FeatureType::kCorner);
    landmark_vec_.resize(num, nullptr);
    seed_ref_vec_.resize(num);

    // initial values
    level_vec_.tail(n_new).setZero();
    track_id_vec_.tail(n_new).setConstant(-1);
    score_vec_.tail(n_new).setConstant(-1);
  }
}

// 0x180094580  upstream-modified: no CHECK_LT, extra Ref to f_vec_raw_.
FeatureWrapper Frame::getFeatureWrapper(size_t index)
{
  return FeatureWrapper(
        type_vec_[index], px_vec_.col(index), f_vec_.col(index), f_vec_raw_.col(index),
        grad_vec_.col(index), score_vec_(index), level_vec_(index), landmark_vec_[index],
        seed_ref_vec_[index], track_id_vec_(index));
}

// 0x180094550  upstream-identical
FeatureWrapper Frame::getEmptyFeatureWrapper()
{
  return getFeatureWrapper(num_features_);
}

// 0x180096730  upstream-modified:
//   * centre = principal point (cam_->getIntrinsicParameters()(2), (3)) as float,
//     upstream imageWidth()/2, imageHeight()/2;
//   * a feature is skipped when its track id is -1 or it is an outlier (upstream: no landmark);
//   * the quadrant tests use cu for u (upstream bug: cv) and the two quadrants with a negative
//     (u-cu)*(v-cv) product keep the more negative product (upstream: always ">").
void Frame::setKeyPoints()
{
  const FloatType cu = cam_->getIntrinsicParameters()(2);
  const FloatType cv = cam_->getIntrinsicParameters()(3);

  for(size_t i = 0; i < num_features_; ++i)
  {
    if(track_id_vec_(i) == -1 || type_vec_[i] == FeatureType::kOutlier)
      continue;

    const FloatType& u = px_vec_(0,i);
    const FloatType& v = px_vec_(1,i);

    // center
    if(key_pts_[0].first == -1)
      key_pts_[0] = std::make_pair(i, landmark_vec_[i]->pos_);

    else if(std::max(std::fabs(u-cu), std::fabs(v-cv))
            < std::max(std::fabs(px_vec_(0, key_pts_[0].first) - cu),
                       std::fabs(px_vec_(1, key_pts_[0].first) - cv)))
      key_pts_[0] = std::make_pair(i, landmark_vec_[i]->pos_);

    // corner
    if(u >= cu)
    {
      if(v >= cv)
      {
        if(key_pts_[1].first == -1)
          key_pts_[1] = std::make_pair(i, landmark_vec_[i]->pos_);
        else if((u-cu) * (v-cv)
                > (px_vec_(0, key_pts_[1].first) - cu) * (px_vec_(1, key_pts_[1].first)-cv))
          key_pts_[1] = std::make_pair(i, landmark_vec_[i]->pos_);
      }
      else
      {
        if(key_pts_[2].first == -1)
          key_pts_[2] = std::make_pair(i, landmark_vec_[i]->pos_);
        else if((px_vec_(0, key_pts_[2].first) - cu) * (px_vec_(1, key_pts_[2].first)-cv)
                > (u-cu) * (v-cv))
          key_pts_[2] = std::make_pair(i, landmark_vec_[i]->pos_);
      }
    }
    else
    {
      if(v < cv)
      {
        if(key_pts_[3].first == -1)
          key_pts_[3] = std::make_pair(i, landmark_vec_[i]->pos_);
        else if((u-cu) * (v-cv)
                > (px_vec_(0, key_pts_[3].first) - cu) * (px_vec_(1, key_pts_[3].first)-cv))
          key_pts_[3] = std::make_pair(i, landmark_vec_[i]->pos_);
      }
      else
      {
        if(key_pts_[4].first == -1)
          key_pts_[4] = std::make_pair(i, landmark_vec_[i]->pos_);
        else if((px_vec_(0, key_pts_[4].first) - cu) * (px_vec_(1, key_pts_[4].first)-cv)
                > (u-cu) * (v-cv))
          key_pts_[4] = std::make_pair(i, landmark_vec_[i]->pos_);
      }
    }
  }
}

// 0x180094FB0  upstream-modified: replaces the pinhole field-of-view test and
// ProjectionResult::isKeypointVisible() by z >= 0 and hard-coded 640x480 bounds with a 16 px
// border: 16 < u < 624, 16 < v < 464.
bool Frame::isVisible(const Eigen::Vector3d& xyz_w,
                      Eigen::Vector2d* px) const
{
  Eigen::Vector3d xyz_f = T_f_w_*xyz_w;
  if (xyz_f.z() < 0.0)
  {
    return false;
  }

  if (px)
  {
    cam_->project3(xyz_f, px);
    if ((*px)[0] >= 624.0 || (*px)[0] <= 16.0)
      return false;
    return (*px)[1] < 464.0 && (*px)[1] > 16.0;
  }
  else
  {
    Eigen::Vector2d px_temp;
    cam_->project3(xyz_f, &px_temp);
    if (px_temp[0] >= 624.0 || px_temp[0] <= 16.0)
      return false;
    return px_temp[1] < 464.0 && px_temp[1] > 16.0;
  }
}

// 0x180094EC0  pimax-new (c06: "isSaturated").  5x5 window (clipped to the image) of img_pyr_[0] around px; returns
// true when more than 4/5 of the pixels are > 220.
bool Frame::isSaturatedPatch(const Eigen::Vector2i& px) const
{
  const cv::Mat& img = img_pyr_[0];
  const int x_min = std::max(px[0] - 2, 0);
  const int x_max = std::min(px[0] + 2, img.cols - 1);
  const int y_min = std::max(px[1] - 2, 0);
  const int y_max = std::min(px[1] + 2, img.rows - 1);
  int n_total = 0;
  int n_bright = 0;
  for (int y = y_min; y <= y_max; ++y)
  {
    for (int x = x_min; x <= x_max; ++x)
    {
      ++n_total;
      if (img.at<uint8_t>(y, x) > 220)
        ++n_bright;
    }
  }
  return 5 * n_bright > 4 * n_total;
}

// 0x180095450  pimax-new (inline helper emitted out of line).
Eigen::Vector2d Frame::w2c(const Eigen::Vector3d& xyz_w, Eigen::Vector2d* px) const
{
  const Eigen::Vector3d xyz_f = T_f_w_ * xyz_w;
  if (px)
  {
    cam_->project3(xyz_f, px);
    return *px;
  }
  Eigen::Vector2d px_temp;
  cam_->project3(xyz_f, &px_temp);
  return px_temp;
}

// 0x1800955C0  pimax-new (inline helper emitted out of line).
Eigen::Vector2d Frame::f2c(const Eigen::Vector3d& f, Eigen::Vector2d* px) const
{
  if (px)
  {
    cam_->project3(f, px);
    return *px;
  }
  Eigen::Vector2d px_temp;
  cam_->project3(f, &px_temp);
  return px_temp;
}

// 0x180094880  upstream-identical (camera mask member at Camera+0x38)
const cv::Mat& Frame::getMask() const
{
  return cam_->getMask();
}

// 0x180094570  upstream-identical (vk camera vtable slot 5, +0x28; caller PoseOptimizer::run)
// (c05 had the two addresses swapped; the vtable offsets and the callers decide.)
double Frame::getErrorMultiplier() const
{
  return cam_->errorMultiplier();
}

// 0x180094540  upstream-identical (vk camera vtable slot 8, +0x40; caller updateSeed)
double Frame::getAngleError(double img_err) const
{
  return cam_->getAngleError(img_err);
}

// 0x1800928D0  upstream-modified:
//   * bundle id from a function-static counter (dword_18047DB48) as upstream,
//   * the per-frame loop runs in cv::parallel_for_ and, besides setting the bundle id,
//     blurs img_pyr_[0] IN PLACE with a 3x3 box filter (the cv::Mat copy shares the buffer --
//     quirk preserved) and stores its mean intensity and the too-dark (<25) / too-bright (>220)
//     flags in the frame.
// Lambda body: 0x1800933E0 (_Func_impl::_Do_call thunk 0x1800937D0).
FrameBundle::FrameBundle(const std::vector<FramePtr>& frames)
  : frames_(frames)
{
  static BundleId bundle_counter = 0;   // dword_18047DB48
  bundle_id_ = bundle_counter++;
  cv::parallel_for_(cv::Range(0, static_cast<int>(frames.size())),
                    [&](const cv::Range& range)
  {
    for (int i = range.start; i < range.end; ++i)
    {
      FramePtr frame = frames[i];
      frame->bundle_id_ = bundle_id_;
      cv::Mat img = frame->img_pyr_[0];
      cv::blur(img, img, cv::Size(3, 3), cv::Point(-1, -1), cv::BORDER_DEFAULT);
      const double mean = cv::mean(img, cv::noArray())[0];
      frame->mean_intensity_ = mean;
      frame->is_too_dark_ = mean < 25.0;
      frame->is_too_bright_ = mean > 220.0;
    }
  });
}

// 0x180095150  upstream-identical
size_t FrameBundle::numFeatures() const
{
  size_t n = 0;
  for(const FramePtr& f : frames_)
    n += f->numFeatures();
  return n;
}

// 0x180095180  upstream-modified: Frame::numLandmarks() (inlined) counts features with an
// assigned track id in this fork.
size_t FrameBundle::numLandmarks() const
{
  size_t n = 0;
  for(const FramePtr& f : frames_)
    n += f->numLandmarks();
  return n;
}

// 0x180095240  upstream-identical (Frame::numLandmarksInBA inlined; Point::in_ba_graph_ @+0x80)
size_t FrameBundle::numLandmarksInBA() const
{
  size_t n = 0;
  for(const FramePtr& f : frames_)
    n += f->numLandmarksInBA();
  return n;
}

// 0x1800952D0  pimax-new (Frame::numTrackedEdgelets inlined).  The binary tests
//   landmark != nullptr && (type & 0xF9) == 0 && type != 2,  i.e. type in {0, 4, 6}.
// Name from the in-object (decorated-name) order: between numLandmarksInBA and
// numTrackedFeatures.  c05/c10 called it numTrackedLandmarks.  TODO(verify) name.
size_t FrameBundle::numTrackedEdgelets() const
{
  size_t n = 0;
  for(const FramePtr& f : frames_)
    n += f->numTrackedEdgelets();
  return n;
}

// 0x180095360  upstream-modified (Frame::numTrackedFeatures inlined): landmark != nullptr
// || isCornerEdgeletSeed(type)  (the fixed-landmark / map-point exclusions are gone).
size_t FrameBundle::numTrackedFeatures() const
{
  size_t n = 0;
  for(const FramePtr& f : frames_)
    n += f->numTrackedFeatures();
  return n;
}

// 0x1800953F0  upstream-modified (Frame::numTrackedLandmarks inlined): counts the non-null
// entries of the first num_features_ slots (no fixed-landmark / map-point exclusions).
// c05 called it numLandmarks, c10 numLandmarks.
size_t FrameBundle::numTrackedLandmarks() const
{
  size_t n = 0;
  for(const FramePtr& f : frames_)
    n += f->numTrackedLandmarks();
  return n;
}

namespace frame_utils {

// 0x1800942B0  upstream-modified: no CHECKs, cv::pyrDown (default size / BORDER_DEFAULT) into a
// temporary which is then assigned (upstream: vk::halfSample).
void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr)
{
  pyr.resize(n_levels);
  pyr[0] = img_level_0;
  for(int i=1; i<n_levels; ++i)
  {
    cv::Mat dst;
    cv::pyrDown(pyr[i-1], dst);
    pyr[i] = dst;
  }
}

// 0x180094890  upstream-modified (rewritten):
//   * returns false immediately for dark frames (mean_intensity_ < 15);
//   * float arithmetic; landmarks: distance of T_cam_world * pos (only if 0 < z <= 10);
//     seeds (track id <= -1 and a keyframe): distance between the seed's world position
//     (kf T_world_cam * f * 1/mu) and the camera centre, without a z test;
//   * result is stored in the frame: min_depth_ (+0x1A8) and median_depth_ (+0x1AC);
//   * LOGW("Frame %d has no obs!\n", id) instead of SVO_WARN_STREAM.
bool getSceneDepth(const FramePtr& frame)
{
  if (frame->mean_intensity_ < 15.0)
    return false;

  std::vector<float> depth_vec;
  depth_vec.reserve(frame->num_features_);
  float depth_min = std::numeric_limits<float>::max();
  const Eigen::Vector3f cam_pos = frame->T_world_cam().getPosition().cast<float>();
  const TransformationF T_cam_world = frame->T_cam_world().cast<float>();

  for(size_t i = 0; i < frame->num_features_; ++i)
  {
    if(frame->track_id_vec_(i) > -1)
    {
      const Eigen::Vector3f xyz_cam = T_cam_world * frame->landmark_vec_[i]->pos_;
      const float depth = xyz_cam.norm();
      if (xyz_cam.z() > 0 && xyz_cam.z() <= 10.0f)
      {
        depth_vec.push_back(depth);
        depth_min = std::min(depth, depth_min);
      }
    }
    else if(frame->seed_ref_vec_[i].keyframe)
    {
      const SeedRef& seed_ref = frame->seed_ref_vec_[i];
      const auto seed = seed_ref.keyframe->invmu_sigma2_a_b_vec_.col(seed_ref.seed_id);
      const double depth_kf = 1.0 / seed(0);   // seed(0) = inverse depth mu (float)
      // float bearing * double depth, rounded back to float per component
      const Eigen::Vector3f pos_in_kf =
          (seed_ref.keyframe->f_vec_.col(seed_ref.seed_id).cast<double>() * depth_kf).cast<float>();
      const Eigen::Vector3f pos_world =
          seed_ref.keyframe->T_world_cam().cast<float>() * pos_in_kf;
      const float depth = (pos_world - cam_pos).norm();
      depth_vec.push_back(depth);
      depth_min = std::min(depth, depth_min);
    }
  }

  if(depth_vec.empty())
  {
    LOGW("Frame %d has no obs!\n", frame->id_);
    return false;
  }
  frame->min_depth_ = depth_min;
  frame->median_depth_ = vk::getMedian(depth_vec);
  return true;
}

// 0x180093BB0  upstream-modified:
//   * Keypoints / Bearings are float: px_vec is converted to double for
//     cam.backProject3(), the result converted back to float;
//   * the un-normalised float bearings are also returned (f_vec_raw);
//   * normalisation divides every column by its norm (colwise().norm()).
void computeNormalizedBearingVectors(
    const Keypoints& px_vec,
    const Camera& cam,
    Bearings* f_vec,
    Bearings* f_vec_raw)
{
  std::vector<bool> success;
  Eigen::Matrix3Xd f_vec_d;
  cam.backProject3(px_vec.cast<double>(), &f_vec_d, &success);
  *f_vec = f_vec_d.cast<float>();
  *f_vec_raw = *f_vec;
  const Eigen::RowVectorXf norms = f_vec->colwise().norm();
  *f_vec = f_vec->array().rowwise() / norms.array();
}

} // namespace frame_utils

} // namespace totem
} // namespace pimax
