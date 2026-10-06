// pimax_slam.pi.dll -- src/common/point.cpp
// Pimax fork of svo_common/src/point.cpp.  point.obj = 0x180097F70 .. 0x18009C6F0: point.obj
// COMDATs, then KeypointIdentifier ctor 0x180099F40 and Point ctor 0x180099F80 (c05 draft
// point_c05_part.cpp), then 0x18009A530 .. 0x18009C6F0 (c06 draft).
//
// Pimax changes vs upstream (see notes/c06_common_depthfilter.md):
//  * FloatType == float: pos_ is Vector3f, f_vec_ is Matrix3Xf; all geometry that upstream
//    did in double is partly done in float here (exact mix reproduced below).
//  * obs_ is std::unordered_map<int /*frame id*/, KeypointIdentifier>.
//  * glog CHECK/SVO_*_STREAM replaced by the Pimax logger (LOGD/LOGE).

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include <Eigen/Cholesky>
#include <vikit/math_utils.h>       // vk::project2, vk::norm_max

#include "common/logger.h"          // LOGD/LOGI/LOGW/LOGE (global logger 0x18046A000)
#include "common/point.h"
#include "common/frame.h"

namespace pimax {
namespace totem {

std::atomic<int> PointIdProvider::last_id_ { 0 };   // 0x18047DB60

// 0x180099F40  upstream-modified: also stores the frame's bundle id (+0x14).
// (also inlined into Point::addObservation 0x18009A530)
KeypointIdentifier::KeypointIdentifier(const FramePtr& _frame, const size_t _feature_index)
  : frame(_frame)                     // std::weak_ptr<Frame>   +0x00
  , frame_id(_frame->id_)             // int                    +0x10
  , bundle_id(_frame->bundle_id_)     // int  [pimax-new]       +0x14  TODO(verify) name
  , keypoint_index_(_feature_index)   // size_t                 +0x18
{ ; }

// 0x180099F80  upstream-modified (Pimax Point layout, float position):
// Point(pos) : Point(PointIdProvider::getNewPointId(), pos) with the delegating ctor inlined
// (id_ = lock xadd 0x18047DB60, members: see point.h).
Point::Point(const Position& pos)
  : Point(PointIdProvider::getNewPointId(), pos)
{ ; }

Point::Point(const int id, const Position& pos)
  : id_(id)
  , pos_(pos)
{ ; }

// 0x18009A530
void Point::addObservation(const FramePtr& frame, const size_t feature_index)
{
  if (frame == nullptr)
  {
    LOGE("addObservation frame NULL\n");
    return;
  }

  // check that we don't have yet a reference to this frame
  const int id = frame->id_;
  if (obs_.find(id) == obs_.end())
  {
    obs_.insert(std::make_pair(id, KeypointIdentifier(frame, feature_index)));
  }
  // Pimax: upstream's CHECK_EQ(it->keypoint_index_, feature_index) for an existing entry is gone.
}

// 0x18009BBE0  (unordered_map<int,KeypointIdentifier>::erase(key) fully inlined)
void Point::removeObservation(int frame_id)
{
  obs_.erase(frame_id);
}

// 0x18009A780
bool Point::getCloseViewObs(
    const Eigen::Vector3d& framepos,
    FramePtr& ref_frame,
    size_t& ref_feature_index) const
{
  double min_cos_angle = -1.1;                        // Pimax: upstream 0.0
  Eigen::Vector3f obs_dir(framepos.cast<float>() - pos_);
  obs_dir.normalize();

  for (const auto& obs : obs_)
  {
    // Pimax: a frame that cannot be locked is silently skipped (upstream logged and returned false).
    if (FramePtr frame = obs.second.frame.lock())
    {
      Eigen::Vector3f dir(frame->pos() - pos_);       // Frame::pos() returns Vector3f in Pimax
      dir.normalize();
      const double cos_angle = obs_dir.dot(dir);      // float dot, widened to double
      if (cos_angle > min_cos_angle)
      {
        min_cos_angle = cos_angle;
        ref_frame = frame;
        ref_feature_index = obs.second.keypoint_index_;
      }
    }
  }
  if (min_cos_angle < 0.5)                            // Pimax: upstream 0.4
  {
    LOGD("getCloseViewObs(): obs is from too far away: %f\n", min_cos_angle);
    return false;
  }
  return true;
}

// 0x18009AA50
double Point::getTriangulationParallax() const
{
  // Pimax: no CHECK(!obs_.empty()); begin() of an empty map is dereferenced (UB in the original too).
  const FramePtr ref_frame = obs_.begin()->second.frame.lock();
  if (!ref_frame)
  {
    LOGE("getTriangualtionParallax(): Could not lock ref_frame\n");
    return 0.0;
  }

  const Eigen::Vector3d r = (ref_frame->pos() - pos_).cast<double>().normalized();
  // Pimax: returns acos of the smallest |cos| (upstream: max of acos(cos)).
  double min_cos = 1.0;
  for (const auto& obs : obs_)
  {
    if (const FramePtr frame = obs.second.frame.lock())
    {
      const Eigen::Vector3d v = (frame->pos() - pos_).cast<double>().normalized();
      min_cos = std::min(std::fabs(r.dot(v)), min_cos);
    }
  }
  return std::acos(std::max(0.0, std::min(min_cos, 1.0)));
}

// 0x18009BE10
void Point::updateHessianGradientUnitPlane(
    const Eigen::Ref<BearingVector>& f,
    const Eigen::Vector3d& p_in_f,
    const Eigen::Matrix3d& R_f_w,
    Eigen::Matrix3d& A,
    Eigen::Vector3d& b,
    double& new_chi2)
{
  Matrix23d J;
  Point::jacobian_xyz2uv(p_in_f, R_f_w, J);
  // f is float: f[0]/f[2], f[1]/f[2] are computed in float, then widened.
  const Eigen::Vector2d e(vk::project2(f).cast<double>() - vk::project2(p_in_f));
  A.noalias() += J.transpose() * J;
  b.noalias() -= J.transpose() * e;
  new_chi2 += e.squaredNorm();
}

// 0x18009C130
void Point::updateHessianGradientUnitSphere(
    const Eigen::Ref<BearingVector>& f,
    const Eigen::Vector3d& p_in_f,
    const Eigen::Matrix3d& R_f_w,
    Eigen::Matrix3d& A,
    Eigen::Vector3d& b,
    double& new_chi2)
{
  Eigen::Matrix3d J;
  Point::jacobian_xyz2f(p_in_f, R_f_w, J);
  const Eigen::Vector3d e = f.cast<double>() - p_in_f.normalized();
  A.noalias() += J.transpose() * J;
  b.noalias() -= J.transpose() * e;
  new_chi2 += e.squaredNorm();
}

// 0x18009B240
void Point::optimize(const size_t n_iter, bool using_bearing_vector)
{
  if (obs_.size() < 2)
  {
    LOGE("optimizing point with less than two observations");
    return;
  }

  Eigen::Vector3d old_point = pos_.cast<double>();
  double chi2 = 0.0;

  // Pimax: lock every observing frame once up-front (vector element = 24 bytes).
  std::vector<std::pair<FramePtr, size_t>> frames;
  frames.reserve(obs_.size());
  for (const auto& obs : obs_)
  {
    if (const FramePtr frame = obs.second.frame.lock())
    {
      frames.push_back(std::make_pair(frame, obs.second.keypoint_index_));
    }
    else
    {
      LOGE("could not unlock weak_ptr<Frame> in Point::optimize\n");
    }
  }

  if (frames.size() < 2)
  {
    LOGE("optimizing point with less than two valid observations");
    return;
  }

  const double eps = 0.0000000001;
  for (size_t i = 0; i < n_iter; i++)
  {
    Eigen::Matrix3d A;
    Eigen::Vector3d b;
    A.setZero();
    b.setZero();
    double new_chi2 = 0.0;
    const Eigen::Vector3d pos = pos_.cast<double>();

    // compute residuals
    for (const std::pair<FramePtr, size_t>& obs : frames)
    {
      const FramePtr& frame = obs.first;
      if (using_bearing_vector)
      {
        updateHessianGradientUnitSphere(
              frame->f_vec_.col(obs.second), frame->T_f_w_ * pos,
              frame->T_f_w_.getRotation().getRotationMatrix(),
              A, b, new_chi2);
      }
      else
      {
        updateHessianGradientUnitPlane(
              frame->f_vec_.col(obs.second), frame->T_f_w_ * pos,
              frame->T_f_w_.getRotation().getRotationMatrix(),
              A, b, new_chi2);
      }
    }

    // solve linear system
    const Eigen::Vector3d dp(A.ldlt().solve(b));

    // check if error increased
    // Pimax: upstream's std::isnan(dp[0]) test is replaced by a step-size test.
    if ((i > 0 && new_chi2 > chi2) || dp.squaredNorm() > 1.0)
    {
      pos_ = old_point.cast<float>(); // roll-back
      break;
    }

    // update the model
    Eigen::Vector3d new_point = pos_.cast<double>() + dp;
    old_point = pos_.cast<double>();
    pos_ = new_point.cast<float>();
    chi2 = new_chi2;

    // stop when converged
    if (vk::norm_max(dp) <= eps)      // norm_max(const VectorXd&): dp is copied to a VectorXd
      break;
  }
}

} // namespace totem
} // namespace pimax
