// pimax_slam.pi.dll -- src/frontend/frame_processor_base_ground.cpp  (phase B2; draft c08)
//
// Part of frontend/frame_processor_base.cpp in the original tree (same object
// frame_processor_base.obj, 0x1800EC2B0 and 0x1800F19D0..0x1800F64A0); split out for size only.
// The mesh, plane and ground-plane members of FrameProcessorBase.  All of it is pimax-new; the
// plane association test is Kimera-VIO's Plane::geometricEqual (UtilsOpenCV.h) without its
// CHECKs, the 2-D Delaunay on keypoints is the cv::Subdiv2D variant of Kimera's createMesh2D.
//
// Integration (B2) changes to the c08 draft: plane/plane_types.h -> plane/plane.h + plane/mesher.h
// (A1), Mesher::populate3dMesh -> populate3dMeshTimeHorizon (public, 0x18019DB90) with a Mesh2D,
// polygonArea -> computePolygonArea (0x1800FE140, frame_processor_base.cpp),
// g_wall_distance_tolerance -> g_distance_tolerance_polygon_plane_association (0x18046A2B8),
// file-static depthInFrame -> cameraDepth (avoids an overload with ceres_backend's
// depthInFrame(TransformationF, Vector3f) 0x18008ACB0, a different (float) function).
//
// External: Estimator::setGroundPlaneConstraint 0x18002C530 / resetGroundPlaneConstraint
// 0x1800298A0 (on CeresBackendInterface::backend_ +176), isFinite(const Vector3d&) 0x180027A20.
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <opencv2/imgproc.hpp>

#include "ceres_backend/ceres_backend_interface.hpp"
#include "ceres_backend/estimator.hpp"          // GroundPlaneConstraint
#include "ceres_backend/estimator_types.hpp"    // isFinite
#include "common/frame.h"
#include "common/logger.h"
#include "common/point.h"
#include "frontend/frame_processor_base.h"
#include "plane/mesh.h"
#include "plane/mesher.h"
#include "plane/plane.h"

namespace pimax {
namespace totem {

namespace {

// 0x1800F19D0 -- median by value; NaN (0x7FF8000000000000) for an empty vector. std::sort
// (0x1800BFB10 = _Sort_unchecked), not nth_element; even count -> mean of the two middles.
double median(std::vector<double> values)
{
  if (values.empty())
    return std::numeric_limits<double>::quiet_NaN();
  std::sort(values.begin(), values.end());
  const size_t mid = values.size() / 2;
  double m = values[mid];
  if ((values.size() & 1) == 0)
    m = (m + values[mid - 1]) * 0.5;
  return m;
}

// z component of T_f_w * p_w as the binary computes it (0x1800F2520..0x1800F258D):
//   (1 - 2(qy^2 + qx^2)) * pz + 2((qx qz - qy qw) px + (qw qx + qy qz) py) + tz
// i.e. row 2 of the rotation matrix with the factors 2 pulled out (results identical to
// Eigen's toRotationMatrix() row 2 dot p). TODO(verify) original spelling, most likely
// `(frame.T_f_w_.getRotationMatrix() * p.cast<double>() + frame.T_f_w_.getPosition()).z()`.
double cameraDepth(const Transformation& T_f_w, const Eigen::Vector3f& p_w)
{
  const Eigen::Quaterniond& q = T_f_w.getRotation().toImplementation();
  const double px = p_w.x(), py = p_w.y(), pz = p_w.z();
  const double qx = q.x(), qy = q.y(), qz = q.z(), qw = q.w();
  double a = (qx * qz - qy * qw) * px + (qw * qx + qy * qz) * py;
  double s = qy * qy + qx * qx;
  s = s + s;
  a = a + a;
  return (1.0 - s) * pz + a + T_f_w.getPosition().z();
}

}  // namespace

// =============================================================================================
// 0x1800F2070 -- pimax-new. Builds a 2-D Delaunay mesh on the corner features of `frame` that
// have a landmark in front of the camera (0 < depth <= 10 m) and below the IMU, lifts it to 3-D
// in the Kimera mesher, re-clusters planes and refreshes the ground-plane model.
// Called from addFrameBundle (0x1800FB650) with *last_last_frames_->at(0) while
// mesh_enabled_ (+3720) and mesh_update_count_ (+3724) < 500.
void FrameProcessorBase::updateGroundPlane(const Frame& frame)
{
  LmkPixelMap lmk_px;                      // max_load_factor 1.0, 8 buckets
  std::vector<int> triangle_lmk_ids;       // 3 ids per kept triangle
  std::vector<cv::Vec6f> triangles;
  // Rect from the float ROI by truncation (cvttss2si on height, width, y, x)
  cv::Subdiv2D subdiv(cv::Rect(static_cast<int>(mesh_roi_.x), static_cast<int>(mesh_roi_.y),
                               static_cast<int>(mesh_roi_.width), static_cast<int>(mesh_roi_.height)));
  std::vector<cv::Point2f> points;
  std::vector<int> point_ids;
  LmkPositionMap lmk_positions;

  mesher_->R_w_c_ = frame.T_f_w_.inverse().getRotationMatrix().cast<float>();
  mesher_->t_w_c_ = frame.T_f_w_.inverse().getPosition().cast<float>();

  for (size_t i = 0; i < frame.num_features_; ++i)
  {
    const auto px = frame.px_vec_.col(i);
    int track_id = frame.track_id_vec_(i);
    if (track_id < 0)
      continue;
    const PointPtr& landmark = frame.landmark_vec_[i];
    if (!landmark)
      continue;
    if (frame.type_vec_[i] != FeatureType::kCorner)
      continue;
    const cv::Point2f pt(px(0), px(1));
    if (!mesh_roi_.contains(pt))
      continue;
    const double depth = cameraDepth(frame.T_f_w_, landmark->pos_);
    if (depth <= 0.0 || depth > 10.0)          // NaN passes (comisd/jnb, comisd/ja in the binary)
      continue;
    if (landmark->pos_.z() > frame.imuPos().z())   // only points below the IMU
      continue;

    const Eigen::Vector3f pos = frame.landmark_vec_[i]->pos_;
    points.push_back(pt);
    point_ids.push_back(track_id);
    lmk_px.emplace(track_id, px.cast<double>());
    lmk_positions.emplace(track_id, pos);
    lmk_points_[track_id] = pos;
  }
  mesher_->lmk_px_ = lmk_px;

  try
  {
    subdiv.insert(points);
  }
  catch (...)
  {
  }
  subdiv.getTriangleList(triangles);

  // keep the triangles whose three corners are inside the ROI and map back to known points
  for (auto it = triangles.begin(); it != triangles.end();)
  {
    const cv::Vec6f& t = *it;
    if (!mesh_roi_.contains(cv::Point2f(t[0], t[1])) || !mesh_roi_.contains(cv::Point2f(t[2], t[3])) ||
        !mesh_roi_.contains(cv::Point2f(t[4], t[5])))
    {
      it = triangles.erase(it);
      continue;
    }
    int ids[3];
    bool ok = true;
    for (int k = 0; k < 3; ++k)
    {
      const cv::Point2f p(t[2 * k], t[2 * k + 1]);
      const int vertex = subdiv.findNearest(p) - 4;   // first 4 Subdiv2D vertices are virtual
      if (vertex >= 0 && static_cast<size_t>(vertex) < points.size())
      {
        const float dx = points[vertex].x - p.x;
        const float dy = points[vertex].y - p.y;
        if (dx * dx + dy * dy < 1e-4f)
        {
          ids[k] = point_ids[vertex];
          continue;
        }
      }
      // fallback: brute-force nearest input point
      float best = FLT_MAX;
      int best_idx = -1;
      for (size_t j = 0; j < points.size(); ++j)
      {
        const float dx = points[j].x - p.x;
        const float dy = points[j].y - p.y;
        const float d = dx * dx + dy * dy;
        if (d < best)
        {
          best = d;
          best_idx = static_cast<int>(j);
        }
      }
      if (best_idx >= 0 && best < 1e-4f)
      {
        ids[k] = point_ids[best_idx];
        continue;
      }
      ok = false;
      break;
    }
    if (!ok)
    {
      it = triangles.erase(it);
      continue;
    }
    triangle_lmk_ids.insert(triangle_lmk_ids.end(), ids, ids + 3);
    ++it;
  }

  for (const auto& kv : lmk_px)
  {
    auto found = lmk_points_.find(kv.first);
    if (found != lmk_points_.end())
      lmk_positions.insert(*found);
  }

  Mesh2D mesh(3);   // 0x1801996D0 (Mesh2D/Mesh3D ctor, COMDAT-folded)
  if (!mesher_->populate3dMeshTimeHorizon(triangles, triangle_lmk_ids, lmk_positions,
                                          g_min_ratio_btw_largest_smallest_side,
                                          g_min_elongation_ratio,
                                          g_max_triangle_side * 0.5, &mesh))   // 0x18019DB90
    return;

  mesher_->clusterPlanesFromMesh(&planes_, &other_planes_, lmk_positions);   // 0x1801A0AF0
  refinePlanes(planes_);
  mergeSimilarPlanes(planes_);

  double max_area = 0.0;
  for (Plane& plane : planes_)
  {
    plane.area_ = computePolygonArea(plane);   // 0x1800FE140
    max_area = std::max(max_area, plane.area_);
  }
  selectGroundPlane();
  if (max_area > 1.5 && ground_valid_)
    mesh_enabled_ = false;
}

// =============================================================================================
// 0x1800F3430 -- pimax-new. Per-plane refinement after clustering:
//  * horizontal planes (cluster 2): robust running mean of the landmark heights (z) of
//    all_lmk_ids_; walls (cluster 1): mean of n.p, or a 2-D line re-fit when flagged;
//    the plane distance is replaced by the mean when it moved more than a view-dependent gate;
//  * hull: polygon vertices are refreshed from lmk_points_ (unless > 0.4 m off the plane) and,
//    for non-walls, the convex hull of the rotated vertices replaces polygon_.
void FrameProcessorBase::refinePlanes(std::vector<Plane>& planes)
{
  // ids of planes created by the mesher since the last call (global vector 0x18047EF68,
  // g_new_plane_ids; c08 drafted the 0.15 global 0x18046A2B8 as g_wall_distance_tolerance)
  for (int id : g_new_plane_ids)
    g_max_plane_id = std::max(g_max_plane_id, id);
  g_new_plane_ids.clear();

  for (Plane& plane : planes)
  {
    if (plane.age_ > 5 && std::fabs(plane.distance_ref_ - 0.0) < 0.001)
      plane.distance_ref_ = plane.distance_;

    double sum = 0.0;
    double mean = 0.0;
    int n = 0;
    const Eigen::Vector3f normal(plane.normal_.x, plane.normal_.y, plane.normal_.z);
    if (plane.cluster_id_ == 2)
    {
      for (int id : plane.all_lmk_ids_)
      {
        auto it = lmk_points_.find(id);
        if (it == lmk_points_.end())
          continue;
        const float z = it->second.z();
        const bool accept = n ? std::fabs(z - mean) <= g_distance_tolerance_polygon_plane_association + 0.15
                              : std::fabs(z - plane.distance_) <= 10.0;
        if (accept)
        {
          sum += z;
          mean = sum / static_cast<double>(++n);
        }
      }
      if (n <= 6)
        continue;
      const double avg = sum / static_cast<double>(n);
      const float proj = mesher_->t_w_c_.dot(normal);
      if (std::fabs(plane.distance_ - avg) >
          (g_plane_distance_update_tolerance - 0.12) * std::min(1.0, std::max(0.5, std::fabs(avg - proj))))
        plane.distance_ = avg;
    }
    else if (plane.cluster_id_ == 1)
    {
      if (!plane.needs_wall_refit_)
      {
        for (int id : plane.all_lmk_ids_)
        {
          auto it = lmk_points_.find(id);
          if (it == lmk_points_.end())
            continue;
          sum += it->second.dot(normal);
          ++n;
        }
        if (n <= 6)
          continue;
        const double avg = sum / static_cast<double>(n);
        const float proj = normal.dot(mesher_->t_w_c_);
        if (std::fabs(plane.distance_ - avg) >
            (g_plane_distance_update_tolerance - 0.12) * std::min(1.0, std::max(0.5, std::fabs(avg - proj))))
          plane.distance_ = avg;
      }
      else
      {
        plane.needs_wall_refit_ = !refitWallPlane(plane);
      }
    }

    // ---- hull refresh ----
    std::vector<cv::Point2f> pts2d;
    std::vector<PolygonVertex> verts;
    const Eigen::Vector3f ez(0.f, 0.f, 1.f);
    const Eigen::Vector3f n2(plane.normal_.x, plane.normal_.y, plane.normal_.z);   // re-read (wall re-fit)
    const Eigen::Matrix3f R = Eigen::Quaternionf::FromTwoVectors(ez, n2).toRotationMatrix();
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();   // accumulated for walls, never used
    bool changed = false;
    for (PolygonVertex& v : plane.polygon_)
    {
      auto it = lmk_points_.find(v.lmk_id);
      if (it == lmk_points_.end())
      {
        const Eigen::Vector3f rp = R * v.pos;
        pts2d.emplace_back(rp.x(), rp.y());
        verts.push_back(v);
      }
      else if (std::fabs(n2.dot(it->second) - plane.distance_) > 0.4)
      {
        const Eigen::Vector3f rp = R * v.pos;
        pts2d.emplace_back(rp.x(), rp.y());
        verts.push_back(v);
      }
      else
      {
        v.pos = it->second;
        const Eigen::Vector3f rp = R * v.pos;
        pts2d.emplace_back(rp.x(), rp.y());
        verts.push_back(v);
        changed = true;
      }
      if (plane.cluster_id_ == 1)
        centroid += v.pos;
    }
    if (plane.cluster_id_ != 1 && changed)
    {
      std::vector<int> hull;
      if (!pts2d.empty())
      {
        cv::convexHull(pts2d, hull, plane.hull_clockwise_);   // returnPoints = true (ignored for int output)
        if (!hull.empty())
        {
          plane.polygon_.clear();
          plane.polygon_.reserve(hull.size());
          for (int idx : hull)
            plane.polygon_.push_back(verts[idx]);
        }
      }
    }
  }
}

// =============================================================================================
// 0x1800F4190 -- pimax-new. Re-fit a vertical plane as a 2-D line in x/y (SVD of the centred
// x/y coordinates of up to 51 landmarks). Updates normal_.x/.y and distance_ BEFORE the outlier
// test, i.e. a rejected fit still leaves the new parameters in the plane (quirk, preserved).
bool FrameProcessorBase::refitWallPlane(Plane& plane)
{
  std::vector<int> ids = plane.lmk_ids_;
  std::vector<Eigen::Vector3f> pts;
  float sx = 0.f, sy = 0.f, sz = 0.f;
  for (size_t i = 0; i < ids.size(); ++i)
  {
    auto it = lmk_points_.find(ids[i]);
    if (it == lmk_points_.end())
      continue;
    pts.push_back(it->second);
    sx += it->second.x();
    sy += it->second.y();
    sz += it->second.z();
    if (pts.size() > 50)
      break;
  }
  if (pts.size() < 15)
    return false;

  const float count = static_cast<float>(pts.size());
  const float cx = sx / count;
  const float cy = sy / count;
  Eigen::Matrix<float, Eigen::Dynamic, 2> A(pts.size(), 2);
  for (size_t i = 0; i < pts.size(); ++i)
  {
    A(i, 0) = pts[i].x() - cx;
    A(i, 1) = pts[i].y() - cy;
  }
  // MatrixX2f -> MatrixXf temporary (resize 0x18011CAB0 + copy), flags 0x14
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  // matrixV()(1,1) is read before (0,1): right-to-left argument evaluation
  float nx = svd.matrixV()(0, 1);
  float ny = svd.matrixV()(1, 1);
  const Eigen::VectorXf sv = svd.singularValues();
  if (sv(0) < 0.3)
    return false;

  float d = -nx * cx - ny * cy;
  if (plane.normal_.x * nx + plane.normal_.y * ny < 0.0f)
  {
    nx = -nx;
    ny = -ny;
  }
  if (d * plane.distance_ < 0.0)
    d = -d;
  plane.normal_.x = nx;
  plane.normal_.y = ny;
  plane.distance_ = d;

  size_t outliers = 0;
  for (int id : ids)
  {
    auto it = lmk_points_.find(id);
    if (it == lmk_points_.end())
      continue;
    const cv::Point3f p(it->second.x(), it->second.y(), it->second.z());
    if (std::fabs(plane.normal_.ddot(p) - d) > 0.2)
      ++outliers;
    if (outliers >= ids.size() / 2)
      return false;
  }
  return true;
}

// =============================================================================================
// 0x1800EC2B0 -- pimax-new (Kimera geometricEqual + hull merge). Merges every later plane of
// the same cluster that is geometrically equal and whose hull overlaps into the earlier one.
// Quirks preserved: the size gate reads the member planes_ (the argument is the same object in
// the only call); walls (cluster 1) are skipped as first plane, so the wall-only branch that
// builds rotated landmark positions is dead in practice.
void FrameProcessorBase::mergeSimilarPlanes(std::vector<Plane>& planes)
{
  if (planes_.size() < 2)
    return;

  LmkPositionMap rotated_lmks;
  for (auto it1 = planes.begin(); it1 != planes.end(); ++it1)
  {
    if (it1 + 1 == planes.end())
      break;
    if (it1->cluster_id_ == 1)
      continue;
    for (auto it2 = it1 + 1; it2 != planes.end();)
    {
      if (it1->cluster_id_ != it2->cluster_id_)
      {
        ++it2;
        continue;
      }
      const Eigen::Vector3f n1(it1->normal_.x, it1->normal_.y, it1->normal_.z);
      const Eigen::Vector3f n2(it2->normal_.x, it2->normal_.y, it2->normal_.z);
      const float proj = n1.dot(mesher_->t_w_c_);
      const double distance_tolerance =
          std::min(1.3, std::max(0.5, std::fabs(proj - it1->distance_))) * g_distance_tolerance_plane_plane;
      // Kimera Plane::geometricEqual(rhs, normal_tol, distance_tol) without the CHECKs
      const bool equal =
          (it1->normal_.ddot(it2->normal_) > 1.0 - g_normal_tolerance_plane_plane &&
           std::fabs(it2->distance_ - it1->distance_) < distance_tolerance) ||
          ((-it1->normal_).ddot(it2->normal_) > 1.0 - g_normal_tolerance_plane_plane &&
           std::fabs(it2->distance_ - (-it1->distance_)) < distance_tolerance);
      if (!equal)
      {
        ++it2;
        continue;
      }

      const Eigen::Vector3f ez(0.f, 0.f, 1.f);
      const Eigen::Matrix3f R1 = Eigen::Quaternionf::FromTwoVectors(n1, ez).toRotationMatrix();
      const Eigen::Matrix3f R2 = Eigen::Quaternionf::FromTwoVectors(n2, ez).toRotationMatrix();
      std::vector<PolygonVertex> poly1 = it1->polygon_;
      std::vector<PolygonVertex> poly2 = it2->polygon_;
      for (PolygonVertex& v : poly1)
        v.pos = R1 * v.pos;
      for (PolygonVertex& v : poly2)
        v.pos = R2 * v.pos;

      if (it1->cluster_id_ == 1)
      {
        rotated_lmks.clear();
        rotated_lmks.reserve(lmk_points_.size());   // 0x18011B240
        for (const auto& kv : lmk_points_)
        {
          Eigen::Vector3f p = kv.second;
          p = R1 * p;
          const std::pair<const int, Eigen::Vector3f> entry(kv.first, p);
          rotated_lmks.insert(entry);
        }
      }
      bool overlap;
      if (it1->cluster_id_ == 1)
        overlap = polygonsOverlap(poly1, it1->lmk_ids_, poly2, it2->lmk_ids_, rotated_lmks);
      else
        overlap = polygonsOverlap(poly1, it1->lmk_ids_, poly2, it2->lmk_ids_, lmk_points_);
      if (!overlap)
      {
        ++it2;
        continue;
      }

      mergePolygons(&it1->polygon_, it2->polygon_, n1, &it1->hull_clockwise_);   // 0x18019CA00
      for (int id : it2->lmk_ids_)
      {
        if (std::find(it1->lmk_ids_.begin(), it1->lmk_ids_.end(), id) == it1->lmk_ids_.end())
          it1->lmk_ids_.push_back(id);
      }
      it2 = planes.erase(it2);
    }
  }
}

// =============================================================================================
// 0x1800F64A0 -- pimax-new. Picks the best horizontal plane as ground and filters the ground
// model (normal +3760, distance +3784, sigma +3792) over up to 5 confirmations.
void FrameProcessorBase::selectGroundPlane()
{
  const Plane* best = nullptr;
  Eigen::Vector3d best_normal(0.0, 0.0, 1.0);
  double best_distance = 0.0;
  double best_gate = 0.08;
  double best_score = 0.0;
  size_t best_inliers = 0;

  LOGD("ground-plane scan planes %lu map_points %lu\n", planes_.size(), lmk_points_.size());
  for (const Plane& plane : planes_)
  {
    LOGD("ground-plane raw id %d cluster %d area %.3f local_ids %lu all_ids %lu hull %lu distance %.4f\n",
         plane.id, plane.cluster_id_, plane.area_, plane.lmk_ids_.size(), plane.all_lmk_ids_.size(),
         plane.polygon_.size(), plane.distance_);
    if (plane.cluster_id_ != 2)
      continue;
    if (plane.area_ < 0.3)
      continue;
    if (plane.polygon_.size() < 3)
      continue;

    Eigen::Vector3d n(plane.normal_.x, plane.normal_.y, plane.normal_.z);
    const double norm = n.norm();
    if (norm <= 1e-6)
      continue;
    if (!std::isfinite(n.x()) || !std::isfinite(n.y()) || !std::isfinite(n.z()))
      continue;
    n /= norm;
    if (n.dot(Eigen::Vector3d::UnitZ()) < 0.98)
      continue;

    std::vector<double> heights;
    heights.reserve(plane.lmk_ids_.size());
    for (int id : plane.lmk_ids_)
    {
      auto it = lmk_points_.find(id);   // 0x1800CFFF0
      if (it == lmk_points_.end())
        continue;
      const double h = it->second.x() * n.x() + it->second.y() * n.y() + it->second.z() * n.z();
      if (std::isfinite(h))
        heights.push_back(h);
    }
    if (heights.size() < 8)
    {
      LOGD("ground-plane reject id %d projections %lu\n", plane.id, heights.size());
      continue;
    }

    const double med = median(heights);
    std::vector<double> abs_dev;
    abs_dev.reserve(heights.size());
    for (double h : heights)
      abs_dev.push_back(std::fabs(h - med));
    const double robust_sigma = median(abs_dev) * 1.4826;
    const double gate = std::min(0.1, std::max(0.03, robust_sigma * 3.0));
    std::vector<double> inliers;
    inliers.reserve(heights.size());
    for (double h : heights)
    {
      if (std::fabs(h - med) <= gate)
        inliers.push_back(h);
    }
    const double ratio = static_cast<double>(inliers.size()) / static_cast<double>(heights.size());
    LOGD("ground-plane quality id %d projections %lu inliers %lu ratio %.3f robust_sigma %.4f gate %.4f\n",
         plane.id, heights.size(), inliers.size(), ratio, robust_sigma, gate);
    if (inliers.size() < 8 || ratio < 0.7 || robust_sigma > 0.05)
      continue;

    const double distance = median(inliers);
    const Eigen::Vector3f& cam = mesher_->t_w_c_;
    const double height = n.x() * cam.x() + n.y() * cam.y() + n.z() * cam.z() - distance;
    LOGD("ground-plane height id %d value %.4f\n", plane.id, height);
    if (height >= 0.2 && height <= 3.0)
    {
      const double candidate_gate = std::min(0.1, std::max(0.05, robust_sigma + robust_sigma));
      const double score = std::sqrt(static_cast<double>(inliers.size())) * plane.area_ /
                           (robust_sigma * 10.0 + 1.0);
      if (!best || score > best_score)
      {
        best = &plane;
        best_normal = n;
        best_distance = distance;
        best_gate = candidate_gate;
        best_score = score;
        best_inliers = inliers.size();
      }
    }
  }

  if (!best)
  {
    if (++ground_miss_count_ > 5)
    {
      ground_valid_ = false;
      ground_confirmations_ = 0;
      ground_polygon_.clear();
    }
    return;
  }

  ground_miss_count_ = 0;
  const double sigma = std::max(ground_sigma_, best_gate);
  const double tolerance = std::max(0.04, sigma * 3.0);   // maxsd
  if (ground_confirmations_ <= 0 || best_normal.dot(ground_normal_) <= 0.995 ||
      tolerance < std::fabs(ground_distance_ - best_distance))
  {
    ground_valid_ = false;
    ground_confirmations_ = 1;
    ground_normal_ = best_normal;
    ground_distance_ = best_distance;
    ground_sigma_ = best_gate;
  }
  else
  {
    const int k = std::min(5, ground_confirmations_ + 1);
    const double alpha = 1.0 / static_cast<double>(k);
    const double beta = 1.0 - alpha;
    ground_normal_ = (beta * ground_normal_ + alpha * best_normal).normalized();
    ground_distance_ = beta * ground_distance_ + alpha * best_distance;
    ground_sigma_ = alpha * best_gate + beta * ground_sigma_;
    ground_confirmations_ = std::min(1000, ground_confirmations_ + 1);
  }
  ground_area_ = best->area_;
  ground_polygon_.clear();
  ground_polygon_.reserve(best->polygon_.size());   // 0x18011B330
  for (const PolygonVertex& v : best->polygon_)
    ground_polygon_.emplace_back(v.pos.x(), v.pos.y());
  const int confirmations = ground_confirmations_;
  const bool valid = confirmations >= 3 && ground_polygon_.size() >= 3;
  ground_valid_ = valid;
  LOGD("ground-plane model valid %d confirmations %d distance %.4f sigma %.4f area %.3f inliers %lu\n",
       valid, confirmations, ground_distance_, ground_sigma_, ground_area_, best_inliers);
}

// =============================================================================================
// 0x1800F5170 -- pimax-new. While a valid ground model with a polygon exists, re-fits the ground
// normal from the closest map points (<= 48) seen in the current bundle and, when the fit is
// consistent with the previous one (angle <= 2 deg, >= 8 shared ids, overlap >= 0.6, different
// bundle), hands a relative ground-plane constraint between the two bundles to the backend.
// Called from addFrameBundle (0x1800FB650) with new_frames_.get() when the backend exists and
// the IMU is initialised and stage is 2/3.
void FrameProcessorBase::estimateGroundPlane(const FrameBundle& frame_bundle)
{
  if (!bundle_adjustment_)
    return;
  if (!ground_valid_ || ground_polygon_.size() < 3)
  {
    ground_rel_init_ = false;
    bundle_adjustment_->backend_.resetGroundPlaneConstraint();   // 0x1800298A0 on +176
    return;
  }

  const double gate = std::min(0.15, std::max(0.06, ground_sigma_ * 2.5));
  std::unordered_map<int, double> min_dist;       // node 0x20
  std::unordered_map<int, PointPtr> lmks;         // node 0x28
  for (const FramePtr& frame : frame_bundle.frames_)
  {
    for (size_t i = 0; i < frame->num_features_; ++i)
    {
      int track_id = frame->track_id_vec_(i);
      const PointPtr& landmark = frame->landmark_vec_[i];
      if (track_id < 0 || !landmark)
        continue;
      if (frame->type_vec_[i] != FeatureType::kCorner || landmark->obs_.size() < 3)
        continue;
      const Eigen::Vector3d p = landmark->pos_.cast<double>();
      double dist = std::fabs(p.dot(ground_normal_) - ground_distance_);
      if (!std::isfinite(dist) || dist > gate)
        continue;
      if (cv::pointPolygonTest(ground_polygon_, cv::Point2f(p.x(), p.y()), true) < -0.1)
        continue;
      auto res = min_dist.emplace(track_id, dist);   // 0x1800CC140
      if (!res.second && res.first->second > dist)
        res.first->second = dist;
      lmks[track_id] = landmark;                     // 0x1800C0740 + shared_ptr assign 0x1800B2920
    }
  }
  if (min_dist.size() < 8)
  {
    bundle_adjustment_->backend_.resetGroundPlaneConstraint();
    return;
  }

  std::vector<std::pair<double, int>> sorted;
  sorted.reserve(min_dist.size());
  for (const auto& kv : min_dist)
    sorted.emplace_back(kv.second, kv.first);
  std::sort(sorted.begin(), sorted.end());
  if (sorted.size() > 48)
    sorted.resize(48);

  std::vector<Eigen::Vector3d> pts;
  pts.reserve(sorted.size());
  std::vector<int> ids;
  ids.reserve(sorted.size());
  for (const auto& s : sorted)
  {
    auto it = lmks.find(s.second);
    if (it == lmks.end() || !it->second)
      continue;
    pts.emplace_back(it->second->pos_.cast<double>());
    ids.push_back(s.second);
  }

  bool constraint_added = false;
  if (pts.size() >= 12)
  {
    double ratio = 0.0;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const Eigen::Vector3d& p : pts)
      mean += p;
    mean /= static_cast<double>(pts.size());
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const Eigen::Vector3d& p : pts)
    {
      const Eigen::Vector3d d = p - mean;
      cov += d * d.transpose();
    }
    cov /= static_cast<double>(pts.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);   // 0x1800651F0, ComputeEigenvectors
    if (es.info() == Eigen::Success)
    {
      const Eigen::Vector3d ev = es.eigenvalues();
      if (isFinite(ev) && ev(1) >= 0.01 && ev(2) >= 0.04 && ev(0) / ev(1) <= 0.08)   // 0x180027A20
      {
        Eigen::Vector3d n = es.eigenvectors().col(0).normalized();
        if (n.dot(ground_normal_) < 0.0)
          n = -n;
        const double c = std::max(-1.0, std::min(1.0, n.dot(ground_normal_)));
        if (std::cos(0.2617993877991494) <= c)   // 15 deg
        {
          const FramePtr& frame0 = frame_bundle.at(0);
          // four separate T_world_imu() calls (0x180024D50), evaluated z, y, x, w
          Eigen::Quaterniond q_w_b(frame0->T_world_imu().getRotation().w(),
                                   frame0->T_world_imu().getRotation().x(),
                                   frame0->T_world_imu().getRotation().y(),
                                   frame0->T_world_imu().getRotation().z());
          q_w_b.normalize();
          const Eigen::Vector3d n_b = q_w_b.conjugate() * n;   // 0x180104070 conjugate
          const double sigma = std::max(
              0.12, std::min(0.25, std::atan2(std::sqrt(std::max(0.0, ev(0))), std::sqrt(std::max(1e-12, ev(1)))) * 3.0));
          const int bundle_id = frame_bundle.bundle_id_;   // FrameBundle +252
          if (ground_rel_init_)
          {
            const double c_rel = std::max(-1.0, std::min(1.0, n.dot(ground_rel_normal_w_)));
            size_t overlap = 0;
            for (int id : ids)
            {
              if (std::find(ground_rel_ids_.begin(), ground_rel_ids_.end(), id) != ground_rel_ids_.end())
                ++overlap;
            }
            const size_t denom = std::min(ids.size(), ground_rel_ids_.size());
            if (denom)
              ratio = static_cast<double>(overlap) / static_cast<double>(denom);
            if (std::cos(0.03490658503988659) <= c_rel && overlap >= 8 && ratio >= 0.6 &&   // 2 deg
                ground_rel_bundle_id_ != bundle_id)
            {
              GroundPlaneConstraint constraint;   // c03 type (72 bytes)
              constraint.valid = true;
              constraint.bundle_id_0 = ground_rel_bundle_id_;
              constraint.bundle_id_1 = bundle_id;
              constraint.normal_0 = ground_rel_normal_b_;
              constraint.normal_1 = n_b;
              const double s = std::sqrt(sigma * sigma + ground_rel_sigma_ * ground_rel_sigma_);
              constraint.sigma = std::max(0.08, std::min(0.18, s * 0.5));
              bundle_adjustment_->backend_.setGroundPlaneConstraint(constraint);   // 0x18002C530
              ground_rel_bundle_id_ = bundle_id;
              ground_rel_normal_b_ = n_b;
              ground_rel_normal_w_ = n;
              ground_rel_sigma_ = sigma;
              ground_rel_ids_ = ids;
              LOGD("ground-plane relative normal candidates %lu fit %lu overlap %.3f delta %.3f sigma %.3f\n",
                   min_dist.size(), pts.size(), ratio, std::acos(c_rel) * 180.0 / 3.141592653589793,
                   constraint.sigma * 180.0 / 3.141592653589793);
              constraint_added = true;
            }
          }
          else
          {
            ground_rel_init_ = true;
          }
          if (!constraint_added)
          {
            ground_rel_bundle_id_ = bundle_id;
            ground_rel_normal_b_ = n_b;
            ground_rel_normal_w_ = n;
            ground_rel_sigma_ = sigma;
            ground_rel_ids_ = ids;
          }
        }
      }
    }
  }
  if (!constraint_added)
    bundle_adjustment_->backend_.resetGroundPlaneConstraint();
}

// =============================================================================================
// 0x1800F4BD0 -- pimax-new. True when the frame has fewer than 5 tracked features or when at
// least 90 % of them see their landmark under < 4.1 deg parallax from every other frame that
// observed it at the same pyramid level. Name from c07 (caller 0x1800B2F80).
bool FrameProcessorBase::shouldRemoveKeyframe(const FramePtr& frame)
{
  size_t num_low = 0;
  size_t num_total = 0;
  const Eigen::Vector3f cam = frame->T_f_w_.inverse().getPosition().cast<float>();
  for (size_t i = 0; i < frame->num_features_; ++i)
  {
    if (frame->track_id_vec_[i] <= -1)
      continue;
    ++num_total;
    PointPtr landmark = frame->landmark_vec_[i];
    const int level = frame->level_vec_[i];
    const Eigen::Vector3f p = landmark->pos_;
    const Eigen::Vector3f d1 = (cam - p).normalized();
    float max_cos = -1.1f;
    for (const auto& kv : landmark->obs_)
    {
      const KeypointIdentifier& obs = kv.second;
      FramePtr other = obs.frame.lock();
      if (!other || other->id_ == frame->id_)
        continue;
      if (level == other->level_vec_[obs.keypoint_index_])
      {
        FramePtr other2 = obs.frame.lock();   // locked a second time in the binary
        const Eigen::Vector3f cam2 = other2->T_f_w_.inverse().getPosition().cast<float>();
        other2.reset();
        const Eigen::Vector3f d2 = (cam2 - p).normalized();
        max_cos = std::max(max_cos, d1.dot(d2));
      }
    }
    // the binary calls the CRT fmaxl/fminl (long double == double on MSVC)
    const float c = static_cast<float>(std::fmin(std::fmax(static_cast<double>(max_cos), -1.0), 1.0));
    const float angle = acosf(c) * 180.0 / 3.141592653589793;
    if (angle < 4.1)
      ++num_low;
  }
  if (num_total < 5)
    return true;
  return static_cast<float>(num_low) / static_cast<float>(num_total) >= 0.9;
}

}  // namespace totem
}  // namespace pimax
