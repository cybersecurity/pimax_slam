// src/plane/mesher.h -- pimax::totem::Mesher, derived from Kimera-VIO mesh/Mesher.h (v4/v5 era,
// before the 2020-05 "colour in Vertex" refactor), heavily modified by Pimax:
//   * no gtsam: the camera pose is a float Eigen R_w_c_/t_w_c_ written directly by
//     FrameProcessorBase (0x1800F2070),
//   * the 2D mesh / landmark association is done by the caller; populate3dMesh takes the
//     landmark id of every triangle vertex and checks it against lmk_px_,
//   * only horizontal planes are segmented (z histogram); the wall histogram (hist_2d_) is gone,
//     although the wall statistics are still collected and the 2D-histogram parameters are
//     still built in the ctor,
//   * planes carry a convex hull ("polygon_") that gates polygon->plane and plane->plane
//     association, plus many distance-dependent ad-hoc thresholds,
//   * "ground plane" bookkeeping in the globals g_new_plane_ids / g_max_plane_id.
//
// make_shared<pimax::totem::Mesher> block is 0x4D8 bytes => sizeof(Mesher) == 0x4C8 (1224).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "plane/histogram.h"
#include "plane/mesh.h"
#include "plane/plane.h"

namespace pimax {
namespace totem {

// ---------------------------------------------------------------------------------------------
// Kimera gflags turned into plain mutable globals (.data).  Names: Kimera flag names where the
// use matches, otherwise descriptive guesses.  Values are the image's initial values.
// Ownership (which .cpp defines them) is a guess: mesher.cpp.  TODO(verify) with the coordinator
// (c08 uses some of them under other names, noted per line).
extern double g_min_ratio_btw_largest_smallest_side;      // 0x18046A240 = 0.5  (c08 same)
extern double g_min_elongation_ratio;                     // 0x18046A248 = 0.5  (c08 same)
extern double g_max_triangle_side;                        // 0x18046A250 = 1.0  (Kimera 0.5; c08 same)
extern bool g_unknown_bool_258;                           // 0x18046A258 = true (not used in c17)
extern bool g_only_associate_a_polygon_to_a_single_plane; // 0x18046A259 = true
extern bool g_do_double_association;                      // 0x18046A25A = true
extern bool g_ground_plane_association;                   // 0x18046A25B = true (Pimax; name guess)
extern int g_plane_min_lmk_num;                           // 0x18046A25C = 6    (Pimax; used as value - 2)
extern int g_z_histogram_window_size;                     // 0x18046A260 = 2    (Kimera 3)
extern int g_z_histogram_max_number_of_peaks_to_select;   // 0x18046A264 = 3
extern double g_z_histogram_peak_per;                     // 0x18046A268 = 0.5
extern double g_z_histogram_min_separation;               // 0x18046A270 = 0.1
extern int g_z_histogram_bins;                            // 0x18046A278 = 512  (used as bins - 312 = 200)
extern int g_hist_2d_theta_bins;                          // 0x18046A27C = 15   (Kimera 40)
extern double g_z_histogram_min_range;                    // 0x18046A280 = -4.0 (Kimera -0.75)
extern double g_z_histogram_max_range;                    // 0x18046A288 = 4.0  (Kimera 3.0)
extern int g_hist_2d_distance_bins;                       // 0x18046A290 = 30   (Kimera 40)
extern int g_plane_overlap_min_points;                    // 0x18046A294 = 3    (Pimax; name guess)
extern double g_hist_2d_theta_range_max;                  // 0x18046A298 = 3.14 (Kimera M_PI)
extern double g_hist_2d_distance_range_min;               // 0x18046A2A0 = -4.0 (Kimera -6)
extern double g_hist_2d_distance_range_max;               // 0x18046A2A8 = 4.0  (Kimera 6)
extern double g_normal_tolerance_polygon_plane_association;   // 0x18046A2B0 = 0.025 (Kimera 0.011)
extern double g_distance_tolerance_polygon_plane_association; // 0x18046A2B8 = 0.15 (Kimera 0.10; c08: g_wall_distance_tolerance)
extern double g_normal_tolerance_horizontal_surface;      // 0x18046A2C0 = 0.015 (Kimera 0.011)
extern double g_normal_tolerance_walls;                   // 0x18046A2C8 = 0.015 (Kimera 0.0165)
extern double g_normal_tolerance_plane_plane_association; // 0x18046A2D0 = 0.015 (Kimera 0.011)
extern double g_normal_tolerance_plane_plane;             // 0x18046A2D8 = 0.015 (c08 only)
extern double g_distance_tolerance_plane_plane_association; // 0x18046A2E0 = 0.2
extern double g_distance_tolerance_plane_plane;           // 0x18046A2E8 = 0.1  (c08 only)
extern double g_plane_distance_update_tolerance;          // 0x18046A2F0 = 0.15 (c08 only)
extern int g_max_plane_id;                                // 0x18046A2F8 = -1   (max of g_new_plane_ids, computed by c08)
extern double g_hist_2d_theta_range_min;                  // 0x18047EF60 (.bss) = 0.0
extern std::vector<int> g_new_plane_ids;                  // 0x18047EF68 (.bss), atexit dtor 0x1803A6380

// ---------------------------------------------------------------------------------------------
// Free helpers (no `this`; also called from FrameProcessorBase 0x1800EC2B0).

// 0x18019F330 -- PNPOLY crossing test; an empty polygon counts as "inside".
bool isPointInPolygon(float x, float y, const std::vector<cv::Point2f>& polygon);

// 0x18019E480 -- do the two hulls (already rotated into the plane frame) overlap by at least
// g_plane_overlap_min_points points?  lmk id vectors passed by value.
bool polygonsOverlap(const std::vector<PolygonVertex>& polygon_a, std::vector<int> lmk_ids_a,
                     const std::vector<PolygonVertex>& polygon_b, std::vector<int> lmk_ids_b,
                     const LmkPositionMap& lmk_positions);

// 0x18019CA00 -- merge polygon_b into polygon_a (QUIRK: only effective when polygon_a is empty;
// otherwise the convex hull is computed and discarded).
void mergePolygons(std::vector<PolygonVertex>* polygon_a,
                   const std::vector<PolygonVertex>& polygon_b, const Eigen::Vector3f& normal_a,
                   bool* clockwise);

// ---------------------------------------------------------------------------------------------
class Mesher {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 0x18019C020
  Mesher();
  ~Mesher() = default;  // run by _Ref_count_obj2<Mesher>::_Destroy (other chunk)

  // 0x18019DB90
  bool populate3dMeshTimeHorizon(const std::vector<cv::Vec6f>& mesh_2d_pixels,
                                 const LandmarkIds& triangle_lmk_ids,
                                 const LmkPositionMap& points_with_id_map,
                                 double min_ratio_largest_smallest_side,
                                 double min_elongation_ratio, double max_triangle_side,
                                 Mesh2D* mesh_2d = nullptr);

  // 0x1801A0AF0
  void clusterPlanesFromMesh(std::vector<Plane>* planes, std::vector<Plane>* new_planes,
                             const LmkPositionMap& points_with_id_vio);

 private:
  // 0x18019D000
  bool populate3dMesh(const std::vector<cv::Vec6f>& mesh_2d_pixels,
                      const LandmarkIds& triangle_lmk_ids,
                      const LmkPositionMap& points_with_id_map,
                      double min_ratio_largest_smallest_side, double min_elongation_ratio,
                      double max_triangle_side, Mesh2D* mesh_2d = nullptr);
  // 0x18019F590
  void updatePolygonMeshToTimeHorizon(const LmkPositionMap& points_with_id_map);

  // inlined everywhere
  double getRatioBetweenSmallestAndLargestSide(const double& d12, const double& d23,
                                               const double& d31) const {
    double minSide = std::min(d12, std::min(d23, d31));
    double maxSide = std::max(d12, std::max(d23, d31));
    return minSide / maxSide;
  }
  // 0x18019DC30
  double getRatioBetweenTangentialAndRadialDisplacement(const Vertex3D& p1, const Vertex3D& p2,
                                                        const Vertex3D& p3,
                                                        const Eigen::Matrix3f& R_w_c,
                                                        const Eigen::Vector3f& t_w_c) const;
  // 0x18019E1A0
  bool isBadTriangle(const Mesh3D::Polygon& polygon, const Eigen::Matrix3f& R_w_c,
                     const Eigen::Vector3f& t_w_c,
                     const double& min_ratio_between_largest_an_smallest_side,
                     const double& min_elongation_ratio, const double& max_triangle_side) const;

  // 0x1801A0870
  bool calculateNormal(const Vertex3D& p1, const Vertex3D& p2, const Vertex3D& p3,
                       cv::Point3f* normal) const;

  // Kimera helpers, inlined, CHECK_NEARs removed.
  bool isNormalAroundAxis(const cv::Point3f& axis, const cv::Point3f& normal,
                          const double& tolerance) const {
    return (std::fabs(normal.ddot(axis)) > 1.0 - tolerance);
  }
  bool isNormalPerpendicularToAxis(const cv::Point3f& axis, const cv::Point3f& normal,
                                   const double& tolerance) const {
    return (cv::norm(normal.ddot(axis)) < tolerance);
  }
  bool isPointAtDistanceFromPlane(const Vertex3D& point, const double& plane_distance,
                                  const cv::Point3f& plane_normal,
                                  const double& distance_tolerance) const {
    return (std::fabs(plane_distance - point.ddot(plane_normal)) <= distance_tolerance);
  }
  bool isPolygonAtDistanceFromPlane(const Mesh3D::Polygon& polygon, const double& plane_distance,
                                    const cv::Point3f& plane_normal,
                                    const double& distance_tolerance) const {
    for (const Mesh3D::VertexType& vertex : polygon) {
      if (!isPointAtDistanceFromPlane(vertex.getVertexPosition(), plane_distance, plane_normal,
                                      distance_tolerance)) {
        return false;
      }
    }
    return true;
  }
  double getLongitude(const cv::Point3f& triangle_normal, const cv::Point3f& vertical) const {
    cv::Point3f equatorial_proj = triangle_normal - vertical.ddot(triangle_normal) * vertical;
    return std::atan2(equatorial_proj.y, equatorial_proj.x);
  }
  void appendLmkIdsOfPolygon(const Mesh3D::Polygon& polygon, LandmarkIds* lmk_ids) const {
    for (const Mesh3D::VertexType& vertex : polygon) {
      const auto& it = std::find(lmk_ids->begin(), lmk_ids->end(), vertex.getLmkId());
      if (it == lmk_ids->end()) {
        lmk_ids->push_back(vertex.getLmkId());
      }
    }
  }

  // 0x1801A24D0
  void segmentPlanesInMesh(std::vector<Plane>* seed_planes, std::vector<Plane>* new_planes,
                           const LmkPositionMap& points_with_id_vio,
                           const double& normal_tolerance_polygon_plane_association,
                           const double& distance_tolerance_polygon_plane_association,
                           const double& normal_tolerance_horizontal_surface,
                           const double& normal_tolerance_walls);
  // inlined into segmentPlanesInMesh
  void segmentNewPlanes(std::vector<Plane>* new_segmented_planes, const cv::Mat& z_components,
                        const cv::Mat& walls);
  // 0x1801A1DB0
  void segmentHorizontalPlanes(std::vector<Plane>* horizontal_planes, size_t* plane_id,
                               const Plane::Normal& normal, const cv::Mat& z_components);
  // 0x1801A3310
  void updatePlanesLmkIdsFromMesh(std::vector<Plane>* planes, double normal_tolerance,
                                  double distance_tolerance,
                                  const LmkPositionMap& points_with_id_vio) const;
  // 0x1801A3750
  bool updatePlanesLmkIdsFromPolygon(std::vector<Plane>* seed_planes,
                                     const Mesh3D::Polygon& polygon, const size_t& triangle_id,
                                     const cv::Point3f& triangle_normal, double normal_tolerance,
                                     double distance_tolerance, double distance_scale,
                                     const LmkPositionMap& points_with_id_vio,
                                     bool only_associate_a_polygon_to_a_single_plane) const;
  // 0x18019EF20
  bool isPolygonInPlaneHull(const Mesh3D::Polygon& polygon, const Plane& plane) const;
  // 0x1801A1500
  void updatePlanesPolygon(std::vector<Plane>* planes,
                           const LmkPositionMap& points_with_id_vio) const;
  // 0x18019FA90
  void associatePlanes(const std::vector<Plane>& segmented_planes,
                       const std::vector<Plane>& planes,
                       std::vector<Plane>* non_associated_planes,
                       std::vector<Plane>* associated_segmented_planes,
                       LandmarkIds* associated_plane_ids,
                       const LmkPositionMap& points_with_id_vio, const double& normal_tolerance,
                       const double& distance_tolerance) const;

 public:
  // Named mesh_3d (no underscore): the glog CHECK texts say "mesh_3d.getPolygon(i, &polygon)"
  // and "mesh_3d.getMeshPolygonDimension() == mesh_polygon_dim".
  Mesh3D mesh_3d;               // +0x000
  Mesh3D mesh_output;           // +0x150 (Kimera: local in updatePolygonMeshToTimeHorizon)
  Histogram z_hist_;            // +0x2A0 (0x1B0 bytes)
  Eigen::Matrix3f R_w_c_;       // +0x450 written by FrameProcessorBase (0x1800F2070)
  Eigen::Vector3f t_w_c_;       // +0x474 written by FrameProcessorBase
  LmkPixelMap lmk_px_;          // +0x480 assigned by FrameProcessorBase
  bool is_first_frame_;         // +0x4C0 = planes->empty() at clusterPlanesFromMesh entry

  static void layout_check();
};

inline void Mesher::layout_check() {
  static_assert(sizeof(Mesher) == 0x4C8, "sizeof(Mesher)");
  static_assert(offsetof(Mesher, mesh_output) == 0x150, "Mesher::mesh_output");
  static_assert(offsetof(Mesher, z_hist_) == 0x2A0, "Mesher::z_hist_");
  static_assert(offsetof(Mesher, R_w_c_) == 0x450, "Mesher::R_w_c_");
  static_assert(offsetof(Mesher, t_w_c_) == 0x474, "Mesher::t_w_c_");
  static_assert(offsetof(Mesher, lmk_px_) == 0x480, "Mesher::lmk_px_");
  static_assert(offsetof(Mesher, is_first_frame_) == 0x4C0, "Mesher::is_first_frame_");
}

}  // namespace totem
}  // namespace pimax
