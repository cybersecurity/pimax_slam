// src/plane/mesher.cpp -- pimax::totem::Mesher (Kimera-VIO Mesher.cpp derived, Pimax-modified).
// __FILE__ = "E:\code_codex\pimax_slam\beta111_5a7902_dll\src\plane\mesher.cpp".
//
// glog CHECK line numbers used by the original (the FATAL messages print file:line):
//   updatePlanesLmkIdsFromMesh: CHECK_NOTNULL(planes) 796, CHECK_EQ(dim) 798
//   segmentPlanesInMesh: CHECK_NOTNULL(seed_planes) 982, CHECK_NOTNULL(new_planes) 983,
//                        CHECK_EQ(dim) 999, CHECK(getPolygon) 1010, CHECK_EQ(polygon.size()) 1011
// They are forced with #line directives at the statements below.
#define _USE_MATH_DEFINES  // M_PI
#include "plane/mesher.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <limits>

#include <glog/logging.h>

namespace pimax {
namespace totem {

// ---------------------------------------------------------------------------------------------
// Globals (see mesher.h).  TODO(verify) owning translation unit.
double g_min_ratio_btw_largest_smallest_side = 0.5;           // 0x18046A240
double g_min_elongation_ratio = 0.5;                          // 0x18046A248
double g_max_triangle_side = 1.0;                             // 0x18046A250
bool g_unknown_bool_258 = true;                               // 0x18046A258
bool g_only_associate_a_polygon_to_a_single_plane = true;     // 0x18046A259
bool g_do_double_association = true;                          // 0x18046A25A
bool g_ground_plane_association = true;                       // 0x18046A25B
int g_plane_min_lmk_num = 6;                                  // 0x18046A25C
int g_z_histogram_window_size = 2;                            // 0x18046A260
int g_z_histogram_max_number_of_peaks_to_select = 3;          // 0x18046A264
double g_z_histogram_peak_per = 0.5;                          // 0x18046A268
double g_z_histogram_min_separation = 0.1;                    // 0x18046A270
int g_z_histogram_bins = 512;                                 // 0x18046A278
int g_hist_2d_theta_bins = 15;                                // 0x18046A27C
double g_z_histogram_min_range = -4.0;                        // 0x18046A280
double g_z_histogram_max_range = 4.0;                         // 0x18046A288
int g_hist_2d_distance_bins = 30;                             // 0x18046A290
int g_plane_overlap_min_points = 3;                           // 0x18046A294
double g_hist_2d_theta_range_max = 3.14;                      // 0x18046A298
double g_hist_2d_distance_range_min = -4.0;                   // 0x18046A2A0
double g_hist_2d_distance_range_max = 4.0;                    // 0x18046A2A8
double g_normal_tolerance_polygon_plane_association = 0.025;  // 0x18046A2B0
double g_distance_tolerance_polygon_plane_association = 0.15; // 0x18046A2B8
double g_normal_tolerance_horizontal_surface = 0.015;         // 0x18046A2C0
double g_normal_tolerance_walls = 0.015;                      // 0x18046A2C8
double g_normal_tolerance_plane_plane_association = 0.015;    // 0x18046A2D0
double g_normal_tolerance_plane_plane = 0.015;                // 0x18046A2D8
double g_distance_tolerance_plane_plane_association = 0.2;    // 0x18046A2E0
double g_distance_tolerance_plane_plane = 0.1;                // 0x18046A2E8
double g_plane_distance_update_tolerance = 0.15;              // 0x18046A2F0
int g_max_plane_id = -1;                                      // 0x18046A2F8
double g_hist_2d_theta_range_min = 0.0;                       // 0x18047EF60 (.bss)
std::vector<int> g_new_plane_ids;                             // 0x18047EF68 (.bss)

// ---------------------------------------------------------------------------------------------
// 0x18019C020
// upstream-modified: no MesherParams; z histogram 200 bins over [-4, 4]; the 2D (walls)
// histogram is no longer constructed but its parameter vectors are still built (dead code kept
// because the allocations are observable to the compiler).  R/t identity, lmk_px_ cleared,
// is_first_frame_ = true.
Mesher::Mesher() : mesh_3d(3), mesh_output(3), z_hist_(), lmk_px_() {
  // Create z histogram.
  std::vector<int> hist_size = {g_z_histogram_bins - 312};
  // We cannot use an array of doubles here bcs the function cv::calcHist asks
  // for pointers to floats... And it is not easy to cast!
  std::array<float, 2> z_range = {static_cast<float>(g_z_histogram_min_range),
                                  static_cast<float>(g_z_histogram_max_range)};
  std::vector<std::array<float, 2>> ranges = {z_range};
  std::vector<int> channels = {0};
  z_hist_ = Histogram(1, channels, cv::Mat(), 1, hist_size, ranges, true, false);

  // Create 2d histogram (its construction was removed; the arguments remain).
  std::vector<int> hist_2d_size = {g_hist_2d_theta_bins, g_hist_2d_distance_bins};
  std::array<float, 2> theta_range = {static_cast<float>(g_hist_2d_theta_range_min),
                                      static_cast<float>(g_hist_2d_theta_range_max)};
  std::array<float, 2> distance_range = {static_cast<float>(g_hist_2d_distance_range_min),
                                         static_cast<float>(g_hist_2d_distance_range_max)};
  std::vector<std::array<float, 2>> ranges_2d = {theta_range, distance_range};
  std::vector<int> channels_2d = {0, 1};

  R_w_c_ = Eigen::Matrix3f::Identity();
  t_w_c_ = Eigen::Vector3f::Zero();
  lmk_px_.clear();
  is_first_frame_ = true;
}

/* -------------------------------------------------------------------------- */
// 0x18019DB90
// upstream-modified: returns whether populate3dMesh did anything; updatePolygonMeshToTimeHorizon
// lost its pose/threshold parameters.
bool Mesher::populate3dMeshTimeHorizon(const std::vector<cv::Vec6f>& mesh_2d_pixels,
                                       const LandmarkIds& triangle_lmk_ids,
                                       const LmkPositionMap& points_with_id_map,
                                       double min_ratio_largest_smallest_side,
                                       double min_elongation_ratio, double max_triangle_side,
                                       Mesh2D* mesh_2d) {
  if (populate3dMesh(mesh_2d_pixels, triangle_lmk_ids, points_with_id_map,
                     min_ratio_largest_smallest_side, min_elongation_ratio, max_triangle_side,
                     mesh_2d)) {
    updatePolygonMeshToTimeHorizon(points_with_id_map);
    return true;
  }
  return false;
}

/* -------------------------------------------------------------------------- */
// 0x18019D000
// upstream-modified (heavily):
//  * the landmark id of every triangle vertex comes from `triangle_lmk_ids` (running index over
//    all triangle vertices); the pixel is only cross-checked against lmk_px_ (printf on mismatch),
//  * INT_MIN marks "no landmark": skip the rest of the triangle, keeping the index in sync,
//  * QUIRK: a landmark missing from points_with_id_map aborts the triangle WITHOUT advancing
//    the running index for the remaining vertices, so later triangles read shifted ids,
//  * the mesh is capped at 1000 polygons,
//  * the side-ratio pre-check uses min_elongation_ratio (not min_ratio_largest_smallest_side),
//  * only triangles whose normal is close to vertical (horizontal surfaces) are kept; the
//    max side threshold is scaled by the camera-to-triangle distance along the normal.
bool Mesher::populate3dMesh(const std::vector<cv::Vec6f>& mesh_2d_pixels,
                            const LandmarkIds& triangle_lmk_ids,
                            const LmkPositionMap& points_with_id_map,
                            double min_ratio_largest_smallest_side, double min_elongation_ratio,
                            double max_triangle_side, Mesh2D* mesh_2d) {
  if (mesh_2d_pixels.empty()) {
    return false;
  }

  // Create face and add it to the 2d mesh.
  Mesh2D::Polygon face;
  face.resize(3);

  // Clean output 2d mesh.
  if (mesh_2d != nullptr) mesh_2d->clearMesh();

  // Create polygon and add it to the 3d mesh.
  Mesh3D::Polygon polygon;
  polygon.resize(3);

  int lmk_index = -1;
  // Iterate over the 2d mesh triangles.
  for (size_t i = 0; i < mesh_2d_pixels.size(); i++) {
    if (mesh_3d.getNumberOfPolygons() > 1000) break;

    Eigen::Vector3f centroid(0.f, 0.f, 0.f);
    const cv::Vec6f triangle_2d = mesh_2d_pixels.at(i);

    // Iterate over each vertex (pixel) of the triangle.
    for (size_t j = 0; j < 3; j++) {
      ++lmk_index;
      // Extract pixel.
      const cv::Point2f pixel(triangle_2d[j * 2], triangle_2d[j * 2 + 1]);

      // Look the pixel up among the landmarks of the current frame (linear scan, exact compare).
      LandmarkId lmk_id_2d = std::numeric_limits<int>::min();
      for (const auto& lmk_px : lmk_px_) {
        if (lmk_px.second.x() == pixel.x && lmk_px.second.y() == pixel.y) {
          lmk_id_2d = lmk_px.first;
          break;
        }
      }
      if (lmk_id_2d != triangle_lmk_ids[lmk_index]) {
        printf("lmk id not equal to mesh 2d ids\n");
      }

      const LandmarkId lmk_id = triangle_lmk_ids[lmk_index];
      if (lmk_id == std::numeric_limits<int>::min()) {
        lmk_index += 2 - static_cast<int>(j);
        break;
      }

      // Try to find this landmark id in points_with_id_map.
      const auto& lmk_it = points_with_id_map.find(lmk_id);
      if (lmk_it == points_with_id_map.end()) {
        break;  // QUIRK: lmk_index not advanced for the remaining vertices.
      }

      // We found the landmark.
      const cv::Point3f lmk(lmk_it->second.x(), lmk_it->second.y(), lmk_it->second.z());
      centroid += Eigen::Vector3f(lmk.x, lmk.y, lmk.z);
      polygon.at(j) = Mesh3D::VertexType(lmk_id, lmk);
      if (mesh_2d != nullptr) {
        face.at(j) = Mesh2D::VertexType(lmk_id, pixel);
      }

      if (j == 2) {
        // Last iteration.
        const Vertex3D& p1 = polygon.at(0).getVertexPosition();
        const Vertex3D& p2 = polygon.at(1).getVertexPosition();
        const Vertex3D& p3 = polygon.at(2).getVertexPosition();

        // Side ratio pre-filter (getRatioBetweenSmallestAndLargestSide inlined; note the
        // argument order / subtraction directions of the binary).
        const double d32 = cv::norm(p3 - p2);
        const double d13 = cv::norm(p1 - p3);
        const double d12 = cv::norm(p1 - p2);
        if (min_elongation_ratio > getRatioBetweenSmallestAndLargestSide(d12, d13, d32)) {
          break;
        }

        centroid /= 3.0f;

        cv::Point3f triangle_normal;
        if (calculateNormal(p1, p2, p3, &triangle_normal)) {
          const Eigen::Vector3f normal(triangle_normal.x, triangle_normal.y, triangle_normal.z);
          // Distance between camera and triangle along the triangle normal.
          const float dist_to_cam = (centroid - t_w_c_).dot(normal);
          const double dist_to_cam_abs = std::fabs(dist_to_cam);
          const double r22_abs = std::fabs(static_cast<double>(R_w_c_(2, 2)));

          double horizontal_threshold;
          if (r22_abs < 0.7 && dist_to_cam_abs >= 1.2) {
            horizontal_threshold =
                1.0 - ((1.0 - r22_abs) * 0.1 + g_normal_tolerance_horizontal_surface);
          } else {
            horizontal_threshold = 1.0 - g_normal_tolerance_horizontal_surface;
          }
          const double normal_z = triangle_normal.ddot(cv::Point3f(0.f, 0.f, 1.f));
          if (!(std::fabs(normal_z) > horizontal_threshold)) {
            // TODO(verify) source form: only a discarded cv::norm(normal_z) call survives here
            // (Kimera's wall test isNormalPerpendicularToAxis with an empty body).
            (void)cv::norm(normal_z);
            break;
          }

          const double distance_scale = std::min(1.0, std::max(0.4, dist_to_cam_abs));
          const bool far_away = distance_scale >= 1.0;

          bool keep = false;
          if (distance_scale < 1.0) {
            const double max_side = (max_triangle_side - 0.15) * distance_scale;
            keep = !isBadTriangle(polygon, R_w_c_, t_w_c_, min_ratio_largest_smallest_side,
                                  min_elongation_ratio, max_side);
          }
          if (!keep && distance_scale >= 1.0) {
            const double max_side = distance_scale * max_triangle_side;
            keep = !isBadTriangle(polygon, R_w_c_, t_w_c_, min_ratio_largest_smallest_side,
                                  min_elongation_ratio, max_side);
          }
          if (!keep && far_away) {
            const double max_side = (2.0 - r22_abs * 1.5 + max_triangle_side) * distance_scale;
            keep = !isBadTriangle(polygon, R_w_c_, t_w_c_, min_ratio_largest_smallest_side,
                                  min_elongation_ratio, max_side);
          }
          if (keep) {
            // Save the valid triangular polygon.
            mesh_3d.addPolygonToMesh(polygon);
            if (mesh_2d != nullptr) {
              mesh_2d->addPolygonToMesh(face);
            }
          }
        }
      }
    }
  }
  return true;
}

/* -------------------------------------------------------------------------- */
// 0x18019F590
// upstream-modified: always reduces to the time horizon, no re-filtering with isBadTriangle,
// no CHECK on getPolygon, uses the member mesh_output (cleared afterwards) instead of a local.
void Mesher::updatePolygonMeshToTimeHorizon(const LmkPositionMap& points_with_id_map) {
  const auto& end = points_with_id_map.end();

  // Loop over each face in the mesh.
  Mesh3D::Polygon polygon;
  for (size_t i = 0; i < mesh_3d.getNumberOfPolygons(); i++) {
    mesh_3d.getPolygon(i, &polygon);
    bool save_polygon = true;
    for (Mesh3D::VertexType& vertex : polygon) {
      const auto& point_with_id_it = points_with_id_map.find(vertex.getLmkId());
      if (point_with_id_it == end) {
        // Vertex of current polygon is not in points_with_id_map: delete the polygon.
        save_polygon = false;
        break;
      } else {
        // Update the vertex with newest landmark position.
        vertex.setVertexPosition(Vertex3D(point_with_id_it->second.x(),
                                          point_with_id_it->second.y(),
                                          point_with_id_it->second.z()));
      }
    }

    if (save_polygon) {
      mesh_output.addPolygonToMesh(polygon);
    }
  }

  mesh_3d = mesh_output;
  mesh_output.clearMesh();
}

/* -------------------------------------------------------------------------- */
// 0x18019DC30
// upstream-modified: float Eigen instead of gtsam (camera-frame points R^T (p - t)); no LOG.
double Mesher::getRatioBetweenTangentialAndRadialDisplacement(
    const Vertex3D& p1, const Vertex3D& p2, const Vertex3D& p3, const Eigen::Matrix3f& R_w_c,
    const Eigen::Vector3f& t_w_c) const {
  std::vector<Eigen::Vector3f> points;

  // get 3D points
  const Eigen::Vector3f p1_C(p1.x, p1.y, p1.z);
  const Eigen::Vector3f p2_C(p2.x, p2.y, p2.z);
  const Eigen::Vector3f p3_C(p3.x, p3.y, p3.z);
  points.emplace_back(R_w_c.transpose() * (p1_C - t_w_c));  // checks elongation in *camera frame*
  points.emplace_back(R_w_c.transpose() * (p2_C - t_w_c));
  points.emplace_back(R_w_c.transpose() * (p3_C - t_w_c));

  // UtilsGeometry::getRatioBetweenTangentialAndRadialDisplacement inlined:
  // Compute radial directions.
  double min_z = std::numeric_limits<double>::max();
  double max_z = 0;
  for (size_t i = 0; i < points.size(); i++) {
    double z_i = points.at(i).z();
    if (z_i < min_z) {
      min_z = z_i;
    }
    if (z_i > max_z) {
      max_z = z_i;
    }
  }

  std::vector<Eigen::Vector3f> points_rescaled;
  for (size_t i = 0; i < points.size(); i++) {
    // Point rescaled is a projection of point on a virtual img plane at distance minZ.
    points_rescaled.emplace_back((points.at(i) / points.at(i).z()) *
                                 static_cast<float>(min_z));
  }

  double max_t = 0.0;
  double tangential_elongation_1 = (points_rescaled.at(0) - points_rescaled.at(1)).norm();
  max_t = std::max(max_t, tangential_elongation_1);
  double tangential_elongation_2 = (points_rescaled.at(1) - points_rescaled.at(2)).norm();
  max_t = std::max(max_t, tangential_elongation_2);
  double tangential_elongation_3 = (points_rescaled.at(0) - points_rescaled.at(2)).norm();
  max_t = std::max(max_t, tangential_elongation_3);

  // Points must be in front of the camera, and min should be less than max.
  if (min_z < 0 || max_z < 0 || min_z > max_z) {
    return 0;
  }
  return max_t / (max_z - min_z);
}

/* -------------------------------------------------------------------------- */
// 0x18019E1A0
// upstream-modified: CHECK_EQ(size, 3) -> "bad"; early-return form (tangential ratio is only
// computed last); pose given as float R/t.
bool Mesher::isBadTriangle(const Mesh3D::Polygon& polygon, const Eigen::Matrix3f& R_w_c,
                           const Eigen::Vector3f& t_w_c,
                           const double& min_ratio_between_largest_an_smallest_side,
                           const double& min_elongation_ratio,
                           const double& max_triangle_side) const {
  if (polygon.size() != 3) {
    return true;
  }
  const Vertex3D& p1 = polygon.at(0).getVertexPosition();
  const Vertex3D& p2 = polygon.at(1).getVertexPosition();
  const Vertex3D& p3 = polygon.at(2).getVertexPosition();

  // Check geometric dimensions.
  double d12 = cv::norm(p1 - p2);
  double d23 = cv::norm(p2 - p3);
  double d31 = cv::norm(p3 - p1);

  // If threshold is disabled, avoid computation.
  if (min_ratio_between_largest_an_smallest_side > 0.0 &&
      min_ratio_between_largest_an_smallest_side >
          getRatioBetweenSmallestAndLargestSide(d12, d23, d31)) {
    return true;
  }

  // If threshold is disabled, avoid computation.
  if (max_triangle_side > 0.0) {
    std::array<double, 3> sidesLen;
    sidesLen.at(0) = d12;
    sidesLen.at(1) = d23;
    sidesLen.at(2) = d31;
    const auto& it = std::max_element(sidesLen.begin(), sidesLen.end());
    if (*it > max_triangle_side) {
      return true;
    }
  }

  // If threshold is disabled, avoid computation.
  return min_elongation_ratio > 0.0 &&
         min_elongation_ratio >
             getRatioBetweenTangentialAndRadialDisplacement(p1, p2, p3, R_w_c, t_w_c);
}

/* -------------------------------------------------------------------------- */
// 0x1801A0870
// upstream-modified: CHECK_NOTNULL / CHECK_GT(v21_norm) / CHECK_GT(v31_norm) / CHECK_NEAR and
// the LOG(WARNING) removed; CHECK_GT(norm, 0) became `return false`.
bool Mesher::calculateNormal(const Vertex3D& p1, const Vertex3D& p2, const Vertex3D& p3,
                             cv::Point3f* normal) const {
  // Calculate vectors of the triangle.
  cv::Point3f v21 = p2 - p1;
  cv::Point3f v31 = p3 - p1;

  // Normalize vectors.
  double v21_norm = cv::norm(v21);
  v21 /= v21_norm;

  double v31_norm = cv::norm(v31);
  v31 /= v31_norm;

  // Check that vectors are not aligned, dot product should not be 1 or -1.
  static constexpr double epsilon = 1e-3;  // 2.5 degrees aperture.
  if (std::fabs(v21.ddot(v31)) >= 1.0 - epsilon) {
    return false;
  }

  // Calculate normal (follows the right-hand rule).
  *normal = v21.cross(v31);

  // Normalize.
  double norm = cv::norm(*normal);
  if (norm <= 0.0) {
    return false;
  }
  *normal /= norm;
  return true;
}

/* -------------------------------------------------------------------------- */
// 0x1801A24D0
// upstream-modified: lmk ids leaving the time horizon are removed from the seed planes
// (instead of clearing all), polygon association gets a distance-dependent scale, the
// horizontal tolerance is relaxed (+0.1) for far triangles seen with a tilted camera, z values
// are stored relative to the camera height, walls are collected but never segmented.
void Mesher::segmentPlanesInMesh(std::vector<Plane>* seed_planes, std::vector<Plane>* new_planes,
                                 const LmkPositionMap& points_with_id_vio,
                                 const double& normal_tolerance_polygon_plane_association,
                                 const double& distance_tolerance_polygon_plane_association,
                                 const double& normal_tolerance_horizontal_surface,
                                 const double& normal_tolerance_walls) {
#line 982
  CHECK_NOTNULL(seed_planes);  // mesher.cpp:982
  CHECK_NOTNULL(new_planes);   // mesher.cpp:983

  // Remove from the seed planes the lmk ids that left the optimization time horizon.
  for (Plane& seed_plane : *seed_planes) {
    if (seed_plane.is_valid_) {
      for (auto it = seed_plane.lmk_ids_.begin(); it != seed_plane.lmk_ids_.end();) {
        if (points_with_id_vio.find(*it) == points_with_id_vio.end()) {
          it = seed_plane.lmk_ids_.erase(it);
        } else {
          ++it;
        }
      }
    }
  }

  static constexpr size_t mesh_polygon_dim = 3;
#line 999
  CHECK_EQ(mesh_3d.getMeshPolygonDimension(), mesh_polygon_dim) << "Expecting 3 vertices in triangle.";  // mesher.cpp:999

  // Cluster new lmk ids for seed planes.
  // Loop over the mesh only once.
  Mesh3D::Polygon polygon;
  cv::Mat z_components(1, 0, CV_32F);
  cv::Mat walls(0, 0, CV_32FC2);
  std::vector<Mesh3D::Polygon> wall_polygons;  // TODO(verify) name; filled, never read
  for (size_t i = 0; i < mesh_3d.getNumberOfPolygons(); i++) {
#line 1010
    CHECK(mesh_3d.getPolygon(i, &polygon)) << "Could not retrieve polygon.";  // mesher.cpp:1010
    CHECK_EQ(polygon.size(), mesh_polygon_dim);                               // mesher.cpp:1011
    const Vertex3D& p1 = polygon.at(0).getVertexPosition();
    const Vertex3D& p2 = polygon.at(1).getVertexPosition();
    const Vertex3D& p3 = polygon.at(2).getVertexPosition();

    // Calculate normal of the triangle in the mesh.
    // The normals are in the world frame of reference.
    cv::Point3f triangle_normal;
    if (calculateNormal(p1, p2, p3, &triangle_normal)) {
      const cv::Point3f centroid = (p1 + p2 + p3) / 3.0;
      const Eigen::Vector3f normal(triangle_normal.x, triangle_normal.y, triangle_normal.z);
      const float dist_to_cam =
          normal.dot(Eigen::Vector3f(centroid.x, centroid.y, centroid.z) - t_w_c_);
      const double distance_scale =
          std::min(1.0, std::max(0.5, static_cast<double>(std::fabs(dist_to_cam))));

      ////////////////////////// Update seed planes ////////////////////////////
      bool is_polygon_on_a_plane = updatePlanesLmkIdsFromPolygon(
          seed_planes, polygon, i, triangle_normal, normal_tolerance_polygon_plane_association,
          distance_tolerance_polygon_plane_association, distance_scale, points_with_id_vio,
          g_only_associate_a_polygon_to_a_single_plane);

      bool relax_horizontal_tolerance = false;
      if (std::fabs(static_cast<double>(R_w_c_(2, 2))) < 0.7) {
        relax_horizontal_tolerance = distance_scale >= 1.0;
      }

      ////////////////// Build Histogram for new planes ////////////////////////
      static const cv::Point3f vertical(0, 0, 1);  // 0x18046A220
      if (!is_polygon_on_a_plane &&
          isNormalAroundAxis(vertical, triangle_normal,
                             relax_horizontal_tolerance
                                 ? normal_tolerance_horizontal_surface + 0.1
                                 : normal_tolerance_horizontal_surface)) {
        // Store z components (relative to the camera height) to build histogram.
        z_components.push_back(p1.z - t_w_c_.z());
        z_components.push_back(p2.z - t_w_c_.z());
        z_components.push_back(p3.z - t_w_c_.z());
      }
      if (!is_polygon_on_a_plane &&
          isNormalPerpendicularToAxis(vertical, triangle_normal, normal_tolerance_walls)) {
        /// Values for walls Histogram.
        double theta = getLongitude(triangle_normal, vertical);

        // Distance of the plane through the triangle, relative to the camera.
        const cv::Point3f cam(t_w_c_.x(), t_w_c_.y(), t_w_c_.z());
        double distance = (p1.ddot(triangle_normal) + p2.ddot(triangle_normal) +
                           p3.ddot(triangle_normal)) / 3.0 -
                          triangle_normal.dot(cam);
        if (theta < 0) {
          // Say theta is -pi/2, then normalized theta is pi/2.
          theta = theta + M_PI;
          // Change distance accordingly.
          distance = -distance;
        }
        walls.push_back(cv::Point2f(theta, distance));
        wall_polygons.push_back(polygon);
      }
    }
  }

  // Segment new planes.
  segmentNewPlanes(new_planes, z_components, walls);
}

/* -------------------------------------------------------------------------- */
// inlined into 0x1801A24D0
// upstream-modified: walls are not segmented any more.
void Mesher::segmentNewPlanes(std::vector<Plane>* new_segmented_planes,
                              const cv::Mat& z_components, const cv::Mat& walls) {
  new_segmented_planes->clear();

  // Segment horizontal planes.
  static size_t plane_id = 0;                    // 0x18047EF58
  static const Plane::Normal vertical(0, 0, 1);  // 0x18046A230
  segmentHorizontalPlanes(new_segmented_planes, &plane_id, vertical, z_components);
  (void)walls;
}

/* -------------------------------------------------------------------------- */
// 0x1801A1DB0
// upstream-modified: no CHECKs/logs; histogram not logged; 1x3 smoothing kernel; adaptive
// min_support = clamp(rows / 10, 7, 20); Pimax getLocalMaximum1D (camera height, first frame);
// plane distance = camera height + peak value (z_components are camera-relative).
void Mesher::segmentHorizontalPlanes(std::vector<Plane>* horizontal_planes, size_t* plane_id,
                                     const Plane::Normal& normal, const cv::Mat& z_components) {
  ////////////////////////////// 1D Histogram //////////////////////////////////
  z_hist_.calculateHistogram(z_components, false);

  const cv::Size kernel_size(1, 3);
  std::vector<Histogram::PeakInfo> peaks;
  int min_support = static_cast<int>(z_components.rows / 10.0);
  if (min_support < 7.0) {
    min_support = 7;
  } else if (min_support > 20) {
    min_support = 20;
  }
  peaks = z_hist_.getLocalMaximum1D(kernel_size, g_z_histogram_window_size,
                                    static_cast<float>(g_z_histogram_peak_per),
                                    static_cast<float>(min_support), t_w_c_.z(),
                                    is_first_frame_);

  size_t i = 0;
  std::vector<Histogram::PeakInfo>::iterator previous_peak_it;
  double previous_plane_distance = -DBL_MAX;
  for (std::vector<Histogram::PeakInfo>::iterator peak_it = peaks.begin();
       peak_it != peaks.end();) {
    // Make sure it is below min possible value for distance.
    double plane_distance = peak_it->value_;

    // Remove duplicates, and, for peaks that are too close, take the one with
    // maximum support.
    // Assuming repeated peaks are ordered...
    if (i > 0 && *peak_it == peaks.at(i - 1)) {
      // Repeated element, delete it.
      peak_it = peaks.erase(peak_it);
      i--;
    } else if (i > 0 && std::fabs(previous_plane_distance - plane_distance) <
                            g_z_histogram_min_separation) {
      // Not enough separation between planes, delete the one with less support.
      if (previous_peak_it->support_ < peak_it->support_) {
        // Delete previous_peak.
        peaks.erase(previous_peak_it);
        peak_it = peaks.begin() + i - 1;
        i--;
      } else {
        // Delete peak_it.
        peak_it = peaks.erase(peak_it);
        i--;
      }
    } else {
      previous_peak_it = peak_it;
      previous_plane_distance = plane_distance;
      peak_it++;
      i++;
    }
  }

  for (int peak_nr = 0; peak_nr < g_z_histogram_max_number_of_peaks_to_select; peak_nr++) {
    // Get the peaks in order of max support.
    std::vector<Histogram::PeakInfo>::iterator it = std::max_element(peaks.begin(), peaks.end());
    if (it != peaks.end()) {
      double plane_distance = t_w_c_.z() + it->value_;
      static constexpr int cluster_id = 2;  // 2 = ground / horizontal.
      horizontal_planes->push_back(
          Plane(static_cast<int>(*plane_id), normal, plane_distance, cluster_id));
      (*plane_id)++;  // CRITICAL TO GET THIS RIGHT: ensure no duplicates.

      // Delete current peak from set of peaks, so that we can find next maximum.
      peaks.erase(it);
    } else {
      break;
    }
  }
}

/* -------------------------------------------------------------------------- */
// 0x1801A3310
// upstream-modified: getPolygon result unchecked (CHECK removed), distance-dependent scale
// passed to updatePlanesLmkIdsFromPolygon.
void Mesher::updatePlanesLmkIdsFromMesh(std::vector<Plane>* planes, double normal_tolerance,
                                        double distance_tolerance,
                                        const LmkPositionMap& points_with_id_vio) const {
#line 796
  CHECK_NOTNULL(planes);  // mesher.cpp:796
  static constexpr size_t mesh_polygon_dim = 3;
#line 798
  CHECK_EQ(mesh_3d.getMeshPolygonDimension(), mesh_polygon_dim) << "Expecting 3 vertices in triangle.";  // mesher.cpp:798
  Mesh3D::Polygon polygon;
  for (size_t i = 0; i < mesh_3d.getNumberOfPolygons(); i++) {
    mesh_3d.getPolygon(i, &polygon);
    const Vertex3D& p1 = polygon.at(0).getVertexPosition();
    const Vertex3D& p2 = polygon.at(1).getVertexPosition();
    const Vertex3D& p3 = polygon.at(2).getVertexPosition();

    // Calculate normal of the triangle in the mesh.
    cv::Point3f triangle_normal;
    if (calculateNormal(p1, p2, p3, &triangle_normal)) {
      const cv::Point3f centroid = (p1 + p2 + p3) / 3.0;
      const Eigen::Vector3f normal(triangle_normal.x, triangle_normal.y, triangle_normal.z);
      const float dist_to_cam =
          (Eigen::Vector3f(centroid.x, centroid.y, centroid.z) - t_w_c_).dot(normal);
      // Loop over newly segmented planes, and update lmk ids field if
      // the current polygon is on the plane.
      updatePlanesLmkIdsFromPolygon(
          planes, polygon, i, triangle_normal, normal_tolerance, distance_tolerance,
          std::min(1.0, std::max(0.3, static_cast<double>(std::fabs(dist_to_cam)))),
          points_with_id_vio, g_only_associate_a_polygon_to_a_single_plane);
    }
  }
}

/* -------------------------------------------------------------------------- */
// 0x1801A3750
// upstream-modified (heavily):
//  * only planes with is_valid_ are considered,
//  * walls (cluster 1): fixed tolerances 0.115 / 0.3 and a camera-distance gate (1.5 m);
//    QUIRK: the tolerance parameters are overwritten and stay overwritten for the following
//    planes of the loop,
//  * horizontal planes (cluster 2): distance tolerance *= distance_scale, compounding over the
//    loop (QUIRK), plus a per-vertex |z - distance| check,
//  * the polygon must project inside the plane hull, unless the plane is the current ground
//    plane (g_max_plane_id) and ground association is enabled,
//  * planes without a hull use a centroid-distance gate,
//  * triangle_cluster_.triangle_ids_ is no longer maintained; triangle_id and
//    points_with_id_vio are unused,
//  * horizontal planes far from the camera height are recorded in g_new_plane_ids.
bool Mesher::updatePlanesLmkIdsFromPolygon(std::vector<Plane>* seed_planes,
                                           const Mesh3D::Polygon& polygon,
                                           const size_t& triangle_id,
                                           const cv::Point3f& triangle_normal,
                                           double normal_tolerance, double distance_tolerance,
                                           double distance_scale,
                                           const LmkPositionMap& points_with_id_vio,
                                           bool only_associate_a_polygon_to_a_single_plane) const {
  (void)triangle_id;
  (void)points_with_id_vio;
  bool is_polygon_on_a_plane = false;
  for (Plane& seed_plane : *seed_planes) {
    if (!seed_plane.is_valid_) continue;

    bool plane_polygon_empty = false;
    if (seed_plane.cluster_id_ == 1) {
      normal_tolerance = 0.115;
      distance_tolerance = 0.3;
      plane_polygon_empty = seed_plane.polygon_.empty();
      const Eigen::Vector3f plane_normal(seed_plane.normal_.x, seed_plane.normal_.y,
                                         seed_plane.normal_.z);
      const float cam_dist = plane_normal.dot(t_w_c_);
      if (std::fabs(cam_dist - seed_plane.distance_) > 1.5) continue;
    } else if (seed_plane.cluster_id_ == 2) {
      distance_tolerance = distance_tolerance * distance_scale;
    }

    // Only cluster if normal and distance of polygon are close to plane.
    if (!(isNormalAroundAxis(seed_plane.normal_, triangle_normal, normal_tolerance) &&
          isPolygonAtDistanceFromPlane(polygon, seed_plane.distance_, seed_plane.normal_,
                                       distance_tolerance))) {
      continue;
    }

    if (!seed_plane.polygon_.empty()) {
      if (!isPolygonInPlaneHull(polygon, seed_plane)) {
        if (!g_ground_plane_association) continue;
        if (seed_plane.cluster_id_ == 1) continue;
        if (seed_plane.cluster_id_ == 2 &&
            std::fabs(seed_plane.distance_ - t_w_c_.z()) < 1.2) {
          continue;
        }
        if (seed_plane.id != g_max_plane_id) continue;
      }
    }

    if (seed_plane.cluster_id_ == 2) {
      bool vertices_close = true;
      for (const Mesh3D::VertexType& vertex : polygon) {
        if (std::fabs(vertex.getVertexPosition().z - seed_plane.distance_) > distance_tolerance) {
          vertices_close = false;
          break;
        }
      }
      if (!vertices_close) continue;
    }

    if (seed_plane.polygon_.empty()) {
      if (seed_plane.lmk_ids_.empty()) {
        // First polygon of the plane: its centroid.
        for (const Mesh3D::VertexType& vertex : polygon) {
          const Vertex3D& p = vertex.getVertexPosition();
          seed_plane.centroid_ += Eigen::Vector3f(p.x, p.y, p.z);
        }
        seed_plane.centroid_ /= 3.0f;
      }
      if (!seed_plane.lmk_ids_.empty()) {
        bool is_close = false;
        for (const Mesh3D::VertexType& vertex : polygon) {
          const Vertex3D& p = vertex.getVertexPosition();
          if (distance_scale * 0.5 >
                  (Eigen::Vector3f(p.x, p.y, p.z) - seed_plane.centroid_).norm() ||
              is_first_frame_) {
            is_close = true;
            break;
          }
        }
        if (!is_close) continue;
      }
    }

    // Update lmk_ids of seed plane.
    appendLmkIdsOfPolygon(polygon, &seed_plane.lmk_ids_);

    if (seed_plane.cluster_id_ == 1 && plane_polygon_empty) {
      // Orient the hull of a new wall so that it faces the camera.
      const cv::Point3f centroid = (polygon[0].getVertexPosition() +
                                    polygon[1].getVertexPosition() +
                                    polygon[2].getVertexPosition()) / 3.0;
      const cv::Point3f cam(t_w_c_.x(), t_w_c_.y(), t_w_c_.z());
      if ((centroid - cam).ddot(seed_plane.normal_) > 0.0) {
        seed_plane.hull_clockwise_ = false;
      }
    }

    // Acknowledge that the polygon is at least in one plane.
    is_polygon_on_a_plane = true;
    if (g_ground_plane_association && seed_plane.cluster_id_ == 2 &&
        std::fabs(seed_plane.distance_ - t_w_c_.z()) >= 1.2) {
      if (std::find(g_new_plane_ids.begin(), g_new_plane_ids.end(), seed_plane.id) ==
          g_new_plane_ids.end()) {
        g_new_plane_ids.push_back(seed_plane.id);
      }
    }
    if (only_associate_a_polygon_to_a_single_plane) {
      break;
    }
  }
  return is_polygon_on_a_plane;
}

/* -------------------------------------------------------------------------- */
// 0x18019EF20  (pimax-new)
// Does any vertex of the triangle fall inside the plane hull, both rotated so that the plane
// normal becomes +Z?  Always true on the first frame or for planes without hull.
bool Mesher::isPolygonInPlaneHull(const Mesh3D::Polygon& polygon, const Plane& plane) const {
  if (!plane.polygon_.empty() && !is_first_frame_) {
    const Eigen::Vector3f plane_normal(plane.normal_.x, plane.normal_.y, plane.normal_.z);
    const Eigen::Matrix3f R =
        Eigen::Quaternionf::FromTwoVectors(plane_normal, Eigen::Vector3f(0.f, 0.f, 1.f))
            .toRotationMatrix();
    std::vector<cv::Point2f> plane_polygon_2d;
    for (const PolygonVertex& vertex : plane.polygon_) {
      const Eigen::Vector3f p = R * vertex.pos;
      plane_polygon_2d.push_back(cv::Point2f(p.x(), p.y()));
    }
    if (plane_polygon_2d.empty()) {
      return true;
    }
    for (const Mesh3D::VertexType& vertex : polygon) {
      const Vertex3D& pos = vertex.getVertexPosition();
      const Eigen::Vector3f p = R * Eigen::Vector3f(pos.x, pos.y, pos.z);
      if (isPointInPolygon(p.x(), p.y(), plane_polygon_2d)) {
        return true;
      }
    }
    return false;
  }
  return true;
}

/* -------------------------------------------------------------------------- */
// 0x1801A1500  (pimax-new)
// Recompute the convex hull of every valid plane from its previous hull vertices plus its
// landmarks that still lie within (distance tolerance + 0.05) of the plane.
void Mesher::updatePlanesPolygon(std::vector<Plane>* planes,
                                 const LmkPositionMap& points_with_id_vio) const {
  for (Plane& plane : *planes) {
    if (!plane.is_valid_) continue;

    const Eigen::Vector3f plane_normal(plane.normal_.x, plane.normal_.y, plane.normal_.z);
    const Eigen::Matrix3f R =
        Eigen::Quaternionf::FromTwoVectors(plane_normal, Eigen::Vector3f(0.f, 0.f, 1.f))
            .toRotationMatrix();

    std::vector<cv::Point2f> points_2d;
    std::vector<int> hull_indices;
    std::vector<PolygonVertex> candidates;

    for (const PolygonVertex& vertex : plane.polygon_) {
      const Eigen::Vector3f pos = vertex.pos;
      if (std::find_if(candidates.begin(), candidates.end(),
                       [&vertex](const PolygonVertex& c) { return c.lmk_id == vertex.lmk_id; }) ==
          candidates.end()) {
        candidates.push_back(vertex);
        const Eigen::Vector3f p = R * pos;
        points_2d.push_back(cv::Point2f(p.x(), p.y()));
      }
    }

    for (const LandmarkId& lmk_id : plane.lmk_ids_) {
      const auto it = points_with_id_vio.find(lmk_id);
      if (it != points_with_id_vio.end()) {
        const Eigen::Vector3f pos = it->second;
        if (std::fabs(pos.dot(plane_normal) - plane.distance_) <=
            g_distance_tolerance_polygon_plane_association + 0.05) {
          if (std::find_if(candidates.begin(), candidates.end(),
                           [&lmk_id](const PolygonVertex& c) { return c.lmk_id == lmk_id; }) ==
              candidates.end()) {
            PolygonVertex candidate;
            candidate.lmk_id = lmk_id;
            candidate.pos = pos;
            candidates.push_back(candidate);
            const Eigen::Vector3f p = R * pos;
            points_2d.push_back(cv::Point2f(p.x(), p.y()));
          }
        }
      }
    }

    if (!points_2d.empty()) {
      cv::convexHull(points_2d, hull_indices, plane.hull_clockwise_, true);
      if (!hull_indices.empty()) {
        plane.polygon_.clear();
        for (const int& index : hull_indices) {
          plane.polygon_.push_back(candidates[index]);
        }
        for (const LandmarkId& lmk_id : plane.lmk_ids_) {
          if (std::find(plane.all_lmk_ids_.begin(), plane.all_lmk_ids_.end(), lmk_id) ==
              plane.all_lmk_ids_.end()) {
            plane.all_lmk_ids_.push_back(lmk_id);
          }
        }
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
// 0x18019FA90
// upstream-modified: only valid planes of the same cluster are compared; tolerances are
// scaled by the mean camera distance of both planes (clamped to [0.3, 1]); walls get a second,
// looser normal test; an association additionally requires overlapping hulls (in the frame of
// the backend plane; walls also test landmarks rotated into that frame); associated segmented
// planes and the backend ids are returned.  QUIRK: with !g_do_double_association a double
// association is still appended to the outputs before searching on.
void Mesher::associatePlanes(const std::vector<Plane>& segmented_planes,
                             const std::vector<Plane>& planes,
                             std::vector<Plane>* non_associated_planes,
                             std::vector<Plane>* associated_segmented_planes,
                             LandmarkIds* associated_plane_ids,
                             const LmkPositionMap& points_with_id_vio,
                             const double& normal_tolerance,
                             const double& distance_tolerance) const {
  non_associated_planes->clear();
  if (planes.size() == 0) {
    // There are no previous planes, data association unnecessary, just copy
    // segmented planes to output planes.
    *non_associated_planes = segmented_planes;
  } else {
    // To avoid associating several segmented planes to the same plane_backend
    std::vector<uint64_t> backend_plane_ids;
    LmkPositionMap rotated_points;
    for (const Plane& segmented_plane : segmented_planes) {
      bool is_segmented_plane_associated = false;
      for (const Plane& plane_backend : planes) {
        if (!plane_backend.is_valid_) continue;
        if (segmented_plane.cluster_id_ != plane_backend.cluster_id_) continue;

        const Eigen::Vector3f backend_normal(plane_backend.normal_.x, plane_backend.normal_.y,
                                             plane_backend.normal_.z);
        const Eigen::Vector3f segmented_normal(segmented_plane.normal_.x,
                                               segmented_plane.normal_.y,
                                               segmented_plane.normal_.z);
        const float backend_cam_dist = backend_normal.dot(t_w_c_);
        const float segmented_cam_dist = t_w_c_.dot(segmented_normal);
        const double scale = std::min(
            1.0, std::max(0.3, std::fabs((backend_cam_dist - plane_backend.distance_ +
                                          segmented_cam_dist - segmented_plane.distance_) *
                                         0.5)));

        if (!(plane_backend.geometricEqual(segmented_plane, normal_tolerance * scale,
                                           scale * distance_tolerance) ||
              (plane_backend.cluster_id_ == 1 && segmented_plane.cluster_id_ == 1 &&
               plane_backend.geometricEqual(segmented_plane, (normal_tolerance + 0.1) * scale,
                                            scale * distance_tolerance)))) {
          continue;
        }

        // Check the hulls in the frame of the backend plane.
        std::vector<PolygonVertex> backend_polygon;
        std::vector<PolygonVertex> segmented_polygon;
        const Eigen::Matrix3f R =
            Eigen::Quaternionf::FromTwoVectors(backend_normal, Eigen::Vector3f(0.f, 0.f, 1.f))
                .toRotationMatrix();
        for (const PolygonVertex& vertex : plane_backend.polygon_) {
          PolygonVertex rotated;
          rotated.pos = R * vertex.pos;
          rotated.lmk_id = vertex.lmk_id;
          backend_polygon.push_back(rotated);
        }
        for (const PolygonVertex& vertex : segmented_plane.polygon_) {
          PolygonVertex rotated;
          rotated.pos = R * vertex.pos;
          rotated.lmk_id = vertex.lmk_id;
          segmented_polygon.push_back(rotated);
        }
        if (plane_backend.cluster_id_ == 1) {
          rotated_points.clear();
          rotated_points.reserve(points_with_id_vio.size());
          for (const auto& point : points_with_id_vio) {
            rotated_points.insert(LmkPositionMap::value_type(point.first, R * point.second));
          }
        }
        bool overlap;
        if (plane_backend.cluster_id_ == 1) {
          overlap = polygonsOverlap(backend_polygon, plane_backend.all_lmk_ids_,
                                    segmented_polygon, segmented_plane.all_lmk_ids_,
                                    rotated_points);
        } else {
          overlap = polygonsOverlap(backend_polygon, plane_backend.all_lmk_ids_,
                                    segmented_polygon, segmented_plane.all_lmk_ids_,
                                    points_with_id_vio);
        }
        if (!overlap) continue;

        // We found a plane association.
        const int backend_plane_id = plane_backend.id;
        associated_segmented_planes->push_back(segmented_plane);
        associated_plane_ids->push_back(backend_plane_id);
        // Check that it was not associated before.
        if (std::find(backend_plane_ids.begin(), backend_plane_ids.end(), backend_plane_id) ==
            backend_plane_ids.end()) {
          // It is the first time we associate this plane.
          backend_plane_ids.emplace_back(backend_plane_id);
          is_segmented_plane_associated = true;
          break;
        } else {
          // Double plane association of backend plane.
          if (g_do_double_association) {
            is_segmented_plane_associated = true;
            break;
          } else {
            continue;
          }
        }
      }

      if (!is_segmented_plane_associated) {
        // The segmented plane could not be associated to any existing plane: add it.
        non_associated_planes->push_back(segmented_plane);
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
// 0x1801A0AF0
// upstream-modified: new_planes is an output owned by the caller; hulls are refreshed; weak
// new horizontal planes are dropped; associated planes merge hull + landmarks into the backend
// plane; ground-plane bookkeeping in g_new_plane_ids.
void Mesher::clusterPlanesFromMesh(std::vector<Plane>* planes, std::vector<Plane>* new_planes,
                                   const LmkPositionMap& points_with_id_vio) {
  is_first_frame_ = planes->empty();

  // Segment planes in the mesh, using seeds.
  const double normal_tolerance_walls = g_normal_tolerance_walls + 0.115;
  segmentPlanesInMesh(planes, new_planes, points_with_id_vio,
                      g_normal_tolerance_polygon_plane_association,
                      g_distance_tolerance_polygon_plane_association,
                      g_normal_tolerance_horizontal_surface, normal_tolerance_walls);
  updatePlanesPolygon(planes, points_with_id_vio);

  if (new_planes->size() > 0) {
    // Update lmk ids of the newly segmented planes.
    updatePlanesLmkIdsFromMesh(new_planes, g_normal_tolerance_polygon_plane_association,
                               g_distance_tolerance_polygon_plane_association,
                               points_with_id_vio);

    // Drop new horizontal planes with too few landmarks (more needed when close to the camera).
    for (auto it = new_planes->begin(); it != new_planes->end();) {
      const Eigen::Vector3f plane_normal(it->normal_.x, it->normal_.y, it->normal_.z);
      const float cam_dist = plane_normal.dot(t_w_c_);
      const double scale = std::min(1.0, std::max(0.5, std::fabs(cam_dist - it->distance_)));
      if (it->cluster_id_ == 1) {
        printf("plane->lmk_ids_map size: %lu", it->lmk_ids_.size());
      }
      if (it->cluster_id_ == 2 &&
          (((g_plane_min_lmk_num - 2) / scale > it->lmk_ids_.size() && !is_first_frame_) ||
           (it->lmk_ids_.size() < static_cast<size_t>(g_plane_min_lmk_num - 2) &&
            is_first_frame_))) {
        it = new_planes->erase(it);
      } else {
        ++it;
      }
    }
    updatePlanesPolygon(new_planes, points_with_id_vio);

    // Do data association between the planes given and the ones segmented.
    std::vector<Plane> new_non_associated_planes;
    std::vector<Plane> associated_planes;
    LandmarkIds associated_plane_ids;
    associatePlanes(*new_planes, *planes, &new_non_associated_planes, &associated_planes,
                    &associated_plane_ids, points_with_id_vio,
                    g_normal_tolerance_plane_plane_association,
                    g_distance_tolerance_plane_plane_association);

    // Append new planes that where not associated to original planes.
    for (const Plane& new_plane : new_non_associated_planes) {
      planes->push_back(new_plane);
    }

    // Merge the associated segmented planes into their backend planes.
    for (size_t i = 0; i < associated_planes.size(); i++) {
      const Plane associated_plane = associated_planes[i];
      const int plane_id = associated_plane_ids[i];
      for (Plane& plane : *planes) {
        if (plane.is_valid_ && plane.id == plane_id) {
          const Eigen::Vector3f plane_normal(plane.normal_.x, plane.normal_.y, plane.normal_.z);
          const float cam_dist = plane_normal.dot(t_w_c_);
          const double scale =
              std::min(1.0, std::max(0.5, std::fabs(cam_dist - plane.distance_)));
          mergePolygons(&plane.polygon_, associated_plane.polygon_, plane_normal,
                        &plane.hull_clockwise_);
          for (const LandmarkId& lmk_id : associated_plane.lmk_ids_) {
            if (std::find(plane.lmk_ids_.begin(), plane.lmk_ids_.end(), lmk_id) !=
                plane.lmk_ids_.end()) {
              continue;
            }
            const auto it = points_with_id_vio.find(lmk_id);
            if (it == points_with_id_vio.end()) continue;
            if (plane.cluster_id_ == 2 &&
                std::fabs(plane.distance_ - it->second.z()) >
                    (g_distance_tolerance_polygon_plane_association + 0.05) * scale) {
              continue;
            }
            plane.lmk_ids_.push_back(lmk_id);
            if (g_ground_plane_association && plane.cluster_id_ == 2 &&
                std::fabs(plane.distance_ - t_w_c_.z()) >= 1.2) {
              if (std::find(g_new_plane_ids.begin(), g_new_plane_ids.end(), plane.id) ==
                  g_new_plane_ids.end()) {
                g_new_plane_ids.push_back(plane.id);
              }
              const auto old_it =
                  std::find(g_new_plane_ids.begin(), g_new_plane_ids.end(), associated_plane.id);
              if (old_it != g_new_plane_ids.end()) {
                g_new_plane_ids.erase(old_it);
              }
            }
          }
        }
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
// 0x18019F330  (pimax-new; W. R. Franklin's PNPOLY)
bool isPointInPolygon(float x, float y, const std::vector<cv::Point2f>& polygon) {
  if (polygon.empty()) {
    return true;
  }
  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    if (((polygon[i].y > y) != (polygon[j].y > y)) &&
        (x < (polygon[j].x - polygon[i].x) * (y - polygon[i].y) / (polygon[j].y - polygon[i].y) +
                 polygon[i].x)) {
      inside = !inside;
    }
  }
  return inside;
}

/* -------------------------------------------------------------------------- */
// 0x18019E480  (pimax-new)
bool polygonsOverlap(const std::vector<PolygonVertex>& polygon_a, std::vector<int> lmk_ids_a,
                     const std::vector<PolygonVertex>& polygon_b, std::vector<int> lmk_ids_b,
                     const LmkPositionMap& lmk_positions) {
  if (polygon_a.empty()) {
    return true;
  }
  if (polygon_b.empty()) {
    return false;
  }

  std::vector<cv::Point2f> contour;
  // Vertices of b inside a?
  int count = 0;
  for (const PolygonVertex& vertex : polygon_a) {
    contour.push_back(cv::Point2f(vertex.pos.x(), vertex.pos.y()));
  }
  for (const PolygonVertex& vertex : polygon_b) {
    if (isPointInPolygon(vertex.pos.x(), vertex.pos.y(), contour)) {
      if (++count >= g_plane_overlap_min_points) return true;
    }
  }

  // Vertices of a inside b?
  contour.clear();
  count = 0;
  for (const PolygonVertex& vertex : polygon_b) {
    contour.push_back(cv::Point2f(vertex.pos.x(), vertex.pos.y()));
  }
  for (const PolygonVertex& vertex : polygon_a) {
    if (isPointInPolygon(vertex.pos.x(), vertex.pos.y(), contour)) {
      if (++count >= g_plane_overlap_min_points) return true;
    }
  }

  if (lmk_positions.size() == 0 || lmk_ids_a.empty() || lmk_ids_b.empty()) {
    return false;
  }

  // Landmarks of a inside b?
  contour.clear();
  count = 0;
  for (const PolygonVertex& vertex : polygon_b) {
    contour.push_back(cv::Point2f(vertex.pos.x(), vertex.pos.y()));
  }
  for (const int& lmk_id : lmk_ids_a) {
    const auto it = lmk_positions.find(lmk_id);
    if (it != lmk_positions.end()) {
      if (isPointInPolygon(it->second.x(), it->second.y(), contour)) {
        if (++count >= g_plane_overlap_min_points) return true;
      }
    }
  }

  // Landmarks of b inside a?
  contour.clear();
  count = 0;
  for (const PolygonVertex& vertex : polygon_a) {
    contour.push_back(cv::Point2f(vertex.pos.x(), vertex.pos.y()));
  }
  for (const int& lmk_id : lmk_ids_b) {
    const auto it = lmk_positions.find(lmk_id);
    if (it != lmk_positions.end()) {
      if (isPointInPolygon(it->second.x(), it->second.y(), contour)) {
        if (++count >= g_plane_overlap_min_points) return true;
      }
    }
  }
  return false;
}

/* -------------------------------------------------------------------------- */
// 0x18019CA00  (pimax-new)
// QUIRK 1: the rotation is FromTwoVectors(+Z, normal), i.e. the inverse of the one used by
//          isPolygonInPlaneHull / updatePlanesPolygon / associatePlanes.
// QUIRK 2: when polygon_a is not empty the merged hull is computed and thrown away; polygon_a
//          is only ever filled when it was empty.
void mergePolygons(std::vector<PolygonVertex>* polygon_a,
                   const std::vector<PolygonVertex>& polygon_b, const Eigen::Vector3f& normal_a,
                   bool* clockwise) {
  std::vector<cv::Point2f> points_2d;
  std::vector<int> hull_indices;
  const Eigen::Matrix3f R =
      Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0.f, 0.f, 1.f), normal_a)
          .toRotationMatrix();
  if (polygon_a->empty()) {
    polygon_a->insert(polygon_a->begin(), polygon_b.begin(), polygon_b.end());
  } else {
    std::vector<PolygonVertex> merged;
    std::vector<int> merged_ids;
    merged.reserve(polygon_a->size() + polygon_b.size());
    merged_ids.reserve(polygon_b.size() + polygon_a->size());
    for (const PolygonVertex& vertex : polygon_b) {
      const Eigen::Vector3f p = R * vertex.pos;
      points_2d.push_back(cv::Point2f(p.x(), p.y()));
      merged.push_back(vertex);
      merged_ids.push_back(vertex.lmk_id);
    }
    for (const PolygonVertex& vertex : *polygon_a) {
      const Eigen::Vector3f p = R * vertex.pos;
      if (std::find(merged_ids.begin(), merged_ids.end(), vertex.lmk_id) == merged_ids.end()) {
        points_2d.push_back(cv::Point2f(p.x(), p.y()));
        merged.push_back(vertex);
        merged_ids.push_back(vertex.lmk_id);
      }
    }
    if (!points_2d.empty()) {
      cv::convexHull(points_2d, hull_indices, *clockwise, true);
    }
  }
}

}  // namespace totem
}  // namespace pimax
