// src/plane/plane.h  (TODO(verify) file name; no __FILE__ string references it)
//
// Plane / mesh helper types used by src/plane/mesher.cpp (chunk c17) and by the
// frontend (FrameProcessorBase ground-plane code, chunks c08/c09/c10).
// Names follow draft/c08_preint_ground/plane/plane_types.h where c08 proved them;
// this header adds what c17 proves (ctor, +112 centroid, +48 map type, geometricEqual).
//
// Kimera-VIO origin: VIO::Plane (utils/UtilsOpenCV.h), heavily modified by Pimax.
#pragma once

#include <cmath>
#include <cstddef>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace pimax {
namespace totem {

typedef int LandmarkId;                     // Kimera: long; here 4 bytes (Mesh vertex lmk id is a dword, -1 default)
typedef std::vector<LandmarkId> LandmarkIds;

// Landmark world positions keyed by landmark id.  Node {next, prev, int key(+16), float[3](+20)},
// FNV-1a over the 4 key bytes (std::hash<int>).  Kimera: PointsWithIdMap (gtsam::Point3 values).
// Value type: c08 proves Eigen float use; in c17 every use first copies the 12 bytes into a local
// (consistent with either Eigen::Vector3f or cv::Point3f).  Kept as Eigen::Vector3f (c08).
using LmkPositionMap = std::unordered_map<LandmarkId, Eigen::Vector3f>;

// Mesher +1152 (0x480): pixel of every landmark of the current frame, filled by
// FrameProcessorBase (0x1800F2070).  Node size 0x30: key int (+16), value 16-aligned at +32.
// c17 compares value[0] (node+32) with pixel.x and value[1] (node+40) with pixel.y (as doubles).
using LmkPixelMap = std::unordered_map<LandmarkId, Eigen::Vector2d>;  // TODO(verify) value type (c08: Vector2d)

// 16-byte hull vertex {lmk id, world position}.  Copied as dword + qword + dword.
struct PolygonVertex {
  int lmk_id;            // +0
  Eigen::Vector3f pos;   // +4
};
static_assert(sizeof(PolygonVertex) == 16, "PolygonVertex");

// sizeof == 232 (0xE8).  Copy ctor 0x1800E2910 (c09), move ctor 0x18019C640 (c17, compiler
// generated), dtor 0x1800E5230 (c09), element copy-assign loop 0x18019ADC0 (c17, compiler
// generated operator=).
//
//   off  type                                 name                 evidence
//   +0   int                                  id                   compared with plane ids, pushed to g_new_plane_ids (sure)
//   +4   cv::Point3f                          normal_              ddot with triangle normals (sure)
//   +16  double                               distance_            (sure)
//   +24  std::vector<cv::Point3f>             unknown_24_          12-byte elements, only cleared/copied here (type sure, name unknown)
//   +48  std::unordered_map<int, Vector3f>    lmk_ids_map_         node {key int, 12-byte value} (0x18019A5D0); "plane->lmk_ids_map" log string (name likely)
//   +112 Eigen::Vector3f                      centroid_            accumulated from the first polygon, /3.0f, Eigen norm vs. vertices (sure)
//   +128 LandmarkIds                          lmk_ids_             (sure)
//   +152 std::vector<PolygonVertex>           polygon_             convex hull vertices (sure)
//   +176 int                                  cluster_id_          1 = wall, 2 = horizontal (sure)
//   +184 LandmarkIds                          all_lmk_ids_         union of lmk_ids_ after each hull update (sure)
//   +208 bool                                 is_valid_            only planes with it set are processed (name unsure)
//   +209 bool                                 hull_clockwise_      convexHull `clockwise`, cleared for walls facing the camera (name unsure)
//   +210 bool                                 needs_wall_refit_    c08 (name unsure)
//   +212 int                                  age_                 c08 (name unsure)
//   +216 double                               distance_ref_        c08 (name unsure)
//   +224 double                               area_                c08
struct Plane {
  typedef cv::Point3f Normal;  // Kimera: cv::Point3d

  Plane() = default;

  // Inlined in Mesher::segmentHorizontalPlanes (0x1801A1DB0).  The stores in the binary are:
  // member init (+208 = 1, +176 = cluster_id), then clear() of +24, +48, +152, +128, +184, then
  // +208/+209 = 1/1, +210 = 1, +212 = 0, +216 = 0, +224 = 0.  TODO(verify) exact source form.
  Plane(int plane_id, const Normal& normal, double distance, int cluster_id)
      : id(plane_id), normal_(normal), distance_(distance), cluster_id_(cluster_id) {
    unknown_24_.clear();
    lmk_ids_map_.clear();
    polygon_.clear();
    lmk_ids_.clear();
    all_lmk_ids_.clear();
    is_valid_ = true;
    hull_clockwise_ = true;
    needs_wall_refit_ = true;
    age_ = 0;
    distance_ref_ = 0.0;
    area_ = 0.0;
  }

  // 0x1801A13D0 (emitted out of line once, inlined once in Mesher::associatePlanes).
  // Kimera Plane::geometricEqual with isNormalStrictlyEqual / isPlaneDistanceStrictlyEqual
  // inlined and the CHECKs removed.
  bool geometricEqual(const Plane& rhs, const double& normal_tolerance,
                      const double& distance_tolerance) const {
    return (isNormalStrictlyEqual(rhs.normal_, normal_, normal_tolerance) &&
            isPlaneDistanceStrictlyEqual(rhs.distance_, distance_, distance_tolerance)) ||
           (isNormalStrictlyEqual(rhs.normal_, -normal_, normal_tolerance) &&
            isPlaneDistanceStrictlyEqual(rhs.distance_, -distance_, distance_tolerance));
  }

  int id = 0;                                   // +0
  Normal normal_;                               // +4
  double distance_ = 0.0;                       // +16
  std::vector<cv::Point3f> unknown_24_;         // +24  TODO(verify) meaning
  LmkPositionMap lmk_ids_map_;                  // +48  TODO(verify) value type (12 bytes)
  Eigen::Vector3f centroid_ = Eigen::Vector3f::Zero();  // +112
  LandmarkIds lmk_ids_;                         // +128
  std::vector<PolygonVertex> polygon_;          // +152
  int cluster_id_ = 0;                          // +176
  LandmarkIds all_lmk_ids_;                     // +184
  bool is_valid_ = true;                        // +208
  bool hull_clockwise_ = true;                  // +209
  bool needs_wall_refit_ = true;                // +210
  int age_ = 0;                                 // +212
  double distance_ref_ = 0.0;                   // +216
  double area_ = 0.0;                           // +224

 private:
  // normal.ddot(axis) > 1 - tol   (double products, order x, y, z)
  static bool isNormalStrictlyEqual(const Normal& axis, const Normal& normal,
                                    const double& tolerance) {
    return normal.ddot(axis) > 1.0 - tolerance;
  }
  static bool isPlaneDistanceStrictlyEqual(const double& a, const double& b,
                                           const double& tolerance) {
    return std::fabs(a - b) < tolerance;
  }
};
static_assert(sizeof(Plane) == 232, "Plane size");
static_assert(offsetof(Plane, normal_) == 4, "Plane::normal_");
static_assert(offsetof(Plane, distance_) == 16, "Plane::distance_");
static_assert(offsetof(Plane, unknown_24_) == 24, "Plane::unknown_24_");
static_assert(offsetof(Plane, lmk_ids_map_) == 48, "Plane::lmk_ids_map_");
static_assert(offsetof(Plane, centroid_) == 112, "Plane::centroid_");
static_assert(offsetof(Plane, lmk_ids_) == 128, "Plane::lmk_ids_");
static_assert(offsetof(Plane, polygon_) == 152, "Plane::polygon_");
static_assert(offsetof(Plane, cluster_id_) == 176, "Plane::cluster_id_");
static_assert(offsetof(Plane, all_lmk_ids_) == 184, "Plane::all_lmk_ids_");
static_assert(offsetof(Plane, is_valid_) == 208, "Plane::is_valid_");
static_assert(offsetof(Plane, hull_clockwise_) == 209, "Plane::hull_clockwise_");
static_assert(offsetof(Plane, needs_wall_refit_) == 210, "Plane::needs_wall_refit_");
static_assert(offsetof(Plane, age_) == 212, "Plane::age_");
static_assert(offsetof(Plane, distance_ref_) == 216, "Plane::distance_ref_");
static_assert(offsetof(Plane, area_) == 224, "Plane::area_");

}  // namespace totem
}  // namespace pimax
