// src/plane/mesh.h  (TODO(verify) path) -- Kimera-VIO mesh/Mesh.h (v4/v5 era, mid/late 2019),
// Pimax-modified:
//   * per-vertex colours removed entirely (no vertex_color_, no vertices_mesh_color_):
//     Vertex<Point3f> is 28 bytes {int lmk_id; Point3f position; Point3f normal},
//     Vertex<Point2f> is 24 bytes {int lmk_id; Point2f position; Point3f normal},
//   * LandmarkId is a 4-byte int,
//   * CHECK/CHECK_NOTNULL/DCHECK turned into plain conditions (see mesh.cpp),
//   * the CHECK_EQ in operator= turned into a std::cout error line.
// sizeof(Mesh<...>) == 0x150 (336): Mesher has two Mesh3D at +0 and +0x150.
#pragma once

#include <cstddef>
#include <map>
#include <vector>

#include <opencv2/core.hpp>

#include "plane/plane.h"   // LandmarkId

namespace pimax {
namespace totem {

template <typename VertexPosition = cv::Point3f>
class Mesh {
 public:
  typedef cv::Point3f VertexNormal;

 public:
  // 0x1801996D0 (identical code for Mesh2D/Mesh3D, COMDAT-folded).  Kimera's
  // CHECK_GE(polygon_dimension, 3) was removed.
  Mesh(const size_t& polygon_dimension = 3);
  // 0x1801997A0 (Mesh3D; the Mesh2D instance is unreferenced)
  Mesh& operator=(const Mesh& mesh);
  Mesh(Mesh&& mesh) = default;
  Mesh& operator=(Mesh&& mesh) = delete;
  ~Mesh() = default;   // 0x180009730 (other chunk; destroys 3 Mats + 2 maps)

 private:
  typedef int VertexId;
  typedef std::map<VertexId, LandmarkId> VertexToLmkIdMap;
  typedef std::map<LandmarkId, VertexId> LmkIdToVertexMap;

 public:
  template <typename PositionType = cv::Point3f>
  struct Vertex {
   public:
    Vertex() : lmk_id_(-1), vertex_position_(), vertex_normal_() {}
    Vertex(const LandmarkId& lmk_id, const VertexPosition& vertex_position,
           const VertexNormal& vertex_normal = VertexNormal())
        : lmk_id_(lmk_id), vertex_position_(vertex_position), vertex_normal_(vertex_normal) {}

    Vertex(const Vertex& rhs_mesh_vertex) = default;
    Vertex& operator=(const Vertex& rhs_mesh_vertex) = default;
    Vertex(Vertex&& rhs_mesh_vertex) = default;
    Vertex& operator=(Vertex&& rhs_mesh_vertex) = default;
    ~Vertex() = default;

    inline const VertexPosition& getVertexPosition() const { return vertex_position_; }
    inline const VertexNormal& getVertexNormal() const { return vertex_normal_; }
    inline const LandmarkId& getLmkId() const { return lmk_id_; }
    inline void setVertexPosition(const VertexPosition& position) { vertex_position_ = position; }

   private:
    LandmarkId lmk_id_;               // +0
    VertexPosition vertex_position_;  // +4
    VertexNormal vertex_normal_;      // +4 + sizeof(VertexPosition)
  };
  typedef Vertex<VertexPosition> VertexType;
  typedef std::vector<VertexType> Polygon;

 public:
  // 0x180199A00 (Mesh3D, updateMeshDataStructures inlined), 0x180199C30 (Mesh2D)
  void addPolygonToMesh(const Polygon& polygon);
  // 0x180199D30 (identical code for both instantiations)
  void clearMesh();

  inline size_t getNumberOfPolygons() const {
    return static_cast<size_t>(polygons_mesh_.rows / (polygon_dimension_ + 1));
  }
  inline size_t getNumberOfUniqueVertices() const { return vertices_mesh_.rows; }
  inline size_t getMeshPolygonDimension() const { return polygon_dimension_; }

  // 0x180199F40 (Mesh3D)
  bool getPolygon(const size_t& polygon_idx, Polygon* polygon) const;

 private:
  // 0x18019A300 (Mesh2D; inlined into the Mesh3D addPolygonToMesh)
  void updateMeshDataStructures(const LandmarkId& lmk_id, const VertexPosition& lmk_position,
                                std::map<VertexId, LandmarkId>* vertex_to_lmk_id_map,
                                std::map<LandmarkId, VertexId>* lmk_id_to_vertex_map,
                                cv::Mat* vertices_mesh,
                                cv::Mat_<VertexNormal>* vertices_mesh_normal,
                                cv::Mat* polygon_mesh) const;

 private:
  VertexToLmkIdMap vertex_to_lmk_id_map_;        // +0x00
  LmkIdToVertexMap lmk_id_to_vertex_map_;        // +0x10
  cv::Mat vertices_mesh_;                        // +0x20
  cv::Mat_<VertexNormal> vertices_mesh_normal_;  // +0x80
  bool normals_computed_ = false;                // +0xE0
  cv::Mat polygons_mesh_;                        // +0xE8
  const size_t polygon_dimension_;               // +0x148

  friend struct MeshLayout;
};

typedef cv::Point2f Vertex2D;
typedef Mesh<Vertex2D> Mesh2D;
typedef cv::Point3f Vertex3D;
typedef Mesh<Vertex3D> Mesh3D;

struct MeshLayout {
  static_assert(sizeof(Mesh3D) == 0x150, "sizeof(Mesh3D)");
  static_assert(sizeof(Mesh2D) == 0x150, "sizeof(Mesh2D)");
  static_assert(offsetof(Mesh3D, vertices_mesh_) == 0x20, "Mesh::vertices_mesh_");
  static_assert(offsetof(Mesh3D, vertices_mesh_normal_) == 0x80, "Mesh::vertices_mesh_normal_");
  static_assert(offsetof(Mesh3D, normals_computed_) == 0xE0, "Mesh::normals_computed_");
  static_assert(offsetof(Mesh3D, polygons_mesh_) == 0xE8, "Mesh::polygons_mesh_");
  static_assert(offsetof(Mesh3D, polygon_dimension_) == 0x148, "Mesh::polygon_dimension_");
  static_assert(sizeof(Mesh3D::VertexType) == 28, "sizeof(Vertex<Point3f>)");
  static_assert(sizeof(Mesh2D::VertexType) == 24, "sizeof(Vertex<Point2f>)");
};

}  // namespace totem
}  // namespace pimax
