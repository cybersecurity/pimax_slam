// src/plane/mesh.cpp (TODO(verify) path) -- Kimera-VIO src/mesh/Mesh.cpp, Pimax-modified.
// The object sits right before mesher.cpp in the image (0x180198C90..0x18019A5D0, interleaved
// with the std::map / std::vector / cv::Mat::push_back instantiations it needs).
#include "plane/mesh.h"

#include <iostream>

#include <opencv2/core/core.hpp>

namespace pimax {
namespace totem {

/* -------------------------------------------------------------------------- */
// 0x1801996D0
template <typename VertexPositionType>
Mesh<VertexPositionType>::Mesh(const size_t& polygon_dimension)
    : vertex_to_lmk_id_map_(),
      lmk_id_to_vertex_map_(),
      vertices_mesh_(0, 1, CV_32FC3),
      vertices_mesh_normal_(0, 1),
      normals_computed_(false),
      polygons_mesh_(0, 1, CV_32SC1),
      polygon_dimension_(polygon_dimension) {}

/* -------------------------------------------------------------------------- */
// 0x1801997A0
template <typename VertexPositionType>
Mesh<VertexPositionType>& Mesh<VertexPositionType>::operator=(
    const Mesh<VertexPositionType>& rhs_mesh) {
  // Check for self-assignment.
  if (&rhs_mesh == this) return *this;
  if (polygon_dimension_ != rhs_mesh.polygon_dimension_) {
    std::cout << "Error: The Mesh that you are trying to copy has different dimensions for the "
                 "polygons!"
              << std::endl;
  }
  // Deep copy internal data.
  // NOTE: the binary implements the map copies as clear() + range insert (0x180012AB0 +
  // 0x180199300); that is the STL's operator= codegen.  TODO(verify) against the STL in use.
  lmk_id_to_vertex_map_ = rhs_mesh.lmk_id_to_vertex_map_;
  vertex_to_lmk_id_map_ = rhs_mesh.vertex_to_lmk_id_map_;
  vertices_mesh_ = rhs_mesh.vertices_mesh_.clone();
  vertices_mesh_normal_ = rhs_mesh.vertices_mesh_normal_.clone();  // Mat_<Point3f>::clone 0x180199E10
  normals_computed_ = rhs_mesh.normals_computed_;
  polygons_mesh_ = rhs_mesh.polygons_mesh_.clone();
  return *this;
}

/* -------------------------------------------------------------------------- */
// 0x180199A00 (Mesh3D) / 0x180199C30 (Mesh2D)
// upstream-modified: CHECK_EQ(polygon.size(), polygon_dimension_) became a condition.
template <typename VertexPositionType>
void Mesh<VertexPositionType>::addPolygonToMesh(const Polygon& polygon) {
  if (polygon.size() == polygon_dimension_) {
    // Reset flag to know if normals are valid or not.
    normals_computed_ = false;
    // Specify number of point ids per face in the mesh.
    polygons_mesh_.push_back(static_cast<int>(polygon_dimension_));
    // Loop over each vertex in the given polygon.
    for (const VertexType& vertex : polygon) {
      // Add or update vertex in the mesh, and encode its connectivity in the mesh.
      updateMeshDataStructures(vertex.getLmkId(), vertex.getVertexPosition(),
                               &vertex_to_lmk_id_map_, &lmk_id_to_vertex_map_, &vertices_mesh_,
                               &vertices_mesh_normal_, &polygons_mesh_);
    }
  }
}

/* -------------------------------------------------------------------------- */
// 0x18019A300 (Mesh2D; the Mesh3D copy is inlined into 0x180199A00)
// upstream-modified: the CHECK_NOTNULLs and DCHECK(!normals_computed_) became one guard.
template <typename VertexPositionType>
void Mesh<VertexPositionType>::updateMeshDataStructures(
    const LandmarkId& lmk_id, const VertexPositionType& lmk_position,
    std::map<VertexId, LandmarkId>* vertex_to_lmk_id_map,
    std::map<LandmarkId, VertexId>* lmk_id_to_vertex_map, cv::Mat* vertices_mesh,
    cv::Mat_<VertexNormal>* vertices_mesh_normal, cv::Mat* polygon_mesh) const {
  if (vertex_to_lmk_id_map != nullptr && lmk_id_to_vertex_map != nullptr &&
      vertices_mesh != nullptr && vertices_mesh_normal != nullptr && polygon_mesh != nullptr &&
      !normals_computed_) {
    const auto& lmk_id_to_vertex_map_end = lmk_id_to_vertex_map->end();
    const auto& vertex_it = lmk_id_to_vertex_map->find(lmk_id);

    int row_id_vertex;
    // Check whether this landmark is already in the set of vertices of the mesh.
    if (vertex_it == lmk_id_to_vertex_map_end) {
      // New landmark, create a new entrance in the set of vertices.
      vertices_mesh->push_back(lmk_position);
      vertices_mesh_normal->push_back(VertexNormal());
      row_id_vertex = vertices_mesh->rows - 1;
      // Book-keeping.
      (*lmk_id_to_vertex_map)[lmk_id] = row_id_vertex;
      (*vertex_to_lmk_id_map)[row_id_vertex] = lmk_id;
    } else {
      // Update old landmark with new position.
      vertices_mesh->at<VertexPositionType>(vertex_it->second) = lmk_position;
      row_id_vertex = vertex_it->second;
    }
    // Store corresponding ids (row index) to the 3d point in map_points_3d.
    polygon_mesh->push_back(row_id_vertex);
  }
}

/* -------------------------------------------------------------------------- */
// 0x180199F40 (Mesh3D)
// upstream-modified: CHECK_NOTNULL/VLOG removed; the two DCHECKs became `return false`.
template <typename VertexPositionType>
bool Mesh<VertexPositionType>::getPolygon(const size_t& polygon_idx, Polygon* polygon) const {
  if (polygon_idx >= getNumberOfPolygons()) {
    return false;
  }

  size_t idx_in_polygon_mesh = polygon_idx * (polygon_dimension_ + 1);
  polygon->resize(polygon_dimension_);
  for (size_t j = 0; j < polygon_dimension_; j++) {
    const int32_t& row_id_pt_j = polygons_mesh_.at<int32_t>(
        static_cast<int>(idx_in_polygon_mesh + j + 1));
    if (vertex_to_lmk_id_map_.find(row_id_pt_j) == vertex_to_lmk_id_map_.end()) {
      return false;
    }
    if (row_id_pt_j >= vertices_mesh_.rows) {
      return false;
    }
    // MSVC evaluates the arguments right to left: normal, position, lmk id (map::at).
    polygon->at(j) = VertexType(vertex_to_lmk_id_map_.at(row_id_pt_j),
                                vertices_mesh_.at<VertexPositionType>(row_id_pt_j),
                                vertices_mesh_normal_.template at<VertexNormal>(row_id_pt_j));
  }
  return true;
}

/* -------------------------------------------------------------------------- */
// 0x180199D30
// upstream-modified: colour Mat gone; the two maps are cleared in the opposite order.
template <typename VertexPositionType>
void Mesh<VertexPositionType>::clearMesh() {
  vertices_mesh_ = cv::Mat(0, 1, CV_32FC3);
  vertices_mesh_normal_ = cv::Mat_<VertexNormal>(0, 1);
  polygons_mesh_ = cv::Mat(0, 1, CV_32SC1);
  lmk_id_to_vertex_map_.clear();
  vertex_to_lmk_id_map_.clear();
}

// explicit instantiations
template class Mesh<Vertex2D>;
template class Mesh<Vertex3D>;

}  // namespace totem
}  // namespace pimax
