#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/dllexport.h>
#include <uipc/common/span.h>
#include <string_view>

namespace uipc::geometry
{
class SimplicialComplex;
class Geometry;

/**
 * @brief Find all vertex-edge pairs between two meshes within max_distance and return as a Geometry.
 *
 * One SimplicialComplex provides vertices (point set), the other provides edges.
 * Returns a Geometry with one instance per (vertex_index, edge_index) pair; each instance has
 * attribute `topo` (Vector2i): (vertex_index, edge_index). No constitution_uid or geo_ids.
 * Compatible with SoftVertexEdgeStitch::create_geometry(..., pair_geometry, ...).
 *
 * @param vertex_mesh  Mesh providing vertices (positions used for point-edge distance).
 * @param edge_mesh    Mesh providing edges (edges().topo() and vertex positions).
 * @param max_distance Only pairs with point-edge distance <= this are returned.
 * @return Geometry    Instances have topo Vector2i (vertex_index, edge_index).
 */
Geometry UIPC_GEOMETRY_API closest_vertex_edge_pairs(const SimplicialComplex& vertex_mesh,
                                                     const SimplicialComplex& edge_mesh,
                                                     Float max_distance);
}  // namespace uipc::geometry
