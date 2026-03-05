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
 * @brief Find all vertex–triangle pairs between two meshes within max_distance and return as a Geometry.
 *
 * One SimplicialComplex provides vertices (point set), the other provides triangles.
 * Returns a Geometry with one instance per (vertex_index, triangle_index) pair; each instance has
 * attribute `topo` (Vector2i): (vertex_index, triangle_index). No constitution_uid or geo_ids.
 * Compatible with SoftVertexTriangleStitch::create_geometry(..., pair_geometry, ...).
 *
 * @param vertex_mesh  Mesh providing vertices (positions used for point–triangle distance).
 * @param triangle_mesh Mesh providing triangles (triangles().topo() and vertex positions).
 * @param max_distance  Default max distance; only pairs with point–triangle distance <= this (or per-vertex value) are returned.
 * @param max_distance_attr  If non-empty, name of Float attribute on vertices of vertex_mesh; per-vertex max distance (overrides scalar). If empty or absent, scalar is used.
 * @param group  If non-empty, name of IndexT attribute on vertices (vertex_mesh) and on triangles (triangle_mesh); only elements with value 1 are considered (0 = not selected). If empty, no filtering.
 * @return Geometry  Instances have topo Vector2i (vertex_index, triangle_index).
 */
Geometry UIPC_GEOMETRY_API
closest_vertex_triangle_pairs(const SimplicialComplex& vertex_mesh,
                              const SimplicialComplex& triangle_mesh,
                              Float                    max_distance,
                              std::string_view         max_distance_attr = {},
                              std::string_view         group             = {});
}  // namespace uipc::geometry
