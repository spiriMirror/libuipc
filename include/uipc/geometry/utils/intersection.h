#pragma once
#include <uipc/common/dllexport.h>
#include <uipc/common/type_define.h>

namespace uipc::geometry
{
/**
 * @brief Check if a triangle and an edge intersect.
 * 
 * T0, T1, T2 the vertices of the triangle
 * E0, E1 the vertices of the edge
 * 
 * 
 * @param[out] uvw_in_tri the barycentric coordinates of the intersection point in the triangle.
 * Even if the function return false, the barycentric coordinates are still calculated correctly.
 * 
 * @param[out] uv_in_edge the barycentric coordinates of the intersection point in the edge.
 * Even if the function return false, the barycentric coordinates are still calculated correctly.
 * 
 * @return true if the triangle and the edge intersect
 */
UIPC_GEOMETRY_API bool tri_edge_intersect(const Vector3& T0,
                                          const Vector3& T1,
                                          const Vector3& T2,
                                          const Vector3& E0,
                                          const Vector3& E1,
                                          bool&          coplanar,
                                          Vector3&       uvw_in_tri,
                                          Vector2&       uv_in_edge);

/**
 * @brief Check if a point is in a tetrahedron.
 * 
 * T0, T1, T2, T3 the vertices of the tetrahedron
 * P is the point
 * 
 * 
 * @param[out] tuvw_in_tet the barycentric coordinates of the intersection point in the tet.
 * Even if the function return false, the barycentric coordinates are still calculated correctly.
 * 
 * @return true if the point is in the tetrahedron
 */
UIPC_GEOMETRY_API bool is_point_in_tet(const Vector3& T0,
                                       const Vector3& T1,
                                       const Vector3& T2,
                                       const Vector3& T3,
                                       const Vector3& P,
                                       Vector4& tuvw_in_tet  // the barycentric coordinates of the intersection point in the tet
);
}  // namespace uipc::geometry