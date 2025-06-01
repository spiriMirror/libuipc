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
 * @return true if the triangle and the edge intersect
 */
UIPC_GEOMETRY_API bool tri_edge_intersect(const Vector3& T0,
                                          const Vector3& T1,
                                          const Vector3& T2,
                                          const Vector3& E0,
                                          const Vector3& E1);

/**
 * @brief Check if a point is in a tetrahedron.
 * 
 * T0, T1, T2, T3 the vertices of the tetrahedron
 * P is the point
 * 
 * @return true if the point is in the tetrahedron
 */
UIPC_GEOMETRY_API bool is_point_in_tet(const Vector3& T0,
                                       const Vector3& T1,
                                       const Vector3& T2,
                                       const Vector3& T3,
                                       const Vector3& P);
}  // namespace uipc::geometry