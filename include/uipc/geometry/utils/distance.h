#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/dllexport.h>

namespace uipc::geometry
{
/**
 * @brief Compute the distance between a half-plane (P, N) and a vertex V (with thickness V_thickness).
 * 
 * @param P The Origin point of the half-plane.
 * @param N The Normal vector of the half-plane.
 * @param V The Vertex point.
 * @param V_thickness The thickness of the vertex.
 */
Float UIPC_GEOMETRY_API halfplane_vertex_signed_distance(const Vector3& P,
                                                         const Vector3& N,
                                                         const Vector3& V,
                                                         Float V_thickness = 0.0);

Float UIPC_GEOMETRY_API point_point_squared_distance(const Vector3& P0, const Vector3& P1);

Float UIPC_GEOMETRY_API point_edge_squared_distance(const Vector3& P,
                                                    const Vector3& E0,
                                                    const Vector3& E1);

Float UIPC_GEOMETRY_API point_triangle_squared_distance(const Vector3& P,
                                                        const Vector3& T0,
                                                        const Vector3& T1,
                                                        const Vector3& T2);

Float UIPC_GEOMETRY_API edge_edge_squared_distance(const Vector3& Ea0,
                                                   const Vector3& Ea1,
                                                   const Vector3& Eb0,
                                                   const Vector3& Eb1);
}  // namespace uipc::geometry
