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
Float UIPC_CORE_API halfplane_vertex_signed_distance(const Vector3& P,
                                                     const Vector3& N,
                                                     const Vector3& V,
                                                     Float V_thickness = 0.0);
}  // namespace uipc::geometry
