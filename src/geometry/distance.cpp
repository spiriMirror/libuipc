#include <uipc/geometry/utils/distance.h>

namespace uipc::geometry
{
Float halfplane_vertex_signed_distance(const Vector3& P, const Vector3& N, const Vector3& V, Float V_thickness)
{
    return (V - P).dot(N) - V_thickness;
}
}  // namespace uipc::geometry
