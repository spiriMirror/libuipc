#pragma once
#include <span>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
SimplicialComplex tetmesh(std::span<const Vector3> Vs, std::span<const Vector4i> Ts);
}  // namespace uipc::geometry
