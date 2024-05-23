#pragma once
#include <span>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
SimplicialComplex tetmesh(std::span<const Vector3> Vs, std::span<const Vector4i> Ts);
SimplicialComplex trimesh(std::span<const Vector3> Vs, std::span<const Vector3i> Fs);
}  // namespace uipc::geometry
