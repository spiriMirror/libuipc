#pragma once
#include <span>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Create a simplicial complex from a tetrahedral mesh.
 */
[[nodiscard]] SimplicialComplex tetmesh(std::span<const Vector3>  Vs,
                                        std::span<const Vector4i> Ts);
/**
 * @brief Create a simplicial complex from a triangular mesh.
 */
[[nodiscard]] SimplicialComplex trimesh(std::span<const Vector3>  Vs,
                                        std::span<const Vector3i> Fs);
/**
 * @brief Create a simplicial complex from a line mesh.
 */
[[nodiscard]] SimplicialComplex linemesh(std::span<const Vector3>  Vs,
                                         std::span<const Vector2i> Es);
/**
 * @brief Create a simplicial complex from a point cloud.
 */
[[nodiscard]] SimplicialComplex pointcloud(std::span<const Vector3> Vs);
}  // namespace uipc::geometry
