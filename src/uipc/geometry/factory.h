#pragma once
#include <span>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Create a simplicial complex from a tetrahedral mesh.
 */
SimplicialComplex tetmesh(std::span<const Vector3> Vs, std::span<const Vector4i> Ts);
/**
 * @brief Create a simplicial complex from a triangular mesh.
 */
SimplicialComplex trimesh(std::span<const Vector3> Vs, std::span<const Vector3i> Fs);
/**
 * @brief Create a simplicial complex from a line mesh.
 */
SimplicialComplex linemesh(std::span<const Vector3> Vs, std::span<const Vector2i> Es);
/**
 * @brief Create a simplicial complex from a point cloud.
 */
SimplicialComplex pointcloud(std::span<const Vector3> Vs);
}  // namespace uipc::geometry
