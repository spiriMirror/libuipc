#pragma once
#include <uipc/common/span.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/geometry/implicit_geometry.h>

namespace uipc::geometry
{
/**
 * @brief Create a simplicial complex from a tetrahedral mesh.
 * 
 * @param Vs The vertex positions of the tetrahedral mesh
 * @param Ts The tetrahedra of the tetrahedral mesh
 */
[[nodiscard]] UIPC_GEOMETRY_API SimplicialComplex tetmesh(span<const Vector3> Vs,
                                                          span<const Vector4i> Ts);

/**
 * @brief Create a simplicial complex from a triangle mesh.
 * 
 * @param Vs The vertex positions of the triangle mesh
 * @param Fs The triangles of the triangle mesh
 * @return SimplicialComplex 
 */
[[nodiscard]] UIPC_GEOMETRY_API SimplicialComplex trimesh(span<const Vector3> Vs,
                                                          span<const Vector3i> Fs);

/**
 * @brief Create a 2D simplicial complex from a quad mesh (4-sided polygons).
 * 
 * @param Vs The vertex positions of the quad mesh
 * @param Fs The quads of the quad mesh
 * @return SimplicialComplex 
 */
[[nodiscard]] UIPC_GEOMETRY_API SimplicialComplex trimesh(span<const Vector3> Vs,
                                                          span<const Vector4i> Fs);

/**
 * @brief Create a simplicial complex from a line mesh.
 * 
 * @param Vs The vertex positions of the line mesh
 * @param Es The edges of the line mesh
 * @return SimplicialComplex 
 */
[[nodiscard]] UIPC_GEOMETRY_API SimplicialComplex linemesh(span<const Vector3> Vs,
                                                           span<const Vector2i> Es);

/**
 * @brief Create a simplicial complex from a point cloud.
 * 
 * @param Vs The vertex positions of the point cloud
 * @return SimplicialComplex 
 */
[[nodiscard]] UIPC_GEOMETRY_API SimplicialComplex pointcloud(span<const Vector3> Vs);


/**
 * @brief Create a half-plane.
 * 
 * @param height The height of the ground plane
 */
[[nodiscard]] UIPC_GEOMETRY_API ImplicitGeometry halfplane(
    const Vector3& P = Vector3::Zero(), const Vector3& N = Vector3::UnitY());

/**
 * @brief Create a gound plane.
 * 
 * @param height The height of the ground plane
 */
[[nodiscard]] UIPC_GEOMETRY_API ImplicitGeometry ground(Float height = 0.0,
                                                        const Vector3& N = Vector3::UnitY());
}  // namespace uipc::geometry
