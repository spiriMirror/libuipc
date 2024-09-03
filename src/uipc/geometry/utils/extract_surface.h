#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Extract the surface of a tetrahedral mesh.
 *  
 * @param src The tetrahedral mesh.
 * @return SimplicialComplex The surface of the tetrahedral mesh.
 */
UIPC_CORE_API [[nodiscard]] SimplicialComplex extract_surface(const SimplicialComplex& src);

/**
 * @brief Extract the surface of a list of tetrahedral meshes and merge them into one.
 *  
 * All the instances of the input tetrahedral meshes will be applied to the output surface.
 * 
 * @param complexes The list of tetrahedral meshes.
 * @return SimplicialComplex The surface of the tetrahedral meshes.
 */
UIPC_CORE_API [[nodiscard]] SimplicialComplex extract_surface(span<const SimplicialComplex*> complexes);

/**
 * @brief Extract the top-dimensional surface of a simplicial complex.
 *
 * A .obj file like format is returned. The format is as follows:
 * - tetmesh is returned as its triangle surface (ignore edges).
 * - trimesh is returned as is (ignore edges).
 * - linemesh is returned as is.
 * - pointcloud is returned as is.
 *
 * @param sc The simplicial complex.
 * @return SimplicialComplex The top-dimensional surface of the simplicial complex.
 */
UIPC_CORE_API [[nodiscard]] SimplicialComplex extract_top_dim_surface(span<const SimplicialComplex*> sc);
}  // namespace uipc::geometry
