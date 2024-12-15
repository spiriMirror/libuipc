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
UIPC_GEOMETRY_API [[nodiscard]] SimplicialComplex extract_surface(const SimplicialComplex& src);

/**
 * @brief Extract the surface of a list of tetrahedral meshes and merge them into one.
 *  
 * All the instances of the input tetrahedral meshes will be applied to the output surface.
 * 
 * @param complexes The list of tetrahedral meshes.
 * @return SimplicialComplex The surface of the tetrahedral meshes.
 */
UIPC_GEOMETRY_API [[nodiscard]] SimplicialComplex extract_surface(span<const SimplicialComplex*> complexes);
}  // namespace uipc::geometry
