#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Merge a list of simplicial complexes into one simplicial complex.
 * 
 * All input simplicial complexes must have only one instance. 
 * 
 * @return SimplicialComplex the merged simplicial complex.
 */
UIPC_GEOMETRY_API [[nodiscard]] SimplicialComplex merge(span<const SimplicialComplex*> complexes);

UIPC_GEOMETRY_API [[nodiscard]] SimplicialComplex merge(
    std::initializer_list<const SimplicialComplex*>&& complexes);
}  // namespace uipc::geometry
