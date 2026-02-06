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
[[nodiscard]] UIPC_GEOMETRY_API SimplicialComplex merge(span<const SimplicialComplex*> complexes);

[[nodiscard]] UIPC_GEOMETRY_API SimplicialComplex merge(span<const SimplicialComplex> complexes);

[[nodiscard]] UIPC_GEOMETRY_API SimplicialComplex merge(
    std::initializer_list<const SimplicialComplex*>&& complexes);
}  // namespace uipc::geometry
