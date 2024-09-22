#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Apply the instance transform to the simplicial complex.
 * 
 * 1) Vertex position of the result simplicial complex will be transformed.
 * 2) The rest meta/instance/vertex/edge/face/tetrahedron attributes will be kept.
 * 
 * @return vector<SimplicialComplex> the transformed simplicial complexes.
 */
UIPC_GEOMETRY_API [[nodiscard]] vector<SimplicialComplex> apply_transform(const SimplicialComplex& complex);
}  // namespace uipc::geometry
