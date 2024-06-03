#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Compute the closure of a simplicial complex.
 * 
 * This operation takes the `position` attribute of the vertices and the top dimension topology of the simplicial complex as input,
 * returns a new simplicial complex with the same vertices and the closure of the input simplicial complex (with the lower dimension simplices).
 * 
 * @return SimplicialComplex The closure of the input simplicial complex.
 */
SimplicialComplex closure(const SimplicialComplex& complex);
}
