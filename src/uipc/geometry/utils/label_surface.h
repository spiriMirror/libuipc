#pragma once
#include <uipc/geometry/simplicial_complex.h>


namespace uipc::geometry
{
UIPC_CORE_API P<AttributeSlot<IndexT>> label_surface_vertices(SimplicialComplex& sc);
}
