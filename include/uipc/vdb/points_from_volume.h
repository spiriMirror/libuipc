#pragma once
#include <uipc/vdb/dllexport.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::vdb
{
UIPC_VDB_API geometry::SimplicialComplex points_from_volume(const geometry::SimplicialComplex& sc,
                                                            Float resolution);
}