#pragma once
#include "dll_export.h"
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::vdb
{
UIPC_VDB_API geometry::SimplicialComplex mesh_to_point_cloud(const geometry::SimplicialComplex& sc,
                                                             Float resolution);
}