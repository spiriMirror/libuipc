#pragma once
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::geometry
{
/**
 * @brief Compute the volume of an instance in the simplicial complex. Attribute `volume` <Float>
 * will be created in the instance vertices.
 * 
 * Only tetmesh and closed trimesh are supported.
 * 
 * @param R The simplicial complex.
 * @return The attribute slot of the instance volume.
 */
UIPC_GEOMETRY_API S<AttributeSlot<Float>> compute_instance_volume(SimplicialComplex& R);
}  // namespace uipc::geometry
