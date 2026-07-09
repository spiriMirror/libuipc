#pragma once
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/common/dllexport.h>

namespace uipc::constitution
{
// Barycentric embedding of a dense surface mesh onto a coarse tet mesh.
//
// Usage (in IIPCSolver::build_scene):
//   EmbeddedCollisionMesh ecm;
//   ecm.apply_to(tet_sc, surface_sc);
//
// The surface_sc must be passive (no FEM constitution).
// The tet_sc must already have a FEM constitution applied.
//
// At runtime the CUDA backend reads attributes written here:
//   ecm_tet_index  (IndexT  per surface vertex) — containing tet index
//   ecm_bary       (Vector4  per surface vertex) — barycentric coords
//   ecm_driven     (IndexT  meta, value=1)       — marks the mesh as driven
//
// Reference: SOFA BarycentricMapping — apply() / applyJT()
class UIPC_CONSTITUTION_API EmbeddedCollisionMesh
{
  public:
    // Build barycentric embedding CPU-side (point-in-tet query).
    // Returns false if any surface vertex falls outside all tets (clamped).
    bool apply_to(geometry::SimplicialComplex& tet_sc,
                  geometry::SimplicialComplex& surface_sc) const;
};
}
