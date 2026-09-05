// Copyright (C) 2026 spiriMirror
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <uipc/geometry/simplicial_complex.h>
#include <utility>

namespace uipc::geometry
{
/**
 * @brief Default options for the native, boundary-conforming volume mesher.
 *
 * preserve_surface=true retains input vertex indices, coordinates and boundary
 * triangles. Only interior Steiner vertices may be introduced in this mode.
 * Quality/refinement budgets limit optimization, never boundary construction.
 */
UIPC_GEOMETRY_API Json tetrahedralization_default_config();

/**
 * @brief Construct a positive-volume tetrahedral mesh of a closed triangle surface.
 *
 * The input must be an embedded, consistently oriented, closed two-manifold.
 * Returns the volume mesh and a quality/construction report. A valid conservative
 * mesh is established before optimization; rejected optimization steps retain it.
 * Original vertices occupy the first input.vertices().size() output entries.
 * Input geometry is never modified. Coordinates are interpreted directly, without
 * applying instance transforms. Apply transforms to the input before calling.
 */
UIPC_GEOMETRY_API std::pair<SimplicialComplex, Json> tetrahedralize(
    const SimplicialComplex& surface,
    const Json&              config = tetrahedralization_default_config());
}  // namespace uipc::geometry
