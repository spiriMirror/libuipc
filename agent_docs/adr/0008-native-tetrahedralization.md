# ADR 0008 — Native tetrahedralization with a protected boundary

- Status: Accepted
- Date: 2026-09-05
- First implementation: current `refactor-main` change

## Context

The owner requires an in-tree C++ tetrahedralizer under
`src/geometry/tetrahedralization/`. Strict preservation means unchanged original
vertices, coordinates and triangle connectivity: not even boundary subdivision
is allowed. Quality-search failure must retain/construct a conservative valid
mesh; returning an approximate surface or a partial mesh is not acceptable.

## Decision

Use a direct-simplex/visibility-kernel path and a general oriented advancing
front with backtracking and nested interior candidate sets. Certify the
conservative mesh before quality optimization. Quality budgets do not cap the
conservative path. Each failed optimization restores the certified baseline.
The alternate mode permits conforming boundary refinement before volume meshing.
No fTetWild/TetGen code or runtime dependency is used. The implementation,
bindings and independent Blender worker/protocol remain Apache-2.0.

## Consequences

- Input must define an embedded closed solid; malformed surfaces are input errors.
- Preserve original vertex IDs as an output prefix and expose source vertex/face
  ID attributes. Respect an input `orient` attribute, including extracted surfaces.
- Local orientation uses a floating-expansion fallback and strict compiler FP
  options in both build systems. Geometry-only rebuilds must refresh Python's DLL.
- Validate oriented face incidence and enclosed volume, not only cell count.
  Output never replaces a concave model by its convex hull or fills its cavities.
- Conservative front search may be slow. Revisit earlier candidate prefixes with
  larger budgets instead of allowing later points to starve an earlier solution.
- Quality is measured, not claimed globally optimal. Interior bisections, 2-to-3
  swaps and smoothing preserve a valid baseline; strict surfaces stay locked.
- Blender preparation is explicit and happens before selecting added interior
  pins. All volume nodes live in the Blender mesh; only actual boundary faces
  render. A private source mesh/group snapshot supports restoration.

## Alternatives considered

- Delegating to fTetWild/TetGen was rejected: the owner requires an original,
  in-tree implementation and an exact, unchanged triangle boundary in strict mode.
- Returning failure or silently switching to an approximate surface after a
  quality budget expires was rejected by the owner.
- A single visibility-kernel fan alone does not handle non-star-shaped solids;
  the general constructive front and interior-point search provide that path.

## Validation

`apps/tests/geometry/tetrahedralization.cpp` checks exact per-face/per-vertex
preservation, positive volume, inner-face pairing, a non-star U, an internal void,
twisted prism, simplex, scale, and ambiguous orientation predicates.
`integrations/blender/tests/blender_fem.py` checks the actual C++ generator through
Blender, original groups, selected surface/internal pins, whole-body fixing for
ABD/FEM/cloth, contact, all-node MDD playback, save/reopen/render and restoration.

User-facing contracts and options are in `docs/specification/tetrahedralization.md`.
