# 03 — Geometry Module and IO

## SimplicialComplex (Core Geometry Type)

Header: `include/uipc/geometry/simplicial_complex.h`; implementation: `src/geometry/simplicial_complex*.cpp`.

A simplicial complex with four dimensional slots:
- `vertices()` — dimension 0 (points)
- `edges()` — dimension 1 (line segments)
- `triangles()` — dimension 2 (triangles)
- `tetrahedra()` — dimension 3 (tetrahedra)

Organization: **AttributeCollection attribute sets**. Each slot is an `AttributeCollection` (a per-element-indexed attribute table where each attribute is a column of homogeneous data, with type-safe template access via `find<T>(name)->view()`); the geometry itself also has `meta()` (global meta-information attributes, e.g. `constitution_uid`) and `instances()` (per-instance attributes, e.g. `is_fixed`, `transform`).

Common base classes of the attribute system: `IAttribute` / `IAttributeSlot` (`include/uipc/geometry/attribute*.h`), supporting lazy rebuilding, resize, and views (`view()` returns span-style read-only/read-write views). See `include/uipc/geometry/geometry.h` for the Geometry abstract base class; the handles in a scene are `GeometrySlot` / `SimplicialComplexSlot` (including the rest-geometry dual: `rest_geo_slots()`).

## Geometry Factory Functions (`include/uipc/geometry/factory.h`, etc.)

- `tetmesh(Vs, Ts)`, `trimesh(Vs, Fs)`, `linemesh(Vs, Es)`, `pointcloud(Vs)` construct from arrays.
- `merge(span<SimplicialComplex*>)` merges multiple meshes.
- Implicit geometry: `ground(height)` (HalfPlane), etc.; see `docs/specification/implicit_geometry_uid.md`.

## Topology and Geometry Algorithms (`src/geometry/`, depends on libigl + TBB + octree)

Common functions (headers with the same names under `include/uipc/geometry/`):
- `label_surface(sc)` — marks surface vertices/edges/triangles (`builtin::is_surface`); **required to enable contact**
- `extract_surface(sc)` — extracts the surface trimesh
- `label_triangle_orient(sc)` — marks triangle orientation (`builtin::parent_id`, etc.); required for frictional contact
- `label_open_edge` / `flip_inward_triangles`
- `compute_vertex_volume` / `compute_vertex_mass`
- `affine_body/compute_affine_body_*` — ABD mass properties (dyadic mass triples, etc.)
- BVH: `include/uipc/geometry/bvh.h` (CPU-side bounding volume hierarchy)
- Distance/intersection queries, closest point, octree spatial index
- `implicit_geometry/`: HalfPlane distance field, etc.

## Built-in Attribute Names (`include/uipc/builtin/`)

The `builtin::` namespace centrally defines conventional attribute names, avoiding scattered string literals:
- `builtin::is_fixed` (instance, whether fixed)
- `builtin::is_surface` (vertex/edge/triangle, whether on the surface)
- `builtin::parent_id`, `builtin::orient` (source tetrahedron and orientation of a surface triangle)
- `builtin::is_constrained`, `aim_position`, `aim_transform` (animation constraints)
- `builtin::constitution_uid` (meta), `builtin::thickness` (codim vertex thickness), etc.
- Full list: `include/uipc/builtin/attribute_name.h`, `uid.h`, `constitution_uid.h`.

## IO (`include/uipc/io/` + `src/io/`)

| Class | Capabilities |
|---|---|
| `SimplicialComplexIO` | Unified extension-based `read/write`: reads `.msh` (tet)/`.obj`/`.ply`/`.stl` (tri/line/point), writes `.obj`/`.msh`; the constructor accepts a `pre_transform` (Matrix4x4/Transform) to pre-transform the vertices being read |
| `SceneIO` | `write_surface(path, scene)` exports the scene surface as `.obj`; `simplicial_surface(dim)` takes the surface of any dimension; scene serialization `load/save` (`.json`/`.bson`); incremental `commit/update`; `to_json/from_json` |
| `SpreadSheetIO` | Exports all attributes of a Geometry as JSON/CSV (one table per attribute), for debugging |
| `UrdfIO` / `UrdfController` | Reads a URDF robot as an Object; the controller provides `move_root`, `rotate_to(joint, angle)`, and exposes revolute joints (ImplicitGeometrySlot) and links (SimplicialComplexSlot), `sync_visual_mesh` |
| `AttributeIO` | Reads a single named attribute from a file into an `IAttributeSlot` |
| glTF | `test_gltf` experimental interface (tinygltf) |

STL parsing is in `src/io/stl_reader.h`.

## Common Pitfalls

- You **must** call `label_surface` before contact simulation; friction requires `label_triangle_orient` to guarantee consistent normals.
- Instances with `is_fixed=1` do not participate in dynamics but still participate in contact.
- What `apply_to` writes is an **attribute**; modifying the attribute modifies the material parameters. For multi-instance geometry, pay attention to whether the attribute lives in the instance slot or the vertex slot.
- When reading a mesh, prefer `pre_transform` for uniform scaling rather than modifying vertices afterwards (the rest geometry would become inconsistent).
