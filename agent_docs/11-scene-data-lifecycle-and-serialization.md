# 11 — Scene Data Lifecycle and Serialization

This guide describes the state model behind the user-facing Scene API. It is the
place to start when a bug involves stale data, current/rest geometry, animation,
pending objects, contact labels, snapshots, or SceneIO commits.

## Ownership and Lifetime

```text
Engine public handle
  -> shared internal::Engine
       -> dynamically loaded backend module and backend IEngine

World public handle
  -> shared internal::World
       -> weak internal::Engine       (Engine must outlive World)
       -> shared internal::Scene

Scene public handle
  -> shared internal::Scene
       -> config, Objects, current/rest GeometryCollections
       -> ConstitutionTabular, ContactTabular, SubsceneTabular
       -> Animator, DiffSim, SceneVisitor state
```

Keeping only a `World` is insufficient: its Engine reference is weak. This is
especially easy to violate in Python when a builder returns `(world, scene)` but
lets the local `engine` variable die.

`World::init` is one-shot. Sanity-check or backend failure permanently invalidates
that World; create a new Engine/World/Scene chain after fixing the input.

## What a Scene Stores

| Component | Storage model | Important consequence |
|---|---|---|
| config | size-one `AttributeCollection` with slash-separated names | `Scene(Json)` validates and copies values; later edits to the input JSON do nothing |
| objects | stable object IDs plus names and owned geometry IDs | deleting an object delegates destruction of its geometry slots |
| geometries | current `GeometryCollection` keyed by geometry ID | backend retrieval writes simulated state here |
| rest geometries | parallel collection with the same logical IDs | created with each object geometry; animation/constitutions use it as reference state |
| constitution table | sorted set of UIDs scanned from geometry metadata at init | adding a brand-new UID after initialization is not equivalent to adding it before init |
| contact table | element IDs plus symmetric pair-to-model lookup | an unregistered pair falls back to model index 0 |
| subscene table | element IDs plus symmetric enable/disable lookup | unspecified pairs use identity behavior: same element enabled, different elements disabled |
| animator | at most one animation record per object ID | captures that object's current/rest slots during backend initialization |
| DiffSim | parameter collection and feature state | non-const `scene.diff_sim()` enables `diff_sim/enable` as a side effect |

## Geometry and Attribute Layers

The principal concrete geometry is `SimplicialComplex`:

```text
SimplicialComplex
  meta()          geometry-wide attributes (usually one row)
  instances()     per-instance transform/fixed/dynamic state
  vertices()      per-vertex attributes, including position
  edges()         per-edge topology and attributes
  triangles()     per-triangle topology and attributes
  tetrahedra()    per-tetrahedron topology and attributes
```

Every `AttributeCollection` is column-oriented: attributes in one collection have
the same row count. `create`, `share`, `destroy`, `find`, `resize`, `reorder`, and
`copy_from` operate at that collection boundary.

A non-const attribute view is not observational. Requesting read/write access
updates the attribute's modification generation so that commit logic can detect
it. An attribute marked `evolving` is included in commits even when ordinary
change detection would omit it. Avoid taking mutable views merely to inspect data.

### Current versus rest geometry

`Object::Geometries::create` clones the supplied geometry into both current and
rest slots with a shared logical ID. The two copies then have different roles:

- FEM positions in the current geometry are degrees of freedom; the rest copy
  supplies the reference configuration.
- ABD vertex positions remain local/rest coordinates. World motion is stored in
  per-instance `transform` (and velocity) attributes.
- Animator callbacks receive both `geo_slots()` and `rest_geo_slots()` so targets
  can be derived without accumulating drift.

After `world.retrieve()`, read current geometry for simulated output. Do not mutate
rest data accidentally when intending to teleport or drive current state.

## Lifecycle and Mutation Timeline

| Phase | What is safe/visible | What is frozen or deferred |
|---|---|---|
| build Scene, before `World::init` | create/destroy objects and geometry immediately; assign materials/contact/subscene labels; edit config | nothing has been uploaded to a backend |
| `World::init` | optional sanity check; Scene marks started; constitution UIDs are scanned; backend builds systems and captures scene state | failure invalidates World; Animator binds object slots; backend capabilities are now fixed |
| between frames | user may edit attributes and enqueue object/geometry changes | changes after Scene start enter pending collections |
| CUDA rebuild at `advance` | rebuild events settle pending creates/destroys and refresh backend data | the mutation is not guaranteed visible before this point |
| `World::sync` | waits for asynchronous backend work | does not itself copy all state into Scene |
| `World::retrieve` | implicitly syncs when needed, then writes device state into current geometry | repeated retrieve calls may sync again because an implicit sync does not currently update the wrapper's sync flag |
| after `retrieve` | inspect/export state; take a stable `SceneSnapshot` | avoid snapshotting while pending geometry exists |

The CUDA backend calls `begin_pending` at initialization and `solve_pending` in
its rebuild phase. The `none` backend does not run that machinery: additions made
after init remain pending there. Treat `none` as an interface/template backend,
not as a faithful CPU execution or mutation backend.

## Animator Contract

`Animator::insert(object, callback, substep)` accepts one animation per object;
`substep` must be positive and defaults to 1. During backend initialization, the
animator records the current/rest slots belonging to the object. The callback sees:

- Scene `dt` and current frame;
- the Object handle;
- current and rest geometry slot arrays;
- a hint object (currently only `fixed_vertices_changing`).

Animation normally drives a constraint target rather than overwriting physics
state:

- FEM: set `is_constrained` and update `aim_position`, used by
  `SoftPositionConstraint`;
- ABD: set `is_constrained` and update `aim_transform`, used by
  `SoftTransformConstraint`.

Writing an ABD `transform` directly is a teleport. If an old `aim_transform`
remains active, the constraint pulls the body back on the next step.

## Contact and Subscene Metadata

`ContactElement::apply_to(geometry)` writes the geometry meta attribute
`contact_element_id`; `SubsceneElement::apply_to(geometry)` writes
`subscene_element_id`. Despite a few Python docstrings saying “object”, the
argument is a Geometry.

### ContactTabular

- Element 0 is the default element.
- The built-in default model is enabled with friction `0.5` and resistance
  `1 GPa`; it is not marked “user set” until `default_model(...)` is called.
- Pair insertion is symmetric. A missing pair resolves to model index 0.
- The optional JSON `config` arguments accepted by `create/insert/default_model`
  are currently ignored; returned model configs are empty objects.
- `build_from` clears and reconstructs model topology, while `update_from` does
  not clear the pair map first and can retain stale keys.

### SubsceneTabular

- Element 0 is the default subscene.
- Pair insertion is symmetric.
- An unspecified pair returns `true` only when both element IDs are equal.
- Its JSON config argument is also currently ignored.
- `update_from` likewise does not clear the existing pair map first.

Subscenes filter whether collision/contact is allowed; they are not nested Scene
objects and do not create separate solvers.

## Snapshots, Full Serialization, and Incremental Commits

These three concepts are related but not interchangeable:

| Mechanism | Purpose | Includes | Important limits |
|---|---|---|---|
| `SceneSnapshot(scene)` | deep in-memory baseline for change detection | config, current/rest geometry, objects, contact data, and an internal copy of subscene data | refuses pending geometry; public getters do not expose the copied subscene data |
| `SceneIO::save/load`, `to_json/from_json` | full Scene persistence | full serializers for Scene components | use for durable files and topology-changing transfers; still add a round-trip test for every schema extension |
| `SceneIO::commit/update`, `commit_to_json/update_from_json` | incremental changes relative to a snapshot | config, objects, current/rest geometry commits, contact elements/models | currently append/update oriented; not a general lossless Scene replication protocol |

Verified incremental-path limitations at `2b954973`:

1. `internal::Scene::update_from` does not apply the commit's subscene elements or
   models.
2. `GeometryCollection::update_from` updates existing geometries and adds new ones,
   but does not remove a geometry missing from the source commit.
3. A newly committed geometry is inserted using the destination collection's next
   ID instead of explicitly restoring the committed map key. Append-only scenes
   usually align; sparse/non-contiguous IDs are unsafe.
4. Attribute commits do represent removed attribute names, but that does not imply
   that removed GeometrySlots propagate.
5. `GeometryAtlas::create(..., evolving_only)` accepts the flag, but the current
   implementation does not use it while collecting geometry or named attributes.
6. `ContactTabular::update_from` and `SubsceneTabular::update_from` can preserve
   stale pair-map entries because they do not clear topology first.
7. `SceneSnapshotCommit` calls the same setup lambda for current and rest commit
   maps, but that lambda always iterates `dst.m_geometries/src.m_geometries`.
   Consequently the rest-geometry commit currently duplicates current-geometry
   diffs instead of comparing `m_rest_geometries`.

The legacy `include/uipc/core/scene_archieve.h` and matching implementation are
fully commented artifacts and are not an alternate supported serializer.

### Choosing the right path

- Use full `save/load` for durable scenes, initial transfer, topology deletion, or
  when IDs may be sparse.
- Use `commit/update` for controlled append/update streams similar to sample
  `15_scene_commit`, after establishing a full baseline and keeping both ends in
  lockstep.
- Take snapshots immediately after a successful `world.init()` or after
  `world.advance(); world.retrieve();`, with no unresolved pending mutations.
- When changing any serializer, test contact and subscene pair tables separately;
  equality of visible mesh positions is not sufficient.

## Source and Example Pointers

- Scene internals: `src/core/core/internal/scene.cpp`
- Object/current-rest creation: `src/core/core/object.cpp` and related internal files
- Attribute commit logic: `src/core/geometry/attribute_collection_commit.cpp`
- Geometry atlas/collection commits: `src/core/geometry/geometry_atlas.cpp`,
  `src/core/geometry/geometry_collection.cpp`
- Snapshot construction: `src/core/core/scene_snapshot.cpp`
- Scene IO: `src/io/scene_io.cpp`
- Contact/subscene tables: `src/core/core/contact_tabular.cpp`,
  `src/core/core/subscene_tabular.cpp`
- Full persistence sample: `libuipc-samples/examples/14_load_scene/`
- Incremental sample: `libuipc-samples/examples/15_scene_commit/`
- Animator sample: `libuipc-samples/examples/3_periodically_pressed_tetrahedron/`
- Subscene sample: `libuipc-samples/examples/29_subscene/`
