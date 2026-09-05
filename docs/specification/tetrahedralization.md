# Native Tetrahedralization

The implementation lives in `src/geometry/tetrahedralization/`, with public
declarations in `include/uipc/geometry/tetrahedralization.h`. It is original
Apache-2.0 code, copyright spiriMirror, and adds no external mesher dependency.

## Surface contract

`preserve_surface=true` keeps all original vertex coordinates/indices and every
triangle's connectivity. No edge/face splitting, boundary vertex relocation,
or triangle replacement is permitted. Additional nodes are strictly internal.
Output original vertices occupy the original index prefix. The vertex attribute
`input_vertex_id` and triangle attribute `input_triangle_id` expose source IDs;
new/internal elements have `-1`. Output winding is outward; the optional source
`builtin::orient` attribute is respected without mutating the source.

`preserve_surface=false` permits conforming midpoint refinement of the boundary
to improve resolution. Its surface triangulation may differ and it may add
surface vertices. The present implementation retains the original geometric
surface and sharp features while relaxing its triangulation; it does not promise
global optimality, a minimum tetrahedron count, or removal of all poor elements.

Input must be a closed, consistently oriented, embedded triangle surface with
finite coordinates, distinct used vertices and nondegenerate triangles. Input
validation rejects open/non-manifold edges, duplicate triangles and improper
surface intersections. These data errors do not define a valid solid to mesh.
Instance transforms are not applied; pass the desired rest positions explicitly.

## Constructive conservative path

1. A tetrahedral input boundary returns its single volume cell directly.
2. If the surface has an interior visibility kernel, cone every boundary triangle
   to a certified interior point. Half-space projection can locate a kernel away
   from the average vertex position; acceptance uses orientation predicates.
3. Otherwise use an oriented advancing front. Each accepted tetrahedron lies on
   the unfilled side, cannot cross the front or consume unresolved boundary
   vertices, and cancels/replaces its four front faces consistently.
4. If the quality-oriented front search does not finish, continue with interior
   Steiner candidates and backtracking. Candidate sets are nested and previous
   finite sets are revisited with increasing search budgets. Growing the candidate
   pool cannot starve a solution on an earlier set. The quality budget never ends
   this construction or authorizes a change to the protected boundary.
5. Certify the conservative mesh before optimization: positive tetrahedral
   orientation, oppositely oriented paired interior faces, exact oriented outer
   boundary, original coordinates, and matching enclosed volume.

The general search can be expensive on complex inputs. It has no promised
interactive completion time; callers requiring cancellation should run it in an
owned subprocess, as the Blender extension does. It never substitutes a convex
hull, fills a cavity, returns a partial volume, or relaxes strict surface mode
because an optimization/search budget was reached.

## Quality improvement

After constructing the conservative mesh, interior-edge bisection targets
resolution, local 2-to-3 swaps improve the minimum quality of a cell patch, and
orientation-preserving interior-node smoothing improves incident cells. All
protected surface vertices and edges remain locked. Failed quality validation
restores the already-certified conservative mesh.

The reported dimensionless mean-ratio quality is

$$
q(T)=\frac{12(3V_T)^{2/3}}{\sum_{e\in T}\|e\|^2},\qquad 0<q\le1,
$$

where $V_T$ is positive tetrahedral volume and the denominator sums its six
squared edge lengths. An equilateral tetrahedron has quality 1. Refinement
balances quality and target resolution, so resolution-driven subdivision can
lower the minimum ratio while producing smaller useful cells.

## Options

| Key | Default | Domain and meaning |
|---|---:|---|
| `preserve_surface` | `true` | Boolean; exact input boundary connectivity or permitted refinement |
| `target_edge_length` | `0` | Finite >=0, in input coordinate units; zero selects the median surface edge |
| `quality_passes` | `4` | Integer 0–100; local swapping/smoothing passes |
| `refinement_budget` | `256` | Integer >=0; accepted interior-edge bisections; does not bound conservative construction |

The report includes construction method, conservative-search work, counts,
conservative/final minimum quality, effective target edge length, accepted
improvements, whether optimization reverted, volume, boundary checks and elapsed
time. Inspect these values instead of treating the requested edge length as a
hard element-size guarantee.

## API

=== "C++"

    ```cpp
    #include <uipc/geometry/tetrahedralization.h>
    #include <uipc/io/simplicial_complex_io.h>

    int main()
    {
        uipc::geometry::SimplicialComplexIO io;
        auto surface = io.read("closed_surface.obj");
        auto options = uipc::geometry::tetrahedralization_default_config();
        options["preserve_surface"] = true;
        auto [volume, report] = uipc::geometry::tetrahedralize(surface, options);
    }
    ```

=== "Python"

    ```python
    from uipc.geometry import (
        SimplicialComplexIO,
        tetrahedralization_default_config,
        tetrahedralize,
    )

    surface = SimplicialComplexIO().read("closed_surface.obj")
    options = tetrahedralization_default_config()
    options["preserve_surface"] = True
    volume, report = tetrahedralize(surface, options)
    print(report)
    ```

The volume already has surface/orientation labels. Apply a volumetric
constitution, set per-node `is_fixed` flags, then add it to a Scene; see the
[FEM tutorial](../tutorial/fem.md) and [Blender guide](../build_install/blender.md).

## Numerical/build contract

Near-zero orientation determinants use floating expansions with FMA product
residuals. CMake and XMake disable reassociation for this module (`/fp:strict` on
MSVC; `-fno-fast-math -ffp-contract=off` elsewhere). This is CPU preprocessing,
not a CUDA kernel or per-frame allocation path. Geometry-only rebuilds refresh
the Python runtime DLL so testing does not silently load an older implementation.
