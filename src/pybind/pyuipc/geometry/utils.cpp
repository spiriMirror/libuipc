#include <pyuipc/geometry/utils.h>
#include <uipc/geometry/utils.h>
#include <Eigen/Geometry>

namespace pyuipc::geometry
{
using namespace uipc::geometry;

static vector<const SimplicialComplex*> vector_of_sc(py::list list_of_sc)
{
    vector<const SimplicialComplex*> simplicial_complexes;

    for(auto sc : list_of_sc)
    {
        auto& simplicial_complex = py::cast<const SimplicialComplex&>(sc);
        simplicial_complexes.push_back(&simplicial_complex);
    }

    return simplicial_complexes;
}

static py::list list_of_sc(const vector<SimplicialComplex>& simplicial_complexes)
{
    py::list list;
    for(auto& sc : simplicial_complexes)
    {
        list.append(sc);
    }
    return list;
}

PyUtils::PyUtils(py::module_& m)
{
    m.def("label_surface",
          &label_surface,
          py::arg("sc"),
          R"(Label surface elements in a simplicial complex.
Args:
    sc: SimplicialComplex to label.)");

    m.def("label_triangle_orient",
          &label_triangle_orient,
          py::arg("sc"),
          R"(Label triangle orientations in a simplicial complex.
Args:
    sc: SimplicialComplex to label.)");

    m.def("flip_inward_triangles",
          &flip_inward_triangles,
          py::arg("sc"),
          R"(Flip inward-facing triangles to face outward.
Args:
    sc: SimplicialComplex to modify.)");

    m.def(
        "extract_surface",
        [](const SimplicialComplex& simplicial_complex)
        { return extract_surface(simplicial_complex); },
        py::arg("sc"),
        R"(Extract surface from a simplicial complex.
Args:
    sc: SimplicialComplex to extract surface from.
Returns:
    SimplicialComplex: Surface mesh.)");

    m.def(
        "extract_surface",
        [&](py::list list_of_sc)
        {
            auto simplicial_complexes = vector_of_sc(list_of_sc);
            return extract_surface(simplicial_complexes);
        },
        py::arg("sc"),
        R"(Extract surface from multiple simplicial complexes.
Args:
    sc: List of SimplicialComplex objects.
Returns:
    SimplicialComplex: Combined surface mesh.)");

    m.def(
        "merge",
        [&](py::list list_of_sc)
        {
            auto simplicial_complexes = vector_of_sc(list_of_sc);
            return merge(simplicial_complexes);
        },
        py::arg("sc_list"),
        R"(Merge multiple simplicial complexes into one.
Args:
    sc_list: List of SimplicialComplex objects to merge.
Returns:
    SimplicialComplex: Merged simplicial complex.)");

    m.def(
        "apply_transform",
        [](const SimplicialComplex& simplicial_complex) -> py::list
        {
            auto scs  = apply_transform(simplicial_complex);
            auto list = list_of_sc(scs);
            return list;
        },
        py::arg("sc"),
        R"(Apply transforms to a simplicial complex, splitting into multiple complexes.
Args:
    sc: SimplicialComplex with transforms.
Returns:
    list: List of transformed SimplicialComplex objects.)");

    m.def("facet_closure",
          &facet_closure,
          py::arg("sc"),
          R"(Compute the facet closure of a simplicial complex.
Args:
    sc: SimplicialComplex to compute closure for.
Returns:
    SimplicialComplex: Facet closure.)");


    m.def("label_connected_vertices",
          &label_connected_vertices,
          py::arg("sc"),
          R"(Label connected components of vertices.
Args:
    sc: SimplicialComplex to label.)");

    m.def("label_region",
          &label_region,
          py::arg("sc"),
          R"(Label regions in a simplicial complex.
Args:
    sc: SimplicialComplex to label.)");

    m.def("label_open_edge",
          &label_open_edge,
          py::arg("sc"),
          R"(Label open edges in a trimesh.
An edge is considered open if it is shared by exactly 1 triangle.
An edge shared by 2 triangles is a normal closed edge.
Args:
    sc: SimplicialComplex to label.
Returns:
    AttributeSlot: The `is_open` attribute slot on edges (1 = open, 0 = closed).)");

    m.def(
        "apply_region",
        [](const SimplicialComplex& simplicial_complex) -> py::list
        {
            auto scs  = apply_region(simplicial_complex);
            auto list = list_of_sc(scs);
            return list;
        },
        py::arg("sc"),
        R"(Split a simplicial complex by regions.
Args:
    sc: SimplicialComplex with region labels.
Returns:
    list: List of SimplicialComplex objects, one per region.)");

    m.def(
        "optimal_transform",
        [](numpy_array<const Float> S, numpy_array<const Float> D)
        {
            auto S_ = as_span_of<const Vector3>(S);
            auto D_ = as_span_of<const Vector3>(D);
            return as_numpy(optimal_transform(S_, D_));
        },
        py::arg("src"),
        py::arg("dst"),
        R"(Compute optimal transform between two point sets.
Args:
    src: Source points (Nx3 array).
    dst: Destination points (Nx3 array).
Returns:
    numpy.ndarray: 4x4 transformation matrix.)");

    m.def(
        "optimal_transform",
        [](const SimplicialComplex& S, const SimplicialComplex& D)
        { return as_numpy(optimal_transform(S, D)); },
        py::arg("src"),
        py::arg("dst"),
        R"(Compute optimal transform between two simplicial complexes.
Args:
    src: Source SimplicialComplex.
    dst: Destination SimplicialComplex.
Returns:
    numpy.ndarray: 4x4 transformation matrix.)");

    m.def("is_trimesh_closed",
          &is_trimesh_closed,
          py::arg("sc"),
          R"(Check if a triangular mesh is closed (watertight).
Args:
    sc: SimplicialComplex to check.
Returns:
    bool: True if closed, False otherwise.)");

    m.def("constitution_type",
          &constitution_type,
          py::arg("geo"),
          R"(Get the constitution type for a geometry.
Args:
    geo: Geometry to check.
Returns:
    str: Constitution type name.)");

    m.def("compute_mesh_d_hat",
          &compute_mesh_d_hat,
          py::arg("sc"),
          py::arg("max_d_hat"),
          R"(Compute mesh d_hat (characteristic length) parameter.
Args:
    sc: SimplicialComplex to compute for.
    max_d_hat: Maximum d_hat value.
Returns:
    float: Computed d_hat value.)");

    m.def("points_from_volume",
          &points_from_volume,
          py::arg("sc"),
          py::arg("resolution") = 0.01,
          R"(Generate points from a volume mesh.
Args:
    sc: SimplicialComplex (volume mesh).
    resolution: Point sampling resolution (default: 0.01).
Returns:
    SimplicialComplex: Point cloud.)");

    m.def("mesh_partition",
          &mesh_partition,
          py::arg("sc"),
          py::arg("part_max_size") = 16,
          R"(Partition the simplicial complex using METIS graph partitioning.

Creates a 'mesh_part' vertex attribute on the simplicial complex.
This must be called before world.init() to enable the MAS preconditioner.

Args:
    sc: SimplicialComplex to partition.
    part_max_size: Maximum number of vertices per partition (default: 16).
)");

    m.def("closest_vertex_triangle_pairs",
          &closest_vertex_triangle_pairs,
          py::arg("vertex_mesh"),
          py::arg("triangle_mesh"),
          py::arg("max_distance"),
          py::arg("max_distance_attr") = "",
          py::arg("group")             = "",
          R"(Find vertex–triangle pairs between two meshes within max_distance.

Args:
    vertex_mesh: SimplicialComplex providing vertices (point set).
    triangle_mesh: SimplicialComplex providing triangles.
    max_distance: Max point–triangle distance; only pairs within this are returned.
    max_distance_attr: Optional vertex Float attribute name for per-vertex max distance.
    group: Optional IndexT attribute name on vertices/triangles; only value 1 is selected.

Returns:
    Geometry: One instance per pair; each instance has topo (Vector2i) = (vertex_index, triangle_index).
    Compatible with SoftVertexTriangleStitch.create_geometry(..., pair_geometry, ...).)");

    m.def("closest_vertex_edge_pairs",
          &closest_vertex_edge_pairs,
          py::arg("vertex_mesh"),
          py::arg("edge_mesh"),
          py::arg("max_distance"),
          R"(Find vertex-edge pairs between two meshes within max_distance.

Args:
    vertex_mesh: SimplicialComplex providing vertices (point set).
    edge_mesh: SimplicialComplex providing edges.
    max_distance: Max point-edge distance; only pairs within this are returned.

Returns:
    Geometry: One instance per pair; each instance has topo (Vector2i) = (vertex_index, edge_index).
    Compatible with SoftVertexEdgeStitch.create_geometry(..., pair_geometry, ...).)");
}
}  // namespace pyuipc::geometry
