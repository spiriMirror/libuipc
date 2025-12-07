#include <pyuipc/geometry/utils.h>
#include <pyuipc/as_numpy.h>
#include <pyuipc/common/json.h>
#include <Eigen/Geometry>
#include <uipc/geometry/utils.h>

namespace pyuipc::geometry
{
using namespace uipc::geometry;

static vector<const SimplicialComplex*> vector_of_sc(py::list list_of_sc)
{
    vector<const SimplicialComplex*> simplicial_complexes;

    for(auto sc : list_of_sc)
    {
        auto& simplicial_complex = sc.cast<const SimplicialComplex&>();
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

PyUtils::PyUtils(py::module& m)
{
    m.def("label_surface", &label_surface, py::arg("sc"),
          R"(Label surface elements in a simplicial complex.
Args:
    sc: SimplicialComplex to label.)");

    m.def("label_triangle_orient", &label_triangle_orient, py::arg("sc"),
          R"(Label triangle orientations in a simplicial complex.
Args:
    sc: SimplicialComplex to label.)");

    m.def("flip_inward_triangles", &flip_inward_triangles, py::arg("sc"),
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

    m.def("facet_closure", &facet_closure, py::arg("sc"),
          R"(Compute the facet closure of a simplicial complex.
Args:
    sc: SimplicialComplex to compute closure for.
Returns:
    SimplicialComplex: Facet closure.)");


    m.def("label_connected_vertices", &label_connected_vertices, py::arg("sc"),
          R"(Label connected components of vertices.
Args:
    sc: SimplicialComplex to label.)");

    m.def("label_region", &label_region, py::arg("sc"),
          R"(Label regions in a simplicial complex.
Args:
    sc: SimplicialComplex to label.)");

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
        [](py::array_t<const Float> S, py::array_t<const Float> D)
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

    m.def("is_trimesh_closed", &is_trimesh_closed, py::arg("sc"),
          R"(Check if a triangular mesh is closed (watertight).
Args:
    sc: SimplicialComplex to check.
Returns:
    bool: True if closed, False otherwise.)");

    m.def("constitution_type", &constitution_type, py::arg("geo"),
          R"(Get the constitution type for a geometry.
Args:
    geo: Geometry to check.
Returns:
    str: Constitution type name.)");

    m.def("compute_mesh_d_hat", &compute_mesh_d_hat, py::arg("sc"), py::arg("max_d_hat"),
          R"(Compute mesh d_hat (characteristic length) parameter.
Args:
    sc: SimplicialComplex to compute for.
    max_d_hat: Maximum d_hat value.
Returns:
    float: Computed d_hat value.)");

    m.def("points_from_volume", &points_from_volume, py::arg("sc"), py::arg("resolution") = 0.01,
          R"(Generate points from a volume mesh.
Args:
    sc: SimplicialComplex (volume mesh).
    resolution: Point sampling resolution (default: 0.01).
Returns:
    SimplicialComplex: Point cloud.)");
}
}  // namespace pyuipc::geometry
