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
    m.def("label_surface", &label_surface, py::arg("sc"));

    m.def("label_triangle_orient", &label_triangle_orient, py::arg("sc"));

    m.def("flip_inward_triangles", &flip_inward_triangles, py::arg("sc"));

    m.def(
        "extract_surface",
        [](const SimplicialComplex& simplicial_complex)
        { return extract_surface(simplicial_complex); },
        py::arg("sc"));

    m.def(
        "extract_surface",
        [&](py::list list_of_sc)
        {
            auto simplicial_complexes = vector_of_sc(list_of_sc);
            return extract_surface(simplicial_complexes);
        },
        py::arg("sc"));

    m.def(
        "merge",
        [&](py::list list_of_sc)
        {
            auto simplicial_complexes = vector_of_sc(list_of_sc);
            return merge(simplicial_complexes);
        },
        py::arg("sc_list"));

    m.def(
        "apply_transform",
        [](const SimplicialComplex& simplicial_complex) -> py::list
        {
            auto scs  = apply_transform(simplicial_complex);
            auto list = list_of_sc(scs);
            return list;
        },
        py::arg("sc"));

    m.def("facet_closure", &facet_closure, py::arg("sc"));


    m.def("label_connected_vertices", &label_connected_vertices, py::arg("sc"));

    m.def("label_region", &label_region, py::arg("sc"));

    m.def(
        "apply_region",
        [](const SimplicialComplex& simplicial_complex) -> py::list
        {
            auto scs  = apply_region(simplicial_complex);
            auto list = list_of_sc(scs);
            return list;
        },
        py::arg("sc"));

    m.def(
        "tetrahedralize",
        [](const SimplicialComplex& simplicial_complex, const Json& options) -> SimplicialComplex
        { return tetrahedralize(simplicial_complex, options); },
        py::arg("simplicial_complex"),
        py::arg("options") = Json::object());

    m.def(
        "optimal_transform",
        [](py::array_t<const Float> S, py::array_t<const Float> D)
        {
            auto S_ = as_span_of<const Vector3>(S);
            auto D_ = as_span_of<const Vector3>(D);
            return as_numpy(optimal_transform(S_, D_));
        },
        py::arg("src"),
        py::arg("dst"));

    m.def(
        "optimal_transform",
        [](const SimplicialComplex& S, const SimplicialComplex& D)
        { return as_numpy(optimal_transform(S, D)); },
        py::arg("src"),
        py::arg("dst"));

    m.def("is_trimesh_closed", &is_trimesh_closed, py::arg("sc"));
}
}  // namespace pyuipc::geometry
