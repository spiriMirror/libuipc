#include <pyuipc/geometry/utils.h>
#include <Eigen/Geometry>
#include <uipc/geometry/utils/label_surface.h>
#include <uipc/geometry/utils/label_triangle_orient.h>
#include <uipc/geometry/utils/flip_inward_triangles.h>
#include <uipc/geometry/utils/extract_surface.h>
#include <uipc/geometry/utils/merge.h>
#include <uipc/geometry/utils/apply_transform.h>
#include <uipc/geometry/utils/closure.h>
#include <uipc/geometry/utils/label_connected_vertices.h>
#include <pyuipc/as_numpy.h>

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
    m.def("label_surface", &label_surface);
    m.def("label_triangle_orient", &label_triangle_orient);
    m.def("flip_inward_triangles", &flip_inward_triangles);
    m.def("extract_surface",
          [](const SimplicialComplex& simplicial_complex)
          { return extract_surface(simplicial_complex); });

    m.def("extract_surface",
          [&](py::list list_of_sc)
          {
              auto simplicial_complexes = vector_of_sc(list_of_sc);
              return extract_surface(simplicial_complexes);
          });

    m.def("merge",
          [&](py::list list_of_sc)
          {
              auto simplicial_complexes = vector_of_sc(list_of_sc);
              return merge(simplicial_complexes);
          });

    m.def("apply_transform",
          [](const SimplicialComplex& simplicial_complex) -> py::list
          {
              auto scs  = apply_transform(simplicial_complex);
              auto list = list_of_sc(scs);
              return list;
          });

    m.def("facet_closure", &facet_closure);
    m.def("label_connected_vertices", &label_connected_vertices);
}
}  // namespace pyuipc::geometry
