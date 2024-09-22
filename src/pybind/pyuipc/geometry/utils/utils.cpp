#include <pyuipc/geometry/utils/utils.h>
#include <Eigen/Geometry>
#include <uipc/geometry/utils/label_surface.h>
#include <uipc/geometry/utils/label_triangle_orient.h>
#include <uipc/geometry/utils/flip_inward_triangles.h>
#include <uipc/geometry/utils/extract_surface.h>
#include <uipc/geometry/utils/merge.h>
#include <uipc/geometry/utils/apply_instances.h>
#include <uipc/geometry/utils/closure.h>
#include <uipc/io/spread_sheet_io.h>
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
          {
              spdlog::info("Extracting surface from simplicial complex");
              return extract_surface(simplicial_complex);
          });

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

    auto class_SpreadSheetIO = py::class_<SpreadSheetIO>(m, "SpreadSheetIO");
    class_SpreadSheetIO.def(py::init<std::string_view>(), py::arg("output_folder") = "./");
    class_SpreadSheetIO.def(
        "write_json",
        [](SpreadSheetIO& self, std::string geo_name, const SimplicialComplex& simplicial_complex)
        { self.write_json(geo_name, simplicial_complex); },
        py::arg("geo_name"),
        py::arg("simplicial_complex"));

    class_SpreadSheetIO.def("write_json",
                            [](SpreadSheetIO& self, const SimplicialComplex& simplicial_complex)
                            { self.write_json(simplicial_complex); });

    class_SpreadSheetIO.def(
        "write_csv",
        [](SpreadSheetIO& self, std::string geo_name, const SimplicialComplex& simplicial_complex)
        { self.write_csv(geo_name, simplicial_complex); },
        py::arg("geo_name"),
        py::arg("simplicial_complex"));

    class_SpreadSheetIO.def("write_csv",
                            [](SpreadSheetIO& self, const SimplicialComplex& simplicial_complex)
                            { self.write_csv(simplicial_complex); });
}
}  // namespace pyuipc::geometry
