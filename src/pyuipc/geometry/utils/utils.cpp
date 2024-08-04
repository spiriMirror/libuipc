#include <pyuipc/geometry/utils/utils.h>
#include <uipc/geometry/utils/label_surface.h>
#include <uipc/geometry/utils/label_triangle_orient.h>
#include <uipc/geometry/utils/spread_sheet_io.h>
#include <uipc/geometry/utils/flip_inward_triangles.h>
namespace pyuipc::geometry
{
using namespace uipc::geometry;
PyUtils::PyUtils(py::module& m)
{
    m.def("label_surface", &label_surface);
    m.def("label_triangle_orient", &label_triangle_orient);
    m.def("flip_inward_triangles", &flip_inward_triangles);

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
