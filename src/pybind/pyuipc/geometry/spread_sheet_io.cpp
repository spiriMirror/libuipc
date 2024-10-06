#include <pyuipc/geometry/spread_sheet_io.h>
#include <uipc/io/simplicial_complex_io.h>
#include <uipc/io/spread_sheet_io.h>
#include <pyuipc/as_numpy.h>
#include <Eigen/Geometry>

namespace pyuipc::geometry
{
using namespace uipc::geometry;
PySpreadSheetIO::PySpreadSheetIO(py::module& m)
{
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
