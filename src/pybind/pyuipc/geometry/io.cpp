#include <pyuipc/geometry/io.h>
#include <uipc/io/simplicial_complex_io.h>
#include <uipc/io/spread_sheet_io.h>
#include <pyuipc/as_numpy.h>
#include <Eigen/Geometry>

namespace pyuipc::geometry
{
using namespace uipc::geometry;
PyIO::PyIO(py::module& m)
{
    auto class_SimplicialComplexIO =
        py::class_<SimplicialComplexIO>(m, "SimplicialComplexIO");

    class_SimplicialComplexIO.def(py::init<>());

    class_SimplicialComplexIO.def(
        py::init<>([](const Transform& pre_transform)
                   { return SimplicialComplexIO(pre_transform); }));

    class_SimplicialComplexIO.def(py::init<>(
        [](py::array_t<Float> pre_transform)
        {
            auto mat = to_matrix<Matrix4x4>(pre_transform);
            return SimplicialComplexIO(mat);
        }));

    class_SimplicialComplexIO.def("read", &SimplicialComplexIO::read);

    class_SimplicialComplexIO.def("write", &SimplicialComplexIO::write);

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
