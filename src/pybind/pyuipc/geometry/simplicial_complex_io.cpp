#include <pyuipc/geometry/simplicial_complex_io.h>
#include <uipc/io/simplicial_complex_io.h>
#include <uipc/io/spread_sheet_io.h>
#include <pyuipc/as_numpy.h>
#include <Eigen/Geometry>

namespace pyuipc::geometry
{
using namespace uipc::geometry;
PySimplicialComplexIO::PySimplicialComplexIO(py::module& m)
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
}
}  // namespace pyuipc::geometry
