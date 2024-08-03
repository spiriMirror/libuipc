#include <pyuipc/geometry/utils/io.h>
#include <uipc/geometry/utils/io.h>
#include <pyuipc/as_numpy.h>
namespace pyuipc::geometry
{
using namespace uipc::geometry;
PyIO::PyIO(py::module& m)
{
    auto class_SimplicialComplexIO =
        py::class_<SimplicialComplexIO>(m, "SimplicialComplexIO");

    class_SimplicialComplexIO.def(py::init<>());

    class_SimplicialComplexIO.def(py::init<>(
        [](py::array_t<Float> transform)
        {
            auto matrix = to_matrix<Matrix4x4>(transform);
            return std::make_unique<SimplicialComplexIO>(matrix);
        }));

    class_SimplicialComplexIO.def("read", &SimplicialComplexIO::read);
}
}  // namespace pyuipc::geometry
