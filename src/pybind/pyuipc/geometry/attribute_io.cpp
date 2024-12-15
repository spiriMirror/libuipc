#include <pyuipc/geometry/attribute_io.h>
#include <uipc/io/attribute_io.h>
#include <pyuipc/as_numpy.h>
#include <Eigen/Geometry>

namespace pyuipc::geometry
{
using namespace uipc::geometry;
PyAttributeIO::PyAttributeIO(py::module& m)
{
    auto class_AttributeIO = py::class_<AttributeIO>(m, "AttributeIO");

    class_AttributeIO.def(py::init<std::string_view>(), py::arg("file"));

    class_AttributeIO.def("read", &AttributeIO::read, py::arg("name"), py::arg("slot"));
}
}  // namespace pyuipc::geometry
