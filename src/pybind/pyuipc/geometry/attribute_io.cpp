#include <pyuipc/geometry/attribute_io.h>
#include <uipc/io/attribute_io.h>
#include <pyuipc/as_numpy.h>
#include <Eigen/Geometry>

namespace pyuipc::geometry
{
using namespace uipc::geometry;
PyAttributeIO::PyAttributeIO(py::module& m)
{
    auto class_AttributeIO = py::class_<AttributeIO>(m, "AttributeIO",
                                                      R"(AttributeIO class for reading attributes from files.)");

    class_AttributeIO.def(py::init<std::string_view>(), py::arg("file"),
                        R"(Create an AttributeIO instance.
Args:
    file: File path to read from.)");

    class_AttributeIO.def("read", &AttributeIO::read, py::arg("name"), py::arg("slot"),
                          R"(Read an attribute from the file.
Args:
    name: Attribute name to read.
    slot: AttributeSlot to store the read data.)");
}
}  // namespace pyuipc::geometry
