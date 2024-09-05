#include <pyuipc/builtin/implicit_geometry_uid_collection.h>
#include <uipc/builtin/implicit_geometry_uid_collection.h>
namespace pyuipc::builtin
{
using namespace uipc::builtin;
PyImplicitGeometryUIDCollection::PyImplicitGeometryUIDCollection(py::module& m)
{
    py::class_<ImplicitGeometryUIDCollection, details::UIDRegister>(m, "ImplicitGeometryUIDCollection")
        .def_static("instance", &ImplicitGeometryUIDCollection::instance);
}
}  // namespace pyuipc::builtin
