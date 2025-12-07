#include <pyuipc/builtin/implicit_geometry_uid_collection.h>
#include <uipc/builtin/implicit_geometry_uid_collection.h>
namespace pyuipc::builtin
{
using namespace uipc::builtin;
PyImplicitGeometryUIDCollection::PyImplicitGeometryUIDCollection(py::module& m)
{
    py::class_<ImplicitGeometryUIDCollection, details::UIDRegister>(m, "ImplicitGeometryUIDCollection",
                                                                     R"(Collection of implicit geometry UIDs (unique identifiers for implicit geometries).)")
        .def_static("instance", &ImplicitGeometryUIDCollection::instance,
                   R"(Get the singleton instance of the implicit geometry UID collection.
Returns:
    ImplicitGeometryUIDCollection: Singleton instance.)");
}
}  // namespace pyuipc::builtin
