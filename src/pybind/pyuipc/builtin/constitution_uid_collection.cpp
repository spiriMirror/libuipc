#include <pyuipc/builtin/constitution_uid_collection.h>
#include <uipc/builtin/constitution_uid_collection.h>
namespace pyuipc::builtin
{
using namespace uipc::builtin;
PyConstitutionUIDCollection::PyConstitutionUIDCollection(py::module& m)
{
    py::class_<ConstitutionUIDCollection, details::UIDRegister>(m, "ConstitutionUIDCollection")
        .def_static("instance", &ConstitutionUIDCollection::instance);
}
}  // namespace pyuipc::builtin
