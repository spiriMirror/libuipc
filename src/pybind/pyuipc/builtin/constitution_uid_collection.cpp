#include <pyuipc/builtin/constitution_uid_collection.h>
#include <uipc/builtin/constitution_uid_collection.h>
namespace pyuipc::builtin
{
using namespace uipc::builtin;
PyConstitutionUIDCollection::PyConstitutionUIDCollection(py::module& m)
{
    py::class_<ConstitutionUIDCollection, details::UIDRegister>(m, "ConstitutionUIDCollection",
                                                                 R"(Collection of constitution UIDs (unique identifiers for constitutions).)")
        .def_static("instance", &ConstitutionUIDCollection::instance,
                   R"(Get the singleton instance of the constitution UID collection.
Returns:
    ConstitutionUIDCollection: Singleton instance.)");
}
}  // namespace pyuipc::builtin
