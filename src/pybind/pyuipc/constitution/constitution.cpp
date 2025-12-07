#include <pyuipc/constitution/constitution.h>
#include <uipc/constitution/constitution.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyConstitution::PyConstitution(py::module& m)
{
    auto class_IConstitution = py::class_<IConstitution>(m, "IConstitution",
                                                          R"(IConstitution interface for all constitution types (material models and constraints).)");

    class_IConstitution.def("uid", &IConstitution::uid,
                            R"(Get the constitution UID (unique identifier).
Returns:
    int: Constitution UID.)");
    class_IConstitution.def("name", &IConstitution::name,
                            R"(Get the constitution name.
Returns:
    str: Constitution name.)");
    class_IConstitution.def("type", &IConstitution::type,
                            R"(Get the constitution type name.
Returns:
    str: Constitution type name.)");
}
}  // namespace pyuipc::constitution
