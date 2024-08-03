#include <pyuipc/world/constitution.h>
#include <uipc/world/constitution.h>

namespace pyuipc::world
{
using namespace uipc::world;
PyConstitution::PyConstitution(py::module& m)
{
    py::enum_<ConstitutionType>(m, "ConstitutionType")
        .value("AffineBody", ConstitutionType::AffineBody)
        .value("FiniteElement", ConstitutionType::FiniteElement)
        .export_values();


    auto class_IConstitution = py::class_<IConstitution>(m, "IConstitution");

    class_IConstitution.def("uid", &IConstitution::uid);
    class_IConstitution.def("name", &IConstitution::name);
    class_IConstitution.def("type", &IConstitution::type);
}
}  // namespace pyuipc::world
