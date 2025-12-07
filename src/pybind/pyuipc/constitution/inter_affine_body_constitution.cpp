#include <pyuipc/constitution/inter_affine_body_constitution.h>
#include <uipc/constitution/inter_affine_body_constitution.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyInterAffineBodyConstitution::PyInterAffineBodyConstitution(py::module& m)
{
    auto class_InterAffineBodyConstitution =
        py::class_<InterAffineBodyConstitution, IConstitution>(m, "InterAffineBodyConstitution",
                                                                R"(InterAffineBodyConstitution base class for constitutions between affine bodies (e.g., joints).)");
}
}  // namespace pyuipc::constitution