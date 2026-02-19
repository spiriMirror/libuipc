#include <pyuipc/constitution/inter_affine_body_extra_constitution.h>
#include <uipc/constitution/inter_affine_body_extra_constitution.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyInterAffineBodyExtraConstitution::PyInterAffineBodyExtraConstitution(py::module& m)
{
    auto class_InterAffineBodyExtraConstitution =
        py::class_<InterAffineBodyExtraConstitution, IConstitution>(
            m,
            "InterAffineBodyExtraConstitution",
            R"(InterAffineBodyExtraConstitution base class for extra constitutions on inter-affine-body geometries.)");
}
}  // namespace pyuipc::constitution
