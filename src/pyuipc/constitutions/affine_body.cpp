#include <pyuipc/constitutions/affine_body.h>
#include <uipc/constitutions/affine_body.h>
#include <uipc/world/constitution.h>
#include <pyuipc/common/json.h>
namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyAffineBodyConstitution::PyAffineBodyConstitution(py::module& m)
{
    auto class_AffineBodyConstitution =
        py::class_<AffineBodyConstitution, world::IConstitution>(m, "AffineBodyConstitution");

    class_AffineBodyConstitution.def_static("default_config",
                                            &AffineBodyConstitution::default_config);

    class_AffineBodyConstitution.def("apply_to", &AffineBodyConstitution::apply_to);
}
}  // namespace pyuipc::constitution
