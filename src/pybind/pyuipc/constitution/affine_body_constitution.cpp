#include <pyuipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/constitution.h>
#include <pyuipc/common/json.h>
namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyAffineBodyConstitution::PyAffineBodyConstitution(py::module& m)
{
    auto class_AffineBodyConstitution =
        py::class_<AffineBodyConstitution, IConstitution>(m, "AffineBodyConstitution");

    class_AffineBodyConstitution.def(py::init<const Json&>(),
                                     py::arg("config") =
                                         AffineBodyConstitution::default_config());

    class_AffineBodyConstitution.def_static("default_config",
                                            &AffineBodyConstitution::default_config);

    class_AffineBodyConstitution.def("apply_to",
                                     &AffineBodyConstitution::apply_to,
                                     py::arg("sc"),
                                     py::arg("kappa"),
                                     py::arg("mass_density") = 1000.0);
}
}  // namespace pyuipc::constitution
