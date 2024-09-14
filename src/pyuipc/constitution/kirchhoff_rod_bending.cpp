#include <pyuipc/constitution/kirchhoff_rod_bending.h>
#include <uipc/constitution/finite_element_extra_constitution.h>
#include <uipc/constitution/kirchhoff_rod_bending.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyKirchhoffRodBending::PyKirchhoffRodBending(py::module& m)
{
    auto class_KirchhoffRodBending =
        py::class_<KirchhoffRodBending, FiniteElementExtraConstitution>(m, "KirchhoffRodBending");

    class_KirchhoffRodBending.def(py::init<const Json&>(),
                                  py::arg("config") = KirchhoffRodBending::default_config());

    class_KirchhoffRodBending.def_static("default_config", &KirchhoffRodBending::default_config);

    class_KirchhoffRodBending.def(
        "apply_to", &KirchhoffRodBending::apply_to, py::arg("sc"), py::arg("E") = 10.0_MPa);
}
}  // namespace pyuipc::constitution
