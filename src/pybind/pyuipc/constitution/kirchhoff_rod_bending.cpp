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
        py::class_<KirchhoffRodBending, FiniteElementExtraConstitution>(m, "KirchhoffRodBending",
                                                                         R"(KirchhoffRodBending constitution for rod bending energy.)");

    class_KirchhoffRodBending.def(py::init<const Json&>(),
                                  py::arg("config") = KirchhoffRodBending::default_config(),
                                  R"(Create a KirchhoffRodBending constitution.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_KirchhoffRodBending.def_static("default_config", &KirchhoffRodBending::default_config,
                                        R"(Get the default KirchhoffRodBending configuration.
Returns:
    dict: Default configuration dictionary.)");

    class_KirchhoffRodBending.def(
        "apply_to", &KirchhoffRodBending::apply_to, py::arg("sc"), py::arg("E") = 10.0_MPa,
        R"(Apply KirchhoffRodBending constitution to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    E: Young's modulus in MPa (default: 10.0 MPa).)");
}
}  // namespace pyuipc::constitution
