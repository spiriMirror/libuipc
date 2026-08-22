#include <pyuipc/constitution/discrete_shell_bending.h>
#include <uipc/constitution/finite_element_extra_constitution.h>
#include <uipc/constitution/discrete_shell_bending.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
using namespace uipc::geometry;
PyDiscreteShellBending::PyDiscreteShellBending(py::module& m)
{
    auto class_DiscreteShellBending =
        py::class_<DiscreteShellBending, FiniteElementExtraConstitution>(
            m, "DiscreteShellBending", R"(DiscreteShellBending constitution for shell bending energy.)");

    class_DiscreteShellBending.def(py::init<const Json&>(),
                                   py::arg("config") = DiscreteShellBending::default_config(),
                                   R"(Create a DiscreteShellBending constitution.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_DiscreteShellBending.def_static("default_config",
                                          &DiscreteShellBending::default_config,
                                          R"(Get the default DiscreteShellBending configuration.
Returns:
    dict: Default configuration dictionary.)");

    class_DiscreteShellBending.def(
        "apply_to",
        [](DiscreteShellBending& self, SimplicialComplex& sc, Float bending_stiffness)
        { self.apply_to(sc, bending_stiffness); },
        py::arg("sc"),
        py::arg("bending_stiffness") = 100.0_kPa,
        R"(Apply DiscreteShellBending constitution to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    bending_stiffness: Bending stiffness in kPa (default: 100.0 kPa).)");

    class_DiscreteShellBending.def(
        "apply_to",
        [](DiscreteShellBending& self, SimplicialComplex& sc, Float young_modulus, Float poisson_ratio)
        { self.apply_to(sc, young_modulus, poisson_ratio); },
        py::arg("sc"),
        py::arg("young_modulus"),
        py::arg("poisson_ratio"),
        R"(Apply DiscreteShellBending constitution to a simplicial complex, computing
the bending stiffness from the material as the classical shell value
E*t^3/(12*(1-nu^2)) (the backend bending measure is the element area, so the
value is used literally). The thickness t is read per edge from the mesh's
vertex "thickness" attribute (average of the two endpoints), so a membrane
(stretch) constitution with thickness must be applied first.
Args:
    sc: SimplicialComplex to apply to.
    young_modulus: Young's modulus of the shell material.
    poisson_ratio: Poisson's ratio of the shell material.)");

    class_DiscreteShellBending.def_static("bending_stiffness",
                                          &DiscreteShellBending::bending_stiffness,
                                          py::arg("young_modulus"),
                                          py::arg("poisson_ratio"),
                                          py::arg("thickness"),
                                          R"(Compute the raw bending_stiffness attribute value E*t^3/(12*(1-nu^2))
(the classical shell bending stiffness; the backend bending measure is the
element area, so the value is used literally).)");
}
}  // namespace pyuipc::constitution
