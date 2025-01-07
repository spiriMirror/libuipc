#include <pyuipc/constitution/module.h>
#include <pyuipc/constitution/empty.h>
#include <pyuipc/constitution/constitution.h>
#include <pyuipc/constitution/elastic_moduli.h>
#include <pyuipc/constitution/finite_element_constitution.h>
#include <pyuipc/constitution/particle.h>
#include <pyuipc/constitution/hookean_spring.h>
#include <pyuipc/constitution/neo_hookean_shell.h>
#include <pyuipc/constitution/stable_neo_hookean.h>
#include <pyuipc/constitution/affine_body_constitution.h>
#include <pyuipc/constitution/constraint.h>
#include <pyuipc/constitution/soft_position_constraint.h>
#include <pyuipc/constitution/finite_element_extra_constitution.h>
#include <pyuipc/constitution/kirchhoff_rod_bending.h>
#include <pyuipc/constitution/soft_transform_constraint.h>
#include <pyuipc/constitution/discrete_shell_bending.h>
#include <pyuipc/constitution/arap.h>

namespace pyuipc::constitution
{
PyModule::PyModule(py::module& m)
{
    PyConstitution{m};
    PyConstraint{m};
    PyElasticModuli{m};

    // Affine Body Constitutions
    PyAffineBodyConstitution{m};

    // Finite Element Constitutions
    PyFiniteElementConstitution{m};
    PyEmpty{m};
    PyParticle{m};
    PyHookeanSpring{m};
    PyNeoHookeanShell{m};
    PyStableNeoHookean{m};
    PyARAP{m};

    // Finite Extra Constitutions
    PyFiniteElementExtraConstitution{m};
    PyKirchhoffRodBending{m};
    PyDiscreteShellBending{m};

    // Constraints
    PySoftPositionConstraint{m};
    PySoftTransformConstraint{m};
}
}  // namespace pyuipc::constitution
