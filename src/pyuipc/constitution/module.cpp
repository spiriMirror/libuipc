#include <pyuipc/constitution/module.h>
#include <pyuipc/constitution/constitution.h>
#include <pyuipc/constitution/elastic_moduli.h>
#include <pyuipc/constitution/fem_constitution.h>
#include <pyuipc/constitution/particle.h>
#include <pyuipc/constitution/hookean_spring.h>
#include <pyuipc/constitution/shell_neo_hookean.h>
#include <pyuipc/constitution/stable_neo_hookean.h>
#include <pyuipc/constitution/affine_body.h>
#include <pyuipc/constitution/constraint.h>
#include <pyuipc/constitution/soft_position_constraint.h>

namespace pyuipc::constitution
{
Module::Module(py::module& m)
{
    PyConstitution{m};
    PyConstraint{m};
    PyElasticModuli{m};

    // Finite element
    PyFiniteElementConstitution{m};
    PyParticle{m};
    PyHookeanSpring{m};
    PyShellNeoHookean{m};
    PyStableNeoHookean{m};

    // Affine body
    PyAffineBodyConstitution{m};

    // Constraints
    PySoftPositionConstraint{m};
}
}  // namespace pyuipc::constitution
