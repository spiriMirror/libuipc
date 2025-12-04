#include <pyuipc/constitution/module.h>
#include <pyuipc/constitution/empty.h>
#include <pyuipc/constitution/constitution.h>
#include <pyuipc/constitution/elastic_moduli.h>
#include <pyuipc/constitution/finite_element_constitution.h>
#include <pyuipc/constitution/particle.h>
#include <pyuipc/constitution/hookean_spring.h>
#include <pyuipc/constitution/neo_hookean_shell.h>
#include <pyuipc/constitution/strain_limiting_baraff_witkin.h>
#include <pyuipc/constitution/stable_neo_hookean.h>
#include <pyuipc/constitution/affine_body_constitution.h>
#include <pyuipc/constitution/constraint.h>
#include <pyuipc/constitution/soft_position_constraint.h>
#include <pyuipc/constitution/finite_element_extra_constitution.h>
#include <pyuipc/constitution/kirchhoff_rod_bending.h>
#include <pyuipc/constitution/soft_transform_constraint.h>
#include <pyuipc/constitution/discrete_shell_bending.h>
#include <pyuipc/constitution/arap.h>
#include <pyuipc/constitution/inter_affine_body_constitution.h>
#include <pyuipc/constitution/affine_body_revolute_joint.h>
#include <pyuipc/constitution/inter_primitive_constitution.h>
#include <pyuipc/constitution/soft_vertex_stitch.h>
#include <pyuipc/constitution/affine_body_external_force.h>

namespace pyuipc::constitution
{
PyModule::PyModule(py::module& m)
{
    PyConstitution{m};
    PyConstraint{m};
    PyElasticModuli{m};

    // Affine Body Constitutions
    PyAffineBodyConstitution{m};
    PyInterAffineBodyConstitution{m};
    PyAffineBodyRevoluteJoint{m};
    PyAffineBodyExternalForce{m};

    // Finite Element Constitutions
    PyFiniteElementConstitution{m};
    PyEmpty{m};
    PyParticle{m};
    PyHookeanSpring{m};
    PyNeoHookeanShell{m};
    PyStrainLimitingBaraffWitkinShell{m};
    PyStableNeoHookean{m};
    PyARAP{m};

    // Finite Extra Constitutions
    PyFiniteElementExtraConstitution{m};
    PyKirchhoffRodBending{m};
    PyDiscreteShellBending{m};

    // Inter Primitive Constitutions
    PyInterPrimitiveConstitution{m};
    PySoftVertexStitch{m};

    // Constraints
    PySoftPositionConstraint{m};
    PySoftTransformConstraint{m};
}
}  // namespace pyuipc::constitution
