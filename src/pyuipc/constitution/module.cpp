#include <pyuipc/constitution/module.h>
#include <pyuipc/constitution/constitution.h>
#include <pyuipc/constitution/elastic_moduli.h>
#include <pyuipc/constitution/fem_constitution.h>
#include <pyuipc/constitution/stable_neo_hookean.h>
#include <pyuipc/constitution/affine_body.h>
namespace pyuipc::constitution
{
Module::Module(py::module& m)
{
    PyConstitution{m};
    PyElasticModuli{m};

    PyFiniteElementConstitution{m};
    PyStableNeoHookean{m};

    PyAffineBodyConstitution{m};
}
}  // namespace pyuipc::constitution
