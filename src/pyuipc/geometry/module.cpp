#include <pyuipc/geometry/module.h>
#include <pyuipc/geometry/attribute_slot.h>
#include <pyuipc/geometry/attribute_collection.h>
#include <pyuipc/geometry/geometry.h>
#include <pyuipc/geometry/simplicial_complex.h>
#include <pyuipc/geometry/simplex_slot.h>
#include <pyuipc/geometry/utils/factory.h>
#include <pyuipc/geometry/implicit_geometry.h>
#include <pyuipc/geometry/geometry_slot.h>
#include <pyuipc/geometry/simplicial_complex_slot.h>
#include <pyuipc/geometry/implicit_geometry_slot.h>

namespace pyuipc::geometry
{
Module::Module(py::module& m)
{
    PyAttributeSlot{m};
    PyAttributeCollection{m};
    PySimplexSlot{m};

    PyGeometry{m};
    PyImplicitGeometry{m};
    PySimplicialComplex{m};

    PyGeometrySlot{m};
    PyImplicitGeometrySlot{m};
    PySimplicialComplexSlot{m};

    PyFactory{m};
}
}  // namespace pyuipc::geometry
