#include <pyuipc/geometry/module.h>
#include <pyuipc/geometry/attribute_slot.h>
#include <pyuipc/geometry/attribute_collection.h>
#include <pyuipc/geometry/geometry.h>
#include <pyuipc/geometry/simplicial_complex.h>
#include <pyuipc/geometry/factory.h>
#include <pyuipc/geometry/implicit_geometry.h>
#include <pyuipc/geometry/geometry_slot.h>
#include <pyuipc/geometry/simplicial_complex_slot.h>
#include <pyuipc/geometry/implicit_geometry_slot.h>
#include <pyuipc/geometry/io.h>
#include <pyuipc/geometry/utils.h>

namespace pyuipc::geometry
{
Module::Module(py::module& m)
{
    PyAttributeSlot{m};
    PyAttributeCollection{m};

    PyGeometry{m};
    PyImplicitGeometry{m};
    PySimplicialComplex{m};

    PyGeometrySlot{m};
    PyImplicitGeometrySlot{m};
    PySimplicialComplexSlot{m};

    PyFactory{m};

    PyIO{m};
    PyUtils{m};
}
}  // namespace pyuipc::geometry
