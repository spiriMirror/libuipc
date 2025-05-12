#include <pyuipc/geometry/module.h>
#include <pyuipc/geometry/attribute_slot.h>
#include <pyuipc/geometry/attribute_collection.h>
#include <pyuipc/geometry/geometry.h>
#include <pyuipc/geometry/simplicial_complex.h>
#include <pyuipc/geometry/factory.h>
#include <pyuipc/geometry/geometry_atlas.h>

#include <pyuipc/geometry/implicit_geometry.h>
#include <pyuipc/geometry/geometry_slot.h>
#include <pyuipc/geometry/simplicial_complex_slot.h>

#include <pyuipc/geometry/attribute_io.h>
#include <pyuipc/geometry/implicit_geometry_slot.h>
#include <pyuipc/geometry/simplicial_complex_io.h>
#include <pyuipc/geometry/spread_sheet_io.h>

#include <pyuipc/geometry/utils.h>

namespace pyuipc::geometry
{
PyModule::PyModule(py::module& m)
{
    PyAttributeSlot{m};
    PyAttributeCollection{m};

    PyGeometry{m};
    PyImplicitGeometry{m};
    PySimplicialComplex{m};

    PyGeometryAtlas{m};

    PyGeometrySlot{m};
    PyImplicitGeometrySlot{m};
    PySimplicialComplexSlot{m};

    PyFactory{m};

    PyAttributeIO{m};
    PySimplicialComplexIO{m};
    PySpreadSheetIO{m};
    PyUtils{m};
}
}  // namespace pyuipc::geometry
