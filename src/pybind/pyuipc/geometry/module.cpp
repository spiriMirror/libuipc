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

#include <pyuipc/geometry/urdf_io.h>
#include <pyuipc/geometry/utils.h>
#include <pyuipc/geometry/affine_body.h>

namespace pyuipc::geometry
{
PyModule::PyModule(py::module& m)
{
    // Data structures are exported early in main module
    // Only utils/IO functions are exported here (after core module)

    PyFactory{m};

    PyAttributeIO{m};
    PySimplicialComplexIO{m};
    PySpreadSheetIO{m};
    PyUrdfIO{m};
    PyUtils{m};

    // pyuipc.geometry.affine_body
    auto affine_body = m.def_submodule("affine_body");
    PyAffineBody{affine_body};
}
}  // namespace pyuipc::geometry
