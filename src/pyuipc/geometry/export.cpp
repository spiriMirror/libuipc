#include <pyuipc/pyuipc.h>
#include <pyuipc/geometry/attribute_slot.h>
#include <pyuipc/geometry/attribute_collection.h>
#include <pyuipc/geometry/geometry.h>
#include <pyuipc/geometry/simplicial_complex.h>
#include <pyuipc/geometry/simplex_slot.h>
#include <pyuipc/geometry/utils/factory.h>
namespace pyuipc::geometry
{
static Module M(
    [](py::module& m)
    {
        auto sub = m.def_submodule("geometry");

        PyAttributeSlot{sub};
        PyAttributeCollection{sub};
        PySimplexSlot{sub};
        PyGeometry{sub};
        PySimplicialComplex{sub};
        PyFactory{sub};
    });
}  // namespace pyuipc::geometry
