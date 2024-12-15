#include <pyuipc/common/unit.h>
#include <uipc/common/unit.h>
namespace pyuipc
{
using namespace uipc;
#define def_unit(name) m.attr(#name) = 1.0_##name;

PyUnit::PyUnit(py::module& m)
{
    def_unit(s);

    def_unit(m);
    def_unit(km);
    def_unit(mm);


    def_unit(Pa);
    def_unit(kPa);
    def_unit(MPa);
    def_unit(GPa);

    def_unit(N);
}

#undef def_unit
}  // namespace pyuipc
