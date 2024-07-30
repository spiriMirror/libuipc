#include <pyuipc/geometry/utils/factory.h>
#include <uipc/geometry/utils/factory.h>
#include <pyuipc/as_numpy.h>
namespace pyuipc::geometry
{
using namespace uipc::geometry;

PyFactory::PyFactory(py::module& m)
{
    m.def("tetmesh",
          [](py::array_t<Float> Vs, py::array_t<IndexT> Ts)
          { return tetmesh(as_span_of<Vector3>(Vs), as_span_of<Vector4i>(Ts)); });
}
}  // namespace pyuipc::geometry
