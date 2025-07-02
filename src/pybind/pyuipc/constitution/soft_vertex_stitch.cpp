#include <pyuipc/constitution/soft_vertex_stitch.h>
#include <uipc/constitution/soft_vertex_stitch.h>
#include <uipc/constitution/constitution.h>
#include <pyuipc/common/json.h>
#include <pyuipc/as_numpy.h>
namespace pyuipc::constitution
{
using namespace uipc::constitution;

PySoftVertexStitch::PySoftVertexStitch(py::module& m)
{
    auto class_SoftVertexStitch =
        py::class_<SoftVertexStitch, InterPrimitiveConstitution>(m, "SoftVertexStitch");

    class_SoftVertexStitch.def(py::init<const Json&>(),
                               py::arg("config") = SoftVertexStitch::default_config());

    class_SoftVertexStitch.def_static("default_config", &SoftVertexStitch::default_config);

    class_SoftVertexStitch.def(
        "create_geometry",
        [](SoftVertexStitch&                  self,
           const SoftVertexStitch::SlotTuple& aim_geo_slots,
           const py::array_t<Float>&          stitched_vert_ids,
           Float                              kappa)
        {
            return self.create_geometry(
                aim_geo_slots, as_span_of<const Vector2i>(stitched_vert_ids), kappa);
        },
        py::arg("aim_geo_slots"),
        py::arg("stitched_vert_ids"),
        py::arg("kappa") = Float{1e6});
}
}  // namespace pyuipc::constitution
