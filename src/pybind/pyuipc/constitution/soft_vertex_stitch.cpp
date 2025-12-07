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
        py::class_<SoftVertexStitch, InterPrimitiveConstitution>(m, "SoftVertexStitch",
                                                                 R"(SoftVertexStitch constitution for stitching vertices between geometries with soft constraints.)");

    class_SoftVertexStitch.def(py::init<const Json&>(),
                               py::arg("config") = SoftVertexStitch::default_config(),
                               R"(Create a SoftVertexStitch constitution.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_SoftVertexStitch.def_static("default_config", &SoftVertexStitch::default_config,
                                     R"(Get the default SoftVertexStitch configuration.
Returns:
    dict: Default configuration dictionary.)");

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
        py::arg("kappa") = Float{1e6},
        R"(Create geometry for vertex stitching.
Args:
    aim_geo_slots: Tuple of geometry slots to stitch.
    stitched_vert_ids: Array of vertex ID pairs [geometry0_vert, geometry1_vert] to stitch.
    kappa: Stitching stiffness (default: 1e6).
Returns:
    Geometry: Created stitching geometry.)");

    class_SoftVertexStitch.def(
        "create_geometry",
        [](SoftVertexStitch&                            self,
           const SoftVertexStitch::SlotTuple&           aim_geo_slots,
           const py::array_t<Float>&                    stitched_vert_ids,
           const SoftVertexStitch::ContactElementTuple& contact_elements,
           Float                                        kappa)
        {
            return self.create_geometry(aim_geo_slots,
                                        as_span_of<const Vector2i>(stitched_vert_ids),
                                        contact_elements,
                                        kappa);
        },
        py::arg("aim_geo_slots"),
        py::arg("stitched_vert_ids"),
        py::arg("contact_elements"),
        py::arg("kappa") = Float{1e6},
        R"(Create geometry for vertex stitching with contact elements.
Args:
    aim_geo_slots: Tuple of geometry slots to stitch.
    stitched_vert_ids: Array of vertex ID pairs [geometry0_vert, geometry1_vert] to stitch.
    contact_elements: Tuple of contact elements.
    kappa: Stitching stiffness (default: 1e6).
Returns:
    Geometry: Created stitching geometry.)");
};
}  // namespace pyuipc::constitution
