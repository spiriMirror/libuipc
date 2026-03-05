#include <pyuipc/constitution/soft_vertex_edge_stitch.h>
#include <uipc/constitution/soft_vertex_edge_stitch.h>
#include <uipc/common/unit.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PySoftVertexEdgeStitch::PySoftVertexEdgeStitch(py::module& m)
{
    auto class_SoftVertexEdgeStitch =
        py::class_<SoftVertexEdgeStitch, InterPrimitiveConstitution>(
            m, "SoftVertexEdgeStitch", R"(SoftVertexEdgeStitch: (vertex, edge) pairs form triangles with StVK membrane energy. UID 29.)");

    class_SoftVertexEdgeStitch.def(py::init<const Json&>(),
                                   py::arg("config") = SoftVertexEdgeStitch::default_config(),
                                   R"(Create a SoftVertexEdgeStitch constitution.)");

    class_SoftVertexEdgeStitch.def_static("default_config",
                                          &SoftVertexEdgeStitch::default_config,
                                          R"(Get the default configuration.)");

    class_SoftVertexEdgeStitch.def(
        "create_geometry",
        [](SoftVertexEdgeStitch&                  self,
           const SoftVertexEdgeStitch::SlotTuple& aim_geo_slots,
           const SoftVertexEdgeStitch::SlotTuple& rest_geo_slots,
           const py::array_t<IndexT>&             stitched_vert_edge_ids,
           const ElasticModuli2D&                 moduli,
           Float                                  thickness,
           Float                                  min_separate_distance)
        {
            return self.create_geometry(aim_geo_slots,
                                        rest_geo_slots,
                                        as_span_of<const Vector2i>(stitched_vert_edge_ids),
                                        moduli,
                                        thickness,
                                        min_separate_distance);
        },
        py::arg("aim_geo_slots"),
        py::arg("rest_geo_slots"),
        py::arg("stitched_vert_edge_ids"),
        py::arg("moduli")    = ElasticModuli2D::youngs_poisson(1.0_MPa, 0.49),
        py::arg("thickness") = 0.001,
        py::arg("min_separate_distance") = 0.001,
        R"(Create stitch geometry from explicit (vertex_index, edge_index) pairs.)");

    class_SoftVertexEdgeStitch.def(
        "create_geometry",
        [](SoftVertexEdgeStitch&                  self,
           const SoftVertexEdgeStitch::SlotTuple& aim_geo_slots,
           const SoftVertexEdgeStitch::SlotTuple& rest_geo_slots,
           const geometry::Geometry&              pair_geometry,
           const ElasticModuli2D&                 moduli,
           Float                                  thickness,
           Float                                  min_separate_distance)
        {
            return self.create_geometry(
                aim_geo_slots, rest_geo_slots, pair_geometry, moduli, thickness, min_separate_distance);
        },
        py::arg("aim_geo_slots"),
        py::arg("rest_geo_slots"),
        py::arg("pair_geometry"),
        py::arg("moduli")    = ElasticModuli2D::youngs_poisson(1.0_MPa, 0.49),
        py::arg("thickness") = 0.001,
        py::arg("min_separate_distance") = 0.001,
        R"(Create stitch geometry from the Geometry returned by closest_vertex_edge_pairs.)");
}
}  // namespace pyuipc::constitution
