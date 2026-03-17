#include <pyuipc/constitution/soft_vertex_triangle_stitch.h>
#include <uipc/constitution/soft_vertex_triangle_stitch.h>
#include <uipc/common/unit.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PySoftVertexTriangleStitch::PySoftVertexTriangleStitch(py::module_& m)
{
    auto class_SoftVertexTriangleStitch =
        py::class_<SoftVertexTriangleStitch, InterPrimitiveConstitution>(
            m, "SoftVertexTriangleStitch", R"(SoftVertexTriangleStitch: (vertex, triangle) pairs form tetrahedra with StableNeoHookean energy. UID 30.)");

    class_SoftVertexTriangleStitch.def(
        py::init<const Json&>(),
        py::arg("config") = SoftVertexTriangleStitch::default_config(),
        R"(Create a SoftVertexTriangleStitch constitution.)");

    class_SoftVertexTriangleStitch.def_static("default_config",
                                              &SoftVertexTriangleStitch::default_config,
                                              R"(Get the default configuration.)");

    class_SoftVertexTriangleStitch.def(
        "create_geometry",
        [](SoftVertexTriangleStitch&                  self,
           const SoftVertexTriangleStitch::SlotTuple& aim_geo_slots,
           const SoftVertexTriangleStitch::SlotTuple& rest_geo_slots,
           numpy_array<IndexT>                 stitched_vert_tri_ids,
           const ElasticModuli&                       moduli,
           Float                                      min_separate_distance)
        {
            return self.create_geometry(aim_geo_slots,
                                        rest_geo_slots,
                                        as_span_of<const Vector2i>(stitched_vert_tri_ids),
                                        moduli,
                                        min_separate_distance);
        },
        py::arg("aim_geo_slots"),
        py::arg("rest_geo_slots"),
        py::arg("stitched_vert_tri_ids"),
        py::arg("moduli") = ElasticModuli::youngs_poisson(120.0_kPa, 0.49),
        py::arg("min_separate_distance") = 0.001,
        R"(Create stitch geometry from explicit (vertex_index, triangle_index) pairs.
    When a tet is degenerate, rest vertex is offset by min_separate_distance (default 1 mm).
Args:
    aim_geo_slots: Tuple (vertex_provider_slot, triangle_provider_slot).
    rest_geo_slots: Tuple of rest geometry slots for Dm_inv/rest_volume.
    stitched_vert_tri_ids: Nx2 array [vertex_id, triangle_id] per pair.
    moduli: ElasticModuli (default: Young's modulus 120 kPa, Poisson's ratio 0.49).
    min_separate_distance: Distance (m) when tet is degenerate. Default 0.001 (1 mm).
Returns:
    Geometry: Stitch geometry.)");

    class_SoftVertexTriangleStitch.def(
        "create_geometry",
        [](SoftVertexTriangleStitch&                  self,
           const SoftVertexTriangleStitch::SlotTuple& aim_geo_slots,
           const SoftVertexTriangleStitch::SlotTuple& rest_geo_slots,
           const geometry::Geometry&                  pair_geometry,
           const ElasticModuli&                       moduli,
           Float                                      min_separate_distance)
        {
            return self.create_geometry(
                aim_geo_slots, rest_geo_slots, pair_geometry, moduli, min_separate_distance);
        },
        py::arg("aim_geo_slots"),
        py::arg("rest_geo_slots"),
        py::arg("pair_geometry"),
        py::arg("moduli") = ElasticModuli::youngs_poisson(120.0_kPa, 0.49),
        py::arg("min_separate_distance") = 0.001,
        R"(Create stitch geometry from the Geometry returned by closest_vertex_triangle_pairs.
    When a tet is degenerate, uses min_separate_distance (default 1 mm). For per-instance values, add instance attribute 'min_separate_distance' (Float) to pair_geometry and modify it.
Args:
    aim_geo_slots: Tuple (vertex_provider_slot, triangle_provider_slot).
    rest_geo_slots: Tuple of rest geometry slots.
    pair_geometry: Geometry with instances holding topo Vector2i (vertex_index, triangle_index).
    moduli: ElasticModuli (default: Young's modulus 120 kPa, Poisson's ratio 0.49).
    min_separate_distance: Distance (m) when tet is degenerate. Default 0.001 (1 mm).
Returns:
    Geometry: Stitch geometry.)");
}
}  // namespace pyuipc::constitution
