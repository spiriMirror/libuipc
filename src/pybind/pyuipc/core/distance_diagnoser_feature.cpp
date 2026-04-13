#include <pyuipc/core/distance_diagnoser_feature.h>
#include <uipc/core/distance_diagnoser_feature.h>

namespace pyuipc::core
{
using namespace uipc::core;
PyDistanceDiagnoserFeature::PyDistanceDiagnoserFeature(py::module& m)
{
    auto class_DistanceDiagnoserFeature =
        py::class_<DistanceDiagnoserFeature, IFeature, S<DistanceDiagnoserFeature>>(
            m, "DistanceDiagnoserFeature");

    class_DistanceDiagnoserFeature.def(
        "compute_point_triangle_distance",
        [](DistanceDiagnoserFeature&    self,
           geometry::Geometry&          R,
           geometry::SimplicialComplex& points,
           geometry::SimplicialComplex& triangles)
        { self.compute_point_triangle_distance(R, points, triangles); },
        py::arg("R"),
        py::arg("points"),
        py::arg("triangles"));

    class_DistanceDiagnoserFeature.def(
        "compute_edge_edge_distance",
        [](DistanceDiagnoserFeature&    self,
           geometry::Geometry&          R,
           geometry::SimplicialComplex& edges_a,
           geometry::SimplicialComplex& edges_b,
           geometry::SimplicialComplex& rest_edges_a,
           geometry::SimplicialComplex& rest_edges_b)
        { self.compute_edge_edge_distance(R, edges_a, edges_b, rest_edges_a, rest_edges_b); },
        py::arg("R"),
        py::arg("edges_a"),
        py::arg("edges_b"),
        py::arg("rest_edges_a"),
        py::arg("rest_edges_b"));

    class_DistanceDiagnoserFeature.def(
        "compute_point_edge_distance",
        [](DistanceDiagnoserFeature&    self,
           geometry::Geometry&          R,
           geometry::SimplicialComplex& points,
           geometry::SimplicialComplex& edges)
        { self.compute_point_edge_distance(R, points, edges); },
        py::arg("R"),
        py::arg("points"),
        py::arg("edges"));

    class_DistanceDiagnoserFeature.def(
        "compute_point_point_distance",
        [](DistanceDiagnoserFeature&    self,
           geometry::Geometry&          R,
           geometry::SimplicialComplex& points_a,
           geometry::SimplicialComplex& points_b)
        { self.compute_point_point_distance(R, points_a, points_b); },
        py::arg("R"),
        py::arg("points_a"),
        py::arg("points_b"));

    class_DistanceDiagnoserFeature.attr("FeatureName") =
        DistanceDiagnoserFeature::FeatureName;
}
}  // namespace pyuipc::core
