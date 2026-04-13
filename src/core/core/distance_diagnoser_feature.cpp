#include <uipc/core/distance_diagnoser_feature.h>

namespace uipc::core
{
DistanceDiagnoserFeature::DistanceDiagnoserFeature(S<DistanceDiagnoserFeatureOverrider> overrider)
    : m_impl(std::move(overrider))
{
}

void DistanceDiagnoserFeature::compute_point_triangle_distance(
    geometry::Geometry&                R,
    const geometry::SimplicialComplex& points,
    const geometry::SimplicialComplex& triangles)
{
    m_impl->do_compute_point_triangle_distance(R, points, triangles);
}

void DistanceDiagnoserFeature::compute_edge_edge_distance(
    geometry::Geometry&                R,
    const geometry::SimplicialComplex& edges_a,
    const geometry::SimplicialComplex& edges_b,
    const geometry::SimplicialComplex& rest_edges_a,
    const geometry::SimplicialComplex& rest_edges_b)
{
    m_impl->do_compute_edge_edge_distance(R, edges_a, edges_b, rest_edges_a, rest_edges_b);
}

void DistanceDiagnoserFeature::compute_point_edge_distance(
    geometry::Geometry&                R,
    const geometry::SimplicialComplex& points,
    const geometry::SimplicialComplex& edges)
{
    m_impl->do_compute_point_edge_distance(R, points, edges);
}

void DistanceDiagnoserFeature::compute_point_point_distance(
    geometry::Geometry&                R,
    const geometry::SimplicialComplex& points_a,
    const geometry::SimplicialComplex& points_b)
{
    m_impl->do_compute_point_point_distance(R, points_a, points_b);
}

std::string_view DistanceDiagnoserFeature::get_name() const
{
    return FeatureName;
}
}  // namespace uipc::core
