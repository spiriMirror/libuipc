#pragma once
#include <sim_system.h>
#include <uipc/core/distance_diagnoser_feature.h>

namespace uipc::backend::cuda
{
class DistanceDiagnoser;

class DistanceDiagnoserFeatureOverrider final : public core::DistanceDiagnoserFeatureOverrider
{
  public:
    DistanceDiagnoserFeatureOverrider(DistanceDiagnoser* diagnoser);

  private:
    void do_compute_point_triangle_distance(
        geometry::Geometry&                R,
        const geometry::SimplicialComplex& points,
        const geometry::SimplicialComplex& triangles) override;

    void do_compute_edge_edge_distance(
        geometry::Geometry&                R,
        const geometry::SimplicialComplex& edges_a,
        const geometry::SimplicialComplex& edges_b,
        const geometry::SimplicialComplex& rest_edges_a,
        const geometry::SimplicialComplex& rest_edges_b) override;

    void do_compute_point_edge_distance(
        geometry::Geometry&                R,
        const geometry::SimplicialComplex& points,
        const geometry::SimplicialComplex& edges) override;

    void do_compute_point_point_distance(
        geometry::Geometry&                R,
        const geometry::SimplicialComplex& points_a,
        const geometry::SimplicialComplex& points_b) override;

    SimSystemSlot<DistanceDiagnoser> m_diagnoser;
};

class DistanceDiagnoser final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    void compute_point_triangle_distance(
        geometry::Geometry&                R,
        const geometry::SimplicialComplex& points,
        const geometry::SimplicialComplex& triangles);

    void compute_edge_edge_distance(
        geometry::Geometry&                R,
        const geometry::SimplicialComplex& edges_a,
        const geometry::SimplicialComplex& edges_b,
        const geometry::SimplicialComplex& rest_edges_a,
        const geometry::SimplicialComplex& rest_edges_b);

    void compute_point_edge_distance(
        geometry::Geometry&                R,
        const geometry::SimplicialComplex& points,
        const geometry::SimplicialComplex& edges);

    void compute_point_point_distance(
        geometry::Geometry&                R,
        const geometry::SimplicialComplex& points_a,
        const geometry::SimplicialComplex& points_b);

  private:
    friend class DistanceDiagnoserFeatureOverrider;

    void do_build() override;
};
}  // namespace uipc::backend::cuda
