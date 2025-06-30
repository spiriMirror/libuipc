#pragma once
#include <sim_system.h>
#include <contact_system/simplex_frictional_contact.h>
#include <collision_detection/global_trajectory_filter.h>
#include <collision_detection/simplex_trajectory_filter.h>
#include <collision_detection/vertex_half_plane_trajectory_filter.h>
#include <implicit_geometry/half_plane_vertex_reporter.h>

namespace uipc::backend::cuda
{
class ContactSystemExporter final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  private:
    friend class ContactSystemFeatureOverrider;

    void do_build() override;

    void get_contact_gradient(geometry::Geometry& vert_grad);

    void get_contact_hessian(geometry::Geometry& vert_hess);

    void get_contact_primtives(std::string_view prim_type, geometry::Geometry& prims);

    vector<std::string> get_contact_primitive_types() const;

    SimSystemSlot<GlobalTrajectoryFilter>  m_global_trajectory_filter;
    SimSystemSlot<GlobalContactManager>    m_global_contact_manager;
    SimSystemSlot<SimplexTrajectoryFilter> m_simplex_trajectory_filter;
    SimSystemSlot<VertexHalfPlaneTrajectoryFilter> m_vertex_half_plane_trajectory_filter;
    SimSystemSlot<HalfPlaneVertexReporter> m_half_plane_vertex_reporter;
};
}  // namespace uipc::backend::cuda