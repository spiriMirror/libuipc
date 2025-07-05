#pragma once
#include <sim_system.h>
#include <contact_system/simplex_frictional_contact.h>
#include <collision_detection/global_trajectory_filter.h>
#include <collision_detection/simplex_trajectory_filter.h>
#include <collision_detection/vertex_half_plane_trajectory_filter.h>
#include <implicit_geometry/half_plane_vertex_reporter.h>
#include <contact_system/simplex_normal_contact.h>
#include <contact_system/simplex_frictional_contact.h>
#include <contact_system/vertex_half_plane_normal_contact.h>
#include <contact_system/vertex_half_plane_frictional_contact.h>

namespace uipc::backend::cuda
{
class ContactSystemExporter final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Exporter
    {
      public:
        std::function<void(geometry::Geometry&)> get_energy;
        std::function<void(geometry::Geometry&)> get_gradient;
        std::function<void(geometry::Geometry&)> get_hessian;
    };

  private:
    friend class ContactSystemFeatureOverrider;

    void do_build() override;

    void get_contact_gradient(geometry::Geometry& vert_grad);

    void get_contact_gradient(std::string_view    prim_type,
                              std::string_view    contact_type,
                              geometry::Geometry& prim_grad);

    void get_contact_hessian(geometry::Geometry& vert_hess);

    void get_contact_hessian(std::string_view    prim_type,
                             std::string_view    contact_type,
                             geometry::Geometry& prim_hess);

    void get_contact_primtives(std::string_view    prim_type,
                               std::string_view    contact_type,
                               geometry::Geometry& prims);

    vector<std::string> get_contact_primitive_types() const;

    vector<std::string> get_contact_types() const;

    SimSystemSlot<GlobalTrajectoryFilter> m_global_trajectory_filter;
    SimSystemSlot<GlobalContactManager>   m_global_contact_manager;

    SimSystemSlot<SimplexNormalContact>     m_simplex_normal_contact;
    SimSystemSlot<SimplexFrictionalContact> m_simplex_frictional_contact;
    SimSystemSlot<VertexHalfPlaneNormalContact> m_vertex_half_plane_normal_contact;
    SimSystemSlot<VertexHalfPlaneFrictionalContact> m_vertex_half_plane_frictional_contact;

    SimSystemSlot<SimplexTrajectoryFilter> m_simplex_trajectory_filter;
    SimSystemSlot<VertexHalfPlaneTrajectoryFilter> m_vertex_half_plane_trajectory_filter;
    SimSystemSlot<HalfPlaneVertexReporter> m_half_plane_vertex_reporter;

    unordered_map<std::string, Exporter> m_exporters;

    void add_exporter(std::string_view prim_type, std::string_view contact_type, Exporter&& exporter);
    Exporter* find_exporter(std::string_view prim_type, std::string_view contact_type);
};
}  // namespace uipc::backend::cuda