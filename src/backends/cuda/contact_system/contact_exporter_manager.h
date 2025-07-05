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
#include <contact_system/contact_line_search_reporter.h>

namespace uipc::backend::cuda
{
class ContactExporter;

class ContactExporterManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  private:
    friend class ContactSystemFeatureOverrider;

    void do_build() override;
    void init();

    void compute_contact();

    void get_contact_gradient(geometry::Geometry& vert_grad);
    void get_contact_hessian(geometry::Geometry& vert_hess);


    void get_contact_energy(std::string_view prim_type, geometry::Geometry& prim_energy);
    void get_contact_gradient(std::string_view prim_type, geometry::Geometry& prim_grad);
    void get_contact_hessian(std::string_view prim_type, geometry::Geometry& prim_hess);

    vector<std::string> get_contact_primitive_types() const;

    SimSystemSlot<GlobalTrajectoryFilter>    m_global_trajectory_filter;
    SimSystemSlot<GlobalContactManager>      m_global_contact_manager;
    SimSystemSlot<ContactLineSearchReporter> m_contact_line_search_reporter;

    unordered_map<std::string, ContactExporter*> m_exporter_map;
    SimSystemSlotCollection<ContactExporter>     m_contact_exporters;
    vector<std::string>                          m_contact_prim_types;

    friend class ContactExporter;
    void add_exporter(ContactExporter* exporter);

    ContactExporter* find_exporter(std::string_view prim_type) const;
    void _create_prim_type_on_geo(std::string_view prim_type, geometry::Geometry& geo);
};
}  // namespace uipc::backend::cuda