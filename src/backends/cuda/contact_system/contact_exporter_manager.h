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
class ContactExporter;

class ContactExporterManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  private:
    friend class ContactSystemFeatureOverrider;

    void do_build() override;
    void init();

    void get_contact_energy(std::string_view prim_type, geometry::Geometry& prim_energy);
    void get_contact_gradient(std::string_view prim_type, geometry::Geometry& prim_grad);
    void get_contact_hessian(std::string_view prim_type, geometry::Geometry& prim_hess);

    vector<std::string> get_contact_primitive_types() const;

    unordered_map<std::string, ContactExporter*> m_exporter_map;
    SimSystemSlotCollection<ContactExporter>     m_contact_exporters;
    vector<std::string>                          m_contact_prim_types;

    friend class ContactExporter;
    void add_exporter(ContactExporter* exporter);

    ContactExporter* find_exporter(std::string_view prim_type) const;
    void _create_prim_type_on_geo(std::string_view prim_type, geometry::Geometry& geo);
};
}  // namespace uipc::backend::cuda