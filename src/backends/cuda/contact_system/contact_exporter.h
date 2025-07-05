#pragma once
#include <sim_system.h>

namespace uipc::backend::cuda
{
class ContactExporter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    std::string_view prim_type() const noexcept;

    class BuildInfo
    {
      public:
    };

  protected:
    virtual void             do_build(BuildInfo& info)      = 0;
    virtual std::string_view get_prim_type() const noexcept = 0;

    virtual void get_contact_energy(std::string_view    prim_type,
                                    geometry::Geometry& energy_geo) = 0;

    virtual void get_contact_gradient(std::string_view    prim_type,
                                      geometry::Geometry& vert_grad) = 0;

    virtual void get_contact_hessian(std::string_view    prim_type,
                                     geometry::Geometry& vert_hess) = 0;

  private:
    virtual void do_build() override final;

    friend class ContactExporterManager;
    void contact_energy(std::string_view prim_type, geometry::Geometry& energy_geo);
    void contact_gradient(std::string_view prim_type, geometry::Geometry& vert_grad);
    void contact_hessian(std::string_view prim_type, geometry::Geometry& vert_hess);
};
}  // namespace uipc::backend::cuda
