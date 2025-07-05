#pragma once
#include <uipc/core/feature.h>
#include <uipc/backend/buffer_view.h>
#include <uipc/geometry/geometry.h>
#include <uipc/constitution/constitution.h>

namespace uipc::core
{
class UIPC_CORE_API ContactSystemFeatureOverrider
{
  public:
    virtual void do_compute_contact() = 0;

    virtual void get_contact_gradient(geometry::Geometry& vert_grad) = 0;

    virtual void get_contact_hessian(geometry::Geometry& vert_hess) = 0;

    virtual void get_contact_gradient(std::string_view    prim_type,
                                      geometry::Geometry& vert_grad) = 0;

    virtual void get_contact_hessian(std::string_view    prim_type,
                                     geometry::Geometry& vert_hess) = 0;

    virtual void get_contact_energy(std::string_view    prim_type,
                                    geometry::Geometry& prims) = 0;

    virtual vector<std::string> get_contact_primitive_types() const = 0;
};

class UIPC_CORE_API ContactSystemFeature final : public Feature
{
  public:
    constexpr static std::string_view FeatureName = "contact_system";

    ContactSystemFeature(S<ContactSystemFeatureOverrider> overrider);

    void compute_contact();

    /**
     * @brief return all contact gradients for the vertices in the system.
     * 
     * @param vert_grad
     */
    void contact_gradient(geometry::Geometry& vert_grad);

    /**
     * @brief return all contact hessians for the vertices in the system.
     * 
     * @param vert_hess
     */
    void contact_hessian(geometry::Geometry& vert_hess);

    void contact_energy(std::string_view prim_type, geometry::Geometry& energy);

    void contact_gradient(std::string_view prim_type, geometry::Geometry& vert_grad);

    void contact_hessian(std::string_view prim_type, geometry::Geometry& vert_hess);

    void contact_energy(const constitution::IConstitution& c, geometry::Geometry& energy);

    void contact_gradient(const constitution::IConstitution& c, geometry::Geometry& vert_grad);

    void contact_hessian(const constitution::IConstitution& c, geometry::Geometry& vert_hess);

    vector<std::string> contact_primitive_types() const;

  private:
    virtual std::string_view         get_name() const final override;
    S<ContactSystemFeatureOverrider> m_impl;
};
}  // namespace uipc::core