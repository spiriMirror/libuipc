#pragma once
#include <uipc/core/feature.h>
#include <uipc/backend/buffer_view.h>
#include <uipc/geometry/geometry.h>

namespace uipc::core
{
class UIPC_CORE_API ContactSystemFeatureOverrider
{
  public:
    virtual void get_contact_gradient(geometry::Geometry& vert_grad) = 0;
    virtual void get_contact_hessian(geometry::Geometry& vert_hess)  = 0;
    virtual void get_contact_primtives(std::string_view    prim_type,
                                       geometry::Geometry& prims)    = 0;
    virtual vector<std::string> get_contact_primitive_types() const  = 0;
};

class UIPC_CORE_API ContactSystemFeature final : public Feature
{
  public:
    constexpr static std::string_view FeatureName = "contact_system";

    ContactSystemFeature(S<ContactSystemFeatureOverrider> overrider);

    /**
     * @brief Get the contact gradient for the given geometry.
     * 
     * @param[out] vert_grad:Geometry The gradient of the contact forces with respect to the vertices.
     */
    void contact_gradient(geometry::Geometry& vert_grad);

    /**
     * @brief Get the contact hessian for the given geometry.
     * 
     * @param[out] vert_hess:Geometry The hessian of the contact forces with respect to the vertices.
     */

    void contact_hessian(geometry::Geometry& vert_hess);

    /**
     * @brief Get the contact primitives of the given type.
     * 
     * @param[in] prim_type:std::string_view The type of the contact primitives to get.
     * @param[out] prims:Geometry The contact primitives of the given type.
     */
    void contact_primitives(std::string_view prim_type, geometry::Geometry& prims);

    /**
     * @brief Get the types of contact primitives available.
     * 
     * @return vector<std::string> The types of contact primitives available.
     */
    vector<std::string> contact_primitive_types() const;

  private:
    virtual std::string_view         get_name() const final override;
    S<ContactSystemFeatureOverrider> m_impl;
};
}  // namespace uipc::core