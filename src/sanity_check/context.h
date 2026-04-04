#pragma once
#include <sanity_checker.h>
#include <uipc/core/i_sanity_checker.h>
#include <uipc/core/object.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/geometry/geometry_slot.h>

namespace uipc::core::internal
{
class Scene;
}

namespace uipc::sanity_check
{
using uipc::core::SanityCheckResult;

class Context final : public SanityChecker, public core::ISanityCheckContext
{
  public:
    explicit Context(SanityCheckerCollection& c, core::internal::Scene& s) noexcept;
    virtual ~Context() override;

    void prepare();
    void destroy();

    const geometry::SimplicialComplex&       scene_simplicial_surface() const noexcept override;
    const core::ContactTabular&              contact_tabular() const noexcept override;
    const core::SubsceneTabular&             subscene_tabular() const noexcept override;
    std::string_view                         workspace() const override;
    S<const core::Object>                    find_object(IndexT id) const override;
    span<S<geometry::GeometrySlot>>          geometries() const override;
    S<geometry::GeometrySlot>                find_geometry(IndexT id) const override;
    const geometry::AttributeCollection&     config() const override;

  private:
    virtual U64 get_id() const noexcept override;

    virtual SanityCheckResult do_check(backend::SceneVisitor&,
                                       backend::SanityCheckMessageVisitor&) override;

    class Impl;
    U<Impl> m_impl;
};
}  // namespace uipc::sanity_check
