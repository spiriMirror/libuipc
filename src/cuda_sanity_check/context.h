#pragma once
#include <cuda_sanity_checker.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::core::internal
{
class Scene;
}

namespace uipc::cuda_sanity_check
{
using uipc::core::SanityCheckResult;

class Context final : public CudaSanityChecker
{
  public:
    explicit Context(CudaSanityCheckerCollection& c, core::internal::Scene& s) noexcept;
    virtual ~Context() override;

    const geometry::SimplicialComplex& scene_simplicial_surface() const noexcept;
    const core::ContactTabular&  contact_tabular() const noexcept;
    const core::SubsceneTabular& subscene_tabular() const noexcept;

  private:
    friend class CudaSanityCheckerCollection;
    void prepare();
    void destroy();

    virtual U64 get_id() const noexcept override;

    virtual SanityCheckResult do_check(backend::SceneVisitor&,
                                       backend::SanityCheckMessageVisitor&) override;

    class Impl;
    U<Impl> m_impl;
};
}  // namespace uipc::cuda_sanity_check
