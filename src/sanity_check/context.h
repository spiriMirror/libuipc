#include <sanity_checker.h>
#include <uipc/geometry/simplicial_complex.h>

namespace uipc::sanity_check
{
using uipc::core::SanityCheckResult;

class Context final : public SanityChecker
{
  public:
    explicit Context(SanityCheckerCollection& c, core::Scene& s) noexcept;
    virtual ~Context() override;

    const geometry::SimplicialComplex& scene_simplicial_surface() const noexcept;

  private:
    virtual void init() noexcept override;
    virtual void deinit() noexcept override;
    virtual U64  get_id() const noexcept override;

    virtual SanityCheckResult do_check(backend::SceneVisitor&) noexcept override;
    class Impl;
    U<Impl> m_impl;
};
}  // namespace uipc::sanity_check
