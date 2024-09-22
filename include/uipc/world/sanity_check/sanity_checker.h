#pragma once
#include <uipc/world/sanity_check/i_sanity_checker.h>
#include <uipc/world/sanity_check/sanity_checker_auto_register.h>
#include <uipc/world/scene.h>

namespace uipc::world
{
class UIPC_CORE_API SanityChecker : public ISanityChecker
{
  public:
    explicit SanityChecker(const Scene& s) noexcept;

    virtual U64               get_id() const noexcept = 0;
    virtual SanityCheckResult do_check() noexcept;

  protected:
    using SceneImpl = Scene::Impl;

    virtual SanityCheckResult do_check(const SceneImpl& scene) noexcept = 0;

    const Scene& m_scene;
};
}  // namespace uipc::world

#define REGISTER_SANITY_CHECKER(SanityChecker)                                                  \
    namespace auto_register                                                                     \
    {                                                                                           \
        static ::uipc::world::SanityCheckerAutoRegister SanityCheckerAutoRegister##__COUNTER__{ \
            ::uipc::world::detail::register_sanity_checker_creator<SanityChecker>()};           \
    }

// End of file