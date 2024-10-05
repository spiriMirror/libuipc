#pragma once
#include <uipc/sanity_check/i_sanity_checker.h>
#include <uipc/sanity_check/sanity_checker_auto_register.h>
#include <uipc/core/scene.h>

namespace uipc::core
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
}  // namespace uipc::core

#define REGISTER_SANITY_CHECKER(SanityChecker)                                                 \
    namespace auto_register                                                                    \
    {                                                                                          \
        static ::uipc::core::SanityCheckerAutoRegister SanityCheckerAutoRegister##__COUNTER__{ \
            ::uipc::core::detail::register_sanity_checker_creator<SanityChecker>()};           \
    }

// End of file