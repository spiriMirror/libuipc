#pragma once
#include <uipc/sanity_check/i_sanity_checker.h>
#include <uipc/sanity_check/sanity_checker_auto_register.h>
#include <uipc/core/scene.h>

namespace uipc::backend
{
class SceneVisitor;
}

namespace uipc::core
{
class UIPC_CORE_API SanityChecker : public ISanityChecker
{
  public:
    explicit SanityChecker(Scene& s) noexcept;

    virtual U64               get_id() const noexcept = 0;
    virtual SanityCheckResult do_check() noexcept;

  protected:
    virtual SanityCheckResult do_check(backend::SceneVisitor& scene) noexcept = 0;

    Scene& m_scene;
};
}  // namespace uipc::core

#define REGISTER_SANITY_CHECKER(SanityChecker)                                                 \
    namespace auto_register                                                                    \
    {                                                                                          \
        static ::uipc::core::SanityCheckerAutoRegister SanityCheckerAutoRegister##__COUNTER__{ \
            ::uipc::core::detail::register_sanity_checker_creator<SanityChecker>()};           \
    }

// End of file