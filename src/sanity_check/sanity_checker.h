#pragma once
#include <uipc/common/macro.h>
#include <sanity_checker_exception.h>
#include <uipc/core/sanity_checker.h>
#include <sanity_checker_auto_register.h>
#include <uipc/core/scene.h>
#include <uipc/backend/visitors/sanity_check_message_visitor.h>

namespace uipc::backend
{
class SceneVisitor;
}

namespace uipc::sanity_check
{
using uipc::core::SanityCheckResult;

class SanityCheckerCollection;

class SanityChecker : public core::ISanityChecker
{
  public:
    SanityChecker(SanityCheckerCollection& c, core::Scene& s) noexcept;

    std::string_view workspace() const noexcept;
    std::string      this_output_path() const noexcept;

  protected:
    virtual std::string get_name() const noexcept override;

    virtual void build(backend::SceneVisitor& scene);

    template <std::derived_from<core::ISanityChecker> SanityCheckerT>
    SanityCheckerT* find() const;

    template <std::derived_from<core::ISanityChecker> SanityCheckerT>
    SanityCheckerT& require() const;

    virtual SanityCheckResult do_check(backend::SceneVisitor& scene,
                                       backend::SanityCheckMessageVisitor& msg) = 0;

    core::Scene::CObjects objects() const noexcept;

  private:
    virtual void              build() override final;
    virtual SanityCheckResult do_check(core::SanityCheckMessage& msg) override;
    SanityCheckerCollection&  m_collection;
    core::Scene&              m_scene;
};
}  // namespace uipc::sanity_check

#define REGISTER_SANITY_CHECKER(SanityChecker)                                                               \
    namespace auto_register                                                                                  \
    {                                                                                                        \
        static ::uipc::sanity_check::SanityCheckerAutoRegister UIPC_NAME_WITH_ID(SanityCheckerAutoRegister){ \
            ::uipc::sanity_check::detail::register_sanity_checker_creator<SanityChecker>()};                 \
    }

#include "details/sanity_checker.inl"
