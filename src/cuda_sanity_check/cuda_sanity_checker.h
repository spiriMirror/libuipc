#pragma once
#include <uipc/common/macro.h>
#include <cuda_sanity_checker_exception.h>
#include <uipc/core/i_sanity_checker.h>
#include <cuda_sanity_checker_auto_register.h>
#include <uipc/core/scene.h>
#include <uipc/backend/visitors/sanity_check_message_visitor.h>

namespace uipc::backend
{
class SceneVisitor;
}

namespace uipc::cuda_sanity_check
{
using uipc::core::SanityCheckResult;

class CudaSanityCheckerCollection;

class CudaSanityChecker : public core::ISanityChecker
{
  public:
    CudaSanityChecker(CudaSanityCheckerCollection& c, core::internal::Scene& s) noexcept;

    std::string_view workspace() const noexcept;
    std::string      this_output_path() const noexcept;

  protected:
    virtual std::string get_name() const noexcept override;

    virtual void build(backend::SceneVisitor& scene);

    template <std::derived_from<core::ISanityChecker> CudaSanityCheckerT>
    CudaSanityCheckerT* find() const;

    template <std::derived_from<core::ISanityChecker> CudaSanityCheckerT>
    CudaSanityCheckerT& require() const;

    virtual SanityCheckResult do_check(backend::SceneVisitor& scene,
                                       backend::SanityCheckMessageVisitor& msg) = 0;

    core::Scene::CObjects objects() const noexcept;

  private:
    virtual void              build() override final;
    virtual SanityCheckResult do_check(core::SanityCheckMessage& msg) override;
    CudaSanityCheckerCollection& m_collection;
    // this scene shares the internal::Scene with the user's one
    S<core::Scene> m_scene;
};
}  // namespace uipc::cuda_sanity_check

#define REGISTER_CUDA_SANITY_CHECKER(CudaSanityChecker)                                                                   \
    namespace auto_register                                                                                               \
    {                                                                                                                     \
        static ::uipc::cuda_sanity_check::CudaSanityCheckerAutoRegister UIPC_NAME_WITH_ID(CudaSanityCheckerAutoRegister){ \
            ::uipc::cuda_sanity_check::detail::register_cuda_sanity_checker_creator<CudaSanityChecker>()};                \
    }

#include "details/cuda_sanity_checker.inl"
