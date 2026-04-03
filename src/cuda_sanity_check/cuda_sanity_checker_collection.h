#pragma once
#include <uipc/common/smart_pointer.h>
#include <uipc/common/list.h>
#include <uipc/core/i_sanity_checker.h>

namespace uipc::cuda_sanity_check
{
using uipc::core::SanityCheckResult;

class CudaSanityCheckerCollection : public core::ISanityCheckerCollection
{
  public:
    CudaSanityCheckerCollection(std::string_view workspace) noexcept;
    ~CudaSanityCheckerCollection();

    virtual void              build(core::internal::Scene& s) override;
    virtual SanityCheckResult check(core::SanityCheckMessageCollection& msgs) const override;

    template <std::derived_from<core::ISanityChecker> CudaSanityCheckerT>
    CudaSanityCheckerT* find() const;

    template <std::derived_from<core::ISanityChecker> CudaSanityCheckerT>
    CudaSanityCheckerT& require() const;

    std::string_view workspace() const noexcept;

  private:
    list<S<core::ISanityChecker>> m_entries;
    list<core::ISanityChecker*>   m_valid_entries;
    std::string                   m_workspace;
};
}  // namespace uipc::cuda_sanity_check

#include "details/cuda_sanity_checker_collection.inl"
