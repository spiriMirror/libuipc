#pragma once
#include <uipc/common/smart_pointer.h>
#include <uipc/common/list.h>
#include <uipc/core/sanity_checker.h>

namespace uipc::sanity_check
{
using uipc::core::SanityCheckResult;

class SanityCheckerCollection : public core::ISanityCheckerCollection
{
  public:
    SanityCheckerCollection(std::string_view workspace) noexcept;
    ~SanityCheckerCollection();

    virtual void build(core::Scene& s) override;
    virtual SanityCheckResult check(core::SanityCheckMessageCollection& msgs) const override;

    template <std::derived_from<core::ISanityChecker> SanityCheckerT>
    SanityCheckerT* find() const;

    template <std::derived_from<core::ISanityChecker> SanityCheckerT>
    SanityCheckerT& require() const;

    std::string_view workspace() const noexcept;

  private:
    list<S<core::ISanityChecker>> m_entries;
    list<core::ISanityChecker*>   m_valid_entries;
    std::string                   m_workspace;
};
}  // namespace uipc::sanity_check

#include "details/sanity_checker_collection.inl"