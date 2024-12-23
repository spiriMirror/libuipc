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

    virtual void      init(core::Scene& s) override;
    SanityCheckResult check() const;

    template <std::derived_from<core::ISanityChecker> SanityCheckerT>
    SanityCheckerT* find() const;

    std::string_view workspace() const noexcept;

  private:
    list<S<core::ISanityChecker>> m_entries;
    std::string                   m_workspace;
};
}  // namespace uipc::sanity_check

#include "details/sanity_checker_collection.inl"