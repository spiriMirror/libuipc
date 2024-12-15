#pragma once
#include <uipc/common/smart_pointer.h>
#include <uipc/common/list.h>
#include <uipc/sanity_check/i_sanity_checker.h>

namespace uipc::core
{
class Scene;
class SanityCheckerCollection
{
  public:
    SanityCheckerCollection() noexcept = default;
    void              init(Scene& s);
    SanityCheckResult check() const;

  private:
    list<U<ISanityChecker>> entries;
};
}  // namespace uipc::core
