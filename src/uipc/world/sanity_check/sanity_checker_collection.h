#pragma once
#include <uipc/common/smart_pointer.h>
#include <uipc/common/list.h>
#include <uipc/world/sanity_check/i_sanity_checker.h>

namespace uipc::world
{
class Scene;
class SanityCheckerCollection
{
  public:
    SanityCheckerCollection() noexcept = default;
    void              init(const Scene& s);
    SanityCheckResult check() const;

  private:
    list<U<ISanityChecker>> entries;
};
}  // namespace uipc::world
