#include <uipc/sanity_check/sanity_checker.h>

namespace uipc::core
{
class NoneCheck final : public SanityChecker
{
  public:
    using SanityChecker::SanityChecker;

  protected:
    virtual U64 get_id() const noexcept override { return 0; }

    virtual SanityCheckResult do_check(backend::SceneVisitor&) noexcept override
    {
        return SanityCheckResult::Success;
    };
};

REGISTER_SANITY_CHECKER(NoneCheck);
}  // namespace uipc::core
