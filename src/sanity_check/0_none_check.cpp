#include <uipc/sanity_check/sanity_checker.h>

#ifdef _MSC_VER
namespace uipc::core::detail
{
class __declspec(dllexport) DLLExport
{
    // dummy class to let the compiler generate the export table
};
}  // namespace uipc::core::detail
#endif

namespace uipc::core
{
class NoneCheck final : public SanityChecker
{
  public:
    using SanityChecker::SanityChecker;

  protected:
    U64 get_id() const noexcept override { return 0; }

    SanityCheckResult do_check(const SceneImpl&) noexcept override
    {
        return SanityCheckResult::Success;
    };
};

REGISTER_SANITY_CHECKER(NoneCheck);
}  // namespace uipc::core
