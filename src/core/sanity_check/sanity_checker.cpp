#include <uipc/sanity_check/sanity_checker.h>
#include <uipc/backend/visitors/scene_visitor.h>

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
SanityChecker::SanityChecker(Scene& s) noexcept
    : m_scene(s)
{
}

SanityCheckResult SanityChecker::do_check() noexcept
{
    backend::SceneVisitor sv{m_scene};
    return do_check(sv);
}
}  // namespace uipc::core
