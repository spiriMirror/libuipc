#include <uipc/common/abort.h>
#include <cstdlib>
#include <cpptrace/cpptrace.hpp>
namespace uipc
{
void abort() noexcept
{
    auto raw_trace = cpptrace::generate_raw_trace(1);
    raw_trace.resolve().print_with_snippets();
    std::abort();
}
}  // namespace uipc
