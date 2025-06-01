#include <uipc/common/abort.h>
#include <cstdlib>
#include <cpptrace/cpptrace.hpp>
namespace uipc
{
void abort() noexcept
{
    auto raw_trace = cpptrace::generate_raw_trace(1 /*skip this `abort` function*/);
    raw_trace.resolve().print_with_snippets();  // better error messages
    std::abort();  // call std::abort to terminate the program
}
}  // namespace uipc
