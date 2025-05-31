#include <uipc/common/demangle.h>
#include <cpptrace/cpptrace.hpp>

namespace uipc
{
std::string demangle(const std::string& mangled_name)
{
    return cpptrace::demangle(mangled_name);
}
}  // namespace uipc
