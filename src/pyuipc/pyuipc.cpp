#include <pyuipc/pyuipc.h>
#include <uipc/common/config.h>
#include <fmt/format.h>

namespace pyuipc
{
std::string remove_project_prefix(std::string_view path)
{
    // find last "pyuipc" in path
    auto pos = path.rfind("pyuipc");
    if(pos == std::string::npos)
    {
        return std::string(path);
    }
    return std::string(path.substr(pos));
}

std::string process_project_prefix(std::string_view path)
{
    if constexpr(uipc::RUNTIME_CHECK)
        return remove_project_prefix(path);
    return std::string(path);
}

std::string detail::string_with_source_location(std::string_view msg,
                                                std::string_view path,
                                                std::size_t      line)
{
    return fmt::format("{} [{}({})]", msg, process_project_prefix(path), line);
}
}  // namespace pyuipc
