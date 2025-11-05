#include <uipc/common/config.h>
#include <uipc/common/log.h>
#include <backends/common/backend_path_tool.h>

namespace uipc::backend
{
BackendPathTool::BackendPathTool(std::string_view workspace) noexcept
    : m_workspace(workspace)
{
}
auto BackendPathTool::workspace() const noexcept -> Path
{
    return m_workspace;
}

auto BackendPathTool::relative(std::string_view _file_,
                               bool with_backend_name) const noexcept -> Path
{
    namespace fs = std::filesystem;

    Path file_path{_file_};

    Path backends_dir = backend_source_dir();

    if constexpr(uipc::RUNTIME_CHECK)
    {
        auto [left, right] = std::mismatch(file_path.begin(),
                                           file_path.end(),
                                           backends_dir.begin(),
                                           backends_dir.end());

        UIPC_ASSERT(right == backends_dir.end(),
                    "The file path {} is not in the backends directory {}",
                    file_path.string(),
                    backends_dir.string());

        auto rel   = fs::relative(file_path, backends_dir);
        Path first = *rel.begin();
        UIPC_ASSERT(first == "common" || first == backend_name(),
                    "The first folder in the relative path [{}] is not the backend name [{}] or [common], why can it happen?",
                    first.string(),
                    backend_name());
    }

    if(with_backend_name)
        backends_dir /= backend_name();

    return fs::relative(file_path, backends_dir);
}

auto BackendPathTool::workspace(std::string_view _file_,
                                std::string_view prefix) const noexcept -> Path
{
    namespace fs = std::filesystem;

    Path file_path{_file_};
    Path out_path = fs::absolute(Path{m_workspace} / prefix);

    Path file_output_path = out_path / relative(_file_, false);

    // create all the intermediate directories if they don't exist
    if(!fs::exists(file_output_path))
        fs::create_directories(file_output_path);

    return (file_output_path / "");
}

std::string_view BackendPathTool::backend_name() noexcept
{
    return UIPC_BACKEND_NAME;
}

std::string_view BackendPathTool::backend_source_dir() noexcept
{
    return UIPC_BACKEND_DIR;
}
}  // namespace uipc::backend
