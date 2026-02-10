#pragma once
#include <string>
#include <filesystem>

namespace uipc::backend
{
class BackendPathTool
{
  public:
    using Path = std::filesystem::path;

    /**
     * @param workspace The workspace path given by the frontend
     */
    BackendPathTool(std::string_view workspace) noexcept;

    Path workspace() const noexcept;

    /**
     * @brief Return the file-local workspace path with the given file name.
     * 
     * Remove the backend source directory from the file path and append the relative path to the workspace.
     * 
     * Example:
     * "src/backends/cuda/MyFolder/MyFile.cu" -> workspace + "/cuda/MyFolder/MyFile.cu"
     * 
     * @param uipc_relative_source_file Must be UIPC_RELATIVE_SOURCE_FILE.
     * @param prefix
     */
    Path workspace(std::string_view uipc_relative_source_file, std::string_view prefix = "") const noexcept;

    static constexpr std::string_view backend_name() noexcept
    {
        return UIPC_BACKEND_NAME;
    }

    static constexpr std::string_view backend_source_dir() noexcept
    {
        return UIPC_BACKEND_DIR;
    }

  private:
    std::string m_workspace;
};
}  // namespace uipc::backend
