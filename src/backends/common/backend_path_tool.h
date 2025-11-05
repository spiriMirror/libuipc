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
     *@brief Return the relative path of the given file to the backend source directory
     * 
     * Remove the backend source directory from the file path.
     * 
     * 
     * Example:
     * - with_backend_name = false:
     *
     *    "xxx/libuipc/src/backends/cuda/MyFolder/MyFile.cu" -> "MyFolder/MyFile.cu"
     * 
     * - with_backend_name = true:
     *
     *    "xxx/libuipc/src/backends/cuda/MyFolder/MyFile.cu" -> "cuda/MyFolder/MyFile.cu"
     * 
     * @param _file_ Must be __FILE__.
     * @param with_backend_name If true, the backend name is included in the relative path.
     */
    Path relative(std::string_view _file_, bool with_backend_name = false) const noexcept;

    /**
     * @brief Return the file-local workspace path with the given file name.
     * 
     * Remove the backend source directory from the file path and append the relative path to the workspace,
     * where the backend name is removed.
     * 
     * Example:
     * "xxx/libuipc/src/backends/cuda/MyFolder/MyFile.cu" -> workspace + "/MyFolder/MyFile.cu"
     * 
     * @param _file_ Must be __FILE__.
     * @param prefix
     */
    Path workspace(std::string_view _file_, std::string_view prefix = "") const noexcept;

    static std::string_view backend_name() noexcept;

    static std::string_view backend_source_dir() noexcept;

  private:
    std::string m_workspace;
};
}  // namespace uipc::backend
