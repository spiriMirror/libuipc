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

    static constexpr std::string_view backend_name() noexcept
    {
        return UIPC_BACKEND_NAME;
    }

    static constexpr std::string_view backend_source_dir() noexcept
    {
        // Return the backend source directory, removing "common/" if present at the end
        constexpr auto full_path  = std::string_view(__FILE__, std::string_view(__FILE__).find_last_of("/\\"));
        constexpr auto last_sep   = full_path.find_last_of("/\\");
        constexpr auto dir_path   = full_path.substr(0, last_sep);
        // If dir_path ends with "/common" or "\\common", remove it
        constexpr auto common_pos = dir_path.rfind("/common");
        if constexpr (common_pos != std::string_view::npos && common_pos == dir_path.size() - 7)
            return dir_path.substr(0, common_pos);
        else
            return dir_path;
    }

  private:
    std::string m_workspace;
};
}  // namespace uipc::backend
