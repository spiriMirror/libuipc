#include <app/asset_dir.h>
#include <uipc/common/log.h>

namespace uipc
{
namespace fs = std::filesystem;
std::string_view AssetDir::asset_path()
{
    return UIPC_ASSET_PATH;
}

std::string_view AssetDir::scene_path()
{
    return UIPC_ASSET_PATH "scenes/";
}

std::string_view AssetDir::sim_data_path()
{
    return UIPC_ASSET_PATH "sim_data/";
}

std::string_view AssetDir::tetmesh_path()
{
    return UIPC_ASSET_PATH "sim_data/tetmesh/";
}

std::string_view AssetDir::trimesh_path()
{
    return UIPC_ASSET_PATH "sim_data/trimesh/";
}

std::string_view AssetDir::linemesh_path()
{
    return UIPC_ASSET_PATH "sim_data/linemesh/";
}

std::string_view AssetDir::output_path()
{
    return UIPC_OUTPUT_PATH;
}

std::string AssetDir::output_path(const char* uipc_relative_source_file)                                                                                                                                                                                                                                                                                                                            
{
    fs::path _output_path{UIPC_OUTPUT_PATH};
    fs::path file_path{uipc_relative_source_file};
    fs::path     project_dir{UIPC_PROJECT_DIR};
    auto     apps_path    = project_dir / "apps";
    UIPC_ASSERT(file_path.is_relative(),
                "UIPC_RELATIVE_SOURCE_FILE must be relative, got {}",
                file_path.string());
    file_path = project_dir / file_path;
    // get the relative path according to the apps path
    auto file_relative_to_apps = fs::relative(file_path, apps_path);
    auto file_output_path      = _output_path / file_relative_to_apps;

    // create all the intermediate directories if they don't exist
    if(!fs::exists(file_output_path))
        fs::create_directories(file_output_path);
    file_output_path = fs::canonical(file_output_path);
    return (file_output_path / "").string();
}

std::string AssetDir::folder(const std::string& _file_)
{
    fs::path file_path{_file_};
    return (file_path.parent_path() / "").string();
}

}  // namespace uipc

