#include <app/asset_dir.h>

namespace uipc
{
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
std::string_view AssetDir::output_path()
{
    return UIPC_OUTPUT_PATH;
}
}  // namespace uipc
