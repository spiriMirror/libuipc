#pragma once
#include <string>

namespace uipc
{
class AssetDir
{
  public:
    static std::string_view asset_path();
    static std::string_view scene_path();
    static std::string_view sim_data_path();
    static std::string_view tetmesh_path();
    static std::string_view trimesh_path();
    static std::string_view output_path();
};
}  // namespace uipc
