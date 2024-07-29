#pragma once
#include <string>
#include <filesystem>

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
    static std::string      output_path(const char* _file_);
    static std::string      folder(const std::string& _file_);
};
}  // namespace uipc
