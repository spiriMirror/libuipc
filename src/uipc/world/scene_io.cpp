#include <uipc/world/scene_io.h>
#include <filesystem>

namespace uipc::world
{
namespace fs = std::filesystem;

SceneIO::SceneIO(Scene& scene)
    : m_scene(scene)
{
}

void SceneIO::write_surface(std::string_view filename)
{
    fs::path path = filename;
    auto     ext  = path.extension();
    if(ext == ".obj")
    {
        write_surface_as_obj(filename);
    }
    else
    {
        throw SceneIOError(
            fmt::format("Unsupported file format when writing {}.", filename));
    }
}

void SceneIO::write_surface_as_obj(std::string_view filename)
{
    UIPC_ASSERT(false, "Not implemented yet.");
}
}  // namespace uipc::world
