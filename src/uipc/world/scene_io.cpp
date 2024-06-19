#include <algorithm>
#include <filesystem>
#include <fmt/printf.h>
#include <fstream>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/builtin/geometry_type.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/geometry/simplicial_complex_slot.h>
#include <uipc/geometry/utils/extract_surface.h>
#include <uipc/geometry/utils/merge.h>
#include <uipc/world/scene_io.h>
#include <uipc/geometry/utils/io.h>

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
        write_surface_obj(filename);
    }
    else
    {
        throw SceneIOError(fmt::format("Unsupported file format when writing {}.", filename));
    }
}

void SceneIO::write_surface_obj(std::string_view filename)
{
    using namespace uipc::geometry;

    auto scene = backend::SceneVisitor{m_scene};
    auto geos  = scene.geometries();

    // 1) find all simplicial complex with surface
    vector<const SimplicialComplex*> simplicial_complex_has_surf;
    simplicial_complex_has_surf.reserve(geos.size());

    for(auto& geo : geos)
    {
        if(geo->geometry().type() == builtin::SimplicialComplex)
        {
            auto simplicial_complex = dynamic_cast<SimplicialComplex*>(&geo->geometry());

            UIPC_ASSERT(simplicial_complex, "type mismatch, why can it happen?");

            if(simplicial_complex->triangles().find<IndexT>(builtin::is_surf))
            {
                simplicial_complex_has_surf.push_back(simplicial_complex);
            }
        }
    }

    if(simplicial_complex_has_surf.empty())
    {
        UIPC_WARN_WITH_LOCATION("No simplicial complex with surface found.");
        return;
    }

    // 2) merge all the surfaces
    SimplicialComplex merged_surface = extract_surface(simplicial_complex_has_surf);
    // spdlog::info("Merged Scene Surface:\n{}", merged_surface);


    // 3) write the merged surface to file
    fs::path path = filename;
    fs::exists(path.parent_path()) || fs::create_directories(path.parent_path());
    SimplicialComplexIO io;
    io.write_obj(path.string(), merged_surface);


    auto abs_path = fs::absolute(path).string();
    spdlog::info("Scene surface with Faces({}), Vertices({}) written to {}",
                 merged_surface.triangles().size(),
                 merged_surface.vertices().size(),
                 abs_path.c_str());
}
}  // namespace uipc::world
