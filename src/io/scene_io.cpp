#include <uipc/io/scene_io.h>
#include <uipc/core/scene_factory.h>
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
#include <uipc/io/simplicial_complex_io.h>
#include <uipc/core/scene_factory.h>


namespace uipc::core
{
namespace fs = std::filesystem;

namespace detail
{

    template <IndexT Dim = -1>  // Dim = -1, 0, 1, 2
    static vector<const geometry::SimplicialComplex*> collect_geometry_with_surf(
        span<S<geometry::GeometrySlot>> geos)
        requires(Dim == -1 || Dim == 0 || Dim == 1 || Dim == 2)
    {
        using namespace uipc::geometry;
        // 1) find all simplicial complex with surface
        vector<const SimplicialComplex*> simplicial_complex_has_surf;
        simplicial_complex_has_surf.reserve(geos.size());

        for(auto& geo : geos)
        {
            if(geo->geometry().type() == builtin::SimplicialComplex)
            {
                auto simplicial_complex =
                    dynamic_cast<SimplicialComplex*>(&geo->geometry());

                UIPC_ASSERT(simplicial_complex, "type mismatch, why can it happen?");

                bool allow = false;

                if constexpr(Dim == -1)
                {
                    allow = true;
                }
                else if constexpr(Dim == 2)
                {
                    // allow tetrahedron and triangle mesh
                    allow = simplicial_complex->dim() >= 2;
                }
                else
                {
                    allow = simplicial_complex->dim() == Dim;
                }

                if(allow)
                {
                    switch(simplicial_complex->dim())
                    {
                        case 0:
                            if(simplicial_complex->vertices().find<IndexT>(builtin::is_surf))
                            {
                                simplicial_complex_has_surf.push_back(simplicial_complex);
                            }
                            break;
                        case 1:
                            if(simplicial_complex->edges().find<IndexT>(builtin::is_surf))
                            {
                                simplicial_complex_has_surf.push_back(simplicial_complex);
                            }
                            break;
                        case 2:
                        case 3:
                            if(simplicial_complex->triangles().find<IndexT>(builtin::is_surf))
                            {
                                simplicial_complex_has_surf.push_back(simplicial_complex);
                            }
                            break;
                        default:
                            break;
                    }
                }
            }
        }

        return simplicial_complex_has_surf;
    }
}  // namespace detail


SceneIO::SceneIO(Scene& scene)
    : m_scene(scene)
{
}

void SceneIO::write_surface_obj(std::string_view filename)
{
    using namespace uipc::geometry;

    auto merged_surface = simplicial_surface();

    fs::path path = fs::absolute(fs::path{filename});

    fs::exists(path.parent_path()) || fs::create_directories(path.parent_path());

    SimplicialComplexIO io;
    io.write_obj(path.string(), merged_surface);

    auto abs_path = fs::absolute(path).string();
    spdlog::info("Scene surface with Faces({}), Edges({}), Vertices({}) written to {}",
                 merged_surface.triangles().size(),
                 merged_surface.edges().size(),
                 merged_surface.vertices().size(),
                 abs_path);
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

geometry::SimplicialComplex SceneIO::simplicial_surface(IndexT dim) const
{
    using namespace uipc::geometry;

    auto scene = backend::SceneVisitor{m_scene};
    auto geos  = scene.geometries();

    vector<const geometry::SimplicialComplex*> simplicial_complex_has_surf;
    switch(dim)
    {
        case -1:
            simplicial_complex_has_surf = detail::collect_geometry_with_surf<-1>(geos);
            break;
        case 0:
            simplicial_complex_has_surf = detail::collect_geometry_with_surf<0>(geos);
            break;
        case 1:
            simplicial_complex_has_surf = detail::collect_geometry_with_surf<1>(geos);
            break;
        case 2:
            simplicial_complex_has_surf = detail::collect_geometry_with_surf<2>(geos);
            break;
        default:
            UIPC_ERROR_WITH_LOCATION("Unsupported input dimension {} (expected dim=0/1/2).", dim);
            break;
    }

    return extract_surface(simplicial_complex_has_surf);
}

void SceneIO::save(const Scene& scene, std::string_view filename)
{
    fs::path path{filename};
    path = fs::absolute(path);

    auto ext = path.extension();

    SceneFactory sf;
    auto         scene_json = sf.to_json(scene);

    if(ext == ".json")
    {
        fs::exists(path.parent_path()) || fs::create_directories(path.parent_path());
        std::ofstream file(path.string());
        if(file)
        {
            file << scene_json.dump(4);
        }
        else
        {
            throw SceneIOError(fmt::format("Failed to open file {} for writing.",
                                           path.string()));
        }
    }
    else if(ext == ".bson")
    {
        fs::exists(path.parent_path()) || fs::create_directories(path.parent_path());
        std::vector<std::uint8_t> v = Json::to_bson(scene_json);
        std::ofstream             file(path, std::ios::binary);
        if(file)
        {
            file.write(reinterpret_cast<const char*>(v.data()), v.size());
        }
        else
        {
            throw SceneIOError(fmt::format("Failed to open file {} for writing.",
                                           path.string()));
        }
    }
    else
    {
        throw SceneIOError(fmt::format("Unsupported file format when writing {}.", filename));
    }
}

void SceneIO::save(std::string_view filename) const
{
    save(m_scene, filename);
}

Scene SceneIO::load(std::string_view filename)
{
    fs::path path{filename};
    path = fs::absolute(path);

    auto ext = path.extension();

    SceneFactory sf;
    if(ext == ".json")
    {
        std::ifstream file(path.string());
        if(file)
        {
            Json scene_json;
            file >> scene_json;
            return sf.from_snapshot(sf.from_json(scene_json));
        }
        else
        {
            throw SceneIOError(fmt::format("Failed to open file {} for reading.",
                                           path.string()));
        }
    }
    else if(ext == ".bson")
    {
        std::ifstream file(path, std::ios::binary);
        if(file)
        {
            std::vector<std::uint8_t> v((std::istreambuf_iterator<char>(file)),
                                        std::istreambuf_iterator<char>());
            Json                      scene_json = Json::from_bson(v);
            return sf.from_snapshot(sf.from_json(scene_json));
        }
        else
        {
            throw SceneIOError(fmt::format("Failed to open file {} for reading.",
                                           path.string()));
        }
    }
    else
    {
        throw SceneIOError(fmt::format("Unsupported file format when loading {}.", filename));
    }

    return Scene{};
}

void SceneIO::commit(const SceneSnapshot& last, std::string_view filename)
{
    fs::path path{filename};
    path = fs::absolute(path);

    auto ext = path.extension();

    Json commit_json = commit_to_json(last);

    if(ext == ".json")
    {
        fs::exists(path.parent_path()) || fs::create_directories(path.parent_path());
        std::ofstream file(path.string());
        if(file)
        {
            file << commit_json.dump(4);
        }
        else
        {
            throw SceneIOError(fmt::format("Failed to open file {} for writing.",
                                           path.string()));
        }
    }
    else if(ext == ".bson")
    {
        fs::exists(path.parent_path()) || fs::create_directories(path.parent_path());
        std::vector<std::uint8_t> v = Json::to_bson(commit_json);
        std::ofstream             file(path, std::ios::binary);
        if(file)
        {
            file.write(reinterpret_cast<const char*>(v.data()), v.size());
        }
        else
        {
            throw SceneIOError(fmt::format("Failed to open file {} for writing.",
                                           path.string()));
        }
    }
    else
    {
        throw SceneIOError(fmt::format("Unsupported file format when writing {}.", filename));
    }
}

void SceneIO::update(std::string_view filename)
{
    fs::path path{filename};
    path = fs::absolute(path);

    auto ext = path.extension();

    Json commit_json;

    if(ext == ".json")
    {
        std::ifstream file(path.string());
        if(file)
        {
            file >> commit_json;
        }
        else
        {
            throw SceneIOError(fmt::format("Failed to open file {} for reading.",
                                           path.string()));
        }
    }
    else if(ext == ".bson")
    {
        std::ifstream file(path, std::ios::binary);
        if(file)
        {
            std::vector<std::uint8_t> v((std::istreambuf_iterator<char>(file)),
                                        std::istreambuf_iterator<char>());
            commit_json = Json::from_bson(v);
        }
        else
        {
            throw SceneIOError(fmt::format("Failed to open file {} for reading.",
                                           path.string()));
        }
    }
    else
    {
        throw SceneIOError(fmt::format("Unsupported file format when loading {}.", filename));
    }

    update_from_json(commit_json);
}

Json SceneIO::to_json() const
{
    SceneFactory sf;
    return sf.to_json(m_scene);
}

Scene SceneIO::from_json(const Json& json)
{
    SceneFactory sf;
    return sf.from_snapshot(sf.from_json(json));
}

Json SceneIO::commit_to_json(const SceneSnapshot& reference) const
{
    SceneFactory        sf;
    SceneSnapshotCommit commit = m_scene - reference;
    return sf.commit_to_json(commit);
}

void SceneIO::update_from_json(const Json& json)
{
    SceneFactory sf;
    auto         commit = sf.commit_from_json(json);

    if(!commit.is_valid())
    {
        UIPC_WARN_WITH_LOCATION("Invalid commit file, no update to scene");
        return;
    }

    m_scene.update_from(commit);
}
}  // namespace uipc::core
