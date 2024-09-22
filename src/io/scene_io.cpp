#include <uipc/io/scene_io.h>
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


namespace uipc::world
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

geometry::SimplicialComplex SceneIO::simplicial_surface() const
{
    using namespace uipc::geometry;

    auto scene = backend::SceneVisitor{m_scene};
    auto geos  = scene.geometries();

    auto simplicial_complex_has_surf = detail::collect_geometry_with_surf<-1>(geos);

    return extract_surface(simplicial_complex_has_surf);
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
}  // namespace uipc::world
