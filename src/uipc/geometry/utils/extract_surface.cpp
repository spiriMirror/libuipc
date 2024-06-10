#include <uipc/geometry/utils/extract_surface.h>
#include <uipc/geometry/utils/label_surface.h>
#include <uipc/common/enumerate.h>
#include <uipc/builtin/attribute_name.h>
namespace uipc::geometry
{
constexpr std::string_view hint =
    "Hint: You may need to call `label_surface()` before calling `extract_surface_to()`";

static void extract_surface_to(const SimplicialComplex& src, SimplicialComplex& dst)
{
    auto is_surf = src.vertices().find<IndexT>(builtin::is_surf);
    UIPC_ASSERT(is_surf, "`is_surf` attribute not found in the mesh vertices. {}", hint);


    UIPC_ASSERT(is_surf->size() == src.vertices().size(),
                "Mismatch in size of is_surf({}) and vertices({})",
                is_surf->size(),
                src.vertices().size());

    auto is_surf_view = is_surf->view();

    // ---------------------------------------------------------------------
    // process the vertices
    // ---------------------------------------------------------------------

    std::vector<IndexT> old_v_to_new_v(src.vertices().size(), -1);
    IndexT              surf_v_count = 0;
    for(auto&& [i, surf] : enumerate(is_surf_view))
    {
        if(surf)
        {
            // record the mapping from old vertex index to new vertex index
            old_v_to_new_v[i] = surf_v_count++;
        }
    }

    // resize the destination vertices
    dst.vertices().resize(surf_v_count);

    auto P = dst.vertices().find<Vector3>(builtin::position);

    if(!P)  // check if the position attribute exists
    {
        P = dst.vertices().create<Vector3>(builtin::position);
    }

    auto new_P_view = view(*P);
    auto old_P_view = src.positions().view();

    // copy_from the vertices
    for(auto&& [i, new_v_id] : enumerate(old_v_to_new_v))
    {
        if(new_v_id != -1)
        {
            new_P_view[new_v_id] = old_P_view[i];
        }
    }

    // ---------------------------------------------------------------------
    // process the edges
    // ---------------------------------------------------------------------

    auto e_is_surf = src.edges().find<IndexT>(builtin::is_surf);

    UIPC_ASSERT(e_is_surf, "`is_surf` attribute not found in the mesh edges. {}", hint);

    auto           e_is_surf_view = e_is_surf->view();
    IndexT         surf_edges     = 0;
    auto           old_edge_view  = src.edges().topo().view();
    vector<IndexT> old_e_to_new_e(src.edges().size(), -1);
    IndexT         surf_e_count = 0;

    for(auto&& [i, edge] : enumerate(old_edge_view))
    {
        if(e_is_surf_view[i])
        {
            old_e_to_new_e[i] = surf_e_count++;
        }
    }

    // resize the destination edges
    dst.edges().resize(surf_e_count);

    auto new_edge_view = view(dst.edges().topo());

    // copy_from the edges
    for(auto&& [i, new_e_id] : enumerate(old_e_to_new_e))
    {
        if(new_e_id != -1)
        {
            auto old_edge           = old_edge_view[i];
            auto old_v0             = old_edge[0];
            auto old_v1             = old_edge[1];
            auto new_v0             = old_v_to_new_v[old_v0];
            auto new_v1             = old_v_to_new_v[old_v1];
            new_edge_view[new_e_id] = {new_v0, new_v1};
        }
    }

    // ---------------------------------------------------------------------
    // process the triangles
    // ---------------------------------------------------------------------
    auto t_is_surf = src.triangles().find<IndexT>(builtin::is_surf);

    UIPC_ASSERT(t_is_surf, "`is_surf` attribute not found in the mesh triangles. {}", hint);

    auto           t_is_surf_view = t_is_surf->view();
    auto           old_tri_view   = src.triangles().topo().view();
    vector<IndexT> old_t_to_new_t(src.triangles().size(), -1);
    IndexT         surf_t_count = 0;

    for(auto&& [i, tri] : enumerate(old_tri_view))
    {
        if(t_is_surf_view[i])
        {
            old_t_to_new_t[i] = surf_t_count++;
        }
    }

    // resize the destination triangles
    dst.triangles().resize(surf_t_count);
    auto new_tri_view = view(dst.triangles().topo());

    // copy_from the triangles
    for(auto&& [i, new_t_id] : enumerate(old_t_to_new_t))
    {
        if(new_t_id != -1)
        {
            auto old_tri           = old_tri_view[i];
            auto old_v0            = old_tri[0];
            auto old_v1            = old_tri[1];
            auto old_v2            = old_tri[2];
            auto new_v0            = old_v_to_new_v[old_v0];
            auto new_v1            = old_v_to_new_v[old_v1];
            auto new_v2            = old_v_to_new_v[old_v2];
            new_tri_view[new_t_id] = {new_v0, new_v1, new_v2};
        }
    }
}

SimplicialComplex extract_surface(const SimplicialComplex& src)
{
    SimplicialComplex dst;
    auto              v_is_surf = src.vertices().find<IndexT>(builtin::is_surf);
    auto              e_is_surf = src.edges().find<IndexT>(builtin::is_surf);
    auto              f_is_surf = src.triangles().find<IndexT>(builtin::is_surf);

    if(!v_is_surf || !e_is_surf || !f_is_surf)
    {
        auto L = label_surface(src);
        extract_surface_to(L, dst);
    }
    else
    {
        extract_surface_to(src, dst);
    }
    return dst;
}
}  // namespace uipc::geometry