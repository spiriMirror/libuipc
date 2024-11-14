#include <uipc/geometry/utils/extract_surface.h>
#include <uipc/geometry/utils/label_surface.h>
#include <uipc/common/enumerate.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/utils/apply_transform.h>
#include <uipc/geometry/utils/merge.h>
#include <numeric>

namespace uipc::geometry
{
constexpr std::string_view hint =
    "Hint: You may need to call `label_surface()` before calling `extract_surface()`";

SimplicialComplex extract_surface(const SimplicialComplex& src)
{
    UIPC_ASSERT(src.dim() == 3,
                "The input mesh must be a tetrahedral mesh. Your dim={}.",
                src.dim());

    SimplicialComplex R;

    auto v_is_surf = src.vertices().find<IndexT>(builtin::is_surf);
    auto e_is_surf = src.edges().find<IndexT>(builtin::is_surf);
    auto f_is_surf = src.triangles().find<IndexT>(builtin::is_surf);

    UIPC_ASSERT(v_is_surf, "`is_surf` attribute not found in the mesh vertices. {}", hint);
    UIPC_ASSERT(e_is_surf, "`is_surf` attribute not found in the mesh edges. {}", hint);
    UIPC_ASSERT(f_is_surf, "`is_surf` attribute not found in the mesh triangles. {}", hint);

    // ---------------------------------------------------------------------
    // copy the meta and instances
    // ---------------------------------------------------------------------

    R.meta().copy_from(src.meta());
    R.instances().resize(src.instances().size());
    R.instances().copy_from(src.instances());


    std::vector<IndexT> old_v_to_new_v(src.vertices().size(), -1);  // mapping from old vertex index to new vertex index

    // ---------------------------------------------------------------------
    // process the vertices
    // ---------------------------------------------------------------------
    {
        auto   v_is_surf_view = v_is_surf->view();
        IndexT surf_v_count   = 0;
        for(auto&& [i, surf] : enumerate(v_is_surf_view))
        {
            if(surf)
            {
                // record the mapping from old vertex index to new vertex index
                old_v_to_new_v[i] = surf_v_count++;
            }
        }

        // resize the destination vertices
        R.vertices().resize(surf_v_count);

        // setup new2old mapping
        std::vector<SizeT> v_new2old(surf_v_count, -1);
        for(auto&& [i, new_v_id] : enumerate(old_v_to_new_v))
        {
            if(new_v_id != -1)
            {
                v_new2old[new_v_id] = static_cast<SizeT>(i);
            }
        }

        // copy vertex attributes
        R.vertices().copy_from(src.vertices(), AttributeCopy::pull(v_new2old));
    }


    // ---------------------------------------------------------------------
    // process the edges
    // ---------------------------------------------------------------------
    {
        vector<string> exclude_attrs = {string{builtin::topo}};  // exclude topo

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
        R.edges().resize(surf_e_count);
        auto topo = R.edges().create<Vector2i>(builtin::topo, Vector2i::Zero(), false);
        auto new_edge_view = view(*topo);

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

        // setup new2old mapping
        std::vector<SizeT> e_new2old(surf_e_count, -1);
        for(auto&& [i, new_e_id] : enumerate(old_e_to_new_e))
        {
            if(new_e_id != -1)
            {
                e_new2old[new_e_id] = static_cast<SizeT>(i);
            }
        }

        // copy other edge attributes
        R.edges().copy_from(src.edges(), AttributeCopy::pull(e_new2old), {}, exclude_attrs);
    }


    // ---------------------------------------------------------------------
    // process the triangles
    // ---------------------------------------------------------------------
    {
        auto           t_is_surf_view = f_is_surf->view();
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
        R.triangles().resize(surf_t_count);
        auto topo = R.triangles().create<Vector3i>(builtin::topo, Vector3i::Zero(), false);
        auto new_tri_view = view(*topo);

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

        // setup new2old mapping
        std::vector<SizeT> t_new2old(surf_t_count, -1);
        for(auto&& [i, new_t_id] : enumerate(old_t_to_new_t))
        {
            if(new_t_id != -1)
            {
                t_new2old[new_t_id] = static_cast<SizeT>(i);
            }
        }

        // copy other triangle attributes

        std::array exclude_attrs = {
            string{builtin::parent_id},  // exclude parent_id
            string{builtin::is_facet},   // exclude is_facet
            string{builtin::topo}        // exclude topo
        };

        R.triangles().copy_from(src.triangles(), AttributeCopy::pull(t_new2old), {}, exclude_attrs);
        auto is_facet      = R.triangles().create<IndexT>(builtin::is_facet);
        auto is_facet_view = view(*is_facet);
        std::ranges::fill(is_facet_view, 1);  // now all the triangles are facets
    }

    return R;
}

static void extract_surface_check_input(span<const SimplicialComplex*> sc)
{
    for(auto [I, complex] : enumerate(sc))
    {
        UIPC_ASSERT(complex != nullptr, "Input[{}] is nullptr", I);
    }
}

SimplicialComplex extract_surface(span<const SimplicialComplex*> sc)
{
    if(sc.empty())
        return SimplicialComplex{};

    extract_surface_check_input(sc);

    // 1) extract the surface from each simplicial complex
    vector<SimplicialComplex> surfaces;
    surfaces.reserve(sc.size());

    std::transform(sc.begin(),
                   sc.end(),
                   std::back_inserter(surfaces),
                   [](const SimplicialComplex* simplicial_complex)
                   {
                       if(simplicial_complex->dim() == 3)
                           return extract_surface(*simplicial_complex);
                       else
                           return *simplicial_complex;
                   });

    // 2) find out all the surface instances, apply the transformation
    SizeT total_surface_instances =
        std::accumulate(sc.begin(),
                        sc.end(),
                        0ull,
                        [](SizeT acc, const SimplicialComplex* simplicial_complex)
                        { return acc + simplicial_complex->instances().size(); });

    vector<SimplicialComplex> all_surfaces;
    all_surfaces.reserve(total_surface_instances);

    for(auto& surface : surfaces)
    {
        vector<SimplicialComplex> instances = apply_transform(surface);
        std::move(instances.begin(), instances.end(), std::back_inserter(all_surfaces));
    }

    vector<const SimplicialComplex*> surfaces_ptr(total_surface_instances);

    std::transform(all_surfaces.begin(),
                   all_surfaces.end(),
                   surfaces_ptr.begin(),
                   [](SimplicialComplex& surface) { return &surface; });

    return merge(surfaces_ptr);
}
}  // namespace uipc::geometry