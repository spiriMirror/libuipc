#include <uipc/geometry/utils/flip_inward_triangles.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/enumerate.h>

namespace uipc::geometry
{
SimplicialComplex flip_inward_triangles(const SimplicialComplex& sc)
{
    SimplicialComplex R = sc;

    auto f_is_surf = R.triangles().find<IndexT>(builtin::is_surf);
    auto f_orient  = R.triangles().find<IndexT>(builtin::orient);

    UIPC_ASSERT(f_is_surf, "Cannot find attribute `is_surf` on triangles. You may need to call `label_surface()` first");
    UIPC_ASSERT(f_orient, "Cannot find attribute `orient` on triangles. You may need to call `label_triangle_orient()` first");

    auto src_Fs = R.triangles().topo().view();
    auto dst_Fs = view(R.triangles().topo());

    auto src_orient_view = f_orient->view();
    auto dst_orient_view = view(*f_orient);

    for(auto&& [i, F] : enumerate(src_Fs))
    {
        auto orient = src_orient_view[i];
        if(orient < 0)  // inward
        {
            dst_Fs[i]          = {F[0], F[2], F[1]};
            dst_orient_view[i] = 1;
        }
    }

    return R;
}
}  // namespace uipc::geometry
