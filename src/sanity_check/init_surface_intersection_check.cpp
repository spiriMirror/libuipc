#include <uipc/core/contact_model.h>
#include <uipc/geometry/simplicial_complex.h>
#include <sanity_checker.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/io/spread_sheet_io.h>
#include <context.h>
#include <uipc/geometry/utils/bvh.h>
#include <uipc/geometry/utils/intersection.h>

namespace uipc::sanity_check
{
class InitSurfaceIntersectionCheck final : public SanityChecker
{
  public:
    using SanityChecker::SanityChecker;

    geometry::BVH              bvh;
    vector<core::ContactModel> contact_table;
    SizeT                      contact_element_count;

  protected:
    virtual U64 get_id() const noexcept override { return 1; }

    virtual SanityCheckResult do_check(backend::SceneVisitor& scene) noexcept override
    {
        auto context = find<Context>();

        const geometry::SimplicialComplex& scene_surface =
            context->scene_simplicial_surface();

        const ContactTabular& contact_tabular = context->contact_tabular();

        auto Vs = scene_surface.positions().view();

        auto Es = scene_surface.edges().topo().view();
        auto Fs = scene_surface.triangles().topo().view();

        vector<geometry::BVH::AABB> tri_aabbs(Fs.size());
        for(auto [i, f] : enumerate(Fs))
        {
            tri_aabbs[i].extend(Vs[f[0]]).extend(Vs[f[1]]).extend(Vs[f[2]]);
        }

        vector<geometry::BVH::AABB> edge_aabbs(Es.size());
        for(auto [i, e] : enumerate(Es))
        {
            edge_aabbs[i].extend(Vs[e[0]]).extend(Vs[e[1]]);
        }

        bvh.build(tri_aabbs);

        bvh.query(edge_aabbs,
                  [](IndexT i, IndexT j)
                  {
                      //TODO:
                  });

        return SanityCheckResult::Success;
    };
};

REGISTER_SANITY_CHECKER(InitSurfaceIntersectionCheck);
}  // namespace uipc::sanity_check
