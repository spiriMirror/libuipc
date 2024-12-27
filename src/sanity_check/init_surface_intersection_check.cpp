#include <uipc/core/contact_model.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/io/simplicial_complex_io.h>
#include <sanity_checker.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/io/spread_sheet_io.h>
#include <context.h>
#include <uipc/geometry/utils/bvh.h>
#include <uipc/geometry/utils/intersection.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/map.h>

namespace std
{
// Vector2i  set comparison
template <>
struct less<uipc::Vector2i>
{
    bool operator()(const uipc::Vector2i& lhs, const uipc::Vector2i& rhs) const
    {
        return lhs[0] < rhs[0] || (lhs[0] == rhs[0] && lhs[1] < rhs[1]);
    }
};
}  // namespace std

namespace uipc::sanity_check
{
class InitSurfaceIntersectionCheck final : public SanityChecker
{
  public:
    constexpr static U64 SanityCheckerUID = 1;
    using SanityChecker::SanityChecker;

    geometry::BVH              bvh;
    vector<core::ContactModel> contact_table;
    SizeT                      contact_element_count;

  protected:
    virtual void build(backend::SceneVisitor& scene) override
    {
        auto enable_contact = scene.info()["contact"]["enable"].get<bool>();
        if(!enable_contact)
        {
            throw SanityCheckerException("Contact is not enabled");
        }
    }

    virtual U64 get_id() const noexcept override { return SanityCheckerUID; }

    geometry::SimplicialComplex extract_intersect_mesh(const geometry::SimplicialComplex& scene_surface,
                                                       span<const IndexT> edge_intersected,
                                                       span<const IndexT> tri_intersected)
    {
        geometry::SimplicialComplex i_mesh;
        i_mesh.vertices().resize(scene_surface.vertices().size());
        auto i_edge_count =
            std::ranges::count_if(edge_intersected,
                                  [&](IndexT i) { return i == 1; });
        auto i_tri_count = std::ranges::count_if(tri_intersected,
                                                 [&](IndexT i) { return i == 1; });
        i_mesh.edges().create<Vector2i>(builtin::topo);
        i_mesh.edges().resize(i_edge_count);
        i_mesh.triangles().create<Vector3i>(builtin::topo);
        i_mesh.triangles().resize(i_tri_count);

        auto pos      = i_mesh.vertices().create<Vector3>(builtin::position);
        auto pos_view = view(*pos);
        std::ranges::copy(scene_surface.positions().view(), pos_view.begin());

        auto src_edge_view = scene_surface.edges().topo().view();
        auto src_tri_view  = scene_surface.triangles().topo().view();

        auto dst_edge_view = view(i_mesh.edges().topo());
        auto dst_tri_view  = view(i_mesh.triangles().topo());


        {
            auto I = 0;
            std::ranges::copy_if(src_edge_view,
                                 dst_edge_view.begin(),
                                 [&](const Vector2i& e)
                                 { return edge_intersected[I++] == 1; });
        }

        {
            auto I = 0;
            std::ranges::copy_if(src_tri_view,
                                 dst_tri_view.begin(),
                                 [&](const Vector3i& f)
                                 { return tri_intersected[I++] == 1; });
        }

        return i_mesh;
    }

    virtual SanityCheckResult do_check(backend::SceneVisitor& scene,
                                       backend::SanityCheckMessageVisitor& msg) noexcept override
    {
        auto context = find<Context>();

        const geometry::SimplicialComplex& scene_surface =
            context->scene_simplicial_surface();

        const ContactTabular& contact_tabular = context->contact_tabular();

        auto Vs = scene_surface.vertices().size() ? scene_surface.positions().view() :
                                                    span<const Vector3>{};
        auto Es = scene_surface.edges().size() ? scene_surface.edges().topo().view() :
                                                 span<const Vector2i>{};
        auto Fs = scene_surface.triangles().size() ?
                      scene_surface.triangles().topo().view() :
                      span<const Vector3i>{};

        if(Vs.size() == 0 || Es.size() == 0 || Fs.size() == 0)  // no neet to check intersection
            return SanityCheckResult::Success;

        auto attr_cids =
            scene_surface.vertices().find<IndexT>("sanity_check/contact_element_id");
        UIPC_ASSERT(attr_cids, "`sanity_check/contact_element_id` is not found in scene surface");
        auto CIds = attr_cids->view();

        auto attr_v_geo_ids =
            scene_surface.vertices().find<IndexT>("sanity_check/geometry_id");
        UIPC_ASSERT(attr_v_geo_ids, "`sanity_check/geometry_id` is not found in scene surface");
        auto VGeoIds = attr_v_geo_ids->view();

        auto attr_v_instance_id =
            scene_surface.vertices().find<IndexT>("sanity_check/instance_id");
        UIPC_ASSERT(attr_v_instance_id,
                    "`sanity_check/instance_id` is not found in scene surface");
        auto VInstanceIds = attr_v_instance_id->view();

        auto attr_v_object_id =
            scene_surface.vertices().find<IndexT>("sanity_check/object_id");
        UIPC_ASSERT(attr_v_object_id, "`sanity_check/object_id` is not found in scene surface");
        auto VObjectIds = attr_v_object_id->view();

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

        vector<IndexT> edge_intersected(Es.size(), 0);
        vector<IndexT> tri_intersected(Fs.size(), 0);

        auto& contact_table = context->contact_tabular();
        auto  objs          = this->objects();

        bool has_intersection = false;

        map<Vector2i, Vector2i> intersected_geo_ids;

        bvh.query(
            edge_aabbs,
            [&](IndexT i, IndexT j)
            {
                Vector2i E = Es[i];
                Vector3i F = Fs[j];

                // 1) if there is a common point, don't consider it as an intersection
                {
                    Vector2i sorted_E = E;
                    Vector3i sorted_F = F;

                    std::sort(sorted_E.begin(), sorted_E.end());
                    std::sort(sorted_F.begin(), sorted_F.end());

                    vector<int> same_points;
                    same_points.reserve(3);
                    std::ranges::set_intersection(
                        sorted_E, sorted_F, std::back_inserter(same_points));
                    if(same_points.size() > 0)
                        return;
                }

                auto L = CIds[E[0]];
                auto R = CIds[E[1]];

                const core::ContactModel& model = contact_table.at(L, R);

                // 2) if the contact model is not enabled, don't consider it as an intersection
                if(!model.is_enabled())
                    return;


                Vector3 uvw;
                Vector2 uv;
                bool    is_coplanar;

                bool intersected = geometry::tri_edge_intersect(
                    Vs[F[0]], Vs[F[1]], Vs[F[2]], Vs[E[0]], Vs[E[1]], is_coplanar, uvw, uv);

                if(intersected)
                {
                    edge_intersected[i] = 1;
                    tri_intersected[j]  = 1;
                    has_intersection    = true;

                    auto GeoIdL = VGeoIds[E[0]];
                    auto GeoIdR = VGeoIds[F[1]];

                    auto ObjIdL = VObjectIds[E[0]];
                    auto ObjIdR = VObjectIds[F[1]];

                    if(GeoIdL > GeoIdR)
                    {
                        std::swap(GeoIdL, GeoIdR);
                        std::swap(ObjIdL, ObjIdR);
                    }

                    intersected_geo_ids[{GeoIdL, GeoIdR}] = {ObjIdL, ObjIdR};
                }
            });

        if(has_intersection)
        {
            // create a fmt buffer
            auto& buffer = msg.message();

            for(auto& [GeoIds, ObjIds] : intersected_geo_ids)
            {
                auto obj_0 = objects().find(ObjIds[0]);
                auto obj_1 = objects().find(ObjIds[1]);

                UIPC_ASSERT(obj_0 != nullptr, "Object[{}] not found", ObjIds[0]);
                UIPC_ASSERT(obj_1 != nullptr, "Object[{}] not found", ObjIds[1]);

                fmt::format_to(std::back_inserter(buffer),
                               "Geometry({}) in Object[{}({})] intersects with Geometry({}) in "
                               "Object[{}({})]\n",
                               GeoIds[0],
                               obj_0->name(),
                               obj_0->id(),
                               GeoIds[1],
                               obj_1->name(),
                               obj_1->id());
            }

            auto intersected_mesh =
                extract_intersect_mesh(scene_surface, edge_intersected, tri_intersected);

            fmt::format_to(std::back_inserter(buffer),
                           "Intersected mesh has {} edges and {} triangles.",
                           intersected_mesh.edges().size(),
                           intersected_mesh.triangles().size());

            std::string name = "intersected_mesh";

            msg.geometries()["intersected_mesh"] =
                uipc::make_shared<geometry::SimplicialComplex>(std::move(intersected_mesh));

            return SanityCheckResult::Error;
        }

        return SanityCheckResult::Success;
    };
};

REGISTER_SANITY_CHECKER(InitSurfaceIntersectionCheck);
}  // namespace uipc::sanity_check
