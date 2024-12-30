#include <uipc/core/contact_model.h>
#include <context.h>
#include <uipc/common/range.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/io/simplicial_complex_io.h>
#include <sanity_checker.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/io/spread_sheet_io.h>
#include <uipc/geometry/utils/bvh.h>
#include <uipc/geometry/utils/intersection.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/builtin/geometry_type.h>
#include <uipc/geometry/utils/distance.h>
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
class HalfPlaneVertexDistanceCheck final : public SanityChecker
{
  public:
    constexpr static U64 SanityCheckerUID = 2;
    constexpr static U64 HalfPlaneUID     = 1;  // ImplicitGeometryUID = 1
    using SanityChecker::SanityChecker;

  protected:
    vector<geometry::Geometry*> halfplanes;


    virtual void build(backend::SceneVisitor& scene) override
    {
        auto enable_contact = scene.info()["contact"]["enable"].get<bool>();
        if(!enable_contact)
        {
            throw SanityCheckerException("Contact is not enabled");
        }
    }

    virtual U64 get_id() const noexcept override { return SanityCheckerUID; }

    void collect_halfplanes(backend::SceneVisitor& scene)
    {
        auto geo_slots = scene.geometries();
        for(auto& slot : geo_slots)
        {
            auto& geo = slot->geometry();
            if(geo.type() == builtin::ImplicitGeometry)
            {
                auto uid = geo.meta().find<U64>(builtin::implicit_geometry_uid);

                UIPC_ASSERT(uid, "ImplicitGeometryUID not found, why can it happen?");

                if(uid->view()[0] == HalfPlaneUID)
                {
                    halfplanes.push_back(&geo);
                }
            }
        }
    }

    geometry::SimplicialComplex create_close_mesh(const geometry::SimplicialComplex& scene_surface,
                                                  const vector<IndexT>& vertex_is_too_close)
    {
        geometry::SimplicialComplex mesh;

        // 1) Copy attributes
        vector<SizeT> close_vertices;
        {
            close_vertices.reserve(vertex_is_too_close.size());

            for(auto&& [I, is_close] : enumerate(vertex_is_too_close))
            {
                if(is_close)
                {
                    close_vertices.push_back(I);
                }
            }

            mesh.vertices().resize(close_vertices.size());
            mesh.vertices().copy_from(scene_surface.vertices(),
                                      geometry::AttributeCopy::pull(close_vertices));
        }


        // 2) Find close edges and triangles
        {
            auto src_Es = scene_surface.edges().topo().view();
            auto src_Fs = scene_surface.triangles().topo().view();

            vector<SizeT> close_edges;
            close_edges.reserve(src_Es.size());
            for(auto&& [I, E] : enumerate(src_Es))
            {
                if(vertex_is_too_close[E[0]] && vertex_is_too_close[E[1]])
                {
                    close_edges.push_back(I);
                }
            }
            mesh.edges().resize(close_edges.size());
            mesh.edges().copy_from(scene_surface.edges(),
                                   geometry::AttributeCopy::pull(close_edges));

            vector<SizeT> close_triangles;
            close_triangles.reserve(src_Fs.size());
            for(auto&& [I, F] : enumerate(src_Fs))
            {
                if(vertex_is_too_close[F[0]] && vertex_is_too_close[F[1]]
                   && vertex_is_too_close[F[2]])
                {
                    close_triangles.push_back(I);
                }
            }
            mesh.triangles().resize(close_triangles.size());
            mesh.triangles().copy_from(scene_surface.triangles(),
                                       geometry::AttributeCopy::pull(close_triangles));
        }

        // 3) Remap the vertex indices in edges and triangles
        {
            vector<SizeT> vert_remap(scene_surface.vertices().size(), -1);
            for(auto&& [I, V] : enumerate(close_vertices))
            {
                vert_remap[V] = I;
            }

            auto Map = [&]<IndexT N>(const Eigen::Vector<IndexT, N>& V) -> Eigen::Vector<IndexT, N>
            {
                auto ret = V;
                for(auto& v : ret)
                    v = vert_remap[v];
                return ret;
            };

            auto edge_topo_view = view(mesh.edges().topo());
            std::ranges::transform(edge_topo_view, edge_topo_view.begin(), Map);

            auto tri_topo_view = view(mesh.triangles().topo());
            std::ranges::transform(tri_topo_view, tri_topo_view.begin(), Map);
        }

        return mesh;
    }

    virtual SanityCheckResult do_check(backend::SceneVisitor& scene,
                                       backend::SanityCheckMessageVisitor& msg) noexcept override
    {
        collect_halfplanes(scene);

        // no halfplane, no need to check
        if(halfplanes.empty())
            return SanityCheckResult::Success;

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

        if(Vs.size() == 0)  // no need to check distance
            return SanityCheckResult::Success;

        auto attr_cids =
            scene_surface.vertices().find<IndexT>("sanity_check/contact_element_id");
        UIPC_ASSERT(attr_cids, "`sanity_check/contact_element_id` is not found in scene surface");
        auto CIds = attr_cids->view();

        auto attr_v_geo_ids =
            scene_surface.vertices().find<IndexT>("sanity_check/geometry_id");
        UIPC_ASSERT(attr_v_geo_ids, "`sanity_check/geometry_id` is not found in scene surface");
        auto VGeoIds = attr_v_geo_ids->view();

        auto attr_v_object_id =
            scene_surface.vertices().find<IndexT>("sanity_check/object_id");
        UIPC_ASSERT(attr_v_object_id, "`sanity_check/object_id` is not found in scene surface");
        auto VObjectIds = attr_v_object_id->view();

        auto attr_thickeness = scene_surface.vertices().find<Float>(builtin::thickness);
        span<const Float> VThickness =
            attr_thickeness ? attr_thickeness->view() : span<const Float>{};  // default 0.0

        auto& contact_table = context->contact_tabular();
        auto  objs          = this->objects();

        bool too_close = false;

        // key: {geo_id_0, geo_id_1}, value: {obj_id_0, obj_id_1}
        map<Vector2i, Vector2i> close_geo_ids;

        vector<IndexT> vertex_too_close(Vs.size(), 0);

        for(auto& halfplane : halfplanes)
        {
            auto instance_count = halfplane->instances().size();
            auto attr_N         = halfplane->instances().find<Vector3>("N");
            UIPC_ASSERT(attr_N, "Normal vector `N` not found in half-plane");
            auto Ns = attr_N->view();

            auto attr_P = halfplane->instances().find<Vector3>("P");
            UIPC_ASSERT(attr_P, "Origin point `P` not found in half-plane");
            auto Ps = attr_P->view();

            auto attr_geo_id =
                halfplane->instances().find<IndexT>("sanity_check/geometry_id");
            UIPC_ASSERT(attr_geo_id, "`sanity_check/geometry_id` not found in half-plane");
            auto HGeoIds = attr_geo_id->view();

            auto attr_object_id =
                halfplane->instances().find<IndexT>("sanity_check/object_id");
            UIPC_ASSERT(attr_object_id, "`sanity_check/object_id` not found in half-plane");
            auto HObjectIds = attr_object_id->view();

            auto attr_cid = halfplane->meta().find<IndexT>(builtin::contact_element_id);
            auto HCid = attr_cid ? attr_cid->view()[0] : 0;


            for(auto I : range(instance_count))
            {
                const Vector3& N = Ns[I];
                const Vector3& P = Ps[I];

                for(auto vI : range(Vs.size()))
                {
                    const auto& CM = contact_table.at(HCid, CIds[vI]);

                    if(!CM.is_enabled())  // if unenabled, skip
                        continue;

                    const Vector3& V = Vs[vI];
                    auto V_thickness = VThickness.size() ? VThickness[vI] : 0.0;

                    auto d = geometry::halfplane_vertex_signed_distance(P, N, V, V_thickness);

                    if(d <= 0)  // too close
                    {
                        too_close = true;

                        auto geo_id_0 = VGeoIds[vI];
                        auto geo_id_1 = HGeoIds[I];

                        auto obj_id_0 = VObjectIds[vI];
                        auto obj_id_1 = HObjectIds[I];

                        close_geo_ids[{geo_id_0, geo_id_1}] = {obj_id_0, obj_id_1};

                        vertex_too_close[vI] = 1;
                    }
                }
            }
        }

        if(too_close)
        {
            auto& buffer = msg.message();

            for(auto& [GeoIds, ObjIds] : close_geo_ids)
            {
                auto obj_0 = objects().find(ObjIds[0]);
                auto obj_1 = objects().find(ObjIds[1]);

                UIPC_ASSERT(obj_0 != nullptr, "Object[{}] not found", ObjIds[0]);
                UIPC_ASSERT(obj_1 != nullptr, "Object[{}] not found", ObjIds[1]);

                fmt::format_to(std::back_inserter(buffer),
                               "Geometry({}) in Object[{}({})] is too close (distance <= 0) to HalfPlane({}) in "
                               "Object[{}({})]\n",
                               GeoIds[0],
                               obj_0->name(),
                               obj_0->id(),
                               GeoIds[1],
                               obj_1->name(),
                               obj_1->id());
            }

            auto close_mesh = create_close_mesh(scene_surface, vertex_too_close);

            fmt::format_to(std::back_inserter(buffer),
                           "Close mesh has {} vertices, {} edges, {} triangles.\n",
                           close_mesh.vertices().size(),
                           close_mesh.edges().size(),
                           close_mesh.triangles().size());

            std::string name = "close_mesh";

            if(scene.info()["sanity_check"]["mode"] == "normal")
            {
                auto output_path = this_output_path();
                namespace fs     = std::filesystem;
                fs::path path{output_path};
                path /= fmt::format("{}.obj", name);
                auto path_str = path.string();

                geometry::SimplicialComplexIO io;
                io.write(path_str, close_mesh);
                fmt::format_to(std::back_inserter(buffer), "Close mesh is saved at {}.\n", path_str);
            }

            fmt::format_to(std::back_inserter(buffer),
                           "Create mesh [{}<{}>] for post-processing.",
                           name,
                           close_mesh.type());

            msg.geometries()[name] =
                uipc::make_shared<geometry::SimplicialComplex>(std::move(close_mesh));

            return SanityCheckResult::Error;
        }

        return SanityCheckResult::Success;
    };
};

REGISTER_SANITY_CHECKER(HalfPlaneVertexDistanceCheck);
}  // namespace uipc::sanity_check
