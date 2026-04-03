#include <type_define.h>
#include <uipc/core/object.h>
#include <uipc/core/contact_tabular.h>
#include <uipc/core/subscene_tabular.h>
#include <uipc/geometry/geometry_slot.h>
#include <sanity_check/backend_sanity_checker.h>
#include <uipc/backend/visitors/sanity_check_message_visitor.h>
#include <uipc/core/contact_model.h>
#include <uipc/common/range.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/io/simplicial_complex_io.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/builtin/geometry_type.h>
#include <uipc/common/map.h>
#include <muda/buffer.h>
#include <muda/launch.h>

namespace std
{
template <>
struct less<uipc::Vector2i>
{
    bool operator()(const uipc::Vector2i& lhs, const uipc::Vector2i& rhs) const
    {
        return lhs[0] < rhs[0] || (lhs[0] == rhs[0] && lhs[1] < rhs[1]);
    }
};
}  // namespace std

namespace uipc::backend::cuda
{
class HalfPlaneVertexDistanceCheck final : public BackendSanityChecker
{
  public:
    constexpr static U64 SanityCheckerUID = 2;
    constexpr static U64 HalfPlaneUID     = 1;
    using BackendSanityChecker::BackendSanityChecker;

    vector<geometry::Geometry*> halfplanes;

    virtual void build() override
    {
        auto enable_contact = context().config().find<IndexT>("contact/enable");
        if(!enable_contact->view()[0])
        {
            throw BackendSanityCheckerException("Contact is not enabled");
        }
    }

    virtual U64 get_id() const noexcept override { return SanityCheckerUID; }

    void collect_halfplanes()
    {
        halfplanes.clear();
        auto geo_slots = context().geometries();
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

    static geometry::SimplicialComplex create_close_mesh(const geometry::SimplicialComplex& scene_surface,
                                                         const vector<IndexT>& vertex_is_too_close)
    {
        geometry::SimplicialComplex mesh;

        vector<SizeT> close_vertices;
        {
            close_vertices.reserve(vertex_is_too_close.size());
            for(auto&& [I, is_close] : enumerate(vertex_is_too_close))
            {
                if(is_close)
                    close_vertices.push_back(I);
            }

            mesh.vertices().resize(close_vertices.size());
            mesh.vertices().copy_from(scene_surface.vertices(),
                                      geometry::AttributeCopy::pull(close_vertices));
        }

        {
            auto src_Es = scene_surface.edges().topo().view();
            auto src_Fs = scene_surface.triangles().topo().view();

            vector<SizeT> close_edges;
            close_edges.reserve(src_Es.size());
            for(auto&& [I, E] : enumerate(src_Es))
            {
                if(vertex_is_too_close[E[0]] && vertex_is_too_close[E[1]])
                    close_edges.push_back(I);
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
                    close_triangles.push_back(I);
            }
            mesh.triangles().resize(close_triangles.size());
            mesh.triangles().copy_from(scene_surface.triangles(),
                                       geometry::AttributeCopy::pull(close_triangles));
        }

        {
            vector<SizeT> vert_remap(scene_surface.vertices().size(), -1);
            for(auto&& [I, V] : enumerate(close_vertices))
                vert_remap[V] = I;

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

    virtual SanityCheckResult do_check(core::SanityCheckMessage& msg) override
    {
        using namespace muda;

        collect_halfplanes();

        if(halfplanes.empty())
            return SanityCheckResult::Success;

        auto& ctx = context();

        const geometry::SimplicialComplex& scene_surface =
            ctx.scene_simplicial_surface();

        auto& contact_tabular  = ctx.contact_tabular();
        auto& subscene_tabular = ctx.subscene_tabular();

        auto Vs = scene_surface.vertices().size() ? scene_surface.positions().view() :
                                                    span<const Vector3>{};
        auto Es = scene_surface.edges().size() ? scene_surface.edges().topo().view() :
                                                 span<const Vector2i>{};
        auto Fs = scene_surface.triangles().size() ?
                      scene_surface.triangles().topo().view() :
                      span<const Vector3i>{};

        if(Vs.size() == 0)
            return SanityCheckResult::Success;

        auto attr_cids =
            scene_surface.vertices().find<IndexT>("sanity_check/contact_element_id");
        UIPC_ASSERT(attr_cids, "`sanity_check/contact_element_id` is not found in scene surface");
        auto CIds = attr_cids->view();

        auto attr_scids = scene_surface.vertices().find<IndexT>(
            "sanity_check/subscene_contact_element_id");
        UIPC_ASSERT(attr_scids, "`sanity_check/subscene_contact_element_id` is not found in scene surface");
        auto SCIds = attr_scids->view();

        auto attr_v_geo_ids =
            scene_surface.vertices().find<IndexT>("sanity_check/geometry_id");
        UIPC_ASSERT(attr_v_geo_ids, "`sanity_check/geometry_id` is not found in scene surface");
        auto VGeoIds = attr_v_geo_ids->view();

        auto attr_v_object_id =
            scene_surface.vertices().find<IndexT>("sanity_check/object_id");
        UIPC_ASSERT(attr_v_object_id, "`sanity_check/object_id` is not found in scene surface");
        auto VObjectIds = attr_v_object_id->view();

        auto attr_thickness = scene_surface.vertices().find<Float>(builtin::thickness);
        span<const Float> VThickness =
            attr_thickness ? attr_thickness->view() : span<const Float>{};

        SizeT num_vertices = Vs.size();

        vector<Float> h_thickness(num_vertices, 0.0);
        if(VThickness.size() > 0)
        {
            for(SizeT i = 0; i < num_vertices; ++i)
                h_thickness[i] = VThickness[i];
        }

        SizeT CN = contact_tabular.element_count();
        vector<IndexT> h_contact_mask(CN * CN);
        for(IndexT i = 0; i < (IndexT)CN; ++i)
            for(IndexT j = 0; j < (IndexT)CN; ++j)
                h_contact_mask[i * CN + j] = contact_tabular.at(i, j).is_enabled() ? 1 : 0;

        SizeT SN = subscene_tabular.element_count();
        vector<IndexT> h_subscene_mask(SN * SN);
        for(IndexT i = 0; i < (IndexT)SN; ++i)
            for(IndexT j = 0; j < (IndexT)SN; ++j)
                h_subscene_mask[i * SN + j] = subscene_tabular.at(i, j).is_enabled() ? 1 : 0;

        DeviceBuffer<Vector3> positions(num_vertices);
        DeviceBuffer<Float>   thickness(num_vertices);
        DeviceBuffer<IndexT>  contact_ids(num_vertices);
        DeviceBuffer<IndexT>  subscene_ids(num_vertices);

        positions.view().copy_from(Vs.data());
        thickness.view().copy_from(h_thickness.data());
        contact_ids.view().copy_from(CIds.data());
        subscene_ids.view().copy_from(SCIds.data());

        DeviceBuffer2D<IndexT> contact_mask;
        contact_mask.resize(Extent2D{CN, CN});
        contact_mask.view().copy_from(h_contact_mask.data());

        DeviceBuffer2D<IndexT> subscene_mask;
        subscene_mask.resize(Extent2D{SN, SN});
        subscene_mask.view().copy_from(h_subscene_mask.data());

        struct HPInstance
        {
            Vector3 N, P;
            IndexT  cid, scid, geo_id, obj_id;
        };

        vector<HPInstance> hp_instances;
        for(auto& halfplane : halfplanes)
        {
            auto instance_count = halfplane->instances().size();
            auto Ns = halfplane->instances().find<Vector3>("N");
            UIPC_ASSERT(Ns, "Normal vector `N` not found in half-plane");
            auto Ps = halfplane->instances().find<Vector3>("P");
            UIPC_ASSERT(Ps, "Origin point `P` not found in half-plane");
            auto HGeoIds = halfplane->instances().find<IndexT>("sanity_check/geometry_id");
            UIPC_ASSERT(HGeoIds, "`sanity_check/geometry_id` not found in half-plane");
            auto HObjIds = halfplane->instances().find<IndexT>("sanity_check/object_id");
            UIPC_ASSERT(HObjIds, "`sanity_check/object_id` not found in half-plane");

            auto attr_cid  = halfplane->meta().find<IndexT>(builtin::contact_element_id);
            auto attr_scid = halfplane->meta().find<IndexT>(builtin::subscene_element_id);
            IndexT HCid  = attr_cid ? attr_cid->view()[0] : IndexT(0);
            IndexT HSCid = attr_scid ? attr_scid->view()[0] : IndexT(0);

            for(SizeT I = 0; I < instance_count; ++I)
                hp_instances.push_back({Ns->view()[I], Ps->view()[I],
                                        HCid, HSCid,
                                        HGeoIds->view()[I], HObjIds->view()[I]});
        }

        if(hp_instances.empty())
            return SanityCheckResult::Success;

        SizeT num_hp = hp_instances.size();

        vector<Vector3> h_hp_N(num_hp), h_hp_P(num_hp);
        vector<IndexT>  h_hp_cid(num_hp), h_hp_scid(num_hp);
        for(SizeT h = 0; h < num_hp; ++h)
        {
            h_hp_N[h]    = hp_instances[h].N;
            h_hp_P[h]    = hp_instances[h].P;
            h_hp_cid[h]  = hp_instances[h].cid;
            h_hp_scid[h] = hp_instances[h].scid;
        }

        DeviceBuffer<Vector3> hp_normals(num_hp);
        DeviceBuffer<Vector3> hp_origins(num_hp);
        DeviceBuffer<IndexT>  hp_contact_ids(num_hp);
        DeviceBuffer<IndexT>  hp_subscene_ids(num_hp);

        hp_normals.view().copy_from(h_hp_N.data());
        hp_origins.view().copy_from(h_hp_P.data());
        hp_contact_ids.view().copy_from(h_hp_cid.data());
        hp_subscene_ids.view().copy_from(h_hp_scid.data());

        DeviceBuffer<IndexT> vertex_too_close(num_vertices);
        vertex_too_close.fill(0);
        DeviceVar<IndexT> has_violation;
        BufferLaunch().fill(has_violation.view(), IndexT(0));

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(num_vertices,
                   [positions       = positions.cviewer().name("positions"),
                    thickness       = thickness.cviewer().name("thickness"),
                    contact_ids     = contact_ids.cviewer().name("contact_ids"),
                    subscene_ids    = subscene_ids.cviewer().name("subscene_ids"),
                    contact_mask    = contact_mask.cviewer().name("contact_mask"),
                    subscene_mask   = subscene_mask.cviewer().name("subscene_mask"),
                    hp_normals      = hp_normals.cviewer().name("hp_normals"),
                    hp_origins      = hp_origins.cviewer().name("hp_origins"),
                    hp_contact_ids  = hp_contact_ids.cviewer().name("hp_contact_ids"),
                    hp_subscene_ids = hp_subscene_ids.cviewer().name("hp_subscene_ids"),
                    vertex_too_close = vertex_too_close.viewer().name("vertex_too_close"),
                    has_violation    = has_violation.viewer().name("has_violation"),
                    num_hp] __device__(int i) mutable
                   {
                       for(SizeT h = 0; h < num_hp; ++h)
                       {
                           if(!subscene_mask(hp_subscene_ids(h), subscene_ids(i)))
                               continue;
                           if(!contact_mask(hp_contact_ids(h), contact_ids(i)))
                               continue;

                           Float d = (positions(i) - hp_origins(h)).dot(hp_normals(h))
                                     - thickness(i);
                           if(d <= 0.0)
                           {
                               vertex_too_close(i) = 1;
                               atomicMax(has_violation.data(), 1);
                               return;
                           }
                       }
                   });

        IndexT h_has_violation = has_violation;
        if(h_has_violation == 0)
            return SanityCheckResult::Success;

        vector<IndexT> h_vertex_too_close(num_vertices, 0);
        vertex_too_close.view().copy_to(h_vertex_too_close.data());

        map<Vector2i, Vector2i> close_geo_ids;
        for(SizeT vI = 0; vI < num_vertices; ++vI)
        {
            if(!h_vertex_too_close[vI])
                continue;
            for(auto& hp : hp_instances)
            {
                Float d = (Vs[vI] - hp.P).dot(hp.N) - h_thickness[vI];
                if(d <= 0.0)
                {
                    close_geo_ids[{(IndexT)VGeoIds[vI], hp.geo_id}] =
                        {(IndexT)VObjectIds[vI], hp.obj_id};
                }
            }
        }

        ::uipc::backend::SanityCheckMessageVisitor scmv{msg};
        auto& buffer = scmv.message();

        for(auto& [GeoIds, ObjIds] : close_geo_ids)
        {
            auto obj_0 = find_object(ObjIds[0]);
            auto obj_1 = find_object(ObjIds[1]);

            UIPC_ASSERT(obj_0 != nullptr, "Object[{}] not found", ObjIds[0]);
            UIPC_ASSERT(obj_1 != nullptr, "Object[{}] not found", ObjIds[1]);

            std::string name_0{obj_0->name()};
            std::string name_1{obj_1->name()};

            fmt::format_to(std::back_inserter(buffer),
                           "Geometry({}) in Object[{}({})] is too close (distance <= 0) to HalfPlane({}) in "
                           "Object[{}({})]\n",
                           GeoIds[0],
                           name_0,
                           obj_0->id(),
                           GeoIds[1],
                           name_1,
                           obj_1->id());
        }

        auto close_mesh = create_close_mesh(scene_surface, h_vertex_too_close);

        fmt::format_to(std::back_inserter(buffer),
                       "Close mesh has {} vertices, {} edges, {} triangles.\n",
                       close_mesh.vertices().size(),
                       close_mesh.edges().size(),
                       close_mesh.triangles().size());

        std::string name = "close_mesh";

        auto sanity_check_mode = ctx.config().find<std::string>("sanity_check/mode");
        if(sanity_check_mode->view()[0] == "normal")
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

        scmv.geometries()[name] =
            uipc::make_shared<geometry::SimplicialComplex>(std::move(close_mesh));

        return SanityCheckResult::Error;
    }

};

REGISTER_BACKEND_SANITY_CHECKER(HalfPlaneVertexDistanceCheck);
}  // namespace uipc::backend::cuda
