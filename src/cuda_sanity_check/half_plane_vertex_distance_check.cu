#include <uipc/core/contact_model.h>
#include <context.h>
#include <uipc/common/range.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/io/simplicial_complex_io.h>
#include <cuda_sanity_checker.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/io/spread_sheet_io.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/builtin/geometry_type.h>
#include <uipc/common/map.h>

// CUDA / muda includes
#include <type_define.h>
#include <muda/buffer.h>
#include <muda/launch.h>
#include <muda/cub/device/device_reduce.h>
#include <muda/ext/eigen/eigen_cxx20.h>

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

namespace uipc::cuda_sanity_check
{
class HalfPlaneVertexDistanceCheck final : public CudaSanityChecker
{
  public:
    constexpr static U64 SanityCheckerUID = 2;
    constexpr static U64 HalfPlaneUID     = 1;  // ImplicitGeometryUID = 1
    using CudaSanityChecker::CudaSanityChecker;

    struct Impl
    {
        // Returns number of violations for a single half-plane instance vs all vertices.
        // Also fills h_violated (size == num_vertices) with 1 for each violated vertex.
        int check_instance(span<const Vector3> Vs,
                           span<const Float>   h_thickness_span,
                           span<const IndexT>  h_enabled_span,
                           const Vector3&      plane_N,
                           const Vector3&      plane_P,
                           vector<IndexT>&     h_violated_out)
        {
            using namespace muda;

            SizeT num_vertices = Vs.size();

            muda::DeviceBuffer<Vector3> d_positions(num_vertices);
            d_positions.view().copy_from(Vs.data());

            muda::DeviceBuffer<Float> d_thickness(num_vertices);
            d_thickness.view().copy_from(h_thickness_span.data());

            muda::DeviceBuffer<IndexT> d_enabled(num_vertices);
            d_enabled.view().copy_from(h_enabled_span.data());

            muda::DeviceVar<int> d_violation_count(0);

            muda::DeviceBuffer<IndexT> d_vertex_violated(num_vertices);
            d_vertex_violated.fill(0);

            muda::ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(num_vertices,
                       [positions = d_positions.cviewer().name("positions"),
                        thickness = d_thickness.cviewer().name("thickness"),
                        enabled   = d_enabled.cviewer().name("enabled"),
                        violated  = d_vertex_violated.viewer().name("violated"),
                        violation = d_violation_count.viewer().name("violation"),
                        plane_N   = plane_N,
                        plane_P   = plane_P] __device__(int i) mutable
                       {
                           if(!enabled(i))
                               return;

                           const Vector3& V = positions(i);
                           Float d = (V - plane_P).dot(plane_N) - thickness(i);
                           if(d <= 0.0)
                           {
                               violated(i) = 1;
                               atomicAdd(violation.data(), 1);
                           }
                       });

            int h_violation_count = d_violation_count;

            h_violated_out.resize(num_vertices);
            d_vertex_violated.view().copy_to(h_violated_out.data());

            return h_violation_count;
        }
    };

  protected:
    vector<geometry::Geometry*> halfplanes;

    virtual void build(backend::SceneVisitor& scene) override
    {
        auto enable_contact = scene.config().find<IndexT>("contact/enable");
        if(!enable_contact->view()[0])
        {
            throw CudaSanityCheckerException("Contact is not enabled");
        }
    }

    virtual U64 get_id() const noexcept override { return SanityCheckerUID; }

    void collect_halfplanes(backend::SceneVisitor& scene)
    {
        halfplanes.clear();
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

        auto& contact_tabular  = context->contact_tabular();
        auto& subscene_tabular = context->subscene_tabular();

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

        auto attr_thickeness = scene_surface.vertices().find<Float>(builtin::thickness);
        span<const Float> VThickness =
            attr_thickeness ? attr_thickeness->view() : span<const Float>{};  // default 0.0

        auto& contact_table = context->contact_tabular();
        auto  objs          = this->objects();

        bool too_close = false;

        // key: {geo_id_0, geo_id_1}, value: {obj_id_0, obj_id_1}
        map<Vector2i, Vector2i> close_geo_ids;

        vector<IndexT> vertex_too_close(Vs.size(), 0);

        SizeT num_vertices = Vs.size();

        // Prepare thickness data: if no thickness attribute, fill with zeros
        vector<Float> h_thickness(num_vertices, 0.0);
        if(VThickness.size() > 0)
        {
            for(SizeT i = 0; i < num_vertices; ++i)
                h_thickness[i] = VThickness[i];
        }

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

            auto attr_scid = halfplane->meta().find<IndexT>(builtin::subscene_element_id);
            auto HSCid = attr_scid ? attr_scid->view()[0] : 0;

            for(auto I : range(instance_count))
            {
                const Vector3& N = Ns[I];
                const Vector3& P = Ps[I];

                // Build per-vertex enabled mask on CPU (contact/subscene filtering)
                vector<IndexT> h_enabled(num_vertices, 0);
                for(auto vI : range(num_vertices))
                {
                    const auto& SCM = subscene_tabular.at(HSCid, SCIds[vI]);
                    if(!SCM.is_enabled())
                        continue;

                    const auto& CM = contact_table.at(HCid, CIds[vI]);
                    if(!CM.is_enabled())
                        continue;

                    h_enabled[vI] = 1;
                }

                // Delegate GPU work to Impl
                vector<IndexT> h_violated;
                int h_violation_count = m_impl.check_instance(
                    Vs,
                    span<const Float>{h_thickness},
                    span<const IndexT>{h_enabled},
                    N,
                    P,
                    h_violated);

                if(h_violation_count > 0)
                {
                    too_close = true;

                    // Merge GPU results into CPU tracking structures
                    for(auto vI : range(num_vertices))
                    {
                        if(h_violated[vI])
                        {
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

            auto sanity_check_mode = scene.config().find<std::string>("sanity_check/mode");

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

            msg.geometries()[name] =
                uipc::make_shared<geometry::SimplicialComplex>(std::move(close_mesh));

            return SanityCheckResult::Error;
        }

        return SanityCheckResult::Success;
    };

  private:
    Impl m_impl;
};

REGISTER_CUDA_SANITY_CHECKER(HalfPlaneVertexDistanceCheck);
}  // namespace uipc::cuda_sanity_check
