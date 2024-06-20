#include <affine_body/affine_body_dynamics.h>
#include <Eigen/Dense>
#include <uipc/common/enumerate.h>
#include <uipc/common/range.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/algorithm/run_length_encode.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/common/zip.h>
#include <sim_engine.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(AffineBodyDynamics);

template <typename ViewGetterF, typename ForEachF>
void for_each_body(AffineBodyDynamics::Impl&       impl,
                   span<P<geometry::GeometrySlot>> geo_slots,
                   ViewGetterF&&                   getter,
                   ForEachF&&                      for_each)
{
    SizeT body_id = 0;
    for(auto&& [body_offset, body_count] : zip(impl.abd_geo_body_offsets, impl.abd_geo_body_counts))
    {
        auto& info = impl.h_body_infos[body_offset];
        auto& sc   = impl.geometry(geo_slots, info);

        auto attr_view = getter(sc);

        for(auto&& value : attr_view)
        {
            for_each(body_id++, value);
        }
    }
}

U<AffineBodyDynamics> AffineBodyDynamics::advanced_creator(SimEngine& engine)
{
    // if no AffineBody constitution, just return nullptr.

    auto  scene = engine.world().scene();
    auto& types = scene.constitution_tabular().types();
    if(types.find(world::ConstitutionTypes::AffineBody) == types.end())
        return nullptr;
    return std::make_unique<AffineBodyDynamics>(engine);
}

void AffineBodyDynamics::on_filter(Filter&& filter)
{
    m_impl.filters.push_back(std::move(filter));
}

void AffineBodyDynamics::build()
{
    // find dependent systems
    m_impl.global_vertex_manager = find<GlobalVertexManager>();

    // Register the action to initialize the affine body geometry
    on_init_scene([this] { m_impl.init_affine_body_geometry(world()); });

    // Register the action to build the vertex info
    m_impl.global_vertex_manager->on_update(
        [this](VertexCountInfo& info) { m_impl.report_vertex_count(info); },
        [this](const GlobalVertexInfo& info)
        { m_impl.receive_global_vertex_info(info); });

    // Register the action to write the scene
    on_write_scene([this] { m_impl.write_scene(world()); });
}

void AffineBodyDynamics::Impl::_find_affine_bodies(WorldVisitor& world)
{
    // 1) sort the filters by uid
    std::sort(filters.begin(),
              filters.end(),
              [](const auto& a, const auto& b) { return a.uid() < b.uid(); });

    vector<U64> filter_uids(filters.size());
    std::ranges::transform(filters,
                           filter_uids.begin(),
                           [](const Filter& filter) { return filter.uid(); });

    // 2) find the affine bodies
    list<BodyInfo> body_infos;
    auto           scene = world.scene();

    auto  geos              = scene.geometries();
    auto  N                 = geos.size();
    SizeT affine_body_count = 0;
    for(auto&& [i, geo_slot] : enumerate(geos))
    {
        auto& geo  = geo_slot->geometry();
        auto  cuid = geo.meta().find<U64>(builtin::constitution_uid);
        if(cuid)  // if has constitution uid
        {
            auto uid            = cuid->view()[0];
            auto instance_count = geo.instances().size();

            if(std::binary_search(filter_uids.begin(), filter_uids.end(), uid))
            {
                for(auto&& j : range(instance_count))
                {
                    // push all instances of the geometry to the body_infos
                    BodyInfo info;
                    info.constitution_uid        = uid;
                    info.abd_geometry_index      = abd_geo_count++;
                    info.global_geometry_index   = i;
                    info.geometry_instance_index = j;
                    info.affine_body_id          = affine_body_count++;

                    body_infos.push_back(std::move(info));
                }
            }
        }
    }

    // 3) sort the body infos by (constitution uid, geometry index, geometry instance index)
    h_body_infos.resize(body_infos.size());

    std::partial_sort_copy(body_infos.begin(),
                           body_infos.end(),
                           h_body_infos.begin(),
                           h_body_infos.end(),
                           [](const BodyInfo& a, const BodyInfo& b)
                           {
                               return a.constitution_uid < b.constitution_uid
                                      || a.constitution_uid == b.constitution_uid
                                             && a.global_geometry_index < b.global_geometry_index
                                      || a.constitution_uid == b.constitution_uid
                                             && a.global_geometry_index == b.global_geometry_index
                                             && a.geometry_instance_index < b.geometry_instance_index;
                           });

    // 4) setup abd_geo_body_offsets and abd_geo_body_counts
    abd_geo_body_offsets.reserve(h_body_infos.size());
    abd_geo_body_counts.reserve(h_body_infos.size());

    encode_offset_count(h_body_infos.begin(),
                        h_body_infos.end(),
                        std::back_inserter(abd_geo_body_offsets),
                        std::back_inserter(abd_geo_body_counts),
                        [](const BodyInfo& a, const BodyInfo& b) {
                            return a.global_geometry_index == b.global_geometry_index;
                        });

    UIPC_ASSERT(abd_geo_count
                    == abd_geo_body_offsets.back() + abd_geo_body_counts.back(),
                "abd_geo_count({}) != geo_body_offsets.back()({}) + geo_body_counts.back()({}). Why can it happen?",
                abd_geo_count,
                abd_geo_body_offsets.back(),
                abd_geo_body_counts.back());
}

void AffineBodyDynamics::Impl::_build_geometry_on_host(WorldVisitor& world)
{
    auto scene     = world.scene();
    auto geo_slots = scene.geometries();

    // 1) setup `q` for every affine body
    h_body_id_to_q.resize(h_body_infos.size());
    for_each_body(
        *this,
        geo_slots,
        [](geometry::SimplicialComplex& sc) { return sc.transforms().view(); },
        [&](SizeT i, const Matrix4x4& trans)
        {
            Vector12& q     = h_body_id_to_q[i];
            q.segment<3>(0) = trans.block<3, 1>(0, 3);

            Float D = trans.block<3, 3>(0, 0).determinant();

            if(!Eigen::Vector<Float, 1>{D}.isApproxToConstant(1.0, 1e-6))
                spdlog::warn("The determinant of the rotation matrix is not 1.0, but {}. Don't apply scaling on Affine Body.",
                             D);

            q.segment<3>(3) = trans.block<1, 3>(0, 0).transpose();
            q.segment<3>(6) = trans.block<1, 3>(1, 0).transpose();
            q.segment<3>(9) = trans.block<1, 3>(2, 0).transpose();
        });


    // 2) setup `J` and `mass` for every vertex
    // a. count the number and offset of the vertices
    vector<SizeT> vertex_count(h_body_infos.size());
    vector<SizeT> vertex_offset(h_body_infos.size());

    std::ranges::transform(h_body_infos,
                           vertex_count.begin(),
                           [&](const BodyInfo& info) -> SizeT
                           {
                               auto& sc = this->geometry(geo_slots, info);
                               return sc.positions().size();
                           });

    std::exclusive_scan(vertex_count.begin(), vertex_count.end(), vertex_offset.begin(), 0);

    auto total = vertex_offset.back() + vertex_count.back();

    // b. setup the J and mass
    h_vertex_id_to_J.resize(total);
    h_vertex_id_to_mass.resize(total);
    span Js          = h_vertex_id_to_J;
    span vertex_mass = h_vertex_id_to_mass;
    std::ranges::for_each(h_body_infos,
                          [&](BodyInfo& info)
                          {
                              auto& sc = geometry(geo_slots, info);

                              auto offset = vertex_offset[info.affine_body_id];
                              auto count  = vertex_count[info.affine_body_id];

                              // setup the vertex info
                              info.vertex_offset = offset;
                              info.vertex_count  = count;

                              // copy init the J from position
                              auto pos_view = sc.positions().view();
                              auto sub_Js   = Js.subspan(offset, count);
                              std::ranges::copy(pos_view, sub_Js.begin());

                              auto mass = sc.vertices().find<Float>(builtin::mass);
                              UIPC_ASSERT(mass, "The mass attribute is not found in the geometry. Why can it happen?");
                              auto mass_view = mass->view();

                              auto sub_mass = vertex_mass.subspan(offset, count);
                              std::ranges::copy(mass_view, sub_mass.begin());
                          });


    // 3) setup `vertex_id_to_body_id`
    h_vertex_id_to_body_id.resize(total);
    auto v2b = span{h_vertex_id_to_body_id};
    std::ranges::for_each(h_body_infos,
                          [&](const BodyInfo& info)
                          {
                              auto offset = vertex_offset[info.affine_body_id];
                              auto count  = vertex_count[info.affine_body_id];
                              std::ranges::fill(v2b.subspan(offset, count), info.affine_body_id);
                          });

    // 4) setup body_abd_mass and body_id_to_volume
    vector<ABDJacobiDyadicMass> geo_masses(abd_geo_count, ABDJacobiDyadicMass{});
    vector<Float> geo_volumes(abd_geo_count, 0.0);

    h_body_id_to_abd_mass.resize(h_body_infos.size());
    h_body_id_to_volume.resize(h_body_infos.size());

    for(const auto& info : h_body_infos)
    {
        auto& geo_mass = geo_masses[info.abd_geometry_index];
        auto& geo_vol  = geo_volumes[info.abd_geometry_index];

        if(geo_mass.mass() == 0.0 || geo_vol == 0.0)  // if not computed yet
        {
            auto& sc = geometry(geo_slots, info);

            auto offset = vertex_offset[info.affine_body_id];
            auto count  = vertex_count[info.affine_body_id];

            // compute the mass of the geometry
            // sub_Js for this body
            auto sub_Js   = Js.subspan(offset, count);
            auto sub_mass = vertex_mass.subspan(offset, count);
            for(auto&& [m, J] : zip(sub_mass, sub_Js))
                geo_mass += ABDJacobiDyadicMass{m, J.x_bar()};

            // compute the volume of the geometry
            auto Vs = sc.positions().view();
            auto Ts = sc.tetrahedra().topo().view();

            for(const auto& t : Ts)
            {
                auto [p0, p1, p2, p3] =
                    std::tuple{Vs[t[0]], Vs[t[1]], Vs[t[2]], Vs[t[3]]};

                Eigen::Matrix<Float, 3, 3> A;
                A.col(0) = p1 - p0;
                A.col(1) = p2 - p0;
                A.col(2) = p3 - p0;
                auto D   = A.determinant();
                UIPC_ASSERT(D > 0.0,
                            "The determinant of the tetrahedron is non-positive ({}), which means the tetrahedron is inverted.",
                            D);

                auto volume = D / 6.0;
                geo_vol += volume;
            }
        }

        h_body_id_to_abd_mass[info.affine_body_id] = geo_mass;
        h_body_id_to_volume[info.affine_body_id]   = geo_vol;
    }

    // 5) compute the inverse of the mass matrix
    h_body_id_to_abd_mass_inv.resize(h_body_infos.size());
    std::ranges::transform(h_body_id_to_abd_mass,
                           h_body_id_to_abd_mass_inv.begin(),
                           [](const ABDJacobiDyadicMass& mass) -> Matrix12x12
                           { return mass.to_mat().inverse(); });

    // 6) setup the affine body gravity
    Vector3 gravity = scene.info()["gravity"];
    h_body_id_to_abd_gravity.resize(h_body_infos.size(), Vector12::Zero());
    if(!gravity.isApprox(Vector3::Zero()))
    {
        vector<Vector12> geo_gravities(abd_geo_count, Vector12::Zero());

        std::ranges::transform(
            h_body_infos,
            h_body_id_to_abd_gravity.begin(),
            [&](const BodyInfo& info) -> Vector12
            {
                auto& g = geo_gravities[info.abd_geometry_index];

                if(g.isZero())  // if not computed yet
                {
                    auto offset = vertex_offset[info.affine_body_id];
                    auto count  = vertex_count[info.affine_body_id];

                    // sub_Js for this body
                    auto sub_Js   = Js.subspan(offset, count);
                    auto sub_mass = vertex_mass.subspan(offset, count);

                    Vector12 G = Vector12::Zero();

                    for(auto&& [m, J] : zip(sub_mass, sub_Js))
                        G += m * (J.T() * gravity);

                    const Matrix12x12& M_inv =
                        h_body_id_to_abd_mass_inv[info.affine_body_id];
                    g = M_inv * G;
                }

                return g;
            });
    }
}

void AffineBodyDynamics::Impl::_build_geometry_on_device(WorldVisitor& world)
{
    auto async_copy = []<typename T>(span<T> src, muda::DeviceBuffer<T>& dst)
    {
        muda::BufferLaunch().resize<T>(dst, src.size());
        muda::BufferLaunch().copy<T>(dst.view(), src.data());
    };

    async_copy(span{h_body_id_to_q}, body_id_to_q);
    async_copy(span{h_vertex_id_to_J}, vertex_id_to_J);
    async_copy(span{h_vertex_id_to_body_id}, vertex_id_to_body_id);
    async_copy(span{h_body_id_to_abd_mass}, body_id_to_abd_mass);
    async_copy(span{h_body_id_to_volume}, body_id_to_volume);
    async_copy(span{h_body_id_to_abd_mass_inv}, body_id_to_abd_mass_inv);
    async_copy(span{h_body_id_to_abd_gravity}, body_id_to_abd_gravity);

    auto async_transfer =
        []<typename T>(muda::DeviceBuffer<T>& src, muda::DeviceBuffer<T> dst)
    {
        muda::BufferLaunch().resize<T>(dst, src.size());
        muda::BufferLaunch().copy<T>(dst.view(), src.view());
    };

    async_transfer(body_id_to_q, body_id_to_q_temp);
    async_transfer(body_id_to_q, body_id_to_q_tilde);
    async_transfer(body_id_to_q, body_id_to_q_prev);

    auto async_resize = []<typename T>(muda::DeviceBuffer<T>& buf, SizeT size, const T& value)
    {
        muda::BufferLaunch().resize<T>(buf, size);
        muda::BufferLaunch().fill<T>(buf.view(), value);
    };

    async_resize(body_id_to_q_v, h_body_infos.size(), Vector12::Zero().eval());
    async_resize(body_id_to_dq, h_body_infos.size(), Vector12::Zero().eval());

    muda::wait_device();
}

void AffineBodyDynamics::Impl::init_affine_body_geometry(WorldVisitor& world)
{
    spdlog::info("Initializing Affine Body Geometry");

    _find_affine_bodies(world);
    _build_geometry_on_host(world);
    _build_geometry_on_device(world);
}

void AffineBodyDynamics::Impl::report_vertex_count(VertexCountInfo& vertex_count_info)
{
    // TODO: now we just report all the affine body vertices
    // later we may extract the surface vertices and report them
    vertex_count_info.count(h_vertex_id_to_J.size());
}

void AffineBodyDynamics::Impl::receive_global_vertex_info(const GlobalVertexInfo& global_vertex_info)
{
    using namespace muda;
    // fill the coindex for later use
    auto N = global_vertex_info.coindex().size();
    // TODO: now we just use `iota` the coindex
    // later we may extract the surface vertices as the reported vertices
    // then the coindex will be a mapping from the surface vertices to the affine body vertices
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(N,
               [coindex = global_vertex_info.coindex().viewer().name(
                    "coindex")] __device__(int i) mutable { coindex(i) = i; });

    // record the global vertex info
    abd_global_vertex_offset = global_vertex_info.offset();
    abd_global_vertex_count  = global_vertex_info.count();
}

void AffineBodyDynamics::Impl::write_scene(WorldVisitor& world)
{
    // 1) download from device to host
    _download_geometry_to_host();

    auto scene     = world.scene();
    auto geo_slots = scene.geometries();

    span qs = h_body_id_to_q;

    // 2) transfer from abd qs to transforms
    for_each_body(
        *this,
        geo_slots,
        [](geometry::SimplicialComplex& sc) { return view(sc.transforms()); },
        [&](SizeT i, Matrix4x4& trans)
        {
            Vector12& q = h_body_id_to_q[i];

            trans                   = Matrix4x4::Identity();
            trans.block<3, 1>(0, 3) = q.segment<3>(0);  // translation
            // rotation
            trans.block<1, 3>(0, 0) = q.segment<3>(3).transpose();
            trans.block<1, 3>(1, 0) = q.segment<3>(6).transpose();
            trans.block<1, 3>(2, 0) = q.segment<3>(9).transpose();
        });
}

void AffineBodyDynamics::Impl::_download_geometry_to_host()
{
    using namespace muda;

    auto aync_copy = []<typename T>(muda::DeviceBuffer<T>& src, span<T> dst)
    { muda::BufferLaunch().copy<T>(dst.data(), src.view()); };


    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(body_id_to_q.size(),
               [qs = body_id_to_q.viewer().name("qs")] __device__(int i) mutable
               {
                   // move it up by 1 unit, just for testing
                   qs(i).segment<3>(0) += Vector3::UnitY();
               });


    aync_copy(body_id_to_q, span{h_body_id_to_q});

    muda::wait_device();
}

geometry::SimplicialComplex& AffineBodyDynamics::Impl::geometry(
    span<P<geometry::GeometrySlot>> geo_slots, const BodyInfo body_info)
{
    auto& geo = geo_slots[body_info.global_geometry_index]->geometry();
    auto  sc  = geo.as<geometry::SimplicialComplex>();
    UIPC_ASSERT(sc,
                "The geometry is not a simplicial complex (it's {}). Why can it happen?",
                geo.type());
    return *sc;
}

AffineBodyDynamics::Filter::Filter(U64 constitution_uid,
                                   std::function<void(const FilteredInfo&)>&& action) noexcept
    : m_constitution_uid{constitution_uid}
    , m_action{std::move(action)}
{
}
}  // namespace uipc::backend::cuda