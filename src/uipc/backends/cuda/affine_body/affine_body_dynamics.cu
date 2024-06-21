#include <affine_body/affine_body_dynamics.h>
#include <Eigen/Dense>
#include <uipc/common/enumerate.h>
#include <uipc/common/range.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/algorithm/run_length_encode.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/common/zip.h>
#include <muda/cub/device/device_reduce.h>

namespace uipc::backend::cuda
{
U<AffineBodyDynamics> AffineBodyDynamics::advanced_creator(SimEngine& engine)
{
    // if no AffineBody constitution, just return nullptr.

    auto  scene = engine.world().scene();
    auto& types = scene.constitution_tabular().types();
    if(types.find(world::ConstitutionTypes::AffineBody) == types.end())
        return nullptr;
    return std::make_unique<AffineBodyDynamics>(engine);
}

REGISTER_SIM_SYSTEM(AffineBodyDynamics);

void AffineBodyDynamics::build()
{
    // find dependent systems
    auto global_vertex_manager = find<GlobalVertexManager>();
    auto dof_predictor         = find<DoFPredictor>();
    auto line_searcher         = find<LineSearcher>();

    // Register the action to initialize the affine body geometry
    on_init_scene([this] { m_impl.init_affine_body_geometry(world()); });

    // Register the action to build the vertex info
    global_vertex_manager->on_update(
        [this](GlobalVertexManager::VertexCountInfo& info)
        { m_impl.report_vertex_count(info); },
        [this](const GlobalVertexManager::VertexAttributes& info)
        { m_impl.receive_global_vertex_info(info); },
        [this](const GlobalVertexManager::VertexDisplacement& info)
        {
            // TODO
            // when we solve the system, we can fill out the displacement
        });

    // Register the action to predict the affine body dof
    dof_predictor->on_predict([this](const DoFPredictor::PredictInfo& info)
                              { m_impl.compute_q_tilde(info); });


    line_searcher->on_record_current_state([this] { m_impl.copy_q_to_q_temp(); });
    line_searcher->on_step_forward([this](const LineSearcher::StepInfo& info)
                                   { m_impl.step_forward(info); });
    line_searcher->on_compute_energy("abd_kinetic_energy",
                                     [this](const LineSearcher::ComputeEnergyInfo& info) {
                                         return m_impl.compute_abd_kinetic_energy(info);
                                     });


    // Register the action to compute the velocity of the affine body dof
    dof_predictor->on_compute_velocity([this](const DoFPredictor::PredictInfo& info)
                                       { m_impl.compute_q_v(info); });

    // Register the action to write the scene
    on_write_scene([this] { m_impl.write_scene(world()); });
}
}  // namespace uipc::backend::cuda


namespace uipc::backend::cuda
{
template <typename ViewGetterF, typename ForEachF>
void for_each_body(AffineBodyDynamics::Impl&       m_impl,
                   span<P<geometry::GeometrySlot>> geo_slots,
                   ViewGetterF&&                   getter,
                   ForEachF&&                      for_each)
{
    SizeT body_id = 0;
    for(auto&& [body_offset, body_count] :
        zip(m_impl.h_abd_geo_body_offsets, m_impl.h_abd_geo_body_counts))
    {
        auto& info = m_impl.h_body_infos[body_offset];
        auto& sc   = m_impl.geometry(geo_slots, info);

        auto attr_view = getter(sc);

        for(auto&& value : attr_view)
        {
            for_each(body_id++, value);
        }
    }
}

void AffineBodyDynamics::on_filter(Filter&& filter)
{
    m_impl.filters.push_back(std::move(filter));
}

void AffineBodyDynamics::Impl::_build_affine_body_infos(WorldVisitor& world)
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

    auto  geo_slots         = scene.geometries();
    auto  N                 = geo_slots.size();
    SizeT affine_body_count = 0;
    for(auto&& [i, geo_slot] : enumerate(geo_slots))
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

                    auto& sc          = geometry(geo_slots, info);
                    info.vertex_count = sc.positions().size();

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

    // 4) setup h_abd_geo_body_offsets and h_abd_geo_body_counts
    h_abd_geo_body_offsets.reserve(h_body_infos.size());
    h_abd_geo_body_counts.reserve(h_body_infos.size());

    encode_offset_count(h_body_infos.begin(),
                        h_body_infos.end(),
                        std::back_inserter(h_abd_geo_body_offsets),
                        std::back_inserter(h_abd_geo_body_counts),
                        [](const BodyInfo& a, const BodyInfo& b) {
                            return a.global_geometry_index == b.global_geometry_index;
                        });

    UIPC_ASSERT(abd_geo_count
                    == h_abd_geo_body_offsets.back() + h_abd_geo_body_counts.back(),
                "abd_geo_count({}) != geo_body_offsets.back()({}) + geo_body_counts.back()({}). Why can it happen?",
                abd_geo_count,
                h_abd_geo_body_offsets.back(),
                h_abd_geo_body_counts.back());

    // 5) setup the vertex offset, count
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
    abd_vertex_count = vertex_offset.back() + vertex_count.back();
    for(auto&& [info, count, offset] : zip(h_body_infos, vertex_count, vertex_offset))
    {
        info.vertex_count  = count;
        info.vertex_offset = offset;
    }
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
    h_vertex_id_to_J.resize(abd_vertex_count);
    h_vertex_id_to_mass.resize(abd_vertex_count);
    span Js          = h_vertex_id_to_J;
    span vertex_mass = h_vertex_id_to_mass;
    std::ranges::for_each(h_body_infos,
                          [&](BodyInfo& info)
                          {
                              auto& sc = geometry(geo_slots, info);

                              auto offset = info.vertex_offset;
                              auto count  = info.vertex_count;

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
    h_vertex_id_to_body_id.resize(abd_vertex_count);
    auto v2b = span{h_vertex_id_to_body_id};
    std::ranges::for_each(h_body_infos,
                          [&](const BodyInfo& info)
                          {
                              auto offset = info.vertex_offset;
                              auto count  = info.vertex_count;
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

            auto offset = info.vertex_offset;
            auto count  = info.vertex_count;

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

        std::ranges::transform(h_body_infos,
                               h_body_id_to_abd_gravity.begin(),
                               [&](const BodyInfo& info) -> Vector12
                               {
                                   auto& g = geo_gravities[info.abd_geometry_index];

                                   if(g.isZero())  // if not computed yet
                                   {
                                       auto offset = info.vertex_offset;
                                       auto count  = info.vertex_count;

                                       // sub_Js for this body
                                       auto sub_Js = Js.subspan(offset, count);
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

    // 7) setup the boundary type
    h_body_id_to_is_fixed.resize(h_body_infos.size(), 0);
    for_each_body(
        *this,
        geo_slots,
        [](geometry::SimplicialComplex& sc)
        { return sc.instances().find<IndexT>(builtin::is_fixed)->view(); },
        [&](SizeT i, IndexT fixed) { h_body_id_to_is_fixed[i] = fixed; });
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
    async_copy(span{h_body_id_to_is_fixed}, body_id_to_is_fixed);

    auto async_transfer = []<typename T>(const muda::DeviceBuffer<T>& src,
                                         muda::DeviceBuffer<T>&       dst)
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

    _build_affine_body_infos(world);
    _build_geometry_on_host(world);
    _build_geometry_on_device(world);
}

void AffineBodyDynamics::Impl::report_vertex_count(GlobalVertexManager::VertexCountInfo& vertex_count_info)
{
    // TODO: now we just report all the affine body vertices
    // later we may extract the surface vertices and report them
    vertex_count_info.count(h_vertex_id_to_J.size());
}

void AffineBodyDynamics::Impl::receive_global_vertex_info(const GlobalVertexManager::VertexAttributes& vertex_attributes)
{
    using namespace muda;
    // fill the coindex for later use
    auto N = vertex_attributes.coindex().size();
    // TODO: now we just use `iota` the coindex
    // later we may extract the surface vertices as the reported vertices
    // then the coindex will be a mapping from the surface vertices to the affine body vertices
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(N,
               [coindex = vertex_attributes.coindex().viewer().name("coindex"),

                dst_pos = vertex_attributes.positions().viewer().name("dst_pos"),
                v2b = vertex_id_to_body_id.cviewer().name("v2b"),
                qs  = body_id_to_q.cviewer().name("qs"),
                src_pos = vertex_id_to_J.cviewer().name("src_pos")] __device__(int i) mutable
               {
                   coindex(i) = i;

                   auto        body_id = v2b(i);
                   const auto& q       = qs(body_id);
                   dst_pos(i)          = src_pos(i).point_x(q);
               });

    // record the global vertex info
    abd_report_vertex_offset = vertex_attributes.coindex().offset();
    abd_report_vertex_count  = vertex_attributes.coindex().size();
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

void AffineBodyDynamics::Impl::compute_q_tilde(const DoFPredictor::PredictInfo& info)
{
    using namespace muda;
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(body_count(),
               [is_fixed = body_id_to_is_fixed.cviewer().name("btype"),
                q_prevs  = body_id_to_q_prev.cviewer().name("q_prev"),
                q_vs     = body_id_to_q_v.cviewer().name("q_velocities"),
                q_tildes = body_id_to_q_tilde.viewer().name("q_tilde"),
                affine_gravity = body_id_to_abd_gravity.cviewer().name("affine_gravity"),
                dt = info.dt] __device__(int i) mutable
               {
                   auto& q_prev = q_prevs(i);
                   auto& q_v    = q_vs(i);
                   auto& g      = affine_gravity(i);
                   // TODO: this time, we only consider gravity
                   if(is_fixed(i))
                   {
                       q_tildes(i) = q_prev;
                   }
                   else
                   {
                       q_tildes(i) = q_prev + q_v * dt + g * (dt * dt);
                   }
               });
}

void AffineBodyDynamics::Impl::compute_q_v(const DoFPredictor::PredictInfo& info)
{
    using namespace muda;
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(body_count(),
               [is_fixed = body_id_to_is_fixed.cviewer().name("btype"),
                qs       = body_id_to_q.cviewer().name("qs"),
                q_vs     = body_id_to_q_v.viewer().name("q_vs"),
                q_prevs  = body_id_to_q_prev.viewer().name("q_prevs"),
                dt       = info.dt] __device__(int i) mutable
               {
                   auto& q_v    = q_vs(i);
                   auto& q_prev = q_prevs(i);

                   const auto& q = qs(i);

                   if(is_fixed(i))
                       q_v = Vector12::Zero();
                   else
                       q_v = (q - q_prev) * (1.0 / dt);
                   q_prev = q;
               });
}

void AffineBodyDynamics::Impl::copy_q_to_q_temp()
{
    body_id_to_q_temp = body_id_to_q;
}

void AffineBodyDynamics::Impl::step_forward(const LineSearcher::StepInfo& info)
{
    using namespace muda;
    ParallelFor(256)
        .kernel_name(__FUNCTION__ "-affine_body")
        .apply(body_count(),
               [is_fixed = body_id_to_is_fixed.cviewer().name("is_fixed"),
                q_temps  = body_id_to_q_temp.cviewer().name("q_temps"),
                qs       = body_id_to_q.viewer().name("qs"),
                dqs      = body_id_to_dq.cviewer().name("dqs"),
                alpha    = info.alpha] __device__(int i) mutable
               {
                   if(is_fixed(i))
                       return;
                   qs(i) = q_temps(i) - alpha * dqs(i);
               });
}

Float AffineBodyDynamics::Impl::compute_abd_kinetic_energy(const LineSearcher::ComputeEnergyInfo& info)
{
    using namespace muda;

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(body_count(),
               [is_fixed = body_id_to_is_fixed.cviewer().name("is_fixed"),
                qs       = body_id_to_q.cviewer().name("qs"),
                q_tildes = body_id_to_q_tilde.viewer().name("q_tildes"),
                masses   = body_id_to_abd_mass.cviewer().name("masses"),
                Ks = body_id_to_kinetic_energy.viewer().name("kinetic_energy")] __device__(int i) mutable
               {
                   auto& K = Ks(i);
                   if(is_fixed(i))
                   {
                       K = 0.0;
                   }
                   else
                   {
                       const auto& q       = qs(i);
                       const auto& q_tilde = q_tildes(i);
                       const auto& M       = masses(i);
                       Vector12    dq      = q - q_tilde;
                       K                   = 0.5 * dq.dot(M * dq);
                   }
               });

    DeviceReduce().Sum(body_id_to_kinetic_energy.data(),
                       abd_kinetic_energy.data(),
                       body_id_to_kinetic_energy.size());

    return abd_kinetic_energy;
}

AffineBodyDynamics::Filter::Filter(U64 constitution_uid,
                                   std::function<void(const FilteredInfo&)>&& action) noexcept
    : m_constitution_uid{constitution_uid}
    , m_action{std::move(action)}
{
}
}  // namespace uipc::backend::cuda