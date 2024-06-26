#include <affine_body/affine_body_dynamics.h>
#include <affine_body/abd_line_search_reporter.h>
#include <affine_body/affine_body_constitution.h>

#include <Eigen/Dense>
#include <uipc/common/enumerate.h>
#include <uipc/common/range.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/algorithm/run_length_encode.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/common/zip.h>
#include <muda/cub/device/device_reduce.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>
#include <muda/ext/eigen/evd.h>

namespace uipc::backend::cuda
{
// specialization of the SimSystemCreator for AffineBodyDynamics, to add some logic to create the system
template <>
class SimSystemCreator<AffineBodyDynamics>
{
  public:
    static U<AffineBodyDynamics> create(SimEngine& engine)
    {
        return has_affine_body_constitution(engine) ?
                   make_unique<AffineBodyDynamics>(engine) :
                   nullptr;
    }
};

REGISTER_SIM_SYSTEM(AffineBodyDynamics);

void AffineBodyDynamics::do_build()
{
    // find dependent systems
    auto dof_predictor             = find<DoFPredictor>();
    auto gradient_hessian_computer = find<GradientHessianComputer>();

    // Register the action to initialize the affine body geometry
    on_init_scene([this] { m_impl.init_affine_body_geometry(world()); });

    // Register the action to predict the affine body dof
    dof_predictor->on_predict([this](DoFPredictor::PredictInfo& info)
                              { m_impl.compute_q_tilde(info); });

    // Register the action to compute the gradient and hessian
    gradient_hessian_computer->on_compute_gradient_hessian(
        [this](GradientHessianComputer::ComputeInfo& info)
        { m_impl.compute_gradient_hessian(info); });

    // Register the action to compute the velocity of the affine body dof
    dof_predictor->on_compute_velocity([this](DoFPredictor::PredictInfo& info)
                                       { m_impl.compute_q_v(info); });

    // Register the action to write the scene
    on_write_scene([this] { m_impl.write_scene(world()); });
}
}  // namespace uipc::backend::cuda


namespace uipc::backend::cuda
{
void AffineBodyDynamics::add_constitution(AffineBodyConstitution* constitution)
{
    check_state(SimEngineState::BuildSystems, "add_constitution()");
    // set the temp index, later we will sort constitution by uid
    // and reset the index
    constitution->m_index = m_impl.constitution_buffer.size();
    m_impl.constitution_buffer.push_back(constitution);
}

void AffineBodyDynamics::Impl::_build_body_infos(WorldVisitor& world)
{
    // 1) sort the constitutions by uid
    constitutions.resize(constitution_buffer.size());
    std::ranges::move(constitution_buffer, constitutions.begin());
    std::sort(constitutions.begin(),
              constitutions.end(),
              [](const AffineBodyConstitution* a, AffineBodyConstitution* b)
              { return a->constitution_uid() < b->constitution_uid(); });
    for(auto&& [i, constitution] : enumerate(constitutions))  // reset the index
        constitution->m_index = i;

    vector<U64> filter_uids(constitutions.size());
    std::ranges::transform(constitutions,
                           filter_uids.begin(),
                           [](const AffineBodyConstitution* filter)
                           { return filter->constitution_uid(); });

    // 2) find the affine bodies
    list<BodyInfo> body_infos;
    auto           scene = world.scene();

    auto geo_slots = scene.geometries();
    auto N         = geo_slots.size();
    abd_body_count = 0;
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
                auto* sc = geo.as<geometry::SimplicialComplex>();
                UIPC_ASSERT(sc,
                            "The geometry is not a simplicial complex (it's {}). Why can it happen?",
                            geo.type());

                auto vert_count = sc->vertices().size();
                for(auto&& j : range(instance_count))
                {
                    // push all instances of the geometry to the body_infos
                    BodyInfo info;
                    info.m_constitution_uid        = uid;
                    info.m_abd_geometry_index      = abd_geo_count;
                    info.m_geometry_slot_index     = i;
                    info.m_geometry_instance_index = j;
                    info.m_affine_body_id          = abd_body_count++;
                    info.m_vertex_count            = vert_count;
                    body_infos.push_back(std::move(info));
                }
                abd_geo_count++;
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
                               return a.m_constitution_uid < b.m_constitution_uid
                                      || a.m_constitution_uid == b.m_constitution_uid
                                             && a.m_geometry_slot_index < b.m_geometry_slot_index
                                      || a.m_constitution_uid == b.m_constitution_uid
                                             && a.m_geometry_slot_index == b.m_geometry_slot_index
                                             && a.m_geometry_instance_index
                                                    < b.m_geometry_instance_index;
                           });

    // 4) setup the vertex offset, count
    vector<SizeT> vertex_count(h_body_infos.size());
    vector<SizeT> vertex_offset(h_body_infos.size());
    std::ranges::transform(h_body_infos,
                           vertex_count.begin(),
                           [&](const BodyInfo& info) -> SizeT
                           { return info.m_vertex_count; });

    std::exclusive_scan(vertex_count.begin(), vertex_count.end(), vertex_offset.begin(), 0);
    abd_vertex_count = vertex_offset.back() + vertex_count.back();
    for(auto&& [info, count, offset] : zip(h_body_infos, vertex_count, vertex_offset))
    {
        info.m_vertex_count  = count;
        info.m_vertex_offset = offset;
    }
}

void AffineBodyDynamics::Impl::_build_related_infos(WorldVisitor& world)
{
    // 1) setup h_abd_geo_body_offsets and h_abd_geo_body_counts
    {
        h_abd_geo_body_offsets.reserve(h_body_infos.size());
        h_abd_geo_body_counts.reserve(h_body_infos.size());

        encode_offset_count(h_body_infos.begin(),
                            h_body_infos.end(),
                            std::back_inserter(h_abd_geo_body_offsets),
                            std::back_inserter(h_abd_geo_body_counts),
                            [](const BodyInfo& a, const BodyInfo& b) {
                                return a.m_geometry_slot_index == b.m_geometry_slot_index;
                            });

        UIPC_ASSERT(abd_body_count
                        == h_abd_geo_body_offsets.back() + h_abd_geo_body_counts.back(),
                    "abd_body_count({}) != geo_body_offsets.back()({}) + geo_body_counts.back()({}). Why can it happen?",
                    abd_body_count,
                    h_abd_geo_body_offsets.back(),
                    h_abd_geo_body_counts.back());
    }

    // 2) setup h_constitution_geo_offsets and h_constitution_geo_counts
    {
        h_constitution_geo_offsets.reserve(constitutions.size());
        h_constitution_geo_counts.reserve(constitutions.size());

        encode_offset_count(h_abd_geo_body_offsets.begin(),
                            h_abd_geo_body_offsets.end(),
                            std::back_inserter(h_constitution_geo_offsets),
                            std::back_inserter(h_constitution_geo_counts),
                            [&](SizeT a, SizeT b)
                            {
                                const auto& info_a = h_body_infos[a];
                                const auto& info_b = h_body_infos[b];
                                return info_a.m_constitution_uid == info_b.m_constitution_uid;
                            });

        UIPC_ASSERT(abd_geo_count
                        == h_constitution_geo_offsets.back()
                               + h_constitution_geo_counts.back(),
                    "abd_body_count({}) != constitution_body_offsets.back()({}) + constitution_body_counts.back()({}). Why can it happen?",
                    abd_body_count,
                    h_constitution_geo_offsets.back(),
                    h_constitution_geo_counts.back());
    }

    // 3) setup h_constitution_body_offsets and h_constitution_body_counts
    {
        h_constitution_body_offsets.reserve(constitutions.size());
        h_constitution_body_counts.reserve(constitutions.size());

        encode_offset_count(h_body_infos.begin(),
                            h_body_infos.end(),
                            std::back_inserter(h_constitution_body_offsets),
                            std::back_inserter(h_constitution_body_counts),
                            [&](const BodyInfo& a, const BodyInfo& b) {
                                return a.m_constitution_uid == b.m_constitution_uid;
                            });

        UIPC_ASSERT(abd_body_count
                        == h_constitution_body_offsets.back()
                               + h_constitution_body_counts.back(),
                    "abd_body_count({}) != constitution_body_offsets.back()({}) + constitution_body_counts.back()({}). Why can it happen?",
                    abd_body_count,
                    h_constitution_body_offsets.back(),
                    h_constitution_body_counts.back());
    }
}

void AffineBodyDynamics::Impl::_build_geometry_on_host(WorldVisitor& world)
{
    auto scene     = world.scene();
    auto geo_slots = scene.geometries();

    // 1) setup `q` for every affine body
    h_body_id_to_q.resize(h_body_infos.size());
    for_each_body(
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

                              auto offset = info.m_vertex_offset;
                              auto count  = info.m_vertex_count;

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
                              auto offset = info.m_vertex_offset;
                              auto count  = info.m_vertex_count;
                              std::ranges::fill(v2b.subspan(offset, count), info.m_affine_body_id);
                          });

    // 4) setup body_abd_mass and body_id_to_volume
    vector<ABDJacobiDyadicMass> geo_masses(abd_geo_count, ABDJacobiDyadicMass{});
    vector<Float> geo_volumes(abd_geo_count, 0.0);

    for(auto&& [I, body_offset] : enumerate(h_abd_geo_body_offsets))
    {
        const auto& info     = h_body_infos[body_offset];
        auto&       geo_mass = geo_masses[I];
        auto&       geo_vol  = geo_volumes[I];


        auto& sc = geometry(geo_slots, info);

        auto offset = info.m_vertex_offset;
        auto count  = info.m_vertex_count;

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
            auto [p0, p1, p2, p3] = std::tuple{Vs[t[0]], Vs[t[1]], Vs[t[2]], Vs[t[3]]};

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

    h_body_id_to_abd_mass.resize(h_body_infos.size());
    h_body_id_to_volume.resize(h_body_infos.size());

    for(auto&& [I, body_offset] : enumerate(h_abd_geo_body_offsets))
    {
        const auto& info = h_body_infos[body_offset];
        auto&       mass = h_body_id_to_abd_mass[info.m_affine_body_id];
        auto&       vol  = h_body_id_to_volume[info.m_affine_body_id];

        mass = geo_masses[I];
        vol  = geo_volumes[I];
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

        std::ranges::transform(h_abd_geo_body_offsets,
                               geo_gravities.begin(),
                               [&](SizeT offset) -> Vector12
                               {
                                   const auto& info  = h_body_infos[offset];
                                   auto        count = info.m_vertex_count;

                                   // sub_Js for this body
                                   auto sub_Js = Js.subspan(offset, count);
                                   auto sub_mass = vertex_mass.subspan(offset, count);

                                   Vector12 G = Vector12::Zero();

                                   for(auto&& [m, J] : zip(sub_mass, sub_Js))
                                       G += m * (J.T() * gravity);

                                   const Matrix12x12& M_inv =
                                       h_body_id_to_abd_mass_inv[info.m_affine_body_id];

                                   return M_inv * G;
                               });

        std::ranges::transform(h_body_infos,
                               h_body_id_to_abd_gravity.begin(),
                               [&](const BodyInfo& info) -> Vector12 {
                                   return geo_gravities[info.m_abd_geometry_index];
                               });
    }

    // 7) setup the boundary type
    h_body_id_to_is_fixed.resize(h_body_infos.size(), 0);
    for_each_body(
        geo_slots,
        [](geometry::SimplicialComplex& sc)
        { return sc.instances().find<IndexT>(builtin::is_fixed)->view(); },
        [&](SizeT i, IndexT fixed) { h_body_id_to_is_fixed[i] = fixed; });

    // 8) misc
    h_constitution_shape_energy.resize(constitutions.size(), 0.0);
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

    // setup body kinetic energy buffer
    async_resize(body_id_to_kinetic_energy, h_body_infos.size(), 0.0);

    // setup constitution shape energy buffer
    async_resize(constitution_shape_energy, constitutions.size(), 0.0);

    // setup hessian and gradient buffers
    async_resize(body_id_to_body_hessian,
                 h_body_infos.size(),
                 Matrix12x12::Zero().eval());
    async_resize(body_id_to_body_gradient, h_body_infos.size(), Vector12::Zero().eval());

    muda::wait_device();
}

void AffineBodyDynamics::Impl::_distribute_body_infos()
{
    // distribute the body infos to each constitution
    for(auto&& [I, constitution] : enumerate(constitutions))
    {
        FilteredInfo info{this};
        info.m_constitution_index = I;
        constitution->filter(info);
    }
}

void AffineBodyDynamics::Impl::init_affine_body_geometry(WorldVisitor& world)
{
    spdlog::info("Initializing Affine Body Geometry");

    _build_body_infos(world);
    _build_related_infos(world);
    _build_geometry_on_host(world);
    _build_geometry_on_device(world);
    _distribute_body_infos();
}

void AffineBodyDynamics::Impl::write_scene(WorldVisitor& world)
{
    // 1) download from device to host
    _download_geometry_to_host();

    auto scene     = world.scene();
    auto geo_slots = scene.geometries();

    span qs = h_body_id_to_q;

    // 2) transfer from m_abd qs to transforms
    for_each_body(
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

    aync_copy(body_id_to_q, span{h_body_id_to_q});

    muda::wait_device();
}

void AffineBodyDynamics::Impl::compute_q_tilde(DoFPredictor::PredictInfo& info)
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

void AffineBodyDynamics::Impl::compute_q_v(DoFPredictor::PredictInfo& info)
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

void AffineBodyDynamics::Impl::compute_gradient_hessian(GradientHessianComputer::ComputeInfo& info)
{
    using namespace muda;

    auto async_fill = []<typename T>(muda::DeviceBuffer<T>& buf, const T& value)
    { muda::BufferLaunch().fill<T>(buf.view(), value); };

    async_fill(body_id_to_body_hessian, Matrix12x12::Zero().eval());
    async_fill(body_id_to_body_gradient, Vector12::Zero().eval());

    // 1) compute all shape gradients and hessians
    for(auto&& [i, cst] : enumerate(constitutions))
    {
        auto body_offset = h_constitution_body_offsets[i];
        auto body_count  = h_constitution_body_counts[i];

        if(body_count == 0)
            continue;

        AffineBodyDynamics::ComputeGradientHessianInfo this_info{
            this,
            i,
            body_id_to_body_gradient.view(body_offset, body_count),
            body_id_to_body_hessian.view(body_offset, body_count),
            info.dt()};
        cst->compute_gradient_hessian(this_info);
    }

    // 2) add kinetic energy gradient and hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(body_count(),
               [is_fixed = body_id_to_is_fixed.cviewer().name("is_fixed"),
                qs       = body_id_to_q.cviewer().name("qs"),
                q_tildes = body_id_to_q_tilde.cviewer().name("q_tildes"),
                masses   = body_id_to_abd_mass.cviewer().name("masses"),
                Ks = body_id_to_kinetic_energy.cviewer().name("kinetic_energy"),
                hessians = body_id_to_body_hessian.viewer().name("hessians"),
                gradients = body_id_to_body_gradient.viewer().name("gradients")] __device__(int i) mutable
               {
                   auto&       H = hessians(i);
                   auto&       G = gradients(i);
                   const auto& M = masses(i);

                   if(is_fixed(i))
                   {
                       H = Matrix12x12::Identity();
                       G = Vector12::Zero();
                       return;
                   }

                   const auto& q       = qs(i);
                   const auto& q_tilde = q_tildes(i);

                   const auto& K = Ks(i);
                   G += M * (q - q_tilde);
                   H += M.to_mat();

                   Vector9   eigen_values;
                   Matrix9x9 eigen_vectors;
                   Matrix9x9 mat9 = H.block<9, 9>(3, 3);
                   muda::eigen::evd(mat9, eigen_values, eigen_vectors);
                   for(int i = 0; i < 9; ++i)
                   {
                       auto& v = eigen_values(i);
                       v       = v < 0.0 ? 0.0 : v;
                   }
                   H.block<9, 9>(3, 3) = eigen_vectors * eigen_values.asDiagonal()
                                         * eigen_vectors.transpose();
               });
}

geometry::SimplicialComplex& AffineBodyDynamics::Impl::geometry(
    span<P<geometry::GeometrySlot>> geo_slots, const BodyInfo& body_info)
{
    auto& geo = geo_slots[body_info.m_geometry_slot_index]->geometry();
    auto  sc  = geo.as<geometry::SimplicialComplex>();
    UIPC_ASSERT(sc,
                "The geometry is not a simplicial complex (it's {}). Why can it happen?",
                geo.type());
    return *sc;
}
}  // namespace uipc::backend::cuda