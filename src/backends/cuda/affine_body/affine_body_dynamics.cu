#include <affine_body/affine_body_dynamics.h>
#include <affine_body/utils.h>
#include <affine_body/abd_line_search_reporter.h>
#include <affine_body/affine_body_constitution.h>
#include <Eigen/Dense>
#include <algorithm>
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
#include <fstream>
#include <sim_engine.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/geometry/utils/affine_body/compute_dyadic_mass.h>
#include <uipc/geometry/utils/affine_body/compute_body_force.h>
#include <muda/ext/eigen/inverse.h>

namespace uipc::backend
{
template <>
class backend::SimSystemCreator<cuda::AffineBodyDynamics>
{
  public:
    static U<cuda::AffineBodyDynamics> create(SimEngine& engine)
    {
        auto  scene = dynamic_cast<cuda::SimEngine&>(engine).world().scene();
        auto& types = scene.constitution_tabular().types();
        if(types.find(string{builtin::AffineBody}) == types.end())
        {
            return nullptr;
        }
        return uipc::make_unique<cuda::AffineBodyDynamics>(engine);
    }
};
}  // namespace uipc::backend

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(AffineBodyDynamics);

void AffineBodyDynamics::do_build()
{
    // find dependent systems
    auto& dof_predictor = require<DofPredictor>();

    // Register the action to initialize the affine body geometry
    on_init_scene([this] { m_impl.init(world()); });

    // Register the action to predict the affine body dof
    dof_predictor.on_predict(*this,
                             [this](DofPredictor::PredictInfo& info)
                             { m_impl.compute_q_tilde(info); });

    // Register the action to compute the velocity of the affine body dof
    dof_predictor.on_compute_velocity(*this,
                                      [this](DofPredictor::ComputeVelocityInfo& info)
                                      { m_impl.compute_q_v(info); });

    // Register the action to write the scene
    on_write_scene([this] { m_impl.write_scene(world()); });
}

bool AffineBodyDynamics::do_dump(DumpInfo& info)
{
    return m_impl.dump(info);
}

bool AffineBodyDynamics::do_try_recover(RecoverInfo& info)
{
    return m_impl.try_recover(info);
}

void AffineBodyDynamics::do_apply_recover(RecoverInfo& info)
{
    m_impl.apply_recover(info);
}

void AffineBodyDynamics::do_clear_recover(RecoverInfo& info)
{
    m_impl.clear_recover(info);
}
}  // namespace uipc::backend::cuda


namespace uipc::backend::cuda
{
void AffineBodyDynamics::add_constitution(AffineBodyConstitution* constitution)
{
    check_state(SimEngineState::BuildSystems, "add_constitution()");
    // set the temp index, later we will sort constitution by uid
    // and reset the index
    m_impl.constitutions.register_subsystem(*constitution);
}

void AffineBodyDynamics::Impl::init(WorldVisitor& world)
{
    _build_constitutions(world);
    _build_geo_infos(world);
    _setup_geometry_attributes(world);

    _build_geometry_on_host(world);
    _build_geometry_on_device(world);

    _distribute_geo_infos();
}

void AffineBodyDynamics::Impl::_build_constitutions(WorldVisitor& world)
{
    // 1) sort the constitutions by uid
    auto constitution_view = constitutions.view();
    std::ranges::sort(constitution_view,
                      [](const AffineBodyConstitution* a, AffineBodyConstitution* b)
                      { return a->uid() < b->uid(); });
    for(auto&& [i, C] : enumerate(constitution_view))  // reset the index
        C->m_index = i;

    vector<U64> filter_uids(constitution_view.size());
    std::ranges::transform(constitution_view,
                           filter_uids.begin(),
                           [](const AffineBodyConstitution* filter)
                           { return filter->uid(); });

    for(auto&& [i, filter] : enumerate(constitution_view))
        constitution_uid_to_index[filter->uid()] = i;

    constitution_infos.resize(constitution_view.size());

    auto geo_slots = world.scene().geometries();

    // +1 for the total count
    vector<SizeT> constitution_geo_counts(constitution_view.size() + 1, 0);
    vector<SizeT> constitution_geo_offsets(constitution_view.size() + 1, 0);
    vector<SizeT> constitution_body_counts(constitution_view.size() + 1, 0);
    vector<SizeT> constitution_body_offsets(constitution_view.size() + 1, 0);
    vector<SizeT> constitution_vertex_counts(constitution_view.size() + 1, 0);
    vector<SizeT> constitution_vertex_offsets(constitution_view.size() + 1, 0);

    list<GeoInfo> geo_info_list;
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

                auto constitution_index = constitution_uid_to_index[uid];

                GeoInfo info;
                info.body_count = instance_count;
                info.vertex_count = vert_count * instance_count;  // total vertex count
                info.geo_slot_index     = i;
                info.constitution_uid   = uid;
                info.constitution_index = constitution_index;
                geo_info_list.push_back(std::move(info));

                constitution_geo_counts[constitution_index] += 1;  // count the geometry
                constitution_body_counts[constitution_index] += info.body_count;  // count the body
                constitution_vertex_counts[constitution_index] += info.vertex_count;  // count the vertex
            }
        }
    }

    // 2) setup the offsets
    std::exclusive_scan(constitution_geo_counts.begin(),
                        constitution_geo_counts.end(),
                        constitution_geo_offsets.begin(),
                        0);

    std::exclusive_scan(constitution_body_counts.begin(),
                        constitution_body_counts.end(),
                        constitution_body_offsets.begin(),
                        0);

    std::exclusive_scan(constitution_vertex_counts.begin(),
                        constitution_vertex_counts.end(),
                        constitution_vertex_offsets.begin(),
                        0);

    for(auto&& [i, info] : enumerate(constitution_infos))
    {
        info.geo_offset    = constitution_geo_offsets[i];
        info.geo_count     = constitution_geo_counts[i];
        info.body_offset   = constitution_body_offsets[i];
        info.body_count    = constitution_body_counts[i];
        info.vertex_offset = constitution_vertex_offsets[i];
        info.vertex_count  = constitution_vertex_counts[i];
    }

    // 3) set the total count
    abd_geo_count    = constitution_geo_offsets.back();
    abd_body_count   = constitution_body_offsets.back();
    abd_vertex_count = constitution_vertex_offsets.back();

    // 4) copy to the geo_infos
    geo_infos.resize(geo_info_list.size());
    std::ranges::move(geo_info_list, geo_infos.begin());
}

void AffineBodyDynamics::Impl::_build_geo_infos(WorldVisitor& world)
{
    auto scene     = world.scene();
    auto geo_slots = scene.geometries();

    // 1) sort the geo_infos by constitution uid
    std::ranges::stable_sort(geo_infos,
                             [](const GeoInfo& a, const GeoInfo& b)
                             { return a.constitution_uid < b.constitution_uid; });


    // 2) setup the body offset and body count
    vector<SizeT> geo_body_counts(geo_infos.size() + 1, 0);  // + 1 for total count
    vector<SizeT> geo_body_offsets(geo_infos.size() + 1, 0);  // + 1 for total count

    std::ranges::transform(geo_infos,
                           geo_body_counts.begin(),
                           [](const GeoInfo& info) -> SizeT
                           { return info.body_count; });

    std::exclusive_scan(
        geo_body_counts.begin(), geo_body_counts.end(), geo_body_offsets.begin(), 0);

    abd_body_count = geo_body_offsets.back();

    for(auto&& [i, info] : enumerate(geo_infos))
    {
        info.body_offset = geo_body_offsets[i];
    }

    // 3) setup the vertex offset and vertex count
    vector<SizeT> geo_vertex_counts(geo_infos.size() + 1);  // + 1 for total count
    vector<SizeT> geo_vertex_offsets(geo_infos.size() + 1);  // + 1 for total count

    std::ranges::transform(geo_infos,
                           geo_vertex_counts.begin(),
                           [](const GeoInfo& info) -> SizeT
                           { return info.vertex_count; });

    std::exclusive_scan(geo_vertex_counts.begin(),
                        geo_vertex_counts.end(),
                        geo_vertex_offsets.begin(),
                        0);

    for(auto&& [i, info] : enumerate(geo_infos))
    {
        info.vertex_offset = geo_vertex_offsets[i];
    }
}

void AffineBodyDynamics::Impl::_setup_geometry_attributes(WorldVisitor& world)
{
    // set the `backend_abd_body_offset` attribute in simplicial complex geometry
    auto geo_slots = world.scene().geometries();
    for_each(geo_slots,
             [&](const ForEachInfo& I, geometry::SimplicialComplex& sc)
             {
                 auto geoI = I.global_index();

                 auto abd_body_offset =
                     sc.meta().find<IndexT>(builtin::backend_abd_body_offset);
                 if(!abd_body_offset)
                     abd_body_offset =
                         sc.meta().create<IndexT>(builtin::backend_abd_body_offset);

                 auto body_offset_view    = geometry::view(*abd_body_offset);
                 body_offset_view.front() = geo_infos[geoI].body_offset;
             });
}

void AffineBodyDynamics::Impl::_build_geometry_on_host(WorldVisitor& world)
{
    auto scene             = world.scene();
    auto geo_slots         = scene.geometries();
    auto rest_geo_slots    = scene.rest_geometries();
    auto constitution_view = constitutions.view();

    // 1) Setup `q` for every affine body
    {
        h_body_id_to_q.resize(abd_body_count);

        // SizeT bodyI = 0;
        for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc)
            { return sc.transforms().view(); },
            [&](const ForEachInfo& I, const Matrix4x4& trans)
            {
                auto bodyI = I.global_index();

                Float D = trans.block<3, 3>(0, 0).determinant();

                Vector12& q = h_body_id_to_q[bodyI];

                q = transform_to_q(trans);
            });
        
        for_each(
            rest_geo_slots,
            [](geometry::SimplicialComplex& sc)
            { return sc.transforms().view(); },
            [&](const ForEachInfo& I, const Matrix4x4 trans)
            {
                auto bodyI = I.global_index();

                Float D = trans.block<3,3>(0,0).determinant();

                if(!Eigen::Vector<Float, 1>{D}.isApproxToConstant(1.0, 1e-6))
                    spdlog::warn("Don't apply scaling on Affine Body's rest shape.");
            });
    }


    // 2) Setup `J` for every vertex
    {
        h_vertex_id_to_J.resize(abd_vertex_count);

        span Js = h_vertex_id_to_J;

        for_each(geo_slots,
                 [&](const ForEachInfo& I, geometry::SimplicialComplex& sc)
                 {
                     auto geoI = I.global_index();

                     auto pos_view = sc.positions().view();

                     auto body_count  = sc.instances().size();
                     auto vert_count  = sc.vertices().size();
                     auto vert_offset = geo_infos[geoI].vertex_offset;

                     for(auto i : range(body_count))
                     {
                         auto body_vert_offset = vert_offset + i * vert_count;
                         auto sub_Js = Js.subspan(body_vert_offset, vert_count);

                         std::ranges::transform(pos_view,
                                                sub_Js.begin(),
                                                [](const Vector3& pos) -> ABDJacobi
                                                { return pos; });
                     }
                 });
    }


    // 3) Setup `vertex_id_to_body_id` and `vertex_id_to_contact_element_id`
    {
        h_vertex_id_to_body_id.resize(abd_vertex_count);
        h_vertex_id_to_contact_element_id.resize(abd_vertex_count);

        span v2b = h_vertex_id_to_body_id;
        span v2c = h_vertex_id_to_contact_element_id;

        for_each(
            geo_slots,
            [&](const ForEachInfo& I, geometry::SimplicialComplex& sc)
            {
                auto geoI = I.global_index();

                auto body_count  = sc.instances().size();
                auto body_offset = geo_infos[geoI].body_offset;
                auto vert_count  = sc.vertices().size();
                auto vert_offset = geo_infos[geoI].vertex_offset;

                auto vert_contact_element_id =
                    sc.vertices().find<IndexT>(builtin::contact_element_id);

                auto contact_element_id =
                    sc.meta().find<IndexT>(builtin::contact_element_id);

                for(auto i : range(body_count))
                {
                    auto body_vert_offset = vert_offset + i * vert_count;
                    auto body_id          = body_offset + i;

                    auto v2b_span = v2b.subspan(body_vert_offset, vert_count);
                    auto v2c_span = v2c.subspan(body_vert_offset, vert_count);

                    std::ranges::fill(v2b_span, body_id);

                    if(vert_contact_element_id)
                    {
                        auto vert_contact_element_id_view =
                            vert_contact_element_id->view();

                        UIPC_ASSERT(vert_contact_element_id_view.size() == vert_count,
                                    "The size of the contact_element_id attribute is not equal to the vertex count ({} != {}).",
                                    vert_contact_element_id_view.size(),
                                    vert_count);

                        std::ranges::copy(vert_contact_element_id_view, v2c_span.begin());
                    }
                    else
                    {
                        if(contact_element_id)
                        {
                            auto contact_element_id_view = contact_element_id->view();
                            std::ranges::fill(v2c_span, contact_element_id_view.front());
                        }
                        else
                        {
                            std::ranges::fill(v2c_span,
                                              0);  // default 0
                        }
                    }
                }
            });
    }

    // 4) Setup body_abd_mass and body_id_to_volume
    {
        h_body_id_to_abd_mass.resize(abd_body_count);
        h_body_id_to_volume.resize(abd_body_count);

        span Js = h_vertex_id_to_J;

        for_each(geo_slots,
                 [&](const ForEachInfo& I, geometry::SimplicialComplex& sc)
                 {
                     auto geoI = I.global_index();

                     auto vert_count  = sc.vertices().size();
                     auto vert_offset = geo_infos[geoI].vertex_offset;
                     auto body_count  = sc.instances().size();
                     auto body_offset = geo_infos[geoI].body_offset;

                     auto sub_Js = Js.subspan(vert_offset, vert_count);

                     ABDJacobiDyadicMass geo_mass;
                     {
                         auto rho = sc.meta().find<Float>(builtin::mass_density);
                         UIPC_ASSERT(rho, "The `mass_density` attribute is not found in the affine body geometry, why can it happen?");

                         auto rho_view = rho->view();

                         Float     m;
                         Vector3   m_x_bar;
                         Matrix3x3 m_x_bar_x_bar;

                         uipc::geometry::affine_body::compute_dyadic_mass(
                             sc, rho_view[0], m, m_x_bar, m_x_bar_x_bar);
                         geo_mass = ABDJacobiDyadicMass::from_dyadic_mass(m, m_x_bar, m_x_bar_x_bar);
                     }

                     auto volume = sc.instances().find<Float>(builtin::volume);
                     UIPC_ASSERT(volume, "The `volume` attribute is not found in the affine body instance, why can it happen?");

                     auto volume_view = volume->view();

                     for(auto i : range(body_count))
                     {
                         auto body_id                   = body_offset + i;
                         h_body_id_to_abd_mass[body_id] = geo_mass;
                         h_body_id_to_volume[body_id]   = volume_view[i];
                     }
                 });
    }


    // 5) Compute the inverse of the mass matrix
    h_body_id_to_abd_mass_inv.resize(abd_body_count);
    std::ranges::transform(h_body_id_to_abd_mass,
                           h_body_id_to_abd_mass_inv.begin(),
                           [](const ABDJacobiDyadicMass& mass) -> Matrix12x12
                           { return muda::eigen::inverse(mass.to_mat()); });

    // 6) Setup the affine body gravity
    {
        span Js = h_vertex_id_to_J;

        Vector3 gravity = scene.info()["gravity"];
        h_body_id_to_abd_gravity.resize(abd_body_count, Vector12::Zero());
        for_each(geo_slots,
                 [&](const ForEachInfo& I, geometry::SimplicialComplex& sc)
                 {
                     auto geoI = I.global_index();

                     auto vert_count  = sc.vertices().size();
                     auto vert_offset = geo_infos[geoI].vertex_offset;
                     auto body_count  = sc.instances().size();
                     auto body_offset = geo_infos[geoI].body_offset;

                     auto sub_Js = Js.subspan(vert_offset, vert_count);
                     // auto sub_mass = vertex_mass.subspan(vert_offset, vert_count);


                     auto gravity_attr = sc.instances().find<Vector3>(builtin::gravity);
                     auto gravity_view = gravity_attr ? gravity_attr->view() :
                                                        span<const Vector3>{};

                     auto rho = sc.meta().find<Float>(builtin::mass_density);
                     UIPC_ASSERT(rho, "The `mass_density` attribute is not found in the affine body geometry, why can it happen?");
                     auto rho_view = rho->view();

                     for(auto i : range(body_count))
                     {

                         Vector3 local_gravity = gravity_attr ? gravity_view[i] : gravity;

                         Vector3 force_density = local_gravity * rho_view[0];

                         Vector12 G =
                             uipc::geometry::affine_body::compute_body_force(sc, force_density);

                         Matrix12x12 abd_body_mass_inv =
                             h_body_id_to_abd_mass_inv[body_offset];
                         Vector12 g = abd_body_mass_inv * G;

                         auto body_id = body_offset + i;

                         h_body_id_to_abd_gravity[body_id] = g;
                     }
                 });
    }


    // 7) Setup the boundary type
    {
        h_body_id_to_is_fixed.resize(abd_body_count, 0);
        h_body_id_to_is_dynamic.resize(abd_body_count, 1);
        for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc)
            {
                auto is_fixed = sc.instances().find<IndexT>(builtin::is_fixed);
                auto is_dynamic = sc.instances().find<IndexT>(builtin::is_dynamic);

                UIPC_ASSERT(is_fixed, "The is_fixed attribute is not found in the affine body geometry, why can it happen?");
                UIPC_ASSERT(is_dynamic, "The is_dynamic attribute is not found in the affine body geometry, why can it happen?");

                return zip(is_fixed->view(), is_dynamic->view());
            },
            [&](const ForEachInfo& I, auto&& data)
            {
                auto&& [fixed, dynamic] = data;

                auto bodyI = I.global_index();

                h_body_id_to_is_fixed[bodyI]   = fixed;
                h_body_id_to_is_dynamic[bodyI] = dynamic;
            });
    }


    // 8) Energy per constitution
    h_constitution_shape_energy.resize(constitution_view.size(), 0.0);
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
    async_copy(span{h_body_id_to_is_dynamic}, body_id_to_is_dynamic);

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

    async_resize(body_id_to_q_v, abd_body_count, Vector12::Zero().eval());
    async_resize(body_id_to_dq, abd_body_count, Vector12::Zero().eval());

    // setup body kinetic energy buffer
    async_resize(body_id_to_kinetic_energy, abd_body_count, 0.0);

    async_resize(body_id_to_shape_energy, abd_body_count, 0.0);

    // setup hessian and gradient buffers
    async_resize(diag_hessian, abd_body_count, Matrix12x12::Zero().eval());
    async_resize(body_id_to_body_hessian, abd_body_count, Matrix12x12::Zero().eval());
    async_resize(body_id_to_body_gradient, abd_body_count, Vector12::Zero().eval());

    muda::wait_device();
}

void AffineBodyDynamics::Impl::_distribute_geo_infos()
{
    // _distribute the body infos to each constitution
    for(auto&& [I, constitution] : enumerate(constitutions.view()))
    {
        FilteredInfo info{this, I};
        constitution->init(info);
    }
}

void AffineBodyDynamics::Impl::write_scene(WorldVisitor& world)
{
    // 1) download from device to host
    _download_geometry_to_host();

    auto scene     = world.scene();
    auto geo_slots = scene.geometries();

    span qs = h_body_id_to_q;

    // 2) transfer from affine_body_dynamics qs to transforms
    for_each(
        geo_slots,
        [](geometry::SimplicialComplex& sc) { return view(sc.transforms()); },
        [&](const ForEachInfo& I, Matrix4x4& trans)
        {
            auto bodyI = I.global_index();

            Vector12& q = h_body_id_to_q[bodyI];

            trans = q_to_transform(q);
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
}  // namespace uipc::backend::cuda


// Dump & Recover:
namespace uipc::backend::cuda
{
bool AffineBodyDynamics::Impl::dump(DumpInfo& info)
{
    auto path  = info.dump_path(__FILE__);
    auto frame = info.frame();

    return dump_q.dump(fmt::format("{}q.{}", path, frame), body_id_to_q)  //
           && dump_q_v.dump(fmt::format("{}q_v.{}", path, frame), body_id_to_q_v)  //
           && dump_q_prev.dump(fmt::format("{}q_prev.{}", path, frame), body_id_to_q_prev);  //
}

bool AffineBodyDynamics::Impl::try_recover(RecoverInfo& info)
{
    auto path  = info.dump_path(__FILE__);
    auto frame = info.frame();

    return dump_q.load(fmt::format("{}q.{}", path, frame))                //
           && dump_q_v.load(fmt::format("{}q_v.{}", path, frame))         //
           && dump_q_prev.load(fmt::format("{}q_prev.{}", path, frame));  //
}

void AffineBodyDynamics::Impl::apply_recover(RecoverInfo& info)
{
    dump_q.apply_to(body_id_to_q);
    dump_q_v.apply_to(body_id_to_q_v);
    dump_q_prev.apply_to(body_id_to_q_prev);
}

void AffineBodyDynamics::Impl::clear_recover(RecoverInfo& info)
{
    dump_q.clean_up();
    dump_q_v.clean_up();
    dump_q_prev.clean_up();
}
}  // namespace uipc::backend::cuda


// Simulation:
namespace uipc::backend::cuda
{
void AffineBodyDynamics::Impl::compute_q_tilde(DofPredictor::PredictInfo& info)
{
    using namespace muda;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(abd_body_count,
               [is_fixed   = body_id_to_is_fixed.cviewer().name("is_fixed"),
                is_dynamic = body_id_to_is_dynamic.cviewer().name("is_dynamic"),
                q_prevs    = body_id_to_q_prev.cviewer().name("q_prev"),
                q_vs       = body_id_to_q_v.cviewer().name("q_velocities"),
                q_tildes   = body_id_to_q_tilde.viewer().name("q_tilde"),
                affine_gravity = body_id_to_abd_gravity.cviewer().name("affine_gravity"),
                dt = info.dt()] __device__(int i) mutable
               {
                   auto& q_prev = q_prevs(i);
                   auto& q_v    = q_vs(i);
                   auto& g      = affine_gravity(i);

                   // 0) fixed: q_tilde = q_prev;
                   Vector12 q_tilde = q_prev;

                   if(!is_fixed(i))
                   {
                       // 1) static problem: q_tilde = q_prev + g * dt * dt;
                       q_tilde += g * dt * dt;

                       // 2) dynamic problem q_tilde = q_prev + q_v * dt + g * dt * dt;
                       if(is_dynamic(i))
                       {
                           q_tilde += q_v * dt;
                       }
                   }

                   q_tildes(i) = q_tilde;
               });
}

void AffineBodyDynamics::Impl::compute_q_v(DofPredictor::ComputeVelocityInfo& info)
{
    using namespace muda;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(abd_body_count,
               [qs      = body_id_to_q.cviewer().name("qs"),
                q_vs    = body_id_to_q_v.viewer().name("q_vs"),
                q_prevs = body_id_to_q_prev.viewer().name("q_prevs"),
                dt      = info.dt()] __device__(int i) mutable
               {
                   auto& q_v    = q_vs(i);
                   auto& q_prev = q_prevs(i);

                   const auto& q = qs(i);

                   q_v = (q - q_prev) * (1.0 / dt);

                   q_prev = q;
               });
}
}  // namespace uipc::backend::cuda
