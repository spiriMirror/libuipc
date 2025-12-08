#include <affine_body/affine_body_dynamics.h>
#include <affine_body/utils.h>
#include <affine_body/abd_line_search_reporter.h>
#include <fstream>
#include <algorithm>

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
#include <sim_engine.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/geometry/utils/affine_body/compute_dyadic_mass.h>
#include <uipc/geometry/utils/affine_body/compute_body_force.h>
#include <muda/ext/eigen/inverse.h>
#include <uipc/builtin/attribute_name.h>

#include <utils/offset_count_collection.h>
#include <affine_body/affine_body_kinetic.h>
#include <affine_body/affine_body_vertex_reporter.h>

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
    auto& config         = world().scene().config();
    auto  d_hat_attr     = config.find<Float>("contact/d_hat");
    m_impl.default_d_hat = d_hat_attr->view()[0];

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

IndexT AffineBodyDynamics::dof_offset(SizeT frame) const
{
    return m_impl.dof_offset(frame);
}

IndexT AffineBodyDynamics::dof_count(SizeT frame) const
{
    return m_impl.dof_count(frame);
}

void AffineBodyDynamics::add_constitution(AffineBodyConstitution* constitution)
{
    check_state(SimEngineState::BuildSystems, "add_constitution()");
    // set the temp index, later we will sort constitution by uid
    // and reset the index
    m_impl.constitutions.register_subsystem(*constitution);
}

void AffineBodyDynamics::add_kinetic(AffineBodyKinetic* kinetic)
{
    check_state(SimEngineState::BuildSystems, "add_kinetic()");
    m_impl.kinetic.register_subsystem(*kinetic);
}

void AffineBodyDynamics::add_reporter(AffineBodyKineticDiffParmReporter* reporter)
{
    UIPC_ASSERT(false, "NOT IMPL IN THIS VERSION");

    //check_state(SimEngineState::BuildSystems, "add_reporter()");
    //m_impl.diff_parm_reporter.register_subsystem(*reporter);
}

void AffineBodyDynamics::init()
{
    m_impl.init(world());
}

void AffineBodyDynamics::Impl::init(WorldVisitor& world)
{
    _init_dof_info();

    _build_constitutions(world);
    _build_geo_infos(world);
    _setup_geometry_attributes(world);
    _build_geometry_on_host(world);
    _build_geometry_on_device(world);

    _distribute_geo_infos();
    _init_diff_reporters();
}

void AffineBodyDynamics::Impl::_build_constitutions(WorldVisitor& world)
{
    // 1) Sort the constitutions by uid
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

    // 2) Build the geo_infos and count the geometries, bodies, and vertices per constitution
    OffsetCountCollection<IndexT> constitution_geo_offsets_counts;
    constitution_geo_offsets_counts.resize(constitution_view.size());

    OffsetCountCollection<IndexT> constitution_body_offsets_counts;
    constitution_body_offsets_counts.resize(constitution_view.size());

    OffsetCountCollection<IndexT> constitution_vertex_offsets_counts;
    constitution_vertex_offsets_counts.resize(constitution_view.size());

    span<IndexT> constitution_geo_counts = constitution_geo_offsets_counts.counts();
    span<IndexT> constitution_body_counts = constitution_body_offsets_counts.counts();
    span<IndexT> constitution_vertex_counts = constitution_vertex_offsets_counts.counts();

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
                info.geo_id     = geo_slot->id();
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

    // 3) Setup the offsets
    constitution_geo_offsets_counts.scan();
    constitution_body_offsets_counts.scan();
    constitution_vertex_offsets_counts.scan();

    span<const IndexT> constitution_geo_offsets =
        constitution_geo_offsets_counts.offsets();
    span<const IndexT> constitution_body_offsets =
        constitution_body_offsets_counts.offsets();
    span<const IndexT> constitution_vertex_offsets =
        constitution_vertex_offsets_counts.offsets();


    for(auto&& [i, info] : enumerate(constitution_infos))
    {
        info.geo_offset    = constitution_geo_offsets[i];
        info.geo_count     = constitution_geo_counts[i];
        info.body_offset   = constitution_body_offsets[i];
        info.body_count    = constitution_body_counts[i];
        info.vertex_offset = constitution_vertex_offsets[i];
        info.vertex_count  = constitution_vertex_counts[i];
    }

    // 4) set the total count
    abd_geo_count    = constitution_geo_offsets_counts.total_count();
    abd_body_count   = constitution_body_offsets_counts.total_count();
    abd_vertex_count = constitution_vertex_offsets_counts.total_count();

    // 4) copy to the geo_infos
    geo_infos.resize(geo_info_list.size());
    std::ranges::move(geo_info_list, geo_infos.begin());
}

void AffineBodyDynamics::Impl::_build_geo_infos(WorldVisitor& world)
{
    auto scene     = world.scene();
    auto geo_slots = scene.geometries();

    // 1) Sort the geo_infos by constitution uid
    std::ranges::stable_sort(geo_infos,
                             [](const GeoInfo& a, const GeoInfo& b)
                             { return a.constitution_uid < b.constitution_uid; });

    for(auto&& [i, info] : enumerate(geo_infos))
    {
        geo_id_to_geo_info_index[info.geo_id] = i;  // build map for geo_id to geo_info index
    }

    // 2) Setup the body offset and body count
    OffsetCountCollection<IndexT> geo_body_offsets_counts;
    geo_body_offsets_counts.resize(geo_infos.size());

    span<IndexT>       geo_body_counts  = geo_body_offsets_counts.counts();
    span<const IndexT> geo_body_offsets = geo_body_offsets_counts.offsets();

    std::ranges::transform(geo_infos,
                           geo_body_counts.begin(),
                           [](const GeoInfo& info) -> IndexT
                           { return static_cast<IndexT>(info.body_count); });

    geo_body_offsets_counts.scan();

    abd_body_count = geo_body_offsets_counts.total_count();

    for(auto&& [i, info] : enumerate(geo_infos))
    {
        info.body_offset = geo_body_offsets[i];
    }

    // 3) Setup the vertex offset and vertex count
    OffsetCountCollection<IndexT> geo_vertex_offsets_counts;
    geo_vertex_offsets_counts.resize(geo_infos.size());

    span<IndexT> geo_vertex_counts = geo_vertex_offsets_counts.counts();

    std::ranges::transform(geo_infos,
                           geo_vertex_counts.begin(),
                           [](const GeoInfo& info) -> SizeT
                           { return info.vertex_count; });

    geo_vertex_offsets_counts.scan();
    span<const IndexT> geo_vertex_offsets = geo_vertex_offsets_counts.offsets();

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

    // 1) Setup `q` and `q_v` for every affine body
    {
        h_body_id_to_q.resize(abd_body_count);
        h_body_id_to_q_v.resize(abd_body_count, Vector12::Zero());

        for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc)
            { return sc.transforms().view(); },
            [&](const ForEachInfo& I, const Matrix4x4& trans)
            {
                auto  bodyI = I.global_index();
                Float D     = trans.block<3, 3>(0, 0).determinant();
                UIPC_ASSERT(D >= 0, "determinant of the transform matrix is non-positive, why can it happen?");
                Vector12& q = h_body_id_to_q[bodyI];
                q           = transform_to_q(trans);
            });

        for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc)
            {
                auto vel = sc.instances().find<Matrix4x4>(builtin::velocity);
                return vel ? vel->view() : span<const Matrix4x4>{};
            },
            [&](const ForEachInfo& I, const Matrix4x4& velocity)
            {
                auto&     geo_info = I.geo_info();
                auto      bodyI    = geo_info.body_offset + I.local_index();
                Vector12& q_v      = h_body_id_to_q_v[bodyI];
                q_v                = transform_to_q(velocity);
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


    // 3) Setup:
    // - `vertex_id_to_body_id`
    // - `vertex_id_to_contact_element_id`
    // - `vertex_id_to_d_hat`
    {
        h_vertex_id_to_body_id.resize(abd_vertex_count);
        h_vertex_id_to_contact_element_id.resize(abd_vertex_count);
        h_vertex_id_to_subscene_contact_element_id.resize(abd_vertex_count);
        h_vertex_id_to_d_hat.resize(abd_vertex_count);

        span v2b     = h_vertex_id_to_body_id;
        span v2c     = h_vertex_id_to_contact_element_id;
        span v2sc    = h_vertex_id_to_subscene_contact_element_id;
        span v2d_hat = h_vertex_id_to_d_hat;

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

                auto vert_contact_subscene_element_id =
                    sc.vertices().find<IndexT>(builtin::subscene_element_id);

                auto contact_element_id =
                    sc.meta().find<IndexT>(builtin::contact_element_id);

                auto subscene_element_id =
                    sc.meta().find<IndexT>(builtin::subscene_element_id);

                for(auto i : range(body_count))
                {
                    auto body_vert_offset = vert_offset + i * vert_count;
                    auto body_id          = body_offset + i;

                    auto v2b_span  = v2b.subspan(body_vert_offset, vert_count);
                    auto v2c_span  = v2c.subspan(body_vert_offset, vert_count);
                    auto v2sc_span = v2sc.subspan(body_vert_offset, vert_count);
                    auto v2d_hat_span = v2d_hat.subspan(body_vert_offset, vert_count);

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
                            std::ranges::fill(v2c_span, 0);  // default 0
                        }
                    }

                    if(vert_contact_subscene_element_id)
                    {
                        auto vert_contact_subscene_element_id_view =
                            vert_contact_subscene_element_id->view();

                        UIPC_ASSERT(vert_contact_subscene_element_id_view.size() == vert_count,
                                    "The size of the contact_element_id attribute is not equal to the vertex count ({} != {}).",
                                    vert_contact_subscene_element_id_view.size(),
                                    vert_count);

                        std::ranges::copy(vert_contact_subscene_element_id_view,
                                          v2sc_span.begin());
                    }
                    else
                    {
                        if(subscene_element_id)
                        {
                            auto contact_subscene_element_id_view =
                                subscene_element_id->view();
                            std::ranges::fill(v2sc_span,
                                              contact_subscene_element_id_view.front());
                        }
                        else
                        {
                            std::ranges::fill(v2sc_span, 0);  // default 0
                        }
                    }

                    auto meta_d_hat = sc.meta().find<Float>(builtin::d_hat);

                    if(meta_d_hat)
                    {
                        auto vert_d_hat_view = meta_d_hat->view();
                        std::ranges::fill(v2d_hat_span, vert_d_hat_view.front());  // use the meta d_hat value
                    }
                    else
                    {
                        std::ranges::fill(v2d_hat_span, default_d_hat);  // default d_hat
                    }
                }
            });
    }

    // 4) Setup:
    // - `body_abd_mass`
    // - `body_id_to_volume`
    // - `body_id_to_dim`
    // - `body_id_to_self_collision`
    {
        h_body_id_to_abd_mass.resize(abd_body_count);
        h_body_id_to_volume.resize(abd_body_count);
        h_body_id_to_dim.resize(abd_body_count);
        h_body_id_to_self_collision.resize(abd_body_count, 0);  // default 0

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

                         //std::cout << "mass: \n"
                         //          << geo_mass.to_mat() << std::endl;
                     }

                     auto volume = sc.instances().find<Float>(builtin::volume);
                     UIPC_ASSERT(volume, "The `volume` attribute is not found in the affine body instance, why can it happen?");

                     auto volume_view = volume->view();
                     //std::cout << "volume: " << volume_view[0] << std::endl;

                     auto self_collision = sc.meta().find<IndexT>(builtin::self_collision);
                     UIPC_ASSERT(self_collision, "The `self_collision` attribute is not found in the affine body `meta`, why can it happen?");

                     IndexT self_collision_value = self_collision->view()[0];

                     for(auto i : range(body_count))
                     {
                         auto body_id = body_offset + i;
                         // mass
                         h_body_id_to_abd_mass[body_id] = geo_mass;
                         // volume
                         h_body_id_to_volume[body_id] = volume_view[i];
                         // dim
                         h_body_id_to_dim[body_id] = sc.dim();
                         // self_collision
                         h_body_id_to_self_collision[body_id] = self_collision_value;
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
        span    Js           = h_vertex_id_to_J;
        auto    gravity_attr = scene.config().find<Vector3>("gravity");
        Vector3 gravity      = gravity_attr->view()[0];
        h_body_id_to_abd_gravity.resize(abd_body_count, Vector12::Zero());
        body_id_to_external_force.resize(abd_body_count);
        body_id_to_external_force_acc.resize(abd_body_count);
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

                         // std::cout << "force: " << G.transpose() << std::endl;

                         Matrix12x12 abd_body_mass_inv =
                             h_body_id_to_abd_mass_inv[body_offset];
                         Vector12 g = abd_body_mass_inv * G;

                         auto body_id = body_offset + i;

                         h_body_id_to_abd_gravity[body_id] = g;

                         // std::cout << "gravity: " << g.transpose() << std::endl;
                     }
                 });
    }


    // 7) Setup the boundary type
    {
        h_body_id_to_is_fixed.resize(abd_body_count, 0);
        h_body_id_to_is_dynamic.resize(abd_body_count, 1);
        h_body_id_to_external_kinetic.resize(abd_body_count, 0);
        for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc)
            {
                auto is_fixed = sc.instances().find<IndexT>(builtin::is_fixed);
                auto is_dynamic = sc.instances().find<IndexT>(builtin::is_dynamic);
                auto external_kinetic =
                    sc.instances().find<IndexT>(builtin::external_kinetic);

                UIPC_ASSERT(is_fixed, "The is_fixed attribute is not found in the affine body geometry, why can it happen?");
                UIPC_ASSERT(is_dynamic, "The is_dynamic attribute is not found in the affine body geometry, why can it happen?");
                UIPC_ASSERT(external_kinetic, "The is_external_kinetic attribute is not found in the affine body geometry, why can it happen?");
                return zip(is_fixed->view(),
                           is_dynamic->view(),
                           external_kinetic->view());
            },
            [&](const ForEachInfo& I, auto&& data)
            {
                auto&& [fixed, dynamic, external_kinetic] = data;

                auto bodyI = I.global_index();

                h_body_id_to_is_fixed[bodyI]         = fixed;
                h_body_id_to_is_dynamic[bodyI]       = dynamic;
                h_body_id_to_external_kinetic[bodyI] = external_kinetic;
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
    async_copy(span{h_body_id_to_q_v}, body_id_to_q_v);
    async_copy(span{h_body_id_to_dim}, body_id_to_dim);
    async_copy(span{h_vertex_id_to_J}, vertex_id_to_J);
    async_copy(span{h_vertex_id_to_body_id}, vertex_id_to_body_id);
    async_copy(span{h_body_id_to_abd_mass}, body_id_to_abd_mass);
    async_copy(span{h_body_id_to_volume}, body_id_to_volume);
    async_copy(span{h_body_id_to_abd_mass_inv}, body_id_to_abd_mass_inv);
    async_copy(span{h_body_id_to_abd_gravity}, body_id_to_abd_gravity);
    async_copy(span{h_body_id_to_is_fixed}, body_id_to_is_fixed);
    async_copy(span{h_body_id_to_is_dynamic}, body_id_to_is_dynamic);
    async_copy(span{h_body_id_to_external_kinetic}, body_id_to_external_kinetic);

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

    async_resize(body_id_to_dq, abd_body_count, Vector12::Zero().eval());

    // setup body kinetic energy buffer
    async_resize(body_id_to_kinetic_energy, abd_body_count, Float{0});
    async_resize(body_id_to_shape_energy, abd_body_count, Float{0});

    // setup hessian and gradient buffers
    async_resize(diag_hessian, abd_body_count, Matrix12x12::Zero().eval());
    async_resize(body_id_to_shape_gradient, abd_body_count, Vector12::Zero().eval());
    async_resize(body_id_to_shape_hessian, abd_body_count, Matrix12x12::Zero().eval());
    async_resize(body_id_to_kinetic_gradient, abd_body_count, Vector12::Zero().eval());
    async_resize(body_id_to_kinetic_hessian, abd_body_count, Matrix12x12::Zero().eval());

    muda::wait_stream(nullptr);
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

void AffineBodyDynamics::Impl::set_dof_info(SizeT frame, IndexT dof_offset, IndexT dof_count)
{
    UIPC_ASSERT(frame > 0, "frame 0 is not used");
    if(frame_to_dof_count.size() <= frame)
    {
        frame_to_dof_count.resize(frame + 1, -1);
        frame_to_dof_offset.resize(frame + 1, -1);
    }
    frame_to_dof_count[frame]  = dof_count;
    frame_to_dof_offset[frame] = dof_offset;
}

void AffineBodyDynamics::Impl::_init_dof_info()
{
    frame_to_dof_count.reserve(1024);
    frame_to_dof_offset.reserve(1024);
    // frame 0 is not used, just fill 0
    frame_to_dof_offset.push_back(0);
    frame_to_dof_count.push_back(0);
}

void AffineBodyDynamics::Impl::_init_diff_reporters()
{
    //for(auto&& reporter : diff_parm_reporter.view())
    //{
    //    reporter->init();
    //}
}

void AffineBodyDynamics::Impl::_download_geometry_to_host()
{
    using namespace muda;

    auto aync_copy = []<typename T>(muda::DeviceBuffer<T>& src, span<T> dst)
    { muda::BufferLaunch().copy<T>(dst.data(), src.view()); };

    aync_copy(body_id_to_q, span{h_body_id_to_q});

    muda::wait_device();
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

IndexT AffineBodyDynamics::Impl::dof_offset(SizeT frame) const
{
    UIPC_ASSERT(frame > 0, "frame 0 is not used");
    return frame_to_dof_offset[frame];
}

IndexT AffineBodyDynamics::Impl::dof_count(SizeT frame) const
{
    UIPC_ASSERT(frame > 0, "frame 0 is not used");
    return frame_to_dof_count[frame];
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

namespace uipc::backend::cuda
{
AffineBodyDynamics::FilteredInfo::FilteredInfo(Impl* impl, SizeT constitution_index) noexcept
    : m_impl(impl)
    , m_constitution_index(constitution_index)
{
}

auto AffineBodyDynamics::FilteredInfo::geo_infos() const noexcept -> span<const GeoInfo>
{

    auto& constitution_info = m_impl->constitution_infos[m_constitution_index];
    return span{m_impl->geo_infos}.subspan(constitution_info.geo_offset,
                                           constitution_info.geo_count);
}

auto AffineBodyDynamics::FilteredInfo::constitution_info() const noexcept
    -> const ConstitutionInfo&
{
    return m_impl->constitution_infos[m_constitution_index];
}

SizeT AffineBodyDynamics::FilteredInfo::body_count() const noexcept
{
    return constitution_info().body_count;
}

SizeT AffineBodyDynamics::FilteredInfo::vertex_count() const noexcept
{
    return constitution_info().vertex_count;
}
}  // namespace uipc::backend::cuda