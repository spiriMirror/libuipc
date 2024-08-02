#include <finite_element/finite_element_method.h>
#include <finite_element/finite_element_constitution.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/common/map.h>
#include <uipc/common/zip.h>
#include <finite_element/fem_utils.h>
#include <uipc/common/algorithm/run_length_encode.h>

// constitutions
#include <finite_element/fem_3d_constitution.h>
#include <finite_element/codim_2d_constitution.h>
#include <finite_element/codim_1d_constitution.h>
#include <finite_element/codim_0d_constitution.h>

bool operator<(const uipc::backend::cuda::FiniteElementMethod::DimUID& a,
               const uipc::backend::cuda::FiniteElementMethod::DimUID& b)
{
    return a.dim < b.dim || (a.dim == b.dim && a.uid < b.uid);
}

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(FiniteElementMethod);

void FiniteElementMethod::add_constitution(FiniteElementConstitution* constitution)
{
    check_state(SimEngineState::BuildSystems, "add_constitution()");
    m_impl.constitutions.register_subsystem(*constitution);
}

void FiniteElementMethod::do_build()
{
    const auto& scene = world().scene();
    auto&       types = scene.constitution_tabular().types();
    if(types.find(world::ConstitutionType::FiniteElement) == types.end())
    {
        throw SimSystemException("No Finite Element Constitution found in the scene");
    }

    // find dependent systems
    auto& dof_predictor             = require<DoFPredictor>();
    auto& gradient_hessian_computer = require<GradientHessianComputer>();

    // Register the action to initialize the finite element geometry
    on_init_scene([this] { m_impl.init(world()); });

    // Register the action to write the scene
    on_write_scene([this] { m_impl.write_scene(world()); });

    gradient_hessian_computer.on_compute_gradient_hessian(
        *this,
        [this](GradientHessianComputer::ComputeInfo& info)
        { m_impl.compute_gradient_and_hessian(info); });
}

void FiniteElementMethod::Impl::init(WorldVisitor& world)
{
    _init_constitutions();
    _build_geo_infos(world);
    _build_on_host(world);
}

void FiniteElementMethod::Impl::_init_constitutions()
{
    auto constitution_view = constitutions.view();

    // 1) sort the constitutions by (dim, uid)
    std::sort(constitution_view.begin(),
              constitution_view.end(),
              [](const FiniteElementConstitution* a, const FiniteElementConstitution* b)
              {
                  auto   uida = a->constitution_uid();
                  auto   uidb = b->constitution_uid();
                  auto   dima = a->dimension();
                  auto   dimb = b->dimension();
                  DimUID uid_dim_a{dima, uida};
                  DimUID uid_dim_b{dimb, uidb};
                  return uid_dim_a < uid_dim_b;
              });

    for(auto&& [i, c] : enumerate(constitution_view))
        c->m_index = i;

    // 2) classify the constitutions
    codim_0d_constitutions.reserve(constitution_view.size());
    codim_1d_constitutions.reserve(constitution_view.size());
    codim_2d_constitutions.reserve(constitution_view.size());
    fem_3d_constitutions.reserve(constitution_view.size());

    for(auto&& constitution : constitution_view)
    {
        auto dim = constitution->dimension();
        switch(dim)
        {
            case 0: {
                auto derived = dynamic_cast<Codim0DConstitution*>(constitution);
                UIPC_ASSERT(derived, "The constitution is not a Codim0DConstitution, its dim = {}", dim);
                derived->m_index_in_dim = codim_0d_constitutions.size();
                codim_0d_constitutions.push_back(derived);
            }
            break;
            case 1: {
                auto derived = dynamic_cast<Codim1DConstitution*>(constitution);
                UIPC_ASSERT(derived, "The constitution is not a Codim1DConstitution, its dim = {}", dim);
                derived->m_index_in_dim = codim_1d_constitutions.size();
                codim_1d_constitutions.push_back(derived);
            }
            break;
            case 2: {
                auto derived = dynamic_cast<Codim2DConstitution*>(constitution);
                UIPC_ASSERT(derived, "The constitution is not a Codim2DConstitution, its dim = {}", dim);
                derived->m_index_in_dim = codim_2d_constitutions.size();
                codim_2d_constitutions.push_back(derived);
            }
            break;
            case 3: {
                auto derived = dynamic_cast<FEM3DConstitution*>(constitution);
                UIPC_ASSERT(derived, "The constitution is not a FEM3DConstitution, its dim = {}", dim);
                derived->m_index_in_dim = fem_3d_constitutions.size();
                fem_3d_constitutions.push_back(derived);
            }
            break;
            default:
                break;
        }
    }
}

void FiniteElementMethod::Impl::_build_geo_infos(WorldVisitor& world)
{
    set<U64> filter_uids;

    for(auto&& filter : constitutions.view())
        filter_uids.insert(filter->constitution_uid());

    // 1) find all the finite element constitutions
    auto geo_slots = world.scene().geometries();
    geo_infos.reserve(geo_slots.size());

    for(auto&& [i, geo_slot] : enumerate(geo_slots))
    {
        auto& geo  = geo_slot->geometry();
        auto  cuid = geo.meta().find<U64>(builtin::constitution_uid);
        if(cuid)
        {
            auto uid = cuid->view()[0];
            if(filter_uids.find(uid) != filter_uids.end())  // if exists
            {
                auto* sc = geo.as<geometry::SimplicialComplex>();
                UIPC_ASSERT(sc,
                            "The geometry is not a simplicial complex (it's {}). Why can it happen?",
                            geo.type());

                GeoInfo info;
                info.geo_slot_index = i;
                info.vertex_count   = sc->vertices().size();
                info.dim_uid.dim    = sc->dim();
                info.dim_uid.uid    = uid;

                switch(sc->dim())
                {
                    case 0:
                        info.primitive_count = sc->vertices().size();
                        break;
                    case 1:
                        info.primitive_count = sc->edges().size();
                        break;
                    case 2:
                        info.primitive_count = sc->triangles().size();
                        break;
                    case 3:
                        info.primitive_count = sc->tetrahedra().size();
                        break;
                    default:
                        break;
                }

                geo_infos.push_back(info);
            }
        }
    }

    // 2) sort geometry by (dim, uid)
    std::sort(geo_infos.begin(),
              geo_infos.end(),
              [](const GeoInfo& a, const GeoInfo& b)
              { return a.dim_uid < b.dim_uid; });

    // 3) setup vertex offsets
    auto count = geo_infos.size() + 1;  // add one to calculate the total size

    vector<SizeT> vertex_counts(count, 0);
    vector<SizeT> vertex_offsets(count, 0);

    std::transform(geo_infos.begin(),
                   geo_infos.end(),
                   vertex_counts.begin(),
                   [](const GeoInfo& info) { return info.vertex_count; });

    std::exclusive_scan(
        vertex_counts.begin(), vertex_counts.end(), vertex_offsets.begin(), 0);

    for(auto&& [i, info] : enumerate(geo_infos))
        info.vertex_offset = vertex_offsets[i];

    h_positions.resize(vertex_offsets.back());

    // 4) setup dim infos
    {
        std::array<SizeT, 4> dim_geo_counts;
        std::array<SizeT, 4> dim_geo_offsets;
        dim_geo_counts.fill(0);

        vector<SizeT> offsets;
        offsets.reserve(dim_geo_counts.size());
        vector<SizeT> counts;
        counts.reserve(dim_geo_counts.size());

        // encode the dimension
        encode_offset_count(geo_infos.begin(),
                            geo_infos.end(),
                            std::back_inserter(offsets),
                            std::back_inserter(counts),
                            [](const GeoInfo& current, const GeoInfo& value)
                            { return current.dim_uid.dim == value.dim_uid.dim; });

        for(auto&& [offset, count] : zip(offsets, counts))
        {
            auto& info                       = geo_infos[offset];
            dim_geo_counts[info.dim_uid.dim] = count;
        }

        std::exclusive_scan(
            dim_geo_counts.begin(), dim_geo_counts.end(), dim_geo_offsets.begin(), 0);

        for(auto&& [i, dim_info] : enumerate(dim_infos))
        {
            dim_info.geo_info_offset = dim_geo_offsets[i];
            dim_info.geo_info_count  = dim_geo_counts[i];
        }
    }


    // 4) setup primitive offsets
    for(auto&& [i, dim_info] : enumerate(dim_infos))
    {
        auto it = std::find_if(geo_infos.begin(),
                               geo_infos.end(),
                               [i](const GeoInfo& info)
                               { return info.dim_uid.dim == i; });

        if(it == geo_infos.end())
            continue;

        auto count = dim_info.geo_info_count + 1;  // add one to calculate the total size

        vector<SizeT> primitive_counts(count, 0);
        vector<SizeT> primitive_offsets(count, 0);

        auto geo_span =
            span{geo_infos}.subspan(dim_info.geo_info_offset, dim_info.geo_info_count);

        std::ranges::transform(geo_span,
                               primitive_counts.begin(),
                               [](const GeoInfo& info)
                               { return info.primitive_count; });

        std::exclusive_scan(primitive_counts.begin(),
                            primitive_counts.end(),
                            primitive_offsets.begin(),
                            0);

        for(auto&& [j, info] : enumerate(geo_span))
        {
            info.primitive_offset = primitive_offsets[j];
            info.primitive_count  = primitive_counts[j];
        }

        dim_info.primitive_count = primitive_offsets.back();
    }

    h_codim_0ds.resize(dim_infos[0].primitive_count);
    h_codim_1ds.resize(dim_infos[1].primitive_count);
    h_codim_2ds.resize(dim_infos[2].primitive_count);
    h_tets.resize(dim_infos[3].primitive_count);
}

void FiniteElementMethod::Impl::_build_constitution_infos()
{
    auto build_infos =
        [&]<std::derived_from<FiniteElementConstitution> ConstitutionT>(
            vector<ConstitutionInfo>& infos, span<ConstitutionT*> constitutions, IndexT dim)
    {
        infos.resize(constitutions.size());
        vector<SizeT> vertex_counts(infos.size(), 0);
        vector<SizeT> primitive_counts(infos.size(), 0);
        vector<SizeT> geometry_counts(infos.size(), 0);

        const auto& dim_info = dim_infos[dim];

        auto geo_info_subspan =
            span{geo_infos}.subspan(dim_info.geo_info_offset, dim_info.geo_info_count);


        {
            // 1) setup geo_info count per constitution
            vector<SizeT> offsets;
            offsets.reserve(constitutions.size());
            vector<SizeT> counts;
            counts.reserve(constitutions.size());

            // encode the constitution uid
            encode_offset_count(geo_info_subspan.begin(),
                                geo_info_subspan.end(),
                                std::back_inserter(offsets),
                                std::back_inserter(counts),
                                [](const GeoInfo& current, const GeoInfo& value) {
                                    return current.dim_uid.uid == value.dim_uid.uid;
                                });

            for(auto&& [offset, count] : zip(offsets, counts))
            {
                geometry_counts[offset] = count;
            }
        }


        vector<SizeT> vertex_offsets(infos.size(), 0);
        vector<SizeT> primitive_offsets(infos.size(), 0);
        vector<SizeT> geometry_offsets(infos.size(), 0);

        SizeT       dim_geo_offset    = dim_info.geo_info_offset;
        const auto& begin_geo         = geo_infos[dim_geo_offset];
        SizeT       dim_vertex_offset = begin_geo.vertex_offset;


        std::exclusive_scan(vertex_counts.begin(),
                            vertex_counts.end(),
                            vertex_offsets.begin(),
                            dim_vertex_offset);

        std::exclusive_scan(primitive_counts.begin(),
                            primitive_counts.end(),
                            primitive_offsets.begin(),
                            0);

        std::exclusive_scan(geometry_counts.begin(),
                            geometry_counts.end(),
                            geometry_offsets.begin(),
                            dim_geo_offset);

        for(auto&& [i, info] : enumerate(infos))
        {
            info.vertex_count     = vertex_counts[i];
            info.vertex_offset    = vertex_offsets[i];
            info.primitive_count  = primitive_counts[i];
            info.primitive_offset = primitive_offsets[i];
            info.geo_info_count   = geometry_counts[i];
            info.geo_info_offset  = geometry_offsets[i];
        }
    };


    build_infos(codim_0d_constitution_infos, span{codim_0d_constitutions}, 0);
    build_infos(codim_1d_constitution_infos, span{codim_1d_constitutions}, 1);
    build_infos(codim_2d_constitution_infos, span{codim_2d_constitutions}, 2);
    build_infos(fem_3d_constitution_infos, span{fem_3d_constitutions}, 3);
}

void FiniteElementMethod::Impl::_build_on_host(WorldVisitor& world)
{
    auto geo_slots      = world.scene().geometries();
    auto rest_geo_slots = world.scene().rest_geometries();

    // resize buffers
    h_rest_positions.resize(h_positions.size());
    h_mass.resize(h_positions.size());

    for(auto&& [i, info] : enumerate(geo_infos))
    {
        auto& geo_slot      = geo_slots[info.geo_slot_index];
        auto& rest_geo_slot = rest_geo_slots[info.geo_slot_index];
        auto& geo           = geo_slot->geometry();
        auto& rest_geo      = rest_geo_slot->geometry();
        auto* sc            = geo.as<geometry::SimplicialComplex>();
        UIPC_ASSERT(sc,
                    "The geometry is not a simplicial complex (it's {}). Why can it happen?",
                    geo.type());
        auto* rest_sc = rest_geo.as<geometry::SimplicialComplex>();
        UIPC_ASSERT(rest_sc,
                    "The geometry is not a simplicial complex (it's {}). Why can it happen?",
                    rest_geo.type());

        // 1) setup primitives
        switch(sc->dim())
        {
            case 0: {
                auto dst_codim_0d_span =
                    span{h_codim_0ds}.subspan(info.primitive_offset, info.primitive_count);
                std::iota(dst_codim_0d_span.begin(), dst_codim_0d_span.end(), info.vertex_offset);
            }
            break;
            case 1: {
                auto dst_codim_1d_span =
                    span{h_codim_1ds}.subspan(info.primitive_offset, info.primitive_count);

                auto edge_view = sc->edges().topo().view();
                UIPC_ASSERT(edge_view.size() == dst_codim_1d_span.size(), "edge size mismatching");

                std::transform(edge_view.begin(),
                               edge_view.end(),
                               dst_codim_1d_span.begin(),
                               [&](const Vector2i& edge) -> Vector2i
                               { return edge.array() + info.vertex_offset; });
            }
            break;
            case 2: {
                auto dst_codim_2d_span =
                    span{h_codim_2ds}.subspan(info.primitive_offset, info.primitive_count);

                auto tri_view = sc->triangles().topo().view();
                UIPC_ASSERT(tri_view.size() == dst_codim_2d_span.size(),
                            "triangle size mismatching");

                std::transform(tri_view.begin(),
                               tri_view.end(),
                               dst_codim_2d_span.begin(),
                               [&](const Vector3i& tri) -> Vector3i
                               { return tri.array() + info.vertex_offset; });
            }
            break;
            case 3: {
                auto dst_tet_span =
                    span{h_tets}.subspan(info.primitive_offset, info.primitive_count);

                auto tet_view = sc->tetrahedra().topo().view();
                UIPC_ASSERT(tet_view.size() == dst_tet_span.size(), "tetrahedra size mismatching");

                std::transform(tet_view.begin(),
                               tet_view.end(),
                               dst_tet_span.begin(),
                               [&](const Vector4i& tet) -> Vector4i
                               { return tet.array() + info.vertex_offset; });
            }
            break;
            default:
                break;
        }

        // 2) setup positions
        auto pos_view = sc->positions().view();
        auto dst_pos_span = span{h_positions}.subspan(info.vertex_offset, info.vertex_count);
        UIPC_ASSERT(pos_view.size() == dst_pos_span.size(), "position size mismatching");
        std::copy(pos_view.begin(), pos_view.end(), dst_pos_span.begin());

        auto rest_pos_view = rest_sc->positions().view();
        auto dst_rest_pos_span =
            span{h_rest_positions}.subspan(info.vertex_offset, info.vertex_count);
        UIPC_ASSERT(rest_pos_view.size() == dst_rest_pos_span.size(),
                    "rest position size mismatching");
        std::copy(rest_pos_view.begin(), rest_pos_view.end(), dst_rest_pos_span.begin());

        // 3) setup mass
        auto mass      = sc->vertices().find<Float>(builtin::mass);
        auto mass_view = mass->view();
        auto dst_mass_span = span{h_mass}.subspan(info.vertex_offset, info.vertex_count);
    }
}

void FiniteElementMethod::Impl::_build_on_device()
{
    using namespace muda;

    x.resize(h_positions.size());
    x.view().copy_from(h_positions.data());

    x_bar.resize(h_rest_positions.size());
    x_bar.view().copy_from(h_rest_positions.data());

    x_temp  = x;
    x_tilde = x;

    dx.resize(x.size(), Vector3::Zero());
    v.resize(x.size(), Vector3::Zero());

    mass.resize(h_mass.size());
    mass.view().copy_from(h_mass.data());

    codim_0ds.resize(h_codim_0ds.size());
    codim_0ds.view().copy_from(h_codim_0ds.data());

    codim_1ds.resize(h_codim_1ds.size());
    codim_1ds.view().copy_from(h_codim_1ds.data());

    codim_2ds.resize(h_codim_2ds.size());
    codim_2ds.view().copy_from(h_codim_2ds.data());

    tets.resize(h_tets.size());
    tets.view().copy_from(h_tets.data());

    // calculate FEM3D Material Basis
    Dm3x3_inv.resize(tets.size());

    ParallelFor()
        .kernel_name("FEM3D Material Basis")
        .apply(tets.size(),
               [tets   = tets.viewer().name("tets"),
                x_bars = x_bar.viewer().name("x_bars"),
                Dm9x9_inv = Dm3x3_inv.viewer().name("Dm3x3_inv")] __device__(int i) mutable
               {
                   const Vector4i& tet = tets(i);
                   const Vector3&  x0  = x_bars(tet[0]);
                   const Vector3&  x1  = x_bars(tet[1]);
                   const Vector3&  x2  = x_bars(tet[2]);
                   const Vector3&  x3  = x_bars(tet[3]);

                   Dm9x9_inv(i) = Dm_inv(x0, x1, x2, x3);
               });
}

void FiniteElementMethod::Impl::write_scene(WorldVisitor& world)
{
    _download_geometry_to_host();

    auto geo_slots = world.scene().geometries();

    auto position_span = span{h_positions};

    for(auto&& [i, info] : enumerate(geo_infos))
    {
        auto& geo_slot = geo_slots[info.geo_slot_index];
        auto& geo      = geo_slot->geometry();
        auto* sc       = geo.as<geometry::SimplicialComplex>();
        UIPC_ASSERT(sc,
                    "The geometry is not a simplicial complex (it's {}). Why can it happen?",
                    geo.type());

        // 1) write positions back
        auto pos_view = geometry::view(sc->positions());
        auto src_pos_span = position_span.subspan(info.vertex_offset, info.vertex_count);
        UIPC_ASSERT(pos_view.size() == src_pos_span.size(), "position size mismatching");
        std::copy(src_pos_span.begin(), src_pos_span.end(), pos_view.begin());

        // 2) write primitives back
        // TODO:
        // Now there is no topology modification, so no need to write back
        // In the future, we may need to write back the topology if the topology is modified
    }
}

void FiniteElementMethod::Impl::_download_geometry_to_host()
{
    x.view().copy_to(h_positions.data());
}

void FiniteElementMethod::Impl::_distribute_constitution_infos() {}

void FiniteElementMethod::Impl::compute_gradient_and_hessian(GradientHessianComputer::ComputeInfo& info)
{
    //
}

}  // namespace uipc::backend::cuda

namespace uipc::backend::cuda
{
auto FiniteElementMethod::FEM3DFilteredInfo::geo_infos() const noexcept -> span<const GeoInfo>
{
    return span<const GeoInfo>();
}
}  // namespace uipc::backend::cuda