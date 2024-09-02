#include <finite_element/finite_element_method.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <finite_element/finite_element_constitution.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/common/map.h>
#include <uipc/common/zip.h>
#include <finite_element/fem_utils.h>
#include <uipc/common/algorithm/run_length_encode.h>
#include <uipc/common/json_eigen.h>
#include <muda/ext/eigen/inverse.h>
#include <ranges>

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
    if(types.find(constitution::ConstitutionType::FiniteElement) == types.end())
    {
        throw SimSystemException("No Finite Element Constitution found in the scene");
    }

    m_impl.gravity = scene.info()["gravity"].get<Vector3>();

    m_impl.global_vertex_manager = &require<GlobalVertexManager>();

    auto& dof_predictor = require<DoFPredictor>();

    dof_predictor.on_predict(*this,
                             [this](DoFPredictor::PredictInfo& info)
                             { m_impl.compute_x_tilde(info); });

    dof_predictor.on_compute_velocity(*this,
                                      [this](DoFPredictor::ComputeVelocityInfo& info)
                                      { m_impl.compute_velocity(info); });

    // Register the action to initialize the finite element geometry
    on_init_scene([this] { m_impl.init(world()); });

    // Register the action to write the scene
    on_write_scene([this] { m_impl.write_scene(world()); });
}

void FiniteElementMethod::Impl::init(WorldVisitor& world)
{
    _init_constitutions();
    _build_geo_infos(world);
    _build_constitution_infos();
    _build_on_host(world);
    _build_on_device();
    _distribute_constitution_filtered_info();
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
                codim_0d_uid_to_index.insert(
                    {derived->constitution_uid(), derived->m_index_in_dim});
            }
            break;
            case 1: {
                auto derived = dynamic_cast<Codim1DConstitution*>(constitution);
                UIPC_ASSERT(derived, "The constitution is not a Codim1DConstitution, its dim = {}", dim);
                derived->m_index_in_dim = codim_1d_constitutions.size();
                codim_1d_constitutions.push_back(derived);
                codim_1d_uid_to_index.insert(
                    {derived->constitution_uid(), derived->m_index_in_dim});
            }
            break;
            case 2: {
                auto derived = dynamic_cast<Codim2DConstitution*>(constitution);
                UIPC_ASSERT(derived, "The constitution is not a Codim2DConstitution, its dim = {}", dim);
                derived->m_index_in_dim = codim_2d_constitutions.size();
                codim_2d_constitutions.push_back(derived);
                codim_2d_uid_to_index.insert(
                    {derived->constitution_uid(), derived->m_index_in_dim});
            }
            break;
            case 3: {
                auto derived = dynamic_cast<FEM3DConstitution*>(constitution);
                UIPC_ASSERT(derived, "The constitution is not a FEM3DConstitution, its dim = {}", dim);
                derived->m_index_in_dim = fem_3d_constitutions.size();
                fem_3d_constitutions.push_back(derived);
                fem_3d_uid_to_index.insert({derived->constitution_uid(), derived->m_index_in_dim});
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

        auto count = dim_info.geo_info_count + 1;  // + 1 to calculate the total size

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
        dim_info.primitive_offset = geo_infos[dim_info.geo_info_offset].primitive_offset;
    }

    h_codim_0ds.resize(dim_infos[0].primitive_count);
    h_codim_1ds.resize(dim_infos[1].primitive_count);
    h_codim_2ds.resize(dim_infos[2].primitive_count);
    h_tets.resize(dim_infos[3].primitive_count);
}

void FiniteElementMethod::Impl::_build_constitution_infos()
{
    auto build_infos = [&]<std::derived_from<FiniteElementConstitution> ConstitutionT>(
                           vector<ConstitutionInfo>& infos,
                           span<ConstitutionT*>      constitutions,
                           IndexT                    dim,
                           unordered_map<U64, SizeT> uid_to_index)
    {
        infos.resize(constitutions.size());
        vector<SizeT> vertex_counts(infos.size(), 0);
        vector<SizeT> primitive_counts(infos.size(), 0);
        vector<SizeT> geometry_counts(infos.size(), 0);

        const auto& dim_info = dim_infos[dim];

        auto geo_info_subspan =
            span{geo_infos}.subspan(dim_info.geo_info_offset, dim_info.geo_info_count);


        for(auto&& geo_info : geo_info_subspan)
        {
            auto index = uid_to_index[geo_info.dim_uid.uid];
            geometry_counts[index]++;
            vertex_counts[index] += geo_info.vertex_count;
            primitive_counts[index] += geo_info.primitive_count;
        }

        vector<SizeT> vertex_offsets(infos.size(), 0);
        vector<SizeT> primitive_offsets(infos.size(), 0);
        vector<SizeT> geometry_offsets(infos.size(), 0);

        SizeT dim_geo_offset    = dim_info.geo_info_offset;
        SizeT dim_vertex_offset = 0;

        if(geo_infos.size() > 0)
        {
            const auto& begin_geo         = geo_infos[dim_geo_offset];
            SizeT       dim_vertex_offset = begin_geo.vertex_offset;
        }

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


    build_infos(codim_0d_constitution_infos, span{codim_0d_constitutions}, 0, codim_0d_uid_to_index);
    build_infos(codim_1d_constitution_infos, span{codim_1d_constitutions}, 1, codim_1d_uid_to_index);
    build_infos(codim_2d_constitution_infos, span{codim_2d_constitutions}, 2, codim_2d_uid_to_index);
    build_infos(fem_3d_constitution_infos, span{fem_3d_constitutions}, 3, fem_3d_uid_to_index);
}

void FiniteElementMethod::Impl::_build_on_host(WorldVisitor& world)
{
    auto geo_slots      = world.scene().geometries();
    auto rest_geo_slots = world.scene().rest_geometries();

    // resize buffers
    h_rest_positions.resize(h_positions.size());
    h_thicknesses.resize(h_positions.size(), 0);  // fill 0 for default
    h_masses.resize(h_positions.size());
    h_vertex_contact_element_ids.resize(h_positions.size(), 0);  // fill 0 for default
    h_vertex_is_fixed.resize(h_positions.size(), 0);  // fill 0 for default

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

        {  // 2) fill backend_fem_vertex_offset in geometry
            auto vertex_offset = sc->meta().find<IndexT>(builtin::backend_fem_vertex_offset);
            if(!vertex_offset)
                vertex_offset =
                    sc->meta().create<IndexT>(builtin::backend_fem_vertex_offset, -1);
            auto vertex_offset_view = geometry::view(*vertex_offset);
            std::ranges::fill(vertex_offset_view, info.vertex_offset);
        }

        {  // 3) setup positions
            auto pos_view = sc->positions().view();
            auto dst_pos_span =
                span{h_positions}.subspan(info.vertex_offset, info.vertex_count);
            UIPC_ASSERT(pos_view.size() == dst_pos_span.size(), "position size mismatching");
            std::copy(pos_view.begin(), pos_view.end(), dst_pos_span.begin());

            auto rest_pos_view = rest_sc->positions().view();
            auto dst_rest_pos_span =
                span{h_rest_positions}.subspan(info.vertex_offset, info.vertex_count);
            UIPC_ASSERT(rest_pos_view.size() == dst_rest_pos_span.size(),
                        "rest position size mismatching");
            std::ranges::copy(rest_pos_view, dst_rest_pos_span.begin());
        }

        {  // 4) setup mass
            auto mass      = sc->vertices().find<Float>(builtin::mass);
            auto mass_view = mass->view();
            auto dst_mass_span =
                span{h_masses}.subspan(info.vertex_offset, info.vertex_count);
            UIPC_ASSERT(mass_view.size() == dst_mass_span.size(), "mass size mismatching");
            std::ranges::copy(mass_view, dst_mass_span.begin());
        }

        {  // 5) setup thickness
            auto thickness = sc->vertices().find<Float>(builtin::thickness);
            auto dst_thickness_span =
                span{h_thicknesses}.subspan(info.vertex_offset, info.vertex_count);

            if(thickness)
            {
                auto thickness_view = thickness->view();
                UIPC_ASSERT(thickness_view.size() == dst_thickness_span.size(),
                            "thickness size mismatching");
                std::ranges::copy(thickness_view, dst_thickness_span.begin());
            }
        }


        {  // 6) setup vertex contact element id
            auto ceid = sc->vertices().find<IndexT>(builtin::contact_element_id);
            auto dst_eid_span =
                span{h_vertex_contact_element_ids}.subspan(info.vertex_offset,
                                                           info.vertex_count);

            if(ceid)
            {
                auto eid_view = ceid->view();
                UIPC_ASSERT(eid_view.size() == dst_eid_span.size(),
                            "contact_element_id size mismatching");
                std::ranges::copy(eid_view, dst_eid_span.begin());
            }
        }

        {  // 7) setup vertex is_fixed

            auto is_fixed = sc->vertices().find<IndexT>(builtin::is_fixed);
            auto dst_is_fixed_span =
                span{h_vertex_is_fixed}.subspan(info.vertex_offset, info.vertex_count);

            if(is_fixed)
            {
                auto is_fixed_view = is_fixed->view();
                UIPC_ASSERT(is_fixed_view.size() == dst_is_fixed_span.size(),
                            "is_fixed size mismatching");
                std::ranges::copy(is_fixed_view, dst_is_fixed_span.begin());
            }
        }
    }
}

void FiniteElementMethod::Impl::_build_on_device()
{
    using namespace muda;

    // 1) Vertex States
    xs.resize(h_positions.size());
    xs.view().copy_from(h_positions.data());

    x_bars.resize(h_rest_positions.size());
    x_bars.view().copy_from(h_rest_positions.data());

    x_temps  = xs;
    x_tildes = xs;
    x_prevs  = xs;

    is_fixed.resize(h_vertex_is_fixed.size());
    is_fixed.view().copy_from(h_vertex_is_fixed.data());

    dxs.resize(xs.size(), Vector3::Zero());
    vs.resize(xs.size(), Vector3::Zero());

    masses.resize(h_masses.size());
    masses.view().copy_from(h_masses.data());

    thicknesses.resize(h_thicknesses.size());
    thicknesses.view().copy_from(h_thicknesses.data());

    diag_hessians.resize(xs.size());

    // 2) Elements
    codim_0ds.resize(h_codim_0ds.size());
    codim_0ds.view().copy_from(h_codim_0ds.data());

    codim_1ds.resize(h_codim_1ds.size());
    codim_1ds.view().copy_from(h_codim_1ds.data());
    rest_lengths.resize(codim_1ds.size());

    codim_2ds.resize(h_codim_2ds.size());
    codim_2ds.view().copy_from(h_codim_2ds.data());
    rest_areas.resize(codim_2ds.size());

    tets.resize(h_tets.size());
    tets.view().copy_from(h_tets.data());
    rest_volumes.resize(tets.size());

    // 3) Material Space Attribute
    // Rod
    ParallelFor()
        .kernel_name("Rod Basis")
        .apply(codim_1ds.size(),
               [codim_1ds = codim_1ds.viewer().name("codim_1ds"),
                x_bars    = x_bars.viewer().name("x_bars"),
                rest_lengths = rest_lengths.viewer().name("rest_lengths")] __device__(int i) mutable
               {
                   const Vector2i& edge = codim_1ds(i);
                   const Vector3&  x0   = x_bars(edge[0]);
                   const Vector3&  x1   = x_bars(edge[1]);

                   rest_lengths(i) = (x1 - x0).norm();
               });


    // Shell
    ParallelFor()
        .kernel_name("Shell Basis")
        .apply(codim_2ds.size(),
               [codim_2ds = codim_2ds.viewer().name("codim_2ds"),
                x_bars    = x_bars.viewer().name("x_bars"),
                rest_areas = rest_areas.viewer().name("rest_areas")] __device__(int i) mutable
               {
                   const Vector3i& tri = codim_2ds(i);
                   const Vector3&  x0  = x_bars(tri[0]);
                   const Vector3&  x1  = x_bars(tri[1]);
                   const Vector3&  x2  = x_bars(tri[2]);

                   Vector3 E01 = x1 - x0;
                   Vector3 E02 = x2 - x0;

                   rest_areas(i) = 0.5 * E01.cross(E02).norm();
               });

    // FEM3D Material Basis
    Dm3x3_invs.resize(tets.size());
    ParallelFor()
        .kernel_name("FEM3D Material Basis")
        .apply(tets.size(),
               [tets      = tets.viewer().name("tets"),
                x_bars    = x_bars.viewer().name("x_bars"),
                Dm9x9_inv = Dm3x3_invs.viewer().name("Dm3x3_inv"),
                rest_volumes = rest_volumes.viewer().name("rest_volumes")] __device__(int i) mutable
               {
                   const Vector4i& tet = tets(i);
                   const Vector3&  x0  = x_bars(tet[0]);
                   const Vector3&  x1  = x_bars(tet[1]);
                   const Vector3&  x2  = x_bars(tet[2]);
                   const Vector3&  x3  = x_bars(tet[3]);

                   Dm9x9_inv(i) = fem::Dm_inv(x0, x1, x2, x3);
                   Float V      = fem::Ds(x0, x1, x2, x3).determinant();
                   MUDA_ASSERT(V > 0.0,
                               "Negative volume tetrahedron (%d, %d, %d, %d)",
                               tet[0],
                               tet[1],
                               tet[2],
                               tet[3]);
                   rest_volumes(i) = V;
               });

    // 4) Allocate memory for energy, gradient and hessian
    vertex_kinetic_energies.resize(xs.size());
    G3s.resize(xs.size());
    H3x3s.resize(xs.size());

    auto constitution_count = constitutions.view().size();

    codim_1d_elastic_energy = 0;
    codim_1d_elastic_energies.resize(codim_1ds.size());
    G6s.resize(codim_1ds.size());
    H6x6s.resize(codim_1ds.size());

    codim_2d_elastic_energy = 0;
    codim_2d_elastic_energies.resize(codim_2ds.size());
    G9s.resize(codim_2ds.size());
    H9x9s.resize(codim_2ds.size());

    fem_3d_elastic_energy = 0;
    fem_3d_elastic_energies.resize(tets.size());
    G12s.resize(tets.size());
    H12x12s.resize(tets.size());
}

void FiniteElementMethod::Impl::_distribute_constitution_filtered_info()
{
    for(auto&& [i, c] : enumerate(codim_0d_constitutions))
    {
        Codim0DFilteredInfo filtered_info{this};
        filtered_info.m_index_in_dim = i;
        c->retrieve(filtered_info);
    }

    for(auto&& [i, c] : enumerate(codim_1d_constitutions))
    {
        Codim1DFilteredInfo filtered_info{this};
        filtered_info.m_index_in_dim = i;
        c->retrieve(filtered_info);
    }

    for(auto&& [i, c] : enumerate(codim_2d_constitutions))
    {
        Codim2DFilteredInfo filtered_info{this};
        filtered_info.m_index_in_dim = i;
        c->retrieve(filtered_info);
    }

    for(auto&& [i, c] : enumerate(fem_3d_constitutions))
    {
        FEM3DFilteredInfo filtered_info{this};
        filtered_info.m_index_in_dim = i;
        c->retrieve(filtered_info);
    }
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
    xs.view().copy_to(h_positions.data());
}

void FiniteElementMethod::Impl::compute_x_tilde(DoFPredictor::PredictInfo& info)
{
    using namespace muda;
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(xs.size(),
               [is_fixed = is_fixed.cviewer().name("fixed"),
                x_prevs  = x_prevs.cviewer().name("x_prevs"),
                vs       = vs.cviewer().name("vs"),
                x_tildes = x_tildes.viewer().name("x_tildes"),
                g        = gravity,
                dt       = info.dt()] __device__(int i) mutable
               {
                   const Vector3& x_prev = x_prevs(i);
                   const Vector3& v      = vs(i);
                   // TODO: this time, we only consider gravity
                   if(is_fixed(i))
                   {
                       x_tildes(i) = x_prev;
                   }
                   else
                   {
                       x_tildes(i) = x_prev + v * dt + g * (dt * dt);
                   }
               });
}

void FiniteElementMethod::Impl::compute_velocity(DoFPredictor::ComputeVelocityInfo& info)
{
    using namespace muda;
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(xs.size(),
               [is_fixed = is_fixed.cviewer().name("fixed"),
                xs       = xs.cviewer().name("xs"),
                vs       = vs.viewer().name("vs"),
                x_prevs  = x_prevs.viewer().name("x_prevs"),
                dt       = info.dt()] __device__(int i) mutable
               {
                   Vector3& v      = vs(i);
                   Vector3& x_prev = x_prevs(i);

                   const Vector3& x = xs(i);

                   if(is_fixed(i))
                       v = Vector3::Zero();
                   else
                       v = (x - x_prev) * (1.0 / dt);

                   x_prev = x;
               });
}
}  // namespace uipc::backend::cuda


// Info:
namespace uipc::backend::cuda
{
Float FiniteElementMethod::ComputeEnergyInfo::dt() const noexcept
{
    return m_dt;
}

FiniteElementMethod::ComputeGradientHessianInfo::ComputeGradientHessianInfo(Float dt) noexcept
    : m_dt(dt)
{
}
}  // namespace uipc::backend::cuda