#include <finite_element/finite_element_method.h>
#include <dof_predictor.h>
#include <gradient_hessian_computer.h>
#include <finite_element/finite_element_constitution.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/common/map.h>
#include <uipc/common/zip.h>

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
        throw SimSystemException("No FiniteElement Constitution found in the scene");
    }

    // find dependent systems
    auto& dof_predictor             = require<DoFPredictor>();
    auto& gradient_hessian_computer = require<GradientHessianComputer>();

    // Register the action to initialize the finite element geometry
    on_init_scene([this] { m_impl.init(world()); });

    // Register the action to write the scene
    on_write_scene([this] { m_impl.write_scene(world()); });
}

void FiniteElementMethod::Impl::init(WorldVisitor& world)
{
    _init_constitutions();
    _init_fem_geo_infos(world);
    _build_on_host(world);
}

void FiniteElementMethod::Impl::_init_constitutions()
{
    constitutions.init();

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
                codim_0d_constitutions.push_back(derived);
            }
            break;
            case 1: {
                auto derived = dynamic_cast<Codim1DConstitution*>(constitution);
                UIPC_ASSERT(derived, "The constitution is not a Codim1DConstitution, its dim = {}", dim);
                codim_1d_constitutions.push_back(derived);
            }
            break;
            case 2: {
                auto derived = dynamic_cast<Codim2DConstitution*>(constitution);
                UIPC_ASSERT(derived, "The constitution is not a Codim2DConstitution, its dim = {}", dim);
                codim_2d_constitutions.push_back(derived);
            }
            break;
            case 3: {
                auto derived = dynamic_cast<FEM3DConstitution*>(constitution);
                UIPC_ASSERT(derived, "The constitution is not a FEM3DConstitution, its dim = {}", dim);
                fem_3d_constitutions.push_back(derived);
            }
            break;
            default:
                break;
        }
    }
}

void FiniteElementMethod::Impl::_init_fem_geo_infos(WorldVisitor& world)
{
    set<U64> filter_uids;

    for(auto&& filter : constitutions.view())
        filter_uids.insert(filter->constitution_uid());

    // 1) find all the finite element constitutions
    auto geo_slots = world.scene().geometries();
    fem_geo_infos.reserve(geo_slots.size());

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

                fem_geo_infos.push_back(info);
            }
        }
    }

    // 2) sort geometry by (dim, uid)
    std::sort(fem_geo_infos.begin(),
              fem_geo_infos.end(),
              [](const GeoInfo& a, const GeoInfo& b)
              { return a.dim_uid < b.dim_uid; });

    // 3) setup vertex offsets
    auto count = fem_geo_infos.size() + 1;  // add one to calculate the total size

    vector<SizeT> vertex_counts(count, 0);
    vector<SizeT> vertex_offsets(count, 0);

    std::transform(fem_geo_infos.begin(),
                   fem_geo_infos.end(),
                   vertex_counts.begin(),
                   [](const GeoInfo& info) { return info.vertex_count; });

    std::exclusive_scan(
        vertex_counts.begin(), vertex_counts.end(), vertex_offsets.begin(), 0);

    for(auto&& [i, info] : enumerate(fem_geo_infos))
        info.vertex_offset = vertex_offsets[i];

    h_positions.resize(vertex_offsets.back());

    // 4) setup primitive offsets
    std::array<SizeT, 4> total_primitive_counts;
    total_primitive_counts.fill(0);

    // from 0D to 3D
    for(SizeT i = 0; i < 4; ++i)
    {
        auto it = std::find_if(fem_geo_infos.begin(),
                               fem_geo_infos.end(),
                               [i](const GeoInfo& info)
                               { return info.dim_uid.dim == i; });

        if(it == fem_geo_infos.end())
            continue;

        // if found, try to figure out the boundary
        auto next_it = std::find_if(it,
                                    fem_geo_infos.end(),
                                    [i](const GeoInfo& info)
                                    {
                                        return info.dim_uid.dim == i + 1;  // next dim
                                    });

        auto count = std::distance(it, next_it) + 1;  // add one to calculate the total size

        dim_geo_info_counts[i] = count;

        vector<SizeT> primitive_counts(count, 0);
        vector<SizeT> primitive_offsets(count, 0);

        auto geo_span = span<GeoInfo>(it, next_it);

        std::ranges::transform(geo_span,
                               primitive_counts.begin(),
                               [](const GeoInfo& info)
                               { return info.primitive_count; });

        std::exclusive_scan(primitive_counts.begin(),
                            primitive_counts.end(),
                            primitive_offsets.begin(),
                            0);

        total_primitive_counts[i] = primitive_offsets.back();

        for(auto&& [j, info] : enumerate(geo_span))
        {
            info.primitive_offset = primitive_offsets[j];
            info.primitive_count  = primitive_counts[j];
        }
    }

    // 5) setup the geometry offsets for each dim
    std::exclusive_scan(dim_geo_info_counts.begin(),
                        dim_geo_info_counts.end(),
                        dim_geo_info_offsets.begin(),
                        0);

    h_codim_0ds.resize(total_primitive_counts[0]);
    h_codim_1ds.resize(total_primitive_counts[1]);
    h_codim_2ds.resize(total_primitive_counts[2]);
    h_tets.resize(total_primitive_counts[3]);
}

void FiniteElementMethod::Impl::_build_on_host(WorldVisitor& world)
{
    auto geo_slots = world.scene().geometries();

    auto position_span = span{h_positions};

    for(auto&& [i, info] : enumerate(fem_geo_infos))
    {
        auto& geo_slot = geo_slots[info.geo_slot_index];
        auto& geo      = geo_slot->geometry();
        auto* sc       = geo.as<geometry::SimplicialComplex>();
        UIPC_ASSERT(sc,
                    "The geometry is not a simplicial complex (it's {}). Why can it happen?",
                    geo.type());

        // 1) setup positions
        auto pos_view = sc->positions().view();
        auto dst_pos_span = position_span.subspan(info.vertex_offset, info.vertex_count);
        UIPC_ASSERT(pos_view.size() == dst_pos_span.size(), "position size mismatching");
        std::copy(pos_view.begin(), pos_view.end(), dst_pos_span.begin());

        // 2) setup primitives
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
    }
}

void FiniteElementMethod::Impl::_build_on_device()
{
    x.resize(h_positions.size());
    x.view().copy_from(h_positions.data());
}

void FiniteElementMethod::Impl::write_scene(WorldVisitor& world)
{
    _download_geometry_to_host();

    auto geo_slots = world.scene().geometries();

    auto position_span = span{h_positions};

    for(auto&& [i, info] : enumerate(fem_geo_infos))
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
}  // namespace uipc::backend::cuda