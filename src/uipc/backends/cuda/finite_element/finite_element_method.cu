#include <finite_element/finite_element_method.h>
#include <dof_predictor.h>
#include <gradient_hessian_computer.h>
#include <finite_element/finite_element_constitution.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/common/map.h>

bool operator<(const uipc::backend::cuda::FiniteElementMethod::DimUID& a,
               const uipc::backend::cuda::FiniteElementMethod::DimUID& b)
{
    return a.dim < b.dim || (a.dim == b.dim && a.uid < b.uid);
}

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(FiniteElementMethod);

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
    constitutions.init();

    _init_fem_geo_infos(world);
}

void FiniteElementMethod::Impl::_init_fem_geo_infos(WorldVisitor& world)
{
    // sort the constituions by their (dim, uid)
    auto constitution_view = constitutions.view();
    std::sort(constitution_view.begin(),
              constitution_view.end(),
              [](FiniteElementConstitution* a, FiniteElementConstitution* b)
              {
                  auto   uida = a->constitution_uid();
                  auto   uidb = b->constitution_uid();
                  auto   dima = a->dimension();
                  auto   dimb = b->dimension();
                  DimUID uid_dim_a{dima, uida};
                  DimUID uid_dim_b{dimb, uidb};
                  return uid_dim_a < uid_dim_b;
              });

    set<U64> filter_uids;

    for(auto&& filter : constitution_view)
        filter_uids.insert(filter->constitution_uid());

    // TOFIX:
    map<DimUID, IndexT> constitution_uid_to_index;
    for(auto&& [i, filter] : enumerate(constitution_view))
    {
        DimUID uid_dim;
        uid_dim.dim = filter->dimension();
        uid_dim.uid = filter->constitution_uid();

        constitution_uid_to_index[uid_dim] = i;
    }

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
                if(sc->dim() == 3)
                    info.tet_count = sc->tetrahedra().size();
                else if(sc->dim() == 2)
                    info.codim_2d_count = sc->triangles().size();
                else if(sc->dim() == 1)
                    info.codim_1d_count = sc->edges().size();
                else if(sc->dim() == 0)
                    info.codim_0d_count = sc->vertices().size();

                info.index = constitution_uid_to_index[info.dim_uid];

                fem_geo_infos.push_back(info);
            }
        }
    }

    // 2) sort geometry by (dim, uid)
    std::stable_sort(fem_geo_infos.begin(),
                     fem_geo_infos.end(),
                     [](const GeoInfo& a, const GeoInfo& b)
                     { return a.dim_uid < b.dim_uid; });
}

void FiniteElementMethod::Impl::write_scene(WorldVisitor& world) {}
}  // namespace uipc::backend::cuda