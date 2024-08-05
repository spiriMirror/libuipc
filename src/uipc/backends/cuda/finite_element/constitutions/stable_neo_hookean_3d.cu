#include <finite_element/constitutions/stable_neo_hookean_3d.h>
#include <uipc/common/zip.h>
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(StableNeoHookean3D);

U64 StableNeoHookean3D::get_constitution_uid() const
{
    return ConstitutionUID;
}

void StableNeoHookean3D::do_build(BuildInfo& info) {}
void StableNeoHookean3D::do_retrieve(FiniteElementMethod::FEM3DFilteredInfo& info)
{
    m_impl.retrieve(world(), info);
}
void StableNeoHookean3D::Impl::retrieve(WorldVisitor& world,
                                        FiniteElementMethod::FEM3DFilteredInfo& info)
{
    auto geo_slots = world.scene().geometries();

    h_mu.resize(info.vertex_count());
    h_lambda.resize(info.vertex_count());

    info.for_each(
        geo_slots,
        [](geometry::SimplicialComplex& sc)
        {
            auto mu     = sc.vertices().find<Float>("mu")->view();
            auto lambda = sc.vertices().find<Float>("lambda")->view();
            return zip(mu, lambda);
        },
        [&](SizeT I, auto mu_and_lambda)
        {
            auto&& [mu, lambda] = mu_and_lambda;
            h_mu[I]             = mu;
            h_lambda[I]         = lambda;
        });
}
}  // namespace uipc::backend::cuda
