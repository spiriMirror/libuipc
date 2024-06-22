#include <affine_body/constitutions/abd_ortho_potential.h>
#include <uipc/common/enumerate.h>
namespace uipc::backend::cuda
{
template <>
class SimSystemCreator<ABDOrthoPotential>
{
  public:
    static U<ISimSystem> create(SimEngine& engine)
    {
        auto scene = engine.world().scene();
        // Check if we have the AffineBodyDynamics Type
        auto& types = scene.constitution_tabular().types();
        if(types.find(world::ConstitutionTypes::AffineBody) == types.end())
            return nullptr;

        // Check if we have the ABDOrthoPotential constitution
        auto uids = scene.constitution_tabular().uids();
        if(!std::binary_search(uids.begin(), uids.end(), ABDOrthoPotential::ConstitutionUID))
            return nullptr;

        return std::make_unique<AffineBodyDynamics>(engine);
    }
};

REGISTER_SIM_SYSTEM(ABDOrthoPotential);

void ABDOrthoPotential::build()
{
    m_impl.affine_body_geometry = find<AffineBodyDynamics>();

    // ConstitutionRegister the action to filter the affine body geometry
    m_impl.affine_body_geometry->on_update(
        ConstitutionUID,  // By libuipc specification
        [this](const AffineBodyDynamics::FilteredInfo& info)
        { m_impl.on_filter(info, world()); },
        [this](const AffineBodyDynamics::ComputeEnergyInfo& info) {

        },
        [this](const AffineBodyDynamics::ComputeGradientHessianInfo& info) {

        });
}

void ABDOrthoPotential::Impl::on_filter(const AffineBodyDynamics::FilteredInfo& info,
                                        WorldVisitor& world)
{
    auto src = info.body_infos();
    h_body_infos.resize(src.size());
    std::ranges::copy(src, h_body_infos.begin());

    // find out constitution coefficients
    h_kappas.resize(src.size());
    auto geo_slots = world.scene().geometries();

    info.for_each_body(
        geo_slots,
        [](geometry::SimplicialComplex& sc)
        { return sc.instances().find<Float>("kappa")->view(); },
        [&](SizeT I, Float kappa) { h_kappas[I] = kappa; });

    _build_on_device();
}

void ABDOrthoPotential::Impl::on_compute_energy(const AffineBodyDynamics::ComputeEnergyInfo& info)
{
    info.q();
    info.shape_energy();

    using namespace muda;

}

void ABDOrthoPotential::Impl::_build_on_device()
{
    auto async_copy = []<typename T>(span<T> src, muda::DeviceBuffer<T>& dst)
    {
        muda::BufferLaunch().resize<T>(dst, src.size());
        muda::BufferLaunch().copy<T>(dst.view(), src.data());
    };

    async_copy(span{h_kappas}, kappas);
}
}  // namespace uipc::backend::cuda
