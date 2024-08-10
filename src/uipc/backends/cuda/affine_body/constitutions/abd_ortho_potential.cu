#include <affine_body/constitutions/abd_ortho_potential.h>
#include <uipc/common/enumerate.h>
#include <affine_body/abd_energy.h>
#include <muda/cub/device/device_reduce.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(ABDOrthoPotential);

void ABDOrthoPotential::do_build(AffineBodyConstitution::BuildInfo& info)
{
    auto scene = world().scene();
    // Check if we have the ABDOrthoPotential constitution
    auto uids = scene.constitution_tabular().uids();
    if(!std::binary_search(uids.begin(), uids.end(), ABDOrthoPotential::ConstitutionUID))
    {
        throw SimSystemException(fmt::format("ABDOrthoPotential requires ABDOrthoPotential Constitution (UID={})",
                                             ABDOrthoPotential::ConstitutionUID));
    }
}

U64 ABDOrthoPotential::get_constitution_uid() const
{
    return ConstitutionUID;
}

void ABDOrthoPotential::Impl::retrieve(const AffineBodyDynamics::FilteredInfo& info,
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

namespace sym::abd_ortho_potential
{
#include "sym/ortho_potential.inl"
}


void ABDOrthoPotential::Impl::compute_energy(const AffineBodyDynamics::ComputeEnergyInfo& info)
{
    using namespace muda;

    auto body_count = info.qs().size();

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(body_count,
               [shape_energies = info.body_shape_energies().viewer().name("body_energies"),
                qs      = info.qs().cviewer().name("qs"),
                kappas  = kappas.cviewer().name("kappas"),
                volumes = info.volumes().cviewer().name("volumes"),
                dt      = info.dt()] __device__(int i) mutable
               {
                   auto& V      = shape_energies(i);
                   auto& q      = qs(i);
                   auto& volume = volumes(i);
                   auto  kappa  = kappas(i);

                   sym::abd_ortho_potential::E(V, kappa * dt * dt, volume, q);

                   // V = kappa * volume * dt * dt * shape_energy(q);
               });
}

void ABDOrthoPotential::Impl::compute_gradient_hessian(const AffineBodyDynamics::ComputeGradientHessianInfo& info)
{
    using namespace muda;
    auto N = info.qs().size();

    ParallelFor(256)
        .kernel_name(__FUNCTION__)
        .apply(N,
               [qs      = info.qs().cviewer().name("qs"),
                volumes = info.volumes().cviewer().name("volumes"),
                gradients = info.shape_gradient().viewer().name("shape_gradients"),
                body_hessian = info.shape_hessian().viewer().name("shape_hessian"),
                kappas = kappas.cviewer().name("kappas"),
                dt     = info.dt()] __device__(int i) mutable
               {
                   Matrix12x12 H = Matrix12x12::Zero();
                   Vector12    G = Vector12::Zero();

                   const auto& q      = qs(i);
                   Float       kappa  = kappas(i);
                   const auto& volume = volumes(i);

                   //auto      kvt2           = kappa * volume * dt * dt;
                   //Vector9   shape_gradient = kvt2 * shape_energy_gradient(q);
                   //Matrix9x9 shape_H        = kvt2 * shape_energy_hessian(q);

                   Float kt2 = kappa * dt * dt;

                   Vector9 shape_gradient = Vector9::Zero();
                   sym::abd_ortho_potential::dEdq(shape_gradient, kt2, volume, q);

                   Matrix9x9 shape_H = Matrix9x9::Zero();
                   sym::abd_ortho_potential::ddEddq(shape_H, kt2, volume, q);

                   H.block<9, 9>(3, 3) += shape_H;
                   G.segment<9>(3) += shape_gradient;

                   gradients(i) += G;
                   body_hessian(i) += H;
               });
}

void ABDOrthoPotential::Impl::_build_on_device()
{
    auto async_copy = []<typename T>(span<T> src, muda::DeviceBuffer<T>& dst)
    {
        muda::BufferLaunch().resize<T>(dst, src.size());
        muda::BufferLaunch().copy<T>(dst.view(), src.data());
    };

    auto async_resize = []<typename T>(muda::DeviceBuffer<T>& dst, SizeT size)
    { muda::BufferLaunch().resize<T>(dst, size); };

    async_copy(span{h_kappas}, kappas);
}

void ABDOrthoPotential::do_retrieve(AffineBodyDynamics::FilteredInfo& info)
{
    m_impl.retrieve(info, world());
}

void ABDOrthoPotential::do_compute_energy(AffineBodyDynamics::ComputeEnergyInfo& info)
{
    m_impl.compute_energy(info);
}

void ABDOrthoPotential::do_compute_gradient_hessian(AffineBodyDynamics::ComputeGradientHessianInfo& info)
{
    m_impl.compute_gradient_hessian(info);
}
}  // namespace uipc::backend::cuda
