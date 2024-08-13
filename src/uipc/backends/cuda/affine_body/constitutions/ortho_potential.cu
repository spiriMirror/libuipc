#include <sim_system.h>
#include <affine_body/affine_body_constitution.h>
#include <affine_body/affine_body_dynamics.h>
#include <uipc/common/enumerate.h>
#include <affine_body/abd_energy.h>
#include <muda/cub/device/device_reduce.h>


namespace uipc::backend::cuda
{
namespace sym::abd_ortho_potential
{
#include "sym/ortho_potential.inl"
}
class OrthoPotential final : public AffineBodyConstitution
{
  public:
    static constexpr U64 ConstitutionUID = 1ull;

    using AffineBodyConstitution::AffineBodyConstitution;


    vector<AffineBodyDynamics::BodyInfo> h_body_infos;
    vector<Float>                        h_kappas;

    muda::DeviceBuffer<Float> kappas;

    virtual void do_build(AffineBodyConstitution::BuildInfo& info) override {}

    U64  get_constitution_uid() const override { return ConstitutionUID; }
    void do_retrieve(AffineBodyDynamics::FilteredInfo& info) override
    {
        auto src = info.body_infos();
        h_body_infos.resize(src.size());
        std::ranges::copy(src, h_body_infos.begin());

        // find out constitution coefficients
        h_kappas.resize(src.size());
        auto geo_slots = world().scene().geometries();

        info.for_each_body(
            geo_slots,
            [](geometry::SimplicialComplex& sc)
            { return sc.instances().find<Float>("kappa")->view(); },
            [&](SizeT I, Float kappa) { h_kappas[I] = kappa; });

        _build_on_device();
    }

    void _build_on_device()
    {
        auto async_copy = []<typename T>(span<T> src, muda::DeviceBuffer<T>& dst)
        {
            muda::BufferLaunch().resize<T>(dst, src.size());
            muda::BufferLaunch().copy<T>(dst.view(), src.data());
        };

        async_copy(span{h_kappas}, kappas);
    }

    virtual void do_compute_energy(AffineBodyDynamics::ComputeEnergyInfo& info) override
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
                   });
    }

    virtual void do_compute_gradient_hessian(AffineBodyDynamics::ComputeGradientHessianInfo& info) override
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
};

REGISTER_SIM_SYSTEM(OrthoPotential);
}  // namespace uipc::backend::cuda
