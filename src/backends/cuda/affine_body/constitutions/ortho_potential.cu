#include <affine_body/affine_body_constitution.h>
#include <affine_body/constitutions/ortho_potential_function.h>
#include <utils/make_spd.h>


namespace uipc::backend::cuda
{
class OrthoPotential final : public AffineBodyConstitution
{
  public:
    static constexpr U64 ConstitutionUID = 1ull;

    using AffineBodyConstitution::AffineBodyConstitution;

    vector<Float> h_kappas;

    cuda_tool::DeviceBuffer<Float> kappas;

    virtual void do_build(AffineBodyConstitution::BuildInfo& info) override {}

    U64 get_uid() const override { return ConstitutionUID; }

    void do_init(AffineBodyDynamics::FilteredInfo& info) override
    {
        using ForEachInfo = AffineBodyDynamics::ForEachInfo;

        // find out constitution coefficients
        h_kappas.resize(info.body_count());
        auto geo_slots = world().scene().geometries();

        info.for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc)
            { return sc.instances().find<Float>("kappa")->view(); },
            [&](const ForEachInfo& I, Float kappa)
            {
                auto bodyI      = I.global_index();
                h_kappas[bodyI] = kappa;
            });

        auto async_copy = []<typename T>(span<T> src, cuda_tool::DeviceBuffer<T>& dst)
        {
            cuda_tool::BufferLaunch().resize<T>(dst, src.size());
            cuda_tool::BufferLaunch().copy<T>(dst.view(), src.data());
        };

        async_copy(span{h_kappas}, kappas);
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace cuda_tool;

        auto body_count = info.qs().size();

        namespace AOP = sym::abd_ortho_potential;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(body_count,
                   [shape_energies = info.energies().viewer(),
                    qs             = info.qs().cviewer(),
                    kappas         = kappas.cviewer(),
                    volumes        = info.volumes().cviewer(),
                    dt             = info.dt()] __device__(int i) mutable
                   {
                       auto& q      = qs(i);
                       auto& volume = volumes(i);
                       auto  kappa  = kappas(i);
                       Float Vdt2   = volume * dt * dt;

                       Float E;
                       AOP::E(E, kappa, q);

                       shape_energies(i) = E * Vdt2;
                   });
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace cuda_tool;
        auto N             = info.qs().size();
        auto gradient_only = info.gradient_only();

        namespace AOP = sym::abd_ortho_potential;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(N,
                   [qs      = info.qs().cviewer(),
                    volumes = info.volumes().cviewer(),
                    gradients = info.gradients().viewer(),
                    body_hessian = info.hessians().viewer(),
                    kappas = kappas.cviewer(),
                    dt     = info.dt(),
                    gradient_only] __device__(int i) mutable
                   {
                       Matrix12x12 H = Matrix12x12::Zero();
                       Vector12    G = Vector12::Zero();

                       const auto& q      = qs(i);
                       Float       kappa  = kappas(i);
                       const auto& volume = volumes(i);

                       Float Vdt2 = volume * dt * dt;

                       Vector9 G9;
                       AOP::dEdq(G9, kappa, q);
                       G.segment<9>(3) = G9 * Vdt2;
                       gradients(i)    = G;

                       if(gradient_only)
                           return;

                       Matrix9x9 H9x9;
                       AOP::ddEddq(H9x9, kappa, q);
                       make_spd(H9x9);

                       H.block<9, 9>(3, 3) = H9x9 * Vdt2;
                       body_hessian(i)     = H;
                   });
    }
};

REGISTER_SIM_SYSTEM(OrthoPotential);
}  // namespace uipc::backend::cuda
