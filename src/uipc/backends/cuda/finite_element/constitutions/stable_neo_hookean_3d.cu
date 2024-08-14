#include <finite_element/fem_3d_constitution.h>
#include <finite_element/constitutions/stable_neo_hookean_3d_function.h>
#include <finite_element/fem_utils.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>
#include <Eigen/Dense>
#include <muda/ext/eigen/evd.h>

namespace uipc::backend::cuda
{
class StableNeoHookean3D final : public FEM3DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64 ConstitutionUID = 10;

    using FEM3DConstitution::FEM3DConstitution;

    vector<Float> h_mus;
    vector<Float> h_lambdas;

    muda::DeviceBuffer<Float> mus;
    muda::DeviceBuffer<Float> lambdas;

    virtual U64 get_constitution_uid() const override
    {
        return ConstitutionUID;
    }

    virtual void do_build(BuildInfo& info) override {}

    virtual void do_retrieve(FiniteElementMethod::FEM3DFilteredInfo& info) override
    {

        auto geo_slots = world().scene().geometries();

        auto N = info.primitive_count();

        h_mus.resize(N);
        h_lambdas.resize(N);

        info.for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc) -> auto
            {
                auto mu     = sc.tetrahedra().find<Float>("mu");
                auto lambda = sc.tetrahedra().find<Float>("lambda");

                return zip(mu->view(), lambda->view());
            },
            [&](SizeT I, auto mu_and_lambda)
            {
                auto&& [mu, lambda] = mu_and_lambda;
                h_mus[I]            = mu;
                h_lambdas[I]        = lambda;
            });

        mus.resize(N);
        mus.view().copy_from(h_mus.data());

        lambdas.resize(N);
        lambdas.view().copy_from(h_lambdas.data());
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace muda;
        namespace SNK = sym::stable_neo_hookean_3d;

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(info.indices().size(),
                   [mus     = mus.cviewer().name("mus"),
                    lambdas = lambdas.cviewer().name("lambdas"),
                    element_energies = info.element_energies().viewer().name("energies"),
                    indices = info.indices().viewer().name("indices"),
                    xs      = info.xs().viewer().name("xs"),
                    Dm_invs = info.Dm_invs().viewer().name("Dm_invs"),
                    volumes = info.rest_volumes().viewer().name("volumes"),
                    dt      = info.dt()] __device__(int I)
                   {
                       const Vector4i&  tet    = indices(I);
                       const Matrix3x3& Dm_inv = Dm_invs(I);
                       Float            mu     = mus(I);
                       Float            lambda = lambdas(I);

                       const Vector3& x0 = xs(tet(0));
                       const Vector3& x1 = xs(tet(1));
                       const Vector3& x2 = xs(tet(2));
                       const Vector3& x3 = xs(tet(3));

                       auto F = fem::F(x0, x1, x2, x3, Dm_inv);

                       auto J = F.determinant();
                       MUDA_ASSERT(J > 0, "detF = %f, StableNeoHookean3D doesn't allow inversion.", J);

                       auto VecF = flatten(F);

                       Float E;

                       //ARAP::E(E, lambda * dt * dt, volumes(I), F);

                       SNK::E(E, mu, lambda, VecF);
                       E *= dt * dt * volumes(I);
                       element_energies(I) = E;
                   });
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace muda;
        namespace SNK = sym::stable_neo_hookean_3d;

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(info.indices().size(),
                   [mus     = mus.cviewer().name("mus"),
                    lambdas = lambdas.cviewer().name("lambdas"),
                    indices = info.indices().viewer().name("indices"),
                    xs      = info.xs().viewer().name("xs"),
                    Dm_invs = info.Dm_invs().viewer().name("Dm_invs"),
                    G12s    = info.gradient().viewer().name("gradient"),
                    H12x12s = info.hessian().viewer().name("hessian"),
                    volumes = info.rest_volumes().viewer().name("volumes"),
                    dt      = info.dt()] __device__(int I)
                   {
                       const Vector4i&  tet    = indices(I);
                       const Matrix3x3& Dm_inv = Dm_invs(I);
                       Float            mu     = mus(I);
                       Float            lambda = lambdas(I);

                       const Vector3& x0 = xs(tet(0));
                       const Vector3& x1 = xs(tet(1));
                       const Vector3& x2 = xs(tet(2));
                       const Vector3& x3 = xs(tet(3));

                       auto F = fem::F(x0, x1, x2, x3, Dm_inv);

                       auto J = F.determinant();
                       MUDA_ASSERT(J > 0, "detF = %f, StableNeoHookean3D doesn't allow inversion.", J);

                       auto VecF = flatten(F);

                       auto vdt2 = volumes(I) * dt * dt;

                       Vector9   dEdF;
                       Matrix9x9 ddEddF;
                       SNK::dEdVecF(dEdF, mu, lambda, VecF);
                       SNK::ddEddVecF(ddEddF, mu, lambda, VecF);
                       dEdF *= vdt2;
                       ddEddF *= vdt2;

                       Matrix9x12 dFdx = fem::dFdx(Dm_inv);

                       G12s(I)    = dFdx.transpose() * dEdF;
                       H12x12s(I) = dFdx.transpose() * ddEddF * dFdx;
                   });
    }
};

REGISTER_SIM_SYSTEM(StableNeoHookean3D);
}  // namespace uipc::backend::cuda
