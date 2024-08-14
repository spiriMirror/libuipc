#include <finite_element/fem_3d_constitution.h>
#include <finite_element/constitutions/arap_function.h>
#include <finite_element/fem_utils.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>
#include <Eigen/Dense>
#include <muda/ext/eigen/evd.h>

namespace uipc::backend::cuda
{
class ARAP3D final : public FEM3DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64 ConstitutionUID = 9;

    using FEM3DConstitution::FEM3DConstitution;

    vector<Float> h_kappas;
    vector<Float> h_lambdas;

    muda::DeviceBuffer<Float> kappas;

    virtual U64 get_constitution_uid() const override
    {
        return ConstitutionUID;
    }

    virtual void do_build(BuildInfo& info) override {}

    virtual void do_retrieve(FiniteElementMethod::FEM3DFilteredInfo& info) override
    {

        auto geo_slots = world().scene().geometries();

        auto N = info.primitive_count();

        h_kappas.resize(N);

        info.for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc) -> auto
            {
                auto kappa = sc.tetrahedra().find<Float>("kappa");

                return kappa->view();
            },
            [&](SizeT I, Float kappa) { h_kappas[I] = kappa; });

        kappas.resize(N);
        kappas.view().copy_from(h_kappas.data());
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace muda;
        namespace ARAP = sym::arap_3d;

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(info.indices().size(),
                   [kappas = kappas.cviewer().name("mus"),
                    element_energies = info.element_energies().viewer().name("energies"),
                    indices = info.indices().viewer().name("indices"),
                    xs      = info.xs().viewer().name("xs"),
                    Dm_invs = info.Dm_invs().viewer().name("Dm_invs"),
                    volumes = info.rest_volumes().viewer().name("volumes"),
                    dt      = info.dt()] __device__(int I)
                   {
                       const Vector4i&  tet    = indices(I);
                       const Matrix3x3& Dm_inv = Dm_invs(I);
                       Float            mu     = kappas(I);

                       const Vector3& x0 = xs(tet(0));
                       const Vector3& x1 = xs(tet(1));
                       const Vector3& x2 = xs(tet(2));
                       const Vector3& x3 = xs(tet(3));

                       auto F = fem::F(x0, x1, x2, x3, Dm_inv);

                       Float E;

                       ARAP::E(E, kappas(I) * dt * dt, volumes(I), F);
                       E *= dt * dt;
                       element_energies(I) = E;
                   });
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        using namespace muda;
        namespace ARAP = sym::arap_3d;

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(info.indices().size(),
                   [kappas  = kappas.cviewer().name("mus"),
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
                       Float            mu     = kappas(I);

                       const Vector3& x0 = xs(tet(0));
                       const Vector3& x1 = xs(tet(1));
                       const Vector3& x2 = xs(tet(2));
                       const Vector3& x3 = xs(tet(3));

                       auto F = fem::F(x0, x1, x2, x3, Dm_inv);

                       Vector9   dEdF;
                       Matrix9x9 ddEddF;

                       auto kt2 = kappas(I) * dt * dt;
                       auto v   = volumes(I);

                       ARAP::dEdF(dEdF, kt2, v, F);
                       ARAP::ddEddF(ddEddF, kt2, v, F);

                       Matrix9x12 dFdx = fem::dFdx(Dm_inv);

                       G12s(I)    = dFdx.transpose() * dEdF;
                       H12x12s(I) = dFdx.transpose() * ddEddF * dFdx;
                   });
    }
};

REGISTER_SIM_SYSTEM(ARAP3D);
}  // namespace uipc::backend::cuda
