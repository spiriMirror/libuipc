#pragma once
#include <sim_system.h>
#include <affine_body/affine_body_constitution.h>
#include <affine_body/affine_body_dynamics.h>

namespace uipc::backend::cuda
{
class ABDARAPPotential : public AffineBodyConstitution
{
  public:
    static constexpr U64 ConstitutionUID = 2ull;

    using AffineBodyConstitution::AffineBodyConstitution;

    virtual void do_build(AffineBodyConstitution::BuildInfo& info) override;

  public:
    class Impl
    {
      public:
        void filter(const AffineBodyDynamics::FilteredInfo& info, WorldVisitor& world);
        void _build_on_device();

        void compute_energy(const AffineBodyDynamics::ComputeEnergyInfo& info);
        void compute_gradient_hessian(const AffineBodyDynamics::ComputeGradientHessianInfo& info);

        vector<AffineBodyDynamics::BodyInfo> h_body_infos;
        vector<Float>                        h_kappas;

        muda::DeviceBuffer<Float> kappas;
        muda::DeviceBuffer<Float> body_energies;
    };

  protected:
  private:
    Impl m_impl;

    // Inherited via AffineBodyConstitution
    U64  get_constitution_uid() const override;
    void do_filter(AffineBodyDynamics::FilteredInfo& info) override;
    void do_compute_energy(AffineBodyDynamics::ComputeEnergyInfo& info) override;
    void do_compute_gradient_hessian(AffineBodyDynamics::ComputeGradientHessianInfo& info) override;
};
}  // namespace uipc::backend::cuda
