#pragma once
#include <sim_system.h>
#include <affine_body/affine_body_dynamics.h>

namespace uipc::backend::cuda
{
class ABDOrthoPotential : public SimSystem
{
  public:
    static constexpr U64 ConstitutionUID = 1ull;

    using SimSystem::SimSystem;

    virtual void build() override;

  private:
    class Impl
    {
      public:
        AffineBodyDynamics* affine_body_geometry;

        void on_filter(const AffineBodyDynamics::FilteredInfo& info);
    } m_impl;
};
}  // namespace uipc::backend::cuda
