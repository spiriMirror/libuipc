#pragma once
#include <finite_element/finite_element_constitution.h>

namespace uipc::backend::cuda
{
class Codim0DConstitution : public FiniteElementConstitution
{
  public:
    using FiniteElementConstitution::FiniteElementConstitution;

    class BuildInfo
    {
      public:
    };

  protected:
    virtual void do_build(BuildInfo& info)                               = 0;
    virtual void do_init(FiniteElementMethod::Codim0DFilteredInfo& info) = 0;

    const FiniteElementMethod::ConstitutionInfo& constitution_info() const noexcept;

  private:
    friend class FiniteElementMethod;
    void         init(FiniteElementMethod::Codim0DFilteredInfo& info);
    virtual void do_report_extent(ReportExtentInfo& info) override final;
    virtual void do_build(FiniteElementConstitution::BuildInfo& info) override final;
    virtual IndexT get_dimension() const noexcept override final;
    virtual void do_compute_energy(FiniteElementConstitution::ComputeEnergyInfo& info) override final;
    virtual void do_compute_gradient_hessian(
        FiniteElementConstitution::ComputeGradientHessianInfo& info) override final;
    virtual Vector2i get_vertex_offset_count() const noexcept override final;
};
}  // namespace uipc::backend::cuda
