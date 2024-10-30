#pragma once
#include <sim_system.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/finite_element_energy_producer.h>

namespace uipc::backend::cuda
{
class FiniteElementConstitution : public FiniteElementEnergyProducer
{
  public:
    using FiniteElementEnergyProducer::FiniteElementEnergyProducer;

    class BuildInfo
    {
      public:
    };

    U64    uid() const noexcept;
    IndexT dim() const noexcept;

  protected:
    virtual U64    get_uid() const noexcept = 0;
    virtual IndexT get_dim() const noexcept = 0;

    virtual void do_build(BuildInfo& info) = 0;
    virtual void do_report_extent(ReportExtentInfo& info);
    virtual void do_init(FiniteElementMethod::FilteredInfo& info) = 0;
    const FiniteElementMethod::DimInfo& dim_info() const noexcept;
    const FiniteElementMethod::ConstitutionInfo& constitution_info() const noexcept;

  private:
    friend class Codim0DConstitution;
    friend class Codim1DConstitution;
    friend class Codim2DConstitution;
    friend class FEM3DConstitution;
    friend class FiniteElementMethod;
    friend class FiniteElementConstitutionDiffParmReporter;

    void         init();  // only be called by FiniteElementMethod
    virtual void do_build(FiniteElementEnergyProducer::BuildInfo& info) final;

    SizeT                      m_index                 = 0;
    SizeT                      m_index_in_dim          = 0;
    FiniteElementMethod*       m_finite_element_method = nullptr;
    FiniteElementMethod::Impl& fem() const noexcept;
};
}  // namespace uipc::backend::cuda
