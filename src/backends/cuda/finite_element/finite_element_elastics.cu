#include <finite_element/finite_element_elastics.h>
#include <finite_element/finite_element_constitution.h>
#include <finite_element/finite_element_extra_constitution.h>
#include <finite_element/fem_linear_subsystem_reporter.h>
#include <finite_element/fem_line_search_subreporter.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(FiniteElementElastics);

void FiniteElementElastics::do_build()
{
    m_impl.finite_element_method = require<FiniteElementMethod>();
    m_impl.finite_element_method->add_elastics(this);
}

void FiniteElementElastics::init()
{
    auto  constitutions = m_impl.fem().constitutions.view();
    auto  extras        = m_impl.fem().extra_constitutions.view();
    SizeT N             = constitutions.size() + extras.size();

    m_impl.constitution_energy_offsets_counts.resize(N);
    m_impl.constitution_gradient_offsets_counts.resize(N);
    m_impl.constitution_hessian_offsets_counts.resize(N);

    auto energy_span = m_impl.constitution_energy_offsets_counts.counts();
    auto grad_span   = m_impl.constitution_gradient_offsets_counts.counts();
    auto hess_span   = m_impl.constitution_hessian_offsets_counts.counts();

    auto offset = 0;
    for(auto&& [I, c] : enumerate(constitutions))
    {
        ReportExtentInfo this_info;
        c->report_extent(this_info);
        energy_span[I] = this_info.m_energy_count;
        grad_span[I]   = this_info.m_gradient_count;
        hess_span[I]   = this_info.m_hessian_count;
    }

    offset += constitutions.size();
    for(auto&& [I, c] : enumerate(extras))
    {
        ReportExtentInfo this_info;
        c->report_extent(this_info);
        energy_span[offset + I] = this_info.m_energy_count;
        grad_span[offset + I]   = this_info.m_gradient_count;
        hess_span[offset + I]   = this_info.m_hessian_count;
    }
    offset += extras.size();

    UIPC_ASSERT(offset == N, "offset mismatching, offset = {}, expected = {}", offset, N);
}

void FiniteElementElastics::Impl::assemble(FEMLinearSubsystem::AssembleInfo& info)
{
    auto constitutions = fem().constitutions.view();
    auto extras        = fem().extra_constitutions.view();

    auto offset = 0;

    for(auto&& [I, c] : enumerate(constitutions))
    {
        FiniteElementConstitution::ComputeGradientHessianInfo this_info{
            this, I, info.dt(), info.gradients(), info.hessians()};

        c->compute_gradient_hessian(this_info);
    }
    offset += constitutions.size();

    for(auto&& [I, c] : enumerate(extras))
    {
        FiniteElementExtraConstitution::ComputeGradientHessianInfo this_info{
            this, offset + I, info.dt(), info.gradients(), info.hessians()};
        c->compute_gradient_hessian(this_info);
    }
    offset += extras.size();

    UIPC_ASSERT(offset == constitutions.size() + extras.size(),
                "offset mismatching, offset = {}, expected = {}",
                offset,
                constitutions.size() + extras.size());
}

void FiniteElementElastics::Impl::compute_energy(FEMLineSearchReporter::EnergyInfo& info)
{
    auto constitutions = fem().constitutions.view();
    auto extras        = fem().extra_constitutions.view();
    auto offset        = 0;
    for(auto&& [I, c] : enumerate(constitutions))
    {
        FiniteElementConstitution::ComputeEnergyInfo this_info{
            this, I, info.dt(), info.energies()};
        c->compute_energy(this_info);
    }
    offset += constitutions.size();
    for(auto&& [I, c] : enumerate(extras))
    {
        FiniteElementExtraConstitution::ComputeEnergyInfo this_info{
            this, offset + I, info.dt(), info.energies()};
        c->compute_energy(this_info);
    }
    offset += extras.size();

    UIPC_ASSERT(offset == constitutions.size() + extras.size(),
                "offset mismatching, offset = {}, expected = {}",
                offset,
                constitutions.size() + extras.size());
}

class FiniteElementElasticsLinearSubsystemReporter final : public FEMLinearSubsystemReporter
{
  public:
    using FEMLinearSubsystemReporter::FEMLinearSubsystemReporter;

    SimSystemSlot<FiniteElementElastics> elastics;

    FiniteElementElastics::Impl& el() { return elastics->m_impl; }

    virtual void do_build(BuildInfo& info) override
    {
        elastics = require<FiniteElementElastics>();
    }

    virtual void do_init(InitInfo& info) override {}

    virtual void do_report_extent(ReportExtentInfo& info) override
    {
        auto grad_count = el().constitution_gradient_offsets_counts.total_count();
        info.gradient_count(grad_count);
        auto hess_count = el().constitution_hessian_offsets_counts.total_count();
        info.hessian_count(hess_count);
    }

    virtual void do_assemble(AssembleInfo& info) override
    {
        el().assemble(info);
    }
};

REGISTER_SIM_SYSTEM(FiniteElementElasticsLinearSubsystemReporter);

class FiniteElementElasticsLineSearchSubreporter final : public FEMLineSearchSubreporter
{
  public:
    using FEMLineSearchSubreporter::FEMLineSearchSubreporter;

    SimSystemSlot<FiniteElementElastics> elastics;

    FiniteElementElastics::Impl& el() { return elastics->m_impl; }

    virtual void do_build(BuildInfo& info) override
    {
        elastics = require<FiniteElementElastics>();
    }

    virtual void do_init(InitInfo& info) override {}

    virtual void do_report_extent(ExtentInfo& info) override
    {
        auto energy_count = el().constitution_energy_offsets_counts.total_count();
        info.energy_count(energy_count);
    }

    virtual void do_report_energy(EnergyInfo& info) override {}
};

REGISTER_SIM_SYSTEM(FiniteElementElasticsLineSearchSubreporter);

}  // namespace uipc::backend::cuda
